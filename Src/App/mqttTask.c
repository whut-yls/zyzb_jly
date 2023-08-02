 #include "string.h"
#include "lwip/api.h"
//private
#include "mqttTask.h"
#include "parameter.h"
#include "user_tcp_client.h"
#include "lcdTask.h"


static uint8_t gMqttRecvBuf[MQTTMESSAGE_MAXLENGTH]={0x00}; 
MQTTPacket_connectData MQTTPacket_ConnectData_Initializer;
char mq_ClientID[10];
void MQTT_ConnectData_Init(MQTTPacket_connectData* data)
{
	MQTTPacket_ConnectData_Initializer.struct_id[0]='M';
	MQTTPacket_ConnectData_Initializer.struct_id[1]='Q';
	MQTTPacket_ConnectData_Initializer.struct_id[2]='T';
	MQTTPacket_ConnectData_Initializer.struct_id[3]='T';
	MQTTPacket_ConnectData_Initializer.struct_version=0;
	MQTTPacket_ConnectData_Initializer.MQTTVersion=4;
	MQTTPacket_ConnectData_Initializer.username.cstring = gDeviceParam.mqArg.MQ_Username;
	MQTTPacket_ConnectData_Initializer.password.cstring = gDeviceParam.mqArg.MQ_Password;
	sprintf(mq_ClientID,"%d",gDeviceParam.devId);
	MQTTPacket_ConnectData_Initializer.clientID.cstring = mq_ClientID;
	MQTTPacket_ConnectData_Initializer.keepAliveInterval = 100;
	MQTTPacket_ConnectData_Initializer.cleansession = 1;	
}

int MQTTConnect(MQTTPacket_connectData* options)
{
	uint16_t l;
	int rc;
//	EventBits_t uxbits;
	unsigned char session,drc;
	unsigned char mq_Send[NET_MSS_LEN]={0x00};
//	MQTTString topic= {pParameter_NET.MQ_Top_Topic,{strlen(pParameter_NET.MQ_Top_Topic),pParameter_NET.MQ_Top_Topic}};
	l=MQTTSerialize_connect(mq_Send,sizeof(mq_Send),&MQTTPacket_ConnectData_Initializer);
	//printf("MQTT Connect\r\n");
	//打开一个tcp连接作为mqtt传输
	if(transport_open(MQ_NET_ID,gDeviceParam.ServerIpAddress,gDeviceParam.mqArg.mqttPort)<0)
	{
		return -1;
	}

	//传输序列化的mqtt包数据
	rc=transport_sendPacketBuffer(MQ_NET_ID,mq_Send,l);
	if(rc==l){
		printf("Success publish\n");
	}else{
		printf("Failed publish\n");
		return -1;
	}

	//等待应答数据
		l=1500;
	if(transport_recvPacketBuffer(MQ_NET_ID,mq_Send,&l,3000)>0)	//简化了MQTTPacket_read()流程的读操作
	{
		drc = 255;
		if(MQTTDeserialize_connack(&session,&drc,mq_Send,l)!=1||drc!=0)
		{
	//			printf("MQTT Connect fail,connack return code fault:%d\r\n",drc);
				return -1;
		}
	
		printf("MQTT Connect success\r\n");
	}else{
			return -1;
	}
	
	return 1;
}

int MQTTDisConnect(MQTTPacket_connectData* options)
{
	uint16_t l;
	int rc;
//	EventBits_t uxbits;
	unsigned char mq_Send[NET_MSS_LEN]={0x00};
	
	l=MQTTSerialize_disconnect(mq_Send,sizeof(mq_Send));
	//printf("MQTT Disconnect\r\n");
	//通知mqtt服务器端 客户端断开
	//传输序列化的mqtt包数据
	rc=transport_sendPacketBuffer(MQ_NET_ID,mq_Send,l);
	if(rc==l){
		printf("Success publish\n");

	}else{
//		printf("Failed publish\n");

		return -1;
	}
	
//	transport_close(MQ_NET_ID);
	transport_release(MQ_NET_ID);

	return 1;
}

/**
功能：mqtt publish qos0/qos1/qos2
返回：MQ_FAILURE  失败，调用函数需要根据qos级别判断是否需要重传 成功：SUCCESS
**/
int MQTTPublish(const char* topicName, MQTTMessage* message)
{
    int rc = MQ_FAILURE;

    static uint16_t PacketId = 0;
    MQTTString topic = MQTTString_initializer;
    topic.cstring = (char *)topicName;
    uint16_t len ,l,t,i= 0;
		uint8_t buf[MQTTMESSAGE_MAXLENGTH]={0x00}; 
		unsigned char dup=0, type=0;
		unsigned short mypacketid;
		if(PacketId == 0xFFFF)
		{
			PacketId =1;
		}
		else
		{
			//判断message的dup 判断是否是一个重传包，来决定id是否累加,qos0时的累加只做内部计数判断
			if(message->dup!=1)
			PacketId++;
		}
    if (message->qos == QOS1 || message->qos == QOS2)
        message->id = PacketId;
    else
    		message->id = 0;
		
    len = MQTTSerialize_publish(buf, sizeof(buf), 0, message->qos, message->retained, message->id,
              topic, (unsigned char*)message->payload, message->payloadlen);
		//printf("MQTT Publish\r\n");
    if (len <= 0)
    {
			printf("serialize lenght fault:%d\r\n",len);
			goto exit;
		}
    
		//分包发送
		t= len /NET_MSS_LEN;
 		for(i = 0; i< t;i++)
		{
				if(transport_sendPacketBuffer(MQ_NET_ID,buf+NET_MSS_LEN*i,NET_MSS_LEN)<=0)
				{
					goto exit;
				}
	  }
		l=(len %NET_MSS_LEN);
	  if(l) 
	  {
			if(transport_sendPacketBuffer(MQ_NET_ID,buf+NET_MSS_LEN*i,l)<=0)
			{
				goto exit;
			}
	  }
		
		/**等待ack***/
	  if(message->qos == QOS0)	//不需要应答
	  {
			//printf("QOS0 Publish success\r\n");
	  	rc = SUCCESS;
	  	goto exit;           // qos0 return
	  }
 	 	else if (message->qos == QOS1)//wait PUBACK
    {
			//超时等待接收
			if(transport_recvPacketBuffer(MQ_NET_ID,buf,&l,3000)<=0)
			{
				printf("QOS1 puback respone timeout\r\n");
				/**qos1 可以考虑添加重传**/
				goto exit;
			}
			if (MQTTDeserialize_ack(&type, &dup, &mypacketid, buf, sizeof(buf)) != 1 || type != PUBACK)
      {
      	printf("no PUBACK,fault:%d\r\n",type);
      	goto exit; 
      }
      else
      {
				//printf("QOS1 Publish success\r\n");
     		rc = SUCCESS;
	  		goto exit;           // qos1 return
      }   
    }
    else if (message->qos == QOS2)         /*QOS2 publis*/
    {
			//超时等待接收 PUBREC
			if(transport_recvPacketBuffer(MQ_NET_ID,buf,&l,3000)<=0)
			{
				printf("QOS2 puback respone timeout\r\n");
				goto exit;
			}
			
			if (MQTTDeserialize_ack(&type, &dup, &mypacketid, buf, sizeof(buf)) != 1 || type != PUBREC)
			{
				printf("no PUBREC,fault:%d\r\n",type);// qos2 fault return
				goto exit; 
			}
			
			/* send PUBREL to MQ server*******/
			len = MQTTSerialize_ack(buf, sizeof(buf),PUBREL,message->dup,message->id);
			if (len <= 0 || len > NET_MSS_LEN)
			{
				printf("Serialize PUBREL exceed length:%d\r\n",len);
				goto exit;
			}
			
			if(transport_sendPacketBuffer(MQ_NET_ID,buf,len)<=0)
			{
				goto exit;
			}	
			
       /*"recv"  ,MQTT SEVER PUBCOMP*/
					//超时等待接收 PUBCOMP
			if(transport_recvPacketBuffer(MQ_NET_ID,buf,&l,3000)<=0)
			{
				printf("QOS2 PUBCOMP respone timeout\r\n");
				goto exit;
			}
			
			if (MQTTDeserialize_ack(&type, &dup, &mypacketid, buf, sizeof(buf)) != 1 || type != PUBCOMP)
			{
				printf("no PUBCOMP,fault:%d\r\n",type);
				goto exit; // qos2 fault response return
			}
			//printf("QOS2 success\r\n");
			rc = SUCCESS;
    }
		/* QOS >2 unkown*/
    else
    {
    	printf("QOS%d unkonw command\r\n",message->qos);
    	rc = MQ_FAILURE;
    }

exit:
    return rc;
}


/**
功能：mqtt消息订阅
返回：成功返回 SUCCESS(1) ,失败返回0 MQ_FAILURE
**/
int MQTTSubscribe(const char* topicName, MQTTMessage* message)
{
    int rc = MQ_FAILURE;

    static uint16_t PacketId = 0;
    MQTTString topic = MQTTString_initializer;
    topic.cstring = (char *)topicName;
    uint16_t len ,l,t,i= 0;
		int req_qos = 0;
		uint8_t buf[MQTTMESSAGE_MAXLENGTH+1]={0x00}; 
		unsigned char  type=0;
		
		unsigned short submsgid;
		int subcount;
		int granted_qos=0xff;
//		unsigned short mypacketid;
		if(PacketId == 0xFFFF)
		{
			PacketId =1;
		}
		else
		{
			PacketId++;
		}
    if (message->qos == QOS1 || message->qos == QOS2)
        message->id = PacketId;
    else
    		message->id = 1;	//订阅消息id 设置为0时，服务器无响应
		req_qos=message->qos;

		//len = MQTTSerialize_subscribe(buf, sizeof(buf), 0,message->id, 1, &topic,&req_qos);
		len = MQTTSerialize_subscribe(buf, sizeof(buf), 0,message->id, 1, &topic,&req_qos);	//id 不能为0	(buf,size(buf),dup,messageID,count,topic,req_qos)
    if (len <= 0)
    {
			printf("serialize lenght fault:%d\r\n",len);
			goto exit;
		}
    
		//分包发送
		t= len /NET_MSS_LEN;
 		for(i = 0; i< t;i++)
		{
				if(transport_sendPacketBuffer(MQ_NET_ID,buf+NET_MSS_LEN*i,NET_MSS_LEN)<=0)
				{
					goto exit;
				}
	  }
		l=(len %NET_MSS_LEN);
	  if(l) 
	  {
			if(transport_sendPacketBuffer(MQ_NET_ID,buf+NET_MSS_LEN*i,l)<=0)
			{
				goto exit;
			}
	  }
		

//	  if(message->qos == QOS0)
//	  {
			
		//超时等待接收
			memset(buf,0,sizeof(buf));
			if(transport_recvPacketBuffer(MQ_NET_ID,buf,&l,3000)<=0)
			{
				printf("QOS0 puback respone timeout\r\n");
				goto exit;
			}

			if (MQTTDeserialize_suback(&submsgid, 1 ,&subcount, &granted_qos,buf, sizeof(buf))!= 1 ||granted_qos!= message->qos)
      {
      	printf("no SUBBACK,fault:%d\r\n",type);
      	goto exit; 
      }
      else
      {
//				printf("QOS0 Publish success\r\n");
     		rc = SUCCESS;
	  		goto exit;           // qos1 return
      }  
//	  }
// 	 	else if (message->qos == QOS1)
//    {
//					//超时等待接收
//			if(transport_recvPacketBuffer(MQ_NET_ID,buf,&l,3000)<=0)
//			{
//				printf("QOS0 puback respone timeout\r\n");
//				goto exit;
//			}

//			if (MQTTDeserialize_suback(&submsgid, 1 ,&subcount, &granted_qos,buf, sizeof(buf))!= 1 ||granted_qos!=0)
//      {
//      	printf("no SUBBACK,fault:%d\r\n",type);
//      	goto exit; 
//      }
//				
//			/* send PUBACK to MQ server*******/
//			len = MQTTSerialize_ack(buf, sizeof(buf),PUBACK,message->dup,message->id);
//			if (len <= 0 || len > NET_MSS_LEN)
//			{
//				printf("Serialize PUBREL exceed length:%d\r\n",len);
//				goto exit;
//			}
//			
//			if(transport_sendPacketBuffer(MQ_NET_ID,buf,len)<=0)
//			{
//				goto exit;
//			}		
//      else
//      {
////				printf("QOS1 Publish success\r\n");
//     		rc = SUCCESS;
//	  		goto exit;           // qos1 return
//      }  
//    }
//    else if (message->qos == QOS2)         /*QOS2 publis*/
//    {
//				//超时等待接收
//			if(transport_recvPacketBuffer(MQ_NET_ID,buf,&l,3000)<=0)
//			{
//				printf("QOS0 puback respone timeout\r\n");
//				goto exit;
//			}

//			if (MQTTDeserialize_suback(&submsgid, 1 ,&subcount, &granted_qos,buf, sizeof(buf))!= 1 ||granted_qos!= message->qos)
//      {
//      	printf("no SUBBACK,fault:%d\r\n",type);
//      	goto exit; 
//      }
//      else
//      {
//				//printf("QOS1 Publish success\r\n");
//     		rc = SUCCESS;
//	  		goto exit;           // qos1 return
//      }  
//    }
//		/* QOS >2 unkown*/
//    else
//    {
//    	printf("QOS%d unkonw command\r\n",message->qos);
//    	rc = MQ_FAILURE;
//    }

exit:
    return rc;
}
/**
功能:mqtt订阅主题消息阻塞等待接收
**/
 int MQTTSubscribe_RecvMessage( MQTTMessage* message,MQTTString*messageTopic)
 {
		int rc=MQ_FAILURE;
		uint16_t len,l= 0;

//		uint8_t buf[MQTTMESSAGE_MAXLENGTH]={0x00};
		uint8_t*buf=gMqttRecvBuf;
		unsigned short mypacketid;
		unsigned char dup,type=0;
		int qos;
		unsigned char retained;
		unsigned short msgid;
		int payloadlen_in;
//		unsigned char* payload_in=buuffer;	 
	 
		unsigned char* payload_in=NULL;
		MQTTString* receivedTopic=messageTopic;
		
		memset(buf,0,sizeof(gMqttRecvBuf));
		rc=transport_recvPacketBuffer(MQ_NET_ID,buf,&l,0);	//阻塞等待

		//判断是ping包的回复则直接返回
		if (MQTTDeserialize_ack(&type, &dup, &mypacketid, buf, sizeof(buf)) != 1 || type == PINGRESP)
		//if(buf[0]==0xd0)	//PINGRESP (0xd0)
		{
			if(type==PINGRESP)
			{
				rc=MQ_PINGRESP;
			}
			goto exit;
		}	
		
		//常规包的处理
	  if(message->qos == QOS0)
	  {
			if(rc<=0)
			{
					printf("QOS0 puback respone timeout\r\n");
					goto exit;
			}		

			//正常包的回复
			if (MQTTDeserialize_publish(&dup, &qos, &retained, &msgid,receivedTopic,&payload_in,&payloadlen_in,buf, l) != 1)    //&payload_in  debug
      {
      	printf("no PUBACK,fault");
      	goto exit; 
      }
      else
      {
				//printf("QOS0 Recv Message success\r\n");
     		rc = MQ_SUCCESS;
	  		goto exit;           // qos1 return
      }   
	  }
 	 	else if (message->qos == QOS1)
    {
			//超时等待接收
			if(rc<=0)
			{
				printf("QOS1 puback respone timeout\r\n");
				goto exit;
			}

			if (MQTTDeserialize_publish(&dup, &qos, &retained, &msgid,receivedTopic,&payload_in,&payloadlen_in,buf, l) != 1)    //&payload_in  debug
      {
      	printf("QOS1 no PUBACK,fault\r\n");
      	goto exit; 
      }
//			MQTTDeserialize_publish(&dup, &qos, &retained, &msgid,&receivedTopic,&payload_in,&payloadlen_in,buf, l);
				//发送应答
			/* send PUBACK to MQ server*******/
//			len = MQTTSerialize_puback(buf, sizeof(buf),message->id);	//buf,sizeof(buf),puback,dup,id
			len = MQTTSerialize_puback(buf, sizeof(buf),msgid);	//buf,sizeof(buf),puback,dup,id
			if (len <= 0 || len > NET_MSS_LEN)
			{
				printf("QOS1 Serialize PUBACK exceed length:%d\r\n",len);
				goto exit;
			}
		
			if(transport_sendPacketBuffer(MQ_NET_ID,buf,len)<=0)
			{
				goto exit;
			}		
      else
      {
     		rc = MQ_SUCCESS;
	  		goto exit;           // qos1 return
      }  
		
			
    }
    else if (message->qos == QOS2)         /*QOS2 publis*/
    {
			//超时等待接收
			if(rc<=0)
			{
				printf("QOS2 puback respone timeout\r\n");
				goto exit;
			}

			if (MQTTDeserialize_publish(&dup, &qos, &retained, &msgid,receivedTopic,&payload_in,&payloadlen_in,buf, l) != 1)    //&payload_in  debug
      {
      	printf("no PUBACK,fault\r\n");
      	goto exit; 
      }
			//==================发送PUBREC 应答信息
			/* send PUBREC to MQ server*******/
			len = MQTTSerialize_ack(buf, sizeof(buf),PUBREC,0,msgid);	//buf,sizeof(buf),puback,dup,id
			if (len <= 0 || len > NET_MSS_LEN)
			{
				printf("QOS2 Serialize PUBREC exceed length:%d\r\n",len);
				goto exit;
			}
			
			if(transport_sendPacketBuffer(MQ_NET_ID,buf,len)<=0)
			{
				goto exit;
			}		
			//===================等待PUBREL
			if(transport_recvPacketBuffer(MQ_NET_ID,buf,&l,3000)<=0)
			{
				printf("QOS2 PUBREL respone timeout\r\n");
				goto exit;
			}
			
			if (MQTTDeserialize_ack(&type, &dup, &mypacketid, buf, sizeof(buf)) != 1 || type != PUBREL)
			{
				printf("QOS2 PUBREL,fault:%d\r\n",type);
				goto exit; // qos2 fault response return
			}					
			//==================发送PUBCOMP 完成应答信息
			len = MQTTSerialize_ack(buf, sizeof(buf),PUBCOMP,0,mypacketid);	//buf,sizeof(buf),puback,dup,id
			if (len <= 0 || len > NET_MSS_LEN)
			{
				printf("QOS2 Serialize PUBCOMP exceed length:%d\r\n",len);
				goto exit;
			}
			
			if(transport_sendPacketBuffer(MQ_NET_ID,buf,len)<=0)
			{
				goto exit;
			}	
      else
      {
//				printf("rc = MQ_SUCCESS\r\n");
     		rc = MQ_SUCCESS;
	  		goto exit;           // qos1 return
      }  
    }
		/* QOS >2 unkown*/
    else
    {
    	printf("QOS%d unkonw command\r\n",message->qos);
    	rc = MQ_FAILURE;
    }

exit:
		message->payloadlen=payloadlen_in;
		message->payload=payload_in;    //payload_in

    return rc;
	
 }

