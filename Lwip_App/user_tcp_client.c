#include "lwip/api.h"
#include "string.h"
#include "stdbool.h"
#include "user_tcp_client.h"
#include "cJSON.h"
#include "parameter.h"
#include "common.h"
#include "main.h"
#include "console.h"
#include "mqttTask.h"
#include "pca9554.h"
#include "lcdTask.h"
#include "stdio.h"
#include "Wt2003hx.h"
#include "wifi.h"
/*mqtt def about*/
MQTT_TOPIC_TYPE gTopicInfo;

/*end*/
struct netconn*  gNetSock[NET_SOCK_NUM];
unsigned char gNetSockStatus[NET_SOCK_NUM];
//static char gRecvNetData[RECV_BUF_MAX-1];
//char gSendNetData[RECV_BUF_MAX-1];
//static unsigned char gRecvCheckData[100];

/**������ź���������ⷢ���¼�**/
SemaphoreHandle_t					TcpSend_Semaphore; 
SemaphoreHandle_t					COM1_Semaphore; 	//�źŴ����ͨ��uart1
SemaphoreHandle_t	NetSendMutexeSemaphore;


/*mqtt fun about*/
/*�����ʼ��
type:��Ʒ��ʶ id:�豸id
*/
void Topic_Init(MQTT_TOPIC_TYPE* topic,char* type,char*id)
{
	memset(topic,0,sizeof(MQTT_TOPIC_TYPE));
	//ָ���
	strcat(topic->cmdReply,"command/reply/");
	strcat(topic->cmdReply,type);
	strcat(topic->cmdReply,"/");
	strcat(topic->cmdReply,id);    //��֮ǰ�� strcat(topic->cmdReply,id);���Ƕ��ĵ����������#��id��ֵ������client id�ϡ������������ֱ�Ӹ�Ϊ"#"��

	//ָ���
	strcat(topic->cmdPost,"command/post/");
	strcat(topic->cmdPost,type);
	strcat(topic->cmdPost,"/");
	strcat(topic->cmdPost,id);

	//�㲥����
	strcat(topic->boardReply,"broadcast/reply/");
	strcat(topic->boardReply,type);
	//�㲥����
	strcat(topic->boardPost,"broatcast/post/");
	strcat(topic->boardPost,type);
	//�ɼ�����
	strcat(topic->streamPost,"stream/post/");  //stream/post/
	strcat(topic->streamPost,type);
	strcat(topic->streamPost,"/");
	strcat(topic->streamPost,id);
}

void mqtt_topic_sub(MQTT_TOPIC_TYPE*topic,MQTTMessage*messageRecv,MQTTMessage*messageSend)
{

	//��������

	MQTTSubscribe(topic->cmdReply,messageRecv);
	
//	MQTTSubscribe(topic->boardReply,messageRecv);

	//��������
//	MQTTSubscribe(topic->cmdPost,messageSend);
	

//	MQTTSubscribe(topic->boardPost,messageSend);

//	MQTTSubscribe(topic->streamPost,messageSend);


	return ;
}

/*end*/
bool  TcpSend_TaskSemaphore_Init(void)
{
	TcpSend_Semaphore = xSemaphoreCreateBinary();

	if (TcpSend_Semaphore == NULL )
	{
//		printf("TcpSend_Semaphore create fail\r\n");
//		NVIC_SystemReset();
		return false;
	}
	 return true;
}

bool NetSend_MutexeSemaphore_Init(void)
{
	NetSendMutexeSemaphore		= xSemaphoreCreateMutex();
	if (NetSendMutexeSemaphore == NULL )
	{
//		NVIC_SystemReset();
		return false;
	}
	 return true;
}


bool COM1_Semaphore_Init(void)
{
	COM1_Semaphore = xSemaphoreCreateBinary();

	if (COM1_Semaphore == NULL )
	{
//		printf("TcpSend_Semaphore create fail\r\n");
//		NVIC_SystemReset();
		return false;
	}
	 return true;
}

/**
���ܣ���һ��tcp�����׽��� ��������
���أ�1�ɹ����� , ʧ�ܷ���-1(false)
**/
int transport_open(unsigned char netId,unsigned char* MQ_Address,unsigned short remotePort)
{	
	ip_addr_t ipaddr;
	err_t connect_err;
	/**���id�Ϸ���**/
	if(netId>NET_SOCK_NUM-1)
	{
		return -1;
	}
	
	/**����Ѿ������ͷ�**/
	if(NET_LINK_OPEN==gNetSockStatus[netId])
	{
		gNetSockStatus[netId]=NET_LINK_CLOSE;
		netconn_close(gNetSock[netId]);
		netconn_delete(gNetSock[netId]);
	}
	gNetSockStatus[netId]=NET_LINK_CLOSE;
	
	/**���ݳ�ʼ��  **/
	IP4_ADDR(&ipaddr, MQ_Address[0], MQ_Address[1],MQ_Address[2],MQ_Address[3]);
	/* Create a new connection identifier. */
	gNetSock[netId]= netconn_new(NETCONN_TCP);
	if (gNetSock[netId]!=NULL)
	{
		//err = netconn_bind(conn, NULL, 5001);	//����Ҫ��
		connect_err = netconn_connect(gNetSock[netId], &ipaddr, remotePort);
		/* Process the new connection. */
		if (connect_err == ERR_OK)
		{
			gNetSockStatus[netId]=NET_LINK_OPEN;
			return 1;
		}else{
			netconn_delete(gNetSock[netId]);
			return -1;
		}
			
	}else
	{
		return -1;
	}
}

/**
���ܣ���һ���򿪵��׽��־���Ϸ�������NetSendMutexeSemaphore ����Ҫ�ź�������Ϊֻ��һ���߳���ʹ�÷���
���أ��ɹ�����ʵ��д����ֽ��� ʧ�ܷ���-1
**/
int transport_sendPacketBuffer(unsigned char netId,unsigned char* send,short int sendLen)
{
	if(netId==1)	//data port
	{
			unsigned int w_len;

			if(netId>NET_SOCK_NUM-1||NET_LINK_OPEN!=gNetSockStatus[netId])
			{
				return -1;
			}
			/*����һ֡����**/			
			gNetSock[netId]->send_timeout=100;
//			if(gEthStatus==false||gEthSendStatus==false)
//			{
//				return -1;
//			}
			if(gEthStatus==false)
			{
				return -1;
			}
			if(ERR_OK==netconn_write_partly(gNetSock[netId], send, sendLen,NETCONN_COPY, &w_len))
			{
				return w_len;
			}else{
				return -1;
			}

	}else{	//ctl port
			unsigned int w_len;

			if(netId>NET_SOCK_NUM-1||NET_LINK_OPEN!=gNetSockStatus[netId])
			{
				return -1;
			}
			/*����һ֡����**/			
			gNetSock[netId]->send_timeout=100;
//			if(gEthStatus==false||gEthRecvStatus==false)
//			{
//				return -1;
//			}
			if(gEthStatus==false)
			{
				return -1;
			}
			if(ERR_OK==netconn_write_partly(gNetSock[netId], send, sendLen,NETCONN_COPY, &w_len))
			{
				return w_len;
			}else{
				return -1;
			}
		}
}


/**
���ܣ���һ���Ѿ��򿪵������׼����Ͻ�������,�����������ջ������������ݶ���
�ɹ������mq_Send ,�������ճ��ȸ�ֵ��l ,�ɹ�����1 ��ʱ����0 ʶ�𷵻�-1
**/
int transport_recvPacketBuffer(unsigned char netId,unsigned char* recv,unsigned short *recvLen,int timeout)
{
	err_t recv_err;
	void *data;
	unsigned short readLen;
	struct netbuf *buf;
	if(netId>NET_SOCK_NUM-1||NET_LINK_OPEN!=gNetSockStatus[netId])
	{
		return -1;
	}
	*recvLen=0;
	
	gNetSock[netId]->recv_timeout=timeout;	//���Ҫʹ�ܽ��ճ�ʱ ����Ҫ�޸� opt.h  LWIP_SO_RCVTIMEO			1
		/*��ʱ�ȴ�����һ֡��recv_timeout��**/
	recv_err=netconn_recv(gNetSock[netId], &buf);
	if (ERR_OK==recv_err)
	{
		do
		{
			netbuf_data(buf, &data, &readLen);
			/**�������ݵ�recv,����recvLen**/
			memcpy(recv,data,readLen);
			*recvLen+=readLen;
			if(*recvLen>RECV_BUF_MAX)
			{
				break;
			}
			/**��д����*/
			//netconn_write(gNetSock[netId], data,readLen, NETCONN_COPY);
		}while (netbuf_next(buf) >= 0);
		netbuf_delete(buf);	
		return 1;
	}else if(ERR_TIMEOUT==recv_err)	//�����˳�ʱ����
	{
		*recvLen=0;
		return 0;
	}else{		//��Ҫ���³�ʼ��������
		*recvLen=0;
		return -1;
	}
	
}

/**
���ܣ��ͷ�һ���Ѿ��򿪵��׽���
**/
void transport_release(int netId)
{
	if(netId>NET_SOCK_NUM-1)
	{
		return;
	}
	gNetSockStatus[netId]=NET_LINK_CLOSE;
	netconn_close(gNetSock[netId]); //��������������ȴ��źţ�����Է�û�л�Ӧ���ܻ�ȴ��ܳ�ʱ��
	netconn_delete(gNetSock[netId]);
}

#if 0
/**
���ܣ���һ���Ѿ��򿪵�tcp�����׽��� ���ӵ�Զ�˵�ַ ������Դ
���أ��ɹ����� ����״̬NET_LINK_LINKED/NET_LINK_AGAIN, ʧ�ܷ���-1(false)
ע�� ���Է��� ���Ӳ��ɹ�ʱ ϵͳ�Զ��ͷ���������Դ ��Ŀǰ�޷������رպ����� �����ͷ�
**/
int transport_connect(unsigned char netId,unsigned char* MQ_Address,unsigned short remotePort)
{	
	return -1;
#if 0
	ip_addr_t ipaddr;
//	err_t err,recv_err;
	err_t connect_err;
	/**���id�Ϸ���**/
	if(netId>NET_SOCK_NUM-1)
	{
		return -1;
	}

	/**���ݳ�ʼ��  **/
	IP4_ADDR(&ipaddr, MQ_Address[0], MQ_Address[1],MQ_Address[2],MQ_Address[3]);
//	IP4_ADDR(&ipaddr, 192, 168,2,209);
	/* Create a new connection identifier. */
	if (gNetSock[netId]!=NULL)
	{
		//err = netconn_bind(conn, NULL, 5001);	//����Ҫ��
		connect_err = netconn_connect(gNetSock[netId], &ipaddr, remotePort);
//		connect_err = netconn_connect(gNetSock[netId], &ipaddr, 5014);
		/* Process the new connection. */
		if (connect_err == ERR_OK)
		{
			gNetSockStatus[netId]=NET_LINK_LINKED;
			return NET_LINK_LINKED;
		}else if(connect_err == ERR_ABRT||connect_err == ERR_RST){
			return NET_LINK_AGAIN;
		}else{	//������� ���³�ʼ��
			return -1;
		}
			
	}else
	{
		return -1;
	}
	#endif
}


/**
���ܣ��Ͽ�һ���Ѿ��򿪵�����,ֻ�رղ��ͷ�
ע��δ����
**/
void transport_deconnect(int netId)
{
	#if 0	
	if(netId>NET_SOCK_NUM-1)
	{
		return;
	}
	gNetSockStatus[netId]=NET_LINK_CLOSE;
	netconn_close(gNetSock[netId]);
	#endif
}
#endif

/**�̣߳�Tcp ����ָ��ͻ���
ע��������˵����Ӧ��ʹ�÷����������ǿ��ǵ��Ժ��ڹ�����ʹ�� ����������Ϊ�ͻ���**/
void StartMqttClientTask(void const *arg)		//�ǵ�����ջ�ռ�ߴ� ������ܵ������
{
	int rc;
	unsigned char buf[60]={0,};
	
	MQTTMessage messageSend;
	MQTTMessage messageRecv;	
	MQTTString	messageTopic;
	//�����ʼ��
	sprintf((char*)buf,"%d",gDeviceParam.devId);
	Topic_Init(&gTopicInfo,(char*)gDeviceParam.productKey,(char*)buf);
	memset(buf,0,sizeof(buf));
	//���Ӳ�����ʼ��
	MQTT_ConnectData_Init(&MQTTPacket_ConnectData_Initializer);
	//
	if(MQTTConnect(&MQTTPacket_ConnectData_Initializer)<0)
	{
		MQTTDisConnect(&MQTTPacket_ConnectData_Initializer);
		gEthRecvStatus=false;	
	}else{
		gEthRecvStatus=true;
	}

	//������Ϣ��ʼ��
	messageSend.dup = 0;//��Ϣ�����ظ�
	messageSend.qos = QOS0;//��Ϣ����QoS
	messageSend.retained = 0;//����
	messageSend.payload =NULL;//gSendNetData;
	messageSend.payloadlen = 0;//strlen(gSendNetData);

	//������Ϣ��ʼ��
	messageRecv.dup = 0;
	messageRecv.qos = QOS2;
	messageRecv.retained = 0;
	messageRecv.payload =NULL;//gRecvNetData;
		
	//��Ϣ����
	if(gEthRecvStatus==true)
	{
		mqtt_topic_sub(&gTopicInfo,&messageRecv,&messageSend);
		if(gGlobalData.ResetStatus==true)
		{
			gGlobalData.ResetStatus=false;
			do_work_ctl(3);
			send_QRInfo(gDeviceParam.qrbuf,strlen((const char *)gDeviceParam.qrbuf));   //���Ͷ�ά�뵽��Ļ�������ʾ
		}
		cnt_heartbag = 0;											  	//���������������������		
		gGlobalData.heartbag_flage=1;			         //�����Ϸ��������������������ս���
	}		
	while(1)
	{
		osDelay(1);	
		if(gEthRecvStatus==false)
		{
			MQTTDisConnect(&MQTTPacket_ConnectData_Initializer);
			if(MQTTConnect(&MQTTPacket_ConnectData_Initializer)<0)
			{
				gEthRecvStatus=false;
				if(gGlobalData.rj45Status ==true && gGlobalData.conFlage==1){
						osDelay(500);
						send_NetSync(0);		
						osDelay (100);
						send_duan_wang(1);   				
				}
				continue;
			}else{
				gEthRecvStatus=true;              																					
				wifi_deinit();   																				 //ǿ�ƶϿ�wifi		 	//wifi����
				HAL_UART_Transmit_DMA(&huart6,"AT+QMTCLOSE=0\r\n",sizeof("AT+QMTCLOSE=0\r\n"));				  //4g����
				gGlobalData.yd4gStatus=false;								
				gGlobalData.isConnect=1;																																//��������
				gGlobalData.conFlage=1;
				gGlobalData.netKind = 1;																																//����
				mqtt_topic_sub(&gTopicInfo,&messageRecv,&messageSend);																																	//ȥ��������ʾ 1900  //**//
				if(gGlobalData.ResetStatus==true)
				{
					gGlobalData.ResetStatus=false;
					do_work_ctl(3);
					send_QRInfo(gDeviceParam.qrbuf,strlen((const char *)gDeviceParam.qrbuf));   //���Ͷ�ά�뵽��Ļ�������ʾ
				}
				send_NetSync(1);  																																		  //����� 1601  //**//
				osDelay (100);
				send_duan_wang(0);
				cnt_heartbag = 0;																																			 //���������������������
				gGlobalData.heartbag_flage=1;			      																						   //�����Ϸ��������������������ս���				
			}
		}				
		//������Ϣ����		
		rc=MQTTSubscribe_RecvMessage(&messageRecv,&messageTopic);
		if(rc==MQ_SUCCESS)	//MQ_SUCCSS 
		{
//			printf("Subscribe_RecvMessage Success\n");
			//��Ϣ����
			if(messageRecv.payloadlen>0)
			{
				mqttMessageProcess(messageRecv.payload,messageRecv.payloadlen,messageTopic.lenstring.data,messageTopic.lenstring.len);
			}
		}else if(rc==MQ_PINGRESP){
			gGlobalData.heart_count = 0;                
		}else{
//			gMqttLinkStatus=false;
			osDelay(5000);       
			gEthRecvStatus=false;
		}				
	} 	 
}

bool Act_music=false;
/*���ݴ���*/
int netData_process(char *payload,int payloadLen)
{
	//memcpy(debug2,payload,sizeof(debug2));
	int ret,i,arryNum,j;
	char strBuf[200]={0,};
	long reet;
	char *pend;
		
	uint8_t intIp[4]={0,},intMask[4]={0,},intGate[4]={0,},intMac[12]={0,};
	bool ipTrue=false,maskTrue=false,gateTrue=false,macTrue=false;
	int tDataStartFlag,tDataStartFlag2;
	int  functionFlag,valueFlag;
	cJSON *root = NULL, *item = NULL,*itemSub=NULL;
	cJSON *cjsonArr=NULL,*cjsonArr2=NULL,*arryItem=NULL;
	/*���ս��� ���лỰid�͹��ܾͻظ�*/
	if(Act_music==false){  //��������һ������
			Act_music=true;
			Send_Output_Change(1);
			osDelay(20);
	}
	memset(strBuf,0,sizeof(strBuf));
	root=cJSON_Parse(payload);
	if (root == NULL || !cJSON_IsObject(root)) {
		Send_Fix_Ack(0,STATUS_FAIL,"json body fail");	//tmp
		cJSON_Delete(root);
		return -1;
	}
	#if 2//����Э��ͷ
	cnt_heartbag = 0;                               //ͨ���ϾͲ�����������
	/*ȡ�豸id*/
	item=cJSON_GetObjectItem(root, KEY_DEV_ID);
	if (item == NULL || !cJSON_IsNumber(item)) {    //item->type=16
		item=NULL;   //item=604131128
		cJSON_Delete(root);
		return -1;
	}
	valueFlag=item->valueint;
	if(gDeviceParam.devId!=valueFlag)
	{
		return -1;
	}

	/*ȡ�����ֶ�*/
	item=cJSON_GetObjectItem(root, KEY_FUN);
	if (item == NULL || !cJSON_IsNumber(item)) {
		item=NULL;
		cJSON_Delete(root);
		Send_Fix_Ack(0,STATUS_FAIL,"key-fun fail");
		return -1;
	}
	functionFlag=item->valueint;   
	
	
	/*ȡ�Ựid�ֶ�*/
	item=cJSON_GetObjectItem(root, KEY_SESSION);
	if (item == NULL || !cJSON_IsNumber(item)) {
		item=NULL;
		Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-session fail");
		cJSON_Delete(root);
		return -1;
	}

//	strcpy((char*)gGlobalData.sessionId,item->valuestring);
	gGlobalData.sessionId=item->valueint; 
	
		//ȡʱ��

	item=cJSON_GetObjectItem(root, KEY_TIME);
	if (item == NULL || !cJSON_IsString(item)) {
		Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-time");
		cJSON_Delete(root);
		return -1;
	}
//		memset(strBuf,0,sizeof(strBuf));
	strcpy((char *)gGlobalData.ack_time ,item->valuestring);
	GBK_Timeprocess((char *)gGlobalData.ack_time );		

	#endif
	
	/*���ݹ��ܽ�����������*/
	switch(functionFlag)
	{
		//�ϴ����
		case FUN_COLLECT_ARG:      //  1 ���òɼ�����
		#if 1//�·����ݲɼ�����
		if(gGlobalData.curWorkState != WORK_START)              //û���������ʱ��ɼ��������յ��Ż��ϴ��ȴ�״̬���Ѿ�����������ʱ��Ͳ�����ϴ���������״̬�����ȴ�
			gGlobalData.cur_heart_state	= WAITING;	 
    printf("Recv Collect Preject!\r\n");
		item=cJSON_GetObjectItem(root, KEY_DATA_UPTIME);
		if (item == NULL || !cJSON_IsNumber(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-uptime object format err");
			cJSON_Delete(root);
			cjsonArr=NULL;
			item=NULL;
			return -1;
		}
		gGlobalData.useWorkArg[0].upTime=item->valueint;
		gGlobalData.useWorkArg[1].upTime=item->valueint;
		
		//ȡ����ģʽ 1���������  2���ֲ�����
		item=cJSON_GetObjectItem(root, KEY_TYPE);
		if (item == NULL || !cJSON_IsNumber(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-type object err");
			cJSON_Delete(root);
			cjsonArr=NULL;
			item=NULL;
			return -1;
		}

		gGlobalData.curWorkMode=item->valueint;
		
		//ȡ��һ���������
		item=cJSON_GetObjectItem(root,  KEY_WORK);
		if (item == NULL || !cJSON_IsObject(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL, "key-work object format err");
			cJSON_Delete(root);
			cjsonArr=NULL;
			item=NULL;
			return -1;
		}
		
			//ȡwork��������ݶ���  interval
		itemSub=cJSON_GetObjectItem(item, KEY_DATA_UPTIME);
		if (item == NULL || !cJSON_IsNumber(itemSub)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key_interval err");
			cJSON_Delete(root);
			cjsonArr=NULL;
			item=NULL;
			return -1;
		}

		gGlobalData.useWorkArg[1].upTime=itemSub->valueint;
			
			//totalTime �ɼ���ʱ������ʱ
		itemSub=cJSON_GetObjectItem(item,KEY_TIME_CJ);
		if (itemSub == NULL || !cJSON_IsNumber(itemSub)) {
				Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-totalTime err");
				cJSON_Delete(root);
				cjsonArr=NULL;
				itemSub=NULL;
				item=NULL;
				return -1;
		}	
		gGlobalData.Alltime=itemSub->valueint;	
		//rate
		itemSub=cJSON_GetObjectItem(item,KEY_DATA_RATE);
		if (itemSub == NULL || !cJSON_IsNumber(itemSub)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-rate err");
			cJSON_Delete(root);
			cjsonArr=NULL;
			itemSub=NULL;
			item=NULL;
			return -1;
		}
			 //gGlobalData.freqUnitTime=itemSub->valueint;  //��������֮��Ĳ������ʱ�� ms
		gGlobalData.useWorkArg[0].rateN=itemSub->valueint;	
		
		/**��Ա�ڵ�inputs����**/
		//inputs
		cjsonArr2=cJSON_GetObjectItem(item,KEY_INPUTS);
		if (cjsonArr2 == NULL || !cJSON_IsArray(cjsonArr2)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL, "key-inputs err");
			cJSON_Delete(root);
			cjsonArr=NULL;
			cjsonArr2=NULL;
			item=NULL;
			return -1;
		}
			
		//inputs����ߴ�
		arryNum=cJSON_GetArraySize(cjsonArr2);
		if(arryNum<1||arryNum>32)
		{
			Send_Fix_Ack(functionFlag,STATUS_FAIL, "key-inputs num err");
			cJSON_Delete(root);
			cjsonArr=NULL;
			cjsonArr2=NULL;
			item=NULL;
			return -1;
		}
		//inputs��Ա
		gGlobalData.useWorkArg[0].chanelNum=arryNum;
		for(i=0;i<arryNum;i++)
		{
			arryItem=cJSON_GetArrayItem(cjsonArr2,i);
			if (arryItem == NULL || !cJSON_IsNumber(arryItem)) {
				Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-inputs obj err");
				cJSON_Delete(root);
				cjsonArr=NULL;
				cjsonArr2=NULL;
				return -1;
			}
			ret=arryItem->valueint;
			if(ret>33||ret<1)
			{
				Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-inputs val err");
				cJSON_Delete(root);
				cjsonArr=NULL;
				cjsonArr2=NULL;
				return -1;
			}
			gGlobalData.useWorkArg[0].inputs[i]=ret;
		}
			
		/**��Ա�ڵ�outs�����Ա**/
			//outs
		cjsonArr2=cJSON_GetObjectItem(item,KEY_OUTS);
		if (cjsonArr2 == NULL || !cJSON_IsArray(cjsonArr2)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL, "key-outs err");
			cJSON_Delete(root);
			cjsonArr=NULL;
			cjsonArr2=NULL;
			item=NULL;
			return -1;
		}
		
			//outs����ߴ�
		arryNum=cJSON_GetArraySize(cjsonArr2);
		if(arryNum<1||arryNum>32)
		{
			Send_Fix_Ack(functionFlag,STATUS_FAIL, "key-outs num err");
			cJSON_Delete(root);
			cjsonArr=NULL;
			cjsonArr2=NULL;
			item=NULL;
			return -1;
		}
		//outs��Ա
		gGlobalData.useWorkArg[0].chanelNum=arryNum;
		for(i=0;i<arryNum;i++)
		{
			arryItem=cJSON_GetArrayItem(cjsonArr2,i);
			if (arryItem == NULL || !cJSON_IsNumber(arryItem)) {
				Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-inputs obj err");
				cJSON_Delete(root);
				cjsonArr=NULL;
				cjsonArr2=NULL;
				return -1;
			}
			ret=arryItem->valueint;
			if(ret>33||ret<1)
			{
				Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-inputs val err");
				cJSON_Delete(root);
				cjsonArr=NULL;
				cjsonArr2=NULL;
				return -1;
			}
			gGlobalData.useWorkArg[0].outs[i]=ret;
		}	

		//time
		itemSub=cJSON_GetObjectItem(item,KEY_TIME_ZL);
		if (itemSub == NULL || !cJSON_IsNumber(itemSub)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-time err");
			cJSON_Delete(root);
			cjsonArr=NULL;
			itemSub=NULL;
			item=NULL;
				return -1;
		}
		gGlobalData.useWorkArg[0].timeCheck=itemSub->valueint;
 
		Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
		osDelay(50);
		//////////////2023.02.03���� WWJZ
	//	if (gGlobalData.curWorkMode==WORK_MODE_WT){
			
		//gGlobalData.curWorkState=WORK_START;//������״̬��ʼ  kardos 2023.02.03
		send_LcdWorkStatus(2);//kardos 2023.02.03 �޸��豸����״̬Ϊ���ȴ�������
		osDelay(50);
		Countdown_Treat(gGlobalData.Alltime);
		osDelay(50);
		gGlobalData.curWorkMode = WORK_MODE_ZT;//2023.1.30����
	//	}
//		gGlobalData.Send_Client_Over= true;	  //  Ӧ��
		break;
		#endif
		
		
		
		#if 4 //�·����Ʒ���
		//�·��������Ʒ���
		case FUN_COLLECT_TREAT:   
			if(gGlobalData.Alltime!=0){    //by yls 2023/6/13 ��ֹ����ʱ�ظ�����
				break;
			} 
			item=cJSON_GetObjectItem(root, KEY_DATA_UPTIME);
			if (item == NULL || !cJSON_IsNumber(item)) {
				Send_Fix_Ack(functionFlag,STATUS_FAIL,KEY_DATA_UPTIME);
				cJSON_Delete(root);
				cjsonArr=NULL;
				item=NULL;
				return -1;
			}
			gGlobalData.useWorkArg[0].upTime=item->valueint;
			gGlobalData.useWorkArg[1].upTime=item->valueint;
			
			//ȡ��һ���������
			cjsonArr=cJSON_GetObjectItem(root,  KEY_WORK);
			if (cjsonArr == NULL || !cJSON_IsArray(cjsonArr)) {
				Send_Fix_Ack(functionFlag,STATUS_FAIL, "key-work array format err");
				cJSON_Delete(root);
				cjsonArr=NULL;
				item=NULL;
				return -1;
			}
			//��ȡwork����ߴ�
			gGlobalData.useWorkArg[0].chanelNum=cJSON_GetArraySize(cjsonArr);
			
			for(j=0;j<gGlobalData.useWorkArg[0].chanelNum;j++)
			{
				//��ȡwork��һ�������Ա
				item=cJSON_GetArrayItem(cjsonArr,j);
				if (item == NULL || !cJSON_IsObject(item)) {
					Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-work obj err");
					cJSON_Delete(root);
					cjsonArr=NULL;
					itemSub=NULL;
					item=NULL;
					return -1;
				}
				/**��Ա�ڵ���ͨ����**/
			
				//level
				itemSub=cJSON_GetObjectItem(item,KEY_LEVEL);
				if (itemSub == NULL || !cJSON_IsNumber(itemSub)) {
					Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-level err");
					cJSON_Delete(root);
					cjsonArr=NULL;
					itemSub=NULL;
					item=NULL;
					return -1;
				}
				gGlobalData.useWorkArg[j].level=itemSub->valueint;
					
					//inputs    
				itemSub=cJSON_GetObjectItem(item,KEY_INPUTS);
				if (itemSub == NULL || !cJSON_IsNumber(itemSub)) {
					Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-inputs err");
					cJSON_Delete(root);
					cjsonArr=NULL;
					itemSub=NULL;
					item=NULL;
					return -1;
				}
					
				gGlobalData.useWorkArg[0].inputs[j]=itemSub->valueint;

					//outs    
				itemSub=cJSON_GetObjectItem(item,KEY_OUTS);
				if (itemSub == NULL || !cJSON_IsNumber(itemSub)) {
					Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-outs err");
					cJSON_Delete(root);
					cjsonArr=NULL;
					itemSub=NULL;
					item=NULL;
					return -1;
				}						
				gGlobalData.useWorkArg[0].outs[j]=itemSub->valueint;
		 
					//wave
				itemSub=cJSON_GetObjectItem(item,KEY_WAVE_TREAT);
				if (itemSub == NULL || !cJSON_IsNumber(itemSub)) {
					Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-wave err");
					cJSON_Delete(root);
					cjsonArr=NULL;
					itemSub=NULL;
					item=NULL;
					return -1;
				}
				gGlobalData.useWorkArg[j].waveTreat=itemSub->valueint;
				//freqTreat
				itemSub=cJSON_GetObjectItem(item,KEY_FREQ_TREAT);
				if (itemSub == NULL || !cJSON_IsNumber(itemSub)) {
					Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-freq err");
					cJSON_Delete(root);
					cjsonArr=NULL;
					itemSub=NULL;
					item=NULL;
					return -1;
				}
				if(itemSub->valueint<=10000) gGlobalData.useWorkArg[j].freqTreat=itemSub->valueint;     //2023/7/2 ����ֻ�������2000hz��Ƶ��  by yls   ���ڿ��Դﵽ1w hz  2023/7/22
				//time
				itemSub=cJSON_GetObjectItem(item,KEY_TIME_ZL);
				if (itemSub == NULL || !cJSON_IsNumber(itemSub)) {
					Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-time err");
					cJSON_Delete(root);
					cjsonArr=NULL;
					itemSub=NULL;
					item=NULL;
					return -1;
				}
				gGlobalData.useWorkArg[j].timeTreat=itemSub->valueint;	
				gGlobalData.Alltime+=gGlobalData.useWorkArg[j].timeTreat;	
					
				//waitTime
				itemSub=cJSON_GetObjectItem(item,KEY_TIME_WAIT);
				if (itemSub == NULL || !cJSON_IsNumber(itemSub)) {
					Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-waitTime err");
					cJSON_Delete(root);
					cjsonArr=NULL;
					itemSub=NULL;
					item=NULL;
					return -1;
				}
				gGlobalData.useWorkArg[j].waitTime=itemSub->valueint;
				gGlobalData.Alltime+=gGlobalData.useWorkArg[j].waitTime;
			}
			Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
			
			Level= gGlobalData.useWorkArg[0].level;//��ʼĬ�ϵ�λ��ֵ 
			set_sampleMode(MODE_ZL);//����һ����������ģʽ
			gGlobalData.curWorkMode = WORK_MODE_ZL;//2023.1.30����
			gGlobalData.curWorkState=WORK_START;//������״̬��ʼ  kardos 2023.02.03
			gGlobalData.cur_heart_state=WORKING;
			Countdown_Treat(gGlobalData.Alltime);
	//		gGlobalData.Send_Client_Over= true;	   //�ɼ��������·������Ʒ���    Ӧ��
			printf("Recv ZL Preject!\r\n");
			break;
		#endif
		
		
		
		//�ɼ�����
		case FUN_COLLECT_CTL:	
//			//workmode		
			item=cJSON_GetObjectItem(root, KEY_VALUE);
			if (item == NULL || !cJSON_IsNumber(item)) {
				Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-value err");
				cJSON_Delete(root);
				return -1;
			}
			tDataStartFlag2=item->valueint;
//			gGlobalData.curWorkState=tDataStartFlag2;
			
			if(tDataStartFlag2!=WORK_START && tDataStartFlag2!=WORK_PAUSE && tDataStartFlag2!=WORK_STOP)
			{	
				Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-value err");
			}else if(tDataStartFlag2 == WORK_STOP){              //��λ�Լ���ack
				
				do_work_ctl(tDataStartFlag2);
			}
			else {
				Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
				do_work_ctl(tDataStartFlag2);
			}
				
				
		break;
//		case FUN_DEV_OFF:	//���ػ�����
		case FUN_DEV_RESET:	//�豸����
			Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
			set_sampleMode(MODE_CLOSE);
			HAL_PCA9554_outputAll(0);
			osDelay(100);
			NVIC_SystemReset();
		break;
		case  FUN_DEV_LOCK:	//�豸��
		item=cJSON_GetObjectItem(root, KEY_VALUE);
		if (item == NULL || !cJSON_IsNumber(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-value fail");
			cJSON_Delete(root);
			return -1;
		}
		tDataStartFlag=item->valueint;
		
		if(tDataStartFlag==LOCK_YES)
		{
			gDeviceParam.devLock=LOCK_YES;
		}else if(tDataStartFlag==LOCK_NO){
			gDeviceParam.devLock=LOCK_NO;
		}else{
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-value err");
			break;
		}
		Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
		break;
			
		case  FUN_SYS_CHECK://ϵͳ�豸���
			Send_SysCheck_Ack(functionFlag,STATUS_OK,"ok");
		break;
		case FUN_JIANCHE_DATA:	//�����ظ�
				//Send_JianChe_Ack(functionFlag,STATUS_OK,"ok");
			gGlobalData.isConnect=1;//����״̬ok
			gGlobalData.heart_count = 0; //������������

		break;
		case FUN_RJ45_SET:
		//ip
				item=cJSON_GetObjectItem(root, KEY_IP);
			if (item == NULL || !cJSON_IsString(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-ip");
			cJSON_Delete(root);
			return -1;
		}
		strcpy(strBuf,item->valuestring);
		ipTrue=isVaildIp(strBuf,intIp);
		//mask
		item=cJSON_GetObjectItem(root, KEY_MASK);
			if (item == NULL || !cJSON_IsString(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-mask");
			cJSON_Delete(root);
			return -1;
		}
		memset(strBuf,0,sizeof(strBuf));
		strcpy(strBuf,item->valuestring);
		maskTrue=isVaildIp(strBuf,intMask);
		//gate
		item=cJSON_GetObjectItem(root, KEY_GATE);
			if (item == NULL || !cJSON_IsString(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-gate");
			cJSON_Delete(root);
			return -1;
		}
		memset(strBuf,0,sizeof(strBuf));
		strcpy(strBuf,item->valuestring);
		gateTrue=isVaildIp(strBuf,intGate);
		//mac
		item=cJSON_GetObjectItem(root, KEY_MAC);
			if (item == NULL || !cJSON_IsString(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-mac");
			cJSON_Delete(root);
			return -1;
		}
		memset(strBuf,0,sizeof(strBuf));
		strcpy(strBuf,item->valuestring);
		macTrue=isVaildMac(strBuf,intMac);

		if(ipTrue==true&&gateTrue==true&&maskTrue==true&&macTrue==true)
		{
			for(i=0;i<4;i++)
			{
				gDeviceParam.rj45Arg.IpAddress[i]=intIp[i];
				gDeviceParam.rj45Arg.MaskAddress[i]=intMask[i];
				gDeviceParam.rj45Arg.GateAddress[i]=intGate[i];
			}
			for(i=0;i<6;i++)
			{
				gDeviceParam.rj45Arg.MacAddress[i]=intMac[i];
			}
			
			Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
			Save_Parameter();
		}else{
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"errno ip");
		}
		break;
		
		case FUN_RJ45_GET:
			Send_Rj45_Ack(functionFlag,STATUS_OK,"ok");
			break;
		case FUN_WIFI_SET:
			//ip
				item=cJSON_GetObjectItem(root, KEY_IP);
			if (item == NULL || !cJSON_IsString(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-ip");
			cJSON_Delete(root);
			return -1;
		}
		strcpy(strBuf,item->valuestring);
		ipTrue=isVaildIp(strBuf,intIp);
		//mask
		item=cJSON_GetObjectItem(root, KEY_MASK);
			if (item == NULL || !cJSON_IsString(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-mask");
			cJSON_Delete(root);
			return -1;
		}
		memset(strBuf,0,sizeof(strBuf));
		strcpy(strBuf,item->valuestring);
		maskTrue=isVaildIp(strBuf,intMask);
		//gate
		item=cJSON_GetObjectItem(root, KEY_GATE);
			if (item == NULL || !cJSON_IsString(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-gate");
			cJSON_Delete(root);
			return -1;
		}
		memset(strBuf,0,sizeof(strBuf));
		strcpy(strBuf,item->valuestring);
		gateTrue=isVaildIp(strBuf,intGate);
		//mac
		item=cJSON_GetObjectItem(root, KEY_MAC);
			if (item == NULL || !cJSON_IsString(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-mac");
			cJSON_Delete(root);
			return -1;
		}
		memset(strBuf,0,sizeof(strBuf));
		strcpy(strBuf,item->valuestring);
		macTrue=isVaildMac(strBuf,intMac);
		
		//wifi name
			item=cJSON_GetObjectItem(root, KEY_WIFI_NAME);
			if (item == NULL || !cJSON_IsString(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-wifiName");
			cJSON_Delete(root);
			return -1;
		}
		memset(strBuf,0,sizeof(strBuf));
		strcpy(strBuf,item->valuestring);
		if(strlen(strBuf)>32)
		{
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-wifiPwd len>32");
			break;
		}
		strcpy((char*)gDeviceParam.wifiArg.WifiName,strBuf);
		//wifi pwd
		item=cJSON_GetObjectItem(root, KEY_WIFI_PWD);
			if (item == NULL || !cJSON_IsString(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-wifiPwd");
			cJSON_Delete(root);
			return -1;
		}
		memset(strBuf,0,sizeof(strBuf));
		strcpy(strBuf,item->valuestring);
		if(strlen(strBuf)>32)
		{
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-wifiPwd len>32");
			break;
		}
		strcpy((char*)gDeviceParam.wifiArg.WifiPwd,strBuf);
		
		if(ipTrue==true&&gateTrue==true&&maskTrue==true&&macTrue==true)
		{
			for(i=0;i<4;i++)
			{
				gDeviceParam.wifiArg.IpAddress[i]=intIp[i];
				gDeviceParam.wifiArg.MaskAddress[i]=intMask[i];
				gDeviceParam.wifiArg.GateAddress[i]=intGate[i];
			}
			for(i=0;i<6;i++)
			{
				gDeviceParam.wifiArg.MacAddress[i]=intMac[i];
			}
			
			Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
			Save_Parameter();
		}else{
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-wifi arg err");
		}
			break;
		case FUN_WIFI_GET:
			Send_Wifi_Ack(functionFlag,STATUS_OK,"ok");
			break;
		case FUN_USER_INFO:
 
			printf("Recv User Info!\r\n");
		//number
		item=cJSON_GetObjectItem(root,KEY_NUMBER);
			if (item == NULL || !cJSON_IsString(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,KEY_NUMBER);
			cJSON_Delete(root);
			cjsonArr=NULL;
			item=NULL;
			return -1;
		}		
		if(strlen(strBuf)>32)
		{
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-number len>32");
			break;
		}
		strcpy(gUserInfo.number,item->valuestring);

		//name
			item=cJSON_GetObjectItem(root,KEY_NAME);
			if (item == NULL || !cJSON_IsString(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,KEY_NAME);
			cJSON_Delete(root);
			cjsonArr=NULL;
			item=NULL;
			return -1;
		}	
		if(strlen(strBuf)>32)
		{
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-name len>32");
			break;
		}
		strcpy(gUserInfo.name,item->valuestring);
	
		//sex
		item=cJSON_GetObjectItem(root,KEY_SEX);
		if (item == NULL || !cJSON_IsString(item)) {
		Send_Fix_Ack(functionFlag,STATUS_FAIL,KEY_SEX);
		cJSON_Delete(root);
		cjsonArr=NULL;
		item=NULL;
		return -1;
		}
		if(strlen(strBuf)>32)
		{
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-sex len>32");
			break;
		}
		strcpy(gUserInfo.sex,item->valuestring);
		reet=strtol(gUserInfo.sex,&pend,16);
		//age
			item=cJSON_GetObjectItem(root,KEY_AGE);
			if (item == NULL || !cJSON_IsNumber(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,KEY_AGE);
			cJSON_Delete(root);
			cjsonArr=NULL;
			item=NULL;
			return -1;
		}
		gUserInfo.infoEn=1;
		gUserInfo.infoUpdateEn=true;
		
		gUserInfo.age=item->valueint;
	
		
		osDelay(50);
		GBK_Msgprocess(gUserInfo.name);    //����ָ������Ļ��ʾ�·����ݵ�����
		osDelay(50);
		send_visitNumber((uint8_t *)gUserInfo.number);          //����ָ������Ļ��ʾ���
		osDelay(50);
		send_visitSex(reet);       //����ָ������Ļ��ʾ�·����ݵ��Ա�
		osDelay(50);
		send_visitAge(gUserInfo.age);   //  �������䵽��Ļ��ʾ
		osDelay(50);
	  Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
    
		
		break;
		case FUN_DEV_SET:
			//serverIp
			item=cJSON_GetObjectItem(root, KEY_SERVER_IP);
			if (item == NULL || !cJSON_IsString(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-serverIp");
			cJSON_Delete(root);
			return -1;
		}
		memset(strBuf,0,sizeof(strBuf));
		strcpy(strBuf,item->valuestring);
		
		//server port
		item=cJSON_GetObjectItem(root,KEY_SERVER_PORT);
			if (item == NULL || !cJSON_IsNumber(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-port");
			cJSON_Delete(root);
			return -1;
		}
		valueFlag=item->valueint;
	
		if(true==isVaildIp(strBuf,intIp)&&valueFlag>0&&valueFlag<65535)
		{
			for(i=0;i<4;i++)
			{
				gDeviceParam.ServerIpAddress[i]=intIp[i];
			}
			gDeviceParam.mqArg.mqttPort=valueFlag;
		}else{
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"arg-mac fail");
		}
		//heart
		item=cJSON_GetObjectItem(root, KEY_HEAR_RATE);
			if (item == NULL || !cJSON_IsNumber(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-port");
			cJSON_Delete(root);
			return -1;
		}
		valueFlag=item->valueint;
		gDeviceParam.heartRate=valueFlag;
		//product key
		item=cJSON_GetObjectItem(root,KEY_PRODUCT);
		if (item == NULL || !cJSON_IsString(item)) {
		Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-productKey err");
		cJSON_Delete(root);
		return -1;
		}
		memset(strBuf,0,sizeof(strBuf));
		strcpy(strBuf,item->valuestring);
		if(strlen(strBuf)>32)
		{
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"arg-productKey err");
			break;
		}
		strcpy(gDeviceParam.productKey,strBuf);
		
		//newDevid
		item=cJSON_GetObjectItem(root, KEY_NEW_ID);
			if (item == NULL || !cJSON_IsNumber(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-newDevid");
			cJSON_Delete(root);
			return -1;
		}
		valueFlag=item->valueint;
		if(valueFlag<100000||valueFlag>999999999)
		{
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"range:10000~999999999");
			break;
		}
		gDeviceParam.devId=valueFlag;

		//��ά��
	    item=cJSON_GetObjectItem(root, KEY_LEFT_Code);
			if (item == NULL || !cJSON_IsString(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-leftCode err");
			cJSON_Delete(root);
			return -1;}
			memset(strBuf,0,sizeof(strBuf));
		  strcpy(strBuf,item->valuestring);
			SendleftQR(strBuf,0);
		//			printf("buf=%s\r\n",strBuf);
				//MQ_Username
			item=cJSON_GetObjectItem(root, KEY_Mquser);
			if (item == NULL || !cJSON_IsString(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-MQ_Username err");
			cJSON_Delete(root);
			return -1;
			}
			strcpy(gDeviceParam.mqArg.MQ_Username,item->valuestring);
			
			//MQ_Password
			item=cJSON_GetObjectItem(root, KEY_Mqpassword);
			if (item == NULL || !cJSON_IsString(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-MQ_Password err");
			cJSON_Delete(root);
			return -1;
			}
			strcpy(gDeviceParam.mqArg.MQ_Password,item->valuestring);
			
			Save_Parameter();
			Send_Fix_Ack(functionFlag,STATUS_OK,"OK");
		break;
			
		case FUN_DEV_GET:
				Send_BaseArg_Ack(functionFlag,STATUS_OK,"OK");
		break;
		
		case FUN_SAVE_ARG://���浱ǰ��������ΪĬ�Ϲ�������
			Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
			memcpy(gDeviceParam.workArg,gGlobalData.useWorkArg,sizeof(Parameter_Work));
			Save_Parameter();
		break;
		
		case FUN_MESSAGE_BOX:            //�·���ʾ��Ϣ��ʾ��
			item=cJSON_GetObjectItem(root, KEY_MSG);
				if (item == NULL || !cJSON_IsString(item)) {
				Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-message err");
				cJSON_Delete(root);
				return -1;
			}
			memset(strBuf,0,sizeof(strBuf));
			strcpy(strBuf,item->valuestring);
			SendMessageDialog((uint8_t *)strBuf,strlen(strBuf));                    //������ʾ��
			Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
		break;   
      
 		case FUN_PLAY_CONTENT:         //�·����������ļ�
			item=cJSON_GetObjectItem(root, KEY_VALUE);
				if (item == NULL || !cJSON_IsString(item)) {
				Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-voice-content err");
				cJSON_Delete(root);
				return -1;
			}
			memset(strBuf,0,sizeof(strBuf));
			strcpy(strBuf,item->valuestring);
			Send_PlayMusic((uint8_t *)strBuf);
			Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
		break;   
			
		case FUN_PLAY_CTL:            //�·����ſ���
			item=cJSON_GetObjectItem(root, KEY_VALUE);
				if (item == NULL || !cJSON_IsNumber(item)) {
				Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-voice-content err");
				cJSON_Delete(root);
				return -1;
			}
			valueFlag=item->valueint;
      Send_ComMusic(valueFlag);     
			Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
		break;
			
		case FUN_VOLUME_CTL:          //�·���������
			item=cJSON_GetObjectItem(root, KEY_VALUE);
				if (item == NULL || !cJSON_IsNumber(item)) {
				Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-voice-content err");
				cJSON_Delete(root);
				return -1;
			}
			valueFlag=item->valueint;
			Send_ComVolume(valueFlag);
			Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
		break;
			
		case FUN_LIGHT_CTL:              //�ƹ����
			item=cJSON_GetObjectItem(root, KEY_VALUE);
				if (item == NULL || !cJSON_IsNumber(item)) {
				Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-voice-content err");
				cJSON_Delete(root);
				return -1;
			}
			valueFlag=item->valueint;
			if(valueFlag){
					WriteOUT13(GPIO_PIN_RESET);
			}
			else{
					WriteOUT13(GPIO_PIN_SET);    // ����
			}  
			Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
		break;     
			
		case FUN_AUDIO_CTL:              //��Ƶ����
			item=cJSON_GetObjectItem(root, KEY_VALUE);
				if (item == NULL || !cJSON_IsNumber(item)) {
				Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-audio-ctrl err");
				cJSON_Delete(root);
				return -1;
			}
			valueFlag=item->valueint;
			if(valueFlag==1){   //��������Ҳ���Ƕ���
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
			}
			else if(valueFlag==2){    //��������Ҳ��������
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);	
			}  
			Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
		break; 
			
		case FUN_LEVEL_CTL:              //�Ӽ�������
			item=cJSON_GetObjectItem(root, KEY_VALUE);
				if (item == NULL || !cJSON_IsNumber(item)) {
				Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-level-ctrl err");
				cJSON_Delete(root);
				return -1;
			}
			valueFlag=item->valueint;
			if(valueFlag==1){     //�ӵ�
				do_work_ctl(4);
			}
			else if(valueFlag==2){    //����
				do_work_ctl(5);
			}
			osDelay(10);
			cnt_heartbag = 0;												//���������������������			
			gGlobalData.heartbag_flage = 1;    //����������ϴ�һ������
		break; 
			
		case FUN_AUTO_LEVEL_CTL:            //�Զ��Ӽ�������
			item=cJSON_GetObjectItem(root, KEY_VALUE);
				if (item == NULL || !cJSON_IsNumber(item)) {
				Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-auto-level-ctrl err");
				cJSON_Delete(root);
				return -1;
			}	
			if(item->valueint >= 5 && item->valueint <= 60 && gGlobalData.Auto_Level_Ctl == 0){
				gGlobalData.Auto_Level_Ctl = item->valueint;
			}
			else{
				Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-control-repeatedly-or-value err");
				break;
			}
			Send_Fix_Ack(functionFlag,STATUS_OK,"ok");			
		break;
			
			
		default:
		break;
	} 
	 //memset(debug2,0,sizeof(debug2)); //debug2
	 cJSON_Delete(root);
	 root=NULL;
	/*������*/
	//transport_sendPacketBuffer(NET_LINK_ID0,payload,payloadLen);
 	return 0;
}


/*�̣߳�tcp�������˿ڼ��*/
void StartMqttSendTask(void const *arg)
{
	int len;
	unsigned char buf[20]={0,};
	char buf4g_ack_head[128]={0,},bufwifi_ack_head[128]={0,};				//����Ϊ����ģʽ
	uint16_t mqCount=0;//mq����������
	MQTTMessage messageSend;

	//������Ϣ��ʼ��
	messageSend.dup = 0;
	messageSend.qos = QOS0;
	messageSend.retained = 0;
	messageSend.payload =NULL;//gSendNetData;
	messageSend.payloadlen = 0;//strlen(gSendNetData);
	
	while(1)
	{
		osDelay(1); 
		//hyadd
		mqCount++;
		if(gGlobalData.heartbag_flage == 1)
		{
			gGlobalData.heartbag_flage =0;
			Send_heartBag(FUN_JIANCHE_DATA,STATUS_OK,gGlobalData.netKind,gGlobalData.cur_heart_state,gGlobalData.Alltime);	
		}
        		//hyadd ����������
		if(gGlobalData.Send_Heart_Bag==true)
		{			
			messageSend.payloadlen = strlen(gAck);
			messageSend.payload=gAck;	
			if(gGlobalData.rj45Status==true)
			{
				MQTTPublish(gTopicInfo.cmdPost, &messageSend);
			}
			gGlobalData.Send_Heart_Bag=false;
		}
		//wifi���������������
		else if(gGlobalData.Send_Heart_Bag_wifi){
			gGlobalData.Send_Heart_Bag_wifi = false;
			taskENTER_CRITICAL();                                    //��ס
			messageSend.payloadlen = strlen(gAck);
			messageSend.payload=gAck;
      taskEXIT_CRITICAL();
			if(gGlobalData.wifiStatus)        													//wifiӦ��
			{	
				sprintf(bufwifi_ack_head,"AT+MQTTPUBRAW=\"%s\",1,0,%d",gTopicInfo.cmdPost,messageSend.payloadlen);  //wifiӦ��ͷ
				osDelay(100);
				atk_8266_send_cmd((uint8_t *)bufwifi_ack_head,strlen(bufwifi_ack_head));
				osDelay(50);
				__disable_irq();                                    //��ס
				HAL_UART_Transmit_DMA(&huart3,(uint8_t *)messageSend.payload,messageSend.payloadlen);	
				__enable_irq();	
				osDelay(10);
			}
		}
	        
		//�����ݻظ�  ���Ͳɼ����ݵ�
		if(gGlobalData.Send_Data_Task == true)
		{
 			messageSend.payload =gGlobalData.PlusePressDataSend;                                 //gGlobalData.PlusePressDataSend;
			messageSend.payloadlen =gGlobalData.PlusePressDataLen;                                     //gGlobalData.PlusePressDataLen;
			if(gGlobalData.rj45Status == true  &&  gGlobalData.netKind == 1){
				MQTTPublish(gTopicInfo.streamPost, &messageSend);
			}
		  else if(gGlobalData.wifiStatus == true && gGlobalData.netKind == 2){
				sprintf(bufwifi_ack_head,"AT+MQTTPUBRAW=\"%s\",1,0,%d",gTopicInfo.streamPost,messageSend.payloadlen);  //wifiӦ��ͷ   ���˻س�
				osDelay(50);
				atk_8266_send_cmd((uint8_t *)bufwifi_ack_head,strlen(bufwifi_ack_head)); 
				osDelay (50);
				HAL_UART_Transmit_DMA(&huart3,(uint8_t *)messageSend.payload,messageSend.payloadlen);     //���л��������
				osDelay(10);                      
			}
			else if(gGlobalData.yd4gStatus == true  &&  gGlobalData.netKind == 3){
				sprintf(buf4g_ack_head,"AT+QMTPUBEX=0,0,0,0,\"%s\",%d\r\n",gTopicInfo.streamPost,messageSend.payloadlen);//4gӦ��ͷ
				osDelay(10);
				HAL_UART_Transmit_DMA(&huart6,(uint8_t *)buf4g_ack_head,strlen(buf4g_ack_head));
				osDelay(50);
				HAL_UART_Transmit_DMA(&huart6,(uint8_t *)messageSend.payload,messageSend.payloadlen);
				osDelay(10);          				
			}
			gGlobalData.Send_Data_Task=false;
		}
    
		//д���ݹ̶�Ӧ��
		if(gGlobalData.Send_Ack_Task == true)
		{           
 			messageSend.payloadlen = strlen(gAck);
			messageSend.payload=gAck;
			if(gGlobalData.rj45Status == true  &&  gGlobalData.netKind == 1)
			{
				MQTTPublish(gTopicInfo.cmdPost, &messageSend);
				printf("ACK  to MQTT!\r\n");
			}
			else if(gGlobalData.wifiStatus == true  &&  gGlobalData.netKind == 2)        													//wifiӦ��
			{					
				sprintf(bufwifi_ack_head,"AT+MQTTPUBRAW=\"%s\",1,0,%d",gTopicInfo.cmdPost,messageSend.payloadlen);  //wifiӦ��ͷ				
				osDelay(50);
				atk_8266_send_cmd((uint8_t *)bufwifi_ack_head,strlen(bufwifi_ack_head));
				osDelay(50);
				__disable_irq(); 
				HAL_UART_Transmit_DMA(&huart3,(uint8_t *)gAck,strlen(gAck));
				__enable_irq();
				osDelay(10);
			}
			else if(gGlobalData.yd4gStatus == true  &&gGlobalData.netKind == 3)																																		//4gӦ��
			{
				sprintf(buf4g_ack_head,"AT+QMTPUBEX=0,0,0,0,\"%s\",%d\r\n",gTopicInfo.cmdPost,messageSend.payloadlen);//4gӦ��ͷ				
				osDelay(10);
				HAL_UART_Transmit_DMA(&huart6,(uint8_t *)buf4g_ack_head,strlen(buf4g_ack_head));
				osDelay(50);
				HAL_UART_Transmit_DMA(&huart6,(uint8_t *)gAck,strlen(gAck));
				osDelay(10);
			}	
			gGlobalData.Send_Ack_Task=false;
		}
		//��ʪ�Ȳɼ���Ϊ����ʱ ���淽����Ч
 		if(gGlobalData.Send_Ping_Task == true)
		{
			len=MQTTSerialize_pingreq(buf,sizeof(buf));
			if(transport_sendPacketBuffer(MQ_NET_ID,buf,len)<=0)
			{
//			printf("PINGREQ Send errno\r\n");
			}
			gGlobalData.Send_Ping_Task=false;
		}

	}
}
void cmdReplyProcess(char* msg,int msgLen)
{
	netData_process(msg,msgLen);
	return;
}

void boarReplyProcess(char*msg,int msgLen)
{
	return;
}
void mqttMessageProcess(char*msg,int msgLen,char*topic,int topicLen)
{
	//printf("msg=%s\r\n",msg);   //debug
	//strcpy(temp,msg);
	/*����ָ������*/
	if(strncmp(topic,gTopicInfo.cmdReply,topicLen)==0)
	{
		cmdReplyProcess(msg,msgLen);
	}
	/*�����㲥����*/
	if(strncmp(topic,gTopicInfo.boardReply,topicLen)==0)
	{
		boarReplyProcess(msg,msgLen);
	}
}

// �����������������Ļ��ʾ
void GBK_Msgprocess(char*buf)
{	
	int i=0,j=0,c=0,k=0;
	char buffer[18];
	char twobuf[2];
	long ret;
	char *pend;
	uint8_t gbk0[8]={0,};
	uint8_t gbk16[8];
	strcpy(buffer,buf);
	for(c=0;c<9;c++ )	
	{
		for(i=0;i<2;i++)
		{
			twobuf[i]=buffer[j++];	
		}
			ret=strtol(twobuf,&pend,16);
			if(ret==0)
			{
					c=9;
//					gbk16[k]='\0';
				send_visitName(gbk0,8);
				osDelay (10);
				send_visitName(gbk16,k);
//				return gbk16;
			}
			else
			{
			gbk16[k++]=ret;
			memset (twobuf,0,sizeof(twobuf));
			}	
	}
	return;
}

//ʱ�䴦�������Ļ��ʾ
void GBK_Timeprocess(char*buf)
{
  int c=0,i=0,j=2,k=0;
  char buffer[18];
	char buf6[6];
  char twobuf[2];
	long ret;
	char *pend;
	strcpy(buffer,buf);
	for(c=0;c<6;c++)
	{
	   for(i=0;i<2;i++)
		{
	    twobuf[i]=buffer[j++];
    }
		ret=strtol(twobuf,&pend,10);
    buf6[k++]=ret;
		memset (twobuf,0,sizeof(twobuf));

	}
	
	send_rtcTime(buf6[0],buf6[1],buf6[2],buf6[3],buf6[4],buf6[5]);
	return;
}
//������Ϣ��ʾ��
void SendMessageDialog(uint8_t *Msg,uint8_t Msglen)
{
	int c,i,j=0,k=0;
	long ret;
	char *pend;
	char twobuf[2];    
	char Recbuf[200],Msgbuf[200];
	memset(Msgbuf,0,sizeof(Msgbuf));
	strcpy(Recbuf,(const char *)Msg);
    
	for(c=0;c<Msglen/2;c++)
	{
	 for(i=0;i<2;i++)
		{
	    twobuf[i]=Recbuf[j++];
		}
		ret=strtol(twobuf,&pend,16);
		Msgbuf[k++]=ret;
		memset (twobuf,0,sizeof(twobuf));
	}
	Send_Text_Box(1);
	osDelay(50);
	Send_Text_Content((uint8_t *)Msgbuf,strlen(Msgbuf));
	osDelay(50);
	Send_Text_SetButton(0,1);
	osDelay(50);

}
// ����ʱ���������Ļ��ʾ
void Countdown_Treat(uint16_t count)
{
	int h,min,second;
	h = count / 3600;				    //�����3600�͵���Сʱ
	min = (count % 3600) / 60;		//�Ȱѿ��Ի����Сʱ�������޳����ٳ�60�õ�����
	second = (count % 3600) % 60;		//�ѿ��Ի����Сʱ�������ͷ��ӵ������޳��͵õ�����
	send_countDown(h,min,second);
	return;
}

//���·��Ķ�ά�봢�������´��ϵ�ʹ�� Location:0����ŵ�������� 1����ŵ�Ѩλ��ʹ��
void SendleftQR(char *buf,char QR_locatoin){
	int c,i,j=0,k=0;
	char twobuf[2];
	long ret;
	char *pend;
	char Recbuf[200];
	memset(Recbuf,0,sizeof(Recbuf));
	strcpy(Recbuf,buf);
	memset(gDeviceParam.qrbuf,0,sizeof(gDeviceParam.qrbuf));
	for(c=0;c<strlen(Recbuf)/2;c++)
	{
	   for(i=0;i<2;i++)
		{
	    twobuf[i]=Recbuf[j++];
    }
		ret=strtol(twobuf,&pend,16);
		if(QR_locatoin==0){
    gDeviceParam.qrbuf[k++]=ret;
		}
		memset (twobuf,0,sizeof(twobuf));
	}
	return;
}
/*�̣߳��������˿ڼ��    δ�õ�  */
void StartSendheartTask(void const *arg)
{
	while(1)
	{
		osDelay(1);
	}

}



