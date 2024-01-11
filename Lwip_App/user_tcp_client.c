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

/**定义的信号量用来检测发送事件**/
SemaphoreHandle_t					TcpSend_Semaphore; 
SemaphoreHandle_t					COM1_Semaphore; 	//信号处理板通信uart1
SemaphoreHandle_t	NetSendMutexeSemaphore;


/*mqtt fun about*/
/*主题初始化
type:产品标识 id:设备id
*/
void Topic_Init(MQTT_TOPIC_TYPE* topic,char* type,char*id)
{
	memset(topic,0,sizeof(MQTT_TOPIC_TYPE));
	//指令订阅
	strcat(topic->cmdReply,"command/reply/");
	strcat(topic->cmdReply,type);
	strcat(topic->cmdReply,"/");
	strcat(topic->cmdReply,id);    //这之前是 strcat(topic->cmdReply,id);我们订阅的主题最后是#，id的值又用于client id上。我们在这儿就直接改为"#"。

	//指令发布
	strcat(topic->cmdPost,"command/post/");
	strcat(topic->cmdPost,type);
	strcat(topic->cmdPost,"/");
	strcat(topic->cmdPost,id);

	//广播订阅
	strcat(topic->boardReply,"broadcast/reply/");
	strcat(topic->boardReply,type);
	//广播发布
	strcat(topic->boardPost,"broatcast/post/");
	strcat(topic->boardPost,type);
	//采集发布
	strcat(topic->streamPost,"stream/post/");  //stream/post/
	strcat(topic->streamPost,type);
	strcat(topic->streamPost,"/");
	strcat(topic->streamPost,id);
}

void mqtt_topic_sub(MQTT_TOPIC_TYPE*topic,MQTTMessage*messageRecv,MQTTMessage*messageSend)
{

	//订阅主题

	MQTTSubscribe(topic->cmdReply,messageRecv);
	
//	MQTTSubscribe(topic->boardReply,messageRecv);

	//发布主题
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
功能：打开一个tcp网络套接字 并且连接
返回：1成功返回 , 失败返回-1(false)
**/
int transport_open(unsigned char netId,unsigned char* MQ_Address,unsigned short remotePort)
{	
	ip_addr_t ipaddr;
	err_t connect_err;
	/**检测id合法性**/
	if(netId>NET_SOCK_NUM-1)
	{
		return -1;
	}
	
	/**如果已经打开则释放**/
	if(NET_LINK_OPEN==gNetSockStatus[netId])
	{
		gNetSockStatus[netId]=NET_LINK_CLOSE;
		netconn_close(gNetSock[netId]);
		netconn_delete(gNetSock[netId]);
	}
	gNetSockStatus[netId]=NET_LINK_CLOSE;
	
	/**数据初始化  **/
	IP4_ADDR(&ipaddr, MQ_Address[0], MQ_Address[1],MQ_Address[2],MQ_Address[3]);
	/* Create a new connection identifier. */
	gNetSock[netId]= netconn_new(NETCONN_TCP);
	if (gNetSock[netId]!=NULL)
	{
		//err = netconn_bind(conn, NULL, 5001);	//不需要绑定
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
功能：从一个打开的套接字句柄上发送数据NetSendMutexeSemaphore 不需要信号量，因为只在一个线程中使用发送
返回：成功返回实际写入的字节数 失败返回-1
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
			/*发送一帧数据**/			
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
			/*发送一帧数据**/			
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
功能：从一个已经打开的网络套件字上接收数据,若超过最大接收缓存则其余数据丢弃
成功：填充mq_Send ,并将接收长度赋值给l ,成功返回1 超时返回0 识别返回-1
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
	
	gNetSock[netId]->recv_timeout=timeout;	//如果要使能接收超时 则需要修改 opt.h  LWIP_SO_RCVTIMEO			1
		/*超时等待接收一帧数recv_timeout据**/
	recv_err=netconn_recv(gNetSock[netId], &buf);
	if (ERR_OK==recv_err)
	{
		do
		{
			netbuf_data(buf, &data, &readLen);
			/**拷贝数据到recv,更新recvLen**/
			memcpy(recv,data,readLen);
			*recvLen+=readLen;
			if(*recvLen>RECV_BUF_MAX)
			{
				break;
			}
			/**回写测试*/
			//netconn_write(gNetSock[netId], data,readLen, NETCONN_COPY);
		}while (netbuf_next(buf) >= 0);
		netbuf_delete(buf);	
		return 1;
	}else if(ERR_TIMEOUT==recv_err)	//设置了超时接收
	{
		*recvLen=0;
		return 0;
	}else{		//需要重新初始化并连接
		*recvLen=0;
		return -1;
	}
	
}

/**
功能：释放一个已经打开的套接字
**/
void transport_release(int netId)
{
	if(netId>NET_SOCK_NUM-1)
	{
		return;
	}
	gNetSockStatus[netId]=NET_LINK_CLOSE;
	netconn_close(gNetSock[netId]); //这个函数会阻塞等待信号，如果对方没有回应可能会等待很长时间
	netconn_delete(gNetSock[netId]);
}

#if 0
/**
功能：将一个已经打开的tcp网络套接字 连接到远端地址 接字资源
返回：成功返回 连接状态NET_LINK_LINKED/NET_LINK_AGAIN, 失败返回-1(false)
注： 测试发现 连接不成功时 系统自动释放了套字资源 ，目前无法单独关闭和连接 而不释放
**/
int transport_connect(unsigned char netId,unsigned char* MQ_Address,unsigned short remotePort)
{	
	return -1;
#if 0
	ip_addr_t ipaddr;
//	err_t err,recv_err;
	err_t connect_err;
	/**检测id合法性**/
	if(netId>NET_SOCK_NUM-1)
	{
		return -1;
	}

	/**数据初始化  **/
	IP4_ADDR(&ipaddr, MQ_Address[0], MQ_Address[1],MQ_Address[2],MQ_Address[3]);
//	IP4_ADDR(&ipaddr, 192, 168,2,209);
	/* Create a new connection identifier. */
	if (gNetSock[netId]!=NULL)
	{
		//err = netconn_bind(conn, NULL, 5001);	//不需要绑定
		connect_err = netconn_connect(gNetSock[netId], &ipaddr, remotePort);
//		connect_err = netconn_connect(gNetSock[netId], &ipaddr, 5014);
		/* Process the new connection. */
		if (connect_err == ERR_OK)
		{
			gNetSockStatus[netId]=NET_LINK_LINKED;
			return NET_LINK_LINKED;
		}else if(connect_err == ERR_ABRT||connect_err == ERR_RST){
			return NET_LINK_AGAIN;
		}else{	//其它情况 重新初始化
			return -1;
		}
			
	}else
	{
		return -1;
	}
	#endif
}


/**
功能：断开一个已经打开的连接,只关闭不释放
注：未测试
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

/**线程：Tcp 接收指令客户端
注：常规来说这里应该使用服务器，但是考虑到以后在公网中使用 所以这里作为客户端**/
void StartMqttClientTask(void const *arg)		//记得增加栈空间尺寸 否则可能导致溢出
{
	int rc;
	unsigned char buf[60]={0,};
	
	MQTTMessage messageSend;
	MQTTMessage messageRecv;	
	MQTTString	messageTopic;
	//主题初始化
	sprintf((char*)buf,"%d",gDeviceParam.devId);
	Topic_Init(&gTopicInfo,(char*)gDeviceParam.productKey,(char*)buf);
	memset(buf,0,sizeof(buf));
	//链接参数初始化
	MQTT_ConnectData_Init(&MQTTPacket_ConnectData_Initializer);
	//
	if(MQTTConnect(&MQTTPacket_ConnectData_Initializer)<0)
	{
		MQTTDisConnect(&MQTTPacket_ConnectData_Initializer);
		gEthRecvStatus=false;	
	}else{
		gEthRecvStatus=true;
	}

	//发布消息初始化
	messageSend.dup = 0;//消息结束重复
	messageSend.qos = QOS0;//消息结束QoS
	messageSend.retained = 0;//保留
	messageSend.payload =NULL;//gSendNetData;
	messageSend.payloadlen = 0;//strlen(gSendNetData);

	//订阅消息初始化
	messageRecv.dup = 0;
	messageRecv.qos = QOS2;
	messageRecv.retained = 0;
	messageRecv.payload =NULL;//gRecvNetData;
		
	//消息订阅
	if(gEthRecvStatus==true)
	{
		mqtt_topic_sub(&gTopicInfo,&messageRecv,&messageSend);
		if(gGlobalData.ResetStatus==true)
		{
			gGlobalData.ResetStatus=false;
			do_work_ctl(3);
			send_QRInfo(gDeviceParam.qrbuf,strlen((const char *)gDeviceParam.qrbuf));   //发送二维码到屏幕左侧上显示
		}
		cnt_heartbag = 0;											  	//发送心跳清空心跳计数器		
		gGlobalData.heartbag_flage=1;			         //连接上服务器立马发送心跳并接收解析
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
				wifi_deinit();   																				 //强制断开wifi		 	//wifi下线
				HAL_UART_Transmit_DMA(&huart6,"AT+QMTCLOSE=0\r\n",sizeof("AT+QMTCLOSE=0\r\n"));				  //4g下线
				gGlobalData.yd4gStatus=false;								
				gGlobalData.isConnect=1;																																//网络正常
				gGlobalData.conFlage=1;
				gGlobalData.netKind = 1;																																//网线
				mqtt_topic_sub(&gTopicInfo,&messageRecv,&messageSend);																																	//去掉断网提示 1900  //**//
				if(gGlobalData.ResetStatus==true)
				{
					gGlobalData.ResetStatus=false;
					do_work_ctl(3);
					send_QRInfo(gDeviceParam.qrbuf,strlen((const char *)gDeviceParam.qrbuf));   //发送二维码到屏幕左侧上显示
				}
				send_NetSync(1);  																																		  //网络灯 1601  //**//
				osDelay (100);
				send_duan_wang(0);
				cnt_heartbag = 0;																																			 //发送心跳清空心跳计数器
				gGlobalData.heartbag_flage=1;			      																						   //连接上服务器立马发送心跳并接收解析				
			}
		}				
		//接收消息测试		
		rc=MQTTSubscribe_RecvMessage(&messageRecv,&messageTopic);
		if(rc==MQ_SUCCESS)	//MQ_SUCCSS 
		{
//			printf("Subscribe_RecvMessage Success\n");
			//消息处理
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
/*数据处理*/
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
	/*接收解析 到有会话id和功能就回复*/
	if(Act_music==false){  //上网激活一下音乐
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
	#if 2//解析协议头
	cnt_heartbag = 0;                               //通信上就不立马发送心跳
	/*取设备id*/
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

	/*取功能字段*/
	item=cJSON_GetObjectItem(root, KEY_FUN);
	if (item == NULL || !cJSON_IsNumber(item)) {
		item=NULL;
		cJSON_Delete(root);
		Send_Fix_Ack(0,STATUS_FAIL,"key-fun fail");
		return -1;
	}
	functionFlag=item->valueint;   
	
	
	/*取会话id字段*/
	item=cJSON_GetObjectItem(root, KEY_SESSION);
	if (item == NULL || !cJSON_IsNumber(item)) {
		item=NULL;
		Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-session fail");
		cJSON_Delete(root);
		return -1;
	}

//	strcpy((char*)gGlobalData.sessionId,item->valuestring);
	gGlobalData.sessionId=item->valueint; 
	
		//取时间

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
	
	/*根据功能解析其它参数*/
	switch(functionFlag)
	{
		//上传间隔
		case FUN_COLLECT_ARG:      //  1 设置采集方案
		#if 1//下发数据采集方案
		if(gGlobalData.curWorkState != WORK_START)              //没点击启动的时候采集方案接收到才会上传等待状态，已经点了启动的时候就不会把上传服务器的状态给到等待
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
		
		//取采样模式 1：整体采样  2：局部采样
		item=cJSON_GetObjectItem(root, KEY_TYPE);
		if (item == NULL || !cJSON_IsNumber(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-type object err");
			cJSON_Delete(root);
			cjsonArr=NULL;
			item=NULL;
			return -1;
		}

		gGlobalData.curWorkMode=item->valueint;
		
		//取第一级数组对象
		item=cJSON_GetObjectItem(root,  KEY_WORK);
		if (item == NULL || !cJSON_IsObject(item)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL, "key-work object format err");
			cJSON_Delete(root);
			cjsonArr=NULL;
			item=NULL;
			return -1;
		}
		
			//取work里面的数据对象  interval
		itemSub=cJSON_GetObjectItem(item, KEY_DATA_UPTIME);
		if (item == NULL || !cJSON_IsNumber(itemSub)) {
			Send_Fix_Ack(functionFlag,STATUS_FAIL,"key_interval err");
			cJSON_Delete(root);
			cjsonArr=NULL;
			item=NULL;
			return -1;
		}

		gGlobalData.useWorkArg[1].upTime=itemSub->valueint;
			
			//totalTime 采集总时长倒计时
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
			 //gGlobalData.freqUnitTime=itemSub->valueint;  //单次数据之间的采样间隔时间 ms
		gGlobalData.useWorkArg[0].rateN=itemSub->valueint;	
		
		/**成员内的inputs数组**/
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
			
		//inputs数组尺寸
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
		//inputs成员
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
			
		/**成员内的outs数组成员**/
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
		
			//outs数组尺寸
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
		//outs成员
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
		//////////////2023.02.03新增 WWJZ
	//	if (gGlobalData.curWorkMode==WORK_MODE_WT){
			
		//gGlobalData.curWorkState=WORK_START;//将运行状态开始  kardos 2023.02.03
		send_LcdWorkStatus(2);//kardos 2023.02.03 修改设备工作状态为：等待经络检测
		osDelay(50);
		Countdown_Treat(gGlobalData.Alltime);
		osDelay(50);
		gGlobalData.curWorkMode = WORK_MODE_ZT;//2023.1.30增加
	//	}
//		gGlobalData.Send_Client_Over= true;	  //  应答
		break;
		#endif
		
		
		
		#if 4 //下发治疗方案
		//下发治疗治疗方案
		case FUN_COLLECT_TREAT:   
			if(gGlobalData.Alltime!=0){    //by yls 2023/6/13 防止倒计时重复接收
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
			
			//取第一级数组对象
			cjsonArr=cJSON_GetObjectItem(root,  KEY_WORK);
			if (cjsonArr == NULL || !cJSON_IsArray(cjsonArr)) {
				Send_Fix_Ack(functionFlag,STATUS_FAIL, "key-work array format err");
				cJSON_Delete(root);
				cjsonArr=NULL;
				item=NULL;
				return -1;
			}
			//获取work数组尺寸
			gGlobalData.useWorkArg[0].chanelNum=cJSON_GetArraySize(cjsonArr);
			
			for(j=0;j<gGlobalData.useWorkArg[0].chanelNum;j++)
			{
				//获取work第一个对象成员
				item=cJSON_GetArrayItem(cjsonArr,j);
				if (item == NULL || !cJSON_IsObject(item)) {
					Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-work obj err");
					cJSON_Delete(root);
					cjsonArr=NULL;
					itemSub=NULL;
					item=NULL;
					return -1;
				}
				/**成员内的普通数据**/
			
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
				if(itemSub->valueint<=10000) gGlobalData.useWorkArg[j].freqTreat=itemSub->valueint;     //2023/7/2 现在只接受最大2000hz的频率  by yls   现在可以达到1w hz  2023/7/22
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
			
			Level= gGlobalData.useWorkArg[0].level;//初始默认档位赋值 
			set_sampleMode(MODE_ZL);//再切一次设置治疗模式
			gGlobalData.curWorkMode = WORK_MODE_ZL;//2023.1.30增加
			gGlobalData.curWorkState=WORK_START;//将运行状态开始  kardos 2023.02.03
			gGlobalData.cur_heart_state=WORKING;
			Countdown_Treat(gGlobalData.Alltime);
	//		gGlobalData.Send_Client_Over= true;	   //采集结束后下发的治疗方案    应答
			printf("Recv ZL Preject!\r\n");
			break;
		#endif
		
		
		
		//采集控制
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
			}else if(tDataStartFlag2 == WORK_STOP){              //复位自己发ack
				
				do_work_ctl(tDataStartFlag2);
			}
			else {
				Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
				do_work_ctl(tDataStartFlag2);
			}
				
				
		break;
//		case FUN_DEV_OFF:	//开关机控制
		case FUN_DEV_RESET:	//设备重启
			Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
			set_sampleMode(MODE_CLOSE);
			HAL_PCA9554_outputAll(0);
			osDelay(100);
			NVIC_SystemReset();
		break;
		case  FUN_DEV_LOCK:	//设备锁
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
			
		case  FUN_SYS_CHECK://系统设备检测
			Send_SysCheck_Ack(functionFlag,STATUS_OK,"ok");
		break;
		case FUN_JIANCHE_DATA:	//心跳回复
				//Send_JianChe_Ack(functionFlag,STATUS_OK,"ok");
			gGlobalData.isConnect=1;//网络状态ok
			gGlobalData.heart_count = 0; //心跳计数清零

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
		GBK_Msgprocess(gUserInfo.name);    //发送指令至屏幕显示下发数据的名字
		osDelay(50);
		send_visitNumber((uint8_t *)gUserInfo.number);          //发送指令至屏幕显示编号
		osDelay(50);
		send_visitSex(reet);       //发送指令至屏幕显示下发数据的性别
		osDelay(50);
		send_visitAge(gUserInfo.age);   //  发送年龄到屏幕显示
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

		//二维码
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
		
		case FUN_SAVE_ARG://保存当前工作参数为默认工作参数
			Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
			memcpy(gDeviceParam.workArg,gGlobalData.useWorkArg,sizeof(Parameter_Work));
			Save_Parameter();
		break;
		
		case FUN_MESSAGE_BOX:            //下发显示消息提示框
			item=cJSON_GetObjectItem(root, KEY_MSG);
				if (item == NULL || !cJSON_IsString(item)) {
				Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-message err");
				cJSON_Delete(root);
				return -1;
			}
			memset(strBuf,0,sizeof(strBuf));
			strcpy(strBuf,item->valuestring);
			SendMessageDialog((uint8_t *)strBuf,strlen(strBuf));                    //处理提示框
			Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
		break;   
      
 		case FUN_PLAY_CONTENT:         //下发播放声音文件
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
			
		case FUN_PLAY_CTL:            //下发播放控制
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
			
		case FUN_VOLUME_CTL:          //下发音量控制
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
			
		case FUN_LIGHT_CTL:              //灯光控制
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
					WriteOUT13(GPIO_PIN_SET);    // 亮灯
			}  
			Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
		break;     
			
		case FUN_AUDIO_CTL:              //音频控制
			item=cJSON_GetObjectItem(root, KEY_VALUE);
				if (item == NULL || !cJSON_IsNumber(item)) {
				Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-audio-ctrl err");
				cJSON_Delete(root);
				return -1;
			}
			valueFlag=item->valueint;
			if(valueFlag==1){   //内置音响也就是耳机
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
			}
			else if(valueFlag==2){    //外置音响也就是喇叭
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);	
			}  
			Send_Fix_Ack(functionFlag,STATUS_OK,"ok");
		break; 
			
		case FUN_LEVEL_CTL:              //加减档控制
			item=cJSON_GetObjectItem(root, KEY_VALUE);
				if (item == NULL || !cJSON_IsNumber(item)) {
				Send_Fix_Ack(functionFlag,STATUS_FAIL,"key-level-ctrl err");
				cJSON_Delete(root);
				return -1;
			}
			valueFlag=item->valueint;
			if(valueFlag==1){     //加档
				do_work_ctl(4);
			}
			else if(valueFlag==2){    //减档
				do_work_ctl(5);
			}
			osDelay(10);
			cnt_heartbag = 0;												//发送心跳清空心跳计数器			
			gGlobalData.heartbag_flage = 1;    //设置完参数上传一次心跳
		break; 
			
		case FUN_AUTO_LEVEL_CTL:            //自动加减档控制
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
	/*任务处理*/
	//transport_sendPacketBuffer(NET_LINK_ID0,payload,payloadLen);
 	return 0;
}


/*线程：tcp数据流端口检测*/
void StartMqttSendTask(void const *arg)
{
	int len;
	unsigned char buf[20]={0,};
	char buf4g_ack_head[128]={0,},bufwifi_ack_head[128]={0,};				//配置为接收模式
	uint16_t mqCount=0;//mq心跳计数器
	MQTTMessage messageSend;

	//发布消息初始化
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
        		//hyadd 发送心跳包
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
		//wifi向服务器发送心跳
		else if(gGlobalData.Send_Heart_Bag_wifi){
			gGlobalData.Send_Heart_Bag_wifi = false;
			taskENTER_CRITICAL();                                    //锁住
			messageSend.payloadlen = strlen(gAck);
			messageSend.payload=gAck;
      taskEXIT_CRITICAL();
			if(gGlobalData.wifiStatus)        													//wifi应答
			{	
				sprintf(bufwifi_ack_head,"AT+MQTTPUBRAW=\"%s\",1,0,%d",gTopicInfo.cmdPost,messageSend.payloadlen);  //wifi应答头
				osDelay(100);
				atk_8266_send_cmd((uint8_t *)bufwifi_ack_head,strlen(bufwifi_ack_head));
				osDelay(50);
				__disable_irq();                                    //锁住
				HAL_UART_Transmit_DMA(&huart3,(uint8_t *)messageSend.payload,messageSend.payloadlen);	
				__enable_irq();	
				osDelay(10);
			}
		}
	        
		//读数据回复  发送采集数据的
		if(gGlobalData.Send_Data_Task == true)
		{
 			messageSend.payload =gGlobalData.PlusePressDataSend;                                 //gGlobalData.PlusePressDataSend;
			messageSend.payloadlen =gGlobalData.PlusePressDataLen;                                     //gGlobalData.PlusePressDataLen;
			if(gGlobalData.rj45Status == true  &&  gGlobalData.netKind == 1){
				MQTTPublish(gTopicInfo.streamPost, &messageSend);
			}
		  else if(gGlobalData.wifiStatus == true && gGlobalData.netKind == 2){
				sprintf(bufwifi_ack_head,"AT+MQTTPUBRAW=\"%s\",1,0,%d",gTopicInfo.streamPost,messageSend.payloadlen);  //wifi应答头   加了回车
				osDelay(50);
				atk_8266_send_cmd((uint8_t *)bufwifi_ack_head,strlen(bufwifi_ack_head)); 
				osDelay (50);
				HAL_UART_Transmit_DMA(&huart3,(uint8_t *)messageSend.payload,messageSend.payloadlen);     //不行还是用这句
				osDelay(10);                      
			}
			else if(gGlobalData.yd4gStatus == true  &&  gGlobalData.netKind == 3){
				sprintf(buf4g_ack_head,"AT+QMTPUBEX=0,0,0,0,\"%s\",%d\r\n",gTopicInfo.streamPost,messageSend.payloadlen);//4g应答头
				osDelay(10);
				HAL_UART_Transmit_DMA(&huart6,(uint8_t *)buf4g_ack_head,strlen(buf4g_ack_head));
				osDelay(50);
				HAL_UART_Transmit_DMA(&huart6,(uint8_t *)messageSend.payload,messageSend.payloadlen);
				osDelay(10);          				
			}
			gGlobalData.Send_Data_Task=false;
		}
    
		//写数据固定应答
		if(gGlobalData.Send_Ack_Task == true)
		{           
 			messageSend.payloadlen = strlen(gAck);
			messageSend.payload=gAck;
			if(gGlobalData.rj45Status == true  &&  gGlobalData.netKind == 1)
			{
				MQTTPublish(gTopicInfo.cmdPost, &messageSend);
				printf("ACK  to MQTT!\r\n");
			}
			else if(gGlobalData.wifiStatus == true  &&  gGlobalData.netKind == 2)        													//wifi应答
			{					
				sprintf(bufwifi_ack_head,"AT+MQTTPUBRAW=\"%s\",1,0,%d",gTopicInfo.cmdPost,messageSend.payloadlen);  //wifi应答头				
				osDelay(50);
				atk_8266_send_cmd((uint8_t *)bufwifi_ack_head,strlen(bufwifi_ack_head));
				osDelay(50);
				__disable_irq(); 
				HAL_UART_Transmit_DMA(&huart3,(uint8_t *)gAck,strlen(gAck));
				__enable_irq();
				osDelay(10);
			}
			else if(gGlobalData.yd4gStatus == true  &&gGlobalData.netKind == 3)																																		//4g应答
			{
				sprintf(buf4g_ack_head,"AT+QMTPUBEX=0,0,0,0,\"%s\",%d\r\n",gTopicInfo.cmdPost,messageSend.payloadlen);//4g应答头				
				osDelay(10);
				HAL_UART_Transmit_DMA(&huart6,(uint8_t *)buf4g_ack_head,strlen(buf4g_ack_head));
				osDelay(50);
				HAL_UART_Transmit_DMA(&huart6,(uint8_t *)gAck,strlen(gAck));
				osDelay(10);
			}	
			gGlobalData.Send_Ack_Task=false;
		}
		//温湿度采集认为开启时 下面方法有效
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
	/*解析指令主题*/
	if(strncmp(topic,gTopicInfo.cmdReply,topicLen)==0)
	{
		cmdReplyProcess(msg,msgLen);
	}
	/*解析广播主题*/
	if(strncmp(topic,gTopicInfo.boardReply,topicLen)==0)
	{
		boarReplyProcess(msg,msgLen);
	}
}

// 姓名解析处理后在屏幕显示
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

//时间处理后在屏幕显示
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
//处理消息提示框
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
// 倒计时处理后在屏幕显示
void Countdown_Treat(uint16_t count)
{
	int h,min,second;
	h = count / 3600;				    //秒除以3600就等于小时
	min = (count % 3600) / 60;		//先把可以换算成小时的秒数剔除，再除60得到分钟
	second = (count % 3600) % 60;		//把可以换算成小时的秒数和分钟的秒数剔除就得到秒数
	send_countDown(h,min,second);
	return;
}

//把下发的二维码储存起来下次上电使用 Location:0代表放到生物电里 1代表放到穴位疼痛里
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
/*线程：心跳包端口检测    未用到  */
void StartSendheartTask(void const *arg)
{
	while(1)
	{
		osDelay(1);
	}

}



