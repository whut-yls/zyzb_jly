#if !defined USER_TCP_CLIENT_H
#define USER_TCP_CLIENT_H

#include "user_tcp_com.h"
#include "stdbool.h"
#include "cmsis_os.h"

int netData_process(char *payload,int payloadLen);

extern struct netconn*  gNetSock[NET_SOCK_NUM];
extern unsigned char gNetSockStatus[NET_SOCK_NUM];
extern char gSendNetData[RECV_BUF_MAX-1];

int transport_open(unsigned char netId,unsigned char* MQ_Address,unsigned short remotePort);
void transport_release(int netId);

int transport_sendPacketBuffer(unsigned char netId,unsigned char* send,short int sendLen);
int transport_recvPacketBuffer(unsigned char netId,unsigned char* recv,unsigned short *recvLen,int timeout);

int transport_connect(unsigned char netId,unsigned char* MQ_Address,unsigned short remotePort);
void transport_deconnect(int netId);

//for test 
int user_tcp_client_send(char *sendData,unsigned short sendLen,char* payload,unsigned short* payloadLen);

void StartMqttClientTask(void const *arg);
void StartMqttSendTask(void const *arg);

void StartTcpSendTask(void const *arg);	
void StartTcpProcessTask(void const *arg);
void StartSendheartTask(void const *arg);

void SendleftQR(char *buf,char QR_locatoin);
void mqttMessageProcess(char*msg,int msgLen,char*topic,int topicLen);
void GBK_Msgprocess(char*buf);
void GBK_Timeprocess(char*buf);
void SendMessageDialog(uint8_t *Msg,uint8_t Msglen);


bool  TcpSend_TaskSemaphore_Init(void);
bool  COM1_Semaphore_Init(void);

extern SemaphoreHandle_t					TcpSend_Semaphore; 
extern SemaphoreHandle_t					COM1_Semaphore; 
extern bool NetSend_MutexeSemaphore_Init(void);

void Countdown_Treat(uint16_t count); 
//mqtt about 
typedef struct mqtt_topic_def
{
	char cmdReply[100];	//指令订阅
	char cmdPost[100];	//指令应答
	char boardReply[100]; //广播订阅
	char boardPost[100];	//广播应答
	char streamPost[100];	//采样数据发布

}MQTT_TOPIC_TYPE;
extern MQTT_TOPIC_TYPE gTopicInfo;

#endif

