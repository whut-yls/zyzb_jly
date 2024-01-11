#include "EC20_4G.h"
#include "main.h"
//#include "usart.h"
#include "stdio.h"
#include "string.h"
//#include "gpio.h"
//#include "delay.h"
#include "common.h"
#include "user_tcp_client.h"
#include "ethernetif.h"
#include "mqttTask.h"
#include "wifi.h"
#include "lcdTask.h"

char *cjdata_f_4g,*cjdata_p_4g;
char _cjdata_BK_Num_4g=0,_cjdata_i_4g;

uint8_t aTxBuffer0[]="AT+QICLOSE=0\r\n";		//关闭上次的socket连接
uint8_t aTxBuffer1[]="AT\r\n";							//测试指令，若返回OK，则模块可正常通信
uint8_t aTxBuffer2[]="AT+CPIN?\r\n";				//检查SIM卡是否在位
uint8_t aTxBuffer3[]="AT+CSQ\r\n";					//查询信号质量
uint8_t aTxBuffer4[]="AT+CREG?\r\n";				//查询入网状态
uint8_t aTxBuffer5[]="AT+CGATT=1\r\n";			//激活网络
uint8_t aTxBuffer6[]="AT+CGATT?\r\n";				//查询激活状态
uint8_t aTxBuffer7[]="AT+CGPADDR\r\n";			//获取PDP地址
uint8_t aTxBuffer8[]="AT+QMTCFG=\"recv/mode\",0,0,1\r\n";				//配置为接收模式
uint8_t aTxBuffer9[]="AT+QMTOPEN=0,\"124.239.10.57\",9300\r\n";				//	配置阿里云上的设备信息
uint8_t aTxBuffer10[]="AT+QMTCONN=0,\"100006\",\"100001\",\"123456\"\r\n";				//配置为接收模式
uint8_t aTxBuffer12[]="AT+QMTPUBEX=0,0,0,0,\"command/post/JLY/100006\",77\r\n";				//配置为接收模式      {devid:100006function:2}
uint8_t aTxBuffer11[]="AT+QMTSUB=0,1,\"command/reply/JLY/100006\",2\r\n";				//配置为接收模式
uint8_t aTxBuffer14[]="{\"devId\":100006,\"sessionId\":1542957,\"fucntion\":2,\"statusCode\":200,\"msg\":\"ok\"}\r\n";   //发送的JSON数据
uint8_t aTxBuffer13[]="AT+QMTRECV: 0,0\r\n";				//配置为接收模式
uint8_t aTxBuffer15[]="ATE0\r\n";	

void EC200MqttClientTask(void const * argument)	//EC20配置初始化，连接MQTT服务器    void const * argument
{
	unsigned char nums=0;
	char *cjdata;
	char buf_MQ_User_4g[50],bufMqid_4g[50],bufServer_4g[50];
	char buf4g_ack_head[128]={0,};				//配置为接收模式
  memset(bufServer_4g,0,sizeof (bufServer_4g));
	memset(buf_MQ_User_4g,0,sizeof (buf_MQ_User_4g));
	memset(bufMqid_4g,0,sizeof (bufMqid_4g));

		HAL_UART_Receive_DMA(&huart6, RecCom6,COM6_REC_MAX);
	__HAL_UART_CLEAR_IDLEFLAG(&huart6);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

	MQTTMessage messageSend;

	//发布消息初始化
	messageSend.dup = 0;
	messageSend.qos = QOS0;
	messageSend.retained = 0;
	messageSend.payload =NULL;//gSendNetData;
	messageSend.payloadlen = 0;//strlen(gSendNetData);
	HAL_UART_Transmit_DMA(&huart6,aTxBuffer15,sizeof(aTxBuffer15));   //ATE0
	while(1)
	{
		osDelay (20);
		//说明断网了 且权限交给了4G去连接
		if(gGlobalData.isConnect == 0 && gGlobalData.conFlage == 3){
			if(gGlobalData.yd4gStatus == false ){
				gGlobalData.yd4gStatus = true;
				wifi_deinit();                                       //切4g挂掉wifi
				osDelay(2000);
				sprintf(buf_MQ_User_4g,"AT+QMTCONN=0,\"%d\",\"%s\",\"%s\"\r\n",gDeviceParam.devId,gDeviceParam.mqArg.MQ_Username,gDeviceParam.mqArg.MQ_Password);
				sprintf(bufMqid_4g,"AT+QMTSUB=0,1,\"command/reply/JLY/%d\",0\r\n",gDeviceParam.devId);                    
				sprintf(bufServer_4g,"AT+QMTOPEN=0,\"%d.%d.%d.%d\",%d\r\n",gDeviceParam.ServerIpAddress[0]
																																 ,gDeviceParam.ServerIpAddress[1]
																																 ,gDeviceParam.ServerIpAddress[2]
																																 ,gDeviceParam.ServerIpAddress[3]
																																 ,gDeviceParam.mqArg.mqttPort); 
	
				send_NetSync(4);    //网络灯 1601  
				osDelay(100);
				send_duan_wang(1);	//断网提示 1900 
				
				gGlobalData.netKind = 3;			//4G				
				osDelay(100);
					 
				HAL_UART_Transmit_DMA(&huart6,aTxBuffer0,sizeof(aTxBuffer0));    //AT+QICLOSE=0
				osDelay(200);

				HAL_UART_Transmit_DMA(&huart6,aTxBuffer1,sizeof(aTxBuffer1));   //AT
				osDelay(200);
				
				HAL_UART_Transmit_DMA(&huart6,aTxBuffer2,sizeof(aTxBuffer2));   //AT+CPIN  检查SIM卡是否在位
				osDelay(200);
				
				HAL_UART_Transmit_DMA(&huart6,aTxBuffer3,sizeof(aTxBuffer3));   //AT+CSQ   查询信号质量
				osDelay(200);
				
				HAL_UART_Transmit_DMA(&huart6,aTxBuffer4,sizeof(aTxBuffer4));		//AT+CEREG  查询入网状态
				osDelay(200);

				HAL_UART_Transmit_DMA(&huart6,aTxBuffer5,sizeof(aTxBuffer5));		//AT+CGATT=1  激活网络
				osDelay(500);

				HAL_UART_Transmit_DMA(&huart6,aTxBuffer6,sizeof(aTxBuffer6));		//AT+CGATT?   查询激活状态
				osDelay(200); 

				HAL_UART_Transmit_DMA(&huart6,aTxBuffer7,sizeof(aTxBuffer7));		//AT+CGPADDR   获取PDP地址
				osDelay(200);
				
				HAL_UART_Transmit_DMA(&huart6,aTxBuffer8,sizeof(aTxBuffer8));		//AT+QMTCFG=\"recv/mode\",0,0,1  配置为接收模式
				osDelay(200);
				
				HAL_UART_Transmit_DMA(&huart6,(uint8_t *)bufServer_4g,sizeof(bufServer_4g));  //AT+QMTOPEN=0    124.239.10.57,9300
				osDelay(1000);
				
				HAL_UART_Transmit_DMA(&huart6,(uint8_t *)buf_MQ_User_4g,sizeof(buf_MQ_User_4g));				//QMTCONN
				osDelay(1000);
		
				HAL_UART_Transmit_DMA(&huart6,(uint8_t *)bufMqid_4g,sizeof(bufMqid_4g));					//QMTSUB
				osDelay(100);
				HAL_UART_Receive_DMA(&huart6, RecCom6,COM6_REC_MAX);
				HAL_UART_Transmit_DMA(&huart6,aTxBuffer13,sizeof(aTxBuffer13));
				osDelay(100);
				printf("4g-working\r\n");
				cnt_heartbag = 0;												//发送心跳清空心跳计数器
				gGlobalData.heartbag_flage=1;  //发一次心跳
				gGlobalData.isConnect=1;//设置为连接正常
			}
		}
			
		//4g模块自己的心跳维持  保证4g模块一直工作
		nums++;
		if(nums > 200 && gGlobalData.yd4gStatus == true && cjdata == NULL){      //cjdata==NULL在处理解析的时候就不会发自身心跳影响解析
			nums = 0;
			HAL_UART_Transmit_DMA(&huart6,aTxBuffer13,sizeof(aTxBuffer13));				//"AT+QMTRECV: 0,0\r\n" 配置为接收模式    心跳包
			osDelay(200);

		}	
			//网络正常 且4g正常 且当前连接为4g
		if(gGlobalData.isConnect == 1 && gGlobalData.yd4gStatus == true && gGlobalData.netKind == 3){			
				//4g处理向服务器发送心跳
			if(gGlobalData.Send_Heart_Bag_4G == true)
			{
				gGlobalData.Send_Heart_Bag_4G = false;
				messageSend.payloadlen = strlen(gAck);
				messageSend.payload = gAck;
				if(gGlobalData.yd4gStatus == true)																																		//4g应答
				{
					sprintf(buf4g_ack_head,"AT+QMTPUBEX=0,0,0,0,\"%s\",%d\r\n",gTopicInfo.cmdPost,messageSend.payloadlen);//4g应答头
					osDelay(10);
					HAL_UART_Transmit_DMA(&huart6,(uint8_t *)buf4g_ack_head,strlen(buf4g_ack_head));
					osDelay (50);
					HAL_UART_Transmit_DMA(&huart6,(uint8_t *)messageSend.payload,messageSend.payloadlen);
					osDelay(10);	
				}	
			}			
			if(cjdata = strstr((const char*)RecCom6,"{"))
			{
				osDelay(10);
				if(strstr((const char*)cjdata,"HeartBag")){
					printf("HeartBag_4g\r\n");
					memset(RecCom6,0,sizeof(RecCom6));				
				}
				else if(strstr((const char*)cjdata,"ACK")){
					printf("ACK_4g\r\n");
					memset(RecCom6,0,sizeof(RecCom6));
				}
				else{  //心跳包不进解析
						cjdata_f_4g = cjdata;
						cjdata_p_4g = cjdata;
						while(1)
						{
							if(cjdata_f_4g = strstr((const char*)(cjdata_f_4g+1),"{"))
								_cjdata_BK_Num_4g++;         																				//反括号数,下面for循环里多检查一个反括号,因为进来判断已经有一个了
							else break;
						}
						for(_cjdata_i_4g = 0;_cjdata_i_4g <= _cjdata_BK_Num_4g;_cjdata_i_4g++)
						{
							if(cjdata_p_4g = strstr((const char*)(cjdata_p_4g+1),"}")){
								if(_cjdata_i_4g == _cjdata_BK_Num_4g){
									_cjdata_BK_Num_4g=0;
									printf("解析成功_4g\r\n");
									gGlobalData.heart_count = 0;             //进来清空倒计时
									gGlobalData.yd4gStatus = true;
									netData_process(cjdata,sizeof(cjdata));
									memset(RecCom6,0,sizeof(RecCom6));
									if(gGlobalData.ResetStatus == true)
									{
										gGlobalData.ResetStatus = false;
										do_work_ctl(3);
										osDelay (100); 
										send_QRInfo(gDeviceParam.qrbuf,strlen((const char*)gDeviceParam.qrbuf));   //发送二维码到屏幕左侧上显示
									}		
									send_NetSync(5);    //网络灯 1601  
									osDelay (100);	
									send_duan_wang(0);	//断网提示 1900									
								}					
							}
							else break;
						}
						_cjdata_BK_Num_4g = 0;
						_cjdata_i_4g = 0;
					}
				}
				HAL_UART_Receive_DMA(&huart6, RecCom6,COM6_REC_MAX);				
			}
	 }//while(1)
}

 






