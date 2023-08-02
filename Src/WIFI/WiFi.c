#include "wifi.h"
#include "user_tcp_client.h"
#include "ethernetif.h"
#include "parameter.h"
#include "lcdTask.h"

//ATK-ESP8266 WIFI STA测试
//用于测试TCP/UDP连接
//返回值:0,正常
//    其他,错误代码
#include "mqttTask.h"

char rec_cmd[255]={0,};     //接收发送指令缓存
char *cjdata,*cjdata_f,*cjdata_p,_wifi_Apcode,_wifi_busy_Num=0;
char _cjdata_BK_Num=0,_cjdata_i;
bool _wifi_AtMqtt_conn=false,_wifi_AtMqtt_sub=false;

void WifiMqttClientTask(void const * argument)        //void StartDefaultTask(void const * argument)
{

	gGlobalData.isSendMq=false;
//	strcpy((char*)gDeviceParam.wifiArg.WifiName,"ZKM");			//wifi名称
//	strcpy((char*)gDeviceParam.wifiArg.WifiPwd,"123456ming");
//	strcpy((char*)gDeviceParam.wifiArg.WifiName,"iphonexr");			//wifi名称
//	strcpy((char*)gDeviceParam.wifiArg.WifiPwd,"12345678");
	//bool gGlobal_WifiChange=true;

	uint8_t buf_MQ_User[50],buf_MQ_pw[50],bufsub[50],bufssid[50],bufServer[50],bufport[50],bufway[50],bufMqid[50],bufCon[50];

	while(1)
	{

		osDelay(10);		
		if(gGlobalData.Wifi_set)																																									//开始打开链接WiFi
		{	
			cnt_heartbag=0;                                                                                         //清空计数以免重启发心跳error
			wifi_init();
			sprintf((char*)bufssid,"AT+WJAP=%s,%s",gDeviceParam.wifiArg.WifiName,gDeviceParam.wifiArg.WifiPwd);  		//设置无线参数:ssid,密码   后续更改
			sprintf((char*)bufServer,"AT+MQTT=1,\"%d.%d.%d.%d\"",gDeviceParam.ServerIpAddress[0]
																													 ,gDeviceParam.ServerIpAddress[1]
																													 ,gDeviceParam.ServerIpAddress[2]
																													 ,gDeviceParam.ServerIpAddress[3]);    						  //设置MQ服务器
			sprintf((char*)bufport,"AT+MQTT=2,\"%d\"",gDeviceParam.mqArg.mqttPort);		
			sprintf((char*)bufway,"AT+MQTT=3,\"%d\"",1);    																												//连接方式
			sprintf((char*)bufMqid,"AT+MQTT=4,\"%d\"",gDeviceParam.devId);   																				//设置MQid
			sprintf((char*)buf_MQ_User,"AT+MQTT=5,\"%s\"",gDeviceParam.mqArg.MQ_Username);
			sprintf((char*)buf_MQ_pw,"AT+MQTT=6,\"%s\"",gDeviceParam.mqArg.MQ_Password);	
			sprintf((char*)bufCon,"AT+MQTT");			
			sprintf((char*)bufsub,"AT+MQTTSUB=\"%s\",0",gTopicInfo.cmdReply);																				//设置MQ订阅信息
			printf("wifi-go-to-con-debug\r\n");
			osDelay (500);
			atk_8266_send_cmd(bufssid,0xFFFF);	
			osDelay (200);
			HAL_UART_Receive_DMA(&huart3, RecCom3,COM3_REC_MAX);																										//读取串口数据  
			gGlobalData.Wifi_set = false;
		}
		if(gUartPcTc == false)
			HAL_UART_Receive_DMA(&huart3, RecCom3,COM3_REC_MAX);																										//读取串口数据
		
		//当网络处于断开 且当前标志让给wifi时  开始连接wifi
		if(gGlobalData.isConnect==0&&gGlobalData.conFlage==2){
			
				//	wifi开始工作		如果当前wifi连接成功了，则开始往下执行 切换wifi
				if(gGlobalData.wifiStatus ==true && !gGlobalData.isSendMq)
				{
					printf("wifi-set-usermsg\r\n");
					gGlobalData.isSendMq=true;
					atk_8266_send_cmd(bufServer,0xFFFF);
					osDelay (200);
					atk_8266_send_cmd(bufport,0xFFFF);
					osDelay (200);
					atk_8266_send_cmd(bufway,0xFFFF);
					osDelay (200);
					atk_8266_send_cmd(bufMqid,0xFFFF);
					osDelay (200);			
					atk_8266_send_cmd(buf_MQ_User,0xFFFF);
					osDelay (200);					
					atk_8266_send_cmd(buf_MQ_pw,0xFFFF);
					osDelay (200);	
					atk_8266_send_cmd(bufCon,0xFFFF);
					HAL_UART_Receive_DMA(&huart3, RecCom3,COM3_REC_MAX);
				}
				if(strstr((const char*)RecCom3,MQTT_CONNECT)){
					printf("wifi-working\r\n");
					atk_8266_send_cmd(bufsub,0xFFFF);
					osDelay (1000);
					gGlobalData.netKind = 2;

					gGlobalData.isConnect=1;																																						//说明连接服务器成功
					send_NetSync(3);
					osDelay(100);
					send_duan_wang(0);
					memset(RecCom3,0,sizeof(RecCom3));
					osDelay(100);
					cnt_heartbag = 0;																																										//发送心跳清空心跳计数器
					gGlobalData.heartbag_flage=1;			     																													    //连接上服务器立马发送心跳并接收解析
					}

		}

		
		//网络状态正常  且为wifi连接的情况下 才去解析服务器发送过来的数据
		if(gGlobalData.isConnect==1&&gGlobalData.wifiStatus ==true&&gGlobalData.netKind==2){
		/*数据接受解析处理*/  	
		if((cjdata=strstr((const char*)RecCom3,MQTT_DATA_REC))!=0 ){
			 osDelay(10);
				if(strstr((const char*)cjdata,"HeartBag")){
					printf("HeartBag_wifi\r\n");
					memset(RecCom3,0,sizeof(RecCom3));				
				}
				else if(strstr((const char*)cjdata,"ACK")){
					printf("ACK_wifi\r\n");
					memset(RecCom3,0,sizeof(RecCom3));
				}
				else{   																																													//心跳包不进解析		
				cjdata_f=cjdata;
				cjdata_p=cjdata;
				for(;;)
					{
						if(cjdata_f=strstr((const char*)(cjdata_f+1),MQTT_DATA_REC))
							_cjdata_BK_Num++;         																																	//反括号数,下面for循环里多检查一个反括号,因为进来判断已经有一个了
						else break;
					}
				for(_cjdata_i=0;_cjdata_i<=_cjdata_BK_Num;_cjdata_i++)
				{
					if(cjdata_p=strstr((const char*)(cjdata_p+1),MQTT_DATA_UNREC)){
						if(_cjdata_i==_cjdata_BK_Num){
							_cjdata_BK_Num=0;
							printf("解析成功_wifi\r\n");
							gGlobalData.heart_count=0;           																											  //进来清空倒计时 发送一次心跳就清空一次
							gGlobalData.wifiStatus=true;
							netData_process(cjdata,sizeof(cjdata));
							memset(RecCom3,0,sizeof(RecCom3));

							if(gGlobalData.ResetStatus==true)
							{
								gGlobalData.ResetStatus=false;
								do_work_ctl(3);
								osDelay (100); 
								send_QRInfo(gDeviceParam.qrbuf,strlen(gDeviceParam.qrbuf));   //发送二维码到屏幕左侧上显示								
							}
							send_NetSync(3);   																																					 //网络灯 wifi正常 

						}
					}
					else break;
				}
				_cjdata_BK_Num=0;
				_cjdata_i=0;
				}
		}
	}
		
#if 1		
		//连接不上WiFi 回复cwjap:_wifi_Apcode      
		if(cjdata=strstr((const char*)RecCom3,WIFI_CON_ERROR)){
			  gGlobalData.wifiStatus = false;
				
				if((gGlobalData.netKind !=1||gGlobalData.rj45Status!=true) && gGlobalData.yd4gStatus == false){     //2023/7/20 by yls
					gGlobalData.conFlage=3;												//跳转到4G
					gGlobalData.isConnect=0;											//断网
					send_NetSync(2);   													 	//网络灯 1601  
					osDelay(100);
					send_duan_wang(1);														//断网提示 1900 
					printf("wifi_error_goto_4g\r\n");
				}
				_wifi_Apcode=cjdata[5];
				printf("wifi_connect_error:%c%c\r\n",_wifi_Apcode,cjdata[6]);
				switch(_wifi_Apcode){
					case 0x31:		break;        									//_wifi_Apcode=1:连接超时
					case 0x32:		break;				 									//_wifi_Apcode=2:密码错误
					case 0x33:		break;													//_wifi_Apcode=3:未找到wifi
					case 0x34:		break;				 									//_wifi_Apcode=4:连接失败 请重新输入WiFi连接
					default  :		break;
				}
				memset(RecCom3,0,sizeof(RecCom3));		
			}
		
		//wifi在使用过程中断开连接
		if(gGlobalData.wifiStatus == true && gGlobalData.netKind == 2 && (cjdata=strstr((const char*)RecCom3,WIFI_DIS_C ))){
			gGlobalData.wifiStatus = false;
			printf("wifi_disconnect\r\n");
      if(gGlobalData.yd4gStatus == false && gGlobalData.rj45Status!=true){
				gGlobalData.conFlage=3;																					//跳转到4G
				gGlobalData.isConnect=0;																				//断网
				osDelay(50);
				send_NetSync(2);   																						 	//wifi故障灯网络灯 1601  
				osDelay(100);
				send_duan_wang(1);																							//断网提示 1900 
				printf("wifi_disconnect_goto_4g\r\n");
			}               
		}
		//获取WiFi ip地址成功上网	
		if(cjdata=strstr((const char*)RecCom3,WIFI_GOT_IP)){
			printf("wifi_connect_success\r\n");
			gGlobalData.wifiStatus = true;																																//说明wifi能上网
			send_duan_wang(0);																																						//说明上网正常了 去掉弹框
			memset(RecCom3,0,sizeof(RecCom3));
			gGlobalData.isSendMq=false;		 																															  //连上网就等待切换	
			if(gGlobalData.rj45Status != true || gGlobalData.yd4gStatus == true ){    	 								  //一开始网口断开并且没网，或者wifi切4g
				gGlobalData.conFlage = 2;	           																											  //点击连切wifi连接    4g连上或者网口没连上的状态
				gGlobalData.isConnect=0;																																		//断网
				if(gGlobalData.yd4gStatus == true){
					HAL_UART_Transmit_DMA(&huart6,"AT+QMTCLOSE=0\r\n",sizeof("AT+QMTCLOSE=0\r\n"));
					gGlobalData.yd4gStatus=false;
				}				
			}					
		}
		if(strstr((const char*)RecCom3,"+MQTTPUBRAW:") || strstr((const char*)RecCom3,"ERROR")){      //wifi指令error后重新设置wifi
			gGlobalData.Wifi_set=true;
			cnt_heartbag=0; 																							 //心跳计数清0
			gGlobalData.isConnect=0;
			printf("MQ_Pub_error_WIFI\r\n");
		} 
		
		

#endif
	}


}




//向esp07s发送命令
//cmd:发送的命令字符串
//waittime:等待时间(单位:10ms)
void atk_8266_send_cmd(uint8_t *cmd,uint16_t waittime)
{
	sprintf(rec_cmd,"%s\r\n",cmd);	//发送命令
	HAL_UART_Transmit_Wifi(rec_cmd,strlen(rec_cmd),0xffff);

	return;
} 
//wifi初始化
void wifi_init(void)
{
	osDelay(200);	
	atk_8266_send_cmd("AT",0xFFFF);
	osDelay(500);	
	atk_8266_send_cmd("AT+RST",0xFFFF);																																					//设置WIFI STA模式
	osDelay(3000);
	atk_8266_send_cmd("AT+WMODE=1,1",0xFFFF);
	osDelay(200);
	return;
}

//wifi初始化挂掉wifi
void wifi_deinit(void)
{
	gGlobalData.wifiStatus = false;
	cnt_heartbag = 0;																																														//切wifi时清空心跳计数器
	osDelay(100);	
	atk_8266_send_cmd("AT",0xFFFF);
	osDelay(200);	
	atk_8266_send_cmd("AT+RST",0xFFFF);																																					//设置WIFI STA模式
	osDelay(3000);
	atk_8266_send_cmd("AT+WJAP=0,0",0xffff);
//	atk_8266_send_cmd("AT+WMODE=0,0",0xffff);
	printf("WIFI-Reset-powerdown");
	return;
}
