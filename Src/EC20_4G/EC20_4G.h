#ifndef __EC20_4G_H
#define __EC20_4G_H	 
#include "sys.h"



void EC200MqttClientTask(void const * argument);

void EC20_4G_UART(void);
void send_NetSync(uint8_t status);
void send_duan_wang(uint8_t duan);


									

extern uint8_t aTxBuffer0[];		//关闭上次的socket连接
extern uint8_t aTxBuffer1[];							//测试指令，若返回OK，则模块可正常通信
extern uint8_t aTxBuffer2[];				//检查SIM卡是否在位
extern uint8_t aTxBuffer3[];					//查询信号质量
extern uint8_t aTxBuffer4[];				//查询入网状态
extern uint8_t aTxBuffer5[];			//激活网络
extern uint8_t aTxBuffer6[];				//查询激活状态
extern uint8_t aTxBuffer7[];			//获取PDP地址
extern uint8_t aTxBuffer8[];				//配置为接收模式
extern uint8_t aTxBuffer9[];				//	配置阿里云上的设备信息
extern uint8_t aTxBuffer10[];						//打开一个MQTT客户端网络
extern uint8_t aTxBuffer11[];					//连接MQTT服务器
extern uint8_t aTxBuffer12[];						//订阅服务器主题,这里订阅主题成功后可以在设备TOPIC查看，这里的主题功能是可以下发数据到设备		
extern uint8_t aTxBuffer13[];									//订阅上报主题，这里的功能是设备上报数据到云端		
extern uint8_t aTxBuffer14[];						//发布主题消息
extern uint8_t aTxBuffer15[];						//发送的JSON数据
extern uint8_t aTxBuffer16[];					//数据结束符1a，需采用HEX形式发送

#endif
