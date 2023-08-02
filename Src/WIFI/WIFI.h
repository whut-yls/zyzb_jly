#ifndef __WIFI_H__
#define __WIFI_H__

#include "string.h"
#include "common.h"
#include "stdlib.h"
#include "main.h"
#include "com.h"

void WifiMqttClientTask(void const * argument);

void wifi_init(void);
void wifi_deinit(void);
void atk_8266_send_cmd(uint8_t *cmd,uint16_t waittime);
void send_NetSync(uint8_t status);
void send_duan_wang(uint8_t duan);
int netData_process(char *payload,int payloadLen);

#define WIFI_CON_ERROR  "WJAP:"               //连接不上WiFi 回复cwjap:_wifi_Apcode
#define WIFI_GOT_IP 		"WIFI_GOT_IP"          //WiFi 正常连接上
#define WIFI_DIS_C 			"WIFI_DISCONNECT"      //wifi断开
#define MQTT_DIS_C   		"MQTT_DISCONNECT"		 //MQTT断开	
#define MQTT_CONNECT    "MQTT_CONNECT"       //MQTT连接
#define MQTT_RECONNECT    "MQTT Reconnect"
#define MQTT_DATA_REC   "{"                    //数据处理
#define MQTT_DATA_UNREC "}"                    //数据处理

#endif

