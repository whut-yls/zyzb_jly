 /******************************************************************************
 * @file					 : MqttTask.h
 * @brief 				 : Header for MqttTask.c file.
 *									 .
 ******************************************************************************
 
 ******************************************************************************
 */

#ifndef	__MQTTTASK_H__
#define __MQTTTASK_H__

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "MQTTPacket.h"
#include "cmsis_os.h"


#ifdef __cplusplus
 extern "C" {
#endif

enum QoS { QOS0, QOS1, QOS2, SUBFAIL=0x80 };

/* all failure return codes must be negative */
enum MQreturnCode { MQ_BUFFER_OVERFLOW = -2, MQ_FAILURE = -1, MQ_SUCCESS = 0 ,MQ_PINGRESP=1};


typedef struct MQTTMessage
{
    enum QoS qos;
    unsigned char retained;
    unsigned char dup;
    unsigned short id;
    void *payload;
    size_t payloadlen;
} MQTTMessage;


typedef struct MessageData
{
    MQTTMessage* message;
    MQTTString* topicName;
} MessageData;

typedef struct MQTTConnackData
{
    unsigned char rc;
    unsigned char sessionPresent;
} MQTTConnackData;


#define EC20_MQ_CONNECTID			0							/*EC20 mq connectID*/
#define MQTTMESSAGE_MAXLENGTH			4095
//#define MQTTMESSAGE_MAXLENGTH			9190
#define MAX_PACKET_ID 65535 /* according to the MQTT specification - do not change! */

#if !defined(MAX_MESSAGE_HANDLERS)
#define MAX_MESSAGE_HANDLERS 5 /* redefinable - how many subscriptions do you want? */
#endif


#define	 MQ_NET_ID	0

extern MQTTPacket_connectData MQTTPacket_ConnectData_Initializer;
void MQTT_ConnectData_Init(MQTTPacket_connectData* data);

int MQTTConnect(MQTTPacket_connectData* options);
int MQTTDisConnect(MQTTPacket_connectData* options);
int MQTTPublish(const char* topicName, MQTTMessage* message);
 int MQTTSubscribe_RecvMessage( MQTTMessage* message,MQTTString*topic);
int MQTTSubscribe(const char* topicName, MQTTMessage* message);
#ifdef __cplusplus
}
#endif

#endif /* __MQTTTASK_H__ */

/************************ (C) COPYRIGHT  hj*****END OF FILE****/

