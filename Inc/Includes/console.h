/**
  ******************************************************************************
  * @file           : console.h
  * @brief          : Header for console.c file.
  *                   This file contains debug serial console defines of the application.
  ******************************************************************************
  
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "main.h"




#ifdef __cplusplus
 extern "C" {
#endif

//#define COM_REC_MAX 		511
#define CONSOLE_PORT		&huart5

typedef bool (*SysCommandInvoke)(char* ,int );
typedef struct tagCommandOpt{
	char *src;
	SysCommandInvoke pf;
	unsigned char l;
}CommandOpt;

extern SemaphoreHandle_t		ConsoleReceive_Semaphore; 
//extern DMA_HandleTypeDef hdma_usart1_rx;

bool Console_TaskSemaphore_Init(void);


void Console_Task(void const * pvParameters);

bool Update_Set(char *src,int l);
bool Update_NetSet(char *src,int l);
bool Sys_Command_Set(char *src,int l);




#ifdef __cplusplus
}
#endif

#endif /* __CONSOLE_H__ */

/************************ (C) COPYRIGHT  hj*****END OF FILE****/

