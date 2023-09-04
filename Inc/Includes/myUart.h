/**
  ******************************************************************************
  * @file           : myTimer.h
  * @brief          : Header for myTimer.c file.
  *                   This file contains the common defines of the application
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MYUART_H
#define __MYUART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define COM1_REC_MAX	 512
#define COM4_REC_MAX	 512
#define COM5_REC_MAX	 4196

extern unsigned char RecCom1[COM1_REC_MAX+1];
extern uint16_t 			RecCom1Num;

extern unsigned char RecCom4[COM4_REC_MAX+1];
extern uint16_t 			RecCom4Num;

extern unsigned char RecCom5[COM5_REC_MAX+1];
extern uint16_t 			RecCom5Num;

void Uart1_IDLE_HandleRec(void);
void Uart4_IDLE_HandleRec(void);
void Uart5_IDLE_HandleRec(void);

/* USER CODE END Private defines */
void MX_USART1_UART_Init(void);
void MX_UART4_Init(void);
void MX_UART5_Init(void);

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
