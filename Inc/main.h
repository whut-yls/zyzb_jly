/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stdbool.h"
#include "cmsis_os.h"
#include "ctype.h"
#include "parameter.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
extern int cnt_heartbag;
/* USER CODE BEGIN EFP */
extern bool gBusyRead[3];
extern uint8_t gAUTO_ENABLE;
extern uint16_t gREFVOL_VAL;
extern bool gEthRecvStatus;
extern bool gWifiChange;    //wifi 
extern bool gEC200SChange;  //EC200S
//extern bool gEthRecvReturn;
//extern bool gEthSendReturn;
extern uint32_t collectDataCount;//采集数据个数
//extern bool gEthSendStatus;
extern bool gRecvEthChange;
//extern bool gSendEthChange;
extern bool gEthStatus;
//extern bool gTransferDon;

extern bool gMqttLinkStatus;      //网络状态
/* USER CODE END EFP */
extern uint16_t  ch1buf[100];

extern float Level;//档位强度控制变量
/* Private defines -----------------------------------------------------------*/
#define RMII_INT_Pin GPIO_PIN_2
#define RMII_INT_GPIO_Port GPIOH
#define RMII_RES_Pin GPIO_PIN_3
#define RMII_RES_GPIO_Port GPIOH
#define WIFI_RESET_Pin GPIO_PIN_10
#define WIFI_RESET_GPIO_Port GPIOD
#define WIFI_EN_Pin GPIO_PIN_11
#define WIFI_EN_GPIO_Port GPIOD
#define SOCKET_LINK_Pin GPIO_PIN_12
#define SOCKET_LINK_GPIO_Port GPIOD
#define WIFI_LINK_Pin GPIO_PIN_13
#define WIFI_LINK_GPIO_Port GPIOD

#define IO1_Pin GPIO_PIN_9
#define IO1_GPIO_Port GPIOE

#define IO2_Pin GPIO_PIN_10
#define IO2_GPIO_Port GPIOE

#define IO3_Pin GPIO_PIN_11
#define IO3_GPIO_Port  GPIOE

#define IO4_Pin GPIO_PIN_13
#define IO4_GPIO_Port GPIOE

#define IO5_Pin GPIO_PIN_14
#define IO5_GPIO_Port GPIOE

#define IO6_Pin GPIO_PIN_15
#define IO6_GPIO_Port GPIOE

#define IO7_Pin GPIO_PIN_15
#define IO7_GPIO_Port GPIOF

#define IO8_Pin GPIO_PIN_0
#define IO8_GPIO_Port GPIOG

#define IO9_Pin GPIO_PIN_1
#define IO9_GPIO_Port GPIOG

#define IO10_Pin GPIO_PIN_13
#define IO10_GPIO_Port GPIOH

#define IO11_Pin GPIO_PIN_14
#define IO11_GPIO_Port GPIOH

#define IO12_Pin GPIO_PIN_15
#define IO12_GPIO_Port GPIOH

#define IO13_Pin GPIO_PIN_0
#define IO13_GPIO_Port GPIOI

#define IO14_Pin GPIO_PIN_1
#define IO14_GPIO_Port GPIOI

#define RS485_EN2_Pin GPIO_PIN_10
#define RS485_EN2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_3
#define LED4_GPIO_Port GPIOD

#define IN4_Pin GPIO_PIN_12       //旧板子是pg14
#define IN4_GPIO_Port GPIOG
#define IN2_Pin GPIO_PIN_15
#define IN2_GPIO_Port GPIOG

#define IN3_Pin GPIO_PIN_8          //旧板子是pb6
#define IN3_GPIO_Port GPIOB
#define IN1_Pin GPIO_PIN_5
#define IN1_GPIO_Port GPIOB
#define RS485_EN_Pin GPIO_PIN_9
#define RS485_EN_GPIO_Port GPIOB

#define TASK_BIT_1    (0x01 << 0)
#define TASK_BIT_2    (0x01 << 1)
#define TASK_BIT_3    (0x01 << 2)
#define TASK_BIT_4    (0x01 << 3)
#define TASK_BIT_5    (0x01 << 4)
#define TASK_BIT_6    (0x01 << 5)
#define TASK_BIT_ALL    ( TASK_BIT_1 )
//#define TASK_BIT_ALL    ( TASK_BIT_1 |TASK_BIT_2 | TASK_BIT_3|TASK_BIT_4)

/* USER CODE BEGIN Private defines */
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_ADC3_Init(void);
void MX_ETH_Init(void);
void MX_I2C2_Init(void);
void MX_SPI5_Init(void);
void MX_UART4_Init(void);
void MX_UART7_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART6_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_I2C1_Init(void);
void MX_SPI1_Init(void);
void MX_SPI3_Init(void);
void MX_DMA_Init(void);
void MX_I2C3_Init(void);
void MX_USART3_UART_Init(void);

void Set_Input_Output(uint8_t Flag);
void Set_I2c_Register_Clear_Reset(void);

bool Get_ADC1_Hex(void);
bool Get_ADC3_Hex(void);
/* USER CODE BEGIN Private defines */
extern I2C_HandleTypeDef hi2c1;  //AHT15
extern I2C_HandleTypeDef hi2c2;  //PCA9554
extern I2C_HandleTypeDef hi2c3;  //MS5637

extern SPI_HandleTypeDef hspi1;  //ad9833
extern SPI_HandleTypeDef hspi2;  //ad9833外部可调电阻
extern SPI_HandleTypeDef hspi3;  //ad9833外部可调电阻
extern SPI_HandleTypeDef hspi5;  //外部FLASH

extern UART_HandleTypeDef huart2;   //音乐
extern UART_HandleTypeDef huart4;   //RS485 2
extern UART_HandleTypeDef huart7;   //TYPEC RS232 CH340
extern UART_HandleTypeDef huart8;  //RS485 1
extern UART_HandleTypeDef huart1;   //J1 外接4G的DTU
extern UART_HandleTypeDef huart3;  //板载J2??
extern UART_HandleTypeDef huart6;  //WIFI模块
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_uart8_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern uint16_t gREFVOL_VAL;

//读取四路GPIO信号 
#define ReadINPUT1 HAL_GPIO_ReadPin(IN1_GPIO_Port,IN1_Pin)
#define ReadINPUT2 HAL_GPIO_ReadPin(IN2_GPIO_Port,IN2_Pin)
#define ReadINPUT3 HAL_GPIO_ReadPin(IN3_GPIO_Port,IN3_Pin)
#define ReadINPUT4 HAL_GPIO_ReadPin(IN4_GPIO_Port,IN4_Pin)

#define WriteOUT1(X)	HAL_GPIO_WritePin(IO1_GPIO_Port,IO1_Pin,X)
#define WriteOUT2(X)	HAL_GPIO_WritePin(IO2_GPIO_Port,IO2_Pin,X)
#define WriteOUT3(X)	HAL_GPIO_WritePin(IO3_GPIO_Port,IO3_Pin,X)
#define WriteOUT4(X)	HAL_GPIO_WritePin(IO4_GPIO_Port,IO4_Pin,X)
#define WriteOUT5(X)	HAL_GPIO_WritePin(IO5_GPIO_Port,IO5_Pin,X)
#define WriteOUT6(X)	HAL_GPIO_WritePin(IO6_GPIO_Port,IO6_Pin,X)
#define WriteOUT7(X)	HAL_GPIO_WritePin(IO7_GPIO_Port,IO7_Pin,X)
#define WriteOUT8(X)	HAL_GPIO_WritePin(IO8_GPIO_Port,IO8_Pin,X)
#define WriteOUT9(X)	HAL_GPIO_WritePin(IO9_GPIO_Port,IO9_Pin,X)
#define WriteOUT10(X)	HAL_GPIO_WritePin(IO10_GPIO_Port,IO10_Pin,X)
#define WriteOUT11(X)	HAL_GPIO_WritePin(IO11_GPIO_Port,IO11_Pin,X)
#define WriteOUT12(X)	HAL_GPIO_WritePin(IO12_GPIO_Port,IO12_Pin,X)
#define WriteOUT13(X)	HAL_GPIO_WritePin(IO13_GPIO_Port,IO13_Pin,X)
#define WriteOUT14(X)	HAL_GPIO_WritePin(IO14_GPIO_Port,IO14_Pin,X)

#define WriteOUT_Mode(X)	HAL_GPIO_WritePin(RS485_EN_GPIO_Port,RS485_EN_Pin,X)

#define WriteOUTRun(X)	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,X)



#define MODE_ZT	1	//整体采样
#define MODE_JB	2	//局部采样
#define MODE_ZL	3	//治疗模式
#define MODE_CLOSE	4	//关闭信号源

#define COM2_REC_MAX	 6         //音频播放接收   
#define COM7_REC_MAX	 256
#define COM3_REC_MAX	 256*5     //wifi串口接收 cjsondata
#define COM6_REC_MAX	 256*8     //4g串口接收 cjsondata



#define Voice_RXBUFFERSIZE 10  //语音模块接收
extern unsigned char Voice_aRxBuffer[Voice_RXBUFFERSIZE];

extern unsigned char RecCom7[COM7_REC_MAX+1];	//pc
extern uint16_t 			RecCom7Num;
extern unsigned char RecCom3[COM3_REC_MAX+1];	//pc
extern uint16_t 			RecCom3Num;
extern unsigned char RecCom6[COM6_REC_MAX+1];	//pc
extern uint16_t 			RecCom6Num;
extern unsigned char RecCom2[COM2_REC_MAX];
extern uint8_t       RecSen[30]; 

extern SemaphoreHandle_t		ConsoleReceive_Semaphore; 
extern SemaphoreHandle_t		lcdSemaphore;
extern bool gUartPcInit,gUartPcTc;
extern bool gUartLcdInit,gUartLcdTc;
extern bool isCollect;
extern bool Collect_Data_state;
extern osThreadId ethernetTaskTaskHandle;
#define W25QX_nCS(x)  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,x ? GPIO_PIN_SET : GPIO_PIN_RESET) 
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
