
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mySpi.h
  * @brief          : Header for mySpi.c file.
  *                   This file contains the common defines of the application
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MYSPI_H
#define __MYSPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stdbool.h"
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

void MX_SPI1_Init(void);
void MX_SPI2_Init(void);
void MX_SPI3_Init(void);
void MX_SPI4_Init(void);
void MX_SPI5_Init(void);
void MX_SPI6_Init(void);
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;
extern SPI_HandleTypeDef hspi6;
void sendData(uint8_t res,uint32_t d);
void readData(uint8_t res,uint32_t* d);
void sendPortData(uint8_t port ,uint8_t res,uint32_t d);
void readPortData(uint8_t port ,uint8_t res,uint32_t* d);

bool SPI_Read(char *src,int l);
bool SPI_Write(char *src,int l);	


void tmc4361A_readArray(uint8_t port,uint8_t *data, size_t length);
void tmc4361A_writeArray(uint8_t port ,uint8_t *data, size_t length);

bool SPI_Motor_Read(uint8_t i,uint8_t *SendByte,uint8_t SendSize,uint8_t *RecByte,uint8_t RecSize);
bool SPI_Motor_Write(uint8_t i,uint8_t *SendByte,uint8_t SendSize);

bool Spi_MutexeSemaphore_Init(void);

extern SemaphoreHandle_t	SpiMutexeSemaphore;



#define W25QX_nCS(x)  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,x ? GPIO_PIN_SET : GPIO_PIN_RESET) //spi4_CS

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
