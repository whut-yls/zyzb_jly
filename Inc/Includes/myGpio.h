/**
  ******************************************************************************
  * @file           : myGpio.h
  * @brief          : Header for myGpio.c file.
  *                   This file contains the common defines of the application
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MYGPIO_H
#define __MYGPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */


/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define IN1_PORT 	GPIOB
#define IN2_PORT 	GPIOC
#define IN3_PORT 	GPIOG
#define IN4_PORT 	GPIOD
#define IN5_PORT 	GPIOB
#define IN6_PORT 	GPIOB
#define IN7_PORT 	GPIOB
#define IN8_PORT 	GPIOB
#define IN9_PORT 	GPIOC
#define IN10_PORT GPIOC

#define IN1_PIN 	GPIO_PIN_4
#define IN2_PIN 	GPIO_PIN_8
#define IN3_PIN 	GPIO_PIN_3
#define IN4_PIN 	GPIO_PIN_11
#define IN5_PIN 	GPIO_PIN_15
#define IN6_PIN 	GPIO_PIN_14
#define IN7_PIN 	GPIO_PIN_13
#define IN8_PIN 	GPIO_PIN_12
#define IN9_PIN 	GPIO_PIN_5
#define IN10_PIN 	GPIO_PIN_4
		
#define ReadIN1()		HAL_GPIO_ReadPin(IN1_PORT,IN1_PIN)
#define ReadIN2()		HAL_GPIO_ReadPin(IN2_PORT,IN2_PIN)
#define ReadIN3()		HAL_GPIO_ReadPin(IN3_PORT,IN3_PIN)
#define ReadIN4()		HAL_GPIO_ReadPin(IN4_PORT,IN4_PIN)
#define ReadIN5()		HAL_GPIO_ReadPin(IN5_PORT,IN5_PIN)
#define ReadIN6()		HAL_GPIO_ReadPin(IN6_PORT,IN6_PIN)
#define ReadIN7()		HAL_GPIO_ReadPin(IN7_PORT,IN7_PIN)
#define ReadIN8()		HAL_GPIO_ReadPin(IN8_PORT,IN8_PIN)
#define ReadIN9()		HAL_GPIO_ReadPin(IN9_PORT,IN9_PIN)
#define ReadIN10()		HAL_GPIO_ReadPin(IN10_PORT,IN10_PIN)

#define OUT1_PORT 	GPIOG
#define OUT2_PORT 	GPIOG
#define OUT3_PORT 	GPIOG
#define OUT4_PORT 	GPIOD
#define OUT5_PORT 	GPIOD
#define OUT6_PORT 	GPIOD
#define OUT7_PORT 	GPIOD
#define OUT8_PORT 	GPIOD
#define OUT9_PORT 	GPIOG
#define OUT10_PORT 	GPIOG

#define OUT1_PIN 		GPIO_PIN_14
#define OUT2_PIN 		GPIO_PIN_13
#define OUT3_PIN 		GPIO_PIN_9
#define OUT4_PIN 		GPIO_PIN_7
#define OUT5_PIN 		GPIO_PIN_4
#define OUT6_PIN 		GPIO_PIN_2
#define OUT7_PIN 		GPIO_PIN_1
#define OUT8_PIN 		GPIO_PIN_0
#define OUT9_PIN 		GPIO_PIN_7
#define OUT10_PIN 		GPIO_PIN_8

#define WriteOUT1(X)	HAL_GPIO_WritePin(OUT1_PORT,OUT1_PIN,X)
#define WriteOUT2(X)	HAL_GPIO_WritePin(OUT2_PORT,OUT2_PIN,X)
#define WriteOUT3(X)	HAL_GPIO_WritePin(OUT3_PORT,OUT3_PIN,X)
#define WriteOUT4(X)	HAL_GPIO_WritePin(OUT4_PORT,OUT4_PIN,X)
#define WriteOUT5(X)	HAL_GPIO_WritePin(OUT5_PORT,OUT5_PIN,X)
#define WriteOUT6(X)	HAL_GPIO_WritePin(OUT6_PORT,OUT6_PIN,X)
#define WriteOUT7(X)	HAL_GPIO_WritePin(OUT7_PORT,OUT7_PIN,X)
#define WriteOUT8(X)	HAL_GPIO_WritePin(OUT8_PORT,OUT8_PIN,X)
#define WriteOUT9(X)	HAL_GPIO_WritePin(OUT9_PORT,OUT9_PIN,X)
#define WriteOUT10(X)	HAL_GPIO_WritePin(OUT10_PORT,OUT10_PIN,X)
/* USER CODE END Private defines */
void MX_GPIO_Init(void);
void urgency_close(void);
	
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
