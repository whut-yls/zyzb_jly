/* USER CODE BEGIN Header */
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
#ifndef __MYTIMER_H
#define __MYTIMER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;	
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim13;

void MX_TIM1_Init(void);
void MX_TIM4_Init(void);	
void MX_TIM7_Init(void);
void MX_TIM13_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
