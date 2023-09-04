/**
  ******************************************************************************
  * @file           : myTimer.h
  * @brief          : Header for myTimer.c file.
  *                   This file contains the common defines of the application
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STARTDEFAULTTASK_H
#define __STARTDEFAULTTASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdbool.h"
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
extern SemaphoreHandle_t  	defaultSemaphore;
extern SemaphoreHandle_t  	defaultSemaphore2;

/* USER CODE END Private defines */
bool startDefatltTask_Semaphore_Init(void);
bool DefaultTask_Init(void);
void startDefaultTask(void const * argument);
void StartDefaultTask2(void const * argument);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
