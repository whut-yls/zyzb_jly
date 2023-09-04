/**
  ******************************************************************************
  * @file           : myTimer.h
  * @brief          : Header for myTimer.c file.
  *                   This file contains the common defines of the application
  *
  ******************************************************************************
  */
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STARTLOOPTASK_H
#define __STARTLOOPTASK_H

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
void startLoopTask(void const * argument);
/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
uint8_t key_scan(void);
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
