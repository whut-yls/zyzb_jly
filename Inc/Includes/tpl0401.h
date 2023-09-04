/**
  ******************************************************************************
  * @file           : ad5722.h
  * @brief          : Header for ad5722.c file.
  *                   This file contains the common defines of the application
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TPL0401_H
#define __TPL0401_H

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdbool.h"
/* Private includes ----------------------------------------------------------*/
#define TPL0401_10K		10			//片内调节电阻
#define	TPL0401A_ADDR		0x5C	//2E
#define	TPL0401B_ADDR		0x7C	//3E	
#define TPL0401A_ID		0
#define TPL0401B_ID		1
#define	TPL0401_TIMEOUT	0xFFFF		//1000ms
///* Exported functions -------------------------------------------------------*/

//AD5722 set value output
bool TPL0401_Comm_Set(uint8_t item, float data);
	
		
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
