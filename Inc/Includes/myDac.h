/**
  ******************************************************************************
  * @file           : myDac.h
  * @brief          : Header for myDac.c file.
  *                   This file contains parameter
  ******************************************************************************
  
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MYDAC_H__
#define __MYDAC_H__
#include "stm32f4xx_hal.h"

extern DAC_HandleTypeDef hdac;
extern void MX_DAC_Init(void);
uint16_t changeDac_2_De(float vol);

#endif
