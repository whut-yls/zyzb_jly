/**
  ******************************************************************************
  * @file           : adc_in.h
  * @brief          : Header for adc_in.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MYADC_H__
#define __MYADC_H__

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define	N_NUM							30						//连续采集多少次 
#define AD1_NUM			  		1
#define AD3_NUM			  		1
//#define	AD_ALL_NUM				AD1_NUM			//AD1_NUM+..
	 
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern uint16_t gADC1_DMA_BUF[AD1_NUM*N_NUM];
extern uint16_t gADC1_VALUE[AD1_NUM];
extern uint16_t gADC1_VALUE_F[AD1_NUM];

extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc3;

extern uint16_t gADC3_DMA_BUF[AD3_NUM*N_NUM];
extern uint16_t gADC3_VALUE[AD3_NUM];
extern uint16_t gADC3_VALUE_F[AD3_NUM];


/**interface function**/	 

/**public function**/	 
bool Get_ADC1(float * buff);

bool Get_ADC1_Hex(void);

bool Get_ADC3(float * buff);

bool Get_ADC3_Hex(void);
/**private function**/

static bool  Read_All_AD1(uint16_t * buff,uint32_t longth);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_IN_H__ */

/************************ (C) COPYRIGHT HJ *****END OF FILE****/

