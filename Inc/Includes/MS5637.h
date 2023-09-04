/**
  ******************************************************************************
  * @file    MS5637.h
  * @author  hj
  * @brief   Header file of MS5637.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 hj</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:

  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MS5637_H
#define __MS5637_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "gpio.h"
//#include "HalError.h"
#include "stdbool.h"
#include "cmsis_os.h"

//#include "i2c.h"



/* Private defines -----------------------------------------------------------*/

//MS5637
#define MS5637_ADDR          0xEC //address

#define MS5637_RESET         0x1E //reset command
#define MS5637_PROM          0xA0 //PROM command
#define MS5637_ADC_READ      0x00 //ADC Read


#define  MS5637_D1_OSR_256   0x40 //OSR value
#define  MS5637_D1_OSR_512   0x42 
#define  MS5637_D1_OSR_1024  0x44 
#define  MS5637_D1_OSR_2048  0x46 
#define  MS5637_D1_OSR_4096  0x48 
#define  MS5637_D1_OSR_8192  0x4A 

#define  MS5637_D2_OSR_256   0x50 
#define  MS5637_D2_OSR_512   0x52 
#define  MS5637_D2_OSR_1024  0x54 
#define  MS5637_D2_OSR_2048  0x56 
#define  MS5637_D2_OSR_4096  0x58 
#define  MS5637_D2_OSR_8192  0x5A 

#define OSR_8192_CONVERSION_TIME	19
#define OSR_4096_CONVERSION_TIME	10
#define OSR_2048_CONVERSION_TIME	5
#define OSR_1024_CONVERSION_TIME	3
#define OSR_512_CONVERSION_TIME		2
#define OSR_256_CONVERSION_TIME		1

#define MS5637_02BA03_HI2C			&hi2c3		/*I2C_HandleTypeDef pointer*/
#define MS5637_30BA_HI2C			&hi2c3		/*I2C_HandleTypeDef pointer*/







/* USER CODE BEGIN Private defines */

typedef enum {
eMS5637_02BA03,
eMS5637_30BA	
}MS5637TypeDef;


bool HAL_MS5637_reset(MS5637TypeDef type);
bool HAL_MS5637_prom_init(MS5637TypeDef type);
bool HAL_MS5637_read_adc(MS5637TypeDef type,uint8_t di);
bool HAL_MS5637_OSR(MS5637TypeDef type,uint8_t osr);
bool HAL_MS5637_calcuate(MS5637TypeDef type,float *temp,float *press);

/* USER CODE END Private defines */

////////////////////////////////////////////////////////////////////////////////
/*------------------ms5637 驱动无关的应用部分start-------------------------------------*/
#if 1	//应用打开

#define SENSORTIM_HTIM						(&htim6)
#define SENSOR2_OPREATE_FAIL				0X02			/*opreate fail set 1*/
#define SENSOR2_NO_REPONSE					0X04
#define SENSOR2_INIT_SUCCESS				0x20	//0X40			/*init success set 1*/
#define SENSOR2_RESET_SUCCESS				0x40	//0X60
#define SENSOR2_CONFIG						0X80  /*before sensor use ,config*/

#define SENSOR2_SUCCESS_MSK				(SENSOR2_INIT_SUCCESS|SENSOR2_RESET_SUCCESS|SENSOR2_CONFIG)

typedef struct {
	float 	InnerPressure; /*MS5637_02BA03 Pressure*/
	float 	InnerTemp;			/*MS5637_02BA03 temperature*/ 
	uint8_t InnerStatus;
}PressDataTypeDef;



extern PressDataTypeDef gSensorData2;

void DataCollectInit(void );
void DataCollectExecute(void );
#endif
/**-----------------------ms5637应用end----------------------*/
#ifdef __cplusplus
}
#endif

#endif /* __MS5637_H */

/************************ (C) COPYRIGHT HJ *****END OF FILE****/

