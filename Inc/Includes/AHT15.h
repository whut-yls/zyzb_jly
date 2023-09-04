/**
  ******************************************************************************
  * @file    AHT15.h
  * @author  hj
  * @brief   Header file of AHT15.
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
#ifndef __AHT15_H
#define __AHT15_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdbool.h"
#include "cmsis_os.h"

//#include "gpio.h"
//#include "HalError.h"
//#include "stdbool.h"
//#include "i2c.h"


/* Private defines -----------------------------------------------------------*/

//AHT15
#define AHT15_ADDR          0x70 //address  0x38 7位设备地址加一位bit0 1就是0x70

#define AHT15_INIT         	0xE1 //Measure Relative Humidity, Hold Master Mode
#define AHT15_RESET        	0xBA //Measure Relative Humidity, No Hold Master Mode  
#define AHT15_SEND_AC       0xAC //Measure Relative Humidity, Hold Master Mode



/*

*/
#define AHT15_SEND_AC_TIME						75
#define AHT15_SOFTWARE_RESET_TIME				200


//#define AHT15__HI2C		&hi2c1		/*I2C_HandleTypeDef pointer*/
#define AHT15__HI2C		&hi2c2		/*I2C_HandleTypeDef pointer*/    //debugi2c


/* USER CODE BEGIN Private defines */


bool HAL_AHT15_software_reset(void);
bool HAL_AHT15_INIT(void);
bool HAL_AHT15_SEND_AC(void);




/*-----------------自定义应用程序代码-----------------*/
#define	SENSOR_INIT_SUCCESS		0x02
#define	SENSOR_NO_REPONSE		0x04
#define	SENSOR_RESET_SUCCESS	0x08
#define SENSOR_SUCCESS_MSK		(SENSOR_INIT_SUCCESS|SENSOR_RESET_SUCCESS)

/* USER CODE END Includes */
/**
 * float在中间出现读取问题
 */
typedef struct{
	float 	RHt;	//温度
	int8_t 	RH; 		//湿度
	uint8_t RHStatus; //状态
}SensorType;

extern SensorType gSensorData1;

bool sensor_init(void);
bool sensor_read(uint8_t* buf,uint8_t num);
/**--------------------------------------------------*/

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __AHT15_H */

/************************ (C) COPYRIGHT HJ *****END OF FILE****/

