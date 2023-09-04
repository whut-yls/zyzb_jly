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
#ifndef __AD5722_H
#define __AD5722_H

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/

#define AD5722_ID0	0
#define AD5722_CH_A		0
#define AD5722_CH_B		2
#define AD5722_CH_AB		4
#define AD5722_NUMS 	1
#define AD5722_TIMEOUT_VALUE		2000
#define REG_MAX		2048			//12 bit
#define REFIN_C		5		//ref vol 2.5v*2=5v
#define	POSITIVE_MASK		0x0800		//|正数掩码
#define	NEGATIVE_MASK		0x07FF		//&负数掩码
#define AD5722_LDAC(x)		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,x ? GPIO_PIN_SET : GPIO_PIN_RESET) //LDAC
#define AD5722_CLR(x)			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5,x ? GPIO_PIN_SET : GPIO_PIN_RESET) //CLR
#define AD5722_CS(x)			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,x ? GPIO_PIN_SET : GPIO_PIN_RESET) //CS
////AD5722 register
#define AD5722_DAC_REG   0x00
#define AD5722_RANGE_REG   0x01
#define AD5722_POWER_CTL_REG    0x02
#define AD5722_CTL_REG 		 0x03
//	
///** <! support chip type,selected one per time  */
////#define AD5752
////#define AD5732
//#define AD5722
/*DAC register*/
typedef struct DAC_REG_ad5722
{
  union
  {
    uint32_t value;
    struct
    {
      unsigned resv:4;
      unsigned data:12;
			unsigned dac_change:3;
			unsigned reg_add:3;
			unsigned zero:1;
      unsigned rw:1;
      unsigned dummy:32;
    }reg;
  }u;
}AD5722_DataDef;	

/*OUTPUT RANGE SELECT register*/
typedef struct RANGE_REG_ad5722
{
  union
  {
    uint32_t value;
    struct
    {
      unsigned range_value:3;	//0x03 +-5	0x04 +-10
      unsigned resv:13;
			unsigned dac_change:3;	//0x4	both A+B  0x0 A   0X02 B
			unsigned reg_add:3;			//see macro default
			unsigned zero:1;				//forever 0
      unsigned rw:1;					//1 R 	0 W
      unsigned dummy:32;
    }reg;
  }u;
}AD5722_RangeDef;

/*control register*/
typedef struct CTL_REG_ad5722
{
  union
  {
    uint32_t value;
    struct
    {
      unsigned sdo_dis:1;	//0x03 +-5	0x04 +-10
      unsigned clr_select:1;
			unsigned clamp_en:1;
			unsigned tsd_en:1;		//0x1	clr tsd clamp sdo	;0x4 clear ;0x5 load  0x0 not care
			unsigned resv:12;
			unsigned fun_select:3;	//0x1	forever
			unsigned reg_add:3;			//see macro default
			unsigned zero:1;				//forever 0
      unsigned rw:1;					//1 R 	0 W
      unsigned dummy:32;
    }reg;
  }u;
}AD5722_CtlClrDef;

/*power control register*/
typedef struct POWER_REG_ad5722
{
  union
  {
    uint32_t value;
    struct
    {
      unsigned pu_a:1;	
      unsigned resv1:1;
			unsigned pu_b:1;
			unsigned resv2:2;
			unsigned tsd:1;
			unsigned resv3:1;
			unsigned oc_a:1;
			unsigned resv4:1;
			unsigned oc_b:1;
			unsigned db10:1;	//set 0
			unsigned resv5:5;
			
			unsigned fun_select:3;	//forever 0x0
			unsigned reg_add:3;			//see macro default	0x02
			unsigned zero:1;				//forever 0
      unsigned rw:1;					//1 R 	0 W
      unsigned dummy:32;
    }reg;
  }u;
}AD5722_PowerDef;

///* Exported functions -------------------------------------------------------*/
//AD5601 init
void AD5722_Init(void);

//AD5722 set value output
void AD5722_Comm_Set(uint8_t item, uint8_t mode, float data);
	
		
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
