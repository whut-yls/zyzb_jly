/********************************************************************************
 Author : CAC (China Applications Support Team) 

 Date :   May, 2014

 File name :   ADE7913.h

 Description :	 ADE7913 registers 

 Hardware plateform : 	EVAL-ADuCM360MKZ and EVAL-ADE7913EBZ
********************************************************************************/

/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _ADE7913DRIVER_H_
#define _ADE7913DRIVER_H_  

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
	
#define ADE7913_CS(x)		SYNC1_nCS(x)		//spi1
#define ADE7913_TIMEOUT_VALUE		0XFFFF
	//AD56xx quantity
#define ADE7913_NUMS          5
#define	ADE7913_N1		1
#define	ADE7913_N2		2
#define	ADE7913_N3		3
#define	ADE7913_N4		4
#define	ADE7913_N5		5
	
#define	ADE7913_N1_V1	1
#define	ADE7913_N1_V2	2
#define	ADE7913_N1_I	3

#define	ADE7913_N2_V1	1
#define	ADE7913_N2_V2	2
#define	ADE7913_N2_I	3

#define	ADE7913_N3_V1	1
#define	ADE7913_N3_V2	2
#define	ADE7913_N3_I	3

#define	ADE7913_N4_V1	1
#define	ADE7913_N4_V2	2
#define	ADE7913_N4_I	3

#define	ADE7913_N5_V1	1
#define	ADE7913_N5_V2	2
#define	ADE7913_N5_I	3

#define	ADE7913_N1_V1_MIN	35.5	
#define	ADE7913_N1_V2_MIN	34
#define	ADE7913_N1_I_MIN	34

#define	ADE7913_N2_V1_MIN	34
#define	ADE7913_N2_V2_MIN	34
#define	ADE7913_N2_I_MIN	34

#define	ADE7913_N3_V1_MIN	34
#define	ADE7913_N3_V2_MIN	34
#define	ADE7913_N3_I_MIN	34

#define	ADE7913_N4_V1_MIN	34
#define	ADE7913_N4_V2_MIN	34
#define	ADE7913_N4_I_MIN	34

#define	ADE7913_N5_V1_MIN	34
#define	ADE7913_N5_V2_MIN	34
#define	ADE7913_N5_I_MIN	34


#define	V1_MIN	34
#define	V2_MIN	41



	
#define   IWV           0x00
#define   V1WV          0x01
#define   V2WV          0x02
#define   ADC_CRC       0x04
#define   CTRL_CRC      0x05
#define   CNT_SNAPSHOT  0x07
#define   CONFIG        0x08
#define   STAUTS0       0x09
#define   LOCK          0x0A
#define   SYNC_SNAP     0x0B
#define   COUNTER0      0x0C
#define   COUNTER1      0x0D
#define   EMI_CTRL      0x0E
#define   STATUS1       0x0F
#define   TEMPOS        0x18
	
/** <! support chip type,selected one per time  */

typedef struct
{
    int Current;
    int Voltage1;
    int Voltage2;
    int ADC_CRCValue;
    int Status0;
    int CounterSnapshot; 
}OutputRegisters;

void  ADE7913WriteOperation(unsigned char Address, unsigned int SendValue);
int   ADE7913ReadOperation(unsigned char Address, unsigned char RegisterByteNumber);
OutputRegisters ADE7913BurstRead(void);

/* Exported functions -------------------------------------------------------*/
//AD5601 init
void ADE7913xx_Init(void);

//AD5601 set value output
void ADE7913_Comm_Set(uint8_t item, uint8_t mode, float vol);
	
		
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
