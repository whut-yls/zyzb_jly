/**
  ******************************************************************************
  * @file           : dac8831.h
  * @brief          : Header for dac8831.c file.
  *                   This file contains the common defines of the application
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DAC8831_H
#define __DAC8831_H

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/
//#include "stm32f4xx_hal.h"
//#include "stdio.h"
#include "main.h"
/* Private includes ----------------------------------------------------------*/
	
#define DAC8831_A_CS(x)		SYNC1_nCS(x)		//spi4
#define DAC8831_B_CS(x)		SYNC2_nCS(x)		//spi4
#define DAC8831_TIMEOUT_VALUE		0XFFFF
	
#define	DAC8831_A	0x01
#define	DAC8831_B	0x02
	
	
static HAL_DMA_MuxSyncConfigTypeDef dmamux_syncParams;


/* Exported functions -------------------------------------------------------*/

//dac8831 set value output
void Wave_select(uint8_t n, uint16_t *_pBuf);
void Dac8831_Set_Amp(float amp, uint16_t *_pBuf);
void DAC8831_Set_Data(uint16_t data);
void DAC8831_Set_Data_Dma(uint16_t *_pbufch1, uint32_t _sizech1,uint32_t _ulFreq);
void Dac_level_CTL(uint8_t work_mode);

void DAC8831_SetCS(uint8_t _flag);
uint16_t changeDac8831Vol_2_De(float Val);

void MakeSquareTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop);
void MakeSinTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop);
void MakeRectangularTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop, float _ratio);
void MakeTriangularTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop);
void MakeSharpTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop);
void MakeSawtoothTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop);
void MakeExponentTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop);
void MakeTrapezoidTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop);
void MakeSectorTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop);
void MakePulseTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop,float _ratio);
void MakeSparseTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop);
void MakeDenseTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop);

void MakeSparse_DenseTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop);
void MakeIntermittentTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop);
void MakeContinuityTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop);
#define	SPI_BUFFER_SIZE		(4 * 1024)	
		
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
