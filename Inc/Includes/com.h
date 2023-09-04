/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    com.h
 * @brief   This file contains the headers of the com.
 ******************************************************************************
 *
 * COPYRIGHT(c) 2019 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COM_H
#define __COM_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "main.h"
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define	HARDWARE_VER	100	
#define SOFTWARE_VER	100	

#define	CMD_BASE_LEN 6
#define	DATA_HEAD	0x8a
#define	DATA_END	0x8f

#define	CMD_TYPE_DPJ	0x80
#define	CMD_TYPE_PC	0x81

#define CMD_BOARD_REF   0x03    //获取参考电压的adc值
#define CMD_BOARD_LINK  0x04  //板连接状态
#define CMD_IO_STATUS   0x05  //读记录的继电器状态
#define CMD_INPUT_CLOSE 0x06  //采集输入通道关闭
#define CMD_INPUT_OPEN  0x07  //采集输入通道选通

#define	CMD_BOARD_TEST	0x01  //板通信测试
#define	CMD_CLOSE_VOL	0x08
#define	CMD_OUTPUT_CLOSE	0x09	//关闭某个通道 输出端
#define	CMD_OUTPUT_OPEN	0x10		//打开某个通道 输出端
#define	CMD_SELECT_VOL	0x11
#define	CMD_READ_ADC	0x12	
#define	CMD_READ_BOARD_ADC	0x13
#define CMD_BUZZ_LEVEL  0x14
#define CMD_SENSOR_DATA  0x15	

#define CMD_SENSOR_RESISTOR  0x16		//可调电阻调整
#define CMD_WORK_MODE  0x20	   //设置经络仪工作模式（1整体采样、2局部采样、3治疗模式、4 断开采样和治疗信号）
#define CMD_IO_CLOSE			  0x22		//关闭所有小板的输入、输出通道
#define CMD_FREQUENCY_CHANGE			  0x23		//改变ad9833频率和波形

#define	SUCCESS	0x01
#define	FAIL	0x0

#define RETURN_LEN_ADC  19
#define RETURN_LEN_STATUS 5 
#define RETURN_LEN_ACK 4 

#define DATA_PC	1
#define DATA_4G	2
#define DATA_NET	3

#pragma pack(1) 
typedef struct __COMMON_RETURN {
	uint8_t cmd_type;
	uint8_t cmd_num;
	uint8_t cmd_status;
} CommonReturn;

/* USER CODE END EFP */
void Console_Handle(uint8_t *src, uint16_t len,uint8_t com);
HAL_StatusTypeDef HAL_UART_Transmit_Lcd( uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit_Wifi( uint8_t *pData, uint16_t Size, uint32_t Timeout);
#ifdef __cplusplus
}
#endif

#endif /* __COM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
