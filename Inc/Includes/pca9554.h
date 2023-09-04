/*
 * pca9554.h
 *
 *  Created on: May 18, 2022
 *      Author: Administrator
 */

#ifndef SRC_USR_INCLUDES_PCA9554_H_
#define SRC_USR_INCLUDES_PCA9554_H_

#include "main.h"
#include "stdbool.h"

#define PCA554A_ADDR          		0x70 //address

#define CONFIGURATION_REGISTER			0X03      //配置寄存器
#define POLARITY_INVERSION_REGISTER		0X02      //奇偶反转寄存器
#define OUTPUT_REGISTER					0X01      //输出寄存器
#define INPUT_REGISTER					0X00      //输入寄存器


#define DIR_IN	0	//输入继电器
#define DIR_OUT	1	//输出继电器
#define sON	1		//打开
#define sOFF	0	//关闭

#define IO_CLEAR_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)&= ~(__FLAG__))
#define IO_EN_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)|= (__FLAG__))

bool HAL_PCA9554_init(void);
bool HAL_PCA9554_outputAll(uint8_t data);
bool set_sampleMode(uint8_t mode);
bool set_channle_status(uint8_t channel,uint8_t dir,uint8_t status);
#endif /* SRC_USR_INCLUDES_PCA9554_H_ */
