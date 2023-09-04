/**
  ******************************************************************************
  * @file           : SpiFlash.h
  * @brief          : Header for SpiFlash.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPIFLASH_H__
#define __SPIFLASH_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
 extern "C" {
#endif



#define W25QX_TIMEOUT_VALUE		2000
#define W25QX_SECTOR_SIZE 		 	4096
#define W25QX_BLOCK_SIZE 		 	4096*16


//#define W25QX_nCS(x)  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,x ? GPIO_PIN_SET : GPIO_PIN_RESET) 


void W25QX_Read_ID(uint8_t *ID);
unsigned char W25QX_Read_SR(void);
void W25QX_Write_SR(unsigned char ret);
void W25QX_Read_Data(uint8_t *data,uint32_t length,uint32_t address);
void W25QX_Fast_Read(uint8_t *data,uint32_t length,uint32_t address);
bool W25QX_Page_Program(uint8_t *data,uint16_t length,uint32_t address);
bool W25QX_Page_Write(uint8_t *data,uint16_t length,uint32_t address);
bool W25QX_Sector_Erase(uint32_t address);
bool W25QX_Block_Erase(uint32_t address);
bool W25QX_Chip_Erase(uint8_t *data,uint8_t length,uint32_t address);
void W25QX_Write_Enable(void);
bool W25QX_Write_Disable(void);


//h743 chip flash 
#define H743_FLASH_SIZE	(0x100000/2)
#define	APP_BASE_ADDR	0x08100000
#define	APP_BASE_NUM	0
#define H743_SECTION_SIZE (128*1024)
bool H743Chip_Sector_Erase(int bankNum,uint32_t sectionAddr,int sectionNum);
bool H743Chip_Data_Write(uint8_t *data,uint32_t length,uint32_t sectionAddress);
void H743Chip_Data_Read(uint8_t *data,uint32_t length,uint32_t address);


#ifdef __cplusplus
}
#endif

#endif /* __SPIFLASH_H__ */

/************************ (C) COPYRIGHT HJ *****END OF FILE****/

