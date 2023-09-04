#ifndef __I2C_H
#define __I2C_H

#include "main.h" 





#define Page_Size 16     //һҳ�洢���ֽ��������忴��Ӧ������datasheet

void IIC_SDA_IN(void);   //IO���������������
void IIC_SDA_OUT(void);
	
void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Write_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(void);
uint8_t Write_Device_Addr(uint8_t addr);
void Search_Device_Addr(void);
uint8_t Device_Read_One_Byte(uint8_t IC_Addr,uint8_t Addr);
void Device_Write_One_Byte(uint8_t IC_Addr,uint8_t Addr,uint8_t data);
void Device_Read_Bytes(uint8_t IC_Addr,uint8_t Addr,uint8_t *buf,uint8_t len);
void Device_Write_Bytes(uint8_t IC_Addr,uint8_t Addr,uint8_t *buf,uint8_t len);
void Device_Page_Write(uint8_t IC_Addr,uint8_t Addr,uint8_t *buf,uint8_t len);
void delay_us(uint32_t nus);

#define IIC_SCL(X) HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,X)  //ģ�����ʱ������
#define IIC_SDA(X) HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,X)  //�����߶������

#define READ_SDA HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_0)   //�����߶�������
#endif

