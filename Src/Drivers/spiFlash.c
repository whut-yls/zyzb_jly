#include "spiFlash.h"
#include "stm32h7xx_hal.h"
#include "main.h"

const unsigned char Flash_Read_ID[] = {0x90,0x00,0x00,0x00};
const unsigned char Flash_Read_SR[] = {0x05};
const unsigned char Flash_WRITE_SR[] = {0x01};
const unsigned char Flash_Read_DATA[] = {0x03,0x00,0x00,0x00};
const unsigned char Flash_FAST_Read[] = {0x0B,0x00,0x00,0x00,0x00};
const unsigned char Flash_PAGE_PROGRAM[] = {0x02,0x00,0x00,0x00};
const unsigned char Flash_SECTOR_ERASE[] = {0x20,0x00,0x00,0x00};
const unsigned char Flash_BLOCK_ERASE[] = {0xD8,0x00,0x00,0x00};
const char Flash_CHIP_ERASE[] = {0xC7};
const char Flash_WRITE_ENABLE[] = {0x06};
const char Flash_WRITE_DISABLE[] = {0x04};
/**
	@brief Read Manufacture Device ID
	@param ID return vaule address
*/
void W25QX_Read_ID(uint8_t *ID)
{
	unsigned char send[sizeof(Flash_Read_ID)];
	int i =0;
	for(i=0;i<sizeof(Flash_Read_ID);i++) {send[i] = Flash_Read_ID[i];}
	W25QX_nCS(0);
	HAL_SPI_Transmit(&hspi5,&send[0],sizeof(send),W25QX_TIMEOUT_VALUE);
	HAL_SPI_Receive(&hspi5,ID,2,W25QX_TIMEOUT_VALUE);
	W25QX_nCS(1);
}

/**
		@brief Read Status Registers  SRP|SEC|TB|BP2|BP1|BP0|WEL|BUSY
*/
unsigned char W25QX_Read_SR(void)
{
	unsigned char ret;
	unsigned char send[sizeof(Flash_Read_SR)];
	int i =0;
	for(i=0;i<sizeof(Flash_Read_SR);i++) {send[i] = Flash_Read_SR[i];}
	W25QX_nCS(0);
	HAL_SPI_Transmit(&hspi5,&send[0],sizeof(send),W25QX_TIMEOUT_VALUE);
	HAL_SPI_Receive(&hspi5,&ret,1,W25QX_TIMEOUT_VALUE);
	W25QX_nCS(1);
	return ret;
}


/**
		@brief write Status Registers  SRP|SEC|TB|BP2|BP1|BP0|WEL|BUSY
*/
void W25QX_Write_SR(unsigned char ret)
{
	unsigned char send[sizeof(Flash_WRITE_SR)];
	int i =0;
	for(i=0;i<sizeof(Flash_WRITE_SR);i++) {send[i] = Flash_WRITE_SR[i];}
	W25QX_nCS(0);
	HAL_SPI_Transmit(&hspi5,&send[0],sizeof(send),W25QX_TIMEOUT_VALUE);
	HAL_SPI_Transmit(&hspi5,&ret,1,W25QX_TIMEOUT_VALUE);
	W25QX_nCS(1);
}

/**
	@brief Read data
	@param data return read data
	@length read data length
	@address flash address
*/
void W25QX_Read_Data(uint8_t *data,uint32_t length,uint32_t address)
{
	unsigned char send[sizeof(Flash_Read_DATA)];
	unsigned char i,j;
	j=sizeof(Flash_Read_DATA)-1;
	send[0] = Flash_Read_DATA[0];
	for(i=0;i < j;i++)
	{
		send[1+i] = address>>(16-i*8);
	}
	W25QX_nCS(0);
	HAL_SPI_Transmit(&hspi5,&send[0],sizeof(send),W25QX_TIMEOUT_VALUE);
	HAL_SPI_Receive(&hspi5,data,length,W25QX_TIMEOUT_VALUE);
	W25QX_nCS(1);
}

/**
	@brief fast Read data
	@param data return read data
	@length read data length
	@address flash address
*/
void W25QX_Fast_Read(uint8_t *data,uint32_t length,uint32_t address)
{
	unsigned char send[sizeof(Flash_FAST_Read)];
	unsigned char i,j;
	j=sizeof(Flash_FAST_Read)-2;
	send[0] = Flash_FAST_Read[0];
	for(i=0;i < j;i++)
	{
		send[1+i] = address>>(16-i*8);
	}
	W25QX_nCS(0);
	HAL_SPI_Transmit(&hspi5,&send[0],sizeof(send),W25QX_TIMEOUT_VALUE);
	HAL_SPI_Receive(&hspi5,data,length,W25QX_TIMEOUT_VALUE);
	W25QX_nCS(1);
}

/**
	@brief page program
	@param data return read data
	@length read data length max 256 bytes
	@address flash address
*/
bool W25QX_Page_Program(uint8_t *data,uint16_t length,uint32_t address)
{
	unsigned char send[sizeof(Flash_PAGE_PROGRAM)];
	unsigned char i,j;
	int t;
	j=sizeof(Flash_PAGE_PROGRAM)-1;
	send[0] = Flash_PAGE_PROGRAM[0];
	for(i=0;i < j;i++)
	{
		send[1+i] = address>>(16-i*8);
	}
	W25QX_Write_Enable();
	W25QX_nCS(0);
	HAL_SPI_Transmit(&hspi5,&send[0],sizeof(send),W25QX_TIMEOUT_VALUE);
	HAL_SPI_Transmit(&hspi5,data,length,W25QX_TIMEOUT_VALUE);
	W25QX_nCS(1);
	for(t=0;t<100000;t++)
	{
		if((W25QX_Read_SR()&0x01) == 0) 
		{
			return true;
		}
	}
	return false;
}


/**
	@brief entry page program
	@param data return read data
	@length read data length
	@address flash address
*/
bool W25QX_Page_Write(uint8_t *data,uint16_t length,uint32_t address)
{
	unsigned char i=0,j;
	int t;
	if(length > W25QX_SECTOR_SIZE) return false;

	j = length/256;
	

	for(i=0;i<j;i++)
	{
		if(W25QX_Page_Program(data+i*256,256,address+256*i) == false)
			return false;
	}
	t=length%256;
	if(t) 
	{
		if(W25QX_Page_Program(data+i*256,t,address+256*i) == false)
			return false;
	}
	
	return true;

}


/**
	@brief sector erase
	@address flash address
*/
bool W25QX_Sector_Erase(uint32_t address)
{
	unsigned char send[sizeof(Flash_SECTOR_ERASE)];
	unsigned char i,j;
	int t;
	j=sizeof(Flash_SECTOR_ERASE)-1;
	send[0] = Flash_SECTOR_ERASE[0];
	for(i=0;i < j;i++)
	{
		send[1+i] = address>>(16-i*8);
	}
	W25QX_Write_Enable();
	W25QX_nCS(0);
	HAL_SPI_Transmit(&hspi5,&send[0],sizeof(send),W25QX_TIMEOUT_VALUE);
	W25QX_nCS(1);
	for(t=0;t<100000;t++)
	{
		if((W25QX_Read_SR()&0x01) == 0) 
		{
			return true;
		}
	}
	return false;
}


/**
	@brief block erase

	@address flash address
*/
bool W25QX_Block_Erase(uint32_t address)
{
	unsigned char send[sizeof(Flash_BLOCK_ERASE)];
	unsigned char i,j;
	int t;
	j=sizeof(Flash_BLOCK_ERASE)-1;
	send[0] = Flash_BLOCK_ERASE[0];
	for(i=0;i < j;i++)
	{
		send[1+i] = address>>(16-i*8);
	}
	W25QX_Write_Enable();
	W25QX_nCS(0);
	HAL_SPI_Transmit(&hspi5,&send[0],sizeof(send),W25QX_TIMEOUT_VALUE);
	W25QX_nCS(1);
	for(t=0;t<100000;t++)
	{
		if((W25QX_Read_SR()&0x01) == 0) 
		{
			return true;
		}
	}
	return false;
}

/**
	@brief page program
	@param data return read data
	@length read data length max 256 bytes
	@address flash address
*/
bool W25QX_Chip_Erase(uint8_t *data,uint8_t length,uint32_t address)
{
	unsigned char send[sizeof(Flash_CHIP_ERASE)];
	int i =0;
	for(i=0;i<sizeof(Flash_CHIP_ERASE);i++) {send[i] = Flash_CHIP_ERASE[i];}
	W25QX_Write_Enable();
	W25QX_nCS(0);
	HAL_SPI_Transmit(&hspi5,&send[0],sizeof(send),W25QX_TIMEOUT_VALUE);
	W25QX_nCS(1);
	for(i=0;i<100000;i++)
	{
		if((W25QX_Read_SR()&0x01) == 0) 
		{
			return true;
		}
	}
	return false;
}

/**
	@brief write enable instruction

*/
void W25QX_Write_Enable(void)
{
	unsigned char send[sizeof(Flash_WRITE_ENABLE)];
	int i =0;
	for(i=0;i<sizeof(Flash_WRITE_ENABLE);i++) {send[i] = Flash_WRITE_ENABLE[i];}
	W25QX_nCS(0);
	HAL_SPI_Transmit(&hspi5,&send[0],sizeof(send),W25QX_TIMEOUT_VALUE);
	W25QX_nCS(1);

}

/**
	@brief write disable instruction
*/
bool W25QX_Write_Disable(void)
{
	unsigned char send[sizeof(Flash_WRITE_DISABLE)];
	int i =0;
	for(i=0;i<sizeof(Flash_WRITE_DISABLE);i++) {send[i] = Flash_WRITE_DISABLE[i];}
	W25QX_nCS(0);
	HAL_SPI_Transmit(&hspi5,&send[0],sizeof(send),W25QX_TIMEOUT_VALUE);
	W25QX_nCS(1);
	return true;
}

/**
 * stm32h743 falsh: 2bank*8扇区*128k(扇区尺寸)
 * FLASH_BANK_1(0x0800 0000~0x080F FFFF)  FLASH_BANK_2(0x0810 0000~0x081F FFFF)
 * **/

/**
	功能：片内flash扇区擦除 
	参数：bankNum:bank 号  sectionStart:扇区起始地址 sectionNum:扇区个数
	成功：true
	失败：false
*/
bool H743Chip_Sector_Erase(int bankNum,uint32_t sectionAStart,int sectionNum)
{
	int status;
	uint32_t sectERR;
	FLASH_EraseInitTypeDef EraseInit;
	EraseInit.Banks=bankNum;
	EraseInit.TypeErase=FLASH_TYPEERASE_SECTORS;
	EraseInit.Sector=sectionAStart;//0~7 sector
	EraseInit.NbSectors=sectionNum;
	EraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_1;	//8 16 32 64

	if(FLASH_BANK_1==bankNum)
	{
		HAL_FLASHEx_Unlock_Bank1();
		status=HAL_FLASHEx_Erase(&EraseInit,&sectERR);
		HAL_FLASHEx_Lock_Bank1();
	}else if(FLASH_BANK_2==bankNum){
		HAL_FLASHEx_Unlock_Bank2();
		status=HAL_FLASHEx_Erase(&EraseInit,&sectERR);
		HAL_FLASHEx_Lock_Bank2();
	}else{
		return false;
	}

	if(HAL_OK==status)
	{
		return true;
	}else{
		return false;
	}
}


/*

*/
bool H743Chip_Data_Write(uint8_t *data,uint32_t length,uint32_t sectionAddress)
{
	unsigned char i,j;
	int status;
	int t;
	if(length >H743_FLASH_SIZE) return false;

	j = length/32;

	for(i=0;i<j;i++)
	{
		HAL_FLASHEx_Unlock_Bank2();
		status=HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, sectionAddress+32*i,(uint32_t)data+i*32);
		HAL_FLASHEx_Lock_Bank2();
		if(HAL_OK!=status)
		{
			return false;
		}
	}

	t=length%32;
	if(t) 
	{
		HAL_FLASHEx_Unlock_Bank2();
		status=HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, sectionAddress+32*i,(uint32_t)data+i*32);
		HAL_FLASHEx_Lock_Bank2();
		if(HAL_OK!=status)
		{
			return false;
		}
	}
	
	return true;
}

void H743Chip_Data_Read(uint8_t *data,uint32_t length,uint32_t address)	
{
	uint32_t dataValue;
	uint32_t  num;
	for(num=0;num<length;num++)
	{
		dataValue=*(__IO uint8_t*)address;	//*(__IO uint32_t*)
		data[num]=dataValue;
		address++;
	}
	return ;
}
