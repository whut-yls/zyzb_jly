#include "AHT15.h"
#include "main.h"


bool HAL_AHT15_software_reset(void)
{	
	#if 1
	uint8_t tran[]={AHT15_RESET};
//	if(HAL_I2C_Master_Transmit(&hi2c1,AHT15_ADDR,tran,1,0xFff)==HAL_OK)		//no need hi2c2
	if(HAL_I2C_Master_Transmit(&hi2c2,AHT15_ADDR,tran,1,0xFff)==HAL_OK)		//no need hi2c2   //debugi2c
		return true;
	else
		return false;
	#endif
}

/* 
	measure flow: SEND AC -> delay 75MS -> MeasureHumidity
*/

/**/

bool HAL_AHT15_SEND_AC(void)
{
	uint8_t tran[]={AHT15_SEND_AC,0X33,0X00};
	if(HAL_I2C_Master_Transmit(&hi2c2,AHT15_ADDR,tran,3,0xfff)==HAL_OK)
		return true;
	else
		return false;
}


bool HAL_AHT15_INIT(void)
{
	uint8_t tran[]={AHT15_INIT,0X08,0X00};

	if(HAL_I2C_Master_Transmit(&hi2c2,AHT15_ADDR,tran,3,0xFff)==HAL_OK)
		return true;
	else
		return false;
}



SensorType gSensorData1;

bool sensor_init(void)
{
	//重置初始化状态
	gSensorData1.RHStatus=0;
	
	if(HAL_AHT15_software_reset())
	{
		osDelay(AHT15_SOFTWARE_RESET_TIME);
		gSensorData1.RHStatus |= SENSOR_RESET_SUCCESS;
		if(HAL_AHT15_INIT())
		{
			gSensorData1.RHStatus |= SENSOR_INIT_SUCCESS;
			osDelay(50);	//change 500-50
		}
		else
		{
			gSensorData1.RHStatus |= SENSOR_NO_REPONSE;
			HAL_I2C_MspDeInit(AHT15__HI2C);
			MX_I2C2_Init();
		}
	}
	else
	{
		gSensorData1.RHStatus |= SENSOR_NO_REPONSE;
		HAL_I2C_MspDeInit(AHT15__HI2C);
		MX_I2C2_Init();
	}

	return true;
}

/*********-------------------------*/



