#include "MS5637.h"


static uint16_t C_MS5637_02BA03[7];
static uint16_t C_MS5637_30BA[7];
static uint32_t D_MS5637_02BA03[2]={0};
static uint32_t D_MS5637_30BA[2]={0};


/*
	note: before use,execute reset
*/
bool HAL_MS5637_reset(MS5637TypeDef type)
{
	if(type == eMS5637_02BA03)
	{
		int i=1000;
		uint8_t tran[]={MS5637_RESET};
		do
		{
			if(HAL_I2C_Master_Transmit(MS5637_02BA03_HI2C,MS5637_ADDR,tran,1,0xFFF) == HAL_OK) break;
		}
		while (i--);
		if(i > 0)
		{
			return true;
		}
		else
		{
			return false;
		}		

	}
	else if(type == eMS5637_30BA)
	{
		int i=1000;
		uint8_t tran[]={MS5637_RESET};
		do
		{
			if(HAL_I2C_Master_Transmit(MS5637_30BA_HI2C,MS5637_ADDR,tran,1,0xFFF) == HAL_OK) break;
		}
		while (i--);
		if(i > 0)
		{
			return true;
		}
		else
		{
			return false;
		}		
	}
	else
	{
		return false;
	}
}

/*
	type,sensor typer
	note: after reset,execute prom crc,return true is success
*/

bool HAL_MS5637_prom_init(MS5637TypeDef type)
{
	uint16_t *C;
	uint8_t tran[1];
	uint8_t rec[2];
	I2C_HandleTypeDef *hi2c;
	int i,cnt;
	uint32_t n_rem=0;
	uint8_t n_bit;
	uint16_t CTEMP[8]={0};
//	uint16_t temp;
	n_rem=0;
	
	if(type == eMS5637_02BA03)
	{
		
		
		C = C_MS5637_02BA03;
		hi2c = MS5637_02BA03_HI2C;
	
	}
	else if(type == eMS5637_30BA)
	{
		C = C_MS5637_30BA;
		hi2c = MS5637_30BA_HI2C;		
	}
	else
	{
		return false;
	}
	/*read C0 .. C6*/
	for(i=0;i<7;i++)
	{
		tran[0] = MS5637_PROM+i*2;
		if(HAL_I2C_Master_Transmit(hi2c,MS5637_ADDR,tran,1,0xFFFF) == HAL_OK)
		{
			if(HAL_I2C_Master_Receive(hi2c,MS5637_ADDR,rec,2,0xFFFF) != HAL_OK)
			{
				return false;
			}
			CTEMP[i]=rec[0]<<8 | rec[1];
		}
		else
		{
			return false;
		}
	}
	C[0] = CTEMP[0];
	
	/*for CRC-4 calculation*/
	CTEMP[0] = CTEMP[0]&0x0FFF;
	for(cnt=0; cnt < 16; cnt++) // operation is performed on bytes
	{ // choose LSB or MSB
		if(cnt%2 )
		{
			n_rem ^= (unsigned short) (CTEMP[cnt>>1] & 0x00FF);
		}
		else
		{
			n_rem ^= (unsigned short) (CTEMP[cnt>>1]>>8);
		}
		for (n_bit = 8; n_bit > 0; n_bit--)
		{
			if (n_rem & (0x8000))
				n_rem = (n_rem << 1) ^ 0x3000;
			else 
				n_rem = (n_rem << 1);
		}
	}
	n_rem= ((n_rem >> 12) & 0x000F);
	if(((C[0]>>12)&0x000F) == n_rem) /*calcuation success*/
	{
		for(i=1;i<7;i++)
		{
			C[i] = CTEMP[i];
		}
		return true;
	}
	return false;
}

/*
 di set 0 D1,set 1 D2
*/
bool HAL_MS5637_read_adc(MS5637TypeDef type,uint8_t di)
{
	uint8_t DTEMP[3];
	uint8_t tran[]={MS5637_ADC_READ};
	if(type == eMS5637_02BA03)
	{
		if(HAL_I2C_Master_Transmit(MS5637_02BA03_HI2C,MS5637_ADDR,tran,1,0xFFFF) == HAL_OK)
		{
			if(HAL_I2C_Master_Receive(MS5637_02BA03_HI2C,MS5637_ADDR,DTEMP,3,0XFFFF) != HAL_OK)
			{
				return false;
			}			
			D_MS5637_02BA03[di?1:0] = DTEMP[0]<<16|DTEMP[1]<<8|DTEMP[2];
		}
		return true;
	}
	else if(type == eMS5637_30BA)
	{
		if(HAL_I2C_Master_Transmit(MS5637_30BA_HI2C,MS5637_ADDR,tran,1,0xFFFF) == HAL_OK)
		{
			if(HAL_I2C_Master_Receive(MS5637_30BA_HI2C,MS5637_ADDR,DTEMP,3,0XFFFF) != HAL_OK)
			{
				return false;
			}			
			D_MS5637_30BA[di?1:0] =  DTEMP[0]<<16|DTEMP[1]<<8|DTEMP[2];
		}
		return true;
	}
	else
	{
		return false;
	}	

}

/*
	note: before read adc ,execute OSR Conversion
*/

bool HAL_MS5637_OSR(MS5637TypeDef type,uint8_t osr)
{

	uint8_t tran[1]={osr};
	if(type == eMS5637_02BA03)
	{
		if(HAL_I2C_Master_Transmit(MS5637_02BA03_HI2C,MS5637_ADDR,tran,1,0xFFFF) == HAL_OK)
		{
			return true;
		}
		
	}
	else if(type == eMS5637_30BA)
	{
		if(HAL_I2C_Master_Transmit(MS5637_30BA_HI2C,MS5637_ADDR,tran,1,0xFFFF) == HAL_OK)
		{
			return true;
		}		
	}
	else
	{
		return false;
	}	
	return false;

}


bool HAL_MS5637_calcuate(MS5637TypeDef type,float* temp,float* press)
{
	uint16_t *C;
	uint32_t D1,D2;
	int dT,TEMP,P,Ti;
	int64_t SENS,OFF,OFFi,SENSi,OFF2,SENS2;
	if(type == eMS5637_02BA03)
	{
		C = C_MS5637_02BA03;
		D1 = D_MS5637_02BA03[0];
		D2 = D_MS5637_02BA03[1];
		// Terms called
		dT = D2-(uint32_t)(C[5])*256L;
		SENS = (int64_t)(C[1])*65536L+((int64_t)(C[3])*dT)/128L;
		OFF = (int64_t)(C[2])*131072L+((int64_t)(C[4])*dT)/64L;
				 
		//Temp and P conversion
		TEMP = 2000L+(int64_t)(dT)*C[6]/8388608LL;
		P = (D1*SENS/(2097152L)-OFF)/(32768L);

		//Second order compensation
		if((TEMP/100)<20)
		{ 				//Low temp
			Ti = (3*(int64_t)(dT)*(int64_t)(dT))/(8589934592LL);
			OFFi = (61*(TEMP-2000)*(TEMP-2000))/16;
			SENSi = (29*(TEMP-2000)*(TEMP-2000))/16;
			if((TEMP/100)<-15)
			{		//Very low temp
				OFFi = OFFi+17*(TEMP+1500L)*(TEMP+1500L);
				SENSi = SENSi+9*(TEMP+1500L)*(TEMP+1500L);
			}
		}
		else if((TEMP/100)>=20)
		{ 	 //High temp
			Ti = ((int64_t)(5*(dT*dT)))/(274877906944LL);
			OFFi = 0;
			SENSi = 0;
		}
			 
		OFF2 = OFF-OFFi;					 //Calculate pressure and temp second order
		SENS2 = SENS-SENSi;
		 
		TEMP = (TEMP-Ti);
		P = (((D1*SENS2)/2097152L-OFF2)/32768L);
		*temp = ((float)TEMP)/100.0f;
		*press = ((float)P)/100.0f;
		return true;
	}
	else if(type == eMS5637_30BA)
	{
		C = C_MS5637_30BA;
		D1 = D_MS5637_30BA[0];
		D2 = D_MS5637_30BA[1];
		
		// Terms called
		dT = D2-(uint32_t)(C[5])*256L;
		SENS = (int64_t)(C[1])*32768L+((int64_t)(C[3])*dT)/256L;
		OFF = (int64_t)(C[2])*65536L+((int64_t)(C[4])*dT)/128L;
				 
		//Temp and P conversion
		TEMP = 2000L+(int64_t)(dT)*C[6]/8388608LL;
		P = (D1*SENS/(2097152L)-OFF)/(8192L);

		//Second order compensation
		if((TEMP/100)<20)
		{ 				//Low temp
			Ti = (3*(int64_t)(dT)*(int64_t)(dT))/(8589934592LL);
			OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
			SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
			if((TEMP/100)<-15)
			{		//Very low temp
				OFFi = OFFi+7*(TEMP+1500L)*(TEMP+1500L);
				SENSi = SENSi+4*(TEMP+1500L)*(TEMP+1500L);
			}
		}
		else if((TEMP/100)>=20)
		{ 	 //High temp
			Ti = ((int64_t)(2*(dT*dT)))/(137438953472);
			OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
			SENSi = 0;
		}
			 
		OFF2 = OFF-OFFi;					 //Calculate pressure and temp second order
		SENS2 = SENS-SENSi;
		 
		TEMP = (TEMP-Ti);
		P = (((D1*SENS2)/2097152L-OFF2)/8192L);
		*temp = ((float)TEMP)/100.0f;
		*press = ((float)P)/10.0f;
		return true;
	}
	else
	{
		return false;
	}
	

}

///////////////////////////////////驱动无关的 app部分//////////////////////////////////////////////////////////

PressDataTypeDef gSensorData2 = {0};
void DataCollectInit(void )
{
	/*intial sensor*/
	gSensorData2.InnerStatus |= SENSOR2_CONFIG;
	if(gSensorData2.InnerStatus & SENSOR2_CONFIG)
	{
		if(HAL_MS5637_reset(eMS5637_02BA03))
		{
			gSensorData2.InnerStatus |= SENSOR2_RESET_SUCCESS;
			if(HAL_MS5637_prom_init(eMS5637_02BA03))
			{
				gSensorData2.InnerStatus |= SENSOR2_INIT_SUCCESS;
			}
			else
			{
				gSensorData2.InnerStatus |= SENSOR2_NO_REPONSE;
			}
		}
		else
		{
			gSensorData2.InnerStatus |= SENSOR2_NO_REPONSE;
		}
	}
}

/*数据读取*/
void DataCollectExecute(void )
{
	float t1,t2;

	/*inner OSR D1*/
 	if((gSensorData2.InnerStatus&SENSOR2_SUCCESS_MSK) == SENSOR2_SUCCESS_MSK)
	{
		if((gSensorData2.InnerStatus&SENSOR2_NO_REPONSE) == 0)
		{
			if(HAL_MS5637_OSR(eMS5637_02BA03,MS5637_D1_OSR_8192)==false)
			{
				gSensorData2.InnerStatus |= SENSOR2_NO_REPONSE;
			}
		}
	}


	/*convert delay */
	osDelay(OSR_8192_CONVERSION_TIME);

	/*inner OSR D2*/
	if((gSensorData2.InnerStatus&SENSOR2_SUCCESS_MSK) == SENSOR2_SUCCESS_MSK)
	{
		if((gSensorData2.InnerStatus&SENSOR2_NO_REPONSE) == 0)
		{
			if(HAL_MS5637_read_adc(eMS5637_02BA03,0))
			{
				if(HAL_MS5637_OSR(eMS5637_02BA03,MS5637_D2_OSR_8192)==false)
				{
					gSensorData2.InnerStatus |= SENSOR2_NO_REPONSE;
				}
			}
			else
			{
				gSensorData2.InnerStatus |= SENSOR2_NO_REPONSE;
			}
		}
	}

	/*convert delay*/
	osDelay(OSR_8192_CONVERSION_TIME);
	
	/*read adc d2 , calcuate inner*/
	if((gSensorData2.InnerStatus&SENSOR2_NO_REPONSE) == 0)
	{
		if(HAL_MS5637_read_adc(eMS5637_02BA03,1))
		{
			HAL_MS5637_calcuate(eMS5637_02BA03,&t1,&t2);
			gSensorData2.InnerTemp = t1;
			gSensorData2.InnerPressure = t2;
		}
		else
		{
			gSensorData2.InnerStatus |= SENSOR2_NO_REPONSE;
		}
	}
	
}



