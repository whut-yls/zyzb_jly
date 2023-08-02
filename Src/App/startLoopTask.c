#include "stdio.h"
#include "string.h"
#include "stdio.h"
/* default header*/
#include "StringTransform.h"
#include "startLoopTask.h"
#include "com.h"

#include "common.h"
#include "myAdc.h"

#include "ad9833.h"

#include "AHT15.h"
#include "MS5637.h"

bool gLedRun=false;
static void run_led(void)
{
	if(gLedRun==false)
	{
		WriteOUTRun(GPIO_PIN_SET);
		gLedRun=true;
	}else{
		WriteOUTRun(GPIO_PIN_RESET);
		gLedRun=false;
	}
}

/*循环任务*/
void startLoopTask(void const * argument)
{
	/*led 计数*/
	uint16_t ledCnt=0,pingCnt=0;
	/*温湿度相关定义*/
	uint8_t temp[8]={0x00};
	uint32_t t1,f1;
	uint8_t SensorCnt1=0,SensorCnt2=0;
	uint8_t Key_input=0;
	/*adc校正--f4无adc校准*/
	//HAL_ADCEx_Calibration_Start(&hadc1);
	
	/*输出dac为0v DAC PA4*/
//	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
//	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	
//	/*初始化频率和波形10k*/
//	AD9833_WaveSeting(10000,0,SIN_WAVE,0);
//	
//	/*初始化幅度值,未初始化时，芯片默认为0x80*/
//	AD9833_AmpSet(0xff); //设置幅值，幅值最大 255
	
	/*开始adc dma循环采集*/
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)gADC1_DMA_BUF,N_NUM* AD1_NUM);
	

	/*气压传感器初始化*/
	sensor_init();
	/*电池采集传感器初始化*/
	/*温湿度传感器初始化*/
	DataCollectInit();
	while(1)
	{
		
		Key_input=key_scan();
		if(Key_input){
			osDelay(10);  //防止一直按按键
			switch(Key_input)
			{
				case 1: 
						if(gGlobalData.curWorkState==WORK_PAUSE||gGlobalData.curWorkState==WORK_STOP){
								do_work_ctl(1);
						}
						else{
								do_work_ctl(2);
						}
						cnt_heartbag = 0;												//发送心跳清空心跳计数器
						gGlobalData.heartbag_flage = 1; 						
					break;
				case 2: 
						Send_Fix_Ack(35,STATUS_OK,"Request CALL!");
					break;
				case 3: 
						do_work_ctl(4);  //加档
						cnt_heartbag = 0;												//发送心跳清空心跳计数器	
						gGlobalData.heartbag_flage = 1; 				
					break;
				case 4: 
					 do_work_ctl(5);  //减档
					 cnt_heartbag = 0;												//发送心跳清空心跳计数器
					 gGlobalData.heartbag_flage = 1; 	
				break;
				default: 
					break;
			
			}

		}
		/*温度湿度采集*/
		if(SensorCnt1==0||SensorCnt1>=10)	//3s
		{
				SensorCnt1=1;
				HAL_I2C_MspDeInit(&hi2c2);	//此处不能去初始化，否则i2c异常，所以此函数内部实际被屏蔽
				MX_I2C2_Init();
					/*RH measure humidity no hold*/
				if((gSensorData1.RHStatus&SENSOR_SUCCESS_MSK) == SENSOR_SUCCESS_MSK)
				{
					if((gSensorData1.RHStatus&SENSOR_NO_REPONSE) == 0)
					{
						if(HAL_AHT15_SEND_AC() ==false)
						{
							gSensorData1.RHStatus |= SENSOR_NO_REPONSE;
							HAL_I2C_MspDeInit(&hi2c2);
							MX_I2C2_Init();
						}
					}
				}
				osDelay(100);		

				/*RH measure humidity no hold read  */
				if((gSensorData1.RHStatus&SENSOR_NO_REPONSE) == 0)
				{
					if(HAL_I2C_Master_Receive(&hi2c2, AHT15_ADDR, temp, 6, 0xFFF)==HAL_OK)
					{
						if((temp[0]&0x80)==0)
						{

								f1= ((temp[1]<<16)|(temp[2]<<8)|temp[3])>>4;
								t1= ((temp[3]&0x0f)<<16)|(temp[4]<<8)|temp[5];
								gSensorData1.RH=((float)f1)/1048576.0*100;
								gSensorData1.RHt=((float)t1)/1048576.0*200.0-50.0;

						}
					
					}
					else
					{
						gSensorData1.RHStatus |= SENSOR_NO_REPONSE;
						HAL_I2C_MspDeInit(&hi2c2);
						MX_I2C2_Init();
					}
				}
				
				//若无应答 重新初始化
				if((gSensorData1.RHStatus&SENSOR_NO_REPONSE) != 0)
				{
						sensor_init();
				}
		
		}
		#if 1
		/****气压传感器**/
		if(SensorCnt2==0||SensorCnt2>=10) //3s
		{
			SensorCnt2=1;
			if((gSensorData2.InnerStatus&SENSOR2_NO_REPONSE)&&(gSensorData2.InnerStatus & SENSOR2_CONFIG))
			{
				if(HAL_MS5637_reset(eMS5637_02BA03))
				{
					gSensorData2.InnerStatus |= SENSOR_RESET_SUCCESS;
					if(HAL_MS5637_prom_init(eMS5637_02BA03))
					{
						gSensorData2.InnerStatus |= SENSOR_INIT_SUCCESS;
						gSensorData2.InnerStatus &= ~SENSOR_NO_REPONSE;
					}
					else
					{
						gSensorData2.InnerStatus |= SENSOR_NO_REPONSE;
						gSensorData2.InnerStatus &= ~SENSOR_INIT_SUCCESS;
					}
				}
				else
				{
					HAL_I2C_MspDeInit(&hi2c3);
					MX_I2C3_Init();
					gSensorData2.InnerStatus |= SENSOR_NO_REPONSE;
					gSensorData2.InnerStatus &= ~SENSOR_RESET_SUCCESS;
				}

			}
			DataCollectExecute();
		}
		#endif
		/*电量采集*/
		/*****/
		
		/*led运行控制*/
		if(ledCnt>=5)
		{
			ledCnt=0;
			run_led();
			
			
		}
		if(pingCnt>=20)
		{
			pingCnt=0;
			gGlobalData.Send_Ping_Task=true;
		}
//		
		SensorCnt1++;
		SensorCnt2++;
		ledCnt++;
		pingCnt++;
		osDelay(500);
	
	}//while
  /* USER CODE END 5 */ 
}


uint8_t key_scan(void)
{
	static uint8_t key_up = 1;//按键没有被按下
	if(key_up&&(ReadINPUT1==0)||(ReadINPUT2==0)||(ReadINPUT3==0)||(ReadINPUT4==0))
	{
		osDelay(10);//延时，防止按键抖动带来误判
		key_up = 0;
			if(ReadINPUT1==0)
			{
				return 1;//返回1代表IN1拉低  停止或者启动
			}
			else if(ReadINPUT2==0)
			{
				return 2;//返回2代表IN2拉低  呼叫
			}
			else if(ReadINPUT3==0)
			{
				return 3;//返回3代表IN3拉低  加档
			}
			else if(ReadINPUT4==0)
			{
				return 4;//返回4代表IN4拉低  减档
			}
	}
	else if(ReadINPUT1==1 && ReadINPUT2==1 && ReadINPUT3==1 && ReadINPUT4==1)key_up=1;//按键没被判定按下
	return 0;
}


