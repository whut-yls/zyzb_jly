/*
 * pca9554.c
 *
 *  Created on: 2022年5月18日
 *      Author: Administrator
 */

#include "pca9554.h"
#include "cmsis_os.h"
#include "main.h"

bool HAL_PCA9554_init(void)
{
	uint8_t send[2]={0x03,0x00};

	for(int i=0;i<8;i++){
		if(HAL_I2C_Master_Transmit(&hi2c2,PCA554A_ADDR+(i*2),send,sizeof(send),0xFff)!=HAL_OK){
		return false;
		}
		osDelay(50);

	}
	return true;

}

//读取输出寄存器先前的值
bool HAL_PCA9554_readIn(uint8_t adr,uint8_t* data)
{
	uint8_t send[1]={0x01};
	uint8_t oldVal=0;
	if(HAL_I2C_Master_Transmit(&hi2c2,adr,send,1,0xFff) == HAL_OK)
	{
		if(HAL_I2C_Master_Receive(&hi2c2,adr,&oldVal,1,0xFFF) != HAL_OK)
		{
			return false;
		}
		*data=oldVal;
	}
	else
	{
		return false;
	}
	return true;
}

//设置所有通道的继电器状态
bool HAL_PCA9554_outputAll(uint8_t data)
{
	uint8_t send[2]={0x01,data};

	for(int i=0;i<8;i++){
		if(HAL_I2C_Master_Transmit(&hi2c2,PCA554A_ADDR+i*2,send,sizeof(send),0xFff)!=HAL_OK){
		return false;
		}
		osDelay(50);
	}
	return true;
}

//更新IO寄存器值
void update_oldValue(uint8_t sNum,uint8_t dir,uint8_t status,uint8_t *oldVal)
{
	uint8_t bitNum,data=0;
	data=*oldVal;
	if(dir==DIR_OUT)
	{
		sNum=sNum;
	}else{
		sNum=sNum+1;
	}
	//打开或关闭设置
	bitNum=1<<sNum;
	if(status==sON)
	{
		IO_EN_FLAG(data,bitNum);
	}else{	//sOFF
		IO_CLEAR_FLAG(data,bitNum);
	}
	*oldVal=data;
	return;
}

//配置4个小板的通道状态
//channel:通道号0~32 dir:某通道的输入\输出继电器选择 status:0 关闭 1打开
bool set_channle_status(uint8_t channel,uint8_t dir,uint8_t status)
{
	uint8_t address=0,addressR=0;
	uint8_t oldVal=0,sNum=0;
	
	uint8_t send[2]={0x01,0x00};
	
	//读取先前的继电器状态
	addressR=address|0x01;
	
	if(channel>32)
		return false;
	
	if(channel==0)	//所有通道
	{
		HAL_PCA9554_outputAll(0);
	}else{	//指定通道
//		if(HAL_PCA9554_readIn(address,&oldVal)==true)
//		{
//		}else{
//			oldVal=0;
//		}

		//地址查找
		if(channel>0&&channel<=4)
		{
			//地址设置
			address=0x7a;
			//找到属于小板上的那一组
			channel=channel-0;
			channel-=1;	//0~3 小板上的那一组 共4组
			//某组的哪个引脚号
			sNum=channel*2;	
		}else if(channel>4&&channel<=8)
		{
			address=0x72;
			//找到属于小板上的那一组
			channel=channel-4;
			channel-=1;	//0~3 小板上的那一组 共4组
			//某组的哪个引脚号
			sNum=channel*2;	
		}else if(channel>8&&channel<=12)
		{
			address=0x74;
			//找到属于小板上的那一组
			channel=channel-8;
			channel-=1;	//0~3 小板上的那一组 共4组
			//某组的哪个引脚号
			sNum=channel*2;	
		}else if(channel>12&&channel<=16)
		{
			address=0x7e;
			//找到属于小板上的那一组
			channel=channel-12;
			channel-=1;	//0~3 小板上的那一组 共4组
			//某组的哪个引脚号
			sNum=channel*2;	
		}else if(channel>16&&channel<=20)
		{
			address=0x7c;
			//找到属于小板上的那一组
			channel=channel-16;
			channel-=1;	//0~3 小板上的那一组 共4组
			//某组的哪个引脚号
			sNum=channel*2;	
		}else if(channel>20&&channel<=24)
		{
			address=0x78;
			//找到属于小板上的那一组
			channel=channel-20;
			channel-=1;	//0~3 小板上的那一组 共4组
			//某组的哪个引脚号
			sNum=channel*2;	
		}else if(channel>24&&channel<=28)
		{
			address=0x76;
			//找到属于小板上的那一组
			channel=channel-24;
			channel-=1;	//0~3 小板上的那一组 共4组
			//某组的哪个引脚号
			sNum=channel*2;	
		}else if(channel>28&&channel<=32)
		{
			address=0x70;
			//找到属于小板上的那一组
			channel=channel-28;
			channel-=1;	//0~3 小板上的那一组 共4组
			//某组的哪个引脚号
			sNum=channel*2;	
		}
		
		if(HAL_PCA9554_readIn(address,&oldVal)==true)
		{
		}else{
			oldVal=0;
		}
		update_oldValue(sNum,dir,status,&oldVal);
	}
	//写操作
	send[1]=oldVal;
	if(HAL_I2C_Master_Transmit(&hi2c2,address,send,sizeof(send),0xFff)!=HAL_OK){
		return false;
	}
	return true;
}

//配置经络仪工作模式
//IO1 治疗信号使能
//IO2 测量信号输出使能
//IO3 测量信号的并行输出 用于整体采集模式
//IO4 测量反馈信号 接入点
bool set_sampleMode(uint8_t mode)
{
	/*关闭后端?*/
	/*前端信号控制*/
	switch(mode)
	{
		case MODE_ZT:	//(IO1_0 IO2_1 IO10-14打开 IO6打开 IO4打开(交流通) IO3_1 IO5_0 IO6_1 IO7_0 IO8_0 IO9_0) 
			WriteOUT1(GPIO_PIN_RESET);
			WriteOUT2(GPIO_PIN_SET);
			WriteOUT3(GPIO_PIN_RESET);
			WriteOUT4(GPIO_PIN_RESET);
			WriteOUT5(GPIO_PIN_RESET);
			WriteOUT6(GPIO_PIN_SET); 
			WriteOUT7(GPIO_PIN_RESET);      //debug    RESET
			WriteOUT8(GPIO_PIN_RESET);       //debug    RESET
			WriteOUT9(GPIO_PIN_RESET);
			WriteOUT10(GPIO_PIN_SET);
			WriteOUT11(GPIO_PIN_SET);
			WriteOUT12(GPIO_PIN_SET);
			WriteOUT13(GPIO_PIN_SET);
			WriteOUT14(GPIO_PIN_SET);
		
			WriteOUT_Mode(GPIO_PIN_RESET);
			break;
		case MODE_JB:	//(IO1_0 IO2_1 IO10-14关闭 IO6打开 IO4打开(交流通) IO3_1 IO5_0 IO6_1 IO7_0 IO8_0 IO9_0)
			WriteOUT1(GPIO_PIN_RESET);
			WriteOUT2(GPIO_PIN_SET);
			WriteOUT3(GPIO_PIN_RESET);
			WriteOUT4(GPIO_PIN_RESET);
			WriteOUT5(GPIO_PIN_RESET);
			WriteOUT6(GPIO_PIN_SET);
			WriteOUT7(GPIO_PIN_RESET);
			WriteOUT8(GPIO_PIN_RESET);
			WriteOUT9(GPIO_PIN_RESET);
			WriteOUT10(GPIO_PIN_RESET);
			WriteOUT11(GPIO_PIN_RESET);
			WriteOUT12(GPIO_PIN_RESET);
			WriteOUT13(GPIO_PIN_RESET);
			WriteOUT14(GPIO_PIN_RESET);
		
			WriteOUT_Mode(GPIO_PIN_RESET);
			break;
//		case MODE_ZL_DC:	//（IO1_1(大椎端子) IO2_1 IO3_0(交流通路)   IO4_0 IO5_1  IO6_0 IO789可配置 IO10-14_0）
//			WriteOUT1(GPIO_PIN_SET);
//			WriteOUT2(GPIO_PIN_SET);
//			WriteOUT3(GPIO_PIN_RESET);
//			WriteOUT4(GPIO_PIN_RESET);
//			WriteOUT5(GPIO_PIN_SET);
//			WriteOUT6(GPIO_PIN_RESET);
//		
//			WriteOUT7(GPIO_PIN_RESET);
//			WriteOUT8(GPIO_PIN_RESET);
//			WriteOUT9(GPIO_PIN_RESET);
//		
//			WriteOUT10(GPIO_PIN_RESET);
//			WriteOUT11(GPIO_PIN_RESET);
//			WriteOUT12(GPIO_PIN_RESET);
//			WriteOUT13(GPIO_PIN_RESET);
//			WriteOUT14(GPIO_PIN_RESET);
//		
//			WriteOUT_Mode(GPIO_PIN_SET);
//			break;
		case MODE_ZL:	//（IO1_1(大椎端子) IO2_1 IO3_1(交流通路)   IO4_1 IO5_0  IO6_0 IO789可配置 IO10-14_0）
			WriteOUT1(GPIO_PIN_SET);
			WriteOUT2(GPIO_PIN_RESET);
			WriteOUT3(GPIO_PIN_SET);
			WriteOUT4(GPIO_PIN_SET);
			WriteOUT5(GPIO_PIN_SET);
			WriteOUT6(GPIO_PIN_RESET);
		
			WriteOUT7(GPIO_PIN_RESET);
			WriteOUT8(GPIO_PIN_RESET);
			WriteOUT9(GPIO_PIN_RESET);
		
			WriteOUT10(GPIO_PIN_RESET);
			WriteOUT11(GPIO_PIN_RESET);
			WriteOUT12(GPIO_PIN_RESET);
			WriteOUT13(GPIO_PIN_RESET);
			WriteOUT14(GPIO_PIN_RESET);
		
			WriteOUT_Mode(GPIO_PIN_SET);
			break;
		case MODE_CLOSE:	//IO2_0 IO7_0 IO8_0 IO9_0 IO3_0 4 5 6 10-14 1 全断开
			WriteOUT1(GPIO_PIN_RESET);
			WriteOUT2(GPIO_PIN_RESET);
			WriteOUT3(GPIO_PIN_RESET);
			WriteOUT4(GPIO_PIN_RESET);
			WriteOUT5(GPIO_PIN_RESET);
			WriteOUT6(GPIO_PIN_RESET);
			WriteOUT7(GPIO_PIN_RESET);
			WriteOUT8(GPIO_PIN_RESET);
			WriteOUT9(GPIO_PIN_RESET);
			WriteOUT10(GPIO_PIN_RESET);
			WriteOUT11(GPIO_PIN_RESET);
			WriteOUT12(GPIO_PIN_RESET);
			WriteOUT13(GPIO_PIN_RESET);
			WriteOUT14(GPIO_PIN_RESET);
			WriteOUT_Mode(GPIO_PIN_RESET);
			break;
		default:
			break;
	}
	return true;
}
