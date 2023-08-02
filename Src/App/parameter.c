#include "string.h"
/**private include **/
#include "parameter.h"
#include "spiFlash.h"
#include "common.h"
#include "main.h"
#include "common.h"
#include <stdlib.h>
Parameter_Dev  gDeviceParam;

RUN_Global gGlobalData;

SIM_INFO_TYPE	gSimInfo;

USER_INFO_TYPE gUserInfo;

char gStatusAck[JSON_ACK_MAX_LEN];

int Init_All_Parameter(void)
{
	int i,j;
	Init_Parameter();
	/*恢复动态参数*/
//	gGlobalData.unitLen=gDeviceParam.workArg.rateN*gDeviceParam.workArg.timeT/1000;
//	if(gGlobalData.unitLen<=0)
//	{
//		gGlobalData.unitLen=1;
//	}
//	gGlobalData.currentUnitNum=0;

	gGlobalData.unitLen=0;
	gGlobalData.currentUnitNum=0;
	gGlobalData.curWorkMode=WORK_MODE_WT;
	gGlobalData.curWorkState=WORK_STOP;
	gGlobalData.netKind=NET_RJ45;
	gGlobalData.ResetStatus=true;
	gGlobalData.cur_heart_state=UNKNOWN;
	
	for(i=0;i<2;i++)
	{
		gGlobalData.useWorkArg[i].workMode=gDeviceParam.workArg[i].workMode;
		gGlobalData.useWorkArg[i].level=gDeviceParam.workArg[i].level;
		gGlobalData.useWorkArg[i].timeTreat=gDeviceParam.workArg[i].timeTreat;
		gGlobalData.useWorkArg[i].timeCheck=gDeviceParam.workArg[i].timeCheck;
		gGlobalData.useWorkArg[i].rateN=gDeviceParam.workArg[i].rateN;
		gGlobalData.useWorkArg[i].upTime=gDeviceParam.workArg[i].upTime;
		gGlobalData.useWorkArg[i].chanelNum=gDeviceParam.workArg[i].chanelNum;
		

		gGlobalData.useWorkArg[i].waveCheck=gDeviceParam.workArg[i].waveCheck;
		gGlobalData.useWorkArg[i].freqCheck=gDeviceParam.workArg[i].freqCheck;
		gGlobalData.useWorkArg[i].waveTreat=gDeviceParam.workArg[i].waveTreat;
		gGlobalData.useWorkArg[i].freqTreat=gDeviceParam.workArg[i].freqTreat;
		
		for(j=0;j<gGlobalData.useWorkArg[i].chanelNum;j++)
		{
			gGlobalData.useWorkArg[i].inputs[j]=gDeviceParam.workArg[i].inputs[j];
			gGlobalData.useWorkArg[i].outs[j]=gDeviceParam.workArg[i].outs[j];
		}
	}
	

	gGlobalData.rj45Status=false;
	gGlobalData.wifiStatus=false;
	gGlobalData.yd4gStatus=false;
	
	gGlobalData.Send_Data_Task=false;//主动上报的采集数据包

	gGlobalData.Send_Client_Over= false; //用户信息下发标志位
	
	gGlobalData.Send_Ack_Task=false;	//被动应答的数据包
	gGlobalData.Send_Data_Sync_Task=false;	//暂未使用
	gGlobalData.Send_Update_Task=false;	//暂未使用
	return 1;
}

int Init_Parameter(void)
{
	 
	uint8_t *p;
	uint32_t *temp;
	int i;
		
	const int length = sizeof(FLASH_MAGIC)+sizeof(Parameter_Dev);
	uint8_t ch[length];

	W25QX_Read_Data(ch,length,SECTOR_PARAMETER);
	temp = (uint32_t *)ch;
	if(*temp == FLASH_MAGIC)
	{
		p = ch+4;
		memcpy(&gDeviceParam,p,sizeof(Parameter_Dev));
		p =NULL;
		
		for(i=0;i<2;i++)
		{
				//检测参数-工作模式
			if(gDeviceParam.workArg[i].workMode>WORK_MODE_ZL)
			{
				gDeviceParam.workArg[i].workMode=WORK_MODE_WT;
			}

				//检测参数-输出档位
			if(gDeviceParam.workArg[i].level>Level_MAX||gDeviceParam.workArg[i].upTime<Level_MIN)
			{
				gDeviceParam.workArg[i].level=DefLevel;
			}
			//检测参数-单通道采样时长
			if(gDeviceParam.workArg[i].timeTreat>TimeT_MAX||gDeviceParam.workArg[i].timeTreat<TimeT_MIN)
			{
				gDeviceParam.workArg[i].timeTreat=DeftimeT;
			}
				if(gDeviceParam.workArg[i].timeCheck>TimeT_MAX||gDeviceParam.workArg[i].timeCheck<TimeT_MIN)
			{
				gDeviceParam.workArg[i].timeCheck=DeftimeT;
			}
			//检测参数-数据率
			if(gDeviceParam.workArg[i].rateN>CollectFreq_MAX||gDeviceParam.workArg[i].rateN<CollectFreq_MIN)
			{
				gDeviceParam.workArg[i].rateN=DefFreqN;
			}

			//检测参数-上传间隔
			if(gDeviceParam.workArg[i].upTime>UpTime_MAX||gDeviceParam.workArg[i].upTime<UpTime_MIN)
			{
				gDeviceParam.workArg[i].upTime=DefUpTime;
			}
			
			//检测参数-波形和频率
			if(gDeviceParam.workArg[i].waveTreat>=3)
			{
				gDeviceParam.workArg[i].waveTreat=DefWaveTreat;
			}
			if(gDeviceParam.workArg[i].freqTreat>=3)
			{
				gDeviceParam.workArg[i].freqTreat=DefFreqTreat;
			}
			
			if(gDeviceParam.workArg[i].waveCheck>=3)
			{
				gDeviceParam.workArg[i].waveCheck=DefWaveCheck;
			}
			if(gDeviceParam.workArg[i].freqCheck>=3)
			{
				gDeviceParam.workArg[i].freqCheck=DefFreqCheck;
			}
		}

		
		if(gDeviceParam.devLock!=LOCK_YES&&gDeviceParam.devLock!=LOCK_NO)
		{
			gDeviceParam.devLock=LOCK_NO;
		}
				 //调试复位
//		Restore_Default_Parameter();  
//		Save_Parameter();		
		if(gDeviceParam.devId==0x6A747379){  			//烧录代码flash里老存其他的值，在这初始化下
      Restore_Default_Parameter();
		  Save_Parameter();
		}else{
		//处理一下上网连接的mac地址,每一台设备必须不一样否则网络会出现延时和中断 扫码不成功的问题所在 kardos 2023.04.19
			gDeviceParam.rj45Arg.MacAddress[1]=rand()%16;//随机数 0-F
			gDeviceParam.rj45Arg.MacAddress[2]=rand()%16;//随机数 0-F
			gDeviceParam.rj45Arg.MacAddress[3]=(gDeviceParam.devId/10000)%255;//设备id前两位
			gDeviceParam.rj45Arg.MacAddress[4]=(gDeviceParam.devId/100)%100;//设备id中间两位
			gDeviceParam.rj45Arg.MacAddress[5]=gDeviceParam.devId%100;//设备id后两位			
			
			gDeviceParam.wifiArg.MacAddress[1]=rand()%16;//随机数 0-F
			gDeviceParam.wifiArg.MacAddress[2]=rand()%16;//随机数 0-F
			gDeviceParam.wifiArg.MacAddress[3]=(gDeviceParam.devId/10000)%255;//设备id前两位 10
			gDeviceParam.wifiArg.MacAddress[4]=(gDeviceParam.devId/100)%100;//设备id中间两位 00
			gDeviceParam.wifiArg.MacAddress[5]=gDeviceParam.devId%100;//设备id后两位 08


		}
	}
	else
	{
		Restore_Default_Parameter();
		
		Save_Parameter();

		return 0;
	}
	return 0;
}


int Restore_Default_Parameter(void)
{
	int i;
	uint8_t qrbuf[30]={0x79,0x73,0x74,0x6a,0x6c,0x79,0x2d,0x6a,0x6c,0x79,0x2d,0x59, 0x32,0x35, 0x34, 0x35, 0x32, 0x31, 0x2d, 0x31,0x30,0x30,0x30,0x30,0x31,0x2d};
	//设备参数
	gDeviceParam.devId=100001;
	memset (gDeviceParam.productKey,0,sizeof(gDeviceParam.productKey));	
	strcpy((char*)gDeviceParam.productKey,"JLY");
	gDeviceParam.heartRate=DefHeartRate;    //心跳间隔	
	gDeviceParam.httpPort=DefHttpPort;
	gDeviceParam.devLock=LOCK_NO;
	
	
	gDeviceParam.ServerIpAddress[0]=124;
	gDeviceParam.ServerIpAddress[1]=239;
	gDeviceParam.ServerIpAddress[2]=10;
	gDeviceParam.ServerIpAddress[3]=57;
						

	//工作参数
	for(i=0;i<2;i++)
	{
		gDeviceParam.workArg[i].workMode=DefWorkMode;
		gDeviceParam.workArg[i].level=DefLevel;
		gDeviceParam.workArg[i].timeTreat=DeftimeT;
		gDeviceParam.workArg[i].timeCheck=DeftimeT;
		gDeviceParam.workArg[i].rateN=DefFreqN;
		gDeviceParam.workArg[i].upTime=DefUpTime;
		
		gDeviceParam.workArg[i].waveCheck=DefWaveCheck;
		gDeviceParam.workArg[i].freqCheck=DefFreqCheck;
		gDeviceParam.workArg[i].waveTreat=DefWaveTreat;
		gDeviceParam.workArg[i].freqTreat=DefFreqTreat;
		
		gDeviceParam.workArg[i].chanelNum=1;
		
	}
	
	for(i=0;i<gDeviceParam.workArg[0].chanelNum;i++)
	{
		gDeviceParam.workArg[0].inputs[i]=2;
		gDeviceParam.workArg[0].outs[i]=33;
	}
	
	for(i=0;i<gDeviceParam.workArg[1].chanelNum;i++)
	{
		gDeviceParam.workArg[1].inputs[i]=2;
		gDeviceParam.workArg[1].outs[i]=1;
	}
	
	//mq参数
	strcpy(gDeviceParam.mqArg.MQ_Username,"100001");
	strcpy(gDeviceParam.mqArg.MQ_Password,"123456");
	gDeviceParam.mqArg.mqttPort=DefMqttPort;
	
	//rj45网络
	gDeviceParam.rj45Arg.IpAddress[0]=192;
	gDeviceParam.rj45Arg.IpAddress[1]=168;
	gDeviceParam.rj45Arg.IpAddress[2]=1;
	gDeviceParam.rj45Arg.IpAddress[3]=30;
	
	gDeviceParam.rj45Arg.MaskAddress[0]=255;
	gDeviceParam.rj45Arg.MaskAddress[1]=255;
	gDeviceParam.rj45Arg.MaskAddress[2]=255;
	gDeviceParam.rj45Arg.MaskAddress[3]=0;
	
	gDeviceParam.rj45Arg.GateAddress[0]=192;
	gDeviceParam.rj45Arg.GateAddress[1]=168;
	gDeviceParam.rj45Arg.GateAddress[2]=1;
	gDeviceParam.rj45Arg.GateAddress[3]=1;
	
	
	gDeviceParam.rj45Arg.MacAddress[0]=0x10;//10表示经络仪，20表示穴位理疗仪
	gDeviceParam.rj45Arg.MacAddress[1]=0x10;//随机数 0-F
	gDeviceParam.rj45Arg.MacAddress[2]=0x20;//随机数 0-F
	gDeviceParam.rj45Arg.MacAddress[3]=0x10;//设备id前两位
	gDeviceParam.rj45Arg.MacAddress[4]=0x00;//设备id中间两位
	gDeviceParam.rj45Arg.MacAddress[5]=0x01;//设备id后两位
	
	//wifi网络
	gDeviceParam.wifiArg.IpAddress[0]=192;
	gDeviceParam.wifiArg.IpAddress[1]=168;
	gDeviceParam.wifiArg.IpAddress[2]=1;
	gDeviceParam.wifiArg.IpAddress[3]=31;
	
	gDeviceParam.wifiArg.MaskAddress[0]=255;
	gDeviceParam.wifiArg.MaskAddress[1]=255;
	gDeviceParam.wifiArg.MaskAddress[2]=255;
	gDeviceParam.wifiArg.MaskAddress[3]=0;
	
	gDeviceParam.wifiArg.GateAddress[0]=192;
	gDeviceParam.wifiArg.GateAddress[1]=168;
	gDeviceParam.wifiArg.GateAddress[2]=1;
	gDeviceParam.wifiArg.GateAddress[3]=1;
	
	
	gDeviceParam.wifiArg.MacAddress[0]=0x10;
	gDeviceParam.wifiArg.MacAddress[1]=0x10;
	gDeviceParam.wifiArg.MacAddress[2]=0x20;
	gDeviceParam.wifiArg.MacAddress[3]=0x10;
	gDeviceParam.wifiArg.MacAddress[4]=0x00;
	gDeviceParam.wifiArg.MacAddress[5]=0x01;
	
	strcpy((char*)gDeviceParam.wifiArg.WifiName,"H3C_TEST");//默认
	strcpy((char*)gDeviceParam.wifiArg.WifiPwd,"ty123456");
//	strcpy((char*)gDeviceParam.wifiArg.WifiName,"612_lab");			//wifi名称
//	strcpy((char*)gDeviceParam.wifiArg.WifiPwd,"612612612");		//wifi密码
//	strcpy((char*)gDeviceParam.wifiArg.WifiName,"hyphone");			//wifi名称
//	strcpy((char*)gDeviceParam.wifiArg.WifiPwd,"012345678");		//wifi密码
//	strcpy((char*)gDeviceParam.wifiArg.WifiName,"xfphone");			//wifi名称
//	strcpy((char*)gDeviceParam.wifiArg.WifiPwd,"12345678");		//wifi密码
	strcpy((char*)gDeviceParam.version,"1.0.0");
	memset (gDeviceParam.qrbuf,0,sizeof(gDeviceParam.qrbuf));	
	//屏幕左侧二维码
	for(i=0;i<strlen(qrbuf);i++)
		{
			gDeviceParam.qrbuf[i]=qrbuf[i];  //JLY二维码
		}	
	return 0;
}


int Save_Parameter(void)
{
	static int length = sizeof(FLASH_MAGIC)+sizeof(Parameter_Dev);

	uint8_t ch[length]; 
	uint8_t *p;
	uint32_t temp = FLASH_MAGIC;

	//保存
	memcpy(ch,&temp,4);
	p =ch+4;
	memcpy(p,&gDeviceParam,sizeof(Parameter_Dev));

	p =NULL;
	
	if(W25QX_Sector_Erase(SECTOR_PARAMETER) == false)
	{
		printf("save parameter erase sector fail\r\n");
		return 0;
	}
	if(W25QX_Page_Write(ch,length,SECTOR_PARAMETER)==false)
	{
		printf("save parameter write sector fail\r\n");
		return 0;
	}
	return 1;
	
}


