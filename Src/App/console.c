#include "cmsis_os.h"
#include "console.h"
#include "StringTransform.h"
#include "stm32h7xx_hal.h"
#include "main.h"
#include "myUart.h"
#include "mySpi.h"
#include "stepMotor.h"	//hgz add
#include "encoder.h"
#include "common.h"
#include "spiflash.h"
#include "user_tcp_client.h"

bool WriteServerIp(char *src,int l);
bool WriteMqttPort(char *src,int l);
bool WriteHttpPort(char *src,int l);

bool WriteMac(char *src,int l);
bool WriteDevIp(char *src,int l);
bool WriteMask(char *src,int l);
bool WriteGate(char *src,int l);

bool WriteMac2(char *src,int l);
bool WriteDevIp2(char *src,int l);
bool WriteMask2(char *src,int l);
bool WriteGate2(char *src,int l);

bool WriteReset(char *src,int l);
bool WriteFactory(char *src,int l);
bool WriteId(char *src,int l);

bool ReadDevIno(char *src,int l);

CommandOpt	Console_opts[] = {
								{
								"WRITEMAC",
								WriteMac,
								8
								},
								{
								"WRITEMAC2",
								WriteMac2,
								9
								},
								{
								"WRITEDEVIP",
								WriteDevIp,
								10
								},
								{"WRITEDEVIP2",
								WriteDevIp2,
								11
								},
								{
								"WRITEMASK",
								WriteMask,
								9
								},
								{
								"WRITEMASK2",
								WriteMask2,
								10
								},
								{
								"WRITEGATE",
								WriteGate,
								9
								},
								{
								"WRITEGATE2",
								WriteGate2,
								10
								},
								{
								"WRITEFACTORY",
								WriteFactory,
								12
								},
								{
								"WRITERESET",
								WriteReset,
								10
								},
								{
								"WRITEID",
								WriteId,
								7
								},
								{
								"WRITESERVERIP",
								WriteServerIp,
								13
								},
								{
								"WRITEMQTTPORT",
								WriteMqttPort,
								13
								},
								{
								"WRITEHTTPPORT",
								WriteHttpPort,
								13
								},
								{
								"READDEVINFO",
								ReadDevIno,
								11
								}
																
};

static const char buf_error[]  = "MSG:error\r\n";
static const char buf_excute[]  = "MSG:excute error\r\n";
static const char buf_ok[]  = "MSG:ok\r\n";
static  uint8_t msg_ok[30] = "MSG:OK\r\n";

bool Sys_Command_Set(char *src,int l)
{
	static int i;
	char* p;
	char tmp[32]={0x00};
	CommandOpt er;
	if(src[l-2]=='\r' && src[l-1]=='\n')
	{
		for(i=0;i< sizeof(Console_opts) /sizeof(CommandOpt);i++)
		{
			memcpy(tmp,src,Console_opts[i].l);
			strupper(tmp,Console_opts[i].l);
			if(strncmp(tmp,Console_opts[i].src,Console_opts[i].l) == 0)
			{
				p = src+Console_opts[i].l;
				er = Console_opts[i];
				if( er.pf(p,l-Console_opts[i].l) )
				{
					printf("MSG:%s",src);
					printf("%s",buf_ok);
					return true;
				}
				else
				{
					printf("MSG:%s",src);
					printf("%s",buf_excute);
					return false;
				}		
			}
		}
		printf("MSG:%s",src);
		printf("%s",buf_error);
		return false;	
	}
	else
	{
		printf("MSG:%s",src);
		printf("%s",buf_error);
		return false;	
	}
}		


bool Update_Set(char *src,int l)
{
	return false;
}

#if 0
void Console_Task1(void const * pvParameters)
{
	#if 1
	int i;

	osDelay(10);

	for(;;)
	{

		xSemaphoreTake(ConsoleReceive_Semaphore,portMAX_DELAY);
		#if 1
		if(Update_Set((char*)RecCom5,RecCom5Num)==false)
		{
			Sys_Command_Set((char*)RecCom5,RecCom5Num);
		}
		for(i=0;i<RecCom5Num;i++)	{RecCom5[i]=0;}
		HAL_UART_Receive_DMA(CONSOLE_PORT, RecCom5, COM5_REC_MAX);
		#endif

		#if 0
		memset(RecCom1,0,COM1_REC_MAX);
		READ_REG(huart1.Instance->RDR);
		SET_BIT(huart1.Instance->CR1, USART_CR1_RE);
		HAL_UART_Receive_DMA(&huart1, RecCom1, COM1_REC_MAX); //若信号丢失 则会导致DMA未开启，不能放在这里
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		#endif

	}
	#endif
	
}
#endif

bool WriteReset(char *src,int l)
{
	NVIC_SystemReset();
	osDelay(100);
	return true;
}

bool WriteFactory(char *src,int l)
{	
	Restore_Default_Parameter();
	Save_Parameter();
	return true;
}

bool WriteServerIp(char *src,int l)
{
	uint8_t i;
	uint8_t intIp[4]={0,};
	int8_t srcBuf[20]={0};
	char *p,*pt;
	p=src;

	if((pt=strstr(p,"=")) != NULL)
	{	
		pt = pt+1;
		strncpy((char*)srcBuf,pt,15);
		srcBuf[15]='\0';
		p=NULL;
		pt=NULL;
		
		if(isVaildIp((char*)srcBuf,intIp)==true)
		{
			for(i=0;i<4;i++)
			{
				gDeviceParam.ServerIpAddress[i]=intIp[i];
			}
			Save_Parameter();
			return true;
		}else{
			return false;
		}
	}else
	{
		p=NULL;
		pt=NULL;
		return false;	
	}
}

bool WriteMqttPort(char *src,int l)
{
	char *p,*pt;
	uint16_t port;
	p=src;

	if((pt=strstr(p,"=")) != NULL)
	{
		pt = pt+1;
		port=atoi(pt);
		if(port>0&&port<65535)
		{

			gDeviceParam.mqArg.mqttPort=port;
			Save_Parameter();
			p=NULL;
			pt=NULL;
			return true;
		}else{
			p=NULL;
			pt=NULL;
			return false;
		}

	}else
	{
		p=NULL;
		pt=NULL;
		return false;	
	}
}

bool WriteHttpPort(char *src,int l)
{
	char *p,*pt;
	uint16_t port;
	p=src;

	if((pt=strstr(p,"=")) != NULL)
	{
		pt = pt+1;
		port=atoi(pt);
		if(port>0&&port<65535)
		{

			gDeviceParam.httpPort=port;
			Save_Parameter();
			p=NULL;
			pt=NULL;
			return true;
		}else{
			p=NULL;
			pt=NULL;
			return false;
		}

	}else
	{
		p=NULL;
		pt=NULL;
		return false;	
	}
}

bool WriteMac(char *src,int l)
{

	uint8_t i;
	uint8_t intMac[12]={0,};
	int8_t srcBuf[20]={0};
	char *p,*pt;
	p=src;

	if((pt=strstr(p,"=")) != NULL)
	{	
		pt = pt+1;
		strncpy((char*)srcBuf,pt,12);
		srcBuf[12]='\0';
		p=NULL;
		pt=NULL;
		
		if(isVaildMac((char*)srcBuf,intMac)==true)
		{
			for(i=0;i<6;i++)
			{
				gDeviceParam.rj45Arg.MacAddress[i]=intMac[i];
			}
			Save_Parameter();
			p=NULL;
			pt=NULL;
			return true;
		}else{
			p=NULL;
			pt=NULL;
			return false;
		}
	}else
	{
		p=NULL;
		pt=NULL;
		return false;	
	}
}

bool WriteMac2(char *src,int l)
{

	uint8_t i;
	uint8_t intMac[12]={0,};
	int8_t srcBuf[20]={0};
	char *p,*pt;
	p=src;

	if((pt=strstr(p,"=")) != NULL)
	{	
		pt = pt+1;
		strncpy((char*)srcBuf,pt,12);
		srcBuf[12]='\0';
		p=NULL;
		pt=NULL;
		
		if(isVaildMac((char*)srcBuf,intMac)==true)
		{
			for(i=0;i<6;i++)
			{
				gDeviceParam.wifiArg.MacAddress[i]=intMac[i];
			}
			Save_Parameter();
			p=NULL;
			pt=NULL;
			return true;
		}else{
			p=NULL;
			pt=NULL;
			return false;
		}
	}else
	{
		p=NULL;
		pt=NULL;
		return false;	
	}
}

bool WriteDevIp(char *src,int l)
{	
	uint8_t i;
	uint8_t intIp[4]={0,};
	int8_t srcBuf[20]={0};
	char *p,*pt;
	p=src;

	if((pt=strstr(p,"=")) != NULL)
	{	
		pt = pt+1;
		strncpy((char*)srcBuf,pt,15);
		srcBuf[15]='\0';
		p=NULL;
		pt=NULL;
		
		if(isVaildIp((char*)srcBuf,intIp)==true)
		{
			for(i=0;i<4;i++)
			{
				gDeviceParam.rj45Arg.IpAddress[i]=intIp[i];
			}
			Save_Parameter();
			return true;
		}else{
			return false;
		}
	}else
	{
		p=NULL;
		pt=NULL;
		return false;	
	}
}

bool WriteDevIp2(char *src,int l)
{	
	uint8_t i;
	uint8_t intIp[4]={0,};
	int8_t srcBuf[20]={0};
	char *p,*pt;
	p=src;

	if((pt=strstr(p,"=")) != NULL)
	{	
		pt = pt+1;
		strncpy((char*)srcBuf,pt,15);
		srcBuf[15]='\0';
		p=NULL;
		pt=NULL;
		
		if(isVaildIp((char*)srcBuf,intIp)==true)
		{
			for(i=0;i<4;i++)
			{
				gDeviceParam.wifiArg.IpAddress[i]=intIp[i];
			}
			Save_Parameter();
			return true;
		}else{
			return false;
		}
	}else
	{
		p=NULL;
		pt=NULL;
		return false;	
	}
}



bool WriteMask(char *src,int l)
{
	uint8_t i;
	uint8_t intMask[4]={0,};
	int8_t srcBuf[20]={0};
	char *p,*pt;
	p=src;

	if((pt=strstr(p,"=")) != NULL)
	{	
		pt = pt+1;
		strncpy((char*)srcBuf,pt,15);
		srcBuf[15]='\0';
		p=NULL;
		pt=NULL;
		
		if(isVaildIp((char*)srcBuf,intMask)==true)
		{
			for(i=0;i<4;i++)
			{
				gDeviceParam.rj45Arg.MaskAddress[i]=intMask[i];
			}
			Save_Parameter();
			return true;
		}else{
			return false;
		}
	}else
	{
		p=NULL;
		pt=NULL;
		return false;	
	}
}

bool WriteMask2(char *src,int l)
{
	uint8_t i;
	uint8_t intMask[4]={0,};
	int8_t srcBuf[20]={0};
	char *p,*pt;
	p=src;

	if((pt=strstr(p,"=")) != NULL)
	{	
		pt = pt+1;
		strncpy((char*)srcBuf,pt,15);
		srcBuf[15]='\0';
		p=NULL;
		pt=NULL;
		
		if(isVaildIp((char*)srcBuf,intMask)==true)
		{
			for(i=0;i<4;i++)
			{
				gDeviceParam.wifiArg.MaskAddress[i]=intMask[i];
			}
			Save_Parameter();
			return true;
		}else{
			return false;
		}
	}else
	{
		p=NULL;
		pt=NULL;
		return false;	
	}
}
bool WriteGate(char *src,int l)
{
	uint8_t i;
	uint8_t intGate[4]={0,};
	int8_t srcBuf[20]={0};
	char *p,*pt;
	p=src;

	if((pt=strstr(p,"=")) != NULL)
	{	
		pt = pt+1;
		strncpy((char*)srcBuf,pt,15);
		srcBuf[15]='\0';
		p=NULL;
		pt=NULL;
		
		if(isVaildIp((char*)srcBuf,intGate)==true)
		{
			for(i=0;i<4;i++)
			{
				gDeviceParam.rj45Arg.GateAddress[i]=intGate[i];
			}
			Save_Parameter();
			return true;
		}else{
			return false;
		}
	}else
	{
		p=NULL;
		pt=NULL;
		return false;	
	}
}

bool WriteGate2(char *src,int l)
{
	uint8_t i;
	uint8_t intGate[4]={0,};
	int8_t srcBuf[20]={0};
	char *p,*pt;
	p=src;

	if((pt=strstr(p,"=")) != NULL)
	{	
		pt = pt+1;
		strncpy((char*)srcBuf,pt,15);
		srcBuf[15]='\0';
		p=NULL;
		pt=NULL;
		
		if(isVaildIp((char*)srcBuf,intGate)==true)
		{
			for(i=0;i<4;i++)
			{
				gDeviceParam.wifiArg.GateAddress[i]=intGate[i];
			}
			Save_Parameter();
			return true;
		}else{
			return false;
		}
	}else
	{
		p=NULL;
		pt=NULL;
		return false;	
	}
}



bool WriteId(char *src,int l)
{
	uint8_t i;
	uint32_t deviceId;
	int8_t srcBuf[10]={0};
	char *p,*pt;
	p=src;
	
	//判断Id是否是纯数字
	for(i=1;i<=8;i++)
	{
		if(src[i]>='0'&&src[i]<='9')
		{
		}else{
			return false;
		}
	}
	
	if(src[1]=='0'||l!=11)	//8+3
		return false;
	
		
	if((pt=strstr(p,"=")) != NULL)
	{	
		pt = pt+1;
		strncpy((char*)srcBuf,pt,8);
		srcBuf[8]='\0';
		deviceId=atoi((char*)srcBuf);	
		p=NULL;
		pt=NULL;
		
		if(deviceId<Device_ID_Max)
		{
			gDeviceParam.devId=deviceId;
			Save_Parameter();
			return true;
		}else{
			return false;
		}
	}else
	{
		p=NULL;
		pt=NULL;
		return false;	
	}
}


bool ReadDevIno(char *src,int l)
{
	Init_Parameter();
	printf("ServerIp:%d.%d.%d.%d\r\n",gDeviceParam.ServerIpAddress[0],gDeviceParam.ServerIpAddress[1],gDeviceParam.ServerIpAddress[2],gDeviceParam.ServerIpAddress[3]);
	
	printf("rj45Mac:%02x-%02x-%02x-%02x-%02x-%02x\r\n",gDeviceParam.rj45Arg.MacAddress[0],gDeviceParam.rj45Arg.MacAddress[1],gDeviceParam.rj45Arg.MacAddress[2],gDeviceParam.rj45Arg.MacAddress[3],gDeviceParam.rj45Arg.MacAddress[4],gDeviceParam.rj45Arg.MacAddress[5]);
	printf("rj45DevIp:%d.%d.%d.%d\r\n",gDeviceParam.rj45Arg.IpAddress[0],gDeviceParam.rj45Arg.IpAddress[1],gDeviceParam.rj45Arg.IpAddress[2],gDeviceParam.rj45Arg.IpAddress[3]);
	printf("rj45Mask:%d.%d.%d.%d\r\n",gDeviceParam.rj45Arg.MaskAddress[0],gDeviceParam.rj45Arg.MaskAddress[1],gDeviceParam.rj45Arg.MaskAddress[2],gDeviceParam.rj45Arg.MaskAddress[3]);
	printf("rj45Gate:%d.%d.%d.%d\r\n",gDeviceParam.rj45Arg.GateAddress[0],gDeviceParam.rj45Arg.GateAddress[1],gDeviceParam.rj45Arg.GateAddress[2],gDeviceParam.rj45Arg.GateAddress[3]);
	
	printf("wifiMac:%02x-%02x-%02x-%02x-%02x-%02x\r\n",gDeviceParam.wifiArg.MacAddress[0],gDeviceParam.wifiArg.MacAddress[1],gDeviceParam.wifiArg.MacAddress[2],gDeviceParam.wifiArg.MacAddress[3],gDeviceParam.wifiArg.MacAddress[4],gDeviceParam.wifiArg.MacAddress[5]);
	printf("wifiDevIp:%d.%d.%d.%d\r\n",gDeviceParam.wifiArg.IpAddress[0],gDeviceParam.wifiArg.IpAddress[1],gDeviceParam.wifiArg.IpAddress[2],gDeviceParam.wifiArg.IpAddress[3]);
	printf("wifiMask:%d.%d.%d.%d\r\n",gDeviceParam.wifiArg.MaskAddress[0],gDeviceParam.wifiArg.MaskAddress[1],gDeviceParam.wifiArg.MaskAddress[2],gDeviceParam.wifiArg.MaskAddress[3]);
	printf("wifiGate:%d.%d.%d.%d\r\n",gDeviceParam.wifiArg.GateAddress[0],gDeviceParam.wifiArg.GateAddress[1],gDeviceParam.wifiArg.GateAddress[2],gDeviceParam.wifiArg.GateAddress[3]);
	
	printf("mqttPort:%d\r\n",gDeviceParam.mqArg.mqttPort);
	printf("httpPort:%d\r\n",gDeviceParam.httpPort);
	printf("DevId:%08d\r\n",gDeviceParam.devId);

	printf("hardVer:%s\r\n",HardVersion);
	printf("softVer:%s\r\n",SoftVersion);
	return true;
}



















