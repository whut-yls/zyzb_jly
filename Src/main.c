/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"
#include "ethernetif.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "user_tcp_server.h"
#include "user_tcp_client.h"
#include "console.h"

#include "stm32h743i_eval_qspi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "myAdc.h"
#include "startLoopTask.h"
#include "pca9554.h"
#include "common.h"
#include "com.h"
#include "ad9833.h"
#include "lcdTask.h"
#include "dac8831.h"
#include "wifi.h"
#include "EC20_4G.h"
#include "Wt2003hx.h"
#include "timer.h"
#include "dac8831.h"
#include "I2C.h"//里面包含iic.h

SemaphoreHandle_t					ConsoleReceive_Semaphore=NULL; 
SemaphoreHandle_t  	lcdSemaphore = NULL;
unsigned char Voice_aRxBuffer[Voice_RXBUFFERSIZE];
unsigned char RecCom7[COM7_REC_MAX+1];	//pc
uint16_t 			RecCom7Num=0;
unsigned char RecCom3[COM3_REC_MAX+1];	//pc
uint16_t 			RecCom3Num=0;
unsigned char RecCom6[COM6_REC_MAX+1];	//pc
uint16_t 			RecCom6Num=0;
unsigned char RecCom2[COM2_REC_MAX];	//pc

uint8_t       RecSen[30];   //接收屏幕返回数据
uint32_t channelTime = 0;
bool gUartPcInit=false,gUartPcTc=false;
bool gUartLcdInit=false,gUartLcdTc=false;
	////////////////////////2023.03.24  kardos/////////////////////////////////////////////
bool isCollect=false;//是否开始采集数据
uint32_t 	collectDataCount;//采集数据个数
static int reTime=0;//采集时间间隔，单位ms	
	//////////////////////////////////////////////////////////////////////////////////////
bool yd_4gfalg;
uint32_t countTime = 0;//计数器，用于倒计时计算 2023.02.06 kardos
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
osThreadId defaultTaskHandle;
osThreadId startLoopTaskHandle;
osThreadId consoleTaskHandle;
osThreadId wifiTaskHandle;
osThreadId ec4gTaskHandle;

osThreadId mqttClientTaskHandle;
osThreadId mqttSendTaskHandle;
osThreadId ethernetTaskTaskHandle;
osThreadId sendheartTaskHandle;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//bool gEthSendStatus=false;  //数据端口状态
bool gEthRecvStatus=false;  //控制端口状态
//bool gEthRecvReturn=false;  //控制端口状态 是否有回执
//bool gEthSendReturn=false;  //数据流端口 是否有回执
int cnt_heartbag = 0;
bool gEthStatus=false;      //网络状态
bool gRecvEthChange=false;  //状态改变标识
bool gWifiChange;
//bool gSendEthChange=false;  //状态改变标识
//bool gTransferDon=false;    //信号量等待状态
void StartDefaultTask(void const * argument);

bool gMqttLinkStatus=false;      //mq网络状态
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

//ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi5;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;
DMA_HandleTypeDef hdma_spi3_tx;

/* USER CODE BEGIN PV */
uint16_t gREFVOL_VAL=3300;  //3300
//int16_t gAdcVol[33];
uint16_t gADC1_DMA_BUF[AD1_NUM*N_NUM];
uint16_t gADC1_VALUE[AD1_NUM];
uint16_t gADC1_VALUE_F[AD1_NUM];

uint16_t gADC3_DMA_BUF[AD3_NUM*N_NUM];
uint16_t gADC3_VALUE[AD3_NUM];
uint16_t gADC3_VALUE_F[AD3_NUM];



bool Collect_Data_state=false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void MPU_Config(void);
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void Console_Task(void const * pvParameters);
void StartDefaultTask(void const * argument);
void StartLoopTask(void const * argument);

IWDG_HandleTypeDef IWDG_Handler; //独立看门狗句柄
void StartIwdgTask(void const *arg);
EventGroupHandle_t xCreatedEventGroup;
void IWDG_Init(uint8_t prer, uint16_t rlr);
void IWDG_Feed(void);

/* USER CODE BEGIN PFP */ 

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool Console_TaskSemaphore_Init(void)
{
	ConsoleReceive_Semaphore = xSemaphoreCreateBinary();

	if (ConsoleReceive_Semaphore == NULL )
	{
		return false;
	}
	 return true;
}
char gbk0[]={0,};
/* 软件定时器*/
uint16_t time_ = 1; //ms

//static volatile uint8_t flagAutoLoadTimerRun = 0;
static void vAutoLoadTimerFunc( TimerHandle_t xTimer );
#define mainAUTO_RELOAD_TIMER_PERIOD pdMS_TO_TICKS( time_ )
uint16_t  ch1buf[100];
static int count1 = 0;
static int cnt_down = 0;
static int Resetcnt = 0;

//int debug_a = 0;
//软件定时器回调函数
static void vAutoLoadTimerFunc( TimerHandle_t xTimer )
{
//	xEventGroupSetBits(xCreatedEventGroup, TASK_BIT_1);//喂狗
	//debug_a = (gADC1_VALUE_F[0]>>8)+(gADC1_VALUE_F[0]<<8);
	if(count1 == 0){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	}
	else if(count1 == 500)
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	Resetcnt++;
	count1++;
	if(gGlobalData.curWorkState == WORK_START && channelTime < 0xFFFFFFF0)
	{channelTime++;
		cnt_down++;}
	if(Resetcnt==1000){
		Resetcnt=0;
		if(gGlobalData.ResetDevidcnt>=3)
		{	
			Restore_Default_Parameter();
			Save_Parameter();
		}
		gGlobalData.ResetDevidcnt=0;
	}
	if(count1 == 1000)
		count1 = 0;
	//治疗计时
	if(cnt_down == 1000 && gGlobalData.curWorkMode == WORK_MODE_ZL && gGlobalData.curWorkState == WORK_START)//一秒进来一次
	{
		if((gGlobalData.Alltime != 0 && gGlobalData.curWorkState == WORK_START))
		{
			gGlobalData.Alltime = gGlobalData.Alltime - 1;	
		}
		cnt_down = 0;
	}

		//处理心跳包   2023.04.07 
	if(cnt_heartbag > gDeviceParam.heartRate *1000)//到时间 且当前网络状态处于正常情况下才发心跳包
	{
		cnt_heartbag = 0;
		gGlobalData.heartbag_flage = 1;
	}
	cnt_heartbag++;
	if(gGlobalData.heart_count < 5* gDeviceParam.heartRate *1000)
		gGlobalData.heart_count++;
	else
	{
			if(gGlobalData.netKind==1){              //说明时网口断开了
				gGlobalData.isConnect=0;
				gGlobalData.conFlage=2;//让wifi去连接       //网口的线程不解析心跳应答     
				gGlobalData.Wifi_set = true;														//重新设置一下wifi参数
			}
			else if(gGlobalData.netKind==2){           //说明时wifi断开了  
				gGlobalData.isConnect=0;//说明网络断开 
				gGlobalData.conFlage=3;//让4g去连接        
				gGlobalData.yd4gStatus=false;
			}
			else if(gGlobalData.netKind==3){      //说明时4g断开了
				gGlobalData.isConnect=0;						//说明网络断开 
				gGlobalData.conFlage=2;							//让wifi去连接					
				gGlobalData.yd4gStatus=false;
				gGlobalData.Wifi_set = true;					
			}
		 gGlobalData.heart_count = 0;//超时计数器置为0 重新开始计数
	}
	
	if(cnt_down == 1000)
		cnt_down = 0;
	
	/////////////////////2023.03.24 kardos 新增数据采集//////////////
		if(gGlobalData.curWorkState == WORK_START && gGlobalData.curWorkMode==WORK_MODE_ZT && isCollect==true && gGlobalData.isConnect == 1)
		{
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *)gADC1_DMA_BUF,N_NUM* AD1_NUM);
			 if(gGlobalData.currentUnitNum>=gGlobalData.unitLen)
			 {
				 gGlobalData.currentUnitNum=0;
					if(gGlobalData.Send_Data_Task==false)
					{
						cnt_heartbag = 0;//心跳包计数器置为0 重新开始计数
						gGlobalData.heart_count = 0;//超时计数器置为0 重新开始计数
						memcpy(&gGlobalData.PlusePressDataSend[10],gGlobalData.PlusePressData,gGlobalData.unitLen*2); //每一个16进制的数据占两个字节
						SendPlusePressData();//采集数据上报一次
					}
			 }
			//装载数据
			if(reTime>=gGlobalData.freqUnitTime)
			{
				reTime=0;
				//gGlobalData.PlusePressData[gGlobalData.currentUnitNum]=(gADC3_VALUE_F[0]>>8)+(gADC3_VALUE_F[0]<<8);
				gGlobalData.PlusePressData[gGlobalData.currentUnitNum] = gADC3_VALUE[0];
				gGlobalData.currentUnitNum++;//采集个数++
				collectDataCount++;//累计采集个数
				if(gGlobalData.currentUnitNum == 50 && Collect_Data_state ==true){     //采集前60个数据不稳定，等稳定后再进行上传
						gGlobalData.currentUnitNum = 0;
						collectDataCount = 0;
						Collect_Data_state = false;
				}
			}else{
				reTime++;
			}
		}
}


/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
	uint8_t version[5];
	SCB->VTOR = FLASH_BANK1_BASE | 0x80000;

	printf("Min Start!!!\r\n");
//new
/* USER CODE BEGIN 1 */
	 MPU_Config();
	
	 SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();
  SCB_DisableDCache();
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
	gGlobalData.current_treatNums = 0;
	gGlobalData.useWorkArg[0].timeTreat = 0;
	gGlobalData.useWorkArg[0].waitTime = 0;
	//gDeviceParam.heartRate = 2;
 
  MakeSinTable(ch1buf, 100, 0, 65535);
	Dac8831_Set_Amp(60, ch1buf);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
    
/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
//	__enable_irq();
  /* USER CODE BEGIN SysInit */
	MX_DMA_Init();
	
	/* Initialize all configured peripherals */ 
  MX_GPIO_Init();

  MX_ADC1_Init();
	MX_ADC3_Init();
//  MX_ETH_Init();
  MX_I2C2_Init();         //屏蔽掉了原始i2c，新写了模拟i2c来控制，i2c有bug
//  IIC_Init();           //2023/8/12 by yls 硬件i2c本身有缺陷不好用，用软件i2c模拟发送数据   8/16 update：还是用的硬件i2c 
  MX_SPI5_Init();
	MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_UART7_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
//  MX_SPI1_Init();
  MX_SPI3_Init();//波形产生
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
	Console_TaskSemaphore_Init();
	
	lcdTask_Semaphore_Init();
  /* USER CODE END 2 */
	MX_I2C1_Init();

  /* USER CODE END SysInit */
	//new end
	Init_All_Parameter();

	gDeviceParam.heartRate=15;    //心跳间隔15s
	gGlobalData.useWorkArg[gGlobalData.current_treatNums].level=0;
	gGlobalData.useWorkArg[gGlobalData.current_treatNums].freqTreat=0;
	gGlobalData.useWorkArg[gGlobalData.current_treatNums].timeTreat=0;
	__enable_irq();
  /* USER CODE BEGIN 2 */
	MX_LWIP_Init();
	send_NetSync(0);
	osDelay(20);
	send_duan_wang(1);	//断网提示框打开 1900
	osDelay(20);
	Send_LcdDevid_id(gDeviceParam.devId);  //发送设备id到屏幕显示
	osDelay(20);
	sprintf(version,"%s",gDeviceParam.version);
	Send_LcdVersion(version,strlen((const char*)version));	 //发送版本号到屏幕显示 
	osDelay(20);	
	Send_Text_SetButton(0,1);
	osDelay(20);

	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_SET);    //中转板切到采集  by yls 2023 6/7 新板子上是PG11
	 
	//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET); //网络灯
	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET); //电源灯绿色
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET);  //运行红灯 set灭 reset亮
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);    //运行绿灯 set灭 reset亮
 	TimerHandle_t xAutoReloadTimer;  
	xAutoReloadTimer = xTimerCreate(	
		"AutoReload",                 /* 名字, 不重要 */
		mainAUTO_RELOAD_TIMER_PERIOD, /* 周期 */
		pdTRUE,                       /* 自动加载 */
		0,                            /* ID */
		vAutoLoadTimerFunc            /* 回调函数 */
	);	
	if (xAutoReloadTimer){
		xTimerStart(xAutoReloadTimer, 0);
	}

	//-----------------------DMASPI发送测试-----------------------//
	DAC8831_Set_Data(0x7fff);       //初始化一下
	//-----------------------音乐测试-----------------------//
//		Send_ComMusic(3);
//-----------------------rtos剩余容量的api初始剩余有44704---------------------------//	
	
	//wifi连接，连接MQTT 接收wifi传递过来的消息
 	osThreadDef(wifiTask, WifiMqttClientTask, osPriorityNormal, 0, 128*5);    //WifiMqttClientTask 线程大概需要占用418字节缓存    之前15
  wifiTaskHandle = osThreadCreate(osThread(wifiTask), NULL);
//	//4G连接，连接MQTT 接收4G传递过来的消息	
	osThreadDef(EC4gTask, EC200MqttClientTask, osPriorityNormal, 0, 128*5); 	//EC200MqttClientTask 线程大概需要占用360字节缓存    之前20
  ec4gTaskHandle = osThreadCreate(osThread(EC4gTask), NULL);
	//业务核心线程，调度处理采集 理疗方案切换等
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128*4);   //StartDefaultTask 线程大概需要占用190字节缓存       之前8                   
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
	
	/*传感器数据采集 温湿度 电池电量等   这个线程里有采集温湿度的  需要打开使用 */
	osThreadDef(startLoopTask, StartLoopTask, osPriorityNormal, 0, 128*4);      //StartLoopTask 线程大概需要占用216字节缓存            之前4
  startLoopTaskHandle = osThreadCreate(osThread(startLoopTask), NULL);
	
	/*串口屏幕数据，串口数据交互*/
  osThreadDef(consoleTask, Console_Task, osPriorityNormal, 0, 128*3);       //Console_Task 线程大概需要占用85字节缓存            之前4  
  consoleTaskHandle = osThreadCreate(osThread(consoleTask), NULL);
	
	/**RJ45网络时 用于接收MQTT传输过来的数据**/
	osThreadDef(mqttRecvClientTask, StartMqttClientTask, osPriorityNormal, 0, 128*15);  //StartMqttClientTask 线程大概需要占用1228字节缓存  之前15
  mqttClientTaskHandle = osThreadCreate(osThread(mqttRecvClientTask), NULL);
	
 	/**RJ45时监测网卡连接状态**/
	osThreadDef(ethernetTask, ethernet_link_thread, osPriorityNormal, 0, 128*3);  //ethernet_link_thread 线程大概需要占用79字节缓存  之前2
  ethernetTaskTaskHandle = osThreadCreate(osThread(ethernetTask), NULL);
 
  /**mqtt 发送数据 发送消息确认任务*/
	osThreadDef(mqttSendTask, StartMqttSendTask, osPriorityNormal, 0, 128*16);  //StartMqttSendTask 线程大概需要占用1296字节缓存   之前10
  mqttSendTaskHandle = osThreadCreate(osThread(mqttSendTask), NULL);
 	/*心跳包发送任务*/
//	osThreadDef(sendheartTask, StartSendheartTask, osPriorityNormal, 0, 128*4); //StartSendheartTask 线程大概需要占用236字节缓存   之前4
//  sendheartTaskHandle = osThreadCreate(osThread(sendheartTask), NULL); 

//-----------------------rtos剩余容量的api最后剩余有  14912  还可以开26*128的线程，但最好省着开  ---------------------------//	

	
	/*串口数据*/
//  osThreadDef(lcdTask, Lcd_Task, osPriorityNormal, 0, 256);
//  lcdTaskHandle = osThreadCreate(osThread(lcdTask), NULL);
 
   /**看门狗任务*/
//  osThreadDef(iwdgTask, StartIwdgTask, osPriorityAboveNormal, 0, 128*2);
//  mqttSendTaskHandle = osThreadCreate(osThread(iwdgTask), NULL);
 
	
  /* Start scheduler */
  osKernelStart(); //vTaskStartScheduler()

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   
  while (1)
  {

	
  }
  /* USER CODE END 3 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t *strstr1;
	 if(huart->Instance == UART4){ //判断是哪个串口发生中断
		strstr1=strstr((const char*)Voice_aRxBuffer,"music:");  //放音乐
		if(strstr1!=NULL)
		{
			Send_PlayMusic(strstr1+6);
		
		}
		else if(Voice_aRxBuffer[0]==0x31){
			switch(Voice_aRxBuffer[1])
			{
				case 0x31:Send_ComMusic(1);
					break;
				case 0x32:Send_ComMusic(2);
					break;
				case 0x33:Send_ComMusic(3);
					break;
				case 0x34:Send_ComMusic(4);
					break;
				case 0x35:Send_ComVolume(Voice_aRxBuffer[2]*10); //音量控制 第三位乘10
					break;
				case 0x36:Send_ComMode(Voice_aRxBuffer[3]);  //发送指令控制播放模式 00:单曲不循环  01:单曲循环播放  02:所有曲目循环播放  03:随机播放
					break;
				case 0x37:Send_ComMusic(2);
					break;
				case 0x38:Send_ComMusic(2);
					break;
				default:
					break;
			}

		}

  HAL_UART_Receive_IT(&huart4, (uint8_t *)Voice_aRxBuffer, Voice_RXBUFFERSIZE); //重新使能接收中断
  }
	 else if(huart->Instance == USART2){
		 if(RecCom2[0] == 0x7E && RecCom2[5] == 0xEF){
			 switch(RecCom2[2]){
				 case 0xA3:
					 if(RecCom2[3] == 0x00 && RecCom2[4] == 0xA7)
						 printf("Play_specified_song:Success\r\n");       
					 else
						 printf("Play_specified_song_Error code:%d\r\n",RecCom2[3]);
					 break;
				case 0xAA:	 
					if(RecCom2[3] == 0x00 && RecCom2[4] == 0xAE)
						printf("Play_Pause:Success\r\n");
					else
						printf("Play_Pause_Error code:%d\r\n",RecCom2[3]);
					break;
				case 0xAB:	 
					if(RecCom2[3] == 0x00 && RecCom2[4] == 0xAF)
						printf("Stop_playing:Success\r\n");
					else
						printf("Stop_playing_Error code:%d\r\n",RecCom2[3]);
					break;
				case 0xAC:	 
					if(RecCom2[3] == 0x00 && RecCom2[4] == 0xB0)
						printf("Next_song:Success\r\n");
					else
						printf("Next_song_Error code:%d\r\n",RecCom2[3]);
					break;
				case 0xAD:	 
					if(RecCom2[3] == 0x00 && RecCom2[4] == 0xB1)
						printf("Previous_song:Success\r\n");
					else
						printf("Previous_song_Error code:%d\r\n",RecCom2[3]);
					break;
				case 0xAE:	 
					if(RecCom2[3] == 0x00 && RecCom2[4] == 0xB2)
						printf("Volume_adjustment:Success\r\n");
					else
						printf("Volume_adjustment_Error code:%d\r\n",RecCom2[3]);
					break;
				default :
					 break; 
			 }	 
			}
			memset(RecCom2,0,sizeof(RecCom2));
			HAL_UART_Receive_IT(&huart2, RecCom2,COM2_REC_MAX); 
	 }

}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}
 
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();
  /** Initializes and configures the Region and the memory to be protected 
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /** Initializes and configures the Region and the memory to be protected 
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30044000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV64;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;//ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;//ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5 ;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}


/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;//ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel =ADC_CHANNEL_5;                        //ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}
#if 0
/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}
#endif
/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0xF010F2FF;     // 0x307075B1  100khz        25khz：0xC0206FFF(Cube)  0xF07075B1(自己改)    15KHZ:0xF010F2FF
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_DeInit(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x307075B1;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;//SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;      								//硬件控制
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4 ;  //速度160M/4=40M速度      4分频刚好控制  50M的dac8831
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;  					/* 使能脉冲输出 */
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;    			/* 低电平有效 */
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_07DATA;   /* 设置FIFO大小是一个数据项 */
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;       							  /* MSS, 插入到NSS有效边沿和第一个数据开始之间的额外延迟，单位SPI时钟周期个数 */
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;  			  /* MIDI, 两个连续数据帧之间插入的最小时间延迟，单位SPI时钟周期个数 */
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;  									  /* 禁止SPI后，SPI相关引脚保持当前状态 */ 
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	
//	hspi3.Instance               = SPI3;                   /* 例化SPI */
//	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;     /* 设置波特率 */
//	hspi3.Init.Direction         = SPI_DIRECTION_2LINES;   /* 全双工 */
//	hspi3.Init.CLKPhase          = SPI_PHASE_1EDGE;              /* 配置时钟相位 */
//	hspi3.Init.CLKPolarity       = SPI_POLARITY_LOW;           /* 配置时钟极性 */
//	hspi3.Init.DataSize          = SPI_DATASIZE_8BIT;      /* 设置数据宽度 */
//	hspi3.Init.FirstBit          = SPI_FIRSTBIT_MSB;       /* 数据传输先传高位 */
//	hspi3.Init.TIMode            = SPI_TIMODE_DISABLE;     /* 禁止TI模式  */
//	hspi3.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE; /* 禁止CRC */
//	hspi3.Init.CRCPolynomial     = 7;                       /* 禁止CRC后，此位无效 */
//	hspi3.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;     /* 禁止CRC后，此位无效 */
//	hspi3.Init.NSS               = SPI_NSS_SOFT;               /* 使用软件方式管理片选引脚 */
//	hspi3.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_16DATA;  /* 设置FIFO大小是一个数据项 */
//	hspi3.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;      /* 禁止脉冲输出 */
//	hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE; /* 禁止SPI后，SPI相关引脚保持当前状态 */  
//	hspi3.Init.Mode 			 	= SPI_MODE_MASTER;            /* SPI工作在主控模式 */
	
	if (HAL_SPI_DeInit(&hspi3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_SPI_Init(&hspi3) != HAL_OK)
	{
		Error_Handler();
	}

  


  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 0x0;
  hspi5.Init.NSSPMode =SPI_NSS_PULSE_DISABLE;// SPI_NSS_PULSE_ENABLE;
  hspi5.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi5.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi5.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi5.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi5.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi5.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi5.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */
  HAL_UART_Receive_IT(&huart4, (uint8_t *)Voice_aRxBuffer, Voice_RXBUFFERSIZE);
//  __HAL_UART_ENABLE_IT(&huart4,UART_IT_IDLE);                //空闲中断使能
//  __HAL_UART_ENABLE_IT(&huart4,UART_IT_RXNE);                //接收中断使能
 /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
	HAL_UART_Receive_IT(&huart2, RecCom2,COM2_REC_MAX);
  /* USER CODE END USART2_Init 2 */

}


/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{
   /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	
	 /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);   //WIFI
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);   //4G
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);   //SPI3
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);


}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
	
	/*spi5 cs PF6*/
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOF, IO7_Pin, GPIO_PIN_RESET);
	
  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOH, RMII_INT_Pin|RMII_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, WIFI_RESET_Pin|WIFI_EN_Pin|LED1_Pin
                          |LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, IO1_Pin|IO2_Pin|IO3_Pin|IO4_Pin|IO5_Pin|IO6_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, IO8_Pin|IO9_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOH, IO10_Pin|IO11_Pin|IO12_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOI, IO13_Pin|IO14_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOI, IO13_Pin|IO14_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_EN2_GPIO_Port, RS485_EN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
  
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);


  /*Configure GPIO pins : RMII_INT_Pin RMII_RES_Pin */
//  GPIO_InitStruct.Pin = RMII_INT_Pin|RMII_RES_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : WIFI_RESET_Pin WIFI_EN_Pin LED1_Pin LED2_Pin
                           LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = WIFI_RESET_Pin|WIFI_EN_Pin|LED1_Pin|LED2_Pin
                          |LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SOCKET_LINK_Pin WIFI_LINK_Pin */
  GPIO_InitStruct.Pin = SOCKET_LINK_Pin|WIFI_LINK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = IO1_Pin|IO2_Pin|IO4_Pin|IO3_Pin|IO5_Pin|IO6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = IO7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = IO8_Pin|IO9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = IO10_Pin|IO11_Pin|IO12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = IO13_Pin|IO14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOI, IO13_Pin, GPIO_PIN_RESET);
	
  /*Configure GPIO pin : RS485_EN2_Pin */
  GPIO_InitStruct.Pin = RS485_EN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RS485_EN2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN2_Pin */
  GPIO_InitStruct.Pin = IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IN2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN1_Pin */
  GPIO_InitStruct.Pin = IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN3_Pin */
  GPIO_InitStruct.Pin = IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IN3_GPIO_Port, &GPIO_InitStruct);	 
	
	/*Configure GPIO pin : IN4_Pin */
  GPIO_InitStruct.Pin = IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IN4_GPIO_Port, &GPIO_InitStruct);	
	
  /*Configure GPIO pin : RS485_EN_Pin */
  GPIO_InitStruct.Pin = RS485_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RS485_EN_GPIO_Port, &GPIO_InitStruct);
	
	/*spi5 cs PF6*/
	  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	//4g
	GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
	//PG11是中转板转治疗
	GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

//音频切换    默认
	GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
//12v风扇
	GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_5, GPIO_PIN_SET);
	
}

/* USER CODE BEGIN 4 */
void Set_Input_Output(uint8_t Flag)
{
	if(sON==Flag)
	 {
//		 taskENTER_CRITICAL(); 
		 __disable_irq(); 
		 set_channle_status(gGlobalData.useWorkArg[0].outs[gGlobalData.channelPos],DIR_OUT,sON); 
		 __enable_irq();         
//   	taskEXIT_CRITICAL(); 
        
     if(gGlobalData.curWorkMode == WORK_MODE_ZL)
			 osDelay(50);
     __disable_irq();                  
		 set_channle_status(gGlobalData.useWorkArg[0].inputs[gGlobalData.channelPos],DIR_IN,sON);           		 
     __enable_irq();
 	 }
	 else if(sOFF==Flag)
   {		 
			__disable_irq();                 
			set_channle_status(gGlobalData.useWorkArg[0].outs[gGlobalData.channelPos],DIR_OUT,sOFF);		 						           
		  __enable_irq();
			if(gGlobalData.curWorkMode == WORK_MODE_ZL)
					osDelay(50);         
			__disable_irq();
			set_channle_status(gGlobalData.useWorkArg[0].inputs[gGlobalData.channelPos],DIR_IN,sOFF);
			__enable_irq();
	 }
}

void Set_I2c_Register_Clear_Reset(void)
{

//    hi2c2.Instance->CR1             &= 0;
//    hi2c2.Instance->CR2             &= 0;
//    hi2c2.Instance->ICR             &= 0;
//    hi2c2.Instance->ISR             |= 1<<0;
//    hi2c2.Instance->OAR1            &= 0;
//    hi2c2.Instance->OAR2            &= 0;
//    hi2c2.Instance->PECR            &= 0;
//    hi2c2.Instance->RXDR            &= 0;
//    hi2c2.Instance->TIMEOUTR        &= 0;
//    hi2c2.Instance->TIMINGR         &= 0;
//    hi2c2.Instance->TXDR            &= 0;
    
    I2C2->CR1       &= 0;
    I2C2->CR2       &= 0;
    I2C2->ICR       &= 0;
    I2C2->ISR       |= 1<<0;
    I2C2->ISR       &= 1<<0;
    I2C2->OAR1      &= 0;
    I2C2->OAR2      &= 0;
    I2C2->PECR      &= 0;
    I2C2->RXDR      &= 0;
    I2C2->TIMEOUTR  &= 0;
    I2C2->TIMINGR   &= 0;
    I2C2->TXDR      &= 0;
    
    MX_I2C2_Init();
    return;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */

void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  uint8_t gbk0[8]={0,};
	uint8_t gbkNum0[13]={0,};
	
	int16_t collectTime=0,unitTime=0;
	/*adc 校准**/
	if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED)!=HAL_OK)	//阻塞方式
	{
		Error_Handler();
	}
	
	if(HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED)!=HAL_OK)
	{
		Error_Handler();
	}
	/*等待校准结束*/
	osDelay(10);	//可以删掉  vTaskDelay()
	
	/*开始dma接收adc，dma满产生中断*/
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)gADC1_DMA_BUF,N_NUM* AD1_NUM);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t *)gADC3_DMA_BUF,N_NUM* AD3_NUM);
	
	/*获取内部参考电压*/
	osDelay(50);	//等待第一轮转换结束
	gREFVOL_VAL=__LL_ADC_CALC_VREFANALOG_VOLTAGE(gADC3_VALUE[0],LL_ADC_RESOLUTION_16B);
	if(gREFVOL_VAL>3500||gREFVOL_VAL<3000)
	{
		gREFVOL_VAL=3300;
	}
	for(;;)
  { 
//		printf("StartMqttClientTask size = %d\r\n",(int32_t)uxTaskGetStackHighWaterMark(NULL));
 		osDelay(10);
		collectTime++;
		if(collectTime>=1)	//每10ms更新一次
		{
			
			collectTime=0;
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *)gADC1_DMA_BUF,N_NUM* AD1_NUM);
			HAL_ADC_Start_DMA(&hadc3, (uint32_t *)gADC3_DMA_BUF,N_NUM* AD3_NUM);//直流采样
		}
		if(gDeviceParam.devLock==true)
		{
			continue;
		} 
		
		//判断当前运行状态
		if(gGlobalData.curWorkMode!=gGlobalData.oldWorkMode)
		{
 			if(gGlobalData.curWorkState==WORK_START)
			{
				gGlobalData.oldWorkMode=gGlobalData.curWorkMode;
				//初始化当前状态
				gGlobalData.channelPos=0;
				gGlobalData.currentUnitNum=0;
				switch(gGlobalData.curWorkMode)
				{
					case WORK_MODE_WT:
						channelTime=0;
						unitTime=0;
						set_sampleMode(MODE_CLOSE);
//						HAL_PCA9554_outputAll(0);
						break;
					case WORK_MODE_ZT:
						gGlobalData.unitLen=gGlobalData.useWorkArg[0].rateN*gGlobalData.useWorkArg[0].upTime/1000;      
						gGlobalData.freqUnitTime=1000/gGlobalData.useWorkArg[0].rateN;
				
						if(gGlobalData.unitLen<1)
						{
								gGlobalData.unitLen=1;
						}
						if(gGlobalData.freqUnitTime<1)
						{
								gGlobalData.freqUnitTime=1;
						}
						channelTime=0;
						unitTime=0;
						//继电器设置为采样模式
						set_sampleMode(MODE_ZT);  

						HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_SET);    //中转板切到采集  by yls 2023 6/7 新板子上是PG11
                        
                        //输出打开 //输入打开 
						Set_Input_Output(sON);   //采集
 
						
						//gGlobalData.curWorkState = WORK_PAUSE;
						HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_SET); //运行红灯 set灭 reset亮
						HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET); //运行绿灯 set灭 reset亮
						send_LcdWorkStatus(3);//kardos 2023.02.03 修改设备工作状态为：经络检测中
            Collect_Data_state = true;
						break;
					case WORK_MODE_ZL:
						unitTime=0;						
						//治疗模式
						set_sampleMode(MODE_ZL);
						osDelay(500);

						Wave_select(gGlobalData.useWorkArg[0].waveTreat, ch1buf);									                  //波形选择
						Dac8831_Set_Amp(gGlobalData.useWorkArg[0].level, ch1buf);									                  //幅值改变
			
						DAC8831_Set_Data_Dma(ch1buf,sizeof(ch1buf)/2,gGlobalData.useWorkArg[0].freqTreat);					//定时器开启产生波形
					
						send_treatSel(gGlobalData.useWorkArg[0].freqTreat,
									gGlobalData.useWorkArg[0].level,
								 (gGlobalData.useWorkArg[0].timeTreat)/60);	 													                  //这一句是发送jly下发治疗方案给屏幕
						osDelay(100);
						Send_LcdVoltage(5.84*gGlobalData.useWorkArg[0].level);	
						osDelay(100);
						HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_RESET);																		  //中转板切到治疗	
                    
						Set_Input_Output(sON);   //治疗 输出打开  //输入打开

						channelTime=0;	//开始计时
						osDelay(200);
						send_LcdWorkStatus(4);//kardos 2023.02.03 修改设备工作状态为：经络理疗中
						osDelay(100);
						send_LcdOutStatus(1);
					break;

					default:
						break;
				}
			}
		}
		else
		{
			switch(gGlobalData.curWorkState)
			{
				case WORK_START:

						if(gGlobalData.curWorkMode==WORK_MODE_ZT && gGlobalData.isConnect==1)     //有网才进行采集
						{
                isCollect=true;                                
								//判断采集时间是否到，且不是最后一个通道，切换下一通道,并更新通道位置、重置包位置计数、时间计数
//								if(channelTime>=gGlobalData.useWorkArg[0].timeCheck*1000)
								if(collectDataCount>(gGlobalData.useWorkArg[0].timeCheck*gGlobalData.useWorkArg[0].rateN))     //计数可以多计数一次才切换保证数据完整性
								{
							    gGlobalData.Alltime = gGlobalData.Alltime - gGlobalData.useWorkArg[0].timeCheck;//倒计时减1 2023.04.04 kardos
									Countdown_Treat(gGlobalData.Alltime);//刷新倒计时
									if(gGlobalData.channelPos>=(gGlobalData.useWorkArg[0].chanelNum-1))
									{  
										isCollect=false;
										set_sampleMode(MODE_CLOSE);
										HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_RESET);//切到治疗
										yd_4gfalg=false ;
										if(gGlobalData.netKind!=1)
										osDelay(1000);          //wifi 4g延时1s后才采集结束，不然可能上传数据报错收不到治疗方案
 
										gGlobalData.channelPos = 0;

										HAL_PCA9554_outputAll(0);//先关闭所有继电器
										yd_4gfalg=true ;
										gGlobalData.curWorkMode =WORK_MODE_WT;
										gGlobalData.oldWorkMode=gGlobalData.curWorkMode;
										taskENTER_CRITICAL();
										Send_Fix_Ack(23,STATUS_OK,"ZT Caiji ok");  //整体采集结束 
										collectDataCount=0;//采集结束计数设置为0
										taskEXIT_CRITICAL();
									}else{
										isCollect=false;
//										gGlobalData.currentUnitNum=0;           //避免导致切换通道导致数据下发不全
										collectDataCount=0;//设置为0
										/*关闭上一次的采集通道*/
                                        
                    Set_Input_Output(sOFF);     //采集
								
										gGlobalData.channelPos++;
										/*切换为下一通道*/
										//输出打开  //输入打开
                                        
                    Set_Input_Output(sON);       //采集
                                        
										unitTime=0;
										osDelay(10);//通道切换后做一个延时 让数据稳定后再采集
										Collect_Data_state = true;
										isCollect=true;
									}
								channelTime=0;//最后设置采集通道时间为0 否则程序处理会占用时间
								}
						}
						else if(gGlobalData.curWorkMode==WORK_MODE_ZL)
						{		//2023.02.01			 		
								//channelTime++;
							
							//最后一个治疗方案，当档位大于10且剩余最后2分钟，每4秒下降一个档位，体感逐步减弱 2023.02.06 kardos
							if(gGlobalData.channelPos>=gGlobalData.useWorkArg[0].chanelNum-1)//说明是最后一个
							{
							  if(((gGlobalData.useWorkArg[gGlobalData.current_treatNums].timeTreat + gGlobalData.useWorkArg[gGlobalData.current_treatNums].waitTime)*1000-channelTime)<=120000)//说明是最后60秒 开始降低档位
								{
									if(channelTime-countTime>=4000&&gGlobalData.useWorkArg[gGlobalData.current_treatNums].level>10){
										gGlobalData.useWorkArg[gGlobalData.current_treatNums].level-=1;
								  	Wave_select(gGlobalData.useWorkArg[gGlobalData.current_treatNums].waveTreat, ch1buf);																		//波形选择
										Dac8831_Set_Amp(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level, ch1buf);																		//幅值直接给成0
										
										DAC8831_Set_Data_Dma(ch1buf,sizeof(ch1buf)/2,gGlobalData.useWorkArg[gGlobalData.current_treatNums].freqTreat);					//定时器开启产生波形
										
										countTime=channelTime;
										
										send_treatSel(gGlobalData.useWorkArg[gGlobalData.current_treatNums].freqTreat,
										gGlobalData.useWorkArg[gGlobalData.current_treatNums].level,
										(gGlobalData.useWorkArg[gGlobalData.current_treatNums].timeTreat)/60);	//这一句是发送jly切换治疗方案给屏幕
										osDelay(100);
										if(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level <= 60)
											Send_LcdVoltage(5.84*gGlobalData.useWorkArg[gGlobalData.current_treatNums].level);	
									}
								}
							}
							
								//判断治疗时间是否到，且不是最后一个通道，切换下一通道,并更新通道位置、重置包位置计数、时间计数
								if(channelTime>=(uint32_t)(gGlobalData.useWorkArg[gGlobalData.current_treatNums].timeTreat + gGlobalData.useWorkArg[gGlobalData.current_treatNums].waitTime)*1000)
								{
									//先判断是否是最后一个治疗方案时间到了
									if(gGlobalData.channelPos>=gGlobalData.useWorkArg[0].chanelNum-1)
									{//说明最后一个治疗方案时间到了 关闭所有
										//先将档位调整到0 防止最后打一下
										Wave_select(gGlobalData.useWorkArg[gGlobalData.current_treatNums].waveTreat, ch1buf);		//波形选择
										Dac8831_Set_Amp(0, ch1buf);																															//幅值直接给成0
										HAL_TIM_Base_DeInit(&htim12);  																												  //不产生波形
										osDelay(200);		
										send_LcdWorkStatus(6);//kardos 2023.02.03 修改设备工作状态为：                                              经络理疗完成
										gGlobalData.curWorkMode=WORK_MODE_WT;
										gGlobalData.oldWorkMode=gGlobalData.curWorkMode;
										gGlobalData.curWorkState=WORK_STOP;
										set_sampleMode(MODE_CLOSE);
										HAL_PCA9554_outputAll(0);
										HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET); 																		//运行红灯 set灭 reset亮
										HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET); 																			//运行绿灯 set灭 reset亮
										//2023.02.01	WWJZ
                    gGlobalData.useWorkArg[gGlobalData.current_treatNums].level=0;
										gGlobalData.Alltime=0;
										send_LcdSync(0);																																				//修改运行状态
										send_LcdOutStatus(0);                                                                   //状态灯
										osDelay(100);		
										send_LcdWorkStatus(1);																																	//kardos 2023.02.03 修改设备工作状态为：设备待机状态 
										osDelay(100);			
										send_treatSel(0,0,0);     																															//jly治疗方案全置成0	
										osDelay(100);										
										Send_LcdVoltage(0);	
										osDelay(100);	
										send_visitNumber(gbkNum0);																															//修改就诊人编号
										osDelay(100);
										send_visitName(gbk0,8);   																															//修改就诊人姓名
										osDelay(100);
										send_visitAge(0);         																															//修改就诊人年龄
										osDelay(100);
										send_visitSex(0);         																															//修改就诊人性别
									  //2023.02.01	WWJZ
                    gGlobalData.cur_heart_state=LEISURE; 																									  //治疗结束后复位状态
										Send_Fix_Ack(24,STATUS_OK,"OK");
										gGlobalData.current_treatNums=0;
										gGlobalData.channelPos=0;
										osDelay(100);
										send_LcdWorkStatus(1);																																	//kardos 2023.02.03 修改设备工作状态为：设备待机状态
										countTime=0;																																						//归零
									}else{
									//说明时间到了但是不是最后一个治疗方案，切下一个治疗方案
										Set_Input_Output(sOFF);    //治疗
		             
										gGlobalData.channelPos++;
										osDelay(1000);
									//切换波形
										gGlobalData.current_treatNums++;				
										Wave_select(gGlobalData.useWorkArg[gGlobalData.current_treatNums].waveTreat, ch1buf);//波形选择
										Dac8831_Set_Amp(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level, ch1buf);//幅值改变
										
										DAC8831_Set_Data_Dma(ch1buf,sizeof(ch1buf)/2,gGlobalData.useWorkArg[gGlobalData.current_treatNums].freqTreat);					//定时器开启产生波形										
										
										send_treatSel(gGlobalData.useWorkArg[gGlobalData.current_treatNums].freqTreat,
											gGlobalData.useWorkArg[gGlobalData.current_treatNums].level,
											(gGlobalData.useWorkArg[gGlobalData.current_treatNums].timeTreat)/60);	//这一句是发送jly切换治疗方案给屏幕
										osDelay(100);
										if(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level <= 60)
											Send_LcdVoltage(0.584*gGlobalData.useWorkArg[gGlobalData.current_treatNums].level);										

										osDelay(200);
										/*切换为下一通道*/
										//输出打开   //输入打开
                                        
										Set_Input_Output(sON);      //治疗
                                        
										channelTime=0;										
									}
								}
								//2023.02.01  
 					}
					break;
				case WORK_PAUSE:	  	 
 					break;
				case WORK_STOP:	      
						 channelTime=0;
					break;
				
				default:
					break;
			}
		}
	}//for
  /* USER CODE END 5 */
}
 
void StartLoopTask(void const * argument)
{
  /* USER CODE BEGIN StartLoopTask */
  /* Infinite loop */
	HAL_PCA9554_outputAll(0);
	HAL_PCA9554_init();
		
	startLoopTask(argument);
  /* USER CODE END StartLoopTask */
}



void Console_Task(void const * pvParameters)
{
	uint8_t gbk0[8]={0,};
	uint8_t gbkNum0[13]={0,};
		HAL_UART_Receive_DMA(&huart7, RecSen,30);
	__HAL_UART_CLEAR_IDLEFLAG(&huart7);
	__HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);

	for(;;)
	{
 		
		osDelay(10);
		xSemaphoreTake(ConsoleReceive_Semaphore,portMAX_DELAY);
//		5a a506 83 1800 00 01 00 02
  
		if(RecSen[0] == 0x5A && RecSen[1] == 0xA5)
			{
					if(RecSen[3] == 0x83) //wifi设置相关
					{
						if(RecSen[4] == 0x18 && RecSen[5] == 0x20)
						{
							//存wifi名称
							memset(gDeviceParam.wifiArg.WifiName,0,sizeof(gDeviceParam.wifiArg.WifiName));
							for(uint8_t i = 7;i < sizeof(gDeviceParam.wifiArg.WifiName); ++i)
							{
								if(RecSen[i] == 0xFF)
									break;
								gDeviceParam.wifiArg.WifiName[i-7] = RecSen[i];
							}
						}
						if(RecSen[4] == 0x18 && RecSen[5] == 0x30)
						{
							//存wifi密码
							memset(gDeviceParam.wifiArg.WifiPwd,0,sizeof(gDeviceParam.wifiArg.WifiPwd));
							for(uint8_t i = 7;i < sizeof(gDeviceParam.wifiArg.WifiPwd); ++i)
							{
								if(RecSen[i] == 0xFF)
									break;
								gDeviceParam.wifiArg.WifiPwd[i-7] = RecSen[i];
							}
						}
						if(RecSen[4] == 0x19 && RecSen[5] == 0x10)  //屏幕提示框按键后便不可用
						{
								Send_Text_SetButton(0,0);	//文本框按键默认不可用
								osDelay(50);

						}
					}
				 if(RecSen[2] == 0x06)
				 {
					 switch(RecSen[8])
						{
							case 0x01:   //开始按钮
									if(gGlobalData.curWorkState != WORK_START)
									{	
										gGlobalData.cur_heart_state=WORKING;
										gGlobalData.curWorkState=WORK_START;
											//2023.1.31 WWJZ
										if(gGlobalData.curWorkMode==WORK_MODE_ZL){//处理在治疗的情况下暂停后再次开始
                                       
                                                set_sampleMode(MODE_ZL);
												osDelay(200);
												send_LcdWorkStatus(4);//kardos 2023.02.03 修改设备工作状态为：经络理疗中	
												send_LcdOutStatus(1);											 
										 }//2023.1.31 WWJZ
										 osDelay(200);
										 send_LcdSync(1);  //修改屏幕运行状态
										
										if(gGlobalData.curWorkMode==WORK_MODE_ZT){
											osDelay(200);	
											send_LcdWorkStatus(3);//kardos 2023.02.03 修改设备工作状态为：经络检测中			
										
											set_sampleMode(MODE_ZT);
											osDelay(200);                                            
											isCollect=true;
											break;
										}

                                        
											cnt_heartbag = 0;												//发送心跳清空心跳计数器
											gGlobalData.heartbag_flage = 1;
									 }
									break;
							case 0x02:  //暂停按钮
									if(gGlobalData.curWorkState == WORK_START){
											gGlobalData.cur_heart_state=PAUSE;
											gGlobalData.curWorkState=WORK_PAUSE;
											set_sampleMode(MODE_CLOSE);
								 
											 osDelay(20);
											 send_LcdSync(0); 
											 osDelay(200);
											 send_LcdWorkStatus(5);//kardos 2023.02.03 修改设备工作状态为：设备暂停状态
											 send_LcdOutStatus(0);
											 if(gGlobalData.curWorkMode == WORK_MODE_ZT)
											 {
													 isCollect=false;//2023.03.31 ZKM
													 Collect_Data_state =true;
											 }
													 
											 cnt_heartbag = 0;												//发送心跳清空心跳计数器
											gGlobalData.heartbag_flage = 1;
									}
								break;
							case 0x03://复位按钮
//									Set_Input_Output(sOFF);                       //多了一句复位先寻址关闭上一次的两通道
									Send_Fix_Ack(100,STATUS_OK,"OK");//发送给上位机告诉执行了复位
									gGlobalData.cur_heart_state=LEISURE;
									gGlobalData.curWorkState=WORK_STOP; 
                                    collectDataCount=0;
                                    gGlobalData.useWorkArg[gGlobalData.current_treatNums].level=0;  //复位后挡位调节为0
									gGlobalData.current_treatNums=0;//2023.02.01
                                   
									set_sampleMode(MODE_CLOSE);
 
									HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_SET);    //中转板切到采集  by yls 2023 6/7 新板子上是PG11
									gGlobalData.channelPos=0;
									HAL_PCA9554_outputAll(0);
									HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET); //运行红灯 set灭 reset亮
									HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET); //运行绿灯 set灭 reset亮
									gGlobalData.curWorkMode=WORK_MODE_WT;
									gGlobalData.oldWorkMode=gGlobalData.curWorkMode;
									gGlobalData.curWorkState=WORK_STOP;
									osDelay(20); 
									send_LcdWorkStatus(1);//kardos 2023.02.03 修改设备工作状态为：设备待机状态
									osDelay(100);
									gGlobalData.Alltime=0; //倒计时清空
									Countdown_Treat(gGlobalData.Alltime);//刷新倒计时
									osDelay(100);
									send_treatSel(0,0,0);
									osDelay(100);
									Send_LcdVoltage(0);
									osDelay(100);
									send_visitNumber(gbkNum0);
									osDelay(100);
									send_visitName(gbk0,8);
									osDelay(100);
									send_visitAge(0);
									osDelay(100);
									send_visitSex(0);
									osDelay(100);	
									send_LcdSync(0);
									osDelay(100);
									Send_ComMusic(3);
									osDelay(100);
									send_lcdQRcode();
									send_LcdOutStatus(0);
									isCollect=false;//2023.03.31 ZKM
									HAL_TIM_Base_DeInit(&htim12);  //不产生波形
									break;
							case 0x05://档位加

								if(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level <=60){
									gGlobalData.useWorkArg[gGlobalData.current_treatNums].level+=1;
									send_treatSel(gGlobalData.useWorkArg[gGlobalData.current_treatNums].freqTreat,
												gGlobalData.useWorkArg[gGlobalData.current_treatNums].level,
												(gGlobalData.useWorkArg[gGlobalData.current_treatNums].timeTreat)/60);
									osDelay(100);
									if(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level <= 60)
										Send_LcdVoltage(5.84*gGlobalData.useWorkArg[gGlobalData.current_treatNums].level);	
									Wave_select(gGlobalData.useWorkArg[gGlobalData.current_treatNums].waveTreat, ch1buf);//波形选择
									Dac8831_Set_Amp(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level, ch1buf);//幅值改变
								}
								Dac_level_CTL(1);   //档位改变后波形产生
								osDelay(500);//延时 防止点击过快
								cnt_heartbag = 0;												//发送心跳清空心跳计数器
								gGlobalData.heartbag_flage = 1;
								break;
								//level  -5
							case 0x06://档位减

								if(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level >=5){
									gGlobalData.useWorkArg[gGlobalData.current_treatNums].level-=1; 
									send_treatSel(gGlobalData.useWorkArg[gGlobalData.current_treatNums].freqTreat,
											gGlobalData.useWorkArg[gGlobalData.current_treatNums].level,
											(gGlobalData.useWorkArg[gGlobalData.current_treatNums].timeTreat)/60);
									osDelay(100);
									if(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level <= 60)
										Send_LcdVoltage(5.84*gGlobalData.useWorkArg[gGlobalData.current_treatNums].level);
									Wave_select(gGlobalData.useWorkArg[gGlobalData.current_treatNums].waveTreat, ch1buf);//波形选择
									Dac8831_Set_Amp(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level, ch1buf);//幅值改变
								}
								Dac_level_CTL(1);   //档位改变后波形产生
								osDelay(500);//延时 防止点击过快
								cnt_heartbag = 0;												//发送心跳清空心跳计数器
							  gGlobalData.heartbag_flage = 1;
								break;
							case 0x0a:
								gGlobalData.ResetDevidcnt++;
								break;
							case 0x50://wifi更改确认指令
								gGlobalData.Wifi_set = true;
								Save_Parameter();
								printf("wifi_go_to_con\r\n");
								send_lcdPage(0);
								break;
							default: 
								break;		
						}
						memset (RecSen,0,sizeof(RecSen));
					}

			}

		memset (RecSen,0,sizeof(RecSen));
		HAL_UART_Receive_DMA(&huart7, RecSen,30); //2023.02.03

	}//for循环
}

bool Get_ADC1_Hex(void)	
{
	//static uint16_t pt[AD1_NUM*10];
	int i,j ;
	uint16_t temp[N_NUM];	//连续采样次数
	//if(Read_All_AD1(pt,AD1_NUM*10) == false) return false;	//DMA一直采集
	
	for(i=0;i<AD1_NUM;i++)
	{
		for(j=0;j<N_NUM;j++) {temp[j] =  gADC1_DMA_BUF[j*AD1_NUM+i];} //取出每一次的值
		gGlobalData.currentNet=Gets_Average(temp,N_NUM);	
		*(gADC1_VALUE+i) = Gets_Average(temp,N_NUM);	//计算平均值
		*(gADC1_VALUE_F+i) = ((*(gADC1_VALUE+i))*gREFVOL_VAL)/65535;   //gREFVOL_VAL
	}
	return true;
}

/*out*/
bool Get_ADC3_Hex(void)
{
	//static uint16_t pt[AD1_NUM*10];
	int i,j ;
	uint16_t temp[N_NUM];
	//if(Read_All_AD1(pt,AD1_NUM*10) == false) return false;	//DMA一直采集
	
	for(i=0;i<AD3_NUM;i++)
	{
		for(j=0;j<N_NUM;j++) {temp[j] =  gADC3_DMA_BUF[j*AD3_NUM+i];} 
		
		*(gADC3_VALUE+i) = Gets_Average(temp,N_NUM);
		//*(gADC3_VALUE_F+i) = ((*(gADC3_VALUE+i))*gREFVOL_VAL)/65535;
		
	}
	return true;
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
	

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	static uint8_t val[2];
//	static uint16_t Vol_De;
//	static int16_t Vol;
	if(GPIO_Pin==GPIO_PIN_2)	//f2
	{	
	}
	
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}


#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xFFFF);       //不能重定向串口3 和 串口6 和串口7 
  return ch;
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

//初始化独立看门狗
//prer:分频数:0~7(只有低3位有效!)
//rlr:自动重装载值,0~0XFFF.
//分频因子=4*2^prer.但最大值只能是256!
//rlr:重装载寄存器值:低11位有效.
//时间计算(大概):Tout=((4*2^prer)*rlr)/32 (ms).
void IWDG_Init(uint8_t prer,uint16_t rlr)
{
    IWDG_Handler.Instance=IWDG1;
    IWDG_Handler.Init.Prescaler=prer;    //设置IWDG分频系数
    IWDG_Handler.Init.Reload=rlr;        //重装载
    IWDG_Handler.Init.Window=IWDG_WINDOW_DISABLE;//关闭窗口功能
    HAL_IWDG_Init(&IWDG_Handler);    
}
//喂独立看门狗
void IWDG_Feed(void)
{   
    HAL_IWDG_Refresh(&IWDG_Handler); //重装载
}

void StartIwdgTask(void const *arg)
{
     EventBits_t uxBits;
     const TickType_t xTicksToWait = 2000 / portTICK_PERIOD_MS; /* 最大延迟2000ms */
    /*
      开始执行启动任务主函数前使能独立看门狗。
      设置LSI是64分频，下面函数参数范围0-0xFFF，分别代表最小值2ms和最大值6552ms

       设置LSI是128分频，下面函数参数范围0-0xFFF，分别代表最小值4ms和最大值13104ms
       Tout = prv/40 * rlv (s)
      下面设置的是5s，如果5s内没有喂狗，系统复位。
    */
    IWDG_Init(0x00000004 ,625*5);
    while(1)
    {
        /* 等待所有任务发来事件标志 */
        uxBits = xEventGroupWaitBits(xCreatedEventGroup, /* 事件标志组句柄 */
                     TASK_BIT_ALL,       /* 等待TASK_BIT_ALL被设置 */
                     pdTRUE,             /* 退出前TASK_BIT_ALL被清除，这里是TASK_BIT_ALL都被设置才表示“退出”*/
                     pdTRUE,             /* 设置为pdTRUE表示等待TASK_BIT_ALL都被设置*/
                     xTicksToWait);      /* 等待延迟时间 */
        if((uxBits & TASK_BIT_ALL) == TASK_BIT_ALL)//判断各个任务是否执行
        {
            IWDG_Feed();
        }
    }
        
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
