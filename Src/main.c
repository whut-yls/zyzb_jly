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
#include "I2C.h"//�������iic.h

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

uint8_t       RecSen[30];   //������Ļ��������
uint32_t channelTime = 0;
bool gUartPcInit=false,gUartPcTc=false;
bool gUartLcdInit=false,gUartLcdTc=false;
	////////////////////////2023.03.24  kardos/////////////////////////////////////////////
bool isCollect=false;//�Ƿ�ʼ�ɼ�����
uint32_t 	collectDataCount;//�ɼ����ݸ���
static int reTime=0;//�ɼ�ʱ��������λms	
	//////////////////////////////////////////////////////////////////////////////////////
bool yd_4gfalg;
uint32_t countTime = 0;//�����������ڵ���ʱ���� 2023.02.06 kardos
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
//bool gEthSendStatus=false;  //���ݶ˿�״̬
bool gEthRecvStatus=false;  //���ƶ˿�״̬
//bool gEthRecvReturn=false;  //���ƶ˿�״̬ �Ƿ��л�ִ
//bool gEthSendReturn=false;  //�������˿� �Ƿ��л�ִ
int cnt_heartbag = 0;
bool gEthStatus=false;      //����״̬
bool gRecvEthChange=false;  //״̬�ı��ʶ
bool gWifiChange;
//bool gSendEthChange=false;  //״̬�ı��ʶ
//bool gTransferDon=false;    //�ź����ȴ�״̬
void StartDefaultTask(void const * argument);

bool gMqttLinkStatus=false;      //mq����״̬
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

IWDG_HandleTypeDef IWDG_Handler; //�������Ź����
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
/* �����ʱ��*/
uint16_t time_ = 1; //ms

//static volatile uint8_t flagAutoLoadTimerRun = 0;
static void vAutoLoadTimerFunc( TimerHandle_t xTimer );
#define mainAUTO_RELOAD_TIMER_PERIOD pdMS_TO_TICKS( time_ )
uint16_t  ch1buf[100];
static int count1 = 0;
static int cnt_down = 0;
static int Resetcnt = 0;

//int debug_a = 0;
//�����ʱ���ص�����
static void vAutoLoadTimerFunc( TimerHandle_t xTimer )
{
//	xEventGroupSetBits(xCreatedEventGroup, TASK_BIT_1);//ι��
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
	//���Ƽ�ʱ
	if(cnt_down == 1000 && gGlobalData.curWorkMode == WORK_MODE_ZL && gGlobalData.curWorkState == WORK_START)//һ�����һ��
	{
		if((gGlobalData.Alltime != 0 && gGlobalData.curWorkState == WORK_START))
		{
			gGlobalData.Alltime = gGlobalData.Alltime - 1;	
		}
		cnt_down = 0;
	}

		//����������   2023.04.07 
	if(cnt_heartbag > gDeviceParam.heartRate *1000)//��ʱ�� �ҵ�ǰ����״̬������������²ŷ�������
	{
		cnt_heartbag = 0;
		gGlobalData.heartbag_flage = 1;
	}
	cnt_heartbag++;
	if(gGlobalData.heart_count < 5* gDeviceParam.heartRate *1000)
		gGlobalData.heart_count++;
	else
	{
			if(gGlobalData.netKind==1){              //˵��ʱ���ڶϿ���
				gGlobalData.isConnect=0;
				gGlobalData.conFlage=2;//��wifiȥ����       //���ڵ��̲߳���������Ӧ��     
				gGlobalData.Wifi_set = true;														//��������һ��wifi����
			}
			else if(gGlobalData.netKind==2){           //˵��ʱwifi�Ͽ���  
				gGlobalData.isConnect=0;//˵������Ͽ� 
				gGlobalData.conFlage=3;//��4gȥ����        
				gGlobalData.yd4gStatus=false;
			}
			else if(gGlobalData.netKind==3){      //˵��ʱ4g�Ͽ���
				gGlobalData.isConnect=0;						//˵������Ͽ� 
				gGlobalData.conFlage=2;							//��wifiȥ����					
				gGlobalData.yd4gStatus=false;
				gGlobalData.Wifi_set = true;					
			}
		 gGlobalData.heart_count = 0;//��ʱ��������Ϊ0 ���¿�ʼ����
	}
	
	if(cnt_down == 1000)
		cnt_down = 0;
	
	/////////////////////2023.03.24 kardos �������ݲɼ�//////////////
		if(gGlobalData.curWorkState == WORK_START && gGlobalData.curWorkMode==WORK_MODE_ZT && isCollect==true && gGlobalData.isConnect == 1)
		{
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *)gADC1_DMA_BUF,N_NUM* AD1_NUM);
			 if(gGlobalData.currentUnitNum>=gGlobalData.unitLen)
			 {
				 gGlobalData.currentUnitNum=0;
					if(gGlobalData.Send_Data_Task==false)
					{
						cnt_heartbag = 0;//��������������Ϊ0 ���¿�ʼ����
						gGlobalData.heart_count = 0;//��ʱ��������Ϊ0 ���¿�ʼ����
						memcpy(&gGlobalData.PlusePressDataSend[10],gGlobalData.PlusePressData,gGlobalData.unitLen*2); //ÿһ��16���Ƶ�����ռ�����ֽ�
						SendPlusePressData();//�ɼ������ϱ�һ��
					}
			 }
			//װ������
			if(reTime>=gGlobalData.freqUnitTime)
			{
				reTime=0;
				//gGlobalData.PlusePressData[gGlobalData.currentUnitNum]=(gADC3_VALUE_F[0]>>8)+(gADC3_VALUE_F[0]<<8);
				gGlobalData.PlusePressData[gGlobalData.currentUnitNum] = gADC3_VALUE[0];
				gGlobalData.currentUnitNum++;//�ɼ�����++
				collectDataCount++;//�ۼƲɼ�����
				if(gGlobalData.currentUnitNum == 50 && Collect_Data_state ==true){     //�ɼ�ǰ60�����ݲ��ȶ������ȶ����ٽ����ϴ�
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
  MX_I2C2_Init();         //���ε���ԭʼi2c����д��ģ��i2c�����ƣ�i2c��bug
//  IIC_Init();           //2023/8/12 by yls Ӳ��i2c������ȱ�ݲ����ã������i2cģ�ⷢ������   8/16 update�������õ�Ӳ��i2c 
  MX_SPI5_Init();
	MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_UART7_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
//  MX_SPI1_Init();
  MX_SPI3_Init();//���β���
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

	gDeviceParam.heartRate=15;    //�������15s
	gGlobalData.useWorkArg[gGlobalData.current_treatNums].level=0;
	gGlobalData.useWorkArg[gGlobalData.current_treatNums].freqTreat=0;
	gGlobalData.useWorkArg[gGlobalData.current_treatNums].timeTreat=0;
	__enable_irq();
  /* USER CODE BEGIN 2 */
	MX_LWIP_Init();
	send_NetSync(0);
	osDelay(20);
	send_duan_wang(1);	//������ʾ��� 1900
	osDelay(20);
	Send_LcdDevid_id(gDeviceParam.devId);  //�����豸id����Ļ��ʾ
	osDelay(20);
	sprintf(version,"%s",gDeviceParam.version);
	Send_LcdVersion(version,strlen((const char*)version));	 //���Ͱ汾�ŵ���Ļ��ʾ 
	osDelay(20);	
	Send_Text_SetButton(0,1);
	osDelay(20);

	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_SET);    //��ת���е��ɼ�  by yls 2023 6/7 �°�������PG11
	 
	//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET); //�����
	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET); //��Դ����ɫ
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET);  //���к�� set�� reset��
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);    //�����̵� set�� reset��
 	TimerHandle_t xAutoReloadTimer;  
	xAutoReloadTimer = xTimerCreate(	
		"AutoReload",                 /* ����, ����Ҫ */
		mainAUTO_RELOAD_TIMER_PERIOD, /* ���� */
		pdTRUE,                       /* �Զ����� */
		0,                            /* ID */
		vAutoLoadTimerFunc            /* �ص����� */
	);	
	if (xAutoReloadTimer){
		xTimerStart(xAutoReloadTimer, 0);
	}

	//-----------------------DMASPI���Ͳ���-----------------------//
	DAC8831_Set_Data(0x7fff);       //��ʼ��һ��
	//-----------------------���ֲ���-----------------------//
//		Send_ComMusic(3);
//-----------------------rtosʣ��������api��ʼʣ����44704---------------------------//	
	
	//wifi���ӣ�����MQTT ����wifi���ݹ�������Ϣ
 	osThreadDef(wifiTask, WifiMqttClientTask, osPriorityNormal, 0, 128*5);    //WifiMqttClientTask �̴߳����Ҫռ��418�ֽڻ���    ֮ǰ15
  wifiTaskHandle = osThreadCreate(osThread(wifiTask), NULL);
//	//4G���ӣ�����MQTT ����4G���ݹ�������Ϣ	
	osThreadDef(EC4gTask, EC200MqttClientTask, osPriorityNormal, 0, 128*5); 	//EC200MqttClientTask �̴߳����Ҫռ��360�ֽڻ���    ֮ǰ20
  ec4gTaskHandle = osThreadCreate(osThread(EC4gTask), NULL);
	//ҵ������̣߳����ȴ���ɼ� ���Ʒ����л���
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128*4);   //StartDefaultTask �̴߳����Ҫռ��190�ֽڻ���       ֮ǰ8                   
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
	
	/*���������ݲɼ� ��ʪ�� ��ص�����   ����߳����вɼ���ʪ�ȵ�  ��Ҫ��ʹ�� */
	osThreadDef(startLoopTask, StartLoopTask, osPriorityNormal, 0, 128*4);      //StartLoopTask �̴߳����Ҫռ��216�ֽڻ���            ֮ǰ4
  startLoopTaskHandle = osThreadCreate(osThread(startLoopTask), NULL);
	
	/*������Ļ���ݣ��������ݽ���*/
  osThreadDef(consoleTask, Console_Task, osPriorityNormal, 0, 128*3);       //Console_Task �̴߳����Ҫռ��85�ֽڻ���            ֮ǰ4  
  consoleTaskHandle = osThreadCreate(osThread(consoleTask), NULL);
	
	/**RJ45����ʱ ���ڽ���MQTT�������������**/
	osThreadDef(mqttRecvClientTask, StartMqttClientTask, osPriorityNormal, 0, 128*15);  //StartMqttClientTask �̴߳����Ҫռ��1228�ֽڻ���  ֮ǰ15
  mqttClientTaskHandle = osThreadCreate(osThread(mqttRecvClientTask), NULL);
	
 	/**RJ45ʱ�����������״̬**/
	osThreadDef(ethernetTask, ethernet_link_thread, osPriorityNormal, 0, 128*3);  //ethernet_link_thread �̴߳����Ҫռ��79�ֽڻ���  ֮ǰ2
  ethernetTaskTaskHandle = osThreadCreate(osThread(ethernetTask), NULL);
 
  /**mqtt �������� ������Ϣȷ������*/
	osThreadDef(mqttSendTask, StartMqttSendTask, osPriorityNormal, 0, 128*16);  //StartMqttSendTask �̴߳����Ҫռ��1296�ֽڻ���   ֮ǰ10
  mqttSendTaskHandle = osThreadCreate(osThread(mqttSendTask), NULL);
 	/*��������������*/
//	osThreadDef(sendheartTask, StartSendheartTask, osPriorityNormal, 0, 128*4); //StartSendheartTask �̴߳����Ҫռ��236�ֽڻ���   ֮ǰ4
//  sendheartTaskHandle = osThreadCreate(osThread(sendheartTask), NULL); 

//-----------------------rtosʣ��������api���ʣ����  14912  �����Կ�26*128���̣߳������ʡ�ſ�  ---------------------------//	

	
	/*��������*/
//  osThreadDef(lcdTask, Lcd_Task, osPriorityNormal, 0, 256);
//  lcdTaskHandle = osThreadCreate(osThread(lcdTask), NULL);
 
   /**���Ź�����*/
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
	 if(huart->Instance == UART4){ //�ж����ĸ����ڷ����ж�
		strstr1=strstr((const char*)Voice_aRxBuffer,"music:");  //������
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
				case 0x35:Send_ComVolume(Voice_aRxBuffer[2]*10); //�������� ����λ��10
					break;
				case 0x36:Send_ComMode(Voice_aRxBuffer[3]);  //����ָ����Ʋ���ģʽ 00:������ѭ��  01:����ѭ������  02:������Ŀѭ������  03:�������
					break;
				case 0x37:Send_ComMusic(2);
					break;
				case 0x38:Send_ComMusic(2);
					break;
				default:
					break;
			}

		}

  HAL_UART_Receive_IT(&huart4, (uint8_t *)Voice_aRxBuffer, Voice_RXBUFFERSIZE); //����ʹ�ܽ����ж�
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
  hi2c2.Init.Timing = 0xF010F2FF;     // 0x307075B1  100khz        25khz��0xC0206FFF(Cube)  0xF07075B1(�Լ���)    15KHZ:0xF010F2FF
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
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;      								//Ӳ������
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4 ;  //�ٶ�160M/4=40M�ٶ�      4��Ƶ�պÿ���  50M��dac8831
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;  					/* ʹ��������� */
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;    			/* �͵�ƽ��Ч */
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_07DATA;   /* ����FIFO��С��һ�������� */
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;       							  /* MSS, ���뵽NSS��Ч���غ͵�һ�����ݿ�ʼ֮��Ķ����ӳ٣���λSPIʱ�����ڸ��� */
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;  			  /* MIDI, ������������֮֡��������Сʱ���ӳ٣���λSPIʱ�����ڸ��� */
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;  									  /* ��ֹSPI��SPI������ű��ֵ�ǰ״̬ */ 
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	
//	hspi3.Instance               = SPI3;                   /* ����SPI */
//	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;     /* ���ò����� */
//	hspi3.Init.Direction         = SPI_DIRECTION_2LINES;   /* ȫ˫�� */
//	hspi3.Init.CLKPhase          = SPI_PHASE_1EDGE;              /* ����ʱ����λ */
//	hspi3.Init.CLKPolarity       = SPI_POLARITY_LOW;           /* ����ʱ�Ӽ��� */
//	hspi3.Init.DataSize          = SPI_DATASIZE_8BIT;      /* �������ݿ�� */
//	hspi3.Init.FirstBit          = SPI_FIRSTBIT_MSB;       /* ���ݴ����ȴ���λ */
//	hspi3.Init.TIMode            = SPI_TIMODE_DISABLE;     /* ��ֹTIģʽ  */
//	hspi3.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE; /* ��ֹCRC */
//	hspi3.Init.CRCPolynomial     = 7;                       /* ��ֹCRC�󣬴�λ��Ч */
//	hspi3.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;     /* ��ֹCRC�󣬴�λ��Ч */
//	hspi3.Init.NSS               = SPI_NSS_SOFT;               /* ʹ�������ʽ����Ƭѡ���� */
//	hspi3.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_16DATA;  /* ����FIFO��С��һ�������� */
//	hspi3.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;      /* ��ֹ������� */
//	hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE; /* ��ֹSPI��SPI������ű��ֵ�ǰ״̬ */  
//	hspi3.Init.Mode 			 	= SPI_MODE_MASTER;            /* SPI����������ģʽ */
	
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
//  __HAL_UART_ENABLE_IT(&huart4,UART_IT_IDLE);                //�����ж�ʹ��
//  __HAL_UART_ENABLE_IT(&huart4,UART_IT_RXNE);                //�����ж�ʹ��
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
	//PG11����ת��ת����
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

//��Ƶ�л�    Ĭ��
	GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
//12v����
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
	/*adc У׼**/
	if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED)!=HAL_OK)	//������ʽ
	{
		Error_Handler();
	}
	
	if(HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED)!=HAL_OK)
	{
		Error_Handler();
	}
	/*�ȴ�У׼����*/
	osDelay(10);	//����ɾ��  vTaskDelay()
	
	/*��ʼdma����adc��dma�������ж�*/
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)gADC1_DMA_BUF,N_NUM* AD1_NUM);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t *)gADC3_DMA_BUF,N_NUM* AD3_NUM);
	
	/*��ȡ�ڲ��ο���ѹ*/
	osDelay(50);	//�ȴ���һ��ת������
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
		if(collectTime>=1)	//ÿ10ms����һ��
		{
			
			collectTime=0;
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *)gADC1_DMA_BUF,N_NUM* AD1_NUM);
			HAL_ADC_Start_DMA(&hadc3, (uint32_t *)gADC3_DMA_BUF,N_NUM* AD3_NUM);//ֱ������
		}
		if(gDeviceParam.devLock==true)
		{
			continue;
		} 
		
		//�жϵ�ǰ����״̬
		if(gGlobalData.curWorkMode!=gGlobalData.oldWorkMode)
		{
 			if(gGlobalData.curWorkState==WORK_START)
			{
				gGlobalData.oldWorkMode=gGlobalData.curWorkMode;
				//��ʼ����ǰ״̬
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
						//�̵�������Ϊ����ģʽ
						set_sampleMode(MODE_ZT);  

						HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_SET);    //��ת���е��ɼ�  by yls 2023 6/7 �°�������PG11
                        
                        //����� //����� 
						Set_Input_Output(sON);   //�ɼ�
 
						
						//gGlobalData.curWorkState = WORK_PAUSE;
						HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_SET); //���к�� set�� reset��
						HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET); //�����̵� set�� reset��
						send_LcdWorkStatus(3);//kardos 2023.02.03 �޸��豸����״̬Ϊ����������
            Collect_Data_state = true;
						break;
					case WORK_MODE_ZL:
						unitTime=0;						
						//����ģʽ
						set_sampleMode(MODE_ZL);
						osDelay(500);

						Wave_select(gGlobalData.useWorkArg[0].waveTreat, ch1buf);									                  //����ѡ��
						Dac8831_Set_Amp(gGlobalData.useWorkArg[0].level, ch1buf);									                  //��ֵ�ı�
			
						DAC8831_Set_Data_Dma(ch1buf,sizeof(ch1buf)/2,gGlobalData.useWorkArg[0].freqTreat);					//��ʱ��������������
					
						send_treatSel(gGlobalData.useWorkArg[0].freqTreat,
									gGlobalData.useWorkArg[0].level,
								 (gGlobalData.useWorkArg[0].timeTreat)/60);	 													                  //��һ���Ƿ���jly�·����Ʒ�������Ļ
						osDelay(100);
						Send_LcdVoltage(5.84*gGlobalData.useWorkArg[0].level);	
						osDelay(100);
						HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_RESET);																		  //��ת���е�����	
                    
						Set_Input_Output(sON);   //���� �����  //�����

						channelTime=0;	//��ʼ��ʱ
						osDelay(200);
						send_LcdWorkStatus(4);//kardos 2023.02.03 �޸��豸����״̬Ϊ������������
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

						if(gGlobalData.curWorkMode==WORK_MODE_ZT && gGlobalData.isConnect==1)     //�����Ž��вɼ�
						{
                isCollect=true;                                
								//�жϲɼ�ʱ���Ƿ񵽣��Ҳ������һ��ͨ�����л���һͨ��,������ͨ��λ�á����ð�λ�ü�����ʱ�����
//								if(channelTime>=gGlobalData.useWorkArg[0].timeCheck*1000)
								if(collectDataCount>(gGlobalData.useWorkArg[0].timeCheck*gGlobalData.useWorkArg[0].rateN))     //�������Զ����һ�β��л���֤����������
								{
							    gGlobalData.Alltime = gGlobalData.Alltime - gGlobalData.useWorkArg[0].timeCheck;//����ʱ��1 2023.04.04 kardos
									Countdown_Treat(gGlobalData.Alltime);//ˢ�µ���ʱ
									if(gGlobalData.channelPos>=(gGlobalData.useWorkArg[0].chanelNum-1))
									{  
										isCollect=false;
										set_sampleMode(MODE_CLOSE);
										HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_RESET);//�е�����
										yd_4gfalg=false ;
										if(gGlobalData.netKind!=1)
										osDelay(1000);          //wifi 4g��ʱ1s��Ųɼ���������Ȼ�����ϴ����ݱ����ղ������Ʒ���
 
										gGlobalData.channelPos = 0;

										HAL_PCA9554_outputAll(0);//�ȹر����м̵���
										yd_4gfalg=true ;
										gGlobalData.curWorkMode =WORK_MODE_WT;
										gGlobalData.oldWorkMode=gGlobalData.curWorkMode;
										taskENTER_CRITICAL();
										Send_Fix_Ack(23,STATUS_OK,"ZT Caiji ok");  //����ɼ����� 
										collectDataCount=0;//�ɼ�������������Ϊ0
										taskEXIT_CRITICAL();
									}else{
										isCollect=false;
//										gGlobalData.currentUnitNum=0;           //���⵼���л�ͨ�����������·���ȫ
										collectDataCount=0;//����Ϊ0
										/*�ر���һ�εĲɼ�ͨ��*/
                                        
                    Set_Input_Output(sOFF);     //�ɼ�
								
										gGlobalData.channelPos++;
										/*�л�Ϊ��һͨ��*/
										//�����  //�����
                                        
                    Set_Input_Output(sON);       //�ɼ�
                                        
										unitTime=0;
										osDelay(10);//ͨ���л�����һ����ʱ �������ȶ����ٲɼ�
										Collect_Data_state = true;
										isCollect=true;
									}
								channelTime=0;//������òɼ�ͨ��ʱ��Ϊ0 ����������ռ��ʱ��
								}
						}
						else if(gGlobalData.curWorkMode==WORK_MODE_ZL)
						{		//2023.02.01			 		
								//channelTime++;
							
							//���һ�����Ʒ���������λ����10��ʣ�����2���ӣ�ÿ4���½�һ����λ������𲽼��� 2023.02.06 kardos
							if(gGlobalData.channelPos>=gGlobalData.useWorkArg[0].chanelNum-1)//˵�������һ��
							{
							  if(((gGlobalData.useWorkArg[gGlobalData.current_treatNums].timeTreat + gGlobalData.useWorkArg[gGlobalData.current_treatNums].waitTime)*1000-channelTime)<=120000)//˵�������60�� ��ʼ���͵�λ
								{
									if(channelTime-countTime>=4000&&gGlobalData.useWorkArg[gGlobalData.current_treatNums].level>10){
										gGlobalData.useWorkArg[gGlobalData.current_treatNums].level-=1;
								  	Wave_select(gGlobalData.useWorkArg[gGlobalData.current_treatNums].waveTreat, ch1buf);																		//����ѡ��
										Dac8831_Set_Amp(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level, ch1buf);																		//��ֱֵ�Ӹ���0
										
										DAC8831_Set_Data_Dma(ch1buf,sizeof(ch1buf)/2,gGlobalData.useWorkArg[gGlobalData.current_treatNums].freqTreat);					//��ʱ��������������
										
										countTime=channelTime;
										
										send_treatSel(gGlobalData.useWorkArg[gGlobalData.current_treatNums].freqTreat,
										gGlobalData.useWorkArg[gGlobalData.current_treatNums].level,
										(gGlobalData.useWorkArg[gGlobalData.current_treatNums].timeTreat)/60);	//��һ���Ƿ���jly�л����Ʒ�������Ļ
										osDelay(100);
										if(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level <= 60)
											Send_LcdVoltage(5.84*gGlobalData.useWorkArg[gGlobalData.current_treatNums].level);	
									}
								}
							}
							
								//�ж�����ʱ���Ƿ񵽣��Ҳ������һ��ͨ�����л���һͨ��,������ͨ��λ�á����ð�λ�ü�����ʱ�����
								if(channelTime>=(uint32_t)(gGlobalData.useWorkArg[gGlobalData.current_treatNums].timeTreat + gGlobalData.useWorkArg[gGlobalData.current_treatNums].waitTime)*1000)
								{
									//���ж��Ƿ������һ�����Ʒ���ʱ�䵽��
									if(gGlobalData.channelPos>=gGlobalData.useWorkArg[0].chanelNum-1)
									{//˵�����һ�����Ʒ���ʱ�䵽�� �ر�����
										//�Ƚ���λ������0 ��ֹ����һ��
										Wave_select(gGlobalData.useWorkArg[gGlobalData.current_treatNums].waveTreat, ch1buf);		//����ѡ��
										Dac8831_Set_Amp(0, ch1buf);																															//��ֱֵ�Ӹ���0
										HAL_TIM_Base_DeInit(&htim12);  																												  //����������
										osDelay(200);		
										send_LcdWorkStatus(6);//kardos 2023.02.03 �޸��豸����״̬Ϊ��                                              �����������
										gGlobalData.curWorkMode=WORK_MODE_WT;
										gGlobalData.oldWorkMode=gGlobalData.curWorkMode;
										gGlobalData.curWorkState=WORK_STOP;
										set_sampleMode(MODE_CLOSE);
										HAL_PCA9554_outputAll(0);
										HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET); 																		//���к�� set�� reset��
										HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET); 																			//�����̵� set�� reset��
										//2023.02.01	WWJZ
                    gGlobalData.useWorkArg[gGlobalData.current_treatNums].level=0;
										gGlobalData.Alltime=0;
										send_LcdSync(0);																																				//�޸�����״̬
										send_LcdOutStatus(0);                                                                   //״̬��
										osDelay(100);		
										send_LcdWorkStatus(1);																																	//kardos 2023.02.03 �޸��豸����״̬Ϊ���豸����״̬ 
										osDelay(100);			
										send_treatSel(0,0,0);     																															//jly���Ʒ���ȫ�ó�0	
										osDelay(100);										
										Send_LcdVoltage(0);	
										osDelay(100);	
										send_visitNumber(gbkNum0);																															//�޸ľ����˱��
										osDelay(100);
										send_visitName(gbk0,8);   																															//�޸ľ���������
										osDelay(100);
										send_visitAge(0);         																															//�޸ľ���������
										osDelay(100);
										send_visitSex(0);         																															//�޸ľ������Ա�
									  //2023.02.01	WWJZ
                    gGlobalData.cur_heart_state=LEISURE; 																									  //���ƽ�����λ״̬
										Send_Fix_Ack(24,STATUS_OK,"OK");
										gGlobalData.current_treatNums=0;
										gGlobalData.channelPos=0;
										osDelay(100);
										send_LcdWorkStatus(1);																																	//kardos 2023.02.03 �޸��豸����״̬Ϊ���豸����״̬
										countTime=0;																																						//����
									}else{
									//˵��ʱ�䵽�˵��ǲ������һ�����Ʒ���������һ�����Ʒ���
										Set_Input_Output(sOFF);    //����
		             
										gGlobalData.channelPos++;
										osDelay(1000);
									//�л�����
										gGlobalData.current_treatNums++;				
										Wave_select(gGlobalData.useWorkArg[gGlobalData.current_treatNums].waveTreat, ch1buf);//����ѡ��
										Dac8831_Set_Amp(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level, ch1buf);//��ֵ�ı�
										
										DAC8831_Set_Data_Dma(ch1buf,sizeof(ch1buf)/2,gGlobalData.useWorkArg[gGlobalData.current_treatNums].freqTreat);					//��ʱ��������������										
										
										send_treatSel(gGlobalData.useWorkArg[gGlobalData.current_treatNums].freqTreat,
											gGlobalData.useWorkArg[gGlobalData.current_treatNums].level,
											(gGlobalData.useWorkArg[gGlobalData.current_treatNums].timeTreat)/60);	//��һ���Ƿ���jly�л����Ʒ�������Ļ
										osDelay(100);
										if(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level <= 60)
											Send_LcdVoltage(0.584*gGlobalData.useWorkArg[gGlobalData.current_treatNums].level);										

										osDelay(200);
										/*�л�Ϊ��һͨ��*/
										//�����   //�����
                                        
										Set_Input_Output(sON);      //����
                                        
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
					if(RecSen[3] == 0x83) //wifi�������
					{
						if(RecSen[4] == 0x18 && RecSen[5] == 0x20)
						{
							//��wifi����
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
							//��wifi����
							memset(gDeviceParam.wifiArg.WifiPwd,0,sizeof(gDeviceParam.wifiArg.WifiPwd));
							for(uint8_t i = 7;i < sizeof(gDeviceParam.wifiArg.WifiPwd); ++i)
							{
								if(RecSen[i] == 0xFF)
									break;
								gDeviceParam.wifiArg.WifiPwd[i-7] = RecSen[i];
							}
						}
						if(RecSen[4] == 0x19 && RecSen[5] == 0x10)  //��Ļ��ʾ�򰴼���㲻����
						{
								Send_Text_SetButton(0,0);	//�ı��򰴼�Ĭ�ϲ�����
								osDelay(50);

						}
					}
				 if(RecSen[2] == 0x06)
				 {
					 switch(RecSen[8])
						{
							case 0x01:   //��ʼ��ť
									if(gGlobalData.curWorkState != WORK_START)
									{	
										gGlobalData.cur_heart_state=WORKING;
										gGlobalData.curWorkState=WORK_START;
											//2023.1.31 WWJZ
										if(gGlobalData.curWorkMode==WORK_MODE_ZL){//���������Ƶ��������ͣ���ٴο�ʼ
                                       
                                                set_sampleMode(MODE_ZL);
												osDelay(200);
												send_LcdWorkStatus(4);//kardos 2023.02.03 �޸��豸����״̬Ϊ������������	
												send_LcdOutStatus(1);											 
										 }//2023.1.31 WWJZ
										 osDelay(200);
										 send_LcdSync(1);  //�޸���Ļ����״̬
										
										if(gGlobalData.curWorkMode==WORK_MODE_ZT){
											osDelay(200);	
											send_LcdWorkStatus(3);//kardos 2023.02.03 �޸��豸����״̬Ϊ����������			
										
											set_sampleMode(MODE_ZT);
											osDelay(200);                                            
											isCollect=true;
											break;
										}

                                        
											cnt_heartbag = 0;												//���������������������
											gGlobalData.heartbag_flage = 1;
									 }
									break;
							case 0x02:  //��ͣ��ť
									if(gGlobalData.curWorkState == WORK_START){
											gGlobalData.cur_heart_state=PAUSE;
											gGlobalData.curWorkState=WORK_PAUSE;
											set_sampleMode(MODE_CLOSE);
								 
											 osDelay(20);
											 send_LcdSync(0); 
											 osDelay(200);
											 send_LcdWorkStatus(5);//kardos 2023.02.03 �޸��豸����״̬Ϊ���豸��ͣ״̬
											 send_LcdOutStatus(0);
											 if(gGlobalData.curWorkMode == WORK_MODE_ZT)
											 {
													 isCollect=false;//2023.03.31 ZKM
													 Collect_Data_state =true;
											 }
													 
											 cnt_heartbag = 0;												//���������������������
											gGlobalData.heartbag_flage = 1;
									}
								break;
							case 0x03://��λ��ť
//									Set_Input_Output(sOFF);                       //����һ�临λ��Ѱַ�ر���һ�ε���ͨ��
									Send_Fix_Ack(100,STATUS_OK,"OK");//���͸���λ������ִ���˸�λ
									gGlobalData.cur_heart_state=LEISURE;
									gGlobalData.curWorkState=WORK_STOP; 
                                    collectDataCount=0;
                                    gGlobalData.useWorkArg[gGlobalData.current_treatNums].level=0;  //��λ��λ����Ϊ0
									gGlobalData.current_treatNums=0;//2023.02.01
                                   
									set_sampleMode(MODE_CLOSE);
 
									HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_SET);    //��ת���е��ɼ�  by yls 2023 6/7 �°�������PG11
									gGlobalData.channelPos=0;
									HAL_PCA9554_outputAll(0);
									HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET); //���к�� set�� reset��
									HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET); //�����̵� set�� reset��
									gGlobalData.curWorkMode=WORK_MODE_WT;
									gGlobalData.oldWorkMode=gGlobalData.curWorkMode;
									gGlobalData.curWorkState=WORK_STOP;
									osDelay(20); 
									send_LcdWorkStatus(1);//kardos 2023.02.03 �޸��豸����״̬Ϊ���豸����״̬
									osDelay(100);
									gGlobalData.Alltime=0; //����ʱ���
									Countdown_Treat(gGlobalData.Alltime);//ˢ�µ���ʱ
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
									HAL_TIM_Base_DeInit(&htim12);  //����������
									break;
							case 0x05://��λ��

								if(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level <=60){
									gGlobalData.useWorkArg[gGlobalData.current_treatNums].level+=1;
									send_treatSel(gGlobalData.useWorkArg[gGlobalData.current_treatNums].freqTreat,
												gGlobalData.useWorkArg[gGlobalData.current_treatNums].level,
												(gGlobalData.useWorkArg[gGlobalData.current_treatNums].timeTreat)/60);
									osDelay(100);
									if(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level <= 60)
										Send_LcdVoltage(5.84*gGlobalData.useWorkArg[gGlobalData.current_treatNums].level);	
									Wave_select(gGlobalData.useWorkArg[gGlobalData.current_treatNums].waveTreat, ch1buf);//����ѡ��
									Dac8831_Set_Amp(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level, ch1buf);//��ֵ�ı�
								}
								Dac_level_CTL(1);   //��λ�ı���β���
								osDelay(500);//��ʱ ��ֹ�������
								cnt_heartbag = 0;												//���������������������
								gGlobalData.heartbag_flage = 1;
								break;
								//level  -5
							case 0x06://��λ��

								if(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level >=5){
									gGlobalData.useWorkArg[gGlobalData.current_treatNums].level-=1; 
									send_treatSel(gGlobalData.useWorkArg[gGlobalData.current_treatNums].freqTreat,
											gGlobalData.useWorkArg[gGlobalData.current_treatNums].level,
											(gGlobalData.useWorkArg[gGlobalData.current_treatNums].timeTreat)/60);
									osDelay(100);
									if(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level <= 60)
										Send_LcdVoltage(5.84*gGlobalData.useWorkArg[gGlobalData.current_treatNums].level);
									Wave_select(gGlobalData.useWorkArg[gGlobalData.current_treatNums].waveTreat, ch1buf);//����ѡ��
									Dac8831_Set_Amp(gGlobalData.useWorkArg[gGlobalData.current_treatNums].level, ch1buf);//��ֵ�ı�
								}
								Dac_level_CTL(1);   //��λ�ı���β���
								osDelay(500);//��ʱ ��ֹ�������
								cnt_heartbag = 0;												//���������������������
							  gGlobalData.heartbag_flage = 1;
								break;
							case 0x0a:
								gGlobalData.ResetDevidcnt++;
								break;
							case 0x50://wifi����ȷ��ָ��
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

	}//forѭ��
}

bool Get_ADC1_Hex(void)	
{
	//static uint16_t pt[AD1_NUM*10];
	int i,j ;
	uint16_t temp[N_NUM];	//������������
	//if(Read_All_AD1(pt,AD1_NUM*10) == false) return false;	//DMAһֱ�ɼ�
	
	for(i=0;i<AD1_NUM;i++)
	{
		for(j=0;j<N_NUM;j++) {temp[j] =  gADC1_DMA_BUF[j*AD1_NUM+i];} //ȡ��ÿһ�ε�ֵ
		gGlobalData.currentNet=Gets_Average(temp,N_NUM);	
		*(gADC1_VALUE+i) = Gets_Average(temp,N_NUM);	//����ƽ��ֵ
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
	//if(Read_All_AD1(pt,AD1_NUM*10) == false) return false;	//DMAһֱ�ɼ�
	
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
  HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xFFFF);       //�����ض��򴮿�3 �� ����6 �ʹ���7 
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

//��ʼ���������Ź�
//prer:��Ƶ��:0~7(ֻ�е�3λ��Ч!)
//rlr:�Զ���װ��ֵ,0~0XFFF.
//��Ƶ����=4*2^prer.�����ֵֻ����256!
//rlr:��װ�ؼĴ���ֵ:��11λ��Ч.
//ʱ�����(���):Tout=((4*2^prer)*rlr)/32 (ms).
void IWDG_Init(uint8_t prer,uint16_t rlr)
{
    IWDG_Handler.Instance=IWDG1;
    IWDG_Handler.Init.Prescaler=prer;    //����IWDG��Ƶϵ��
    IWDG_Handler.Init.Reload=rlr;        //��װ��
    IWDG_Handler.Init.Window=IWDG_WINDOW_DISABLE;//�رմ��ڹ���
    HAL_IWDG_Init(&IWDG_Handler);    
}
//ι�������Ź�
void IWDG_Feed(void)
{   
    HAL_IWDG_Refresh(&IWDG_Handler); //��װ��
}

void StartIwdgTask(void const *arg)
{
     EventBits_t uxBits;
     const TickType_t xTicksToWait = 2000 / portTICK_PERIOD_MS; /* ����ӳ�2000ms */
    /*
      ��ʼִ����������������ǰʹ�ܶ������Ź���
      ����LSI��64��Ƶ�����溯��������Χ0-0xFFF���ֱ������Сֵ2ms�����ֵ6552ms

       ����LSI��128��Ƶ�����溯��������Χ0-0xFFF���ֱ������Сֵ4ms�����ֵ13104ms
       Tout = prv/40 * rlv (s)
      �������õ���5s�����5s��û��ι����ϵͳ��λ��
    */
    IWDG_Init(0x00000004 ,625*5);
    while(1)
    {
        /* �ȴ������������¼���־ */
        uxBits = xEventGroupWaitBits(xCreatedEventGroup, /* �¼���־���� */
                     TASK_BIT_ALL,       /* �ȴ�TASK_BIT_ALL������ */
                     pdTRUE,             /* �˳�ǰTASK_BIT_ALL�������������TASK_BIT_ALL�������òű�ʾ���˳���*/
                     pdTRUE,             /* ����ΪpdTRUE��ʾ�ȴ�TASK_BIT_ALL��������*/
                     xTicksToWait);      /* �ȴ��ӳ�ʱ�� */
        if((uxBits & TASK_BIT_ALL) == TASK_BIT_ALL)//�жϸ��������Ƿ�ִ��
        {
            IWDG_Feed();
        }
    }
        
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
