#include "stdio.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
/* default header*/
#include "main.h"
#include "StringTransform.h"
#include "startLoopTask.h"
#include "com.h"
#include "lcdTask.h"
#include "crc16.h"
#include "common.h"
const uint8_t header[2]={0x5a,0xa5};


unsigned char ccRecCom3[100];

//lcd��Ļ�л�
void send_lcdPage(uint16_t pageNum)
{
	uint8_t buf[20]={0x00};
	short int tmp=0;
	uint16_t utmp16=0,len;
	len=10;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;

	buf[4]=0x00;
	buf[5]=0x84;
	//0x1001 frist addr data
	utmp16 = 0x5a01;
	buf[6]=utmp16>>8;
	buf[7]=(uint8_t)utmp16;
	
	utmp16 =pageNum;
	buf[8]=utmp16>>8;
	buf[9]=(uint8_t)tmp;
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);	
}
//Ѩλͼ�л�����ά��ͼ
void send_lcdQRcode(void)
{
	uint8_t buf[35]={0x00};
	uint16_t utmp16=0,len;
	
	len=8;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;

	utmp16=MERIDIAN_ADD;//address 1700
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;
	
	buf[6]=0;
	buf[7]=0;
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);	
}

//��Ļ��ʾʱ��
void send_rtcTime(uint8_t year,uint8_t month,uint8_t day,uint8_t hour,uint8_t minute,uint8_t sec)
{
	uint8_t buf[20]={0x00};
	uint16_t utmp16=0,len;
	len=14;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;
	buf[3]=0x82;

	utmp16=TIME_SET_ADD;
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;
	//0x1001 frist addr data
	utmp16 = 0x5aa5;
	buf[6]=utmp16>>8;
	buf[7]=(uint8_t)utmp16;
	
	buf[8]=year;
	buf[9]=month;
	buf[10]=day;
	buf[11]=hour;
	buf[12]=minute;
	buf[13]=sec;
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);	
}

//��Ļ����ʱ
void send_countDown(uint8_t hour,uint8_t minute,uint8_t sec)
{
	uint8_t buf[20]={0x00};
	uint16_t utmp16=0,len;
	len=10;   								 //����ҪΪż�����У�������buf��0
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;

	utmp16=COUNT_DOWN_ADD;
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;

	buf[6]=hour;
	buf[7]=minute;
	buf[8]=sec;
	buf[9]=0;
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);	
}


//�޸���Ļ���Ʒ���
void send_treatSel(uint16_t freq,uint16_t geat,uint16_t time)
{
	uint8_t buf[20]={0x00};
	uint16_t utmp16=0,len;
	len=12;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;
	buf[3]=0x82;

	utmp16=TREAT_SEL_ADD;//0x1400
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;

	utmp16=freq;
	buf[6]=utmp16>>8;
	buf[7]=(uint8_t)utmp16;
	
	utmp16=geat;
	buf[8]=utmp16>>8;
	buf[9]=(uint8_t)utmp16;
	
	utmp16=time;
	buf[10]=utmp16>>8;
	buf[11]=(uint8_t)utmp16;
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);	
}

//�޸���Ļ�����˱��
void send_visitNumber(uint8_t* data)
{
	uint8_t buf[25]={0x00},i;
	uint16_t utmp16=0,len;
	len=20;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;

	utmp16=VISIT_NUMBER_ADD;
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;
	
	for(i=0;i<14;i++)
	{
		buf[6+i]=data[i];
	}
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);	
}

//�޸���Ļ����������
void send_visitName(uint8_t* data,uint8_t dataLen)
{
//	uint8_t data1[6]={0xd0,0xdc,0xd7,0xd3,0xd1,0xf4};
	uint8_t buf[35]={0x00},i;
	uint16_t utmp16=0,len;
	if(dataLen>NAME_LEN_MAX)
		return;
	
	len=6+dataLen;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;

	utmp16=VISIT_NAME_ADD;
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;
	
	for(i=0;i<dataLen;i++)
	{
		buf[6+i]=data[i];
	}
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);	
}

//�޸���Ļ����������
void send_visitAge(uint8_t age)
{
	uint8_t buf[35]={0x00};
	uint16_t utmp16=0,len;
	len=8;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;

	utmp16=VISIT_AGE_ADD;
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;
	
	buf[6]=0;
	buf[7]=age;
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);	
}

//�޸���Ļ�������Ա�
void send_visitSex(uint16_t sex)
{
	uint8_t buf[35]={0x00};
	uint16_t utmp16=0,len;
	
	len=8;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;

	utmp16=VISIT_SEX_ADD;
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;
	
	utmp16=sex;
	buf[6]=utmp16>>8;
	buf[7]=(uint8_t)utmp16;
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);	
}

//�޸���Ļ��ά����Ϣ
void send_QRInfo(uint8_t* data,uint8_t dataLen)
{
	uint8_t buf[200]={0x00},i;
	uint16_t utmp16=0,len;
	if(dataLen>NAME_QR_LEN_MAX)
		return;
	
	len=8+dataLen;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;

	utmp16=QR_CODE_ADD;
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;
	
	for(i=0;i<dataLen;i++)
	{
		buf[6+i]=data[i];
	}
	
	buf[6+dataLen]=0xff;
	buf[6+dataLen+1]=0xff;
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);	
}


//�޸���Ļ����״̬
void send_LcdSync(uint8_t status)
{
	uint8_t buf[35]={0x00};
	uint16_t utmp16=0,len;
	
	len=8;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;

	utmp16=LCD_SYNC_ADD;//address 1600
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;
	
	buf[6]=0;
	buf[7]=(status==SYNC_OK)?1:0;
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);		
}
//�޸���Ļ����״̬ kardos 2023.02.03
void send_LcdWorkStatus(uint8_t status)
{
	uint8_t buf[35]={0x00};
	uint16_t utmp16=0,len;

	
	len=8;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;

	utmp16=LCD_WORKSTATUS_ADD;//address 1810
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;
	
	buf[6]=0;
	buf[7]=status;
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);	
}

void send_LcdOutStatus(uint8_t status)
{
	uint8_t buf[35]={0x00};
	uint16_t utmp16=0,len;
	
	len=8;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;

	utmp16=LCD_STATUS_ADD;//address 1620
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;
	
	buf[6]=0;
	buf[7]=status;
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);	
}

/*�޸���Ļ��������״̬
����˵����status 0 �����쳣 1 ��������
								 2 wifi�쳣 3 wifi����
								 4 4g�쳣   5 4g����
*/
void send_NetSync(uint8_t status)
{
	uint8_t buf[35]={0x00};
	uint16_t utmp16=0,len;
	
	len=8;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;

	utmp16=NET_SYNC_ADD;
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;
	
	buf[6]=0;
	//buf[7]=(status==SYNC_OK)?1:0;
	buf[7]=status;
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);	
}

//�޸���Ļ����״̬ͼƬ
void send_StartPic(uint8_t status)
{
	uint8_t buf[35]={0x00};
	uint16_t utmp16=0,len;
	
	len=8;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;

	utmp16=LCD_DISPLAY_ADD;
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;
	
	buf[6]=0;
	buf[7]=status;
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);	
}
//������ʾ  para   0������  1���쳣
void send_duan_wang(uint8_t duan)
{
	uint8_t buf[35]={0x00};
	uint16_t utmp16=0,len;
	
	len=8;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;

	utmp16=duan_wang_ADD;
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;
	
	buf[6]=0;
	buf[7]=duan;
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);	
}

void Send_LcdVoltage(uint16_t voltage)
{
	uint8_t buf[35]={0x00};
	uint16_t utmp16=0,len;
	
	len=8;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;

	utmp16=LCD_VOLTAGE_ADD; 
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;
	
	buf[6]=voltage>>8;
	buf[7]=(uint8_t)voltage;
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);	
}

//�豸id����Ļ��ʾ �Ժ����Ҫ�ӳ�Ҫ�� by yls 2023/5/24
void Send_LcdDevid_id(int32_t Devid)
{
	uint8_t buf[35]={0x00};
	uint16_t utmp16=0,len;
	
	len=10;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;
	
	utmp16=LCD_DEVIDE_ID; 
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;

	buf[6]=Devid>>24;
	buf[7]=Devid>>16;
	buf[8]=Devid>>8;
	buf[9]=Devid>>0;
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);
}

void Send_LcdVersion(uint8_t* version,uint8_t versionLen)
{
	uint8_t buf[35]={0x00},i;
	uint16_t utmp16=0,len;
	
	len=6+versionLen+1;     //����ҪΪż�����У�������buf��0,�汾�Ŵ�����5λ
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;
	
	utmp16=LCD_SYS_VS; 
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;
	
	for(i=0;i<versionLen;i++)
	{
		buf[6+i]=version[i];
	}

	buf[6+versionLen]=0;
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);
}
//������ʾ�ı���ʾ��
/*
Para: 0:���� 1:��
*/
void Send_Text_Box(uint8_t state)  
{
	uint8_t buf[35]={0x00};
	uint16_t utmp16=0,len;
	
	len=8;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;
	
	utmp16=LCD_Text_box; 
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;
	
	buf[6]=0;
	buf[7]=state;
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);
}

//������ʾ�ı�����
/*
Para: data������  dataLen�����ݳ���
*/
void Send_Text_Content(uint8_t* data,uint8_t dataLen)  
{
	uint8_t buf[200]={0x00},i;
	uint16_t utmp16=0,len;
	if(dataLen > 200)   //46
		return;
	len=6+dataLen+2;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;

	utmp16=LCD_Text_content;
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;
	
	for(i=0;i<dataLen;i++)
	{
		buf[6+i]=data[i];
	}
	buf[6+i]=0;
	buf[6+i+1]=0;
	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);
}

//����ȷ�ϰ�������
/*
Para: page��ҳ��  state���ɲ�����->0:������  1������
*/

void Send_Text_SetButton(uint8_t page,uint8_t state)  
{
	uint8_t buf[35]={0x00};
	uint16_t utmp16=0,len;

	len=14;
	memset(buf,0,sizeof(buf));
	buf[0]=header[0];
	buf[1]=header[1];
	buf[2]=len-3;	
	buf[3]=0x82;

	utmp16=LCD_Text_buttonR;
	buf[4]=utmp16>>8;
	buf[5]=(uint8_t)utmp16;
	
	utmp16=LCD_Text_buttonN;
	buf[6]=utmp16>>8;
	buf[7]=(uint8_t)utmp16;
    
  buf[8]=page>>8;
	buf[9]=page;
    
  buf[10]=0;
	buf[11]=8;
    
  buf[12]=0;
	buf[13]=state;

	HAL_UART_Transmit_Lcd(buf,len,0xFFFF);	
}

//��Ļ���ƴ��� 
void page_ctl_process(uint8_t ctlNum)
{
	char strBuf[20]={0,};
	uint8_t hexBuf[20],hexLen,i;
	memset(strBuf,0,sizeof(strBuf));
	switch(ctlNum)
	{
		case WORK_START_VAL:
			//��������ͼ
			 send_StartPic(DIS_FRONT_VAL);
			break;
		case WORK_STOP_VAL:
			//���Ͳ���ͼ
			send_StartPic(DIS_BACK_VAL);
			break;
		case WORK_RESET_VAL:
			//���Ͷ�ά��
			sprintf(strBuf,"%d",gDeviceParam.devId);
			hexLen=strlen(strBuf);
			for(i=0;i<hexLen;i++)
			{
				hexBuf[i]=strBuf[i];	//0~9 gbk=ascii
			}
			break;
		default:
		break;
	}
}

//lcd�������ݴ���
void LcdData_Handle(uint8_t *src,uint16_t len)
{
	uint16_t	addr;
	uint8_t utmp8; 
	
	if(src[0]==header[0]&&src[1]==header[1])	//�����͹���ҳ�������·�
	{		
		gGlobalData.lcdStatus=true;
		addr=src[4]*256+src[5];
		switch(addr)
		{
 			case	WORK_CTL_ADD:	//�豸����״̬����
				utmp8=src[8];
				do_work_ctl(utmp8);
				//ͼƬ�л�
				page_ctl_process(utmp8);
				break;
			default:	
				break;
		}
 	}
	//memset(RecCom7,0,sizeof(RecCom7));
	memset(RecCom7,0,10);
	
	return ;
}

bool lcdTask_Semaphore_Init(void)
{
	/*Ĭ�ϳ�ʼ��Ϊ0*/
	lcdSemaphore = xSemaphoreCreateBinary();
   
     if(lcdSemaphore == NULL)
    {
        /*ʧ�ܷ���false*/
			return false;
    }
   
     /*�����������ͷ�һ�� ��ʼ��Ϊ1 */
//     xSemaphoreGive(xSemaphore);
	return true;
}

void Lcd_Task(void const * argument)
{	

	
	/*
	HAL_UART_Receive_DMA(&huart3, RecCom3,COM3_REC_MAX);

	__HAL_UART_CLEAR_IDLEFLAG(&huart3);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	*/   // debug
	while(1)
	{
		

//		printf("\r\n  com3=%s\r\n",RecCom3);	
//		sprintf(ccRecCom3,"\r\ncom=%s\r\n",RecCom3);

//		HAL_Delay(3000);
/*		xSemaphoreTake(lcdSemaphore,portMAX_DELAY);
	

		LcdData_Handle(RecCom3,RecCom3Num);
		
		READ_REG(huart3.Instance->RDR);		
		
		SET_BIT(huart3.Instance->CR1, USART_CR1_RE);
		HAL_UART_Receive_DMA(&huart3, RecCom3, COM3_REC_MAX); //���źŶ�ʧ ��ᵼ��DMAδ���������ܷ�������
*/  //debug
	}//while
  /* USER CODE END 5 */ 
}

