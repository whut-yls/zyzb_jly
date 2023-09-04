/**
  ******************************************************************************
  * @file           : myTimer.h
  * @brief          : Header for myTimer.c file.
  *                   This file contains the common defines of the application
  *
  ******************************************************************************
  */
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCDTASK_H
#define __LCDTASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdbool.h"
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

#define SYNC_OK	1	//同步正常
#define SYNC_ERR	0	//同步异常

#define NAME_LEN_MAX	8		//4*2
#define NAME_QR_LEN_MAX		100	//32*1 32个字符数字   2023 yls 二维码长度加长
#define DIS_QRCODE_VAL	0x0000	//显示二维码值	复位对应
#define DIS_FRONT_VAL	0x0001	//显示二维码值		启动对应
#define DIS_BACK_VAL	0x0002	//显示二维码值		停止对应

#define WORK_START_VAL	1
#define WORK_STOP_VAL		2
#define WORK_RESET_VAL	3

#define TIME_SET_ADD   0x009c	     //屏幕显示时间设置
#define COUNT_DOWN_ADD	 0x1500   //0x8500	//屏幕倒计时修改	
#define TREAT_SEL_ADD	0x1400	//修改屏幕治疗方案
#define VISIT_NUMBER_ADD	0x1000	//修改就诊者编号
#define VISIT_NAME_ADD	0x1100	//就诊者名字
#define VISIT_AGE_ADD	0x1200	//就诊者年龄
#define VISIT_SEX_ADD	0x1300	//就诊者性别
#define QR_CODE_ADD	0x2000	//屏幕二维码
#define LCD_SYNC_ADD	0x1600	//设备同步状态
#define NET_SYNC_ADD	0x1610	//网络同步状态
#define MERIDIAN_ADD	0x1700  //人体经络照片
#define duan_wang_ADD  0x1900   //断网提示框
#define LCD_Text_box   0x1910   //文本提示框 by yls 2023/6/17    
#define LCD_Text_content 0x1920  //提示框内容  by yls 2023/6/17
#define LCD_Text_buttonR  0x00B0     //模式 by yls 2023/6/17       
#define LCD_Text_buttonN  0x5aa5  //模式 by yls 2023/6/17
#define LCD_DISPLAY_ADD	0x1800	//屏幕启动状态
#define LCD_WORKSTATUS_ADD	0x1810	//屏幕工作状态，kardos 2023.02.03
#define LCD_STATUS_ADD 0x1620
#define WORK_CTL_ADD	0x1800	//设备工作状态控制 01开始 02停止 03复位
#define LCD_VOLTAGE_ADD	    0x1410	//屏幕电压显示
#define LCD_DEVIDE_ID     0x1510     //显示设备id
#define LCD_SYS_VS        0x1520      //显示版本号


void send_lcdPage(uint16_t pageNum);    //lcd屏幕切换
void send_rtcTime(uint8_t year,uint8_t month,uint8_t day,uint8_t hour,uint8_t minute,uint8_t sec);  //屏幕显示时间
void send_countDown(uint8_t hour,uint8_t minute,uint8_t sec);        //屏幕倒计时
void send_treatSel(uint16_t freq,uint16_t geat,uint16_t time);    //修改屏幕治疗方案
void send_visitNumber(uint8_t* data);        //修改屏幕就诊人编号
void send_visitName(uint8_t* data,uint8_t dataLen);       //修改屏幕就诊人姓名
void send_visitAge(uint8_t age);      //修改屏幕就诊人年龄
void send_visitSex(uint16_t sex);   //修改屏幕就诊人性别
void send_QRInfo(uint8_t* data,uint8_t dataLen);      //修改屏幕二维码信息
void send_LcdSync(uint8_t status);      //修改屏幕运行状态
void send_NetSync(uint8_t status);        //修改屏幕网络运行状态
void send_StartPic(uint8_t status);       //修改屏幕启动状态图片
void page_ctl_process(uint8_t ctlNum);      //屏幕控制处理 
void send_duan_wang(uint8_t duan);
void send_lcdQRcode(void);
void send_LcdOutStatus(uint8_t status);      //理疗绿灯 理疗结束红灯
void send_LcdWorkStatus(uint8_t status);
void Send_LcdVoltage(uint16_t voltage);
 
void LcdData_Handle(uint8_t *src,uint16_t len);    //lcd串口数据处理
void Send_LcdDevid_id(int32_t Devid);  //发送设备id到屏幕显示
void Send_LcdVersion(uint8_t* version,uint8_t versionLen);	 //发送版本号到屏幕显示 

void Send_Text_Box(uint8_t state);
void Send_Text_Content(uint8_t* data,uint8_t dataLen);
void Send_Text_SetButton(uint8_t page,uint8_t state);


void Lcd_Task(void const * argument);
bool lcdTask_Semaphore_Init(void);
/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
