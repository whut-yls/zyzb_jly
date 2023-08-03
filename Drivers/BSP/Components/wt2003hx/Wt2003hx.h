#ifndef _WT2003HX_H
#define _WT2003HX_H

#include "stdio.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
/* default header*/
#include "main.h"

#define Play_Root_Index  0xA0     //指定Flash 根目录索引播放
#define Play_by_name     0xA3     //指定Flash 根目录文件名播放  注:文档上写的是A1，但实际上是A3
#define Pause_Start      0xAA     //暂停放音命令（AA）
#define Stop_Com         0xAB     //停止命令（AB）
#define Next_Step_Com    0xAC     //下一曲命令(AC)
#define Last_Step_Com    0xAD     //上一曲命令(AD)

#define Volume_Control    0xAE     //音量控制命令
#define Assigned_Play     0xAF     //指定播放模式
#define Combination_play  0xB0     //组合播放模式
#define Inter_cut         0xB1     //插播指令
#define Audio_Mode_Change 0xB6     //音频输出方式切换
#define Low_Power         0xB8     //进入低功耗模式
#define Current_Version   0xC0     //查询当前软件版本
#define Current_Volume    0xC1     //查询当前设置音量
#define Read_Current_Status 0xC2   //读取当前工作状态
#define Query_file_total   0xC3    //查询FLASH 内音乐文件总数
#define Query_current_play 0xC9    //查询当前播放
#define Change_Baud        0xFB    //波特率切换

void Send_PlayMusic(uint8_t *file);
void Send_ComMusic(uint8_t com);
void Send_ComVolume(uint8_t size);
void Send_ComMode(uint8_t mode);
void Send_Combination_play(uint8_t length,uint16_t *mode);
void Send_Insert_PMusic(uint16_t position);
void Send_Output_Change(uint8_t mode);
void Send_Goto_Low_Power(uint8_t mode);
void Send_Current_Version(void);
void Send_Current_Volume(void);
void Send_Current_Status(void);
void Send_Number_Song(void);
void Send_Number_Address(void);
void Send_Change_Baud(uint8_t *parameter);



#endif


