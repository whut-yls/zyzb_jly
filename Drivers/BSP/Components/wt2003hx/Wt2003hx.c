#include "Wt2003hx.h"



const uint8_t header_Song[1]={0x7E};

//发送指令指定播放音乐
/*
文件名固定4 个字符，不支持中文
*/
void Send_PlayMusic(uint8_t *file)
{
	uint8_t buf[20]={0x00},tmp;
	uint16_t len,crc16=0,i;
	len=7;
	memset(buf,0,sizeof(buf));
	buf[0]=header_Song[0];
	buf[1]=len;

	tmp=Play_by_name;
	buf[2]=tmp;

	for(i=0;i<len-3;i++)
	{
		buf[i+3]=file[i];
	
	}	
	for(i=1;i<len;i++)
	{
		crc16+=buf[i];
	}	
	buf[7]=crc16;    
	buf[8]=0xEF;

	HAL_UART_Transmit(&huart2,buf,strlen(buf),0xFFFF);		
}	


//发送指令控制音乐
/*

0xAA     //暂停放音命令（AA）  1 2
0xAB     //停止命令（AB）   	3
0xAC     //下一曲命令(AC)   	5
0xAD     //上一曲命令(AD) 	  4

*/
void Send_ComMusic(uint8_t com)
{
	uint8_t buf[20]={0x00},tmp;
	uint16_t len,crc16=0;
	len=3;
	memset(buf,0,sizeof(buf));
	buf[0]=header_Song[0];
	buf[1]=len;
	switch(com)
	{
		case 1: tmp=Pause_Start;
			break;
		case 2: tmp=Pause_Start;
			break;
		case 3: tmp=Stop_Com;
			break;
		case 4: tmp=Last_Step_Com;
			break;
		case 5: tmp=Next_Step_Com;
			break;
		default:
			break;
		
	
	}
	buf[2]=tmp;
	crc16 =tmp+len;
	buf[3]=crc16;
	buf[4]=0xEF;

	HAL_UART_Transmit(&huart2,buf,strlen(buf),0xFFFF);	

}

//发送指令控制音量大小  拥有掉电记忆
void Send_ComVolume(uint8_t size)
{
	uint8_t buf[20]={0x00},tmp;
	uint16_t len,crc16=0;
	len=4;
	memset(buf,0,sizeof(buf));
	buf[0]=header_Song[0];
	buf[1]=len;
	
	tmp=Volume_Control;
	buf[2]=tmp;
	buf[3]=size;	
	crc16=size+tmp+len;
	buf[4]=crc16;
	buf[5]=0xEF;

	HAL_UART_Transmit(&huart2,buf,strlen(buf),0xFFFF);	

}

//发送指令控制播放模式
/*
00:单曲不循环
01:单曲循环播放
02:所有曲目循环播放
03:随机播放
*/
void Send_ComMode(uint8_t mode)
{
	uint8_t buf[20]={0x00},tmp;
	uint16_t len,crc16=0;
	len=4;
	memset(buf,0,sizeof(buf));
	buf[0]=header_Song[0];
	buf[1]=len;
	
	tmp=Assigned_Play;
	buf[2]=tmp;
	buf[3]=mode;	
	crc16=mode+tmp+len;
	buf[4]=crc16;

	buf[5]=0xEF;

	HAL_UART_Transmit(&huart2,buf,strlen(buf),0xFFFF);	

}

//发送指令控制组合播放
void Send_Combination_play(uint8_t length,uint16_t *mode)
{
	uint8_t buf[40]={0x00},tmp;
	uint16_t utmp16=0,len,crc16=0,i,j=4;
	len=4+length*2;
	memset(buf,0,sizeof(buf));
	buf[0]=header_Song[0];
	buf[1]=len;
	
	tmp=Combination_play;
	buf[2]=tmp;
	buf[3]=length;

	for(i=0;i<length;i++)
	{
		utmp16=mode[i];
		buf[j++]=utmp16>>8;
		buf[j++]=(uint8_t)utmp16;

	}
	crc16=0;
	for(i=1;i<len;i++)
	{
	crc16+=buf[i++];

	}
	
	buf[j++]=crc16;
	buf[j++]=0xEF;

	HAL_UART_Transmit(&huart2,buf,strlen(buf),0xFFFF);	

}

//发送插播指令
/*
第一次插播命令未播放完时，发第二次插播命令时，命令无效。要等第一次插播音乐播放完后才可以再一次进行插播。
*/
void Send_Insert_PMusic(uint16_t position)
{
	uint8_t buf[20]={0x00},tmp;
	uint16_t utmp16=0,len,crc16=0,i;
	len=5;
	memset(buf,0,sizeof(buf));
	buf[0]=header_Song[0];
	buf[1]=len;
	
	tmp=Inter_cut;
	buf[2]=tmp;
	
	utmp16=position;
	buf[3]=utmp16>>8;
	buf[4]=utmp16;

	crc16=0;
	for(i=1;i<len;i++)
	{
		crc16+=buf[i];
	}
	buf[5]=crc16;
	buf[6]=0xEF;
	
	HAL_UART_Transmit(&huart2,buf,strlen(buf),0xFFFF);	
}
	
//发送音频输出方式切换
/*
参数：00 表示SPK 输出，01 表示DAC 输出  默认spk
*/
void Send_Output_Change(uint8_t mode)
{
	uint8_t buf[20]={0x00},tmp;
	uint16_t len,crc16=0;
	len=4;
	memset(buf,0,sizeof(buf));
	buf[0]=header_Song[0];
	buf[1]=len;
	
	tmp=Audio_Mode_Change;
	buf[2]=tmp;
	buf[3]=mode;

	crc16=len+tmp+mode;
	buf[4]=crc16;

	buf[5]=0xEF;

	
	HAL_UART_Transmit(&huart2,buf,strlen(buf),0xFFFF);	
}


//发送指令进入低功耗
/*
上电后不会主动进入休眠模式，发送指令才会进入
指令"00"功耗小于2uA，指令唤醒时间为100 毫秒
指令"01"功耗小于30uA，指令唤醒时间为2 毫秒

注：芯片在RXD 脚下降沿行唤醒，建议发送0x00 0x00 进行唤醒（也可连发两条同样指令唤醒）。使用外挂Flash 方案时，休眠功耗与外挂FLash 有关。
唤醒后不会再主动进入休眠模式，再次发送进入休眠指令后，才有效！
*/
void Send_Goto_Low_Power(uint8_t mode)
{
	uint8_t buf[20]={0x00},tmp;
	uint16_t len,crc16=0;
	len=4;
	memset(buf,0,sizeof(buf));
	buf[0]=header_Song[0];
	buf[1]=len;
	
	tmp=Low_Power;
	buf[2]=tmp;
	buf[3]=mode;

	crc16=len+tmp+mode;
	buf[4]=crc16;

	buf[5]=0xEF;

	HAL_UART_Transmit(&huart2,buf,strlen(buf),0xFFFF);	
}


//查询当前软件版本
/*
7E 19 C0 57 54 43 32 32 30 33 32 36 2D 31 35 30 2D 41 32 30 56 34 2E 30 30 A1 EF
‘ 57 54 43 32 32 30 33 32 36 2D 31 35 30 2D 41 32 30 56 34 2E 30 30 ’ 16 进制转字符串表示为：
WTC220326-150-A20V4.00 ，”WTC”：公司代指，”220326”：2022 年03 月26 日有发布一版程序，”150”：
本司内部代码，”A20”：对应选型表功能代码，“V4.00”：对应版本号
*/
void Send_Current_Version(void)
{
	uint8_t buf[20]={0x00},tmp;
	uint16_t len,crc16=0;
	len=3;
	memset(buf,0,sizeof(buf));
	buf[0]=header_Song[0];
	buf[1]=len;
	
	tmp=Current_Version;
	buf[2]=tmp;

	crc16=len+tmp;
	buf[3]=crc16;

	buf[4]=0xEF;

	HAL_UART_Transmit(&huart2,buf,strlen(buf),0xFFFF);	
}


//查询当前设置音量
void Send_Current_Volume(void)
{
	uint8_t buf[20]={0x00},tmp;
	uint16_t len,crc16=0;
	len=3;
	memset(buf,0,sizeof(buf));
	buf[0]=header_Song[0];
	buf[1]=len;
	
	tmp=Current_Volume;
	buf[2]=tmp;

	crc16=len+tmp;
	buf[3]=crc16;

	buf[4]=0xEF;

	HAL_UART_Transmit(&huart2,buf,strlen(buf),0xFFFF);	
}


//查询当前工作状态
/*
：?: 01 表示：播放；
?: 02 表示：停止；
?: 03 表示：暂停；

*/

void Send_Current_Status(void)
{
	uint8_t buf[20]={0x00},tmp;
	uint16_t len,crc16=0;
	len=3;
	memset(buf,0,sizeof(buf));
	buf[0]=header_Song[0];
	buf[1]=len;
	
	tmp=Read_Current_Status;
	buf[2]=tmp;

	crc16=len+tmp;
	buf[3]=crc16;

	buf[4]=0xEF;

	HAL_UART_Transmit(&huart2,buf,strlen(buf),0xFFFF);	
}

//查询当前音乐文件总数
void Send_Number_Song(void)
{
	uint8_t buf[20]={0x00},tmp;
	uint16_t len,crc16=0;
	len=3;
	memset(buf,0,sizeof(buf));
	buf[0]=header_Song[0];
	buf[1]=len;
	
	tmp=Query_file_total;
	buf[2]=tmp;

	crc16=len+tmp;
	buf[3]=crc16;

	buf[4]=0xEF;

	HAL_UART_Transmit(&huart2,buf,strlen(buf),0xFFFF);	
}


//查询当前播放音乐文件地址
void Send_Number_Address(void)
{
	uint8_t buf[20]={0x00},tmp;
	uint16_t len,crc16=0;
	len=3;
	memset(buf,0,sizeof(buf));
	buf[0]=header_Song[0];
	buf[1]=len;
	
	tmp=Query_current_play;
	buf[2]=tmp;

	crc16=len+tmp;
	buf[3]=crc16;

	buf[4]=0xEF;

	HAL_UART_Transmit(&huart2,buf,strlen(buf),0xFFFF);	
}

//波特率切换
/*
此命令有掉电记忆（上电默认波特率9600）
波特率参数设置为十六进制显示，以上“参数”值“01 C2 00”（三字节表示）对应115200，
*/
void Send_Change_Baud(uint8_t *parameter)
{
	uint8_t buf[20]={0x00},tmp;
	uint16_t len,crc16=0,i;
	len=6;
	memset(buf,0,sizeof(buf));
	buf[0]=header_Song[0];
	buf[1]=len;
	
	tmp=Change_Baud;
	buf[2]=tmp;
	
	for(i=0;i<len-3;i++)
	{
		buf[i+3]=parameter[i];
	
	}
	for(i=1;i<len;i++)
	{
		crc16+=buf[i];
	}
	buf[6]=crc16;
	buf[7]=0xEF;

	HAL_UART_Transmit(&huart2,buf,strlen(buf),0xFFFF);	
}



