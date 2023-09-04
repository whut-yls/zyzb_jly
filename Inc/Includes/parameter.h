/**
  ******************************************************************************
  * @file           : parameter.h
  * @brief          : Header for parameter.c file.
  *                   This file contains parameter
  ******************************************************************************
  
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETER_H__
#define __PARAMETER_H__

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
/***private include**/


#ifdef __cplusplus
 extern "C" {
#endif


#define SECTOR_PARAMETER		0X00*4096													/***/
//#define SECTOR_HOME					0X01*4096													/***/
//#define SECTOR_GONGYI				0X02*4096													/***/

#define FLASH_MAGIC					0x8765431A										/*parameter header*/



/** 
		* @brief	MCU 
*/


#define	Device_ID_Max	99999999

#define SoftVersion  "v1.0.0"
#define HardVersion  "v1.0.0"

#define NET_UNLINK 0	//断开
#define NET_LINK	1	//连接
#define NET_ERR	2	//异常
//上传间隔
#define DefUpTime    50   //10ms
#define	UpTime_MAX		100
#define	UpTime_MIN		10

//输出档位
//#define DefLevel    128  
#define	Level_MAX		255
#define	Level_MIN		0

#define DeftimeT			3	//单通道采样时长单位s ,1~60000
#define TimeT_MAX			6000	//单位s
#define TimeT_MIN			1	
//默认的数据端口
#define DefMqttPort  9300
#define DefHttpPort  9200

#define DefWorkMode		1	// 默认1 工作模式 1：整体采样，2：局部采样，3：经络治疗
#define DefLevel			10	//默认125 输出档位（可调电阻调整），档位区间0-255区间


#define DefWaveTreat	2	//治疗波形，取值范围 1三角波,2正弦波，3矩形波
#define DefFreqTreat	50	//		//治疗频率，取值范围25，50，100，
#define DefWaveCheck	2		//诊断波形，取值范围 1三角波,2正弦波，3矩形波
#define DefFreqCheck	50		//诊断频率，取值范围25，50，100，

#define DefHeartRate	30	//默认的心跳间隔 单位s

#define HeartRate_MAX 10000
#define HeartRate_MIN 0

#define WORK_MODE_WT	0	//等待
#define WORK_MODE_ZT	1	//整体
#define WORK_MODE_JB	2	//局部
#define WORK_MODE_ZL	3	//治疗 
//#define WORK_MODE_ZLZT	3	//治疗 整体
//#define WORK_MODE_ZLJB	4	//治疗 局部

//#define WORK_MODE_ZLZT_DC	5	//治疗 整体 直流
//#define WORK_MODE_ZLZT_AC 3	//治疗 整体 交流 debug
////#define WORK_MODE_ZLZT_AC 6	//治疗 整体 交流
//#define WORK_MODE_ZLJB_DC	7	//治疗 局部 直流
//#define WORK_MODE_ZLJB_AC	8	//治疗 局部 交流


#define NET_RJ45	1		//当前链接的网络状态
#define NET_WIFI	2
#define NET_4G		3

#define SIM_NO	0	//未插卡
#define SIM_YD	1	//移动
#define SIM_LT	2	//联通
#define SIM_DX	3	//电信

#define WORK_WAIT			1			//1:等待,2:采集中,3,治疗中.
#define WORK_COLLECT	2
#define WORK_TREAT		3

#define WORK_START		1	//开始
#define WORK_PAUSE		2	//暂停
#define WORK_STOP			3	//停止

#define UNKNOWN       0 //未知
#define WORKING		    1	//工作中
#define LEISURE		    2	//空闲中
#define PAUSE         3 //暂停中
#define WAITING		    4	//等待中
//采集频率
#define DefFreqN    100  //100 个点每秒
#define	CollectFreq_MAX	 1000  //1000
#define	CollectFreq_MIN	 1   //1

#define LOCK_YES	1
#define LOCK_NO	2

#define JLY_lenth	18   //JLY二维码长度
#define JSON_ACK_MAX_LEN  1024 
#define JSON_DATA_MAX_LEN  2100//100*7*2+8(id)+2(len)+2(head)=1412

#define STATUS_OK 200
#define STATUS_FAIL 400 //参数错误

#define FORWARD 1
#define INVERT  0
#define PressErrorLeve1  50  //mv 自动压力的偏差电压
#define PressErrorLeve2  2

#define SleepBaseTime         10
#define CtlSendTime    	100
#define DataSendTime  	400

#define FUN_COLLECT_ARG 1      //设置采集方案     debug   1
#define FUN_USER_INFO   2       	//用户基本信息
#define FUN_RJ45_GET    3       	//RJ45网口信息配置
#define FUN_RJ45_SET    4       	//RJ45网口信息获取
#define FUN_WIFI_GET    5      	//WIFI网口信息配置
#define FUN_WIFI_SET    6       	//WIFI网口信息获取
#define FUN_DEV_GET     7      		//DEV信息获取
#define FUN_DEV_SET     8     	 	//DEV信息获取
#define FUN_COLLECT_CTL	9				//采集控制
#define	FUN_DEV_OFF			10			//关机指令
#define FUN_DEV_RESET   11  //设备重启
#define FUN_DEV_LOCK		12			//设备锁定
#define FUN_SYS_CHECK		13			//系统信息
#define FUN_JIANCHE_DATA 14		//心跳指令
#define FUN_UPDATA			15			//升级指令
 
#define FUN_SAVE_ARG	16	//保存当前工作参数为默认工作参数

#define FUN_COLLECT_TREAT	19	  //采集结束后下发的治疗方案

#define FUN_MESSAGE_BOX    29      //下发显示消息提示框
#define FUN_PLAY_CONTENT   30         //下发播放声音文件
#define FUN_PLAY_CTL       31        //下发播放控制
#define FUN_VOLUME_CTL       32       //下发音量控制
#define FUN_LIGHT_CTL      33        //灯光控制
#define FUN_LEVEL_CTL      34        //加减档控制
#define FUN_AUTO_LEVEL_CTL   39    //自动加减档控制

#define FUN_AUDIO_CTL      36        //音频控制


//#define FUN_DEV_ID      11    //设备id 包含devId
//#define FUN_DEV_MAC     12  //设备MAC 包含macValue
//#define FUN_DEV_IP      13  //设备IP 包含devIp,devMask,devGate
//#define FUN_SERVER_IP   14  //服务IP 包含serverIp,dataPort,ctlPort
//#define FUN_MOTOR_STATUS  15  //电机状态
//http 指令key字段
#define KEY_ERY_CODE	"ErrCode"
#define KEY_ERR_MSG		"errMsg"
#define KEY_DATA	"Data"
//mqtt 指令key字段
#define KEY_DEV_ID    "devId"   //设备id
#define KEY_SESSION   "sessionId" //会话id
#define KEY_FUN   "function" //功能标识
#define KEY_TIME  "time"  //包时间
#define KEY_CODE	"statusCode"	//状态码200 正常 400异常
#define KEY_MSG		"msg"		//返回正常或者异常消息
#define	KEY_HEAR_RATE	"heartRate"	//心跳间隔
#define KEY_PRODUCT		"productKey"	//产品标识
#define	KEY_SOFT_VER	"Softversion"	//软件版本
#define	KEY_HARD_VER	"hardwareVersion"	//硬件版本
#define KEY_FLAG  "flage"      //设备标识
#define KEY__WORK_CURRENT "electricity"    //主板总电流

//采集方案相关
#define KEY_DATA_UPTIME  "interval" //数据上传间隔
#define KEY_WORK_MODE  "workmode" //工作模式 1：整体采样，2：局部采样，3：经络治疗
#define	KEY_LEVEL				"level"	//
#define KEY_DATA_RATE   "rate"  //数据上传频率
#define KEY_INPUTS			"inputs"	//输入通道列表
#define KEY_OUTS				"outs"	//输出通道列表，整体采样输出通道固定33，局部采样与输入通道一一对应
#define KEY_WAVE_TREAT			"wave"	//治疗波形，取值范围 1三角波,2正弦波，3矩形波，该字段仅经络治疗使用
#define KEY_FREQ_TREAT			"freq"
#define	KEY_TIME_WAIT			"waitTime"	//等待时间
#define	KEY_TIME_CJ	      "totalTime"  //采集总时长
#define	KEY_TIME_ZL			    "time"    //治疗时间
#define KEY_WAVE_CHECK			"waveCheck"
#define KEY_FREQ_CHECK			"freqCheck"
#define	KEY_TIME_CHECK			"timeCheck"
#define KEY_WAVE_FREQ		"frequency"	//波形频率，取值范围25，50，100，该字段仅经络治疗使用
#define KEY_TYPE		"collectType"	//发采集方案的  整体采样和局部采样


//用户信息下发 相关
#define	KEY_NUMBER	"number"	////用户编号
#define	KEY_NAME		"name"		//名字
#define	KEY_SEX				"sex"	//性别
#define	KEY_AGE				"age"	//年龄

//RJ45网口
#define	KEY_IP	"ip"
#define	KEY_MASK	"mask"
#define	KEY_MAC		"mac"
#define	KEY_GATE	"gate"

//WIFI接口
#define	KEY_WIFI_NAME	"wifiName"
#define	KEY_WIFI_PWD	"wifiPwd"


#define KEY_SERVER_IP "serverIp"  //服务器ip
#define KEY_SERVER_PORT "serverPort"  //数据流端口

//网络状态监控
#define KEY_RJ45	"rj45"	//rj45状态0正常 1断开 2设备异常
#define	KEY_WIFI	"wifi"
#define	KEY_4G		"4g"

//系统监控数据
#define	KEY_LONGITUDE		"longitude"		//经度
#define KEY_LATITUDE	"latitude"	//纬度
#define KEY_SIM	"sim"		
#define KEY_SIGNAL	"signal"	//信号强度
#define KEY_RUN_TIME	"runTime"	//运行时间
#define KEY_UP_RATE		"uplinkRate"	//上传速度
#define KEY_DOWN_RATE	"downlinkRate"	
#define KEY_PING	"ping"	//ping 值

#define KEY_WORK	"works"
#define KEY_VALUE	"value"
#define KEY_NET_STATE	"networkState"	//// 网络状态 0正常，1断开
#define KEY_CUR_NET	"currentNet"	//当前使用网络 1:RJ45,2:wifi,3:4G
#define KEY_WORK_STATE	"workState"	//工作状态：1:等待,2:采集中,3,治疗中.
#define KEY__WORK_TIME_LAST "workTime"//当前工作状态剩余时间

#define KEY_PARA	"parameter"
#define KEY_ENV	"environment"
#define KEY_TEMP	"temperature"
#define KEY_HUMID	"humidity"		//湿度
#define KEY_PRESSURE	"pressure"	//气压
#define KEY_NEW_ID	"newDevid"
#define KEY_LEFT_Code "leftCode"   //jly左侧二维码标识
#define KEY_RIGHT_Flage "rightFlage"
#define KEY_Mquser "mqUser"    //mq用户名
#define KEY_Mqpassword "mqPassword"    //mq密码


//升级指令
#define KEY_UPDATE_URL	"updateUrl"	////更新接口
#define KEY_SYNC_URL	"syncUrl"	//更新后同步接口

#define IO1Stauts          	(0x1UL << 0)
#define IO2Stauts          	(0x1UL << 1)
#define IO3Stauts          	(0x1UL << 2)
#define IO4Stauts          	(0x1UL << 3)
#define IO5Stauts          	(0x1UL << 4)
#define IO6Stauts          	(0x1UL << 5)
#define IO7Stauts          	(0x1UL << 6)
#define IO8Stauts          	(0x1UL << 7)

#define IO_GET_STATUS_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__) & (__FLAG__)) == (__FLAG__))
#define IO_CLEAR_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)&= ~(__FLAG__))
#define IO_EN_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)|= (__FLAG__))

//wifi网络
#define WIFI_NAME_LEN	32
typedef struct __Parameter_WIFI{
	//网络相关
	uint8_t			IpAddress[5];   //4+1 ip
  uint8_t			MaskAddress[5];   //4+1 mask
  uint8_t			GateAddress[5];   //4+1 gate
  uint8_t			ServerIpAddress[5];   //4+1 ip
  uint8_t			MacAddress[7];  //6+1 mac
	uint8_t 		WifiName[33];	//wifi名字
	uint8_t 		WifiPwd[33];	//wifi密码
}Parameter_WIFI;

//rj45网络
typedef struct __Parameter_RJ45{
	uint8_t			IpAddress[5];   //4+1 ip
  uint8_t			MaskAddress[5];   //4+1 mask
  uint8_t			GateAddress[5];   //4+1 gate
  uint8_t			ServerIpAddress[5];   //4+1 ip
  uint8_t			MacAddress[7];  //6+1 mac
}Parameter_RJ45;

//mqtt相关
typedef struct __Parameter_MQ{
	char				MQ_Username[48];		/**/
	char				MQ_Password[48];		/**/
	uint16_t  	mqttPort;    //mqtt端口
}Parameter_MQ;

/*用户信息*/
typedef struct __Arg_USER{
	char		number[33];		/**/
	char		name[33];		/**/
	char  	sex[33]; 
	uint8_t age;
	uint8_t infoEn;	//信息是否存在 1存在 0不存在
	bool infoUpdateEn;
}USER_INFO_TYPE;

//经络仪的工作参数
typedef struct __Parameter_WORK{
	uint8_t workMode;	// 默认1 工作模式 1：整体采样，2：局部采样，3：经络治疗
	uint8_t level;	  //默认125 输出档位（可调电阻调整），档位区间0-255区间
	uint16_t timeTreat;	//单通道治疗时长
	uint16_t timeCheck;	//单通道采样时长单位s ,1~60000
	uint16_t	rateN; //采样频率10~1000 每秒采样个数 
	uint16_t	upTime;         //数据上传间隔
	uint16_t  waitTime;     //等待时长
	
	uint8_t waveTreat;		//治疗波形，取值范围 1三角波,2正弦波，3矩形波
//	uint8_t freqTreat;		//治疗频率，取值范围25，50，100，
    uint32_t freqTreat;		//治疗频率，取值范围25，50，100，
	uint8_t waveCheck;		//诊断波形，取值范围 1三角波,2正弦波，3矩形波
	uint8_t freqCheck;		//诊断频率，取值范围25，50，100，
	uint8_t chanelNum;	//通道数目
	uint8_t inputs[32];
	uint8_t outs[32];
}Parameter_Work;
 
typedef struct __Parameter_Dev  /*设备参数-保存*/
{
	//设备相关//位置不要在动，只能往后加 2023.04.26
  int32_t			devId;						//8终端id
	char 		productKey[20];			//设备标识  JLY
	uint16_t 		heartRate;				//心跳间隔
	uint16_t    httpPort;	   //http请求端口
	uint8_t 		devLock;					//设备锁定1锁定 2解锁
	uint8_t ServerIpAddress[5];	//服务器地址
	
	Parameter_Work workArg[2];	//整体采样工作参数    
	Parameter_MQ 		mqArg;
	Parameter_RJ45 	rj45Arg;
	Parameter_WIFI	wifiArg;
	char 		version[6]; //app版本	
	uint8_t qrbuf[100];    //屏幕左侧JLY二维码	

} Parameter_Dev;

typedef struct __SIM_INFO{
	float longitude;	//经度
	float latitude;	//纬度
	
	uint8_t signal;	//信号强度
	uint8_t sim;	//0未插卡 1移动 2联通 3电信
	uint8_t runTime[12];//运行时长
	uint32_t upLinkRate;//上传速率
	uint32_t downLinkRate;//下载速度
	uint32_t ping;	//ping延时
}SIM_INFO_TYPE;

typedef struct __RUN_Global    /*运行全局参数*/
{
//  bool  DataStartFlag;   //是否开启数据上传 开始上传1/停止上传0
	uint16_t  freqUnitTime;//采集频率对应装载一次数据的间隔时间
	uint16_t  unitLen;        //单个通道数据包长度
    uint16_t  currentUnitNum; //当前的单元数据位置
	uint8_t cur_heart_state;	 //回复心跳状态 1：工作 2：空闲 3：暂停 4：等待
	uint8_t curWorkMode;	//当前的工作模式0等待 1整体 2局部 3治疗
	uint8_t oldWorkMode;	//先前的工作模式
	uint8_t curWorkState;	//当前的工作状态 1：开始 2暂停 3停止
	Parameter_Work useWorkArg[20];	//当前使用的实时工作参数         治疗方案序号     debug 2->7
	uint8_t currentNet;	////当前使用网络 1:RJ45,2:wifi,3:4G 
	uint8_t 			heart_sendStatus;				//解析数据心跳包状态
	uint8_t 			rj45Status;	//rj45网口状态	0断开 1正常 2异常
	uint8_t				wifiStatus;	//wifi状态
	uint8_t 			yd4gStatus;	//4g移动网络状态
	uint8_t       netKind;    //1:RJ45, 2:WIFI, 3:4G
	bool    ResetStatus;      //开机连上网复位
	uint32_t		heart_count;//心跳间隔计数器 累计到某阈值未清零说明已无法和服务器维持心跳包
	uint8_t 		heartbag_flage;//发送心跳包标志位
	uint8_t     Auto_Level_Ctl;
	
	bool lcdStatus;	//lcd屏幕通信状态
	
	
	bool isSendMq;//是否发送了连接mq的信息 wifi使用
	uint8_t			isConnect;//判断当前网络是否正常，0：异常断开，1标识正常连接
	uint8_t			conFlage;//当前是那种通信正在连接，1-rj45 2-wifi  3-4g
	
  uint16_t  PlusePressData[JSON_DATA_MAX_LEN]; 
	uint8_t  PlusePressDataSend[JSON_DATA_MAX_LEN*2];
  uint16_t	PlusePressDataLen;//发送
	uint32_t sessionId; //会话id
	
	uint8_t *PlusePressDataAck; //回应数据的储存地址指针

 	uint8_t 	channelPos;	//当前通道位置
  bool Send_Ctl_Task;
  bool Send_Data_Task;

	bool Send_Client_Over;	  //用户信息下发回应
  bool Send_Update_Task;
  bool Send_Ack_Task;
  int  Send_Update_Len;
  bool Send_Data_Sync_Task;
	bool Send_Ping_Task;
	bool Send_Heart_Bag;
	bool Send_Heart_Bag_4G;
	bool Send_Heart_Bag_wifi;
	
	bool count_net_state;      //倒计时状态，要在网络运行正常时才倒计时	
	bool Wifi_set;
	uint16_t current_treatNums;
	uint16_t  Alltime;
	uint16_t current_time;        //未用到该参数


	uint8_t ack_time[20];
	uint8_t ResetDevidcnt;	
}RUN_Global;


extern Parameter_Dev  gDeviceParam;
extern RUN_Global gGlobalData;
extern char gStatusAck[JSON_ACK_MAX_LEN];
extern SIM_INFO_TYPE	gSimInfo;
extern USER_INFO_TYPE gUserInfo;
int Read_Parameter(void);
int Init_Parameter(void);
int Init_All_Parameter(void);
int Restore_Default_Parameter(void);
int Save_Parameter(void);
 
#ifdef __cplusplus
}
#endif

#endif /* __PARAMETER_H__ */

/************************ (C) COPYRIGHT  hj*****END OF FILE****/

