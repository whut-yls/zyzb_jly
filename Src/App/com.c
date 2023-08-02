#include "FreeRTOS.h"
#include "event_groups.h"
#include "main.h"
#include "string.h"
#include "startDefaultTask.h"
#include "common.h"
#include "com.h"
#include "crc16.h"
#include "MS5637.h"
#include "AHT15.h"
#include "ad9833.h"
#include	"myAdc.h"
#include "pca9554.h"
//HAL_StatusTypeDef HAL_UART_Transmit_Board1( uint8_t *pData, uint16_t Size, uint32_t Timeout)
//{	
//   HAL_UART_Transmit(&huart4, pData, Size, Timeout);
//	return HAL_OK;
//}
//HAL_StatusTypeDef HAL_UART_Transmit_Board2( uint8_t *pData, uint16_t Size, uint32_t Timeout)
//{	
//   HAL_UART_Transmit(&huart2, pData, Size, Timeout);
//	return HAL_OK;
//}
//HAL_StatusTypeDef HAL_UART_Transmit_Board3( uint8_t *pData, uint16_t Size, uint32_t Timeout)
//{	
//   HAL_UART_Transmit(&huart5, pData, Size, Timeout);
//	return HAL_OK;
//}
//HAL_StatusTypeDef HAL_UART_Transmit_Board4( uint8_t *pData, uint16_t Size, uint32_t Timeout)
//{	
//   HAL_UART_Transmit(&huart8, pData, Size, Timeout);
//	return HAL_OK;
//}
HAL_StatusTypeDef HAL_UART_Transmit_Wifi( uint8_t *pData, uint16_t Size, uint32_t Timeout)   //BY YLS 2023/3/10 add
{	
		if(gUartPcInit==true)
	 {
		return HAL_OK;
	 }
	 gUartPcTc=true;

//   HAL_UART_Transmit(&huart3, pData, Size, Timeout); //debug
 	 HAL_UART_Transmit_DMA(&huart3, pData, Size); //debug
	 gUartPcTc=false;
     
   
	return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_Pc( uint8_t *pData, uint16_t Size, uint32_t Timeout,uint8_t port)
{	
		if(gUartPcInit==true)
	 {
		return HAL_OK;
	 }
	 gUartPcTc=true;
   HAL_UART_Transmit(&huart7, pData, Size, Timeout); //debug
	 gUartPcTc=false;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_Lcd( uint8_t *pData, uint16_t Size,uint32_t Timeout)
{	
		if(gUartLcdInit==true)
	 {
		return HAL_OK;
	 }
	 gUartLcdTc=true;
   HAL_UART_Transmit(&huart7, pData, Size, Timeout);   //debug
	 gUartLcdTc=false;
	return HAL_OK;
}


//-------------------------
//常规回执
void common_return(CommonReturn *ret,uint8_t com) {
	uint8_t send[20] = { 0, };
	uint16_t len, crc16;
	memset(send, 0, sizeof(send));
	len = 9; //CMD_BASE_LEN+sizeof(CommonReturn);
	send[0] = DATA_HEAD;
	send[len - 1] = DATA_END;
	memcpy(&send[1], &len, 2); //len
	send[3] = ret->cmd_type;  //type
	send[4] = ret->cmd_num;   //num
	send[5] = ret->cmd_status;
	crc16 = CRC16_MODBUS(&send[3], 3);

	memcpy(&send[6], &crc16, 2);

	HAL_UART_Transmit_Pc( send, len, 0xffff,com);
	return;
}

//单路adc回执
void single_adc_return(int16_t adcValue,uint8_t com) {
	uint8_t send[20] = { 0, };
	uint16_t len, crc16;
	memset(send, 0, sizeof(send));
	len = 11;
	send[0] = DATA_HEAD;
	send[len - 1] = DATA_END;
	memcpy(&send[1], &len, 2); //len
	send[3] = CMD_TYPE_DPJ;  //type
	send[4] = CMD_READ_ADC;   //num
	send[5] = SUCCESS;
	memcpy(&send[6], &adcValue, 2);
	crc16 = CRC16_MODBUS(&send[3], 3);

	memcpy(&send[8], &crc16, 2);

	HAL_UART_Transmit_Pc(send, len, 0xffff,com);
	return;
}

//数据格式检查
bool check_baseData(uint8_t *src, uint16_t len) {
	//bool ret;
	uint16_t proLen, crcVal, trueCrc;
	if (src[0] == DATA_HEAD && src[len - 1] == DATA_END) {
		memcpy(&proLen, &src[1], 2);
		memcpy(&crcVal, &src[len - 3], 2);
		trueCrc = CRC16_MODBUS(&src[3], len - 6);
		if (proLen == len) {
			if (crcVal == trueCrc) {
				return true;
			} else {
				return false;
			}
		} else {
			return false;
		}
	} else {
		return false;
	}
}
/*发送传感器数据*/
void send_sensor_data(bool retFlag,uint8_t com) {
	uint8_t send[20] = { 0, };
	uint16_t len, crc16;
	uint16_t u16val;
	int16_t val16;
	memset(send, 0, sizeof(send));
	len = 15;   //CMD_BASE_LEN+sizeof(CommonReturn);
	send[0] = DATA_HEAD;
	send[len - 1] = DATA_END;
	memcpy(&send[1], &len, 2); //len
	send[3] = CMD_TYPE_DPJ;  //type
	send[4] = CMD_SENSOR_DATA;   //num
	send[5] = retFlag;

	u16val = (uint16_t) ((int) (gSensorData2.InnerPressure * 10)); /* *气压 *10倍 **/
	memcpy(&send[6], &u16val, 2);

	val16 = gSensorData1.RH * 10;	  //湿度数据
	memcpy(&send[8], &val16, 2);

	u16val = gSensorData1.RHt * 10;	  //温度 保留1位小数
	memcpy(&send[10], &u16val, 2);

	crc16 = CRC16_MODBUS(&send[3], len - 6);

	memcpy(&send[12], &crc16, 2);
	HAL_UART_Transmit_Pc(send, 15, 0xffff,com);
	return;
}

//发送电压值单个
void send_single_vol(uint8_t channel, bool retFlag,uint8_t com) {
	uint8_t send[20] = { 0, };
	uint16_t len, crc16;
	memset(send, 0, sizeof(send));
	len = 11;	  //CMD_BASE_LEN+sizeof(CommonReturn);
	send[0] = DATA_HEAD;
	send[len - 1] = DATA_END;
	memcpy(&send[1], &len, 2); //len
	send[3] = CMD_TYPE_DPJ;  //type
	send[4] = CMD_READ_ADC;   //num
	send[5] = retFlag;
//	memcpy(&send[6], &gAdcVol[channel - 1], 2);
	memcpy(&send[6], &gADC1_VALUE_F[0], 2);
	crc16 = CRC16_MODBUS(&send[3], len - 6);

	memcpy(&send[8], &crc16, 2);
	HAL_UART_Transmit_Pc(send, 11, 0xffff,com);
	return;
}

//发送电压值 批量 （保留）
#if 0
void send_board_vol(uint8_t board, bool retFlag,uint8_t com) {
	int i, j;
	uint16_t channel;
	uint8_t send[100] = { 0, };
	uint16_t len, crc16;
	channel = (board - 1) * 8;	//开始位置
	memset(send, 0, sizeof(send));
	len = 9 + 8 * 2;	//CMD_BASE_LEN+sizeof(CommonReturn);
	send[0] = DATA_HEAD;
	send[len - 1] = DATA_END;
	memcpy(&send[1], &len, 2); //len
	send[3] = CMD_TYPE_DPJ;  //type
	send[4] = CMD_READ_BOARD_ADC;   //num
	send[5] = retFlag;
	for (i = 0; i < 8; i++) {
		j = i * 2 + 6;
		memcpy(&send[j], &gAdcVol[channel], 2);
		channel += 1;
	}
	crc16 = CRC16_MODBUS(&send[3], len - 6);

	memcpy(&send[len - 3], &crc16, 2);
	HAL_UART_Transmit_Pc(send, len, 0xffff,com);
	return;
}
#endif
//lcd串口数据处理
void Console_Handle(uint8_t *src, uint16_t len,uint8_t com) {
	uint8_t cmdNum;
	bool retFlag = false;
	CommonReturn ret;
	uint32_t temp;
	//uint16_t pos;
	if (check_baseData(src, len) == false) {
		return;
	}
	cmdNum = src[4];

	switch (cmdNum) {
	case	CMD_INPUT_CLOSE:	//关闭某个通道 输出端
		if (src[5]<1||src[5]>32) {
			return;
		}
		/*关闭继电器*/
		set_channle_status(src[5],DIR_IN,sOFF);
		//回执
		ret.cmd_type = CMD_TYPE_DPJ;
		ret.cmd_num = CMD_INPUT_CLOSE;
		ret.cmd_status = SUCCESS;
		common_return(&ret,com);
		break;
	case CMD_INPUT_OPEN:	//打开某个通道 输出端
		if (src[5]<1||src[5]>32) {
			return;
		}
		/*选通继电器*/
		set_channle_status(src[5],DIR_IN,sON);
		//回执
		ret.cmd_type = CMD_TYPE_DPJ;
		ret.cmd_num = CMD_INPUT_OPEN;
		ret.cmd_status = SUCCESS;
		common_return(&ret,com);
		break;
	case	CMD_OUTPUT_CLOSE:	//关闭某个通道 输出端
		if (src[5]<1||src[5]>32) {
			return;
		}
		/*关闭继电器*/
		set_channle_status(src[5],DIR_OUT,sOFF);
		//回执
		ret.cmd_type = CMD_TYPE_DPJ;
		ret.cmd_num = CMD_OUTPUT_CLOSE;
		ret.cmd_status = SUCCESS;
		common_return(&ret,com);
		break;
	case CMD_OUTPUT_OPEN:	//打开某个通道 输出端
		if (src[5]<1||src[5]>32) {
			return;
		}
		/*选通继电器*/
		set_channle_status(src[5],DIR_OUT,sON);
		//回执
		ret.cmd_type = CMD_TYPE_DPJ;
		ret.cmd_num = CMD_OUTPUT_OPEN;
		ret.cmd_status = SUCCESS;
		common_return(&ret,com);
		break;
	case CMD_READ_ADC:	//单个读取

		retFlag = SUCCESS;
		send_single_vol(src[5], retFlag,com);
		break;
//		case CMD_READ_BOARD_ADC:	//批量读取
//			break;
	case CMD_SENSOR_DATA:	//读取气压、湿度、温度
		retFlag = SUCCESS;
		send_sensor_data(retFlag,com);
		break;
	case CMD_SENSOR_RESISTOR:	//调整输出电压
		AD9833_AmpSet(src[5]);
		ret.cmd_type = CMD_TYPE_DPJ;
		ret.cmd_num = CMD_SENSOR_RESISTOR;
		ret.cmd_status = SUCCESS;
		common_return(&ret,com);
		break;
	case CMD_FREQUENCY_CHANGE:
		temp = src[5]  |src[6] << 8 | src[7]<< 16 | src[8]<< 24;
		AD9833_WaveSeting(temp, 0, src[9] > SQU_WAVE ? SIN_WAVE : src[9], 0);
		ret.cmd_type = CMD_TYPE_DPJ;
		ret.cmd_num = CMD_FREQUENCY_CHANGE;
		ret.cmd_status = SUCCESS;
		common_return(&ret,com);
		break;
	case CMD_WORK_MODE:
		if(src[5]<1||src[5]>4)
			return;
		set_sampleMode(src[5]);
		ret.cmd_type = CMD_TYPE_DPJ;
		ret.cmd_num = CMD_WORK_MODE;
		ret.cmd_status = SUCCESS;
		common_return(&ret,com);
		break;
	case CMD_IO_CLOSE:
//		HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(IO1_GPIO_Port, IO1_Pin, GPIO_PIN_RESET);
		HAL_PCA9554_outputAll(0);
		ret.cmd_type = CMD_TYPE_DPJ;
		ret.cmd_num = CMD_IO_CLOSE;
		ret.cmd_status = SUCCESS;
		common_return(&ret,com);
		break;
	default:
		break;
	}

	return;
}

