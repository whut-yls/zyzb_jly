#include "dac8831.h"
#include "main.h"
#include "timer.h"



uint8_t  g_spi_busy; /* SPIæ״̬��0��ʾ��æ��1��ʾæ */
#define CS_0() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define CS_1() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
uint8_t g_spiTxBuf[SPI_BUFFER_SIZE];
uint32_t g_spiLen;
void DAC8831_Set_Data(uint16_t data)
{
//	DAC8831_SetCS(0);
	g_spiLen = 0;
	g_spiTxBuf[g_spiLen++] = (data >> 8);
	g_spiTxBuf[g_spiLen++] = (data);
	if (g_spiLen > SPI_BUFFER_SIZE)
	{
		return;
	}

  if(HAL_SPI_Transmit(&hspi3, (uint8_t*)g_spiTxBuf, g_spiLen, 0xffff) != HAL_OK)	//1000000
	{
		Error_Handler();
	}
//	DAC8831_SetCS(1);	
}

void DAC8831_Set_Data_Dma(uint16_t *_pbufch1, uint32_t _sizech1,uint32_t _ulFreq)
{
//	uint32_t i;
//	uint32_t _cmd;
//	
//	g_spiLen = 0;
//	
//	for(i = 0; i < _sizech1; i++)
//	{
//		/* ������Ҫ����PD1��PD0����ǰ��ѡ�����������ģʽ */
//		_cmd = (0 << 16) | (_pbufch1[i] << 0);	
//		g_spiTxBuf[g_spiLen++] = (uint8_t)(_cmd >> 8);
//		g_spiTxBuf[g_spiLen++] = (uint8_t)(_cmd);
//	}	
#if 1
	MX_SPI3_Init();
	
  __HAL_RCC_DMA2_CLK_ENABLE();
	
	hdma_spi3_tx.Instance = DMA2_Stream1;
	hdma_spi3_tx.Init.Request = DMA_REQUEST_SPI3_TX;
	hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_spi3_tx.Init.Mode = DMA_CIRCULAR;                      //DMA_NORMAL
	hdma_spi3_tx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_spi3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;       //DISABLE /* ʹ��FIFO*/
	hdma_spi3_tx.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_1QUARTERFULL;    /* �������÷�ֵ�� �����ֹFIFO��λ��������*/
	hdma_spi3_tx.Init.MemBurst=	DMA_MBURST_SINGLE;	   					 /* ���ڴ洢��ͻ���������ֹFIFO��λ��������*/
	hdma_spi3_tx.Init.PeriphBurst=DMA_PBURST_SINGLE;	 	

	/* ��λDMA */
	if(HAL_DMA_DeInit(&hdma_spi3_tx) != HAL_OK)
	{
		if(HAL_DMA_Abort_IT(&hdma_spi3_tx)!=HAL_OK){
			Error_Handler();   
		}			
	}
	
	if (HAL_DMA_Init(&hdma_spi3_tx) != HAL_OK)
	{
		Error_Handler();
	}

	__HAL_LINKDMA(&hspi3,hdmatx,hdma_spi3_tx);	
	
	/* �ر�DMA�����ж� */
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
	HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
	
	/* �ر�SPI�ж� */
	HAL_NVIC_SetPriority(SPI3_IRQn, 6, 0);
	HAL_NVIC_DisableIRQ(SPI3_IRQn);	

	dmamux_syncParams.EventEnable   = ENABLE; 							
	dmamux_syncParams.SyncPolarity  = HAL_DMAMUX_SYNC_RISING;          
	dmamux_syncParams.RequestNumber = 2;                         //���������1�δ���           
	dmamux_syncParams.SyncSignalID  = HAL_DMAMUX1_SYNC_TIM12_TRGO; /* HAL_DMAMUX1_SYNC_TIM12_TRGO HAL_DMAMUX1_SYNC_LPTIM1_OUT*/
	dmamux_syncParams.SyncEnable    = ENABLE; 
	
	HAL_DMAEx_ConfigMuxSync(&hdma_spi3_tx, &dmamux_syncParams);

	MX_TIM12_Init(_ulFreq);
	
	if(HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)g_spiTxBuf, g_spiLen)!= HAL_OK)	
	{
		Error_Handler();
	}
#endif
		
}

void Wave_select(uint8_t n, uint16_t *_pBuf)
{
	switch(n)
	{
		case 1:
			MakeSinTable(_pBuf, 100, 0, 65535);
			break;
		case 2:
			MakeRectangularTable(_pBuf, 100, 0, 65535,0.3);
			break;
		case 3:
			MakeTriangularTable(_pBuf, 100, 0, 65535);
			break;
		case 4:
			MakeSquareTable(_pBuf, 100, 0, 65535);
			break;        
		case 5:
			MakeSharpTable(_pBuf, 100, 0, 65535);
      break;
		case 6:
			MakeSawtoothTable(_pBuf, 100, 0, 65535);
      break;
		case 7:
			MakeTrapezoidTable(_pBuf, 100, 0, 65535);
      break;
		case 8:
			MakeSparseTable(_pBuf, 100, 0, 65535);
      break;
		case 9:
			MakeDenseTable(_pBuf, 100, 0, 65535);
      break;
		case 10:
			MakeSparse_DenseTable(_pBuf, 100, 0, 65535);
      break;
		case 11:
			MakeExponentTable(_pBuf, 100, 0, 65535);
      break;
		case 12:
			MakePulseTable(_pBuf, 100, 0, 65535,0.3);
      break;
		case 13:
			MakeIntermittentTable(_pBuf, 100, 0, 65535);
      break;
		case 14:
			MakeContinuityTable(_pBuf, 100, 0, 65535);
      break;
		case 15:
			MakeSectorTable(_pBuf, 100, 0, 65535);
	
			break;
		default:
			break;
	}
}

void Dac8831_Set_Amp(float amp, uint16_t *_pBuf)
{
	int buf_data;
	for(int i = 0; i < 100; ++i)
	{
		buf_data = _pBuf[i];
		buf_data = (buf_data - 32767)*amp / 255 + 32767;
		_pBuf[i] = (uint16_t)buf_data;	
	}
	uint32_t _cmd;
	
	g_spiLen = 0;
	
	for(int i = 0; i < 100; i++)
	{
		/* ������Ҫ����PD1��PD0����ǰ��ѡ�����������ģʽ */
		_cmd = (0 << 16) | (_pBuf[i] << 0);
		
		g_spiTxBuf[g_spiLen++] = (uint8_t)(_cmd >> 8);
		g_spiTxBuf[g_spiLen++] = (uint8_t)(_cmd);
	}	

}

void DAC8831_SetCS(uint8_t _flag)
{
	if (_flag == 0)
	{
		g_spi_busy = 1;	/* ռ��SPI����  */	
		CS_0();
	}
	else
	{		
		CS_1();	
		g_spi_busy = 0;	/* �ͷ�SPI���� */
	}	
}


/*
*********************************************************************************************************
*	�� �� ��: MakeSquareTable
*	����˵��: ���������������
*	��    ��: _pBuf : Ŀ�껺����
*			  _usSamples : ÿ�����ڵ������� ���������32��������ż����
			  _usBottom��_usTop�����ֵ��0-65535��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MakeSquareTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop)
{
	for(uint16_t i = 0; i < _usSamples; ++i)
	{
		_pBuf[i] = i > (_usSamples / 2) ? _usTop : _usBottom;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: MakeSinTable
*	����˵��: ����������Ҳ�����
*	��    ��: _pBuf : Ŀ�껺����
*			  _usSamples : ÿ�����ڵ������� ���������32��������ż����
			  _usBottom��_usTop�����ֵ��0-65535��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MakeSinTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop)
{
	uint16_t i;
	uint16_t att;	/* ���� */
	att = _usTop - _usBottom;  	/* ���Ҳ����ȣ����ֵ����2 */
	for (i = 0; i < _usSamples; i++)
	{
		_pBuf[i] = (uint16_t)(((int32_t)(att * sin((i * 2 * 3.1415926f) / _usSamples)) + _usTop) / 2);
	}
}

/*
*********************************************************************************************************
*	�� �� ��: MakeRectangularTable
*	����˵��: ����������β�����
*	��    ��: _pBuf : Ŀ�껺����
*			  _usSamples : ÿ�����ڵ������� ���������32��������ż����
			  _usBottom��_usTop�����ֵ��0-65535��
			  _ratio��ռ�ձȣ�0.0 - 1.0��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MakeRectangularTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop, float _ratio)
{
	uint16_t down = (uint16_t)(_usSamples * _ratio);
	uint16_t i;
	for (i = 0; i < _usSamples; i++)
	{
		_pBuf[i] = i > down ? _usBottom : _usTop;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: MakeTriangularTable
*	����˵��: ����������ǲ�����
*	��    ��: _pBuf : Ŀ�껺����
*			  _usSamples : ÿ�����ڵ������� ���������32��������4����������
			  _usBottom��_usTop�����ֵ��0-65535��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MakeTriangularTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop)
{
	uint16_t change = _usSamples / 4;
	uint16_t att = _usTop - _usBottom;
	uint16_t mid = att / 2;
	uint16_t i;
	for (i = 0; i < _usSamples; i++)
	{
		if(i <= change){
			_pBuf[i] = (uint16_t)((_usTop - mid) * i / change + (_usTop - mid));
		}
		else if(i > change && i < 3 * change){
			_pBuf[i] = (uint16_t)(-(_usTop - mid) * i / change + 3 * (_usTop - mid));
		}
		else if(i >= 3 * change){
			_pBuf[i] = (uint16_t)((_usTop - mid) * i / change - 3 * (_usTop - mid));
		}
		_pBuf[change] = _usTop;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: MakeSharpTable
*	����˵��: ��������Ⲩ����
*	��    ��: _pBuf : Ŀ�껺����
*			  _usSamples : ÿ�����ڵ������� ���������32��������4����������
			  _usBottom��_usTop�����ֵ��0-65535��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MakeSharpTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop)
{
	uint16_t change = _usSamples / 4;
	uint16_t att = _usTop - _usBottom;
	uint16_t i;
	for (i = 0; i < _usSamples; i++)
	{
		if(i >= change && i < 3 * change){
			_pBuf[i] = (uint16_t)(-(att/log(1/3))*(-log(i/(3*change))));
		}
		else {
			_pBuf[i] = _usBottom;
		}
	}
}

/*
*********************************************************************************************************
*	�� �� ��: MakeSawtoothTable
*	����˵��: ���������ݲ�����
*	��    ��: _pBuf : Ŀ�껺����
*			  _usSamples : ÿ�����ڵ������� ���������32��������4����������
			  _usBottom��_usTop�����ֵ��0-65535��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MakeSawtoothTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop)
{
	uint16_t att = _usTop - _usBottom;
	uint16_t i;
	for (i = 0; i < _usSamples; i++)
	{
		_pBuf[i] = (uint16_t)(i*(att/_usSamples));
	}
}


/*
*********************************************************************************************************
*	�� �� ��: MakeExponentTable
*	����˵��: �������ָ��������
*	��    ��: _pBuf : Ŀ�껺����
*			  _usSamples : ÿ�����ڵ������� ���������32��������4����������
			  _usBottom��_usTop�����ֵ��0-65535��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MakeExponentTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop)
{
	uint16_t i;
//	uint16_t change = _usSamples / 2;       //warning ע�͵�
	uint16_t att = _usTop - _usBottom;
	for (i = 0; i < _usSamples; i++)
	{
//		if(i < change){
//			_pBuf[i] = (uint16_t)(att*(-(i/change)*(i/change)+2*(i/change)));
//		}
//		else{
//			_pBuf[i] = (uint16_t)(att*((i/change)*(i/change)-4*(i/change)+4));
//		}
		_pBuf[i] = (uint16_t)(att*(exp(-i/5)));
	}
}

/*
*********************************************************************************************************
*	�� �� ��: MakeTrapezoidTable
*	����˵��: ����������β�����
*	��    ��: _pBuf : Ŀ�껺����
*			  _usSamples : ÿ�����ڵ������� ���������32��������4����������
			  _usBottom��_usTop�����ֵ��0-65535��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MakeTrapezoidTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop)
{
	uint16_t i;
	uint16_t change = _usSamples / 4;
	uint16_t att = _usTop - _usBottom;
	for (i = 0; i < _usSamples; i++)
	{
		if(i < change){
			_pBuf[i] = (uint16_t)((att/change)*i);
		}
		else if(i >= change && i < 3*change){
			_pBuf[i] = _usTop;
		}
		else{
			_pBuf[i] = (uint16_t)(-(att/change)*i+(int)att*4);
		}
	}
	_pBuf[3*change] =  _usTop;
}

/*
*********************************************************************************************************
*	�� �� ��: MakeSectorTable
*	����˵��: ����������β�����
*	��    ��: _pBuf : Ŀ�껺����
*			  _usSamples : ÿ�����ڵ������� ���������32��������4����������
			  _usBottom��_usTop�����ֵ��0-65535��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MakeSectorTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop)
{
	uint16_t i;
	uint16_t att;	/* ���� */
	uint16_t change = _usSamples / 4;
	att = _usTop - _usBottom;  	/* ���Ҳ����ȣ����ֵ����2 */
	for (i = 0; i < _usSamples; i++)
	{
		if(i < change)
			_pBuf[i] = (uint16_t)(((int32_t)(att * sin((i * 2 * 3.1415926f) / _usSamples)) + _usTop) / 2);
		else{
			_pBuf[i] = (uint16_t)((int32_t)(att * i / (change - _usSamples)) + (int32_t)(att * _usSamples/(_usSamples - change)));
		}
	}
}

/*
*********************************************************************************************************
*	�� �� ��: MakePulseTable
*	����˵��: ����������岨����
*	��    ��: _pBuf : Ŀ�껺����
*			  _usSamples : ÿ�����ڵ������� ���������32��������4����������
			  _usBottom��_usTop�����ֵ��0-65535��
				_ratio��ռ�ձȣ�0.0 - 1.0��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MakePulseTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop, float _ratio)
{
	uint16_t down = (uint16_t)(_usSamples * _ratio);
	uint16_t i;
	for (i = 0; i < _usSamples; i++)
	{
		_pBuf[i] = i > down ? _usBottom : _usTop;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: MakeSparseTable
*	����˵��: ��������貨����
*	��    ��: _pBuf : Ŀ�껺����
*			  _usSamples : ÿ�����ڵ������� ���������32��������4����������
			  _usBottom��_usTop�����ֵ��0-65535��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MakeSparseTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop)
{
	for(uint16_t i = 0; i < _usSamples; i++)
	{
		if(i == 0)
			_pBuf[i] = _usTop;
		else if(i == _usSamples/2)
			_pBuf[i] = _usBottom;
		else
			_pBuf[i] = _usTop/2;
	}

}

/*
*********************************************************************************************************
*	�� �� ��: MakeDenseTable
*	����˵��: ��������ܲ�����
*	��    ��: _pBuf : Ŀ�껺����
*			  _usSamples : ÿ�����ڵ������� ���������32��������4����������
			  _usBottom��_usTop�����ֵ��0-65535��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MakeDenseTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop)
{
	for(int i=0; i< _usSamples; ++i)
	{
		_pBuf[i] = _usTop/2;
	}
	for(int i=0; i< _usSamples-1; i=i+10)
	{
		_pBuf[i] = _usTop;
		_pBuf[i+1] = _usBottom;
	}
}

void MakeSparse_DenseTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop)
{
	for(int i=0; i< _usSamples; ++i)
	{
		_pBuf[i] = _usTop/2;
	}
	_pBuf[_usSamples/4] = _usTop;
	_pBuf[_usSamples/4 + 1] = _usBottom;
	for(int i=50; i< _usSamples-1; i=i+10)
	{
		_pBuf[i] = _usTop;
		_pBuf[i+1] = _usBottom;
	}
}
void MakeIntermittentTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop)
{
	MakeDenseTable(_pBuf,_usSamples,_usBottom, _usTop);
}
void MakeContinuityTable(uint16_t *_pBuf, uint16_t _usSamples, uint16_t _usBottom, uint16_t _usTop)
{
	MakeDenseTable(_pBuf,_usSamples,_usBottom, _usTop);
}

