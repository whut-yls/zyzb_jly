/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32h7xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "myUart.h"
#include "myTimer.h"
#include "console.h"
#include "myadc.h"
#include "lan8742.h"
#include "user_tcp_client.h"
#include "common.h"
#include "dac8831.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ETH_HandleTypeDef heth;
extern TIM_HandleTypeDef htim17;	
extern TIM_HandleTypeDef htim12;


extern DMA_HandleTypeDef hdma_spi3_tx;

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc3;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart7_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */
	Get_ADC1_Hex();
  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */
	Get_ADC3_Hex();
  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}
/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

//加入spi3的中断处理函数看dma发送有没有效  by yls 2023/6/18
void SPI3_IRQHandler(void)
{
  /* USER CODE BEGIN SPI3_IRQn 0 */

  /* USER CODE END SPI3_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi3);
  /* USER CODE BEGIN SPI3_IRQn 1 */

  /* USER CODE END SPI3_IRQn 1 */
}
/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}
void Uart6_IDLE_HandleRec(void)
{
		uint16_t temp;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);
		temp = COM6_REC_MAX -  __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
	
		if(temp==0)
		{
			return;
		}
			
		HAL_UART_DMAStop(&huart6);
		__HAL_DMA_SET_COUNTER(&hdma_usart6_rx,COM6_REC_MAX);
		RecCom6Num  = temp;
		RecCom6[RecCom6Num] = 0;	
		xSemaphoreGiveFromISR(lcdSemaphore,&xHigherPriorityTaskWoken);
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
		if(RESET != __HAL_UART_GET_FLAG(&huart6, UART_FLAG_ORE)||
			RESET != __HAL_UART_GET_FLAG(&huart6, UART_FLAG_FE)||
		RESET != __HAL_UART_GET_FLAG(&huart6, UART_FLAG_PE)||
		RESET != __HAL_UART_GET_FLAG(&huart6, UART_FLAG_NE))
  {
		/*	if(gUartLcdTc==false)
			{
				gUartLcdInit=true;
				HAL_UART_DeInit(&huart6);
				MX_USART3_UART_Init();
				gUartLcdInit=false;
				HAL_UART_Receive_DMA(&huart6, RecCom3,COM3_REC_MAX);
				__HAL_UART_CLEAR_IDLEFLAG(&huart6);
				__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
				
				return;	
			}*/
	}
  /* USER CODE END UART7_IRQn 0 */
   HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN UART7_IRQn 1 */
	if(RESET != __HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))
  {
		Uart6_IDLE_HandleRec();
	}
  /* USER CODE END USART6_IRQn 0 */

  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}


/*调试串口*/
void Uart7_IDLE_HandleRec(void)
{
		uint16_t temp;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		__HAL_UART_CLEAR_IDLEFLAG(&huart7);
		temp = COM7_REC_MAX -  __HAL_DMA_GET_COUNTER(&hdma_uart7_rx);
	
		if(temp==0)
		{
			return;
		}
			
		HAL_UART_DMAStop(&huart7);
		__HAL_DMA_SET_COUNTER(&hdma_uart7_rx,COM7_REC_MAX);
		RecCom7Num  = temp;
		RecCom7[RecCom7Num] = 0;	
		xSemaphoreGiveFromISR(ConsoleReceive_Semaphore,&xHigherPriorityTaskWoken);
//		xHigherPriorityTaskWoken = pdTRUE;
//		portYIELD_FROM_ISR( xHigherPriorityTaskWoken);
//		HAL_UART_Receive_DMA(CONSOLE_PORT, RecCom5, COM5_REC_MAX);
}

/**
  * @brief This function handles UART7 global interrupt.
  */
void UART7_IRQHandler(void)
{
   /* USER CODE BEGIN UART7_IRQn 0 */
		if(RESET != __HAL_UART_GET_FLAG(&huart7, UART_FLAG_ORE)||
			RESET != __HAL_UART_GET_FLAG(&huart7, UART_FLAG_FE)||
		RESET != __HAL_UART_GET_FLAG(&huart7, UART_FLAG_PE)||
		RESET != __HAL_UART_GET_FLAG(&huart7, UART_FLAG_NE))
  {
			if(gUartLcdTc==false)
			{
				gUartLcdInit=true;
				HAL_UART_DeInit(&huart7);
				MX_UART7_Init();
				gUartLcdInit=false;
				HAL_UART_Receive_DMA(&huart7, RecCom7,COM7_REC_MAX);
				__HAL_UART_CLEAR_IDLEFLAG(&huart7);
				__HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);
				
				return;	//VIP 一定要返回 特别是fe pe ne 错误
			}
	}
  /* USER CODE END UART7_IRQn 0 */
  HAL_UART_IRQHandler(&huart7);
  /* USER CODE BEGIN UART7_IRQn 1 */
	if(RESET != __HAL_UART_GET_FLAG(&huart7, UART_FLAG_IDLE))
  {
		Uart7_IDLE_HandleRec();
	}
  /* USER CODE END UART7_IRQn 1 */
}


/*调试串口*/
void Uart3_IDLE_HandleRec(void)
{
		uint16_t temp;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
		temp = COM3_REC_MAX -  __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
	
		if(temp==0)
		{
			return;
		}
			
		HAL_UART_DMAStop(&huart3);
		__HAL_DMA_SET_COUNTER(&hdma_usart3_rx,COM3_REC_MAX);
		RecCom3Num  = temp;
		RecCom3[RecCom3Num] = 0;	
		xSemaphoreGiveFromISR(lcdSemaphore,&xHigherPriorityTaskWoken);
//		xHigherPriorityTaskWoken = pdTRUE;
//		portYIELD_FROM_ISR( xHigherPriorityTaskWoken);
//		HAL_UART_Receive_DMA(CONSOLE_PORT, RecCom5, COM5_REC_MAX);
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN UART7_IRQn 0 */
		if(RESET != __HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE)||
			RESET != __HAL_UART_GET_FLAG(&huart3, UART_FLAG_FE)||
		RESET != __HAL_UART_GET_FLAG(&huart3, UART_FLAG_PE)||
		RESET != __HAL_UART_GET_FLAG(&huart3, UART_FLAG_NE))
  {
			if(gUartPcTc==false)
			{
				gUartPcInit=true;
				HAL_UART_DeInit(&huart3);
				MX_USART3_UART_Init();
				gUartPcInit=false;
				HAL_UART_Receive_DMA(&huart3, RecCom3,COM3_REC_MAX);
				__HAL_UART_CLEAR_IDLEFLAG(&huart3);
				__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
				
				return;	
			}
	}
  /* USER CODE END UART7_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN UART7_IRQn 1 */
	if(RESET != __HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
  {
		Uart3_IDLE_HandleRec();
	}
}


/**
  * @brief This function handles Ethernet global interrupt.
  */
void ETH_IRQHandler(void)
{

  /* USER CODE BEGIN ETH_IRQn 0 */

  /* USER CODE END ETH_IRQn 0 */
  HAL_ETH_IRQHandler(&heth);
  /* USER CODE BEGIN ETH_IRQn 1 */

  /* USER CODE END ETH_IRQn 1 */
}


/**
  * @brief This function handles TIM17 global interrupt.
  */
void TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM17_IRQn 0 */

  /* USER CODE END TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM17_IRQn 1 */

  /* USER CODE END TIM17_IRQn 1 */
}
/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}
/* USER CODE BEGIN 1 */

void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
  HAL_TIM_IRQHandler(&htim12);
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


