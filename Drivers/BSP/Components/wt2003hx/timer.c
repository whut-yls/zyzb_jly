/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "timer.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim12;
TIM_HandleTypeDef  htim1 ={0};
TIM_MasterConfigTypeDef sMasterConfig = {0};
TIM_OC_InitTypeDef sConfig = {0};


void MX_TIM12_Init(uint32_t _ulFreq)
{
	uint16_t usPeriod;
	uint16_t usPrescaler;
	uint32_t uiTIMxCLK;


  TIM_MasterConfigTypeDef sMasterConfig = {0};

#if 0
  htim12.Instance = TIM12;
  htim12.Init.Prescaler =pre-1;              						//定时器6最大只能配到 240M/120/5=400kHz    一组数据100个 频率=400kHz/100=4khz   周期：1/4k s = 0.25ms 在不发数据的情况下
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;					//定时器6最大只能配到 240M/240/5=200kHz    一组数据100个 频率=200kHz/100=2khz   周期：1/2k s = 0.5ms  在发数据的情况下
  htim12.Init.Period =period-1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
#endif
	
	uiTIMxCLK = SystemCoreClock / 2;
	
	if (_ulFreq < 1000)
	{
		usPrescaler = 240 - 1;					/* 分频比 = 100 */
		usPeriod =  (uiTIMxCLK / 24000) / _ulFreq  - 1;		/* 自动重装的值 */
	}
	else	/* 大于1k的频率，无需分频 */
	{
		usPrescaler = 24-1;					/* 分频比 = 1 */
		usPeriod = (uiTIMxCLK/2400) / _ulFreq - 1;	/* 自动重装的值 */
	}	
	
	
#if 1	
  htim12.Instance = TIM12;
  htim12.Init.Prescaler =usPrescaler;              						//定时器6最大只能配到 240M/120/5=400kHz    一组数据100个 频率=400kHz/100=4khz   周期：1/4k s = 0.25ms 在不发数据的情况下
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;					//定时器6最大只能配到 240M/240/5=200kHz    一组数据100个 频率=200kHz/100=2khz   周期：1/2k s = 0.5ms  在发数据的情况下
  htim12.Init.Period =usPeriod;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	htim12.Init.ClockDivision     = 0;	
	htim12.Init.RepetitionCounter = 0;
	
	if(HAL_TIM_Base_DeInit(&htim12) != HAL_OK)
	{
		Error_Handler();		
	}

  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }	
	

	sConfig.OCMode     = TIM_OCMODE_PWM1;
	sConfig.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfig.Pulse = usPeriod / 2;     /* 占空比50% */
	if(HAL_TIM_OC_ConfigChannel(&htim12, &sConfig, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* 启动OC1 */
	if(HAL_TIM_OC_Start(&htim12, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	
	/* TIM12的TRGO用于触发DMAMUX的请求发生器 */
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;    //TIM_TRGO_UPDATE    TIM_TRGO_OC1REF
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

	HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig);
	
   /* 使能定时器 */
  HAL_TIM_Base_Start(&htim12);	
#endif
}


void MX_TIM1_Init(uint32_t _ulFreq)    //240M
{

	/* 使能时钟 */  
	__HAL_RCC_TIM1_CLK_ENABLE();
	

  HAL_TIM_Base_DeInit(&htim1);
    
  htim1.Instance = TIM1;              
	if(gGlobalData.useWorkArg[0].freqTreat <= 1000){
		htim1.Init.Period            = 1199;    //10KHZ  2399
		htim1.Init.Prescaler         = 20-1;
	}
	else{
		htim1.Init.Period            = 1199;    //100KHZ  239
		htim1.Init.Prescaler         = 10-1;
	}
	htim1.Init.ClockDivision     = 0;
	htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim1.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&htim1);	

	sConfig.OCMode     = TIM_OCMODE_PWM1;
	sConfig.OCPolarity = TIM_OCPOLARITY_LOW;

	/* 占空比50% */
	sConfig.Pulse = 1000;  
	if(HAL_TIM_OC_ConfigChannel(&htim1, &sConfig, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler( );
	}

	/* 启动OC1 */
	if(HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
	{
	Error_Handler( );
	}	
//   /* 使能定时器 */
//  HAL_TIM_Base_Start(&htim1);	
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM12)
  {

    __HAL_RCC_TIM12_CLK_ENABLE();
	
    HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);

  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM12)
  {

    __HAL_RCC_TIM12_CLK_DISABLE();

    HAL_NVIC_DisableIRQ(TIM8_BRK_TIM12_IRQn);

  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
