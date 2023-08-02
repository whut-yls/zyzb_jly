#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "string.h"
#include "stdio.h"
/* default header*/
#include "startDefaultTask.h"
#include "com.h"

SemaphoreHandle_t  	defaultSemaphore = NULL;
SemaphoreHandle_t  	defaultSemaphore2 = NULL;
//EventGroupHandle_t	DefaultTask_Event_Wakeup;	
/**信号量相关
xSemaphoreTake(xSemaphore, portMAX_DELAY);
xSemaphoreGive(xSemaphore);
xSemaphoreGiveFromISR(xSemaphore,&xHigherPriorityTaskWoken);
BaseType_txHigherPriorityTaskWoken = pdFALSE/pdTRUE;
 portYIELD_FROM_ISR(xHigherPriorityTaskWoken);**/

bool DefaultTask_Init(void)
{
//	DefaultTask_Event_Wakeup = xEventGroupCreate();
//	
//	if(DefaultTask_Event_Wakeup == NULL)
//	{
//		printf("default task eventgroup create fail\r\n");
//		return false;
//	}
	return true;
}

bool startDefatltTask_Semaphore_Init(void)
{
	/*默认初始化为0*/
	defaultSemaphore = xSemaphoreCreateBinary();
	defaultSemaphore2 = xSemaphoreCreateBinary();
     if(defaultSemaphore == NULL || defaultSemaphore2 == NULL)
    {
        /*失败返回false*/
			return false;
    }
   
     /*根据需求先释放一次 初始化为1 */
//     xSemaphoreGive(xSemaphore);
	return true;
}


//void StartDefaultTask(void const * argument)
//{
//	uint8_t i;
//
//	while(1)
//	{
//
//		xSemaphoreTake(defaultSemaphore,portMAX_DELAY);
//
//		Console_Handle(RecCom1,RecCom1Num,1);
//
//		for(i=0;i<10;i++)	{RecCom1[i]=0;}	//clean data
//
//		READ_REG(huart1.Instance->DR);
//
//		SET_BIT(huart1.Instance->CR1, USART_CR1_RE);
//		HAL_UART_Receive_DMA(&huart1, RecCom1, COM1_REC_MAX); //若信号丢失 则会导致DMA未开启，不能放在这里
////		__HAL_UART_CLEAR_IDLEFLAG(&huart7);
////		__HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);	//不能关闭空闲中断，否则再发生ore事件后无法恢复。
//	}//while
//  /* USER CODE END 5 */
//}


