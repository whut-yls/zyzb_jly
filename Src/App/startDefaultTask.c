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





