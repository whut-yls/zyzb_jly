/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STEPMOTOR_H
#define __STEPMOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

//void SPI_Test(uint8_t *rec);
void MoveToPosit(uint8_t *rec);
//void MoveToPositV(uint8_t *rec);
//void AlwaysRun(uint8_t *rec);
void StepMotorStop(uint8_t *rec);
//void ReadAndWriteParameter(uint8_t *rec);
//void ReadAndWriteCurrentPos(uint8_t *rec);
//void ReadCurrent(uint8_t *rec);
//void ReadParameter(uint8_t *rec);
//void WriteParameter(uint8_t *rec);
//void ReadSpeed(uint8_t *rec);
//void ResetSystem(uint8_t *rec);
//void ReadFaultStatus(uint8_t *rec);
void MoveToAbs(uint8_t *rec);
//void MoveToAbsV(uint8_t *rec);
//void AutoDistance(uint8_t *rec);
//void MotorMode(uint8_t *rec);
//void WriteFaultStatus(uint8_t *rec);
//void Clear(uint8_t *rec);


/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H */

/************************ (C) COPYRIGHT  *****END OF FILE****/
