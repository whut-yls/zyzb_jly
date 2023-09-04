/**
  ******************************************************************************
  * @file    StepServoTask.h
  * @author  hj
  * @brief   Header file of StepServoTask.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 hj</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:

  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_TASK_H
#define __MOTOR_TASK_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "parameter.h"
#include "TMC4361A.h"
#include "ctype.h" 



/* Private defines -----------------------------------------------------------*/

#define MOTOR_ERROR_UPPERLIMIT			0X00000001		/*upper limit not action*/
#define MOTOR_ERROR_DOWNLIMIT				0X00000002		/*upper limit not action*/

#define MOTOR_FULL_POSTION				-51200*50	//50R
#define	MOTOR_ZERO_NUM						10	//零点循环次数
#define	MOTOR_ZERO_TIME						100	//单次循环毫秒数
#define	MOTOR_ZERO_SUC            3   //编码器连续相同的次数
#define MOTOR_STEP_ZERO_OK        0x00f8  //bit3~bint11 =1
#define MOTOR_DC_ZERO_OK          0x0007  //bit3~bint11 =1

#define	MOTOR_POS_NUM						1000	//零点循环次数
#define	MOTOR_POS_TIME					10	//零点循环次数

#define MOTRO_DC1_ZERO_SET          				  	(0x1U << 0)
#define MOTRO_DC2_ZERO_SET           				  	(0x1U << 1)
#define MOTRO_DC3_ZERO_SET           				  	(0x1U << 2)
#define MOTRO_DC4_ZERO_SET           				  	(0x1U << 3)
#define MOTRO_DC5_ZERO_SET           				  	(0x1U << 4)
#define MOTRO_Step6_ZERO_SET          				  (0x1U << 5)
#define MOTRO_Step7_ZERO_SET          				  (0x1U << 6)
#define MOTRO_Step8_ZERO_SET          				  (0x1U << 7)
#define MOTRO_Step9_ZERO_SET          				  (0x1U << 8)
#define MOTRO_Step10_ZERO_SET          				  (0x1U << 9)
#define MOTRO_Step11_ZERO_SET          				  (0x1U << 10)
#define MOTRO_Step12_ZERO_SET          				  (0x1U << 11)
#define MOTRO_Step13_ZERO_SET       				  	(0x1U << 12)
#define MOTRO_Step18_ZERO_SET       				  	(0x1U << 17)


extern char gStatusAck[JSON_ACK_MAX_LEN];

/* USER CODE BEGIN Private defines */

void ClosedChangemode(uint8_t mode);
void ISRClosedWritePosToDriver(void);
void Set_Step_Pos(int pos);

void StartStepServoTask(void const * argument);
int Read_FullStep_Pos(char num);
int Read_MicroStep_Pos(char num);
int Read_DcStep_Pos(char num);
int Read_encPos(char num);

int Write_FullStep_Pos(char num);
int Write_DcStep_Pos(char num);

int Start_Motor_Run(char motor,char mode,int dir,int unity,int value);

int Stop_Motor_Run(char motor);
void Clean_Errno_Status(uint8_t motor);

bool general_statusAck(bool mode);
void dcMotor_Reset(void);

void Set_zeroPos(void);
void Run_toZeroPos(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __STEP_SERVO_TASK_H */

/************************ (C) COPYRIGHT HJ *****END OF FILE****/

