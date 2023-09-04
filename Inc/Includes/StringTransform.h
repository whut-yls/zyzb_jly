/**
  ******************************************************************************
  * @file           : StringTransform.h
  * @brief          : Header for StringTransform.c file.
  *                   This file contains string handle defines of the application.
  ******************************************************************************
  
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STRING_TRANSFORM_H__
#define __STRING_TRANSFORM_H__

#include "main.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
 extern "C" {
#endif


/*string transform upper */
void strupper(char* src,int l);

/*string transform lower */
void strlower(char* src,int l);

/*transform string to hex*/
int AscToHex(char *a, unsigned char *t, int l);
int IsString(char src);
int CTZEToTime(char *r,int l, unsigned char *year,unsigned char *month,unsigned char *date,unsigned char *hour,unsigned char *minute,unsigned char *Second);
int QLTSToTime(char *r,int l, unsigned char *year,unsigned char *month,unsigned char *date,unsigned char *hour,unsigned char *minute,unsigned char *Second);
uint16_t Gets_Average(uint16_t *pt,int l);
float Getf_Average(float *pt,int l);

#ifdef __cplusplus
}
#endif

#endif /* __STRING_TRANSFORM_H__ */

/************************ (C) COPYRIGHT  hj*****END OF FILE****/

