/*
*******************************************************************************************************
*
* 文件名称 : delay.h
* 版    本 : V1.0
* 作    者 : mzy2364
* 说    明 : 延时函数驱动文件
* 
*******************************************************************************************************
*/

#ifndef _DELAY_H
#define _DELAY_H

#ifdef __cplusplus
extern "C"{
#endif

/* 头文件 -----------------------------------------------------------*/
#include "gd32f30x.h"


/* 宏定义 -----------------------------------------------------------*/





/* 全局变量 ----------------------------------------------------------*/


/* 函数声明 ----------------------------------------------------------*/


void delay_tim_init(void);
void delay_us(uint16_t us);
void delay_ms(uint16_t ms);

#ifdef __cplusplus
extern "C"{
#endif

#endif

/***************************************** (END OF FILE) *********************************************/
