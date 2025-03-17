/*
*******************************************************************************************************
*
* File Name : encoder.h
* Version   : V1.0
* Author    : mzy2364
* brief     : encoder header file
* 
*******************************************************************************************************
*/
#ifndef _ENCODER_H_
#define _ENCODER_H_

#ifdef __cplusplus
extern "C"{
#endif

/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "gd32f30x.h"


/* DEFINES ------------------------------------------------------------------------------------------*/
#define ENCODERA_GPIO_PORT                  GPIOB
#define ENCODERA_GPIO_PIN                   GPIO_PIN_6
#define ENCODERA_GPIO_CLK                   RCU_GPIOB

#define ENCODERB_GPIO_PORT                  GPIOB
#define ENCODERB_GPIO_PIN                   GPIO_PIN_7
#define ENCODERB_GPIO_CLK                   RCU_GPIOB

#define KEY_GPIO_PORT                       GPIOC
#define KEY_GPIO_PIN                        GPIO_PIN_0
#define KEY_GPIO_CLK                        RCU_GPIOC

/* 按键个数 */
#define KEY_COUNT 1
/* 按键滤波时间 单位10ms */
#define KEY_FILTER_TIME   5
/* 按键长按时间 单位10ms */
#define KEY_LONG_TIME     100
/* 定义按键FIFO缓冲区大小 */
#define KEY_FIFO_SIZE	2

typedef enum{
    ENCODER_NONE = 0,
    ENCODER_INC,
    ENCODER_DEC,
}encoder_code_t;

/* 按键是否按下 */
typedef enum
{
	KEY_ISDOWN = 0,
	KEY_NOTDOWN,
}KEY_DOWN;


/* 按键类型 */
typedef enum
{
	KEY_NONE = 0,
	
	KEY_0_PRESS,
	KEY_0_RELEASE,
	KEY_0_LONG_PRESS,
	
}KEY_ENUM;


/* 按键结构体 */
typedef struct
{
	uint8_t (*IsKeyDownFunc)(void);		//按键是否按下的函数指针
	
	uint8_t Count;	//滤波器计数器
	uint16_t LongCount;	//长按计数器
	uint16_t LongTime;	//按键按下的时间
	uint8_t State;		//按键的当前状态
	uint8_t RepeatSpeed;	//连续按键周期
	uint8_t RepeatCount;		//连续按键计数器	
	uint8_t LongPressFlag;
	
	
}KEY_T;


/* fifo结构体 */
typedef struct
{
	uint8_t Buf[KEY_FIFO_SIZE];        //按键缓冲区
	uint8_t Read;                      //读指针
	uint8_t Write;                     //写指针
	
}KEY_FIFO;

/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/
void encoder_init(void);
encoder_code_t encoder_get(void);

void key_clear(void);
void key_scan(void);
uint8_t key_get(void);

#ifdef __cplusplus
extern "C"{
#endif

#endif /* _ENCODER_H_ */

/***************************************** (END OF FILE) *********************************************/
