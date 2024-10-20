/*
*******************************************************************************************************
*
* 文件名称 : key_fifo.h
* 版    本 : V1.0
* 作    者 : mzy2364
* 说    明 : 按键驱动头文件
* 
*******************************************************************************************************
*/

#ifndef _KEY_FIFO_H
#define _KEY_FIFO_H



/* 头文件 -----------------------------------------------------------*/
#include "gd32f30x.h"



/* 宏定义 -----------------------------------------------------------*/

/* 按键端口定义 */
#define KEY0_GPIO_PORT       GPIOC
#define KEY0_GPIO_PIN        GPIO_PIN_0
#define KEY0_GPIO_CLK        RCU_GPIOC

/* 按键个数 */
#define KEY_COUNT 1
/* 按键滤波时间 单位10ms */
#define KEY_FILTER_TIME   5
/* 按键长按时间 单位10ms */
#define KEY_LONG_TIME     100
/* 定义按键FIFO缓冲区大小 */
#define KEY_FIFO_SIZE	4



/* typedef -----------------------------------------------------------*/

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
	
	
}KEY_T;


/* fifo结构体 */
typedef struct
{
	uint8_t Buf[KEY_FIFO_SIZE];        //按键缓冲区
	uint8_t Read;                      //读指针
	uint8_t Write;                     //写指针
	
}KEY_FIFO;




/* 函数声明 ----------------------------------------------------------*/


void key_fifo_init(void);
void key_clear(void);
uint8_t key_is_init(void);
void key_scan(void);
uint8_t key_get(void);


void key_put(uint8_t keyCode);
static void key_gpio_config(void);
static void key_var_init(void);
static void key_detect(uint8_t i);
 
#endif

/***************************** (END OF FILE) *********************************/
