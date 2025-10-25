/*
*******************************************************************************************************
*
* 文件名称 : usart_fifo.h
* 版    本 : V1.0
* 作    者 : mzy2364
* 说    明 : 串口驱动文件
* 
*******************************************************************************************************
*/

#ifndef _USART_FIFO_H
#define _USART_FIFO_H

/* 头文件 -----------------------------------------------------------*/
#include <stdint.h>


/* 宏定义 -----------------------------------------------------------*/

#define  USE_MUTEX   0

/* 环形缓冲区定义 */
typedef struct ringbuff 
{
	uint8_t *buffer;  	/* 数据区域 */
	uint32_t size;      /* 环形缓冲区大小 */
	uint32_t in;        /* 数据入队指针 (in % size) */
	uint32_t out;       /* 数据出队指针 (out % size) */
#if USE_MUTEX
	MUTEX_T *mutex;       /* 支持rtos的互斥 */
#endif
}RingBuff_t ;

/* 错误句柄定义 */
enum {
	ERR_OK = 0,
	ERR_NULL ,
	ERR_NOK
};

/* 变量 -------------------------------------------------------------*/


/* 函数声明 ---------------------------------------------------------*/

uint8_t ring_buffer_create(RingBuff_t *rb,uint8_t *buffer,uint32_t size);
uint8_t ring_buffer_delete(RingBuff_t *rb);
uint32_t ring_buffer_write(RingBuff_t *rb,uint8_t *wbuff,uint32_t len);
uint32_t ring_buffer_read(RingBuff_t *rb, uint8_t *rbuff,uint32_t len);
uint32_t ring_buffer_can_read_length(RingBuff_t *rb);
uint32_t ring_buffer_can_write_length(RingBuff_t *rb);





#endif

/***************************** (END OF FILE) *********************************/
