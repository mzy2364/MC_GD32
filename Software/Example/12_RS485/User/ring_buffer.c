/*
*******************************************************************************************************
*
* 文件名称 : usart_fifo.c
* 版    本 : V1.0
* 作    者 : mzy2364
* 说    明 : 环形缓冲区 https://github.com/jiejieTop/stm32_kfifo.git
* 
*******************************************************************************************************
*/


/* 头文件 -----------------------------------------------------------*/
#include "ring_buffer.h"
#include <stdio.h>
#include <string.h>

/* 宏定义 -----------------------------------------------------------*/

#define min(a,b)  ((a)>(b) ? (b) : (a)) 



/* 变量 -------------------------------------------------------------*/


/* 函数定义 ----------------------------------------------------------*/


#ifdef __GNUC__
#define __attribute_const__     __attribute__((__const__))
#endif

static int32_t fls(int32_t x)
{
  int r = 32;

  if (!x)
          return 0;
  if (!(x & 0xffff0000u)) {
          x <<= 16;
          r -= 16;
  }
  if (!(x & 0xff000000u)) {
          x <<= 8;
          r -= 8;
  }
  if (!(x & 0xf0000000u)) {
          x <<= 4;
          r -= 4;
  }
  if (!(x & 0xc0000000u)) {
          x <<= 2;
          r -= 2;
  }
  if (!(x & 0x80000000u)) {
          x <<= 1;
          r -= 1;
  }
  return r;
}
/**
  * @brief 用于处理数据，使数据长度必须为 2^n
  * @param x:用于转换的数据
  * @retval	转换后的长度
  * @note 
  */
static unsigned long roundup_pow_of_two(unsigned long x)
{
	return (1 << (fls(x-1)-1));				//向下对齐
  //return (1UL << fls(x - 1));			//向上对齐，用动态内存可用使用
}

/**
  * @brief 创建一个环形缓冲区
  * @param rb:环形缓冲区句柄
  *		   buffer:环形缓冲区的数据区域
  *        size:环形缓冲区的大小，缓冲区大小要为2^n
  * @retval	ERR_OK:创建成功 other:创建失败
  * @note 
  */
uint8_t ring_buffer_create(RingBuff_t *rb,uint8_t *buffer,uint32_t size)
{
	if((rb == NULL)||(buffer == NULL)||(size == 0))
	{
		return ERR_NULL;
	}
	/* 缓冲区大小必须为2^n字节,系统会强制转换,
		 否则可能会导致指针访问非法地址。
		 空间大小越大,强转时丢失内存越多 */
	if(size&(size - 1))
	{
		size = roundup_pow_of_two(size);
	}
	
	rb->buffer = buffer;
	rb->size = size;
	rb->in = rb->out = 0;
	
#if USE_MUTEX	
  /* 创建信号量不成功 */
  if(!create_mutex(rb->mutex))
  {
    PRINT_ERR("create mutex fail!");
    ASSERT(ASSERT_ERR);
    return ERR_NOK;
  }
#endif
	
	return ERR_OK;

}


/**
  * @brief 创建一个环形缓冲区
  * @param rb:环形缓冲区句柄
  * @retval	ERR_OK:成功 other:失败
  * @note 
  */
uint8_t ring_buffer_delete(RingBuff_t *rb)
{
	if(rb == NULL)
	{
		return ERR_NULL;
	}
	
	rb->buffer = NULL;
	rb->size = 0;
	rb->in = rb->out = 0;
#if USE_MUTEX	
  if(!deleta_mutex(rb->mutex))
  {
    PRINT_DEBUG("deleta mutex is fail!");
    return ERR_NOK;
  }
#endif
	return ERR_OK;
}

/**
  * @brief 向环形缓冲区写数据
  * @param rb:环形缓冲区句柄
  *		   wbuff:写入的数据起始地址
  *        len:写入数据的长度(字节)
  * @retval	实际写入数据的长度(字节)
  * @note 
  */
uint32_t ring_buffer_write(RingBuff_t *rb,uint8_t *wbuff,uint32_t len)
{
	uint32_t l;
#if USE_MUTEX
  /* 请求互斥量，成功才能进行ringbuff的访问 */
  if(!request_mutex(rb->mutex))
  {
    PRINT_DEBUG("request mutex fail!");
    return 0;
  }
  else  /* 获取互斥量成功 */
  {
#endif
	
	len = min(len, rb->size - rb->in + rb->out);
	
    /* 第一部分的拷贝:从环形缓冲区写入数据直至缓冲区最后一个地址 */
    l = min(len, rb->size - (rb->in & (rb->size - 1)));
    memcpy(rb->buffer + (rb->in & (rb->size - 1)), wbuff, l);
	
    /* 如果溢出则在缓冲区头写入剩余的部分
       如果没溢出这句代码相当于无效 */
    memcpy(rb->buffer, wbuff + l, len - l);
	
	rb->in += len;
	
#if USE_MUTEX
  }
  /* 释放互斥量 */
  release_mutex(rb->mutex);
#endif
  return len;
}


/**
  * @brief 读取环形缓冲区数据
  * @param rb:环形缓冲区句柄
  *		   rbuff:保存读取到的数据的缓冲区
  *        len:想要读取数据的长度(字节)
  * @retval	实际读取数据的长度(字节)
  * @note 
  */
uint32_t ring_buffer_read(RingBuff_t *rb, uint8_t *rbuff,uint32_t len)
{
  uint32_t l;
#if USE_MUTEX
  /* 请求互斥量，成功才能进行ringbuff的访问 */
  if(!request_mutex(rb->mutex))
  {
    PRINT_DEBUG("request mutex fail!");
    return 0;
  }
  else
  {
#endif
    len = min(len, rb->in - rb->out);

    /* 第一部分的拷贝:从环形缓冲区读取数据直至缓冲区最后一个 */
    l = min(len, rb->size - (rb->out & (rb->size - 1)));
    memcpy(rbuff, rb->buffer + (rb->out & (rb->size - 1)), l);

    /* 如果溢出则在缓冲区头读取剩余的部分
       如果没溢出这句代码相当于无效 */
    memcpy(rbuff + l, rb->buffer, len - l);

    rb->out += len;
    
#if USE_MUTEX
  }
  /* 释放互斥量 */
  release_mutex(rb->mutex);
#endif
  return len;
}

/**
  * @brief 获取可读数据长度
  * @param rb:环形缓冲区句柄
  * @retval	可读数据长度
  * @note 
  */
uint32_t ring_buffer_can_read_length(RingBuff_t *rb)
{
	if(NULL == rb)
	{
		return 0;
	}
	if(rb->in == rb->out)
		return 0;
	
	if(rb->in > rb->out)
		return (rb->in - rb->out);
	
	return (rb->size - (rb->out - rb->in));
}

/**
  * @brief 获取可写数据长度
  * @param rb:环形缓冲区句柄
  * @retval	可写数据长度
  * @note 
  */
uint32_t ring_buffer_can_write_length(RingBuff_t *rb)
{
	if(NULL == rb)
	{
		return 0;
	}

	return (rb->size - ring_buffer_can_read_length(rb));
}

/***************************** (END OF FILE) *********************************/
