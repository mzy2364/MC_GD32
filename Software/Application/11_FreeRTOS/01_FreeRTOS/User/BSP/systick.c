/*!
    \file    systick.c
    \brief   the systick configuration file

    \version 2017-02-10, V1.0.0, firmware for GD32F30x
    \version 2018-10-10, V1.1.0, firmware for GD32F30x
    \version 2018-12-25, V2.0.0, firmware for GD32F30x
    \version 2020-09-30, V2.1.0, firmware for GD32F30x 
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f30x.h"
#include "systick.h"

#include "FreeRTOS.h"
#include "task.h"

volatile static uint32_t tick_cnt = 0;

/**
  * @brief 滴答定时器初始化
  * @param void
  * @retval	void
  * @note
  */
void systick_init(void)
{
    /* setup systick timer for 1000Hz interrupts */
    if (SysTick_Config(SystemCoreClock / 1000U)){
        /* capture error */
        while (1){
        }
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}

/**
  * @brief 滴答定时器反初始化
  * @param void
  * @retval	void
  * @note
  */
void systick_deinit(void)
{
	NVIC_DisableIRQ(SysTick_IRQn);
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

/**
  * @brief 获取当前时间
  * @param void
  * @retval	返回当前时间?
  * @note
  */
uint32_t systick_get_current_tick(void)
{
	return tick_cnt;
}


/**
  * @brief 滴答定时器低精度阻塞延时函数
  * @param ms-需要延时的ms数
  * @retval	None?
  * @note 精度1ms
  */
void systick_delay(uint32_t ms)
{
    vTaskDelay(ms);
}

/**
  * @brief 返回系统时钟跟输入参数的差值
  * @param tick:需要比较的时基数
  * @retval 返回差值
  * @note
  */
uint32_t systick_time_diff(uint32_t tick)
{
	uint32_t delta = 0;
	uint32_t current_tick = systick_get_current_tick();
	delta = (current_tick >= tick)?(current_tick - tick):(0xFFFFFFFF - tick + current_tick);
	return delta;

}


//void SysTick_Handler(void)
//{
//    tick_cnt++;
//}
