/*
*******************************************************************************************************
*
* File Name : main.c
* Version   : V1.0
* Author    : mzy2364
* brief     : main function file
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "gd32f30x.h"
#include "gd32f30x_libopt.h"
#include "motor_hardware.h"
#include "motor_config.h"
#include "systick.h"
#include <string.h>

/* DEFINES ------------------------------------------------------------------------------------------*/

/* VARIABLES ----------------------------------------------------------------------------------------*/
uint32_t duty_u_ref = 0,duty_v_ref = 0,duty_w_ref = 0;
uint32_t duty_u = 0,duty_v = 0,duty_w = 0;
int32_t duty_u_offset = 0,duty_v_offset = 0,duty_w_offset = 0;
float temp1 = 0,temp2 = 0,temp3 = 0;
uint8_t uart_data[16] = {0};
/* FUNCTION -----------------------------------------------------------------------------------------*/


/**
  * @brief main function
  * @param None
  * @retval None
  */
int main(void)
{
    SCB->VTOR = FLASH_BASE | 0x4000;
    __enable_irq();
    
    systick_init();
    led_init();
    led_off(LED1);
    led_off(LED2);
    led_off(LED3);
    interrupt_init();
    dac_init();
    adc_init();
    pwm_init();
    usart_init();

    gpio_bit_write(GPIO3_GPIO_PORT,GPIO3_PIN,RESET);

    /* 初始化占空比 */
    duty_u_ref = (float)MOTOR_PWM_PERIOD / 3;          /* 33% */
    duty_v_ref = (float)MOTOR_PWM_PERIOD / 2;          /* 50% */
    duty_w_ref = ((float)MOTOR_PWM_PERIOD * 2) / 3;    /* 66% */
    
	while(1)
	{

	}
}


void TIMER0_UP_IRQHandler(void)
{
    timer_interrupt_flag_clear(TIMER0,TIMER_INT_FLAG_UP);
    if(timer_counter_read(TIMER0) > MOTOR_PWM_PERIOD / 2)
    {
        /* 上溢和下溢都会触发UP中断,只有在计数值向上溢出的时候才需要翻转GPIO */
        gpio_bit_write(GPIO3_GPIO_PORT,GPIO3_PIN,(bit_status)(1-gpio_input_bit_get(GPIO3_GPIO_PORT,GPIO3_PIN)));
    }
}

void TIMER0_BRK_IRQHandler(void)
{
    timer_channel_output_mode_config(TIMER0,TIMER_CH_0,TIMER_OC_MODE_INACTIVE);
    timer_channel_output_mode_config(TIMER0,TIMER_CH_1,TIMER_OC_MODE_INACTIVE);
    timer_channel_output_mode_config(TIMER0,TIMER_CH_2,TIMER_OC_MODE_INACTIVE);
//    timer_primary_output_config(TIMER0,DISABLE);
    /* clear TIMER interrupt flag */
    timer_interrupt_flag_clear(TIMER0,TIMER_INT_FLAG_BRK);
    timer_disable(TIMER0);
    timer_disable(TIMER1);
    
    gpio_bit_write(GPIO2_GPIO_PORT,GPIO2_PIN,SET);
    led_on(LED_FAULT);
}

void TIMER0_Channel_IRQHandler(void)
{
    timer_interrupt_flag_clear(TIMER0,TIMER_INT_FLAG_CH3);
    
    /* 先更新上一个周期的PWM 在计数器到0的时候会自动更新 */
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,duty_u);
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,duty_v);
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,duty_w);

    /* 单电阻电流重构 */
    /* 这里暂时不进行电流重构,只需要验证PWM移相 */
    
    /* FOC运算 */
    /* 这里不需要FOC运算,但是还是需要运行跟FOC计算差不多时间的函数,否则时序无法对应 */
    /* 实际的代码请不要加while等待 */
    while(timer_counter_read(TIMER0) <= 300);
    
    /* 更新上一个周期的PWM移相后的值 这个函数在计数器到最大值之前执行 在计数器到最大值的时候自动更新PWM */
    /* 下面代码计算的是右半侧PWM占空比寄存器的值 */
    duty_u = duty_u - duty_u_offset - duty_u_offset;
    duty_v = duty_v - duty_v_offset - duty_v_offset;
    duty_w = duty_w - duty_w_offset - duty_w_offset;
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,duty_u);
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,duty_v);
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,duty_w);

    /* 移相计算 */
    /* U相向左移动10% offset < 0 */
    /* V相不移动 */
    /* W相向右移动10% offset > 0 */
    duty_u_offset = -((float)MOTOR_PWM_PERIOD / 10);
    duty_v_offset = 0;
    duty_w_offset = (float)MOTOR_PWM_PERIOD / 10;
//    duty_u_offset = 0;
//    duty_v_offset = 0;
//    duty_w_offset = 0;
    
    /* 下面代码计算的是左半侧PWM占空比寄存器的值 */
    duty_u = duty_u_ref + duty_u_offset;
    duty_v = duty_v_ref + duty_v_offset;
    duty_w = duty_w_ref + duty_w_offset;
    
    /* 更新采样点 */
    motor_pwm_set_adc_trigger_point(duty_u + (MOTOR_PWM_PERIOD / 10),duty_v + (MOTOR_PWM_PERIOD / 10));
    
    temp1 = (float)duty_u;
    memcpy(&uart_data[0],&temp1,4);
    temp2 = (float)duty_u_offset;
    memcpy(&uart_data[4],&temp2,4);
    temp3 = (float)duty_w;
    memcpy(&uart_data[8],&temp3,4);
    uart_data[sizeof(uart_data)-2] = 0x80;
    uart_data[sizeof(uart_data)-1] = 0x7f;
    usart_send_data(uart_data,sizeof(uart_data));
}

void ADC0_1_IRQHandler(void)
{
    /* clear the ADC flag */
    gpio_bit_write(GPIO3_GPIO_PORT,GPIO3_PIN,SET);
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOC);
    gpio_bit_write(GPIO3_GPIO_PORT,GPIO3_PIN,RESET);
}


/***************************************** (END OF FILE) *********************************************/
