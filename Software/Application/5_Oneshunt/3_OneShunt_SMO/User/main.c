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
#include <string.h>
#include "gd32f30x.h"
#include "gd32f30x_libopt.h"
#include "motor_hardware.h"
#include "motor_config.h"
#include "systick.h"
#include "foc.h"
#include "pmsm.h"
#include "userparms.h"
#include "single_shunt_cs.h"

/* DEFINES ------------------------------------------------------------------------------------------*/

/* VARIABLES ----------------------------------------------------------------------------------------*/
float temp1 = 0,temp2 = 0,temp3 = 0;
uint8_t uart_data[16] = {0};

uint32_t start_delay = 3000;

extern uint8_t adc_calibration;

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
    
    motor_pwm_set_adc_trigger_point(MOTOR_PWM_PERIOD, MOTOR_PWM_PERIOD);
    
    motor_phase_current_sampling_init();
    pmsm_foc_param.pwm_period = (float)MOTOR_PWM_PERIOD*0.98f;
    pmsm_foc_init();

    gpio_bit_write(GPIO3_GPIO_PORT,GPIO3_PIN,RESET);
    
	while(1)
	{

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
    
    if(start_delay > 0)
    {
        start_delay--;
        return;
    }
    
    if(adc_calibration == 0)
    {
        motor_get_phase_current_zero();
        return;
    }
    
    gpio_bit_write(GPIO1_GPIO_PORT,GPIO1_PIN,SET);
    
    /* 先更新上一个周期的PWM 在计数器到0的时候会自动更新 */
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,duty_u);
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,duty_v);
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,duty_w);

    
    /* 单电阻电流重构 */
    motor_get_phase_current(pmsm_foc_param.sector);
    
    /* FOC运算 */
    pmsm_foc_run();
    
    /* 更新上一个周期的PWM移相后的值 这个函数在计数器到最大值之前执行 在计数器到最大值的时候自动更新PWM */
    duty_u = duty_u - duty_u_offset - duty_u_offset;
    duty_v = duty_v - duty_v_offset - duty_v_offset;
    duty_w = duty_w - duty_w_offset - duty_w_offset;
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,duty_u);
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,duty_v);
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,duty_w);

    /* 移相计算 */
    duty_u = pmsm_foc_param.pwma;
    duty_v = pmsm_foc_param.pwmb;
    duty_w = pmsm_foc_param.pwmc;
    motor_phase_current_sampling_shift(pmsm_foc_param.sector);
    
    temp1 = (float)pmsm_foc_param.ia;
    memcpy(&uart_data[0],&temp1,4);
    temp2 = (float)pmsm_foc_param.iq;
    memcpy(&uart_data[4],&temp2,4);
    temp3 = (float)smc1.ealpha_final;
    memcpy(&uart_data[8],&temp3,4);
    uart_data[sizeof(uart_data)-2] = 0x80;
    uart_data[sizeof(uart_data)-1] = 0x7f;
    usart_send_data(uart_data,sizeof(uart_data));

    gpio_bit_write(GPIO1_GPIO_PORT,GPIO1_PIN,RESET);
}


/***************************************** (END OF FILE) *********************************************/
