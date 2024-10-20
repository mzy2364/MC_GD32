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
#include "systick.h"
#include "led.h"
#include "drv8323rs.h"
#include "motor_hardware.h"

/* DEFINES ------------------------------------------------------------------------------------------*/

/* VARIABLES ----------------------------------------------------------------------------------------*/
uint16_t vdc_adc = 0;
uint16_t ntc_adc = 0;
float temperature = 0;
float vdc_voltage = 0;
uint8_t uart_tx_buf[16] = {0};
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
    DRV8323RS_init();
    
    rcu_periph_clock_enable(USER_IO1_CLK);
    gpio_init(USER_IO1_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, USER_IO1_PIN);
    gpio_bit_write(USER_IO1_PORT,USER_IO1_PIN,RESET);
    
    motor_hardware_init();
    motor_pwm_enable();
    
    motor_pwm_set_duty(MOTOR_PWM_PERIOD/8,MOTOR_PWM_PERIOD/4,MOTOR_PWM_PERIOD/2);
    
	while(1)
	{
        led_toggle(LED_SYS);
        systick_delay(100);
	}
}

void TIMER0_BRK_IRQHandler(void)
{
    timer_channel_output_mode_config(TIMER0,TIMER_CH_0,TIMER_OC_MODE_INACTIVE);
    timer_channel_output_mode_config(TIMER0,TIMER_CH_1,TIMER_OC_MODE_INACTIVE);
    timer_channel_output_mode_config(TIMER0,TIMER_CH_2,TIMER_OC_MODE_INACTIVE);
    /* clear TIMER interrupt flag */
    timer_interrupt_flag_clear(TIMER0,TIMER_INT_FLAG_BRK);
    
    led_on(LED_FAULT);
}

void ADC0_1_IRQHandler(void)
{
    gpio_bit_write(USER_IO1_PORT,USER_IO1_PIN,(1 - gpio_output_bit_get(USER_IO1_PORT,USER_IO1_PIN)));
    /* clear the ADC flag */
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
}


/***************************************** (END OF FILE) *********************************************/
