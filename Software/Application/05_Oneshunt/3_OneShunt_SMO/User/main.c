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
#include "led.h"
#include "uart.h"
#include "encoder.h"
#include "drv8323rs.h"
#include "lcd.h"
#include "lcd_init.h"
#include "systick.h"
#include "adc_simple.h"
#include "foc.h"
#include "pmsm.h"
#include "userparms.h"
#include "task.h"
#include "motor_app.h"
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
    usart_init();
    adc1_init();
    encoder_init();
    LCD_Init();
    LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
    led_off(LED1);
    led_off(LED2);
    led_off(LED3);
    DRV8323RS_init();
    task_init();
    
    motor_hardware_init();
    
    motor_pwm_set_adc_trigger_point(MOTOR_PWM_PERIOD / 2, MOTOR_PWM_PERIOD);
    
    motor_phase_current_sampling_init();
    pmsm_foc_param.pwm_period = (float)MOTOR_PWM_PERIOD*0.98f;
    pmsm_foc_init();
    
	while(1)
	{
        task_scheduler();
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
    
    led_on(LED_FAULT);
}

void TIMER0_Channel_IRQHandler(void)
{
    timer_interrupt_flag_clear(TIMER0,TIMER_INT_FLAG_CH3);
    
    motor_app_isr();
}


/***************************************** (END OF FILE) *********************************************/
