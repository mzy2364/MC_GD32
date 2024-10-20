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
#include "motor_hardware.h"
#include "uart.h"
#include "i2c.h"
#include "eeprom.h"
#include "delay.h"
#include "adc_simple.h"
#include "lcd.h"
#include "lcd_init.h"
#include "can.h"
#include "encoder.h"
#include "drv8323rs.h"
#include "task.h"
#include "foc.h"
#include "pmsm.h"
#include "userparms.h"
#include "motor_app.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/
uint32_t hall_timer_counter = 0;
float hall_theta = 0;
float hall_theta_inc = 0;
float hall_speed = 0;
uint8_t hall_state = 0;
uint8_t hall_last_state = 0;
uint32_t hall_tick = 0;

float hall_theta_tab[6] = {0,PI*2/3,PI/3,PI*4/3,PI*5/3,PI};

encoder_code_t encoder_code = ENCODER_NONE;

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
    can_bus_init();
    encoder_init();
    LCD_Init();
	LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
    DRV8323RS_init();
    task_init();
    
    motor_hardware_init();
    pmsm_foc_param.pwm_period = (float)MOTOR_PWM_PERIOD * 0.95f;
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
    /* clear TIMER interrupt flag */
    timer_interrupt_flag_clear(TIMER0,TIMER_INT_FLAG_BRK);
    
    led_on(LED_FAULT);
}

void ADC0_1_IRQHandler(void)
{
    /* clear the ADC flag */
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
    motor_app_isr();
}

/*!
    \brief      this function handles TIMER2 interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER2_IRQHandler(void)
{
    /* hall_speed = 2PI / (6 * delta_t) */
    /* delta_t = hall_timer_counter / timer_clk */
    if(SET == timer_interrupt_flag_get(TIMER2,TIMER_INT_FLAG_CH0))
    {
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER2,TIMER_INT_FLAG_CH0);
        
        hall_timer_counter = timer_channel_capture_value_register_read(TIMER2,TIMER_CH_0)+1;
        hall_theta_inc = (PI/3)/((float)hall_timer_counter/HALL_TIMER_FREQ_HZ)/MOTOR_PWM_FREQ_HZ;
        hall_speed = (PI/3)/((float)hall_timer_counter/HALL_TIMER_FREQ_HZ);
        hall_state = hall_get();
        if((hall_state > 0) && (hall_state <= 6))
        {
            hall_theta = hall_theta_tab[hall_state - 1] + PHASE_SHIFT_ANGLE;
        }
    }
}

/***************************************** (END OF FILE) *********************************************/
