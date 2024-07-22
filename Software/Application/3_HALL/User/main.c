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
#include "foc.h"
#include "pmsm.h"
#include "userparms.h"

/* DEFINES ------------------------------------------------------------------------------------------*/
#define PHASE_SHIFT_ANGLE (float)       (45.0f/180.0f*ANGLE_2PI)

/* VARIABLES ----------------------------------------------------------------------------------------*/
uint8_t adc_calibration = 0;
uint32_t ia_offset = 0,ib_offset = 0;
uint16_t adc1_ia = 0,adc1_ib = 0;

uint32_t hall_timer_counter = 0;
float hall_theta = 0;
float hall_theta_inc = 0;
float hall_speed = 0;
uint8_t hall_state = 0;
uint8_t hall_last_state = 0;
uint32_t hall_tick = 0;

float hall_theta_tab[6] = {0,PI*2/3,PI/3,PI*4/3,PI*5/3,PI};

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
    
    motor_hardware_init();
    pmsm_foc_param.pwm_period = (float)MOTOR_PWM_PERIOD * 0.96f;
    pmsm_foc_init();
    
    while(1)
    {
        led_toggle(LED1);
        systick_delay(100);
        led_toggle(LED2);
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
    float temp1 = 0;
    float temp2 = 0;
    float temp3 = 0;
    uint8_t uart_data[16] = {0};
    
    /* clear the ADC flag */
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
    
    if(adc_calibration == 0)
    {
        static uint16_t cnt;
        
        cnt++;
        adc1_ia = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_3);
        adc1_ib = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_2);
        ia_offset += adc1_ia;
        ib_offset += adc1_ib;
        if(cnt >= (1 << 12))
        {
            adc_calibration = 1;
            ia_offset = ia_offset >> 12;
            ib_offset = ib_offset >> 12;
            
            hall_state = hall_get();
            if((hall_state > 0) && (hall_state <= 6))
            {
                hall_theta = hall_theta_tab[hall_state - 1] + PHASE_SHIFT_ANGLE;
            }
            pmsm_mc_param.openloop = 0;
        }
    }
    else
    {
        hall_theta = hall_theta + hall_theta_inc;
        if(hall_theta < 0.0f)
        {
            hall_theta += ANGLE_2PI;
        }
        else if(hall_theta > ANGLE_2PI)
        {
            hall_theta -= ANGLE_2PI;
        }
        
        /* read ADC inserted group data register */
        adc1_ia = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_3);
        adc1_ib = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_2);

        temp1 = ((float)ia_offset - adc1_ia) * ADC_TO_CURRENT_COEF;
        temp2 = ((float)ib_offset - adc1_ib) * ADC_TO_CURRENT_COEF;
        
        pmsm_foc_param.ia = temp1;
        pmsm_foc_param.ib = temp2;
        pmsm_foc_param.ic = 0 - pmsm_foc_param.ia - pmsm_foc_param.ib;
        pmsm_foc_param.angle = hall_theta;
        
        pmsm_foc_run();
        
        temp1 = (float)hall_theta;
        memcpy(&uart_data[0],&temp1,4);
        temp2 = (float)hall_state;
        memcpy(&uart_data[4],&temp2,4);
        temp3 = (float)pmsm_foc_param.angle;
        memcpy(&uart_data[8],&temp3,4);
        uart_data[sizeof(uart_data)-2] = 0x80;
        uart_data[sizeof(uart_data)-1] = 0x7f;
        usart_send_data(uart_data,sizeof(uart_data));
        
        timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,pmsm_foc_param.pwma);
        timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,pmsm_foc_param.pwmb);
        timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,pmsm_foc_param.pwmc);
    }
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
