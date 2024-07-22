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

/* VARIABLES ----------------------------------------------------------------------------------------*/
uint8_t adc_calibration = 0;
uint32_t ia_offset = 0,ib_offset = 0;
uint16_t adc1_ia = 0,adc1_ib = 0;

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
    pmsm_foc_param.pwm_period = (float)MOTOR_PWM_PERIOD * 0.95f;
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
        }
    }
    else
    {
        /* read ADC inserted group data register */
        adc1_ia = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_3);
        adc1_ib = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_2);

        temp1 = ((float)ia_offset - adc1_ia) * ADC_TO_CURRENT_COEF;
        temp2 = ((float)ib_offset - adc1_ib) * ADC_TO_CURRENT_COEF;
        
        pmsm_foc_param.ia = temp1;
        pmsm_foc_param.ib = temp2;
        
        pmsm_foc_run();
        
        temp1 = (float)pmsm_foc_param.ia;
        memcpy(&uart_data[0],&temp1,4);
        temp2 = (float)pmsm_foc_param.ib;
        memcpy(&uart_data[4],&temp2,4);
        temp3 = (float)pmsm_foc_param.iq;
        memcpy(&uart_data[8],&temp3,4);
        uart_data[sizeof(uart_data)-2] = 0x80;
        uart_data[sizeof(uart_data)-1] = 0x7f;
        usart_send_data(uart_data,sizeof(uart_data));
        
        timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,pmsm_foc_param.pwma);
        timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,pmsm_foc_param.pwmb);
        timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,pmsm_foc_param.pwmc);
    }
}

/***************************************** (END OF FILE) *********************************************/
