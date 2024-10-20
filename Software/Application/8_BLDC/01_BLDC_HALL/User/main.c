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
#include "drv8323rs.h"
#include "bldc_hall.h"
#include "encoder.h"
#include "key_fifo.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/
float temp1 = 0,temp2 = 0,temp3 = 0;
uint8_t uart_data[16] = {0};

uint32_t tick = 0;
uint8_t key = KEY_NONE;
encoder_code_t encoder_code = ENCODER_NONE;

uint16_t adc1_ia = 0;
uint16_t adc1_ib = 0;
uint16_t adc1_ic = 0;
uint16_t adc1_idc = 0;
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
    encoder_init();
    key_fifo_init();
    DRV8323RS_init();
    
    motor_hardware_init();
    
    timer_enable(TIMER0);
    
    systick_delay(2000);
    
    bldc_init();
    g_bldc_motor.dir = CCW;
    g_bldc_motor.run_flag = RUN;           /* 开启运行 */
    g_bldc_motor.pwm_duty = MOTOR_PWM_PERIOD / 2;
    g_bldc_motor.target_speed = 3000;
	start_motor();                         /* 开启运行 */
    
    while(1)
    {
        tick++;
        if(tick >= 50)
        {
            tick = 0;
            led_toggle(LED_SYS);
            if(g_bldc_motor.run_flag == RUN)
            {
                led_on(LED1);
            }
            else
            {
                led_off(LED1);
            }
        }
        
        key_scan();
        key = key_get();
        switch(key)
        {
        case KEY_0_RELEASE:
            if(g_bldc_motor.run_flag == RUN)
            {
                g_bldc_motor.run_flag = STOP;
                stop_motor();
            }
            else
            {
                g_bldc_motor.run_flag = RUN;
                start_motor();
            }
            break;
        case KEY_0_LONG_PRESS:
            break;
        default:
            break;
        }

        encoder_code = encoder_get();
        switch(encoder_code)
        {
        case ENCODER_NONE:
            break;
        case ENCODER_INC:
#if(SPEED_LOOP == 1)
            if(g_bldc_motor.run_flag == RUN)
            {
                if(g_bldc_motor.target_speed < MAX_SPEED)
                    g_bldc_motor.target_speed += 100;
            }
#else
            if(g_bldc_motor.run_flag == RUN)
            {
                if(g_bldc_motor.pwm_duty < MAX_PWM_DUTY)
                    g_bldc_motor.pwm_duty += 100;
            }
#endif
            break;
        case ENCODER_DEC:
#if(SPEED_LOOP == 1)
            if(g_bldc_motor.run_flag == RUN)
            {
                if(g_bldc_motor.target_speed > MIN_SPEED)
                    g_bldc_motor.target_speed -= 100;
            }
#else
            if(g_bldc_motor.run_flag == RUN)
            {
                if(g_bldc_motor.pwm_duty > MIN_PWM_DUTY)
                    g_bldc_motor.pwm_duty -= 100;
            }
#endif
            break;
        }
        
        systick_delay(10);
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

void TIMER0_Channel_IRQHandler(void)
{
    timer_interrupt_flag_clear(TIMER0,TIMER_INT_FLAG_CH3);
    
    adc1_ia = adc_inserted_data_read(ADC0, IU_INSERTED_CHANNEL);
    adc1_ib = adc_inserted_data_read(ADC0, IV_INSERTED_CHANNEL);
    adc1_ic = adc_inserted_data_read(ADC0, IW_INSERTED_CHANNEL);
    adc1_idc = adc_inserted_data_read(ADC0, IDC_AVER_INSERTED_CHANNEL);
    
    bldc_mainfunction();
    
    temp1 = (float)adc1_ia * ADC_TO_VDC_COEF;
    memcpy(&uart_data[0],&temp1,4);
    temp2 = (float)adc1_ib * ADC_TO_VDC_COEF;
    memcpy(&uart_data[4],&temp2,4);
    temp3 = (float)adc1_ic * ADC_TO_VDC_COEF;
    memcpy(&uart_data[8],&temp3,4);
    uart_data[sizeof(uart_data)-2] = 0x80;
    uart_data[sizeof(uart_data)-1] = 0x7f;
    usart_send_data(uart_data,sizeof(uart_data));
    
//    temp1 = (float)g_bldc_motor.step_sta;
//    memcpy(&uart_data[0],&temp1,4);
//    temp2 = (float)g_bldc_motor.speed;
//    memcpy(&uart_data[4],&temp2,4);
//    temp3 = (float)g_bldc_motor.no_single;
//    memcpy(&uart_data[8],&temp3,4);
//    uart_data[sizeof(uart_data)-2] = 0x80;
//    uart_data[sizeof(uart_data)-1] = 0x7f;
//    usart_send_data(uart_data,sizeof(uart_data));
}

void ADC0_1_IRQHandler(void)
{
    /* clear the ADC flag */
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
}


/***************************************** (END OF FILE) *********************************************/
