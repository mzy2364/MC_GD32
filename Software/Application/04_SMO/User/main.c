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
#include "encoder.h"
#include "eeprom.h"
#include "delay.h"
#include "adc_simple.h"
#include "lcd.h"
#include "lcd_init.h"
#include "can.h"
#include "drv8323rs.h"
#include "foc.h"
#include "pmsm.h"
#include "userparms.h"
#include "task.h"
#include "motor_app.h"

/* DEFINES ------------------------------------------------------------------------------------------*/
#define START_DELAY_SEC     (float)2
#define START_DELAY_TICK    (uint32_t)((float)START_DELAY_SEC/(1/(float)MOTOR_PWM_FREQ_HZ))

/* VARIABLES ----------------------------------------------------------------------------------------*/
uint8_t adc_calibration = 0;
uint32_t ia_offset = 0,ib_offset = 0,ic_offset = 0,idc_offset = 0;
uint16_t adc1_ia = 0,adc1_ib = 0,adc1_ic = 0,adc1_idc = 0;

uint32_t start_delay_tick = 0;

extern uint16_t adc_filter_buffer[ADC_FILTER_COUNT];
extern uint16_t speed_rpm;
extern uint16_t adc_filter_count;

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
    \brief      this function handles CAN0 RX0 exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBD_LP_CAN0_RX0_IRQHandler(void)
{
    can_receive_message_struct receive_message;
    /* check the receive message */
    can_message_receive(CAN0, CAN_FIFO0, &receive_message);
}


/***************************************** (END OF FILE) *********************************************/
