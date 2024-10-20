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
#include "mt6701.h"
#include "foc.h"
#include "pmsm.h"
#include "userparms.h"

/* DEFINES ------------------------------------------------------------------------------------------*/
#define START_DELAY_SEC     (float)2
#define START_DELAY_TICK    (uint32_t)((float)START_DELAY_SEC/(1/(float)MOTOR_PWM_FREQ_HZ))
    
#define PHASE_SHIFT_ANGLE (float)       (45.0f/180.0f*ANGLE_2PI)

/* VARIABLES ----------------------------------------------------------------------------------------*/
uint8_t adc_calibration = 0;
uint32_t ia_offset = 0,ib_offset = 0,ic_offset = 0,idc_offset = 0;
uint16_t adc1_ia = 0,adc1_ib = 0,adc1_ic = 0,adc1_idc = 0;

uint32_t start_delay_tick = 0;

float delta_angle = 0;
float angle_last = 0;
float angle_init = 0;
float angle = 0;
float mt6701_speed = 0;
uint16_t angle_u16 = 0;

uint16_t speed_filter_count = 0;
float speed_buf[10] = {0};
float speed_sum = 0;

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
    DRV8323RS_init();
    mt6701_init();
    
    LCD_Init();//LCD³õÊ¼»¯
	LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
    LCD_ShowString(0,0,(const uint8_t *)"MC_GD32",BLACK,WHITE,32,0);
    LCD_ShowString(0,32,(const uint8_t *)"GD32F303RCT6",BLACK,WHITE,32,0);
    LCD_ShowString(0,64,(const uint8_t *)"ARM Cortex-M4F",BLACK,WHITE,32,0);
    
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
    uint16_t i = 0;
    
    /* clear the ADC flag */
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
    
    if(adc_calibration == 0)
    {
        static uint16_t cnt;
        
        cnt++;
        adc1_ia = adc_inserted_data_read(ADC0, IU_INSERTED_CHANNEL);
        adc1_ib = adc_inserted_data_read(ADC0, IV_INSERTED_CHANNEL);
        adc1_ic = adc_inserted_data_read(ADC0, IW_INSERTED_CHANNEL);
        adc1_idc = adc_inserted_data_read(ADC0, IDC_AVER_INSERTED_CHANNEL);
        ia_offset += adc1_ia;
        ib_offset += adc1_ib;
        ic_offset += adc1_ic;
        idc_offset += adc1_idc;
        if(cnt >= (1 << 12))
        {
            adc_calibration = 1;
            ia_offset = ia_offset >> 12;
            ib_offset = ib_offset >> 12;
            ic_offset = ic_offset >> 12;
            idc_offset = idc_offset >> 12;

            motor_pwm_channel_enable(ENABLE);
            pmsm_mc_param.openloop = 0;
        }
    }
    else if(start_delay_tick < START_DELAY_TICK)
    {
        start_delay_tick++;
        mt6701_read_angle0(&angle_u16);
        angle_u16 = angle_u16 % (16384/7);
        angle = (float)angle_u16 * (2*M_PI) / (16384/7);
        angle_last = angle;
    }
    else
    {
        /* read ADC inserted group data register */
        adc1_ia = adc_inserted_data_read(ADC0, IU_INSERTED_CHANNEL);
        adc1_ib = adc_inserted_data_read(ADC0, IV_INSERTED_CHANNEL);
        adc1_ic = adc_inserted_data_read(ADC0, IW_INSERTED_CHANNEL);
        adc1_idc = adc_inserted_data_read(ADC0, IDC_AVER_INSERTED_CHANNEL);

        temp1 = ((float)ia_offset - adc1_ia) * ADC_TO_CURRENT_COEF;
        temp2 = ((float)ib_offset - adc1_ib) * ADC_TO_CURRENT_COEF;
        
        pmsm_foc_param.ia = ((float)adc1_ia - ia_offset) * ADC_TO_CURRENT_COEF;
        pmsm_foc_param.ib = ((float)adc1_ib - ib_offset) * ADC_TO_CURRENT_COEF;
        pmsm_foc_param.ic = ((float)adc1_ic - ic_offset) * ADC_TO_CURRENT_COEF;
        
        mt6701_read_angle0(&angle_u16);
        angle_u16 = angle_u16 % (16384/7);
        angle = (float)angle_u16 * (2*M_PI) / (16384/7);
        if(angle > angle_last)
        {
            delta_angle = angle - angle_last;
        }
        else
        {
            delta_angle = (2*M_PI) + angle - angle_last;
            if(delta_angle > (M_PI))
                delta_angle = delta_angle - (2*M_PI);
        }
        mt6701_speed = delta_angle * PWMFREQUENCY_HZ;
        speed_buf[speed_filter_count] = mt6701_speed;
        speed_filter_count++;
        if(speed_filter_count >= 10)
        {
            speed_filter_count = 0;
        }
        speed_sum = 0;
        for(i=0;i<10;i++)
        {
            speed_sum += speed_buf[i];
        }
        mt6701_speed = speed_sum / 10;
        
        pmsm_mc_param.hall_speed = mt6701_speed;
        pmsm_foc_param.angle = angle + (M_PI / 4);
        pmsm_foc_run();
        
        angle_last = angle;
        
        temp1 = (float)angle;
        memcpy(&uart_data[0],&temp1,4);
        temp2 = (float)mt6701_speed;
        memcpy(&uart_data[4],&temp2,4);
        temp3 = (float)pmsm_foc_param.iq;
        memcpy(&uart_data[8],&temp3,4);
        uart_data[sizeof(uart_data)-2] = 0x80;
        uart_data[sizeof(uart_data)-1] = 0x7f;
        usart_send_data(uart_data,sizeof(uart_data));
        
        motor_pwm_set_duty(pmsm_foc_param.pwma,pmsm_foc_param.pwmb,pmsm_foc_param.pwmc);
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
    }
}

/***************************************** (END OF FILE) *********************************************/
