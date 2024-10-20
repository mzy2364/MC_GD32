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
#include "task.h"
#include "system_define.h"
#include "pmsm.h"

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
    DRV8323RS_init();
    task_init();
    
    motor_hardware_init();
    pmsm_foc_param.pwm_period = (float)MOTOR_PWM_PERIOD * 0.9f;
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
        }
    }
    else if(start_delay_tick < START_DELAY_TICK)
    {
        start_delay_tick++;
    }
    else
    {
        /* read ADC inserted group data register */
        adc1_ia = adc_inserted_data_read(ADC0, IU_INSERTED_CHANNEL);
        adc1_ib = adc_inserted_data_read(ADC0, IV_INSERTED_CHANNEL);
        adc1_ic = adc_inserted_data_read(ADC0, IW_INSERTED_CHANNEL);
        adc1_idc = adc_inserted_data_read(ADC0, IDC_AVER_INSERTED_CHANNEL);
        
		if(adc_filter_count < ADC_FILTER_COUNT)
		{
			adc_filter_buffer[adc_filter_count++] = adc1_idc;
		}
		else
		{
			memcpy(&adc_filter_buffer[0], &adc_filter_buffer[1], (ADC_FILTER_COUNT - 1) << 1);
			adc_filter_buffer[ADC_FILTER_COUNT-1] = adc1_idc;
		}
        pmsm_foc_param.ia = (adc1_ia - ia_offset) << 3;
        pmsm_foc_param.ib = (adc1_ib - ib_offset) << 3;
//        pmsm_foc_run();
        
//        ParkParm.qIa = (adc1_ia - ia_offset) << 3;
//        ParkParm.qIb = (adc1_ib - ib_offset) << 3;
        pmsm_foc_run();
        
        temp1 = (float)smc1.Ialpha;
        memcpy(&uart_data[0],&temp1,4);
        temp2 = (float)smc1.EstIalpha;
        memcpy(&uart_data[4],&temp2,4);
        temp3 = (float)smc1.Ealpha;
        memcpy(&uart_data[8],&temp3,4);
        uart_data[sizeof(uart_data)-2] = 0x80;
        uart_data[sizeof(uart_data)-1] = 0x7f;
        usart_send_data(uart_data,sizeof(uart_data));
        
        motor_pwm_set_duty(pmsm_foc_param.pwma,pmsm_foc_param.pwmb,pmsm_foc_param.pwmc);
    }
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
