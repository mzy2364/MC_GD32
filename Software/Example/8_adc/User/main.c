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
#include "uart.h"
#include "adc_simple.h"

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
    usart_init();
    adc1_init();
    
	while(1)
	{
        ntc_adc = adc_get_ntc();
        temperature = calculate_temperature_float(ntc_adc);
        vdc_adc = adc_get_vdc();
        vdc_voltage = (float)vdc_adc * 3.3 / 4096 * 26;
        
        memcpy(&uart_tx_buf[0],&temperature,4);
        memcpy(&uart_tx_buf[4],&vdc_voltage,4);
        uart_tx_buf[12] = 0x00;
        uart_tx_buf[13] = 0x00;
        uart_tx_buf[14] = 0x80;
        uart_tx_buf[15] = 0x7f;
        usart_send_data(uart_tx_buf,sizeof(uart_tx_buf));
        
        led_toggle(LED_SYS);
        systick_delay(100);
	}
}


/***************************************** (END OF FILE) *********************************************/
