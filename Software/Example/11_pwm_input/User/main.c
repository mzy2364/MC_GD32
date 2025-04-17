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
#include "pwm_input.h"
#include "motor_hardware.h"

/* DEFINES ------------------------------------------------------------------------------------------*/

/* VARIABLES ----------------------------------------------------------------------------------------*/
uint16_t freq = 0;
uint16_t duty = 0;
uint8_t uart_tx_buf[16] = {0};
/* FUNCTION -----------------------------------------------------------------------------------------*/


/**
  * @brief main function
  * @param None
  * @retval None
  */
int main(void)
{
    float temp = 0;
    
    SCB->VTOR = FLASH_BASE | 0x4000;
    __enable_irq();
    
    systick_init();
    led_init();
    usart_init();
    pwm_input_init();

	while(1)
	{
        pwm_input_get_data(&freq,&duty);
        temp = freq;
        memcpy(&uart_tx_buf[0],&temp,4);
        temp = duty;
        memcpy(&uart_tx_buf[4],&temp,4);
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
