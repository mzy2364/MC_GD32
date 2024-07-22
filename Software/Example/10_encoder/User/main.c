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
#include "encoder.h"
#include "uart.h"

/* DEFINES ------------------------------------------------------------------------------------------*/

/* VARIABLES ----------------------------------------------------------------------------------------*/
uint8_t uart_tx_buf[16] = {0};
float variable1 = 0;
float variable2 = 0;
float variable3 = 10;
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
    encoder_init();
    
	while(1)
	{
        memcpy(&uart_tx_buf[0],&variable1,4);
        memcpy(&uart_tx_buf[4],&variable2,4);
        memcpy(&uart_tx_buf[8],&variable3,4);
        uart_tx_buf[12] = 0x00;
        uart_tx_buf[13] = 0x00;
        uart_tx_buf[14] = 0x80;
        uart_tx_buf[15] = 0x7f;
        encoder_code = encoder_get();
        if(encoder_code == ENCODER_INC)
        {
            variable1 = variable1 + 1;
        }
        else if(encoder_code == ENCODER_DEC)
        {
            variable1 = variable1 - 1;
        }
        usart_send_data(uart_tx_buf,sizeof(uart_tx_buf));
        led_toggle(LED_SYS);
        systick_delay(100);
	}
}

/***************************************** (END OF FILE) *********************************************/
