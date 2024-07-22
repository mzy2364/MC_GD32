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
#include "uart.h"
/* DEFINES ------------------------------------------------------------------------------------------*/

/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/


/**
  * @brief main function
  * @param None
  * @retval None
  */
int main(void)
{
    uint8_t uart_tx_buf[16] = {0};
    float variable1 = 0;
    float variable2 = 10;
    float variable3 = 10;
    
    SCB->VTOR = FLASH_BASE | 0x4000;
    __enable_irq();
    
    systick_init();
    usart_init();

	while(1)
	{
        memcpy(&uart_tx_buf[0],&variable1,4);
        memcpy(&uart_tx_buf[4],&variable2,4);
        memcpy(&uart_tx_buf[8],&variable3,4);
        uart_tx_buf[12] = 0x00;
        uart_tx_buf[13] = 0x00;
        uart_tx_buf[14] = 0x80;
        uart_tx_buf[15] = 0x7f;
        variable1 = variable1 + 1;
        if(variable1 > 100)
        {
            variable1 = 0;
        }
        usart_send_data(uart_tx_buf,sizeof(uart_tx_buf));
        systick_delay(10);
	}
}


/***************************************** (END OF FILE) *********************************************/
