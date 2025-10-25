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
#include "motor_hardware.h"
#include "rs485.h"

/* DEFINES ------------------------------------------------------------------------------------------*/

/* VARIABLES ----------------------------------------------------------------------------------------*/
uint8_t uart_tx_buf[16] = {0};
uint8_t rs485_uart_tx_buf[16] = {0};
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
    
    uint8_t i = 0;
    uint8_t rs485_rx_len = 0;
    
    systick_init();
    led_init();
    usart_init();
    rs485_usart_init();
    
    for(i=0;i<sizeof(rs485_uart_tx_buf);i++)
    {
        rs485_uart_tx_buf[i] = i;
    }

	while(1)
	{
        if (rs485_get_idle_status() == 1)
        {
            rs485_clear_idle_status();
            if ((rs485_rx_len = rs485_fifo_get_data_length())>0)
            {
                if(rs485_rx_len >= sizeof(rs485_uart_tx_buf))
                    rs485_rx_len = rs485_fifo_read(rs485_uart_tx_buf,sizeof(rs485_uart_tx_buf));
                else
                    rs485_rx_len = rs485_fifo_read(rs485_uart_tx_buf,rs485_rx_len);
            }
        }
        
        rs485_usart_send_data(rs485_uart_tx_buf,sizeof(rs485_uart_tx_buf));
        led_toggle(LED_SYS);
        systick_delay(500);
	}
}



/***************************************** (END OF FILE) *********************************************/
