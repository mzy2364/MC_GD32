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
#include "i2c.h"
#include "delay.h"
#include "eeprom.h"
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
    const uint8_t eeprom_write_buffer0[] = "MotorControlUnitGD32";
    const uint8_t eeprom_write_buffer1[] = "MC_GD32 EEPROM TEST!";
    uint8_t eeprom_read_buffer[30] = {0};
    uint8_t enter[2] = {"\r\n"};
    
    SCB->VTOR = FLASH_BASE | 0x4000;
    __enable_irq();
    
    systick_init();
    delay_tim_init();
    i2c_init();
    usart_init();
    
    eeprom_write(0,(uint8_t *)eeprom_write_buffer0,sizeof(eeprom_write_buffer0));
    systick_delay(100);
    eeprom_read(0,eeprom_read_buffer,sizeof(eeprom_write_buffer0));

    usart_send_data(eeprom_read_buffer,sizeof(eeprom_write_buffer0));
    memset(eeprom_read_buffer,0,sizeof(eeprom_read_buffer));
    systick_delay(100);
    usart_send_data(enter,sizeof(enter));
    
    eeprom_write(0,(uint8_t *)eeprom_write_buffer1,sizeof(eeprom_write_buffer1));
    systick_delay(100);
    eeprom_read(0,eeprom_read_buffer,sizeof(eeprom_write_buffer1));
    
    usart_send_data(eeprom_read_buffer,sizeof(eeprom_write_buffer1));
    systick_delay(100);
    usart_send_data(enter,sizeof(enter));
    
	while(1)
	{

	}
}


/***************************************** (END OF FILE) *********************************************/
