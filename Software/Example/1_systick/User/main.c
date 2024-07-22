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
#include "gd32f30x.h"
#include "gd32f30x_libopt.h"
#include "systick.h"
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
    SCB->VTOR = FLASH_BASE | 0x4000;
    __enable_irq();
    
    systick_init();
    
    rcu_periph_clock_enable(RCU_GPIOD);
    gpio_init(GPIOD, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_2);
    gpio_bit_write(GPIOD,GPIO_PIN_2,RESET);
    
	while(1)
	{
        gpio_bit_write(GPIOD,GPIO_PIN_2,(bit_status)(1-gpio_input_bit_get(GPIOD,GPIO_PIN_2)));
        systick_delay(500);
	}
}


/***************************************** (END OF FILE) *********************************************/
