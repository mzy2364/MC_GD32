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
#include "uart.h"
#include "iap.h"

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
    uint8_t gpio = 0;
    systick_init();
    uart3_init();
    iap_init();
    
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
    gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    
    gpio = gpio_input_bit_get(GPIOB,GPIO_PIN_6);
    gpio += gpio_input_bit_get(GPIOB,GPIO_PIN_7);
    gpio += gpio_input_bit_get(GPIOC,GPIO_PIN_0);
    
    if(gpio != 0)
    {
        jump_to_app();
    }
    
	while(1)
	{
        iap_task();
    }
}



/***************************************** (END OF FILE) *********************************************/
