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
#include "lcd.h"
#include "lcd_init.h"

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
    led_init();
    
    LCD_Init();//LCD≥ı ºªØ
	LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
    LCD_ShowString(0,0,(const uint8_t *)"MC_GD32",BLACK,WHITE,32,0);
    LCD_ShowString(0,32,(const uint8_t *)"GD32F303RCT6",BLACK,WHITE,32,0);
    LCD_ShowString(0,64,(const uint8_t *)"ARM Cortex-M4F",BLACK,WHITE,32,0);
    
	while(1)
	{
        led_toggle(LED_SYS);
        systick_delay(100);
	}
}


/***************************************** (END OF FILE) *********************************************/
