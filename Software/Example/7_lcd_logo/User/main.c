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
extern const unsigned char gImage_logo[64800];

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
    LCD_ShowPicture(0,0,240,135,gImage_logo);
    
	while(1)
	{
        led_toggle(LED_SYS);
        systick_delay(100);
	}
}


/***************************************** (END OF FILE) *********************************************/
