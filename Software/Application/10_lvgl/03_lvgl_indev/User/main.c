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
#include "key_fifo.h"
#include "lcd.h"
#include "lcd_init.h"
#include "lvgl.h"
#include "lv_port_disp_mcgd32.h"
#include "demo.h"
#include "benchmark.h"
/* DEFINES ------------------------------------------------------------------------------------------*/

/* VARIABLES ----------------------------------------------------------------------------------------*/
extern lv_obj_t * tv;

/* FUNCTION -----------------------------------------------------------------------------------------*/
void lv_tutorial_hello_world(void);

/**
  * @brief main function
  * @param None
  * @retval None
  */
int main(void)
{
    uint8_t key = KEY_NONE;
    encoder_code_t encoder_code = ENCODER_NONE;
    
    SCB->VTOR = FLASH_BASE | 0x4000;
    __enable_irq();
    
    systick_init();
    led_init();
    key_fifo_init();
    encoder_init();
    
    LCD_Init();//LCD≥ı ºªØ
	LCD_Fill(0,0,LCD_W,LCD_H,BRED);
//    LCD_ShowString(0,0,(const uint8_t *)"MC_GD32",BLACK,WHITE,32,0);
//    LCD_ShowString(0,32,(const uint8_t *)"GD32F303RCT6",BLACK,WHITE,32,0);
//    LCD_ShowString(0,64,(const uint8_t *)"ARM Cortex-M4F",BLACK,WHITE,32,0);
    
	lv_init();
	lv_port_disp_init();
    
    //lv_tutorial_hello_world();
    demo_create();

	while(1)
	{
        key_scan();
        key = key_get();
        switch(key)
        {
        case KEY_0_RELEASE:
            break;
        case KEY_0_LONG_PRESS:
            break;
        default:
            break;
        }

        encoder_code = encoder_get();
        switch(encoder_code)
        {
        case ENCODER_NONE:
            break;
        case ENCODER_INC:
            if(tv)
            {
                if((lv_tabview_get_tab_act(tv)+1) < lv_tabview_get_tab_count(tv))
                    lv_tabview_set_tab_act(tv,lv_tabview_get_tab_act(tv)+1,LV_ANIM_ON);
            }
            break;
        case ENCODER_DEC:
            if(tv)
            {
                if(lv_tabview_get_tab_act(tv) > 0)
                    lv_tabview_set_tab_act(tv,lv_tabview_get_tab_act(tv)-1,LV_ANIM_ON);
            }
            break;
        }

        lv_task_handler();
        
        systick_delay(10);
	}
}


/**
 * Create a simple 'Hello world!' label
 */
void lv_tutorial_hello_world(void)
{
    lv_obj_t * scr = lv_disp_get_scr_act(NULL);     /*Get the current screen*/

    /*Create a Label on the currently active screen*/
    lv_obj_t * label1 =  lv_label_create(scr, NULL);

    /*Modify the Label's text*/
    lv_label_set_text(label1, "Hello world!");

    /* Align the Label to the center
     * NULL means align on parent (which is the screen now)
     * 0, 0 at the end means an x, y offset after alignment*/
    lv_obj_align(label1, NULL, LV_ALIGN_CENTER, 0, 0);
}

/***************************************** (END OF FILE) *********************************************/
