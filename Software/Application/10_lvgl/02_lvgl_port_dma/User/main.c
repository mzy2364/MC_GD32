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
#include "lvgl.h"
#include "lv_port_disp_mcgd32.h"
#include "demo.h"
#include "benchmark.h"
/* DEFINES ------------------------------------------------------------------------------------------*/

/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/
void lv_tutorial_hello_world(void);

//static void refresh_rate_test() {
//    lv_obj_t * label;
//    static char buf[32];
// 
//    /*Create a label to show the refresh rate*/
//    label = lv_label_create(lv_scr_act(), NULL);
//    lv_label_set_text(label, "Refresh rate:");
////    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 10, 10);
// 
//    /*Refresh rate measurement loop*/
//    uint32_t last_time = lv_tick_get();
//    uint32_t count = 0;
//    while(1) {
//        uint32_t diff = lv_tick_get() - last_time;
//        if(diff >= 1000) { /*Refresh every 1000 milliseconds*/
//            last_time = lv_tick_get();
//            uint32_t refresh_rate = count * 1000 / diff;
//            sprintf(buf, "%d fps", refresh_rate);
//            lv_label_set_text(label, buf);
//            count = 0;
//        }
//        count++;
// 
//        /*Demonstrate other GUI activity*/
//        lv_obj_set_pos(label, 10, 40 + 20 * (count % 5));
// 
//        /*Handle LVGL tasks*/
//        lv_task_handler();
//        /*Sleep to simulate a CPU load (optional)*/
//        //lv_tick_inc(1); //Simulate 1 millisecond passed
//    }
//}


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
        lv_task_handler();
        led_toggle(LED_SYS);
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
