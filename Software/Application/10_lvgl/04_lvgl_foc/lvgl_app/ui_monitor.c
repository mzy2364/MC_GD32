/*
 * ui_monitor.c
 *
 *  Created on: 2025-3-17
 *      Author: mzy2364
 */

/*******************************************************************************
* include files
*******************************************************************************/
#include "mainwindow.h"
#include "ee_parameter.h"

/*******************************************************************************
* Defines
*******************************************************************************/

/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Global Constant definition
*******************************************************************************/

/*******************************************************************************
* Local Constant definition
*******************************************************************************/

/*******************************************************************************
* Global Variables definition
*******************************************************************************/
LV_IMG_DECLARE(rotor)

/*******************************************************************************
* Local Variables definition
*******************************************************************************/
lv_obj_t *label_const_status;
lv_obj_t *label_const_speed;
lv_obj_t *label_const_current;
lv_obj_t *label_const_voltage;
lv_obj_t *label_const_temp;
lv_obj_t *lv_preload;
lv_obj_t *lv_led;
lv_obj_t *lv_bar_ref_spd;

lv_obj_t *label_status;
lv_obj_t *label_speed;
lv_obj_t *label_current;
lv_obj_t *label_voltage;
lv_obj_t *label_temp;
lv_obj_t *label_ref_spd;

/*******************************************************************************
* Local Functions prototypes
*******************************************************************************/
static void ui_monitor_event_cb(lv_obj_t* obj, lv_event_t event);
static uint32_t my_pow(uint8_t m,uint8_t n);
static void ui_label_show_int_num(lv_obj_t *label,int32_t num,uint8_t len,const char * unit_text,uint8_t unit_len);
static void ui_label_show_float_num(lv_obj_t *label,float num,uint8_t len,const char * unit_text,uint8_t unit_len);
/*******************************************************************************
*  Global Functions Body
*******************************************************************************/

/**
  * @brief monitor create
  * @param parent - parent object
  * @retval	current object
  * @note
  */
lv_obj_t *ui_monitor_create(lv_obj_t *parent)
{
    static lv_style_t style_label;
    static lv_style_t style_label_const;
    static lv_style_t style_bg;
    static lv_style_t style_preload;
    static lv_style_t style_led;
    static lv_style_t style_bar_bg;
    static lv_style_t style_bar_inc;
    
    lv_style_copy(&style_label_const, &lv_style_plain);
    lv_style_copy(&style_bg, &lv_style_scr);
    style_label_const.text.font = &lv_font_roboto_22;
    style_label_const.text.color = LV_COLOR_WHITE;
    
    lv_style_copy(&style_label, &style_label_const);
    style_label.text.color = LV_COLOR_YELLOW;
   
    style_bg.body.grad_color = LV_COLOR_GRAY;
    style_bg.body.main_color = LV_COLOR_GRAY;
    
    lv_style_copy(&style_preload, &lv_style_plain);
    style_preload.line.width = 10; /* 旋转部分线的宽度 */
//    style_preload.line.color = lv_color_hex3(0x258); /* 旋转部分线的颜色 */
//    style_preload.body.border.color = lv_color_hex3(0xBBB); /* 背景的颜色 */
    style_preload.line.color = LV_COLOR_YELLOW; /* 旋转部分线的颜色 */
    style_preload.body.border.color = LV_COLOR_BLUE; /* 背景的颜色 */
    style_preload.body.border.width = 10; /* 背景的圆圈的宽度 */
    style_preload.body.padding.left = 0; /* 左边部分填充为 0 */
    
    lv_style_copy(&style_led, &lv_style_pretty_color);
    style_led.body.shadow.width = 5;
    style_led.body.radius = LV_RADIUS_CIRCLE;
    style_led.body.border.width = 3;
    style_led.body.border.opa = LV_OPA_30;
    style_led.body.main_color = lv_color_hsv_to_rgb(210, 100, 100);
    style_led.body.grad_color = lv_color_hsv_to_rgb(210, 100, 100);
    style_led.body.border.color = lv_color_hsv_to_rgb(210, 60, 60);
    style_led.body.shadow.color = lv_color_hsv_to_rgb(210, 100, 100);
    
    lv_style_copy(&style_bar_bg, &lv_style_plain);
//    style_bar_bg.body.shadow.width = 0;
//    style_bar_bg.body.border.width = 0;
//    style_bar_bg.body.padding.top = 0;
//    style_bar_bg.body.padding.bottom = 0;
    
    lv_style_copy(&style_bar_inc, &lv_style_plain_color);
    style_bar_inc.body.shadow.width = 0;
    style_bar_inc.body.border.width = 0;
    style_bar_inc.body.padding.top = 0;
    style_bar_inc.body.padding.bottom = 0;
    style_bar_inc.body.padding.left = 0;
    style_bar_inc.body.padding.right = 0;
    
    lv_obj_t *cont = lv_cont_create(parent,NULL);
    lv_cont_set_style(cont, LV_CONT_STYLE_MAIN, &style_bg);
    lv_obj_set_size(cont,LV_HOR_RES_MAX,LV_VER_RES_MAX);
    lv_obj_set_event_cb(cont, ui_monitor_event_cb);
    
    label_const_status = lv_label_create(cont, NULL);
    lv_obj_set_style(label_const_status, &style_label_const);
    lv_label_set_text(label_const_status, "Status:");
    lv_obj_align(label_const_status, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0);
    
    label_status = lv_label_create(cont, NULL);
    lv_obj_set_style(label_status, &style_label);
    lv_label_set_text(label_status, "STOP");
    lv_obj_align(label_status, NULL, LV_ALIGN_IN_TOP_LEFT, 100, 0);
    
    label_const_speed = lv_label_create(cont, NULL);
    lv_obj_set_style(label_const_speed, &style_label_const);
    lv_label_set_text(label_const_speed, "Speed:");
    lv_obj_align(label_const_speed, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 23);
    
    label_speed = lv_label_create(cont, NULL);
    lv_obj_set_style(label_speed, &style_label);
    lv_label_set_text(label_speed, "99999 RPM");
    lv_obj_align(label_speed, NULL, LV_ALIGN_IN_TOP_LEFT, 100, 23);
    
    label_const_current = lv_label_create(cont, NULL);
    lv_obj_set_style(label_const_current, &style_label_const);
    lv_label_set_text(label_const_current, "Current:");
    lv_obj_align(label_const_current, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 46);
    
    label_current = lv_label_create(cont, NULL);
    lv_obj_set_style(label_current, &style_label);
    lv_label_set_text(label_current, "12.5 A");
    lv_obj_align(label_current, NULL, LV_ALIGN_IN_TOP_LEFT, 100, 46);
    
    label_const_voltage = lv_label_create(cont, NULL);
    lv_obj_set_style(label_const_voltage, &style_label_const);
    lv_label_set_text(label_const_voltage, "Voltage:");
    lv_obj_align(label_const_voltage, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 69);
    
    label_voltage = lv_label_create(cont, NULL);
    lv_obj_set_style(label_voltage, &style_label);
    lv_label_set_text(label_voltage, "12.5 V");
    lv_obj_align(label_voltage, NULL, LV_ALIGN_IN_TOP_LEFT, 100, 69);
    
    label_const_temp = lv_label_create(cont, NULL);
    lv_obj_set_style(label_const_temp, &style_label_const);
    lv_label_set_text(label_const_temp, "Temp:");
    lv_obj_align(label_const_temp, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 92);
    
    label_temp = lv_label_create(cont, NULL);
    lv_obj_set_style(label_temp, &style_label);
    lv_label_set_text(label_temp, "105 degC");
    lv_obj_align(label_temp, NULL, LV_ALIGN_IN_TOP_LEFT, 100, 92);
    
//    lv_preload = lv_preload_create(cont, NULL);
//    lv_obj_set_style(lv_preload, &style_preload);
//    lv_obj_set_size(lv_preload, 50, 50);
//    lv_obj_align(lv_preload, NULL, LV_ALIGN_IN_TOP_RIGHT, -10, 50);
    
    lv_led = lv_led_create(cont, NULL);
    lv_obj_set_style(lv_led, &style_led);
    lv_obj_set_size(lv_led, 22, 22);
    lv_obj_align(lv_led, NULL, LV_ALIGN_IN_TOP_RIGHT, -2, 2);
    
    lv_bar_ref_spd = lv_bar_create(cont, NULL);
    lv_bar_set_style(lv_bar_ref_spd,LV_BAR_STYLE_BG,&style_bar_bg);
    lv_bar_set_style(lv_bar_ref_spd,LV_BAR_STYLE_INDIC,&style_bar_inc);
    lv_obj_set_size(lv_bar_ref_spd,LV_HOR_RES_MAX,16);
    lv_obj_align(lv_bar_ref_spd, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 0, 0);
    lv_bar_set_range(lv_bar_ref_spd,0,100);
    lv_bar_set_value(lv_bar_ref_spd,0,LV_ANIM_OFF);
    
    label_ref_spd = lv_label_create(lv_bar_ref_spd,NULL);
    lv_label_set_text(label_ref_spd,"0RPM");
    
    
    return cont;
}

/**
  * @brief monitor data update
  * @param parent - parent object
  * @retval	current object
  * @note
  */
void ui_monitor_update(void)
{
    motor_control_t motor_info;
    motor_get_data(&motor_info);
    
    if(motor_info.state == RUN)
    {
        lv_label_set_text(label_status, "RUN");
        
        ui_label_show_int_num(label_speed,motor_info.speed_rpm,5," RPM",4);
        ui_label_show_float_num(label_current,pmsm_foc_param.iq,2," A",2);
    }
    else if(motor_info.state == FAULT)
    {
        lv_label_set_text(label_status, "FAULT");
        lv_label_set_text(label_speed, " RPM");
        lv_label_set_text(label_current, " A");
    }
    else
    {
        lv_label_set_text(label_status, "STOP");
        lv_label_set_text(label_speed, " RPM");
        lv_label_set_text(label_current, " A");
    }
//        uint16_t speed_rpm;
//    uint16_t speed_input;
    lv_bar_set_value(lv_bar_ref_spd,((motor_info.speed_input * 100) / motor_normal_spd),LV_ANIM_OFF);
    ui_label_show_int_num(label_ref_spd,motor_info.speed_input,5," RPM",4);
    
    lv_led_toggle(lv_led);
    
    ui_label_show_float_num(label_voltage,motor_info.vdc,2," V",2);
    ui_label_show_float_num(label_temp,motor_info.mosfet_temp,3," degC",5);
//    ui_label_show_int_num(label_temp,(int32_t)motor_info.mosfet_temp,3," degC",5);
}


/*******************************************************************************
*  Local Functions Body
*******************************************************************************/
/**
  * @brief object event callback
  * @param void
  * @retval	void
  * @note
  */
static void ui_monitor_event_cb(lv_obj_t* obj, lv_event_t event)
{
    motor_control_t motor_info;
    motor_get_data(&motor_info);
    
    if(event == LV_EVENT_KEY)
    {
        ui_event_t *e = (ui_event_t *)lv_event_get_data();
        switch (*e)
        {
        case KEY_EVENT_ENC0_INC:
            if(motor_info.speed_input < motor_normal_spd)
            {
                motor_set_speed(motor_info.speed_input + 100);
            }
            break;
        case KEY_EVENT_ENC0_DEC:
            if(motor_info.speed_input > 0)
            {
                if(motor_info.speed_input > (open_loop_speed + 100))
                    motor_set_speed(motor_info.speed_input - 100);
                else
                    motor_set_speed(open_loop_speed);
            }
            break;
        case KEY_EVENT_KEY0_REL:
            motor_get_data(&motor_info);
            if(motor_info.state == RUN)
            {
                motor_stop();
            }
            else if((motor_info.state == STOP) || (motor_info.state == FAULT))
            {
                motor_start();
            }
            break;
        case KEY_EVENT_KEY0_LONG_PRE:
            mainwindow_cutover_page();
            break;
        default:
            break;
        }
    }
    else if(event == LV_EVENT_REFRESH)
    {
        ui_monitor_update();
    }
}

/**
  * @brief Power operation
  * @param m - base
           n - exponent
  * @retval	void
  * @note
  */
uint32_t my_pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;
	return result;
}

/**
  * @brief label show int num
  * @param label - label object
           num - num to show
           len - Display length
           unit_text - unit text
           unit_len - unit len
  * @retval	void
  * @note
  */
static void ui_label_show_int_num(lv_obj_t *label,int32_t num,uint8_t len,const char * unit_text,uint8_t unit_len)
{
    uint8_t label_array[32] = {0};
    uint8_t text_index = 0;
    uint8_t header = 0;
    uint32_t u32_num = 0;
    uint8_t t,temp;
    uint8_t i = 0;
    
    if(num == 0)
    {
        label_array[text_index++] = 0x30;
    }
    else
    {
        if(num > 0)
        {
            u32_num = num;
        }
        else
        {
            u32_num = -num;
            label_array[text_index++] = '-';
        }
        for(t=0;t<len;t++)
        {
            temp = (u32_num/my_pow(10,len-t-1))%10;
            if((temp > 0) || (header != 0))
            {
                header = 1;
                label_array[text_index++] = temp + 0x30;
            }
        }
    }
    
    while(unit_len)
    {
        label_array[text_index++] = *unit_text++;
        unit_len--;
    }
    lv_label_set_array_text(label, (const char*)label_array, text_index);
}

/**
  * @brief label show float num
  * @param label - label object
           num - num to show
           len - Display length
           unit_text - unit text
           unit_len - unit len
  * @retval	void
  * @note
  */
static void ui_label_show_float_num(lv_obj_t *label,float num,uint8_t len,const char * unit_text,uint8_t unit_len)
{
    uint8_t label_array[32] = {0};
    uint8_t text_index = 0;
    uint8_t header = 0;
    uint32_t u32_num = 0;
    uint8_t decimals = 0;
    uint8_t t,temp;
    uint8_t i = 0;
    
    if((num >= -0.05f) && (num <= 0.05f))
    {
        label_array[text_index++] = 0x30;
    }
    else
    {
        if(num > 0)
        {
            u32_num = (uint32_t)num;
        }
        else
        {
            u32_num = -(uint32_t)num;
            label_array[text_index++] = '-';
        }
        for(t=0;t<len;t++)
        {
            temp = (u32_num/my_pow(10,len-t-1))%10;
            if((temp > 0) || (header != 0))
            {
                header = 1;
                label_array[text_index++] = temp + 0x30;
            }
        }
        if(header == 0)
        {
            label_array[text_index++] = 0x30;
        }
    }
    decimals = (uint32_t)(num * 10.0f) % 10;
    label_array[text_index++] = '.';
    label_array[text_index++] = decimals + 0x30;
    
    while(unit_len)
    {
        label_array[text_index++] = *unit_text++;
        unit_len--;
    }
    lv_label_set_array_text(label, (const char*)label_array, text_index);
}

/***************************************** (END OF FILE) *********************************************/
