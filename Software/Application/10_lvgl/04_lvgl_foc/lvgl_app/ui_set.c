/*
 * ui_set.c
 *
 *  Created on: 2025-3-17
 *      Author: mzy2364
 */

/*******************************************************************************
* include files
*******************************************************************************/
#include "mainwindow.h"
#include <stdio.h>
#include "ee_parameter.h"

/*******************************************************************************
* Defines
*******************************************************************************/
#define SET_ITEM_NUM    11
/*******************************************************************************
* Macros
*******************************************************************************/
typedef enum{
	KNOB_SELECT = 0,		//编码器用于选择
	KNOB_SET,				//编码器用于设置
}knob_status_t;

/*******************************************************************************
* Global Constant definition
*******************************************************************************/

/*******************************************************************************
* Local Constant definition
*******************************************************************************/

/*******************************************************************************
* Global Variables definition
*******************************************************************************/
extern uint16_t motor_r;
extern uint16_t motor_l;
extern uint16_t motor_bemf;
extern uint8_t motor_pole;
extern uint16_t motor_normal_spd;
extern uint8_t openloop_current;
extern uint8_t rotor_lock_time;
extern uint16_t openloop_ramp_time;
extern uint16_t openloop_hold_time;
extern uint16_t open_loop_speed;
extern uint8_t motor_sensor;

extern lv_font_t gbk_puhui16;

/*******************************************************************************
* Local Variables definition
*******************************************************************************/
static lv_obj_t *label_motor_r;
static lv_obj_t *label_motor_l;
static lv_obj_t *label_motor_bemf;
static lv_obj_t *label_motor_pole;
static lv_obj_t *label_motor_rate_spd;
static lv_obj_t *label_openloop_current;
static lv_obj_t *label_rotor_lock_time;
static lv_obj_t *label_openloop_ramp_time;
static lv_obj_t *label_openloop_hold_time;
static lv_obj_t *label_open_loop_speed;
static lv_obj_t *label_motor_sensor;

static lv_obj_t *label_flash;

static knob_status_t knob_status = KNOB_SELECT;

uint16_t parameter_increment = 1;

uint8_t number_flash = 0;

uint8_t index_sel = 0;
lv_obj_t *label_tab[SET_ITEM_NUM];// = {label_motor_r,label_motor_l,label_motor_bemf,label_motor_pole,label_motor_rate_spd,spinbox_motor_rate_spd};

/*******************************************************************************
* Local Functions prototypes
*******************************************************************************/
static void ui_set_event_cb(lv_obj_t* obj, lv_event_t event);
static void my_task_cb(lv_task_t * task);

/*******************************************************************************
*  Global Functions Body
*******************************************************************************/

/**
  * @brief monitor create
  * @param parent - parent object
  * @retval	current object
  * @note
  */
lv_obj_t *ui_set_create(lv_obj_t *parent)
{
    static lv_style_t style_list;
    static lv_style_t style_list_btn_pre;
    static lv_style_t style_list_btn_tgl_pre;
    static lv_style_t style_list_btn_label;
    static lv_style_t style_flash;
    lv_style_copy(&style_list, &lv_style_pretty);
    lv_style_copy(&style_list_btn_label, &lv_style_pretty);
    lv_style_copy(&style_list_btn_pre, &lv_style_pretty_color);
    lv_style_copy(&style_list_btn_tgl_pre, &lv_style_pretty_color);
    //style_list.body.main_color = LV_COLOR_OLIVE;		/* 设置底色 */
    style_list.body.opa = LV_OPA_TRANSP;
    style_list.body.border.color = LV_COLOR_BLUE;

    style_list.text.font = &lv_font_roboto_22;
    style_list_btn_pre.text.font = &lv_font_roboto_22;
    style_list_btn_tgl_pre.text.font = &lv_font_roboto_22;
    style_list_btn_label.text.font = &gbk_puhui16;
    style_list_btn_label.text.color = LV_COLOR_BLACK;

    style_list_btn_pre.body.main_color = lv_color_hex(0x9fc8ef);
    style_list_btn_pre.body.grad_color = lv_color_hex(0x9fc8ef);
    style_list_btn_pre.text.color = LV_COLOR_BLACK;

    style_list_btn_tgl_pre.body.main_color = lv_color_hex(0x4169E1);
    style_list_btn_tgl_pre.body.grad_color = lv_color_hex(0x4169E1);
    style_list_btn_tgl_pre.text.color = LV_COLOR_BLACK;

    lv_obj_t* list = lv_list_create(parent, NULL);				/* 创建list控件 */
    lv_obj_set_size(list, LV_HOR_RES_MAX, LV_VER_RES_MAX);						/* 设置尺寸 */
    lv_obj_align(list, NULL, LV_ALIGN_IN_TOP_MID, 0, 0);	/* 设置位置 */
    lv_list_set_edge_flash(list, true);						/* 当滑动到顶部或者底部的时候,有个圆圈的效果 */
    lv_list_set_single_mode(list,true);
    lv_obj_set_event_cb(list, ui_set_event_cb);

    lv_list_set_style(list, LV_LIST_STYLE_SCRL, &lv_style_transp);
    lv_list_set_style(list, LV_LIST_STYLE_BG, &lv_style_scr);
    lv_list_set_style(list, LV_LIST_STYLE_BTN_PR, &style_list_btn_pre);
    lv_list_set_style(list, LV_LIST_STYLE_BTN_REL, &style_list);
    lv_list_set_style(list, LV_LIST_STYLE_BTN_INA, &style_list_btn_pre);

    lv_list_set_style(list, LV_LIST_STYLE_BTN_TGL_REL, &style_list);
    lv_list_set_style(list, LV_LIST_STYLE_BTN_TGL_PR, &style_list_btn_tgl_pre);

    lv_obj_t* list_btn;

    list_btn = lv_list_add_btn(list, NULL, "RES(mR)");
    lv_btn_set_layout(list_btn, LV_LAYOUT_OFF);			/* 重新设置一下btn的layout使得后面的对齐可用 */
    label_motor_r = lv_label_create(list_btn,NULL);
    lv_label_set_style(label_motor_r,LV_LABEL_STYLE_MAIN,&style_list_btn_label);
    lv_label_set_text(label_motor_r,"Manual");
    lv_obj_align(label_motor_r, list_btn, LV_ALIGN_IN_RIGHT_MID, -12, 0);
    
    label_flash = lv_cont_create(list_btn,NULL);
    lv_style_copy(&style_flash, lv_cont_get_style(label_flash,LV_CONT_STYLE_MAIN));
    style_flash.body.radius = 0;
    style_flash.body.border.width = 0;
    style_flash.body.opa = 255;
    style_flash.body.grad_color = lv_color_hex(0x4169E1);
    style_flash.body.main_color = lv_color_hex(0x4169E1);
    style_flash.body.padding.top = 0;
    style_flash.body.padding.bottom = 0;
    style_flash.body.padding.inner = 0;
    style_flash.body.padding.left = 0;
    style_flash.body.padding.right = 0;
    lv_cont_set_style(label_flash,LV_CONT_STYLE_MAIN,&style_flash);
    lv_obj_set_size(label_flash,11,lv_obj_get_height(label_motor_r));
    lv_obj_align(label_flash, label_motor_r, LV_ALIGN_IN_RIGHT_MID, 0, 0);
    lv_obj_set_hidden(label_flash,true);
    

    list_btn = lv_list_add_btn(list, NULL, "IND(uH)");
    lv_btn_set_layout(list_btn, LV_LAYOUT_OFF);			/* 重新设置一下btn的layout使得后面的对齐可用 */
    label_motor_l = lv_label_create(list_btn,NULL);
    lv_label_set_style(label_motor_l,LV_LABEL_STYLE_MAIN,&style_list_btn_label);
    lv_label_set_text(label_motor_l," ");
    lv_obj_align(label_motor_l, list_btn, LV_ALIGN_IN_RIGHT_MID, -12, 0);

    list_btn = lv_list_add_btn(list, NULL, "BEMF(mV)");
    lv_btn_set_layout(list_btn, LV_LAYOUT_OFF);			/* 重新设置一下btn的layout使得后面的对齐可用 */
    label_motor_bemf = lv_label_create(list_btn,NULL);
    lv_label_set_style(label_motor_bemf,LV_LABEL_STYLE_MAIN,&style_list_btn_label);
    lv_label_set_text(label_motor_bemf," ");
    lv_obj_align(label_motor_bemf, list_btn, LV_ALIGN_IN_RIGHT_MID, -12, 0);

    list_btn = lv_list_add_btn(list, NULL, "POLE");
    lv_btn_set_layout(list_btn, LV_LAYOUT_OFF);			/* 重新设置一下btn的layout使得后面的对齐可用 */
    label_motor_pole = lv_label_create(list_btn,NULL);
    lv_label_set_style(label_motor_pole,LV_LABEL_STYLE_MAIN,&style_list_btn_label);
    lv_label_set_text(label_motor_pole," ");
    lv_obj_align(label_motor_pole, list_btn, LV_ALIGN_IN_RIGHT_MID, -12, 0);

    list_btn = lv_list_add_btn(list, NULL, "RateRPM");
    lv_btn_set_layout(list_btn, LV_LAYOUT_OFF);
    label_motor_rate_spd = lv_label_create(list_btn,NULL);
    lv_label_set_style(label_motor_rate_spd,LV_LABEL_STYLE_MAIN,&style_list_btn_label);
    lv_label_set_text(label_motor_rate_spd," ");
    lv_obj_align(label_motor_rate_spd, list_btn, LV_ALIGN_IN_RIGHT_MID, -12, 0);
    
    list_btn = lv_list_add_btn(list, NULL, "OLAMP");
    lv_btn_set_layout(list_btn, LV_LAYOUT_OFF);
    label_openloop_current = lv_label_create(list_btn,NULL);
    lv_label_set_style(label_openloop_current,LV_LABEL_STYLE_MAIN,&style_list_btn_label);
    lv_label_set_text(label_openloop_current," ");
    lv_obj_align(label_openloop_current, list_btn, LV_ALIGN_IN_RIGHT_MID, -12, 0);
    
    list_btn = lv_list_add_btn(list, NULL, "OL_LockT");
    lv_btn_set_layout(list_btn, LV_LAYOUT_OFF);
    label_rotor_lock_time = lv_label_create(list_btn,NULL);
    lv_label_set_style(label_rotor_lock_time,LV_LABEL_STYLE_MAIN,&style_list_btn_label);
    lv_label_set_text(label_rotor_lock_time," ");
    lv_obj_align(label_rotor_lock_time, list_btn, LV_ALIGN_IN_RIGHT_MID, -12, 0);
    
    list_btn = lv_list_add_btn(list, NULL, "OL_RampT");
    lv_btn_set_layout(list_btn, LV_LAYOUT_OFF);
    label_openloop_ramp_time = lv_label_create(list_btn,NULL);
    lv_label_set_style(label_openloop_ramp_time,LV_LABEL_STYLE_MAIN,&style_list_btn_label);
    lv_label_set_text(label_openloop_ramp_time," ");
    lv_obj_align(label_openloop_ramp_time, list_btn, LV_ALIGN_IN_RIGHT_MID, -12, 0);
    
    list_btn = lv_list_add_btn(list, NULL, "OL_HoldT");
    lv_btn_set_layout(list_btn, LV_LAYOUT_OFF);
    label_openloop_hold_time = lv_label_create(list_btn,NULL);
    lv_label_set_style(label_openloop_hold_time,LV_LABEL_STYLE_MAIN,&style_list_btn_label);
    lv_label_set_text(label_openloop_hold_time," ");
    lv_obj_align(label_openloop_hold_time, list_btn, LV_ALIGN_IN_RIGHT_MID, -12, 0);
    
    list_btn = lv_list_add_btn(list, NULL, "OL_SPD");
    lv_btn_set_layout(list_btn, LV_LAYOUT_OFF);
    label_open_loop_speed = lv_label_create(list_btn,NULL);
    lv_label_set_style(label_open_loop_speed,LV_LABEL_STYLE_MAIN,&style_list_btn_label);
    lv_label_set_text(label_open_loop_speed," ");
    lv_obj_align(label_open_loop_speed, list_btn, LV_ALIGN_IN_RIGHT_MID, -12, 0);

    list_btn = lv_list_add_btn(list, NULL, "SENSOR");
    lv_btn_set_layout(list_btn, LV_LAYOUT_OFF);
    label_motor_sensor = lv_label_create(list_btn,NULL);
    lv_label_set_style(label_motor_sensor,LV_LABEL_STYLE_MAIN,&style_list_btn_label);
    lv_label_set_text(label_motor_sensor," ");
    lv_obj_align(label_motor_sensor, list_btn, LV_ALIGN_IN_RIGHT_MID, -12, 0);

    lv_list_set_btn_selected(list, lv_list_get_next_btn(list, NULL));
    
    lv_task_t * task = lv_task_create(my_task_cb, 300, LV_TASK_PRIO_MID, NULL);

    label_tab[0] = label_motor_r;
    label_tab[1] = label_motor_l;
    label_tab[2] = label_motor_bemf;
    label_tab[3] = label_motor_pole;
    label_tab[4] = label_motor_rate_spd;
    label_tab[5] = label_openloop_current;
    label_tab[6] = label_rotor_lock_time;
    label_tab[7] = label_openloop_ramp_time;
    label_tab[8] = label_openloop_hold_time;
    label_tab[9] = label_open_loop_speed;
    label_tab[10] = label_motor_sensor;
    
    return list;
}



/*******************************************************************************
*  Local Functions Body
*******************************************************************************/

static uint32_t parameter_get_value(uint8_t index)
{
    switch(index)
    {
        case 0:
            return motor_r;
            break;
        case 1:
            return motor_l;
            break;
        case 2:
            return motor_bemf;
            break;
        case 3:
            return motor_pole;
            break;
        case 4:
            return motor_normal_spd;
            break;
        case 5:
            return openloop_current;
            break;
        case 6:
            return rotor_lock_time;
            break;
        case 7:
            return openloop_ramp_time;
            break;
        case 8:
            return openloop_hold_time;
            break;
        case 9:
            return open_loop_speed;
            break;
        case 10:
            return motor_sensor;
            break;
        default:
            return 0;
            break;
    }
}

static void parameter_set_value(uint8_t index,uint32_t value)
{
    switch(index)
    {
        case 0:
            motor_r = value;
            break;
        case 1:
            motor_l = value;
            break;
        case 2:
            motor_bemf = value;
            break;
        case 3:
            motor_pole = value;
            break;
        case 4:
            motor_normal_spd = value;
            break;
        case 5:
            openloop_current = value;
            break;
        case 6:
            rotor_lock_time = value;
            break;
        case 7:
            openloop_ramp_time = value;
            break;
        case 8:
            openloop_hold_time = value;
            break;
        case 9:
            open_loop_speed = value;
            break;
        case 10:
            motor_sensor = value;
            if(motor_sensor > 1)
                motor_sensor = 1;
            break;
        default:
            break;
    }
}

/**
  * @brief number flash task
  * @param void
  * @retval	void
  * @note
  */
static void my_task_cb(lv_task_t * task)
{
    char value_buf[16] = {0};
    int16_t y_pos = 0;
    uint8_t len = 0;
    
    if(knob_status == KNOB_SET)
    {
        if(parameter_increment == 1)
        {
            y_pos = 0;
        }
        else if(parameter_increment == 10)
        {
            y_pos = -11;
        }
        else if(parameter_increment == 100)
        {
            y_pos = -22;
        }
        else if(parameter_increment == 1000)
        {
            y_pos = -33;
        }
        
        if(number_flash)
        {
            lv_obj_set_hidden(label_flash,number_flash);
            lv_obj_align(label_flash, label_tab[index_sel], LV_ALIGN_IN_RIGHT_MID, y_pos, 0);
            lv_obj_realign(label_flash);
        }
        else
        {
            lv_obj_set_hidden(label_flash,number_flash);
            lv_obj_align(label_flash, label_tab[index_sel], LV_ALIGN_IN_RIGHT_MID, y_pos, 0);
            lv_obj_realign(label_flash);
        }
    }
    else
    {
        lv_obj_set_hidden(label_flash,true);
    }
    number_flash = 1 - number_flash;
}

/**
  * @brief object event callback
  * @param void
  * @retval	void
  * @note
  */
static void ui_set_event_cb(lv_obj_t* obj, lv_event_t event)
{
    lv_obj_t *sel_btn = NULL;
    lv_obj_t *next_btn = NULL;
    lv_obj_t *prev_btn = NULL;
    uint8_t i = 0;
    
    char value_buf[16] = {0};
    
    if(event == LV_EVENT_KEY)
    {
        ui_event_t *e = (ui_event_t *)lv_event_get_data();
        sel_btn = lv_list_get_btn_selected(obj);
        index_sel = lv_list_get_btn_index(obj,sel_btn);
        switch (*e)
        {
        case KEY_EVENT_ENC0_INC:
            if(knob_status == KNOB_SELECT)
            {
                next_btn = lv_list_get_next_btn(obj,sel_btn);
                if(next_btn)
                    lv_list_set_btn_selected(obj, next_btn);
            }
            else
            {
                parameter_set_value(index_sel,parameter_get_value(index_sel) + parameter_increment);
                lv_event_send(obj, LV_EVENT_REFRESH, NULL);
            }
            break;
        case KEY_EVENT_ENC0_DEC:
			if(knob_status == KNOB_SELECT)
			{
				prev_btn = lv_list_get_prev_btn(obj,sel_btn);
				if(prev_btn)
					lv_list_set_btn_selected(obj, prev_btn);
			}
			else
            {
                parameter_set_value(index_sel,parameter_get_value(index_sel) - parameter_increment);
                lv_event_send(obj, LV_EVENT_REFRESH, NULL);
            }
            break;
        case KEY_EVENT_KEY0_REL:
            if(knob_status == KNOB_SELECT)
            {
                knob_status = KNOB_SET;
                lv_btn_set_state(sel_btn,LV_BTN_STATE_TGL_PR);
                
                lv_obj_set_parent(label_flash,label_tab[index_sel]);
                parameter_increment = 1;
            }
            else
            {
                if(parameter_increment < 1000)
                {
                    parameter_increment = parameter_increment * 10;
                }
                else
                {
                    parameter_increment = 1;
                }
            }
            break;
        case KEY_EVENT_KEY0_LONG_PRE:
            if(knob_status == KNOB_SELECT)
            {
                eeprom_save_parameter();
                mainwindow_cutover_page();
            }
            else
            {
                /* 确认 保存 */
                lv_obj_set_hidden(label_flash,true);
                knob_status = KNOB_SELECT;
                lv_btn_set_state(sel_btn,LV_BTN_STATE_PR);
            }
            break;
        default:
            break;
        }
    }
    else if(event == LV_EVENT_FOCUSED)
    {
        motor_stop();
        lv_list_set_btn_selected(obj, lv_list_get_next_btn(obj, NULL));
    }
    else if(event == LV_EVENT_REFRESH)
    {
        for(i=0;i<SET_ITEM_NUM;i++)
        {
            memset(value_buf,0,sizeof(value_buf));
            sprintf(value_buf,"%05d",parameter_get_value(i));
            lv_label_set_text(label_tab[i], value_buf);
            lv_obj_realign(label_tab[i]);
        }
        
    }
}

/***************************************** (END OF FILE) *********************************************/
