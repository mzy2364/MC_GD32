/*
 * mainwindow.c
 *
 *  Created on: 2025-3-17
 *      Author: mzy2364
 */

/*******************************************************************************
* include files
*******************************************************************************/
#include "mainwindow.h"

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

/*******************************************************************************
* Local Variables definition
*******************************************************************************/
lv_obj_t *current_obj;
lv_obj_t *ui_obj_monitor;
lv_obj_t *ui_obj_set;

lv_obj_t *screen_tab[SCREEN_NUM] = {0};

uint8_t current_page = 0;

/*******************************************************************************
* Local Functions prototypes
*******************************************************************************/
static void mainwindow_event_cb(lv_obj_t* obj, lv_event_t event);

/*******************************************************************************
*  Global Functions Body
*******************************************************************************/

/**
  * @brief mainwindow create
  * @param void
  * @retval	void
  * @note
  */
void lvgl_main_app(void)
{
    ui_obj_monitor = ui_monitor_create(lv_scr_act());
    ui_obj_set = ui_set_create(lv_scr_act());
    
    screen_tab[0] = ui_obj_monitor;
    screen_tab[1] = ui_obj_set;

    lv_event_send(ui_obj_monitor, LV_EVENT_REFRESH, NULL);
    lv_event_send(ui_obj_set, LV_EVENT_REFRESH, NULL);

    lv_obj_set_event_cb(lv_scr_act(), mainwindow_event_cb);

    lv_obj_move_foreground(ui_obj_monitor);
    current_obj = ui_obj_monitor;
    current_page = 0;
    return;
}

/**
  * @brief switch the top page
  * @param void
  * @retval	void
  * @note
  */
void mainwindow_cutover_page(void)
{
    lv_event_send(current_obj, LV_EVENT_DEFOCUSED, NULL);
    current_page++;
    if(current_page >= SCREEN_NUM)
    {
        current_page = 0;
    }
    current_obj = screen_tab[current_page];
    lv_event_send(current_obj, LV_EVENT_REFRESH, NULL);
    lv_event_send(current_obj, LV_EVENT_FOCUSED, NULL);
    lv_obj_move_foreground(current_obj);
}

/**
  * @brief send event to ui
  * @param e - event
  * @retval	void
  * @note
  */
void ui_event_send(ui_event_t e)
{
    static ui_event_t ev = KEY_EVENT_NONE;
    ev = e;
    if(lv_scr_act())
        lv_event_send(lv_scr_act(), LV_EVENT_KEY, &ev);
}

/**
  * @brief ui refresh and updata
  * @param void
  * @retval	void
  * @note
  */
void ui_refresh(void)
{
    if(current_obj)
        lv_event_send(current_obj, LV_EVENT_REFRESH, NULL);
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
static void mainwindow_event_cb(lv_obj_t* obj, lv_event_t event)
{
    if(current_obj)
        lv_event_send(current_obj, LV_EVENT_KEY, lv_event_get_data());
}

/***************************************** (END OF FILE) *********************************************/
