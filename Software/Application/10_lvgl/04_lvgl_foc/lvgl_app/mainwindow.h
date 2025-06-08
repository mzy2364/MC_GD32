/*
 * mainwindow.h
 *
 *  Created on: 2025-3-17
 *      Author: mzy2364
 */

#ifndef _MAINDOW_H_
#define _MAINDOW_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "lvgl.h"
#include "motor_app.h"
#include "pmsm.h"

#define SCREEN_NUM      2

typedef enum{
    KEY_EVENT_NONE = 0,
    KEY_EVENT_KEY0_PRE,
    KEY_EVENT_KEY0_LONG_PRE,
    KEY_EVENT_KEY0_REL,

    KEY_EVENT_ENC0_INC,
    KEY_EVENT_ENC0_DEC,
}ui_event_t;


void lvgl_main_app(void);
void ui_event_send(ui_event_t e);
void mainwindow_cutover_page(void);
void ui_monitor_update(void);
void ui_refresh(void);

lv_obj_t *ui_monitor_create(lv_obj_t *parent);
lv_obj_t *ui_set_create(lv_obj_t *parent);
lv_obj_t *ui_program_create(lv_obj_t *parent);

#ifdef __cplusplus
}
#endif

#endif /* _MAINDOW_H_ */
