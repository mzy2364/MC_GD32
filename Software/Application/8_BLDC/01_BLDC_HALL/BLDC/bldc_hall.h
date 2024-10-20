/*
*******************************************************************************************************
*
* File Name : bldc_hall.h
* Version   : V1.0
* Author    : mzy2364
* brief     : bldc hall driver
* 
*******************************************************************************************************
*/
#ifndef _BLDC_HALL_H_
#define _BLDC_HALL_H_

#ifdef __cplusplus
extern "C"{
#endif

/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "gd32f30x.h"
#include "pid.h"

/* TYPEDEF ------------------------------------------------------------------------------------------*/
typedef struct {
    __IO uint8_t    run_flag;       /* 运行标志 */
    __IO uint8_t    locked_rotor;   /* 堵转标记 */
    __IO uint8_t    step_sta;       /* 本次霍尔状态 */
    __IO uint8_t    hall_sta_edge;  /* 单个霍尔状态跳变 */
    __IO uint8_t    step_last;      /* 上次霍尔状态 */
    __IO uint8_t    dir;            /* 电机旋转方向 */
    __IO int32_t    pos;            /* 电机位置 */
    __IO int32_t    target_speed;   /* 期望转速 */
    __IO int32_t    speed;          /* 电机速度 */
    __IO int16_t    current;        /* 电机电流 */
    __IO uint16_t   pwm_duty;       /* 电机占空比 */
    __IO uint32_t   hall_keep_t;    /* 霍尔保持时间 */
    __IO uint32_t   hall_pul_num;   /* 霍尔传感器脉冲数 */
    __IO uint32_t   lock_time;      /* 电机堵转时间 */
    __IO uint32_t   no_single;
    __IO uint32_t   count_j;
    pid_control_t   speed_pi;
} _bldc_obj;

/* DEFINES ------------------------------------------------------------------------------------------*/
#define SPEED_LOOP                  1

#define MAX_PWM_DUTY                ((float)MOTOR_PWM_PERIOD*0.96)        /* 最大占空比限制 */
#define MIN_PWM_DUTY                ((float)MOTOR_PWM_PERIOD*0.1)        /* 最大占空比限制 */

#define     WKP        (0.5)
#define     WKI        (0.0001)
#define     WKC        (0.5)
#define     WOUTMAX    MAX_PWM_DUTY

typedef void(*pctr) (void);

#define CCW                         (1)                 /* 逆时针 */
#define CW                          (2)                 /* 顺时针 */
#define HALL_ERROR                  (0xF0)              /* 霍尔错误标志 */
#define RUN                         (1)                 /* 电机运动标志 */
#define STOP                        (0)                 /* 电机停机标志 */

#define FirstOrderRC_LPF(Yn_1,Xn,a) Yn_1 = (1-a)*Yn_1 + a*Xn;   /* Yn:out;Xn:in;a:系数 */

#define MOTOR_NOPOLESPAIRS          2
#define MAX_SPEED                   5000
#define MIN_SPEED                   100

/* VARIABLES ----------------------------------------------------------------------------------------*/
extern _bldc_obj g_bldc_motor;

/* FUNCTION -----------------------------------------------------------------------------------------*/
void bldc_init(void);
void bldc_ctrl(int32_t dir,float duty);
void stop_motor(void);
void start_motor(void);
void bldc_mainfunction(void);

#ifdef __cplusplus
extern "C"{
#endif

#endif /* _BLDC_HALL_H_ */

/***************************************** (END OF FILE) *********************************************/
