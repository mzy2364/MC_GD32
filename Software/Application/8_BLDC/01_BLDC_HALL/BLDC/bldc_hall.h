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
    __IO uint8_t    run_flag;       /* ���б�־ */
    __IO uint8_t    locked_rotor;   /* ��ת��� */
    __IO uint8_t    step_sta;       /* ���λ���״̬ */
    __IO uint8_t    hall_sta_edge;  /* ��������״̬���� */
    __IO uint8_t    step_last;      /* �ϴλ���״̬ */
    __IO uint8_t    dir;            /* �����ת���� */
    __IO int32_t    pos;            /* ���λ�� */
    __IO int32_t    target_speed;   /* ����ת�� */
    __IO int32_t    speed;          /* ����ٶ� */
    __IO int16_t    current;        /* ������� */
    __IO uint16_t   pwm_duty;       /* ���ռ�ձ� */
    __IO uint32_t   hall_keep_t;    /* ��������ʱ�� */
    __IO uint32_t   hall_pul_num;   /* ���������������� */
    __IO uint32_t   lock_time;      /* �����תʱ�� */
    __IO uint32_t   no_single;
    __IO uint32_t   count_j;
    pid_control_t   speed_pi;
} _bldc_obj;

/* DEFINES ------------------------------------------------------------------------------------------*/
#define SPEED_LOOP                  1

#define MAX_PWM_DUTY                ((float)MOTOR_PWM_PERIOD*0.96)        /* ���ռ�ձ����� */
#define MIN_PWM_DUTY                ((float)MOTOR_PWM_PERIOD*0.1)        /* ���ռ�ձ����� */

#define     WKP        (0.5)
#define     WKI        (0.0001)
#define     WKC        (0.5)
#define     WOUTMAX    MAX_PWM_DUTY

typedef void(*pctr) (void);

#define CCW                         (1)                 /* ��ʱ�� */
#define CW                          (2)                 /* ˳ʱ�� */
#define HALL_ERROR                  (0xF0)              /* ���������־ */
#define RUN                         (1)                 /* ����˶���־ */
#define STOP                        (0)                 /* ���ͣ����־ */

#define FirstOrderRC_LPF(Yn_1,Xn,a) Yn_1 = (1-a)*Yn_1 + a*Xn;   /* Yn:out;Xn:in;a:ϵ�� */

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
