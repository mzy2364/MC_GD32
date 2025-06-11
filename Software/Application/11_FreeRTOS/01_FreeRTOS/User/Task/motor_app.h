/*
*******************************************************************************************************
*
* File Name : motor_app.h
* Version   : V1.0
* Author    : mzy2364
* brief     : motor control main file
* 
*******************************************************************************************************
*/
#ifndef _MOTOR_APP_H_
#define _MOTOR_APP_H_

#ifdef __cplusplus
extern "C"{
#endif

/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "motor_hardware.h"
#include "motor_config.h"

/* DEFINES ------------------------------------------------------------------------------------------*/
#define START_DELAY_SEC     (float)2
#define START_DELAY_TICK    (uint32_t)((float)START_DELAY_SEC/(1/(float)MOTOR_PWM_FREQ_HZ))

#define ADC_CAL_COUNT       1024

typedef enum{
    INIT = 0,
    ADC_CAL,
    START_DELAY,
    STOP,
    RUN,
    FAULT,
}motor_state_t;

typedef struct{
    uint16_t adc_ia;
    uint16_t adc_ib;
    uint16_t adc_ic;
    uint16_t ia_offset;
    uint16_t ib_offset;
    uint16_t ic_offset;
    uint32_t adc_cal_cnt;
    
    uint32_t start_delay_tick;
    
    uint16_t ntc_adc;
    uint16_t vdc_adc;
    uint16_t idc_adc;
    float mosfet_temp;
    float vdc;
    float idc;
    
    uint16_t speed_rpm;
    
    motor_state_t state;
}motor_control_t;

/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/
void pmsm_init(void* pvParameters);

#ifdef __cplusplus
extern "C"{
#endif

#endif /* _MOTOR_APP_H_ */

/***************************************** (END OF FILE) *********************************************/
