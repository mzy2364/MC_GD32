/*
*******************************************************************************************************
*
* File Name : single_shunt_cs.h
* Version   : V1.0
* Author    : mzy2364
* brief     : single_shunt_cs header file
* 
*******************************************************************************************************
*/
#ifndef _SINGLE_SHUNT_CS_H_
#define _SINGLE_SHUNT_CS_H_

#ifdef __cplusplus
extern "C"{
#endif

/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "gd32f30x.h"
#include "motor_config.h"


/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/
extern int32_t duty_u_offset;
extern int32_t duty_v_offset;
extern int32_t duty_w_offset;
extern uint16_t trigger_point1;
extern uint16_t trigger_point2;
extern int32_t duty_u;
extern int32_t duty_v;
extern int32_t duty_w;

/* FUNCTION -----------------------------------------------------------------------------------------*/
void motor_phase_current_sampling_init(void);
void motor_phase_current_sampling_shift(uint8_t sector);
void motor_get_phase_current(uint8_t sector);
void motor_get_phase_current_zero(void);

#ifdef __cplusplus
extern "C"{
#endif

#endif /* _SINGLE_SHUNT_CS_H_ */

/***************************************** (END OF FILE) *********************************************/
