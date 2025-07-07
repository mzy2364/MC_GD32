/*
*******************************************************************************************************
*
* File Name : ee_parameter.h
* Version   : V1.0
* Author    : mzy2364
* brief     : EEPROM Config parameter
* 
*******************************************************************************************************
*/
#ifndef _EE_PARAMETER_H_
#define _EE_PARAMETER_H_

#ifdef __cplusplus
extern "C"{
#endif

/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "eeprom.h"

/* DEFINES ------------------------------------------------------------------------------------------*/
#define EEPROM_INIT_MASK        0xAA551248

#define EEPROM_PARAMETER_ADDR   0
#define EEPROM_PARAMETER_LEN    64

#define MOTOR_R_ADDR            0
#define MOTOR_R_LEN             2
#define MOTOR_L_ADDR            2
#define MOTOR_L_LEN             2
#define MOTOR_BF_ADDR           4
#define MOTOR_BF_LEN            2
#define MOTOR_PL_ADDR           6
#define MOTOR_PL_LEN            1
#define MOTOR_NML_SPD_ADDR      7
#define MOTOR_NML_SPD_LEN       2
#define OL_AMP_ADDR             9
#define OL_AMP_LEN              1
#define ROTOR_LOCK_TIME_ADDR    10
#define ROTOR_LOCK_TIME_LEN     1
#define OL_RAMP_TIME_ADDR       11
#define OL_RAMP_TIME_LEN        2
#define OL_HOLD_TIME_ADDR       13
#define OL_HOLD_TIME_LEN        2
#define OL_SPEED_ADDR           15
#define OL_SPEED_LEN            2
#define MOTOR_SENSOR_ADDR       17
#define MOTOR_SENSOR_LEN        1

#define EEPROM_MASK_ADDR        60
#define EEPROM_MASK_LEN         4


/* VARIABLES ----------------------------------------------------------------------------------------*/
extern uint16_t motor_r;     /* unit mR */
extern uint16_t motor_l;  /* unit uH */
extern uint16_t motor_bemf;    /* unit mV/KRPM */
extern uint8_t motor_pole;
extern uint16_t motor_normal_spd;
extern uint8_t openloop_current;      /* unit 0.1A */
extern uint8_t rotor_lock_time;           /* unit 0.1s */
extern uint16_t openloop_ramp_time;   /* unit 0.1s */
extern uint16_t openloop_hold_time;   /* unit 0.1s */
extern uint16_t open_loop_speed;     /* unit RPM */
extern uint8_t motor_sensor;

/* FUNCTION -----------------------------------------------------------------------------------------*/
void eeprom_load_parameter(void);
void eeprom_save_parameter(void);

#ifdef __cplusplus
extern "C"{
#endif

#endif /* _EE_PARAMETER_H_ */

/***************************************** (END OF FILE) *********************************************/
