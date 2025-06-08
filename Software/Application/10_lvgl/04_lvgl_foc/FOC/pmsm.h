/*
*******************************************************************************************************
*
* File Name : pmsm.h
* Version   : V1.0
* Author    : mzy2364
* brief     : pmsm driver file
* 
*******************************************************************************************************
*/
#ifndef _PMSM_H_
#define _PMSM_H_

#ifdef __cplusplus
extern "C"{
#endif

/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include <stdint.h>
#include "foc.h"
#include "pll_estimator.h"
/* DEFINES ------------------------------------------------------------------------------------------*/
typedef enum{
    ROTOR_LOCK = 0,
    RAMP_SPEED,
    OL_HOLD,
    WAIT_CLOSELOOP
}open_loop_state;

typedef struct{
	uint8_t openloop;           /* 0-closeloop  1-openloop */
	uint8_t run_motor;          /* 0-stop   1-stop */
	uint8_t change_mode;
	
    uint16_t lock_counter;      /* 转子锁定时间 */
    uint16_t hold_counter;      /* 开环保持时间 */
	/* Rotor positioning time */
	uint16_t startup_lock_count;    /* 转子锁定计数器 */
	uint16_t openloop_hold_count;   /* 开环保持计数器 */
	/* Start up ramp in open loop */
	float startup_ramp;         /* 开环速度斜坡 */
    
    float max_phase_voltage;
    
    float vel_input;
    float vel_ref;
    float vel_diff;
    float actual_speed;
    
    float id_ref;
    float iq_ref;
    
    float hall_speed;
    
    float open_loop_current;
    
    float end_speed_rads_elec_counter;
    float openloop_rampspeed_inc;
    
    uint32_t speed_loop_count;
    
	float end_speed;
    
    open_loop_state ol_state;
	
}pmsm_mc_t;



/* VARIABLES ----------------------------------------------------------------------------------------*/
extern pmsm_foc_t pmsm_foc_param;
extern pmsm_mc_t pmsm_mc_param;
extern mcParam_PLLEstimator mcApp_EstimParam;;

/* FUNCTION -----------------------------------------------------------------------------------------*/
void pmsm_foc_init(void);
void pmsm_foc_run(void);


#ifdef __cplusplus
extern "C"{
#endif

#endif /* _PMSM_H_ */

/***************************************** (END OF FILE) *********************************************/
