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
#include "luenberger.h"
/* DEFINES ------------------------------------------------------------------------------------------*/
typedef struct{
	uint8_t openloop;
	uint8_t run_motor;
	uint8_t change_mode;
	
	/* Rotor positioning time */
	uint16_t startup_lock_count;
	/* Start up ramp in open loop */
	float startup_ramp;
    
    float vel_input;
    float vel_ref;
    float vel_diff;
    float actual_speed;
    
    float id_ref;
    float iq_ref;
    
    float hall_speed;
    
    float max_phase_voltage;
    
    uint32_t speed_loop_count;
    
	int32_t end_speed;
	
}pmsm_mc_t;

/* VARIABLES ----------------------------------------------------------------------------------------*/
extern pmsm_foc_t pmsm_foc_param;
extern pmsm_mc_t pmsm_mc_param;
extern sto_luenberger_t sto1;

/* FUNCTION -----------------------------------------------------------------------------------------*/
void pmsm_foc_init(void);
void pmsm_foc_run(void);


#ifdef __cplusplus
extern "C"{
#endif

#endif /* _PMSM_H_ */

/***************************************** (END OF FILE) *********************************************/
