/*
*******************************************************************************************************
*
* File Name : foc.h
* Version   : V1.0
* Author    : mzy2364
* brief     : foc library
* 
*******************************************************************************************************
*/
#ifndef _FOC_H_
#define _FOC_H_

#ifdef __cplusplus
extern "C"{
#endif

/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include <stdint.h>
#include "arm_math.h"

/* DEFINES ------------------------------------------------------------------------------------------*/
#define 	SQRT3_BY2     			(float)0.866025403788  // Defines value for sqrt(3)/2
#define 	ONE_BY_SQRT3     		(float)0.5773502691    // Defines value for 1/sqrt(3)
#define 	M_PI     				(float)3.1415926535

#define 	ANGLE_2PI              	(2*M_PI)               // Defines value for 2*PI
/* TYPEDEF ------------------------------------------------------------------------------------------*/
typedef struct{
	float sum;
	float kp;
	float ki;
	float kc;
	float out_max;
	float out_min;
	float ref; 
	float meas;
	float out;
    float error;
}pid_control_t;

typedef struct{
	float angle;
	float sin;
	float cos;
	float ia;
	float ib;
    float ic;
	float ialpha;
	float ibeta;
	float id;
	float iq;
	float vd;
	float vq;
	float valpha;
	float vbeta;
	
	/* PID */
	pid_control_t pi_d;
	pid_control_t pi_q;
	pid_control_t pi_w;
	
	/* SVPWM */
	float pwm_period;
	float pwma;
	float pwmb;
	float pwmc;
	uint8_t sector;
    
    float vdc;
    float max_phase_voltage;
	
}pmsm_foc_t;

/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/
void sin_cos(pmsm_foc_t *foc);
void inv_park(pmsm_foc_t *foc);
void park(pmsm_foc_t *foc);
void clark(pmsm_foc_t *foc);
void svpwm(pmsm_foc_t *foc);
void init_pi(pid_control_t *p_parm);
void calc_pi(pid_control_t *p_parm);

#ifdef __cplusplus
extern "C"{
#endif

#endif /* _FOC_H_ */

/***************************************** (END OF FILE) *********************************************/
