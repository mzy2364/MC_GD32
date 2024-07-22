/*
*******************************************************************************************************
*
* File Name : motor_config.h
* Version   : V1.0
* Author    : mzy2364
* brief     : motor_config header file
* 
*******************************************************************************************************
*/
#ifndef _MOTOR_CONFIG_H_
#define _MOTOR_CONFIG_H_

#ifdef __cplusplus
extern "C"{
#endif

/* INCLUDE FILES ------------------------------------------------------------------------------------*/



/* DEFINES ------------------------------------------------------------------------------------------*/
#define ADC_FULLSCALE_VOLT              (float)3.3
#define ADC_FULL_BIT                    (1<<12)
#define SHUNT_RES                       (float)0.005
#define SHUNT_OPA_GAIN                  (float)8
    
#define VDC_SERIES_RES                  (float)75000
#define VDC_PARALLEL_RES                (float)3000
    
#define ADC_TO_CURRENT_COEF             (float)(ADC_FULLSCALE_VOLT/ADC_FULL_BIT/SHUNT_RES/SHUNT_OPA_GAIN)
#define ADC_TO_VDC_COEF                 (float)((VDC_SERIES_RES+VDC_PARALLEL_RES)/VDC_PARALLEL_RES*ADC_FULLSCALE_VOLT/ADC_FULL_BIT)

#define PHASE_OVER_CURRENT_AMP          (float)15
#define PHASE_OVER_CURRENT_DAC_VAULE    (float)(2048.0f + PHASE_OVER_CURRENT_AMP*SHUNT_RES*SHUNT_OPA_GAIN*ADC_FULL_BIT/ADC_FULLSCALE_VOLT)
    
#define SYSTEM_CORE_CLOCK               120000000
#define PWM_PRESCALER                   0
#define MOTOR_PWM_FREQ_HZ               16000
#define MOTOR_PWM_PERIOD                ((SYSTEM_CORE_CLOCK/(PWM_PRESCALER+1)/MOTOR_PWM_FREQ_HZ/2)-1)

#define HALL_TIMER_PRESCALER            119
#define HALL_TIMER_FREQ_HZ              (SYSTEM_CORE_CLOCK/(HALL_TIMER_PRESCALER+1))

/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/


#ifdef __cplusplus
extern "C"{
#endif

#endif /* _MOTOR_CONFIG_H_ */

/***************************************** (END OF FILE) *********************************************/
