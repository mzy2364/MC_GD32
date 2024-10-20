/**
*   @file           userparms.h
*   @implements     userparms.h_Artifact
*   @version        v1.0.0
*   @date           2023-12-03
*   @author         mzy2364
*
*   @brief          Motor control user parameter definition
*   @details        
*
*   @addtogroup     PMSM Motor Control
*   @log
*       data        author            notes
*   2023-12-03      mzy2364           Create
*/

#ifndef _USERPARMS_H
#define _USERPARMS_H

#ifdef __cplusplus
extern "C"{
#endif

/*==================================================================================================
*                                        INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
==================================================================================================*/
#include <stdint.h>
#include "motor_config.h"
/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/
/*************** General Parameters ********************************************/
#define RPM_TO_RADS                                     ((float)2*M_PI/60)
    
#define HALL_DEBUG                                      0

/*************** Motor Parameters *********************************************/
//24V
//#define MOTOR_PHASE_RESISTANCE                          ((float)0.12)
//#define MOTOR_PHASE_INDUCTANCE                          ((float)0.000143f)
//#define MOTOR_PHASE_INDUCTANCE_DIV_2_PI                 ((float)(MOTOR_PHASE_INDUCTANCE/(2*M_PI)))	
//#define MOTOR_BACK_EMF_Vpeak                            (float)1.24
//#define MOTOR_NOPOLESPAIRS                               2

//12V
//#define     MOTOR_PHASE_RESISTANCE                          ((float)0.063f)			// Resistance in Ohms 相电阻
//#define     MOTOR_PHASE_INDUCTANCE                          ((float)0.000032f)		// Inductance in Henrys	 相电感
//#define     MOTOR_PHASE_INDUCTANCE_DIV_2_PI                 ((float)(MOTOR_PHASE_INDUCTANCE/(2*M_PI)))	
//#define     MOTOR_BACK_EMF_Vpeak   (float)0.85f				// Back EMF Constant in Vpeak/KRPM
//#define     MOTOR_NOPOLESPAIRS                                        2						//极对数

//57BL
//#define MOTOR_PHASE_RESISTANCE                          ((float)0.42)
//#define MOTOR_PHASE_INDUCTANCE                          ((float)0.00112)
//#define MOTOR_PHASE_INDUCTANCE_DIV_2_PI                 ((float)(MOTOR_PHASE_INDUCTANCE/(2*M_PI)))	
//#define MOTOR_BACK_EMF_Vpeak                            (float)4.27
//#define MOTOR_NOPOLESPAIRS                               2


//60V DC
//#define MOTOR_PHASE_RESISTANCE                          ((float)0.6)
//#define MOTOR_PHASE_INDUCTANCE                          ((float)0.000242)
//#define MOTOR_PHASE_INDUCTANCE_DIV_2_PI                 ((float)(MOTOR_PHASE_INDUCTANCE/(2*M_PI)))	
//#define MOTOR_BACK_EMF_Vpeak                            (float)1.24
//#define MOTOR_NOPOLESPAIRS                               2

//105V DC
#define MOTOR_PHASE_RESISTANCE                          ((float)3.485)
#define MOTOR_PHASE_INDUCTANCE                          ((float)0.002675)
#define MOTOR_PHASE_INDUCTANCE_DIV_2_PI                 ((float)(MOTOR_PHASE_INDUCTANCE/(2*M_PI)))	
#define MOTOR_BACK_EMF_Vpeak                            (float)12.25
#define MOTOR_NOPOLESPAIRS                               2

/*************** PWM and Control Timing Parameters ****************************/
#define PWMFREQUENCY_HZ                                 (float)MOTOR_PWM_FREQ_HZ
#define LOOPTIME_SEC                                    (float)(1/PWMFREQUENCY_HZ)

/*************** Startup Parameters *******************************************/
#define OPEN_LOOP_CURRENT                               2

#define SPEED_LOOP_CYCLE                                10

#define LOCK_TIME_SEC  		                            0.1
#define LOCK_COUNTER                                    (unsigned int)((float)LOCK_TIME_SEC/(float)LOOPTIME_SEC)

#define OPENLOOPTIMEINSEC 	                            1
#define OPEN_LOOP_END_SPEED_RPM                         500

#define END_SPEED_RADS_MECH                             (float)(OPEN_LOOP_END_SPEED_RPM*RPM_TO_RADS)
#define END_SPEED_RADS_ELEC                             (float)(END_SPEED_RADS_MECH*MOTOR_NOPOLESPAIRS)
#define END_SPEED_RADS_ELEC_COUNTER                     (float)(END_SPEED_RADS_ELEC * LOOPTIME_SEC)
#define OPENLOOP_RAMPSPEED_INCREASERATE                 (float)(END_SPEED_RADS_ELEC_COUNTER/(OPENLOOPTIMEINSEC/LOOPTIME_SEC))
    
#define CL_RAMP_RATE_RPM_SEC                             500 // CLosed Loop Speed Ramp rate in Rev/min/Sec
#define CL_RAMP_RATE_RPS_SEC                             ((float)CL_RAMP_RATE_RPM_SEC/60) // CLosed Loop  Speed Ramp rate in Rev/sec^2 
#define CL_RAMP_RATE_RADS_PER_SEC2_MECH                  (float)(CL_RAMP_RATE_RPS_SEC*2*M_PI) // CLosed Loop  Speed Ramp Rate in Mechanical Radians/Sec^2
#define CL_RAMP_RATE_RADS_PER_SEC2_ELEC                  (float)(CL_RAMP_RATE_RADS_PER_SEC2_MECH*MOTOR_NOPOLESPAIRS) // CLosed Loop  Speed Ramp rate in Electrical Radians/Sec^2
#define CL_SPEED_RAMP_RATE_DELTA                         (float)(CL_RAMP_RATE_RADS_PER_SEC2_ELEC*LOOPTIME_SEC) // CLosed Loop  Speed Ramp Rate in Electrical Radians/sec^2 in each control loop time
#define CL_SPEED_HYSTERESIS                              (float)(5*CL_SPEED_RAMP_RATE_DELTA)
    
#define MIN_SPEED_RPM                                      (float)100
#define MIN_SPEED_RAD_PER_SEC_ELEC                      (float)(((MIN_SPEED_RPM/60)*2*M_PI)*MOTOR_NOPOLESPAIRS) // Value in RPM
 

#define NOMINAL_SPEED_RPM                                   (float)3000 // Value in RPM
#define NOMINAL_SPEED_RAD_PER_SEC_ELEC                      (float)(((NOMINAL_SPEED_RPM/60)*2*M_PI)*MOTOR_NOPOLESPAIRS) // Value in RPM

    
/*************** PID Parameters ***********************************************/

//******** D Control Loop Coefficients *******
#define     DKP        (0.01)
#define     DKI        (0.0001)
#define     DKC        (0.5)
#define     DOUTMAX    (0.99999)

//******** Q Control Loop Coefficients *******
#define     QKP        (0.01)
#define     QKI        (0.0001)
#define     QKC        (0.5)
#define     QOUTMAX    (0.99999)

//*** Velocity Control Loop Coefficients *****
#define     WKP        (0.02)
#define     WKI        (0.0000020)
#define     WKC        (0.5)
#define     WOUTMAX    (10)



#if(HALL_DEBUG == 1)
#define OPEN_LOOP_MODE
#endif
//#define OPEN_LOOP_VF_VQ     0.3f

#ifdef __cplusplus
}
#endif

#endif /* _USERPARMS_H */

/************************************************EOF************************************************/
