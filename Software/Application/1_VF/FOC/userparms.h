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
#define LOCK_TIME_SEC  		                            0.1
#define LOCK_COUNTER                                    (unsigned int)((float)LOCK_TIME_SEC/(float)LOOPTIME_SEC)

#define OPENLOOPTIMEINSEC 	                            3
#define OPEN_LOOP_END_SPEED_RPM                         500

#define END_SPEED_RADS_MECH                             (float)(OPEN_LOOP_END_SPEED_RPM*RPM_TO_RADS)
#define END_SPEED_RADS_ELEC                             (float)(END_SPEED_RADS_MECH*MOTOR_NOPOLESPAIRS)
#define END_SPEED_RADS_ELEC_COUNTER                     (float)(END_SPEED_RADS_ELEC * LOOPTIME_SEC)
#define OPENLOOP_RAMPSPEED_INCREASERATE                 (float)(END_SPEED_RADS_ELEC_COUNTER/(OPENLOOPTIMEINSEC/LOOPTIME_SEC))


#define OPEN_LOOP_MODE
#define OPEN_LOOP_VF_VQ     0.6f

#ifdef __cplusplus
}
#endif

#endif /* _USERPARMS_H */

/************************************************EOF************************************************/
