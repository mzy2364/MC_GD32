#ifndef UserParms_H
#define UserParms_H
#include "system_define.h"
//************** Start-Up Parameters **************

#define LOCKTIMEINSEC  		1.00	
#define OPENLOOPTIMEINSEC   1

#define INITIALTORQUE			6		// Initial Torque demand in Amps.

#define ENDSPEEDOPENLOOP MINSPEEDINRPM

#define POLEPAIRS      		2       // Number of pole pairs
#define PHASERES			((float)0.12)	//38 Phase resistance in Ohms.
#define PHASEIND_LQ		    ((float)0.0000730f)		//95
#define PHASEIND			((float)0.0000730f)//(Ld+Lq)/2 Phase inductance in Henrys.

#define MAX_SPEED_RPM           15000
#define NOMINALSPEEDINRPM       3000
#define MINSPEEDINRPM			500	// Minimum speed in RPM. Closed loop will operate at this

/* State observer constants */
/* Observer start-up output conditions  */
#define OBS_MINIMUM_SPEED_RPM          1440
#define NB_CONSECUTIVE_TESTS           2 /* corresponding to
                                                         former NB_CONSECUTIVE_TESTS/
                                                         (TF_REGULATION_RATE/
                                                         MEDIUM_FREQUENCY_TASK_RATE) */
#define OBS_MEAS_ERRORS_BEFORE_FAULTS    3  /*!< Number of consecutive errors
                                                           on variance test before a speed
                                                           feedback error is reported */
#define MAX_APPLICATION_SPEED_UNIT ((MAX_SPEED_RPM*SPEED_UNIT)/U_RPM)
#define MIN_APPLICATION_SPEED_UNIT ((MIN_APPLICATION_SPEED_RPM*SPEED_UNIT)/U_RPM)

#define TF_REGULATION_RATE 	(uint32_t) ((uint32_t)(PWMFREQUENCY)/(REGULATION_EXECUTION_RATE))
#define MAX_BEMF_VOLTAGE  (uint16_t)((MAX_SPEED_RPM * 1.2 *\
                           MOTOR_VOLTAGE_CONSTANT*SQRT_2)/(1000u*SQRT_3))
/*max phase voltage, 0-peak Volts*/
#define MAX_VOLTAGE (int16_t)((ADC_FULL_BIT/SQRT_3)*ADC_TO_VDC_COEF)
#define MAX_CURRENT (ADC_FULLSCALE_VOLT/(2*SHUNT_RES*SHUNT_OPA_GAIN))
#define OBS_MINIMUM_SPEED_UNIT    (uint16_t) ((OBS_MINIMUM_SPEED_RPM*SPEED_UNIT)/U_RPM)

#define MOTOR_VOLTAGE_CONSTANT  1.2 /*!< Volts RMS ph-ph /kRPM */
#define GAIN1                            -23566
#define GAIN2                            23107
/*Only in case PLL is used, PLL gains */
#define PLL_KP_GAIN                      798
#define PLL_KI_GAIN                      28
#define PLL_KPDIV     16384
#define PLL_KPDIV_LOG LOG2((PLL_KPDIV))
#define PLL_KIDIV     65535
#define PLL_KIDIV_LOG LOG2((PLL_KIDIV))

#define STO_FIFO_DEPTH_DPP               64  /*!< Depth of the FIFO used
                                                            to average mechanical speed
                                                            in dpp format */
#define STO_FIFO_DEPTH_DPP_LOG           LOG2((64))

#define STO_FIFO_DEPTH_UNIT              64  /*!< Depth of the FIFO used
                                                            to average mechanical speed
                                                            in the unit defined by #SPEED_UNIT */
#define BEMF_CONSISTENCY_TOL             64   /* Parameter for B-emf
                                                            amplitude-speed consistency */
#define BEMF_CONSISTENCY_GAIN            64   /* Parameter for B-emf
                                                           amplitude-speed consistency */

/* State observer scaling factors F1 */
#define RS                     0.12 /* Stator resistance , ohm*/
#define LS                     0.000073 /* Stator inductance, H
                                                 For I-PMSM it is equal to Lq */
#define F1                               16384
#define F2                               8192
#define F1_LOG                           LOG2((16384))
#define F2_LOG                           LOG2((8192))
#define C1 (int32_t)((((int16_t)F1)*RS)/(LS*TF_REGULATION_RATE))
#define C2 (int32_t) GAIN1
#define C3 (int32_t)((((int16_t)F1)*MAX_BEMF_VOLTAGE)/(LS*MAX_CURRENT*TF_REGULATION_RATE))
#define C4 (int32_t) GAIN2
#define C5 (int32_t)((((int16_t)F1)*MAX_VOLTAGE)/(LS*MAX_CURRENT*TF_REGULATION_RATE))

/****** State Observer + PLL ****/
#define VARIANCE_THRESHOLD              0.1 /*!<Maximum accepted
                                                            variance on speed
                                                            estimates (percentage) */
#define PERCENTAGE_FACTOR    (uint16_t)(VARIANCE_THRESHOLD*128u)

#define SPEED_BAND_UPPER_LIMIT         17 /*!< It expresses how much
                                                            estimated speed can exceed
                                                            forced stator electrical
                                                            without being considered wrong.
                                                            In 1/16 of forced speed */
#define SPEED_BAND_LOWER_LIMIT         15  /*!< It expresses how much
                                                             estimated speed can be below
                                                             forced stator electrical
                                                             without being considered wrong.
                                                             In 1/16 of forced speed */

//************** PWM and Control Timing Parameters **********

#define PWMFREQUENCY		16000		// PWM Frequency in Hertz
#define PWM_FREQ_SCALING    1
#define REGULATION_EXECUTION_RATE     1    /*!< FOC execution rate in
                                                           number of PWM cycles */

#define SPEEDLOOPFREQ		1000		// Speed loop Frequency in Hertz. This value must
									// be an integer to avoid pre-compiler error
#define MAXDUTYE				0.95
//************** Slide Mode Controller Parameters **********

#define SMCGAIN					0.85		// Slide Mode Controller Gain (0.0 to 0.9999)
#define MAXLINEARSMC    0.01		// If measured current - estimated current
								// is less than MAXLINEARSMC, the slide mode
								// Controller will have a linear behavior
								// instead of ON/OFF. Value from (0.0 to 0.9999)
//************** Hardware Parameters ****************

#define RSHUNT			0.005	// Value in Ohms of shunt resistors used.
#define DIFFAMPGAIN	    10.0		// Gain of differential amplifier.
#define VDD					3.3		// VDD voltage, only used to convert torque
								// reference from Amps to internal variables

#define SPEEDDELAY 10 // Delay for the speed ramp.
					  // Necessary for the PI control to work properly at high speeds.

//************** PI Coefficients **************

//******** D Control Loop Coefficients *******
#define     DKP        Q15(0.05)
#define     DKI        Q15(0.0005)
#define     DKC        Q15(0.99999)
#define     DOUTMAX    Q15(0.99999)

//******** Q Control Loop Coefficients *******
#define     QKP        Q15(0.05)
#define     QKI        Q15(0.0005)
#define     QKC        Q15(0.99999)
#define     QOUTMAX    Q15(0.99999)

//*** Velocity Control Loop Coefficients *****
#define     WKP        Q15(0.1)
#define     WKI        Q15(0.000105)
#define     WKC        Q15(0.99999)
#define     WOUTMAX    Q15(0.95)

// Scaling constants: Determined by calibration or hardware design. 
#define     DQK        Q15((OMEGA10 - OMEGA1)/2.0)	// POT Scaling
#define     DQKA       Q15(0.99999)	// Current feedback software gain
#define     DQKB       Q15(0.99999)	// Current feedback software gain

#define ID_REF		0

//************** Derived Parameters ****************
#define LOOPTIMEINSEC (1.0/PWMFREQUENCY) // PWM Period = 1.0 / PWMFREQUENCY
#define IRP_PERCALC (unsigned short)(SPEEDLOOPTIME/LOOPTIMEINSEC)	// PWM loops per velocity calculation
#define SPEEDLOOPTIME (float)(1.0/SPEEDLOOPFREQ) // Speed Control Period
#define LOCKTIME	(unsigned short)(LOCKTIMEINSEC*(1.0/LOOPTIMEINSEC))
// Time it takes to ramp from zero to MINSPEEDINRPM. Time represented in seconds
#define DELTA_STARTUP_RAMP	(unsigned short)(MINSPEEDINRPM*POLEPAIRS*LOOPTIMEINSEC* \
							LOOPTIMEINSEC*65536*65536/(60*OPENLOOPTIMEINSEC))


#define OPEN_LOOP_MODE
//#define OPEN_LOOP_VF_VQ     Q15(0.2)
#define TORQUE_MODE

#endif
