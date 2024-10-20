#ifndef UserParms_H
#define UserParms_H
#include "system_define.h"
//************** Start-Up Parameters **************

#define LOCKTIMEINSEC  		1.00	
#define OPENLOOPTIMEINSEC 3.0	

#define INITIALTORQUE			1		// Initial Torque demand in Amps.

#define ENDSPEEDOPENLOOP MINSPEEDINRPM

#define POLEPAIRS      		2       // Number of pole pairs
#define PHASERES			((float)0.42)	//38 Phase resistance in Ohms.

#define PHASEIND_LQ		((float)0.00112)		//95
#define PHASEIND			((float)0.00112)//(Ld+Lq)/2 Phase inductance in Henrys.

#define NOMINALSPEEDINRPM       600
#define MINSPEEDINRPM			500	// Minimum speed in RPM. Closed loop will operate at this

//************** PWM and Control Timing Parameters **********

#define PWMFREQUENCY		15000		// PWM Frequency in Hertz
#define DEADTIMESEC			0.0000015	// Deadtime in seconds
#define	BUTPOLLOOPTIME	0.100		// Button polling loop period in sec
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

//************** Real Time Data Monitor, RTDM *******************

#undef DMCI_DEMO	// Define this if a demo with DMCI is done. Start/stop of motor
					// and speed variation is done with DMCI instead of push button
					// and POT. Undefine "DMCI_DEMO" is user requires operating
					// this application with no DMCI

#define SPEEDDELAY 10 // Delay for the speed ramp.
					  // Necessary for the PI control to work properly at high speeds.

//*************** Optional Modes **************
//#define TORQUEMODE
//#define ENVOLTRIPPLE

//************** PI Coefficients **************

//******** D Control Loop Coefficients *******
#define     DKP        Q15(0.200)
#define     DKI        Q15(0.010)
#define     DKC        Q15(0.99999)
#define     DOUTMAX    Q15(0.99999)

//******** Q Control Loop Coefficients *******
#define     QKP        Q15(0.500)
#define     QKI        Q15(0.030)
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
#define DFCY        SystemCoreClock		// Instruction cycle frequency (Hz)
#define DTCY        (1.0/DFCY)		// Instruction cycle period (sec)
#define DDEADTIME   (unsigned short)(DEADTIMESEC*DFCY)	// Dead time in dTcys
#define LOOPTIMEINSEC (1.0/PWMFREQUENCY) // PWM Period = 1.0 / PWMFREQUENCY
#define IRP_PERCALC (unsigned short)(SPEEDLOOPTIME/LOOPTIMEINSEC)	// PWM loops per velocity calculation
#define SPEEDLOOPTIME (float)(1.0/SPEEDLOOPFREQ) // Speed Control Period
#define LOCKTIME	(unsigned short)(LOCKTIMEINSEC*(1.0/LOOPTIMEINSEC))
// Time it takes to ramp from zero to MINSPEEDINRPM. Time represented in seconds
#define DELTA_STARTUP_RAMP	(unsigned short)(MINSPEEDINRPM*POLEPAIRS*LOOPTIMEINSEC* \
							LOOPTIMEINSEC*65536*65536/(60*OPENLOOPTIMEINSEC))
// Number of control loops that must execute before the button routine is executed.
#define	BUTPOLLOOPCNT	(unsigned short)(BUTPOLLOOPTIME/LOOPTIMEINSEC)


//John catch speed data
#define CATCHSPEEDTIME (unsigned short)(0.02*(1.0/LOOPTIMEINSEC))
#define CATCHSPEEDMIN		OMEGA0
#define ERROROMEGAMIN	2000

//#define OPEN_LOOP_MODE
//#define OPEN_LOOP_VF_VQ     Q15(0.2)
//#define TORQUE_MODE

#endif
