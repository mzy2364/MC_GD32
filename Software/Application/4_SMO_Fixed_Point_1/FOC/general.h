#ifndef general_H
#define general_H
#include "system_define.h"

typedef unsigned short WORD;
typedef signed 	short  SFRAC16;
typedef unsigned char  BYTE;
#define False  0
#define True   1

#define _0_05DEG 9	// The value for 0.05 degrees is converted
					// to Q15 as follows:
					// .05 * 32768 / 180 = 9.1, approx 9.

#define Q15(Float_Value)	\
        ((Float_Value < 0.0) ? (SFRAC16)(32768 * (Float_Value) - 0.5) \
        : (SFRAC16)(32767 * (Float_Value) + 0.5))
				
#define _Q15abs(value)	(value<0 ? -value : value)
#define step(shift) \
	if((0x40000000l >> shift) + root <= value)          \
	{                                                   \
	    value -= (0x40000000l >> shift) + root;         \
	    root = (root >> 1) | (0x40000000l >> shift);    \
	}                                                   \
	else                                                \
	{                                                   \
	    root = root >> 1;                               \
	}	

//#define REFINAMPS(Amperes_Value)	\
//		(Q15((Amperes_Value)*(DQKA/32768.0)*RSHUNT*DIFFAMPGAIN/(VDD/2)))

#define pinButton1	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)
#define pinButton2	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)


typedef union   
{
	struct
	{
		unsigned OpenLoop:1;	// Indicates if motor is running in open or closed loop
		unsigned RunMotor:1;	// If motor is running, or stopped.
		unsigned EnTorqueMod:1;	// This bit enables Torque mode when running closed loop
		unsigned EnVoltRipCo:1;	// Bit that enables Voltage Ripple Compensation
		unsigned Btn1Pressed:1;	// Button 1 has been pressed.
		unsigned Btn2Pressed:1;	// Button 2 has been pressed.
		unsigned ChangeMode:1;	// This flag indicates that a transition from open to closed
		// loop, or closed to open loop has happened. This
		// causes DoControl subroutine to initialize some variables
		// before executing open or closed loop for the first time
		unsigned ChangeSpeed:1;	// This flag indicates a step command in speed reference.
		// This is mainly used to analyze step response
		//add by John
		unsigned MotorFail 		:1;
		
		unsigned CatchSpeed 	:1;
		unsigned MotorStatic 	:2;
		unsigned    :20;
	}bit;
  WORD Word;
} uGFt;

extern uGFt uGF;

// Main functions prototypes
void SensorlessFOCinit(void);
void SensorlessFOCRUN(void);
bool SetupParm(void);
void DoControl( void );
void CalculateParkAngle(void);
void SetupControlParameters(void);
void DebounceDelay(void);
SFRAC16 VoltRippleComp(SFRAC16 Vdq1);
int32_t _Q15sqrt(int32_t SqrtData);

#ifdef INITIALIZE
    // allocate storage
    #define EXTERN
#else
    #define EXTERN extern
#endif

#endif      // end of general_H
