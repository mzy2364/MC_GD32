
#ifndef smcpos_H
#define smcpos_H

#include "system_define.h"

typedef struct	{  
		int16_t  Valpha;   		// Input: Stationary alfa-axis stator voltage ��ֹ���ᶨ�ӵ�ѹ
		int16_t  Ealpha;   		// Variable: Stationary alfa-axis back EMF ��ֹ���ᷴ�綯��
		int16_t  EalphaFinal;	// Variable: Filtered EMF for Angle calculation ����EMF�Ƕȼ���
		int16_t  Zalpha;      	// Output: Stationary alfa-axis sliding control ��ֹ���Ử������
		int16_t  Gsmopos_a;    	// Parameter: Motor dependent control gain �����ؿ�������
		int16_t  Gsmopos_b;    	// Parameter: Motor dependent control gain 	�����ؿ�������
		int16_t  EstIalpha;   	// Variable: Estimated stationary alfa-axis stator current ���ƾ�ֹ�������ᶨ�ӵ���
		int16_t  Fsmopos_a;    	// Parameter: Motor dependent plant matrix 
		int16_t  Fsmopos_b;    	// Parameter: Motor dependent plant matrix 
		int16_t  Vbeta;   		// Input: Stationary beta-axis stator voltage ��ֹ���ᶨ�ӵ�ѹ
		int16_t  Ebeta;  		// Variable: Stationary beta-axis back EMF ��ֹ�ᷴ�綯��
		int16_t  EbetaFinal;	// Variable: Filtered EMF for Angle calculation ����EMF�Ƕȼ���
		int16_t  Zbeta;      	// Output: Stationary beta-axis sliding control ��ֹ�Ử������
		int16_t  EstIbeta;    	// Variable: Estimated stationary beta-axis stator current ���ƾ�ֹ���ᶨ�ӵ���
		int16_t  Ialpha;  		// Input: Stationary alfa-axis stator current ��ֹ���ᶨ�ӵ���
		int16_t  IalphaError; 	// Variable: Stationary alfa-axis current error ��ֹ����������
		int16_t  Kslide;     	// Parameter: Sliding control gain ������������
		int16_t  MaxSMCError;  	// Parameter: Maximum current error for linear SMC ����SMC�����������
		int16_t  Ibeta;  		// Input: Stationary beta-axis stator current ��ֹ���ᶨ�ӵ���
		int16_t  IbetaError;  	// Variable: Stationary beta-axis current error ��ֹ����������
		int16_t  Kslf;       	// Parameter: Sliding control filter gain ���������˲�������
		int16_t  KslfFinal;    	// Parameter: BEMF Filter for angle calculation ���ڽǶȼ����BEMF�˲���
		int16_t  FiltOmCoef;   	// Parameter: Filter Coef for Omega filtered calc ����ϵ��ΪOmega����calc
		int16_t  ThetaOffset;	// Output: Offset used to compensate rotor angle ���ڲ���ת�ӽǶȵ�ƫ����
		int16_t  Theta;			// Output: Compensated rotor angle ����ת�ӽ�
		int16_t  Omega;     	// Output: Rotor speed ת��ת��
		int16_t  OmegaFltred;  	// Output: Filtered Rotor speed for speed PI ����ת��ת��Ϊ�ٶ�PI
} SMC;	            

extern SMC smc1;

#define SMC_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

// Define this in RPMs

#define SPEED0 MINSPEEDINRPM
#define SPEED1 NOMINALSPEEDINRPM

// Define this in Degrees, from 0 to 360

#define THETA_AT_ALL_SPEED 90

#define OMEGA0 				(float)(SPEED0 * LOOPTIMEINSEC * \
										IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGANOMINAL	(float)(NOMINALSPEEDINRPM * LOOPTIMEINSEC * \
                		IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
	
#define THETA_ALL (float)(THETA_AT_ALL_SPEED * 180.0 / 32768.0)

#define CONSTANT_PHASE_SHIFT Q15(THETA_ALL)

#define _PI 3.1416

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

#define REFINAMPS(Amperes_Value)	\
		(Q15((Amperes_Value)*(DQKA/32768.0)*RSHUNT*DIFFAMPGAIN/(VDD/2)))

extern int16_t PrevTheta;
extern int16_t AccumTheta;
extern uint32_t AccumThetaCnt;

void SMC_Position_Estimation (SMC* s);
void SMCInit(SMC* s);
void CalcEstI(SMC* s);
void CalcIError(SMC *s);
void CalcZalpha(SMC *s);
void CalcZbeta(SMC *s);
void CalcBEMF(SMC *s);
void CalcOmegaFltred(SMC *s);
int16_t FracMpy(int16_t mul_1, int16_t mul_2);
int16_t FracDiv(int16_t num_1, int16_t den_1);

#endif
		
