#include "smcpos.h"


void SMC_Position_Estimation (SMC *s)
{
	CalcEstI(&smc1);

	CalcIError(&smc1);

	// Sliding control calculator

	if (_Q15abs(s->IalphaError) < s->MaxSMCError)
	{
		// s->Zalpha = (s->Kslide * s->IalphaError) / s->MaxSMCError
		// If we are in the linear range of the slide mode controller,
		// then correction factor Zalpha will be proportional to the
		// error (Ialpha - EstIalpha) and slide mode gain, Kslide.
		CalcZalpha(&smc1);
	}
	else if (s->IalphaError > 0)
		s->Zalpha = s->Kslide;
	else
		s->Zalpha = -s->Kslide;

	if (_Q15abs(s->IbetaError) < s->MaxSMCError)
	{
		// s->Zbeta = (s->Kslide * s->IbetaError) / s->MaxSMCError
		// If we are in the linear range of the slide mode controller,
		// then correction factor Zbeta will be proportional to the
		// error (Ibeta - EstIbeta) and slide mode gain, Kslide.
		CalcZbeta(&smc1);
	}
	else if (s->IbetaError > 0)
		s->Zbeta = s->Kslide;
	else
		s->Zbeta = -s->Kslide;

	// Sliding control filter -> back EMF calculator
	// s->Ealpha = s->Ealpha + s->Kslf * s->Zalpha -
	//						   s->Kslf * s->Ealpha
	// s->Ebeta = s->Ebeta + s->Kslf * s->Zbeta -
	//						 s->Kslf * s->Ebeta
	// s->EalphaFinal = s->EalphaFinal + s->KslfFinal * s->Ealpha
	//								   - s->KslfFinal * s->EalphaFinal
	// s->EbetaFinal = s->EbetaFinal + s->KslfFinal * s->Ebeta
	//								 - s->KslfFinal * s->EbetaFinal
	CalcBEMF(&smc1);

	// Rotor angle calculator -> Theta = atan(-EalphaFinal,EbetaFinal)

	s->Theta = atan2CORDIC(-s->EalphaFinal,s->EbetaFinal);

	AccumTheta += s->Theta - PrevTheta;
	PrevTheta = s->Theta;
	
	AccumThetaCnt++;
	if (AccumThetaCnt == IRP_PERCALC)
	{
		s->Omega = AccumTheta;
		AccumThetaCnt = 0;
		AccumTheta = 0;
	}
    //                    Q15(Omega) * 60
    // Speed RPMs = -----------------------------
    //               SpeedLoopTime * Motor Poles
    // For example:
    //    Omega = 0.5
    //    SpeedLoopTime = 0.001
    //    Motor Poles (pole pairs * 2) = 10
    // Then:
    //    Speed in RPMs is 3,000 RPMs

	// s->OmegaFltred = s->OmegaFltred + FilterCoeff * s->Omega
	//								   - FilterCoeff * s->OmegaFltred

	CalcOmegaFltred(&smc1);

	// Adaptive filter coefficients calculation
	// Cutoff frequency is defined as 2*_PI*electrical RPS
	//
	// 		Wc = 2*_PI*Fc.
	// 		Kslf = Tpwm*2*_PI*Fc
	//
	// Fc is the cutoff frequency of our filter. We want the cutoff frequency
	// be the frequency of the drive currents and voltages of the motor, which
	// is the electrical revolutions per second, or eRPS.
	//
	// 		Fc = eRPS = RPM * Pole Pairs / 60
	//
	// Kslf is then calculated based on user parameters as follows:
	// First of all, we have the following equation for RPMS:
	//
	// 		RPM = (Q15(Omega) * 60) / (SpeedLoopTime * Motor Poles)
	//		Let us use: Motor Poles = Pole Pairs * 2
	//		eRPS = RPM * Pole Pairs / 60), or
	//		eRPS = (Q15(Omega) * 60 * Pole Pairs) / (SpeedLoopTime * Pole Pairs * 2 * 60)
	//	Simplifying eRPS
	//		eRPS = Q15(Omega) / (SpeedLoopTime * 2)
	//	Using this equation to calculate Kslf
	//		Kslf = Tpwm*2*_PI*Q15(Omega) / (SpeedLoopTime * 2)
	//	Using diIrpPerCalc = SpeedLoopTime / Tpwm
	//		Kslf = Tpwm*2*Q15(Omega)*_PI / (diIrpPerCalc * Tpwm * 2)
	//	Simplifying:
	//		Kslf = Q15(Omega)*_PI/diIrpPerCalc
	//
	// We use a second filter to get a cleaner signal, with the same coefficient
	//
	// 		Kslf = KslfFinal = Q15(Omega)*_PI/diIrpPerCalc
	//
	// What this allows us at the end is a fixed phase delay for theta compensation
	// in all speed range, since cutoff frequency is changing as the motor speeds up.
	// 
	// Phase delay: Since cutoff frequency is the same as the input frequency, we can
	// define phase delay as being constant of -45 DEG per filter. This is because
	// the equation to calculate phase shift of this low pass filter is 
	// arctan(Fin/Fc), and Fin/Fc = 1 since they are equal, hence arctan(1) = 45 DEG.
	// A total of -90 DEG after the two filters implemented (Kslf and KslfFinal).
	
	s->Kslf = s->KslfFinal = FracMpy(s->OmegaFltred,Q15(_PI / IRP_PERCALC));

	// Since filter coefficients are dynamic, we need to make sure we have a minimum
	// so we define the lowest operation speed as the lowest filter coefficient

	if (s->Kslf < Q15(OMEGA0 * _PI / IRP_PERCALC))
	{
		s->Kslf = Q15(OMEGA0 * _PI / IRP_PERCALC);
		s->KslfFinal = Q15(OMEGA0 * _PI / IRP_PERCALC);
	}
	s->ThetaOffset = CONSTANT_PHASE_SHIFT;
	s->Theta = s->Theta + s->ThetaOffset;
	return;
}

void SMCInit(SMC *s)
{
    //                R * Ts
    // Fsmopos = 1 - --------
    //                  L
    //            Ts
    // Gsmopos = ----
    //            L
    // Ts = Sampling Period. If sampling at PWM, Ts = 50 us
    // R = Phase Resistance. If not provided by motor datasheet,
    //     measure phase to phase resistance with multimeter, and
    //     divide over two to get phase resistance. If 4 Ohms are
    //     measured from phase to phase, then R = 2 Ohms
    // L = Phase inductance. If not provided by motor datasheet,
    //     measure phase to phase inductance with multimeter, and
    //     divide over two to get phase inductance. If 2 mH are
    //     measured from phase to phase, then L = 1 mH
//L0=(Ld+Lq)/2     detaL=(Ld-Lq)/2
//L¦Á=L0	=(Ld+Lq)/2
//L¦Â=L0-detaL	=Lq
//	
	if (Q15(PHASERES * LOOPTIMEINSEC) > Q15(PHASEIND))
		s->Fsmopos_a = Q15(0.0);
	else
		s->Fsmopos_a = Q15(1 - PHASERES * LOOPTIMEINSEC / PHASEIND);

	if (Q15(LOOPTIMEINSEC) > Q15(PHASEIND))
		s->Gsmopos_a = Q15(0.99999);
	else
		s->Gsmopos_a = Q15(LOOPTIMEINSEC / PHASEIND);
//--------------------------------------------------
	if (Q15(PHASERES * LOOPTIMEINSEC) > Q15(PHASEIND_LQ))
		s->Fsmopos_b = Q15(0.0);
	else
		s->Fsmopos_b = Q15(1 - PHASERES * LOOPTIMEINSEC / PHASEIND_LQ);

	if (Q15(LOOPTIMEINSEC) > Q15(PHASEIND_LQ))
		s->Gsmopos_b = Q15(0.99999);
	else
		s->Gsmopos_b = Q15(LOOPTIMEINSEC / PHASEIND_LQ);
	
	s->Kslide = Q15(SMCGAIN);
	s->MaxSMCError = Q15(MAXLINEARSMC);
	s->FiltOmCoef = Q15(OMEGA0 * _PI / IRP_PERCALC); // Cutoff frequency for omega filter
													 // is minimum omega, or OMEGA0

	return;
}

void CalcEstI(SMC *s)
{
	long LSData;
	LSData= s->Gsmopos_a * s->Valpha - s->Gsmopos_a * s->Ealpha - s->Gsmopos_a * s->Zalpha + s->Fsmopos_a * s->EstIalpha;
	s->EstIalpha = LSData/32768;
	
	LSData= s->Gsmopos_b * s->Vbeta - s->Gsmopos_b * s->Ebeta - s->Gsmopos_b * s->Zbeta + s->Fsmopos_b * s->EstIbeta;
	s->EstIbeta = LSData/32768;
}

void CalcIError(SMC *s)
{
	s->IalphaError = s->EstIalpha - s->Ialpha;
	s->IbetaError = s->EstIbeta - s->Ibeta;
}

void CalcZalpha(SMC *s)
{
	long LSData;
	LSData = (s->Kslide * s->IalphaError);
	s->Zalpha = LSData/ 256;	//s->MaxSMCError;
}

void CalcZbeta(SMC *s)
{
	long LSData;
	LSData = (s->Kslide * s->IbetaError); 
	s->Zbeta = LSData/ 256;	//s->MaxSMCError;
}


void CalcBEMF(SMC *s)
{
	long LSData1,LSData2;
	LSData1=s->Kslf * s->Zalpha;
	LSData2=s->Kslf * s->Ealpha;
	s->Ealpha = s->Ealpha + (LSData1- LSData2)/32768;
		
	LSData1=s->Kslf * s->Zbeta;
	LSData2=s->Kslf * s->Ebeta;
	s->Ebeta = s->Ebeta + (LSData1- LSData2)/32768;
	
	LSData1=s->KslfFinal * s->Ealpha;
	LSData2=s->KslfFinal * s->EalphaFinal;
	s->EalphaFinal = s->EalphaFinal + (LSData1- LSData2)/32768;
	
	LSData1=s->KslfFinal * s->Ebeta;
	LSData2=s->KslfFinal * s->EbetaFinal;	
	s->EbetaFinal = s->EbetaFinal + (LSData1- LSData2)/32768;
}

void CalcOmegaFltred(SMC *s)
{
	long LSData1,LSData2;
	
	LSData1=s->FiltOmCoef * s->Omega;
	LSData2=s->FiltOmCoef * s->OmegaFltred;
	
	s->OmegaFltred = s->OmegaFltred + (LSData1- LSData2)/32768;
}

SFRAC16 FracMpy(SFRAC16 mul_1, SFRAC16 mul_2)
{
	long LSData1;
	SFRAC16 LSData2;
	LSData1=mul_1*mul_2;
	LSData2=LSData1/32768;
	return(LSData2);
}

SFRAC16 FracDiv(SFRAC16 num_1, SFRAC16 den_1)
{
	long LSData1;
	SFRAC16 LSData2;
	LSData1=num_1*32768;
	LSData2=LSData1/den_1;
	return(LSData2);
}

int32_t _Q15sqrt(int32_t value)
{
	  long root = 0;
	
    step( 0);
    step( 2);
    step( 4);
    step( 6);
    step( 8);
    step(10);
    step(12);
    step(14);
    step(16);
    step(18);
    step(20);
    step(22);
    step(24);
    step(26);
    step(28);
    step(30);

    // round to the nearest integer, cuts max error in half
	if(root < value)++root;
	return root;	
}


