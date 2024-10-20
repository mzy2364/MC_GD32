#define INITIALIZE
#include "system_define.h"
#include "pmsm.h"

#include "sto_pll_speed_pos_fdbk.h"
#include "mc_math.h"

/********************* Variables to display data using DMCI *********************************/
int16_t count = 0; // delay for ramping the reference velocity 
int16_t VelReq = 0; 

SMC smc1 = SMC_DEFAULTS;

uint32_t Startup_Ramp = 0;	/* Start up ramp in open loop. This variable
								is incremented in CalculateParkAngle()
								subroutine, and it is assigned to 
								ParkParm.qAngle as follows:
								ParkParm.qAngle += (int)(Startup_Ramp >> 16);*/

uint16_t Startup_Lock = 0;	/* This is a counter that is incremented in
								CalculateParkAngle() every time it is called. 
								Once this counter has a value of LOCK_TIME, 
								then theta will start increasing moving the 
								motor in open loop. */
uint16_t CatchSpeed_Cnt=0;

// Speed Calculation Variables

uint32_t iADCisrCnt = 0;	// This Counter is used as a timeout for polling the push buttons
						// in main() subroutine. It will be reset to zero when it matches
						// dButPolLoopCnt defined in UserParms.h
int16_t PrevTheta = 0;	// Previous theta which is then substracted from Theta to get
						// delta theta. This delta will be accumulated in AccumTheta, and
						// after a number of accumulations Omega is calculated.
int16_t AccumTheta = 0;	// Accumulates delta theta over a number of times
uint32_t AccumThetaCnt = 0;	// Counter used to calculate motor speed. Is incremented
						// in SMC_Position_Estimation() subroutine, and accumulates
						// delta Theta. After N number of accumulations, Omega is 
						// calculated. This N is diIrpPerCalc which is defined in
						// UserParms.h.

// Vd and Vq vector limitation variables

int32_t qVdSquared = 0;	// This variable is used to know what is left from the VqVd vector
						// in order to have maximum output PWM without saturation. This is
						// done before executing Iq control loop at the end of DoControl()

int16_t DCbus = 0;		// DC Bus measured continuously and stored in this variable
						// while motor is running. Will be compared with TargetDCbus
						// and Vd and Vq will be compensated depending on difference
						// between DCbus and TargetDCbus

int16_t TargetDCbus = 0;// DC Bus is measured before running motor and stored in this
						// variable. Any variation on DC bus will be compared to this value
						// and compensated linearly.	

int16_t Theta_error = 0;// This value is used to transition from open loop to closed looop. 
						// At the end of open loop ramp, there is a difference between 
						// forced angle and estimated angle. This difference is stored in 
						// Theta_error, and added to estimated theta (smc1.Theta) so the 
						// effective angle used for commutating the motor is the same at 
						// the end of open loop, and at the begining of closed loop. 
						// This Theta_error is then substracted from estimated theta 
						// gradually in increments of 0.05 degrees until the error is less
						// than 0.05 degrees.

pmsm_foc_t pmsm_foc_param = {0};
pmsm_mc_t pmsm_mc_param = {0};


STO_PLL_Handle_t STO_PLL_M1 =
{
  ._Super = {
	.bElToMecRatio                     =	POLEPAIRS,
    .SpeedUnit                         =    SPEED_UNIT,
    .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*((MAX_SPEED_RPM*SPEED_UNIT)/U_RPM)),
    .hMinReliableMecSpeedUnit          =	(uint16_t)(0),
    .bMaximumSpeedErrorsNumber         =	0,
    .hMaxReliableMecAccelUnitP         =	65535,
    .hMeasurementFrequency             =	((uint32_t)(PWMFREQUENCY)/(REGULATION_EXECUTION_RATE*PWM_FREQ_SCALING)),
    .DPPConvFactor                     =    (65536/PWM_FREQ_SCALING),
  },
 .hC1                         =	C1,
 .hC2                         =	C2,
 .hC3                         =	C3,
 .hC4                         =	C4,
 .hC5                         =	C5,
 .hF1                         =	F1,
 .hF2                         =	F2,
 .PIRegulator = {
     .hDefKpGain = PLL_KP_GAIN,
     .hDefKiGain = PLL_KI_GAIN,
	 .hDefKdGain = 0x0000U,
     .hKpDivisor = PLL_KPDIV,
     .hKiDivisor = PLL_KIDIV,
	 .hKdDivisor = 0x0000U,
     .wUpperIntegralLimit = INT32_MAX,
     .wLowerIntegralLimit = -INT32_MAX,
     .hUpperOutputLimit = INT16_MAX,
     .hLowerOutputLimit = -INT16_MAX,
     .hKpDivisorPOW2 = PLL_KPDIV_LOG,
     .hKiDivisorPOW2 = PLL_KIDIV_LOG,
     .hKdDivisorPOW2       = 0x0000U,
   },
 .SpeedBufferSizeUnit                =	STO_FIFO_DEPTH_UNIT,
 .SpeedBufferSizeDpp                 =	STO_FIFO_DEPTH_DPP,
 .VariancePercentage                 =	PERCENTAGE_FACTOR,
 .SpeedValidationBand_H              =	SPEED_BAND_UPPER_LIMIT,
 .SpeedValidationBand_L              =	SPEED_BAND_LOWER_LIMIT,
 .MinStartUpValidSpeed               =	OBS_MINIMUM_SPEED_UNIT,
 .StartUpConsistThreshold            =	NB_CONSECUTIVE_TESTS,
 .Reliability_hysteresys             =	OBS_MEAS_ERRORS_BEFORE_FAULTS,
 .BemfConsistencyCheck               =	BEMF_CONSISTENCY_TOL,
 .BemfConsistencyGain                =	BEMF_CONSISTENCY_GAIN,
 .MaxAppPositiveMecSpeedUnit         =	(uint16_t)(MAX_APPLICATION_SPEED_UNIT*1.15),
 .F1LOG                              =	F1_LOG,
 .F2LOG                              =	F2_LOG,
 .SpeedBufferSizeDppLOG              =	STO_FIFO_DEPTH_DPP_LOG,
 .hForcedDirection                   =  0x0000U
};


static void pmsm_foc_init_control_parameters(void);
static void CalculateParkAngle(void);

/************* START OF MAIN FUNCTION ***************/
void pmsm_foc_init(void)
{
	SMCInit(&smc1);
    
    STO_PLL_Init (&STO_PLL_M1);

    pmsm_mc_param.run_motor = 1;
    pmsm_mc_param.openloop = 1;
    pmsm_mc_param.change_mode = 1;
    pmsm_mc_param.startup_ramp = 0;
    pmsm_mc_param.startup_lock = 0;
    pmsm_foc_init_control_parameters();
}

//---------------------------------------------------------------------
// Executes one PI itteration for each of the three loops Id,Iq,Speed,

void DoControl( void )
{
    if(pmsm_mc_param.openloop)
    {
        if(pmsm_mc_param.change_mode)
        {
            // just changed to openloop
            pmsm_mc_param.change_mode = 0;
            // synchronize angles
            // VqRef & VdRef not used
            Startup_Lock = 0;
            Startup_Ramp = 0;
            // Initialize SMC
            smc1.Valpha = 0;
            smc1.Ealpha = 0;
            smc1.EalphaFinal = 0;
            smc1.Zalpha = 0;
            smc1.EstIalpha = 0;
            smc1.Vbeta = 0;
            smc1.Ebeta = 0;
            smc1.EbetaFinal = 0;
            smc1.Zbeta = 0;
            smc1.EstIbeta = 0;
            smc1.Ialpha = 0;
            smc1.IalphaError = 0;
            smc1.Ibeta = 0;
            smc1.IbetaError = 0;
            smc1.Theta = 0;
            smc1.Omega = 0;
        }

        if(AccumThetaCnt == 0)
        {
            pmsm_foc_param.pi_w.meas = smc1.Omega;
        }
        
        pmsm_mc_param.iq_ref = REFINAMPS(INITIALTORQUE);
        
        // PI control for D
        pmsm_foc_param.pi_d.meas = pmsm_foc_param.id;
        pmsm_foc_param.pi_d.ref = pmsm_mc_param.id_ref;
        calc_pi(&pmsm_foc_param.pi_d);
        pmsm_foc_param.vd = pmsm_foc_param.pi_d.out;

        // Vector limitation
        // Vd is not limited
        // Vq is limited so the vector Vs is less than a maximum of 95%.
        // The 5% left is needed to be able to measure current through
        // shunt resistors.
        // Vs = SQRT(Vd^2 + Vq^2) < 0.95
        // Vq = SQRT(0.95^2 - Vd^2)
        pmsm_mc_param.vd_squared = pmsm_foc_param.pi_d.out * pmsm_foc_param.pi_d.out;
        pmsm_foc_param.pi_q.out_max = sqrt_q15(Q15(MAXDUTYE)*Q15(MAXDUTYE)- pmsm_mc_param.vd_squared);		
        pmsm_foc_param.pi_q.out_min = -pmsm_foc_param.pi_q.out_max;

        // PI control for Q
        pmsm_foc_param.pi_q.meas = pmsm_foc_param.iq;
        pmsm_foc_param.pi_q.ref = pmsm_mc_param.iq_ref;
        calc_pi(&pmsm_foc_param.pi_q);
        pmsm_foc_param.vq = pmsm_foc_param.pi_q.out;
        
#ifdef OPEN_LOOP_VF_VQ
        ParkParm.qVq = OPEN_LOOP_VF_VQ;
        ParkParm.qVd = 0;
#endif
  }
  else
  {
//        if( ++count == SPEEDDELAY ) 
//        {
//			//VelReq = FracMpy(ReadADCParm.qADValue,Q15(OMEGANOMINAL-OMEGA0)) + Q15(OMEGA0);
//            VelReq = Q15(OMEGANOMINAL);
//            
//            pmsm_mc_param.vel_ref = VelReq;
//		}
//        if(pmsm_mc_param.change_mode)
//        {
//            pmsm_mc_param.change_mode = 0;
//            Startup_Lock = 0;
//            Startup_Ramp = 0;
//            
//            pmsm_mc_param.vel_ref = Q15(OMEGA0);
//            pmsm_foc_param.pi_w.sum = (long)pmsm_mc_param.iq_ref << 14;
//            pmsm_mc_param.id_ref = ID_REF;
//        }  

#ifdef TORQUE_MODE
        pmsm_mc_param.iq_ref = REFINAMPS(INITIALTORQUE);
        pmsm_mc_param.id_ref = ID_REF;
#else
        if(AccumThetaCnt == 0)
        {
            /* Execute the velocity control loop */
            pmsm_foc_param.pi_w.meas = smc1.Omega;
            pmsm_foc_param.pi_w.ref = pmsm_mc_param.vel_ref;
            calc_pi(&pmsm_foc_param.pi_w);
            pmsm_mc_param.iq_ref = pmsm_foc_param.pi_w.out;
        }
#endif
        
        // PI control for D
        pmsm_foc_param.pi_d.meas = pmsm_foc_param.id;
        pmsm_foc_param.pi_d.ref = pmsm_mc_param.id_ref;
        calc_pi(&pmsm_foc_param.pi_d);
        pmsm_foc_param.vd = pmsm_foc_param.pi_d.out;

        // Vector limitation
        // Vd is not limited
        // Vq is limited so the vector Vs is less than a maximum of 95%.
        // The 5% left is needed to be able to measure current through
        // shunt resistors.
        // Vs = SQRT(Vd^2 + Vq^2) < 0.95
        // Vq = SQRT(0.95^2 - Vd^2)
        pmsm_mc_param.vd_squared = pmsm_foc_param.pi_d.out * pmsm_foc_param.pi_d.out;
        pmsm_foc_param.pi_q.out_max = sqrt_q15(Q15(MAXDUTYE)*Q15(MAXDUTYE)- pmsm_mc_param.vd_squared);		
        pmsm_foc_param.pi_q.out_min = -pmsm_foc_param.pi_q.out_max;

        // PI control for Q
        pmsm_foc_param.pi_q.meas = pmsm_foc_param.iq;
        pmsm_foc_param.pi_q.ref = pmsm_mc_param.iq_ref;
        calc_pi(&pmsm_foc_param.pi_q);
        pmsm_foc_param.vq = pmsm_foc_param.pi_q.out;
	}
}

void pmsm_foc_run(void)
{
    if(pmsm_mc_param.run_motor)
    {
        clark(&pmsm_foc_param);
        park(&pmsm_foc_param);

        CalculateParkAngle();

        DoControl();

        sin_cos(&pmsm_foc_param);
        inv_park(&pmsm_foc_param);
        svpwm_gen(&pmsm_foc_param);
    }
}

//---------------------------------------------------------------------

static void CalculateParkAngle(void)
{
    Observer_Inputs_t STO_Inputs; /*  only if sensorless main*/

	smc1.Ialpha = pmsm_foc_param.ialpha;
	smc1.Ibeta = pmsm_foc_param.ibeta;
	smc1.Valpha = pmsm_foc_param.valpha;
	smc1.Vbeta = pmsm_foc_param.vbeta;

	SMC_Position_Estimation(&smc1);
    
    STO_Inputs.Valfa_beta.alpha = pmsm_foc_param.valpha;
    STO_Inputs.Valfa_beta.beta = pmsm_foc_param.vbeta;
    STO_Inputs.Ialfa_beta.alpha = pmsm_foc_param.ialpha;
    STO_Inputs.Ialfa_beta.beta = pmsm_foc_param.ibeta;
    STO_Inputs.Vbus = 23830;            //12位ADC转成U16格式
    STO_PLL_CalcElAngle(&STO_PLL_M1, &STO_Inputs);        /* 龙伯格观测器 */

	if(pmsm_mc_param.openloop)	
	{
        if (Startup_Lock < pmsm_mc_param.lock_time)
            Startup_Lock += 1;
        else if (Startup_Ramp < pmsm_mc_param.end_speed)
            Startup_Ramp += DELTA_STARTUP_RAMP;
        else
        {
#ifndef OPEN_LOOP_MODE
            pmsm_mc_param.change_mode = 1;
            pmsm_mc_param.openloop = 0;
            //Theta_error = pmsm_foc_param.angle - STO_PLL_M1._Super.hElAngle;
            Theta_error = pmsm_foc_param.angle - smc1.Theta;
#endif
        }
        pmsm_foc_param.angle += (int16_t)(Startup_Ramp >> 16);			
	}
	else
	{
		//pmsm_foc_param.angle = STO_PLL_M1._Super.hElAngle + Theta_error;
        pmsm_foc_param.angle = smc1.Theta + Theta_error;
		if (_Q15abs(Theta_error) > _0_05DEG)
		{
			if (Theta_error < 0)
				Theta_error += _0_05DEG;
			else
				Theta_error -= _0_05DEG;
		}
	}
	return;
}



/**
  * @brief parameters init
  * @param None
  * @retval None
  * @note
  */
static void pmsm_foc_init_control_parameters(void)
{
	pmsm_foc_param.pi_d.kp = DKP;
	pmsm_foc_param.pi_d.ki = DKI;
	pmsm_foc_param.pi_d.kc = DKC;
	pmsm_foc_param.pi_d.out_max = DOUTMAX;
	pmsm_foc_param.pi_d.out_min = -pmsm_foc_param.pi_d.out_max;

	init_pi(&pmsm_foc_param.pi_d);

	pmsm_foc_param.pi_q.kp = QKP;
	pmsm_foc_param.pi_q.ki = QKI;
	pmsm_foc_param.pi_q.kc = QKC;
	pmsm_foc_param.pi_q.out_max = QOUTMAX;
	pmsm_foc_param.pi_q.out_min = -pmsm_foc_param.pi_q.out_max;

	init_pi(&pmsm_foc_param.pi_q);
    
	pmsm_foc_param.pi_w.kp = WKP;
	pmsm_foc_param.pi_w.ki = WKI;
	pmsm_foc_param.pi_w.kc = WKC;
	pmsm_foc_param.pi_w.out_max = WOUTMAX;
	pmsm_foc_param.pi_w.out_min = -pmsm_foc_param.pi_w.out_max;
	
	init_pi(&pmsm_foc_param.pi_w);
    
    pmsm_mc_param.end_speed = ENDSPEEDOPENLOOP * POLEPAIRS * LOOPTIMEINSEC * 65536 * 65536 / 60.0;
    pmsm_mc_param.lock_time = LOCKTIME;
}
