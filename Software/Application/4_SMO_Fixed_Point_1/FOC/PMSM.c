#define INITIALIZE
#include "system_define.h"


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

uGFt uGF;

tPIParm     PIParmD;	// Structure definition for Flux component of current, or Id
tPIParm     PIParmQ;	// Structure definition for Torque component of current, or Iq
tPIParm     PIParmW;	// Structure definition for Speed, or Omega

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

void SetupControlParameters(void);

/************* START OF MAIN FUNCTION ***************/
void SensorlessFOCinit(void)
{
	SMCInit(&smc1);
	SetupControlParameters(); 
	uGF.Word = 0;                   // clear flags
    uGF.bit.OpenLoop = 1;
    uGF.bit.RunMotor = 1;
}

//---------------------------------------------------------------------
// Executes one PI itteration for each of the three loops Id,Iq,Speed,

void DoControl( void )
{
    if( uGF.bit.OpenLoop )
    {
        if( uGF.bit.ChangeMode )
        {
            // just changed to openloop
            uGF.bit.ChangeMode = 0;
            // synchronize angles
            // VqRef & VdRef not used
            CtrlParm.qVqRef = 0;
            CtrlParm.qVdRef = 0;
            CtrlParm.qVelRef = 0;
            Startup_Lock = 0;
            CatchSpeed_Cnt=0;
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

            PIParmW.qdSum=0;
            PIParmD.qdSum=0;
            PIParmQ.qdSum=0;
        }
        CtrlParm.qVqRef = REFINAMPS(INITIALTORQUE);

        if(AccumThetaCnt == 0)
        {
            PIParmW.qInMeas = smc1.Omega;
        }

        // PI control for D
        PIParmD.qInMeas = ParkParm.qId;
        PIParmD.qInRef  = CtrlParm.qVdRef;
        CalcPI(&PIParmD);
        ParkParm.qVd    = PIParmD.qOut;

        qVdSquared=PIParmD.qOut*PIParmD.qOut;				//int32_t  
        PIParmQ.qOutMax = _Q15sqrt( Q15(MAXDUTYE)*Q15(MAXDUTYE)- qVdSquared);		
        PIParmQ.qOutMin = -PIParmQ.qOutMax;
            
        // PI control for Q
        PIParmQ.qInMeas = ParkParm.qIq;
        PIParmQ.qInRef  = CtrlParm.qVqRef;
        CalcPI(&PIParmQ);
        ParkParm.qVq    = PIParmQ.qOut;
#ifdef OPEN_LOOP_VF_VQ
        ParkParm.qVq = OPEN_LOOP_VF_VQ;
        ParkParm.qVd = 0;
#endif
  }
  else
  {
        if( ++count == SPEEDDELAY ) 
        {
			//VelReq = FracMpy(ReadADCParm.qADValue,Q15(OMEGANOMINAL-OMEGA0)) + Q15(OMEGA0);
            VelReq = Q15(OMEGANOMINAL);          
			
//			if (CtrlParm.qVelRef <= VelReq) //normal speed
//			{
//			     CtrlParm.qVelRef += 1; 
//			}
//			else CtrlParm.qVelRef -= 1 ; 
//	 		count = 0;
            
            CtrlParm.qVelRef = VelReq;
		}
        if( uGF.bit.ChangeMode )
        {
            uGF.bit.ChangeMode = 0;
            PIParmW.qdSum = (long)CtrlParm.qVqRef << 14;
            Startup_Lock = 0;
            Startup_Ramp = 0;
            CtrlParm.qVelRef = Q15(OMEGA0);
        }  

        if(AccumThetaCnt == 0)
        {
            PIParmW.qInMeas = smc1.Omega;
            PIParmW.qInRef  = CtrlParm.qVelRef;
            CalcPI(&PIParmW);
            CtrlParm.qVqRef = PIParmW.qOut;
        }
		CtrlParm.qVdRef = ID_REF;

        // PI control for D
        PIParmD.qInMeas = ParkParm.qId;
        PIParmD.qInRef  = CtrlParm.qVdRef;
        CalcPI(&PIParmD);
        ParkParm.qVd = PIParmD.qOut;

		qVdSquared=ParkParm.qVd*ParkParm.qVd;
		PIParmQ.qOutMax = _Q15sqrt( Q15(MAXDUTYE)*Q15(MAXDUTYE)- qVdSquared);		
		PIParmQ.qOutMin = -PIParmQ.qOutMax;

        // PI control for Q
        PIParmQ.qInMeas = ParkParm.qIq;
        PIParmQ.qInRef  = CtrlParm.qVqRef;
        CalcPI(&PIParmQ);
        ParkParm.qVq = PIParmQ.qOut;
	}
}

void SensorlessFOCRUN(void)
{
    if( uGF.bit.RunMotor )
    {
        ClarkePark(&ParkParm);
        
        CalculateParkAngle();

        DoControl();

        SinCos(&ParkParm);

        InvPark(&ParkParm);
        CalcSVGen(&ParkParm);
    }
}

//---------------------------------------------------------------------

void CalculateParkAngle(void)
{
	smc1.Ialpha = ParkParm.qIalpha;
	smc1.Ibeta = ParkParm.qIbeta;
	smc1.Valpha = ParkParm.qValpha;
	smc1.Vbeta = ParkParm.qVbeta;

	SMC_Position_Estimation(&smc1);

	if(uGF.bit.OpenLoop)	
	{
        if (Startup_Lock < MotorParm.LockTime)
            Startup_Lock += 1;
        else if (Startup_Ramp < MotorParm.EndSpeed)
            Startup_Ramp += DELTA_STARTUP_RAMP;
        else
        {
#ifndef OPEN_LOOP_MODE
            uGF.bit.ChangeMode = 1;
            uGF.bit.OpenLoop = 0;
            Theta_error = ParkParm.qAngle - smc1.Theta;
#endif
        }
        ParkParm.qAngle += (int16_t)(Startup_Ramp >> 16);			
	}
	else
	{
		ParkParm.qAngle = smc1.Theta + Theta_error;
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

void SetupControlParameters(void)
{

// ============= PI D Term ===============      
    PIParmD.qKp = DKP;       
    PIParmD.qKi = DKI;              
    PIParmD.qKc = DKC;       
    PIParmD.qOutMax = DOUTMAX;
    PIParmD.qOutMin = -PIParmD.qOutMax;

    InitPI(&PIParmD);

// ============= PI Q Term ===============
    PIParmQ.qKp = QKP;    
    PIParmQ.qKi = QKI;
    PIParmQ.qKc = QKC;
    PIParmQ.qOutMax = QOUTMAX;
    PIParmQ.qOutMin = -PIParmQ.qOutMax;

    InitPI(&PIParmQ);

// ============= PI W Term ===============
    PIParmW.qKp = WKP;       
    PIParmW.qKi = WKI;       
    PIParmW.qKc = WKC;       
    PIParmW.qOutMax = WOUTMAX;   
    PIParmW.qOutMin = -PIParmW.qOutMax;

    InitPI(&PIParmW);
    
	MotorParm.EndSpeed = ENDSPEEDOPENLOOP * POLEPAIRS * LOOPTIMEINSEC * 65536 * 65536 / 60.0;
	MotorParm.LockTime = LOCKTIME;
    
	return;
}
