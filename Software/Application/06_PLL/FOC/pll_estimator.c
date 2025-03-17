#include "pll_estimator.h"
#include "foc.h"

void mcLib_ParkTransform(mcParam_AlphaBeta *alphabetaParam , mcParam_SinCos *scParam, mcParam_DQ *dqParam)
{
    dqParam->d =  alphabetaParam->alpha*scParam->Cos + alphabetaParam->beta*scParam->Sin;
    dqParam->q = -alphabetaParam->alpha*scParam->Sin + alphabetaParam->beta*scParam->Cos;
}

// *****************************************************************************
// *****************************************************************************
// Section: MC PLL Estimator Routines
// *****************************************************************************
// *****************************************************************************
void mcLib_PLLEstimator(mcParam_PLLEstimator *pllestimatorParam)
{
	float tempqVelEstim;
    mcParam_DQ BEMF_DQParam;
    mcParam_AlphaBeta BEMF_AlphaBetaParam;
    mcParam_SinCos scParam;
    
    if(pllestimatorParam->qVelEstim < 0)
    {
        tempqVelEstim = pllestimatorParam->qVelEstim * (-1);    
    }
    else
    {
        tempqVelEstim = pllestimatorParam->qVelEstim;
    }
    
	pllestimatorParam->qDIalpha	=	(pllestimatorParam->qIalpha-pllestimatorParam->qLastIalpha);
    pllestimatorParam->qVIndalpha = (pllestimatorParam->qLsDt * pllestimatorParam->qDIalpha);
    pllestimatorParam->qDIbeta	=	(pllestimatorParam->qIbeta-pllestimatorParam->qLastIbeta);
    pllestimatorParam->qVIndbeta= (pllestimatorParam->qLsDt * pllestimatorParam->qDIbeta);
    
    // Update LastIalpha and LastIbeta
    pllestimatorParam->qLastIalpha	=	pllestimatorParam->qIalpha;
    pllestimatorParam->qLastIbeta 	=	pllestimatorParam->qIbeta;
    
    // Stator voltage equations
 	pllestimatorParam->qEsa = BEMF_AlphaBetaParam.alpha = pllestimatorParam->qLastValpha -
							((pllestimatorParam->qRs  * pllestimatorParam->qIalpha))
							- pllestimatorParam->qVIndalpha;
							
 	pllestimatorParam->qEsb = BEMF_AlphaBetaParam.beta	= 	pllestimatorParam->qLastVbeta -
							((pllestimatorParam->qRs  * pllestimatorParam->qIbeta ))
							- pllestimatorParam->qVIndbeta;
    
    // Update LastValpha and LastVbeta
	pllestimatorParam->qLastValpha = (pllestimatorParam->qMaxPhaseVoltage * pllestimatorParam->qValpha);
	pllestimatorParam->qLastVbeta = (pllestimatorParam->qMaxPhaseVoltage * pllestimatorParam->qVbeta);

    // Calculate Sin(Rho) and Cos(Rho)
    scParam.Angle 	=	pllestimatorParam->qRho + pllestimatorParam->RhoOffset; 
    scParam.Cos = arm_cos_f32(scParam.Angle);
    scParam.Sin = arm_sin_f32(scParam.Angle);

    // Translate Back EMF (Alpha,beta)  ESA, ESB to Back EMF(D,Q) ESD, ESQ using Park Transform. 

    mcLib_ParkTransform(&BEMF_AlphaBetaParam , &scParam, &BEMF_DQParam);
    
     pllestimatorParam->qEsd = BEMF_DQParam.d;
     pllestimatorParam->qEsq = BEMF_DQParam.q;
    // Filter first order for Esd and Esq
	pllestimatorParam->qEsdf			= pllestimatorParam->qEsdf+
							  ((pllestimatorParam->qEsd - pllestimatorParam->qEsdf) * pllestimatorParam->qKfilterEsdq) ;

	pllestimatorParam->qEsqf			= pllestimatorParam->qEsqf+
							  ((pllestimatorParam->qEsq - pllestimatorParam->qEsqf) * pllestimatorParam->qKfilterEsdq) ;


    pllestimatorParam->qInvKFi = pllestimatorParam->qInvKFi_Below_Nominal_Speed;
   
	if (tempqVelEstim>pllestimatorParam->qDecimate_Nominal_Speed)
    {
    	if(pllestimatorParam->qEsqf>0)
    	{
    		pllestimatorParam->qOmegaMr	=	((pllestimatorParam->qInvKFi*(pllestimatorParam->qEsqf- pllestimatorParam->qEsdf))) ;
    	} 
		else
    	{
    		pllestimatorParam->qOmegaMr	=	((pllestimatorParam->qInvKFi*(pllestimatorParam->qEsqf + pllestimatorParam->qEsdf)));
    	}
    } 
	else // if est speed<10% => condition VelRef<>0
    {
    	if(pllestimatorParam->qVelEstim>0)
    	{
    		pllestimatorParam->qOmegaMr	=	((pllestimatorParam->qInvKFi*(pllestimatorParam->qEsqf- pllestimatorParam->qEsdf))) ;
    	} 
		else
    	{
    		pllestimatorParam->qOmegaMr	=	((pllestimatorParam->qInvKFi*(pllestimatorParam->qEsqf+ pllestimatorParam->qEsdf))) ;
    	}
    }
    	
	// The integral of the angle is the estimated angle */
	pllestimatorParam->qRho	= 	pllestimatorParam->qRho+
							(pllestimatorParam->qOmegaMr)*(pllestimatorParam->qDeltaT);
    
    if(pllestimatorParam->qRho >= ANGLE_2PI)
        pllestimatorParam->qRho = pllestimatorParam->qRho - ANGLE_2PI;      

     if(pllestimatorParam->qRho <= 0)
        pllestimatorParam->qRho = pllestimatorParam->qRho + ANGLE_2PI; 
    

    // The estimated speed is a filter value of the above calculated OmegaMr. The filter implementation
    // is the same as for BEMF d-q components filtering
	pllestimatorParam->qVelEstim = (pllestimatorParam->qVelEstim+
						( (pllestimatorParam->qOmegaMr-pllestimatorParam->qVelEstim)*pllestimatorParam->qVelEstimFilterK ));
}

