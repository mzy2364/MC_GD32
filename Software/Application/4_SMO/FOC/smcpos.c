/*
*******************************************************************************************************
*
* File Name : smcpos.c
* Version   : V1.0
* Author    : mzy2364
* brief     : PMSM SMO control file
* 
*******************************************************************************************************
*/

/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "smcpos.h"
#include "foc.h"
#include "userparms.h"

/* DEFINES ------------------------------------------------------------------------------------------*/

/* VARIABLES ----------------------------------------------------------------------------------------*/


/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
static void calc_pll(smc_t *psmc);

/* GLOBAL FUNCTION ----------------------------------------------------------------------------------*/

/**
  * @brief Initialises the SMC structure
  * @param *psmc -  Pointer to SMC structure
  * @retval void
  * @note
  */
void smc_init_parameters(smc_t *psmc)
{
    psmc->valpha = 0;
    psmc->ealpha = 0;
    psmc->ealpha_final = 0;
    psmc->zalpha = 0;
    psmc->gsmopos = 0;
    psmc->est_ialpha = 0;
    psmc->fsmopos = 0;
    psmc->vbeta = 0;
    psmc->ebeta = 0;
    psmc->ebeta_final = 0;
    psmc->zbeta = 0;
    psmc->est_ibeta = 0;
    psmc->ialpha = 0;
    psmc->ialpha_error = 0;
    psmc->kslide = 0;
    psmc->max_smc_error = 0;
    psmc->ibeta = 0;
    psmc->ibeta_error = 0;
    psmc->kslf = 0;
    psmc->kslf_final = 0;
    psmc->filt_omega_coef = 0;
    psmc->theta_offset = 0;
    psmc->theta = 0;
    psmc->omega = 0;
    psmc->omega_fltred = 0;
    psmc->theta_error = 0;
    
    if(MOTOR_PHASE_RESISTANCE * LOOPTIME_SEC > MOTOR_PHASE_INDUCTANCE)
    {
        psmc->fsmopos = 0;
    }
    else
    {
        psmc->fsmopos = 1 - MOTOR_PHASE_RESISTANCE * LOOPTIME_SEC / MOTOR_PHASE_INDUCTANCE;
    }
    if(LOOPTIME_SEC > MOTOR_PHASE_INDUCTANCE)
    {
        psmc->gsmopos = 0.9999f;
    }
    else
    {
        psmc->gsmopos = LOOPTIME_SEC / MOTOR_PHASE_INDUCTANCE;
    }
    psmc->kslide = MA_SMCGAIN;
    psmc->max_smc_error = MA_MAXLINEARSMC;
    psmc->filt_omega_coef = LOOPTIME_SEC * M_PI * 2 * (OPEN_LOOP_END_SPEED_RPM * MOTOR_NOPOLESPAIRS / 60);  //T*2¦Ð*fc  fc = erps
    psmc->kslf = psmc->filt_omega_coef;
    
    psmc->pll_err_sum = 0;
    psmc->pll_kp = 20;
    psmc->pll_ki = 0.1;

}


/**
  * @brief Motor speed and angle estimator
  * @param *psmc -  Pointer to SMC structure
  * @retval void
  * @note
  */
void smc_position_estimation (smc_t *psmc)
{
    /* Sliding mode current observer */
    psmc->est_ialpha = psmc->fsmopos * psmc->est_ialpha + \
                       psmc->gsmopos * (psmc->valpha- psmc->ealpha - psmc->zalpha);
    psmc->est_ibeta = psmc->fsmopos * psmc->est_ibeta + \
                       psmc->gsmopos * (psmc->vbeta- psmc->ebeta - psmc->zbeta);
    
    /* Current errors */
	psmc->ialpha_error = psmc->est_ialpha - psmc->ialpha;
	psmc->ibeta_error = psmc->est_ibeta - psmc->ibeta;
    
    /* Sliding control calculator */
    if(ABS(psmc->ialpha_error) < psmc->max_smc_error)
    {
        psmc->zalpha = psmc->kslide * psmc->ialpha_error / psmc->max_smc_error;
    }
	else
    {
		psmc->zalpha = (psmc->ialpha_error > 0)?psmc->kslide:-psmc->kslide;
    }
    
    if(ABS(psmc->ibeta_error) < psmc->max_smc_error)
    {
        psmc->zbeta = psmc->kslide * psmc->ibeta_error / psmc->max_smc_error;
    }
	else
    {
		psmc->zbeta = (psmc->ibeta_error > 0) ? psmc->kslide : -psmc->kslide;
    }
    
    /* Sliding control filter -> back EMF calculator */
    psmc->ealpha = psmc->ealpha + psmc->kslf * (psmc->zalpha - psmc->ealpha);
    psmc->ebeta = psmc->ebeta + psmc->kslf * (psmc->zbeta - psmc->ebeta);
    
    /* pll omega and theta calculator */
    calc_pll(psmc);
}

/*==================================================================================================
*                                       LOCAL FUNCTIONS
==================================================================================================*/

static void calc_pll(smc_t *psmc)
{
    float sin_value = 0;
    float cos_value = 0;
    
    cos_value = arm_cos_f32(psmc->theta);
    sin_value = arm_sin_f32(psmc->theta);
    
    psmc->pll_err = -psmc->ealpha * cos_value - psmc->ebeta * sin_value;

    psmc->pll_err = (psmc->pll_err > PI/6)  ?  (PI/6) : (psmc->pll_err);
    psmc->pll_err = (psmc->pll_err < -PI/6) ? (-PI/6) : (psmc->pll_err);
    
    psmc->pll_err_sum += psmc->pll_err;
    
    psmc->omega = psmc->pll_err * psmc->pll_kp + psmc->pll_err_sum * psmc->pll_ki;
    
    psmc->theta	= psmc->theta + (psmc->omega)*(LOOPTIME_SEC);
    if(psmc->theta >= ANGLE_2PI)
    {
        psmc->theta = psmc->theta - ANGLE_2PI;
    }
    if(psmc->theta <= 0)
    {
        psmc->theta = psmc->theta + ANGLE_2PI;
    }
    psmc->omega_fltred = psmc->omega_fltred + psmc->filt_omega_coef * (psmc->omega - psmc->omega_fltred);
}

/************************************************EOF************************************************/
