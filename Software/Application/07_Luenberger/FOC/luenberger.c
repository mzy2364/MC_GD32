/*
*******************************************************************************************************
*
* File Name : luenberger.c
* Version   : V1.0
* Author    : mzy2364
* brief     : PMSM Luenberger control file
* 
*******************************************************************************************************
*/

/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "luenberger.h"
#include "foc.h"
#include "userparms.h"

/* DEFINES ------------------------------------------------------------------------------------------*/

/* VARIABLES ----------------------------------------------------------------------------------------*/


/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/


/* GLOBAL FUNCTION ----------------------------------------------------------------------------------*/
/**
  * @brief Initialises the Luenberger structure
  * @param *s -  Pointer to Luenberger structure
  * @retval void
  * @note
  */
void sto_luenberger_init(sto_luenberger_t *s)
{
    s->C1 = LOOPTIME_SEC / MOTOR_PHASE_INDUCTANCE;
    s->C2 = 1-(MOTOR_PHASE_RESISTANCE*LOOPTIME_SEC/MOTOR_PHASE_INDUCTANCE);
    s->C3 = K1 * LOOPTIME_SEC;
    s->C4 = K2 * LOOPTIME_SEC;
    
    s->ialpha_est = 0;
    s->ibeta_est = 0;
    s->ealpha_est = 0;
    s->ebeta_est = 0;
    
    s->pll_pi_regulator.kp = 0.004;
    s->pll_pi_regulator.ki = 0.0004;
    s->pll_pi_regulator.kp = 60;
    s->pll_pi_regulator.ki = 0.1;
    s->pll_pi_regulator.kc = 0;
    s->pll_pi_regulator.out_max = 100000 / 60 * ANGLE_2PI;
    s->pll_pi_regulator.out_min = -s->pll_pi_regulator.out_max;
    init_pi(&s->pll_pi_regulator);
}

/**
  * @brief Call Luenberger observer to computing apeed and electrical angle
  * @param *s -  Pointer to Luenberger structure
  * @retval void
  * @note
  */
void sto_luenberger_estimation(sto_luenberger_t *s)
{
    float ialpha_est_next = 0;
    float ibeta_est_next = 0;
    float ealpha_est_next = 0;
    float ebeta_est_next = 0;
    float sin_value = 0;
    float cos_value = 0;
    
    ialpha_est_next = s->C2*s->ialpha_est + s->C1*(s->valpha - s->ealpha_est) + s->C3 * (s->ialpha_est - s->ialpha);
    ibeta_est_next = s->C2*s->ibeta_est + s->C1*(s->vbeta - s->ebeta_est) + s->C3 * (s->ibeta_est - s->ibeta);
    
    ealpha_est_next = s->ealpha_est - LOOPTIME_SEC * s->omega * s->ebeta_est + s->C4 * (s->ialpha_est - s->ialpha);
    ebeta_est_next = s->ebeta_est + LOOPTIME_SEC * s->omega * s->ealpha_est + s->C4 * (s->ibeta_est - s->ibeta);
    
    s->ialpha_est = ialpha_est_next;
    s->ibeta_est = ibeta_est_next;
    s->ealpha_est = ealpha_est_next;
    s->ebeta_est = ebeta_est_next;
    
    cos_value = arm_cos_f32(s->theta);
    sin_value = arm_sin_f32(s->theta);
    s->pll_pi_regulator.ref = -s->ealpha_est * cos_value;
    s->pll_pi_regulator.meas = s->ebeta_est * sin_value;
    calc_pi(&s->pll_pi_regulator);
    
    s->omega = s->pll_pi_regulator.out;
    s->theta = s->theta + s->omega * LOOPTIME_SEC;
    if(s->theta >= ANGLE_2PI)
    {
        s->theta = s->theta - ANGLE_2PI;
    }
    if(s->theta <= 0)
    {
        s->theta = s->theta + ANGLE_2PI;
    }
}

/*==================================================================================================
*                                       LOCAL FUNCTIONS
==================================================================================================*/

//static void calc_pll(smc_t *psmc)
//{
//    float sin_value = 0;
//    float cos_value = 0;
//    
//    cos_value = arm_cos_f32(psmc->theta + psmc->theta_offset);
//    sin_value = arm_sin_f32(psmc->theta + psmc->theta_offset);
//    
//    psmc->pll_err = -psmc->ealpha_final * cos_value - psmc->ebeta_final * sin_value;

//    psmc->pll_err = (psmc->pll_err > PI/6)  ?  (PI/6) : (psmc->pll_err);
//    psmc->pll_err = (psmc->pll_err < -PI/6) ? (-PI/6) : (psmc->pll_err);
//    
//    psmc->pll_err_sum += psmc->pll_err;
//    
//    psmc->omega = psmc->pll_err * psmc->pll_kp + psmc->pll_err_sum * psmc->pll_ki;
//    
//    psmc->theta	= psmc->theta + (psmc->omega)*(LOOPTIME_SEC);
//    if(psmc->theta >= ANGLE_2PI)
//    {
//        psmc->theta = psmc->theta - ANGLE_2PI;
//    }
//    if(psmc->theta <= 0)
//    {
//        psmc->theta = psmc->theta + ANGLE_2PI;
//    }
//    psmc->omega_fltred = psmc->omega_fltred + psmc->filt_omega_coef * (psmc->omega - psmc->omega_fltred);
//}

/************************************************EOF************************************************/
