/*
*******************************************************************************************************
*
* File Name : pmsm.c
* Version   : V1.0
* Author    : mzy2364
* brief     : pmsm driver file
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "pmsm.h"
#include "foc.h"
#include "userparms.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/
pmsm_foc_t pmsm_foc_param;
pmsm_mc_t pmsm_mc_param;

/* FUNCTION -----------------------------------------------------------------------------------------*/
static void pmsm_foc_init_control_parameters(void);
static void calculate_park_angle(void);

/**
  * @brief 
  * @param None
  * @retval None
  * @note
  */
void pmsm_foc_init(void)
{
	/* SMC Init */
	
	pmsm_foc_init_control_parameters();
	
	pmsm_mc_param.run_motor = 1;
	pmsm_mc_param.openloop = 1;
	pmsm_mc_param.change_mode = 1;
	pmsm_mc_param.startup_lock_count = 0;
	pmsm_mc_param.startup_ramp = 0;
}

/**
  * @brief FOC Loop function
  * @param None
  * @retval None
  * @note
  */
void pmsm_foc_run(void)
{
    pmsm_foc_param.vd = 0;
    pmsm_foc_param.vq = OPEN_LOOP_VF_VQ;
    
	calculate_park_angle();
	
	sin_cos(&pmsm_foc_param);
	
	inv_park(&pmsm_foc_param);
	
	svpwm(&pmsm_foc_param);
}

/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
/**
  * @brief parameters init
  * @param None
  * @retval None
  * @note
  */
static void pmsm_foc_init_control_parameters(void)
{

}


/**
  * @brief Angle calculation
  * @param None
  * @retval None
  * @note
  */
static void calculate_park_angle(void)
{
	if(pmsm_mc_param.openloop)
	{
		/* begin with the lock sequence, for field alignment */
		if(pmsm_mc_param.startup_lock_count < LOCK_COUNTER)
		{
			pmsm_mc_param.startup_lock_count++;
		}
		else if(pmsm_mc_param.startup_ramp < END_SPEED_RADS_ELEC_COUNTER)
		{
			pmsm_mc_param.startup_ramp += OPENLOOP_RAMPSPEED_INCREASERATE;
		}
		else
		{
#ifndef OPEN_LOOP_MODE
			pmsm_mc_param.change_mode = 1;
			pmsm_mc_param.openloop = 0;
#endif
		}
		pmsm_foc_param.angle += pmsm_mc_param.startup_ramp;
        if(pmsm_foc_param.angle > ANGLE_2PI)
            pmsm_foc_param.angle = pmsm_foc_param.angle - ANGLE_2PI;
	}
	else
	{
	}
}
/***************************************** (END OF FILE) *********************************************/
