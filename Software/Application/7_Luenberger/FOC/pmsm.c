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
#include "luenberger.h"
#include "userparms.h"
#include "motor_hardware.h"
/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/
pmsm_foc_t pmsm_foc_param;
pmsm_mc_t pmsm_mc_param;
sto_luenberger_t sto1;

float theta_error = 0;
float iq_error = 0;
uint32_t iq_error_count = 0;

/* FUNCTION -----------------------------------------------------------------------------------------*/
static void pmsm_foc_init_control_parameters(void);
static void calculate_park_angle(void);
static void pid_control(void);

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
    
    pmsm_mc_param.vel_input = NOMINAL_SPEED_RAD_PER_SEC_ELEC;
    
    sto_luenberger_init(&sto1);
}

/**
  * @brief FOC Loop function
  * @param None
  * @retval None
  * @note
  */
void pmsm_foc_run(void)
{
    if(pmsm_mc_param.run_motor)
    {
        clark(&pmsm_foc_param);
        park(&pmsm_foc_param);
        
        sto1.ialpha = pmsm_foc_param.ialpha;
        sto1.ibeta = pmsm_foc_param.ibeta;
        sto1.valpha = pmsm_foc_param.valpha * pmsm_mc_param.max_phase_voltage;
        sto1.vbeta = pmsm_foc_param.vbeta * pmsm_mc_param.max_phase_voltage;
        sto_luenberger_estimation(&sto1);
        pmsm_mc_param.actual_speed = sto1.omega;
        
        calculate_park_angle();
        
        pid_control();
        
        sin_cos(&pmsm_foc_param);
        
        inv_park(&pmsm_foc_param);
        
        svpwm(&pmsm_foc_param);
    }
    else
    {
        timer_channel_output_state_config(MOTOR_PWM_TIMER,PWM_U_CHANNEL,TIMER_CCX_DISABLE);
        timer_channel_output_state_config(MOTOR_PWM_TIMER,PWM_V_CHANNEL,TIMER_CCX_DISABLE);
        timer_channel_output_state_config(MOTOR_PWM_TIMER,PWM_W_CHANNEL,TIMER_CCX_DISABLE);
        timer_channel_complementary_output_state_config(MOTOR_PWM_TIMER,PWM_U_CHANNEL,TIMER_CCXN_DISABLE);
        timer_channel_complementary_output_state_config(MOTOR_PWM_TIMER,PWM_V_CHANNEL,TIMER_CCXN_DISABLE);
        timer_channel_complementary_output_state_config(MOTOR_PWM_TIMER,PWM_W_CHANNEL,TIMER_CCXN_DISABLE);
    }
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
            theta_error = pmsm_foc_param.angle - sto1.theta;
#ifndef OPEN_LOOP_MODE
            if((pmsm_foc_param.angle < M_PI) && (sto1.theta < M_PI))
            {
                pmsm_mc_param.change_mode = 1;
                pmsm_mc_param.openloop = 0;
            }
#endif
		}
		pmsm_foc_param.angle += pmsm_mc_param.startup_ramp;
        if(pmsm_foc_param.angle > ANGLE_2PI)
            pmsm_foc_param.angle = pmsm_foc_param.angle - ANGLE_2PI;
	}
	else
	{
        pmsm_foc_param.angle = sto1.theta + theta_error;
        if((fabsf(theta_error) > 0.01f))
        {
            if (theta_error < 0)
            {
                theta_error += 0.01f;
            }
            else
            {
                theta_error -= 0.01f;
            }
        }
        else
        {
            theta_error = 0;
        }
//        if(sto1.theta_offset > ((90 * (M_PI/180)) / 65535))
//        {
//            sto1.theta_offset -= ((90 * (M_PI/180)) / 65535);
//        }
//        else
//        {
//            sto1.theta_offset = 0;
//        }
	}
}

/**
  * @brief PID Control
  * @param None
  * @retval None
  * @note
  */
static void pid_control(void)
{
	if(pmsm_mc_param.openloop)
	{
		if(pmsm_mc_param.change_mode)
		{
			pmsm_mc_param.change_mode = 0;
            pmsm_mc_param.id_ref = 0;
		}
        
        pmsm_mc_param.iq_ref = OPEN_LOOP_CURRENT;
        
        // PI control for Q
        pmsm_foc_param.pi_q.meas = pmsm_foc_param.iq;
        pmsm_foc_param.pi_q.ref  = pmsm_mc_param.iq_ref;
        calc_pi(&pmsm_foc_param.pi_q);
        pmsm_foc_param.vq = pmsm_foc_param.pi_q.out;
       
        // PI control for D
        pmsm_foc_param.pi_d.meas = pmsm_foc_param.id;
        pmsm_foc_param.pi_d.ref  = pmsm_mc_param.id_ref;
        calc_pi(&pmsm_foc_param.pi_d);
        pmsm_foc_param.vd = pmsm_foc_param.pi_d.out;
        
#ifdef OPEN_LOOP_VF_VQ
        pmsm_foc_param.vq = OPEN_LOOP_VF_VQ;
        pmsm_foc_param.vd = 0;
#endif
        
	}
	else
	{
        float vd_squared = 0;
        float vq_squared_max = 0;

		if(pmsm_mc_param.change_mode)
		{
            pmsm_mc_param.change_mode = 0;
            pmsm_mc_param.vel_ref = END_SPEED_RADS_ELEC;
            pmsm_mc_param.id_ref = 0;
            pmsm_foc_param.pi_w.sum = OPEN_LOOP_CURRENT;
            pmsm_mc_param.speed_loop_count = SPEED_LOOP_CYCLE;
		}
        pmsm_mc_param.vel_diff =  pmsm_mc_param.vel_input - pmsm_mc_param.vel_ref;

       //Speed Rate Limiter implementation.    
        if(pmsm_mc_param.vel_diff >= CL_SPEED_HYSTERESIS)
        {
             pmsm_mc_param.vel_ref += CL_SPEED_RAMP_RATE_DELTA;   
        }
        else if(pmsm_mc_param.vel_diff <=- CL_SPEED_HYSTERESIS)
        {
             pmsm_mc_param.vel_ref -= CL_SPEED_RAMP_RATE_DELTA; 
        }
        else
        {
             pmsm_mc_param.vel_ref = pmsm_mc_param.vel_input;
        }
        
#ifdef TORQUE_MODE
//        pmsm_mc_param.iq_ref = OPEN_LOOP_CURRENT;
#else
        if(pmsm_mc_param.speed_loop_count++ >= SPEED_LOOP_CYCLE)
        {
            pmsm_mc_param.speed_loop_count = 0;
            // PI control for SPEED
            pmsm_foc_param.pi_w.meas = sto1.omega;
            pmsm_foc_param.pi_w.ref  = pmsm_mc_param.vel_ref;
            calc_pi(&pmsm_foc_param.pi_w);
            pmsm_mc_param.iq_ref = pmsm_foc_param.pi_w.out;
        }
#endif
        iq_error = pmsm_mc_param.iq_ref - pmsm_foc_param.iq;
        if(iq_error > 1.0f)
        {
            iq_error_count++;
            if(iq_error_count >= 2000)
            {
                iq_error_count = 0;
                pmsm_foc_param.pi_w.out_max = pmsm_foc_param.pi_w.out_max - 0.5f;
            }
        }
        else
        {
            if(iq_error_count > 0)
            {
                iq_error_count--;
            }
            else
            {
                if(pmsm_foc_param.pi_w.out_max < WOUTMAX)
                {
                    pmsm_foc_param.pi_w.out_max = pmsm_foc_param.pi_w.out_max + 0.5f;
                }
                pmsm_foc_param.pi_w.out_min = -pmsm_foc_param.pi_w.out_max;
            }
        }
            
        // PI control for D
        pmsm_foc_param.pi_d.meas = pmsm_foc_param.id;
        pmsm_foc_param.pi_d.ref  = pmsm_mc_param.id_ref;
        calc_pi(&pmsm_foc_param.pi_d);
        pmsm_foc_param.vd = pmsm_foc_param.pi_d.out;
        
        vd_squared = pmsm_foc_param.vd * pmsm_foc_param.vd;
        vq_squared_max = 0.99f - vd_squared;
		arm_sqrt_f32(vq_squared_max,&pmsm_foc_param.pi_q.out_max);
        pmsm_foc_param.pi_q.out_min = -pmsm_foc_param.pi_q.out_max;

        // PI control for Q
        pmsm_foc_param.pi_q.meas = pmsm_foc_param.iq;
        pmsm_foc_param.pi_q.ref  = pmsm_mc_param.iq_ref;
        calc_pi(&pmsm_foc_param.pi_q);
        pmsm_foc_param.vq = pmsm_foc_param.pi_q.out;
	}
}

/***************************************** (END OF FILE) *********************************************/
