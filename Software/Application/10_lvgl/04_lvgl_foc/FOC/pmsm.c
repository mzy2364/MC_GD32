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
#include "pll_estimator.h"
#include "userparms.h"
#include "motor_hardware.h"
#include "ee_parameter.h"
/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/
pmsm_foc_t pmsm_foc_param;
pmsm_mc_t pmsm_mc_param;
mcParam_PLLEstimator mcApp_EstimParam;

float theta_error = 0;
float iq_error = 0;
uint32_t iq_error_count = 0;

/* FUNCTION -----------------------------------------------------------------------------------------*/
static void pmsm_foc_init_control_parameters(void);
static void calculate_park_angle(void);
static void pid_control(void);
static void mcApp_InitEstimParm(void);
static void hall_speed_pll(float angle);

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
    pmsm_mc_param.openloop_hold_count = 0;
	pmsm_mc_param.startup_ramp = 0;
    pmsm_mc_param.ol_state = ROTOR_LOCK;
    
//    pmsm_mc_param.vel_input = (float)motor_normal_spd*RPM_TO_RADS*motor_pole;
    
    pmsm_mc_param.lock_counter = (float)rotor_lock_time/((float)LOOPTIME_SEC * 10);
    pmsm_mc_param.hold_counter = (float)openloop_hold_time/((float)LOOPTIME_SEC * 10);
    pmsm_mc_param.open_loop_current = (float)openloop_current / 10.0f;
    pmsm_mc_param.end_speed_rads_elec_counter = (float)open_loop_speed*RPM_TO_RADS*motor_pole*LOOPTIME_SEC;
    pmsm_mc_param.openloop_rampspeed_inc = pmsm_mc_param.end_speed_rads_elec_counter/((float)openloop_ramp_time/10/LOOPTIME_SEC);
    pmsm_mc_param.end_speed = (float)open_loop_speed*RPM_TO_RADS*motor_pole;
    pmsm_mc_param.sensor = motor_sensor;
    
    mcApp_InitEstimParm();
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
        
        if(pmsm_mc_param.sensor == 0)
        {
            mcApp_EstimParam.qIalpha = pmsm_foc_param.ialpha;
            mcApp_EstimParam.qIbeta = pmsm_foc_param.ibeta;
            mcApp_EstimParam.qValpha = pmsm_foc_param.valpha;
            mcApp_EstimParam.qVbeta = pmsm_foc_param.vbeta;
            mcApp_EstimParam.qMaxPhaseVoltage = pmsm_mc_param.max_phase_voltage;
            mcLib_PLLEstimator(&mcApp_EstimParam);
            pmsm_mc_param.actual_speed = mcApp_EstimParam.qVelEstim;
            
            calculate_park_angle();
        }
        else
        {
            hall_speed_pll(pmsm_mc_param.hall_theta);
            pmsm_foc_param.angle = pmsm_mc_param.theta_pll;
            pmsm_mc_param.actual_speed = pmsm_mc_param.omega_pll_filter;
            pmsm_mc_param.openloop = 0;
        }
        
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
        switch(pmsm_mc_param.ol_state)
        {
            case ROTOR_LOCK:
                if(pmsm_mc_param.startup_lock_count < pmsm_mc_param.lock_counter)
                {
                    pmsm_mc_param.startup_lock_count++;
                }
                else
                {
                    pmsm_mc_param.ol_state = RAMP_SPEED;
                }
                break;
            case RAMP_SPEED:
                if(pmsm_mc_param.startup_ramp < pmsm_mc_param.end_speed_rads_elec_counter)
                {
                    pmsm_mc_param.startup_ramp += pmsm_mc_param.openloop_rampspeed_inc;
                }
                else
                {
                    pmsm_mc_param.ol_state = OL_HOLD;
                }
                break;
            case OL_HOLD:
                if(pmsm_mc_param.openloop_hold_count < pmsm_mc_param.hold_counter)
                {
                    pmsm_mc_param.openloop_hold_count++;
                }
                else
                {
                    pmsm_mc_param.ol_state = WAIT_CLOSELOOP;
                }
                break;
            case WAIT_CLOSELOOP:
                theta_error = pmsm_foc_param.angle - mcApp_EstimParam.qRho;
#ifndef OPEN_LOOP_MODE
                pmsm_mc_param.change_mode = 1;
                pmsm_mc_param.openloop = 0;
#endif
                break;
            default:
                break;
        }
		pmsm_foc_param.angle += pmsm_mc_param.startup_ramp;
        if(pmsm_foc_param.angle > ANGLE_2PI)
            pmsm_foc_param.angle = pmsm_foc_param.angle - ANGLE_2PI;
	}
	else
	{
        pmsm_foc_param.angle = mcApp_EstimParam.qRho + theta_error;
        if((fabsf(theta_error) > 0.05f))
        {
            if (theta_error < 0)
            {
                theta_error += 0.05f;
            }
            else
            {
                theta_error -= 0.05f;
            }
        }
        else
        {
            theta_error = 0;
        }
   	    // in closed loop slowly decrease the offset add to the estimated angle
   	    if(mcApp_EstimParam.RhoOffset>(M_PI/(float)32767))
            mcApp_EstimParam.RhoOffset = mcApp_EstimParam.RhoOffset - ((M_PI/(float)32767)) ; 
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
        
        pmsm_mc_param.iq_ref = pmsm_mc_param.open_loop_current;
        
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
            pmsm_mc_param.vel_ref = pmsm_mc_param.end_speed;
            pmsm_mc_param.id_ref = 0;
            pmsm_foc_param.pi_w.sum = pmsm_mc_param.iq_ref;
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
            pmsm_foc_param.pi_w.meas = pmsm_mc_param.actual_speed;
            pmsm_foc_param.pi_w.ref  = pmsm_mc_param.vel_ref;
            calc_pi(&pmsm_foc_param.pi_w);
            pmsm_mc_param.iq_ref = pmsm_foc_param.pi_w.out;
        }
#endif
            
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


static void mcApp_InitEstimParm(void)  
{
    float l = (float)motor_l / 1000000.0f;
    float r = (float)motor_r / 1000.0f;
    float speed = (float)motor_normal_spd*RPM_TO_RADS*motor_pole;
    float bemf = (float)motor_bemf / 1000.0f;
    
	mcApp_EstimParam.qLsDt = (float)(l/LOOPTIME_SEC);
	mcApp_EstimParam.qRs = r;
	mcApp_EstimParam.qKFi = (bemf/1.732f)/1000 * 60/ANGLE_2PI/motor_pole;
	mcApp_EstimParam.qInvKFi_Below_Nominal_Speed = (float)(1/mcApp_EstimParam.qKFi);
    mcApp_EstimParam.qLs_DIV_2_PI = motor_l/(2*M_PI);
    mcApp_EstimParam.qNominal_Speed = speed;
    mcApp_EstimParam.qDecimate_Nominal_Speed = mcApp_EstimParam.qNominal_Speed / 10.0f;
   	mcApp_EstimParam.qOmegaMr = 0;
	 
    mcApp_EstimParam.qKfilterEsdq = KFILTER_ESDQ;
    mcApp_EstimParam.qVelEstimFilterK = KFILTER_VELESTIM;

    mcApp_EstimParam.qDeltaT = LOOPTIME_SEC;
    mcApp_EstimParam.RhoOffset = (45 * (M_PI/180));
}

static void hall_speed_pll(float angle)
{
    // 1. 定义时间相关参数（需确保PWM_FREQ单位正确）
    #define PWM_PERIOD_SEC      (1.0f / MOTOR_PWM_FREQ_HZ)  // PWM周期（秒）
    // 2. 定义PLL参数（浮点数，显式标注单位）
    #define SpeedPllBandWidth   250.0f      // 环路带宽（rad/s）
    #define SpeedPllKp          (2.0f * SpeedPllBandWidth)                // Kp = 2*Wn
    #define SpeedPllKi          (SpeedPllBandWidth * SpeedPllBandWidth * PWM_PERIOD_SEC)  // Ki = Wn^2 * Ts
    // #define OmegaToTheta        (PWM_PERIOD_SEC * 16384.0f * 32768.0f / (2.0f * PI))       // 角速度→角度转换因子
	#define OmegaToTheta  (PWM_PERIOD_SEC)  // 直接使用PWM周期作为积分系数
    // 3. 霍尔信号积分（预测角度）
    //stcHallAngle.hallAngle += stcHallAngle.hallspeed;
    // 4. 生成实际角度的正弦/余弦信号
	float sin = arm_sin_f32(angle);
	float cos = arm_cos_f32(angle);
	float sin_pll = arm_sin_f32(pmsm_mc_param.theta_pll);
	float cos_pll = arm_cos_f32(pmsm_mc_param.theta_pll);
    // 6. 算相位误差（浮点数运算）
    pmsm_mc_param.delta_theta_pll = sin * cos_pll - cos * sin_pll;
    // 7. 积分相位误差（PI控制器的积分项）
    pmsm_mc_param.pll_sum += pmsm_mc_param.delta_theta_pll;
    // 8. PI控制器计算角速度
    pmsm_mc_param.omega_pll = 
        SpeedPllKp * pmsm_mc_param.delta_theta_pll + 
        SpeedPllKi * pmsm_mc_param.pll_sum;
    // 9. 更新PLL角度（浮点数积分）
    pmsm_mc_param.theta_pll += pmsm_mc_param.omega_pll * OmegaToTheta;
    // 10. 角度归一化（防止溢出，可选）
    if (pmsm_mc_param.theta_pll > ANGLE_2PI) 
    {
        pmsm_mc_param.theta_pll -= ANGLE_2PI;
    } 
    else if (pmsm_mc_param.theta_pll < 0) 
    {
        pmsm_mc_param.theta_pll += ANGLE_2PI;
    }
    // 11. 低通滤波输出角速度（一阶滤波，时间常数≈4个PWM周期）
    pmsm_mc_param.omega_pll_filter += 0.005f * (pmsm_mc_param.omega_pll - pmsm_mc_param.omega_pll_filter);
}

/***************************************** (END OF FILE) *********************************************/
