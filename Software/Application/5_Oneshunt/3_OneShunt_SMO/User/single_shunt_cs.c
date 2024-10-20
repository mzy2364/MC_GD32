/*
*******************************************************************************************************
*
* File Name : single_shunt_cs.c
* Version   : V1.0
* Author    : mzy2364
* brief     : single_shunt_cs file
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include <stddef.h>
#include "single_shunt_cs.h"
#include "motor_hardware.h"
#include "foc.h"
#include "pmsm.h"
/* DEFINES ------------------------------------------------------------------------------------------*/
#define MIN_EDGE_WIDTH  50

#define ONE_SHUNT_IA    0x01
#define ONE_SHUNT_IB    0X02
#define ONE_SHUNT_IAIB  (ONE_SHUNT_IA|ONE_SHUNT_IB)

/* VARIABLES ----------------------------------------------------------------------------------------*/
int32_t duty_u_offset = 0;
int32_t duty_v_offset = 0;
int32_t duty_w_offset = 0;
uint16_t trigger_point1 = 0;
uint16_t trigger_point2 = 0;
int32_t duty_u = 0;
int32_t duty_v = 0;
int32_t duty_w = 0;
uint32_t _pwm_period = MOTOR_PWM_PERIOD;
uint16_t t_sample_delay_cycle = MOTOR_PWM_PERIOD / 20;
uint16_t t_min_dur_cycle = MOTOR_PWM_PERIOD / 10;

int16_t signle_shunt_ia = 0;
int16_t signle_shunt_ib = 0;

uint8_t sector_last = 0;

uint8_t sample_mode = 0;
uint8_t sample_mode_last = 0;

uint32_t idc_offset = 0;
uint32_t idc_offset_sum = 0;
uint8_t adc_calibration;

/* FUNCTION -----------------------------------------------------------------------------------------*/

/**
  * @brief 相电流采样初始化
  * @param None
  * @retval None
  * @note 
  */
void motor_phase_current_sampling_init(void)
{
    float sample_delay_time = 0;
    float sample_time = 0;
    float min_dur_time = 0;
    float pwm_period_time = 0;

    sample_time = CIRCUIT_SAMPLE_TIME_S;
    sample_delay_time = CIRCUIT_DEAD_TIME_S + CIRCUIT_PWM_DELAY_TIME_S + CIRCUIT_SAMPLE_VIBRATE_TIME_S;
    pwm_period_time = (1.0f / MOTOR_PWM_FREQ_HZ);
    min_dur_time = sample_delay_time + sample_time;
    t_sample_delay_cycle = (uint16_t)(sample_delay_time * (float)_pwm_period / pwm_period_time) + 1;
    t_min_dur_cycle = (uint16_t)(min_dur_time * (float)_pwm_period / pwm_period_time) + 1;
}

/**
  * @brief 3相电流采样重构
  * @param sector-SVPWM扇区
  * @retval None
  * @note 
  */
void motor_get_phase_current_zero(void)
{
    if(adc_calibration == 0)
    {
        static uint16_t cnt = 0;
        int16_t adc_dc0 = 0;
        int16_t adc_dc1 = 0;
        cnt++;
        motor_get_phase_current_adc(&adc_dc0,&adc_dc1);
        idc_offset_sum += adc_dc0;
        if(cnt >= (1<<10))
        {
            int16_t error = 0;
            adc_calibration = 1;
            idc_offset = idc_offset_sum >> 10;
            error = idc_offset - (ADC_FULL_BIT >> 1);
            if((error > 500) || (error < -500))
            {
                idc_offset = ADC_FULL_BIT >> 1;
            }
        }
    }
}

/**
  * @brief 3相电流采样重构
  * @param sector-SVPWM扇区
  * @retval None
  * @note 
  */
void motor_get_phase_current(uint8_t sector)
{
    float iu = 0,iv = 0,iw = 0;
	motor_get_phase_current_adc(&signle_shunt_ia,&signle_shunt_ib);
    signle_shunt_ia = signle_shunt_ia - idc_offset;
    signle_shunt_ib = signle_shunt_ib - idc_offset;
    
    
    if(sample_mode_last == ONE_SHUNT_IAIB)
    {
        switch(sector_last)
        {
            case 1:
                iw = -signle_shunt_ia;
                iu =  signle_shunt_ib;
                iv = 0 + signle_shunt_ia - signle_shunt_ib;
                break;
            case 2:
                iw = -signle_shunt_ia;
                iv =  signle_shunt_ib;
                iu = 0 + signle_shunt_ia - signle_shunt_ib;
                break;
            case 3:
                iu = -signle_shunt_ia;
                iv =  signle_shunt_ib;
                iw = 0 + signle_shunt_ia - signle_shunt_ib;
                break;
            case 4:
                iu = -signle_shunt_ia;
                iw =  signle_shunt_ib;
                iv = 0 + signle_shunt_ia - signle_shunt_ib;
                break;
            case 5:
                iv = -signle_shunt_ia;
                iw =  signle_shunt_ib;
                iu = 0 + signle_shunt_ia - signle_shunt_ib;
                break;
            case 6:
                iv = -signle_shunt_ia;
                iu =  signle_shunt_ib;
                iw = 0 + signle_shunt_ia - signle_shunt_ib;
                break;
            default:
                break;
        }
    }
    else if(sample_mode_last == ONE_SHUNT_IA)
    {
        switch(sector_last)
        {
            case 1:
                iw = -signle_shunt_ia;
                break;
            case 2:
                iw = -signle_shunt_ia;
                break;
            case 3:
                iu = -signle_shunt_ia;
                break;
            case 4:
                iu = -signle_shunt_ia;
                break;
            case 5:
                iv = -signle_shunt_ia;
                break;
            case 6:
                iv = -signle_shunt_ia;
                break;
            default:
                break;
        }
    }
    else if(sample_mode_last == ONE_SHUNT_IB)
    {
        switch(sector_last)
        {
            case 1:
                iu =  signle_shunt_ib;
                break;
            case 2:
                iv =  signle_shunt_ib;
                break;
            case 3:
                iv =  signle_shunt_ib;
                break;
            case 4:
                iw =  signle_shunt_ib;
                break;
            case 5:
                iw =  signle_shunt_ib;
                break;
            case 6:
                iu =  signle_shunt_ib;
                break;
            default:
                break;
        }
    }
    else
    {
        sector_last = sector;
    }
    sector_last = sector;
    
    pmsm_foc_param.ia = iu * ADC_TO_CURRENT_COEF;
    pmsm_foc_param.ib = iv * ADC_TO_CURRENT_COEF;
    pmsm_foc_param.ic = iw * ADC_TO_CURRENT_COEF;
}


/**
  * @brief 相电流采样点偏移
  * @param sector-SVPWM扇区
  * @retval None
  * @note 单电阻采样方式,需要根据PWM占空比在合适位置采样,当某两相的占空比相等时,
          采样窗口时间为0,所以需要进行偏移
  */
void motor_phase_current_sampling_shift(uint8_t sector)
{
	//带PWM相位偏移的采样点设置
	int32_t *duty_min = NULL, *duty_mid = NULL, *duty_max = NULL;
	int32_t *duty_min_offset = NULL, *duty_mid_offset = NULL, *duty_max_offset = NULL;
	
	/* 计算占空比最小值 中间值 最大值 */
	trigger_point1  = 0;
	trigger_point2 = 0;
	duty_u_offset = 0;
	duty_v_offset = 0;
	duty_w_offset = 0;
    sample_mode_last = sample_mode;

	switch(sector)
	{
		case 1:
			duty_max = &duty_u;
			duty_min = &duty_w;
			duty_mid = &duty_v;
			duty_max_offset = &duty_u_offset;
			duty_min_offset = &duty_w_offset;
			duty_mid_offset = &duty_v_offset;
			break;
		case 2:
			duty_max = &duty_v;
			duty_min = &duty_w;
			duty_mid = &duty_u;
			duty_max_offset = &duty_v_offset;
			duty_min_offset = &duty_w_offset;
			duty_mid_offset = &duty_u_offset;
			break;
		case 3:
			duty_max = &duty_v;
			duty_min = &duty_u;
			duty_mid = &duty_w;
			duty_max_offset = &duty_v_offset;
			duty_min_offset = &duty_u_offset;
			duty_mid_offset = &duty_w_offset;
			break;
		case 4:
			duty_max = &duty_w;
			duty_min = &duty_u;
			duty_mid = &duty_v;
			duty_max_offset = &duty_w_offset;
			duty_min_offset = &duty_u_offset;
			duty_mid_offset = &duty_v_offset;
			break;
		case 5:
			duty_max = &duty_w;
			duty_min = &duty_v;
			duty_mid = &duty_u;
			duty_max_offset = &duty_w_offset;
			duty_min_offset = &duty_v_offset;
			duty_mid_offset = &duty_u_offset;
			break;
		case 6:
			duty_max = &duty_u;
			duty_min = &duty_v;
			duty_mid = &duty_w;
			duty_max_offset = &duty_u_offset;
			duty_min_offset = &duty_v_offset;
			duty_mid_offset = &duty_w_offset;
			break;
		default:
			duty_max = &duty_u;
			duty_min = &duty_v;
			duty_mid = &duty_w;
			duty_max_offset = &duty_u_offset;
			duty_min_offset = &duty_v_offset;
			duty_mid_offset = &duty_w_offset;
			break;
	}
    
    /* MAX MIN 首先固定移动到最靠边位置 */
    sample_mode = 0;
    if(MIN_EDGE_WIDTH < *duty_min)
    {
        /* duty_min_offset < 0 */
        *duty_min_offset = MIN_EDGE_WIDTH - *duty_min;
        *duty_min += *duty_min_offset;
    }
    if(*duty_max <= (_pwm_period / 2))
    {
        *duty_max_offset = *duty_max - MIN_EDGE_WIDTH;
    }
    else
    {
        if((_pwm_period-MIN_EDGE_WIDTH) > *duty_max)
        {
            /* duty_max_offset > 0 */
            *duty_max_offset = _pwm_period - MIN_EDGE_WIDTH - *duty_max;
            *duty_max += *duty_max_offset;
        }
    }
    if(*duty_mid <= (_pwm_period / 2))
    {
        /* duty_mid_offset >= 0 */
        sample_mode |= ONE_SHUNT_IB;
        if(((_pwm_period / 2) - *duty_mid) < *duty_mid)
        {
            *duty_mid_offset = (_pwm_period / 2) - *duty_mid;   
        }
        else
        {
            *duty_mid_offset = *duty_mid;
        }
        *duty_mid += *duty_mid_offset;
        if((*duty_mid - *duty_min) > t_min_dur_cycle)
        {
            sample_mode |= ONE_SHUNT_IA;
        }
    }
    else
    {
        /* duty_mid_offset < 0 */
        sample_mode |= ONE_SHUNT_IA;
        if((*duty_mid - (_pwm_period / 2)) < (_pwm_period - *duty_mid))
        {
            *duty_mid_offset = (_pwm_period / 2) - *duty_mid;
        }
        else
        {
            *duty_mid_offset = -(_pwm_period - *duty_mid);
        }
        *duty_mid += *duty_mid_offset;
        if((*duty_max - *duty_mid) > t_min_dur_cycle)
        {
            sample_mode |= ONE_SHUNT_IB;
        }
    }
    
    
//    sample_mode = 0;
//    if((*duty_mid - *duty_min) > t_min_dur_cycle)
//    {
//        trigger_point1 = *duty_min + t_sample_delay_cycle;
//        sample_mode |= ONE_SHUNT_IA;
//    }
//    if((*duty_max - *duty_mid) > t_min_dur_cycle)
//    {
//        trigger_point2 = *duty_mid + t_sample_delay_cycle;
//        sample_mode |= ONE_SHUNT_IB;
//    }
    
    trigger_point1 = *duty_min + t_sample_delay_cycle;
    trigger_point2 = *duty_mid + t_sample_delay_cycle;
    motor_pwm_set_adc_trigger_point(trigger_point1, trigger_point2);
}


/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
/**
  * @brief 
  * @param None
  * @retval None
  */


/***************************************** (END OF FILE) *********************************************/
