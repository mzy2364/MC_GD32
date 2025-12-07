/*
*******************************************************************************************************
*
* File Name : motor_app.c
* Version   : V1.0
* Author    : mzy2364
* brief     : motor control main file
* 
*******************************************************************************************************
*/

/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "motor_app.h"
#include "adc_simple.h"
#include "lcd.h"
#include "uart.h"
#include "encoder.h"
#include "mt6701.h"
#include "pmsm.h"
#include "userparms.h"

/* DEFINES ------------------------------------------------------------------------------------------*/

/* VARIABLES ----------------------------------------------------------------------------------------*/
motor_control_t motor1 = {0};
static uint32_t ia_sum = 0,ib_sum = 0,ic_sum = 0;

can_trasnmit_message_struct transmit_message;

float sensor_mech_angle = 0;
float sensor_last_mech_angle = 0;
float sensor_elec_angle = 0;

#ifndef OFFSET_ANGLE_CAL_DONE
float sensor_offset_angle = 0;
uint8_t sensor_dir = SENSOR_DIR_POS;
uint8_t motor_poles = 0;
#else
/* 如果标定完成,这三个参数可以直接填进去 */
float sensor_offset_angle = 0;
uint8_t sensor_dir = SENSOR_DIR_POS;
uint8_t motor_poles = 0;
#endif


uint8_t sensor_cal_dir = SENSOR_DIR_POS;
uint8_t sensor_offset_cal_done =0;
float offset_total_angle = 0;
uint32_t offset_angle_cal_index = 0;
uint32_t offset_angle_cal_delay = 0;

float delta_mech_angle = 0;
float delta_mech_angle_total = 0;
static float motor_omega = 0;
static float motor_mech_omega = 0;

float last_foc_angle = 0;

uint32_t rotor_turns = 0;   /* 转子圈数 */

/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
static void motor_can_transmit(void);
static float calc_elec_speed(float angle);
static float calc_mech_speed(float angle);

/* GLOBAL FUNCTION ----------------------------------------------------------------------------------*/

/**
  * @brief motor app init
  * @param None
  * @retval None
  */
void motor_app_init(void)
{
    motor1.state = INIT;
    motor1.start = 1;
    
    motor1.sensor_cal_state = WAIT_STABLE;
    
    transmit_message.tx_sfid = 0x7ab;
    transmit_message.tx_efid = 0x00;
    transmit_message.tx_ft = CAN_FT_DATA;
    transmit_message.tx_ff = CAN_FF_STANDARD;
    transmit_message.tx_dlen = 8;
    
    transmit_message.tx_data[0] = 0x00;
    transmit_message.tx_data[1] = 0xA1;
    transmit_message.tx_data[2] = 0xA2;
    transmit_message.tx_data[3] = 0xA3;
    transmit_message.tx_data[4] = 0xA4;
    transmit_message.tx_data[5] = 0xA5;
    transmit_message.tx_data[6] = 0xA6;
    transmit_message.tx_data[7] = 0xA7;

}

/**
  * @brief motor task 10ms
  * @param None
  * @retval None
  */
void motor_app_task10ms(void)
{
    encoder_code_t encoder_code = ENCODER_NONE;
    uint8_t key_code = KEY_NONE;
    
    motor1.ntc_adc = adc_read_data(ADC_CH_NTC);
    motor1.vdc_adc = adc_read_data(ADC_CH_VDC);
    motor1.idc_adc = adc_read_data(ADC_CHIDC_AVER);
    motor1.mosfet_temp = calculate_temperature_float(motor1.ntc_adc);
    motor1.vdc = motor1.vdc_adc * ADC_TO_VDC_COEF;
    if(motor1.idc_adc > (ADC_FULL_BIT / 2))
        motor1.idc = (motor1.idc_adc - 2048) * ADC_TO_CURRENT_COEF;
    else
        motor1.idc = 0;
    
    encoder_code = encoder_get();
    key_scan();
    key_code = key_get();
    
    if(pmsm_mc_param.run_motor == 1)
    {
        motor1.speed_rpm = pmsm_mc_param.actual_speed * ((float)60 / ANGLE_2PI);
        if(encoder_code == ENCODER_INC)
        {
            if(pmsm_mc_param.vel_input < NOMINAL_SPEED_RAD_PER_SEC_MECH)
            {
                pmsm_mc_param.vel_input = pmsm_mc_param.vel_input + 1.0f;
            }
        }
        else if(encoder_code == ENCODER_DEC)
        {      
            if(pmsm_mc_param.vel_input > END_SPEED_RADS_MECH)
            {
                pmsm_mc_param.vel_input = pmsm_mc_param.vel_input - 1.0f;
            }
        }
    }
    
    if(key_code == KEY_0_RELEASE)
    {
        if(motor1.start)
        {
            motor1.start = 0;
        }
        else
        {
            motor1.start = 1;
        }
    }
    else if(key_code == KEY_0_LONG_PRESS)
    {
        
    }
    
    motor_can_transmit();
    
}

/**
  * @brief motor task 100ms
  * @param None
  * @retval None
  */
void motor_app_task100ms(void)
{
    
}

/**
  * @brief motor task 500ms
  * @param None
  * @retval None
  */
void motor_app_task500ms(void)
{
    
}

/**
  * @brief motor task 1s
  * @param None
  * @retval None
  */
void motor_app_task1s(void)
{

}

/**
  * @brief motor task idle
  * @param None
  * @retval None
  */
void motor_app_idle(void)
{
    adc_task();
}

/**
  * @brief motor isr func,called in pwm isr
  * @param None
  * @retval None
  */
void motor_app_isr(void)
{
    float temp1 = 0;
    float temp2 = 0;
    float temp3 = 0;
    uint8_t uart_data[16] = {0};
    
    mt6701_read_angle2(&sensor_mech_angle);
    if(sensor_dir == SENSOR_DIR_REV)
    {
        sensor_mech_angle = ANGLE_2PI - sensor_mech_angle;
    }
    motor_mech_omega = calc_mech_speed(sensor_mech_angle);
#ifndef OFFSET_ANGLE_CAL_DONE
    if(sensor_offset_cal_done)
    {
        sensor_elec_angle = sensor_mech_angle * motor_poles;
        utils_norm_angle_rad(&sensor_elec_angle);
        motor_omega = calc_elec_speed(sensor_elec_angle);
    }
#else
        sensor_elec_angle = sensor_mech_angle * motor_poles;
        utils_norm_angle_rad(&sensor_elec_angle);
        motor_omega = calc_elec_speed(sensor_elec_angle);
#endif
    
    
    if(sensor_offset_cal_done)
    {
        sensor_elec_angle = sensor_mech_angle * motor_poles;
        utils_norm_angle_rad(&sensor_elec_angle);
        motor_omega = calc_elec_speed(sensor_elec_angle);
    }
    
    switch(motor1.state)
    {
        case INIT:
            motor1.state = ADC_CAL;
            break;
        case ADC_CAL:
            if(motor1.adc_cal_cnt < ADC_CAL_COUNT)
            {
                motor1.adc_cal_cnt++;
                motor1.adc_ia = adc_inserted_data_read(ADC0, IU_INSERTED_CHANNEL);
                motor1.adc_ib = adc_inserted_data_read(ADC0, IV_INSERTED_CHANNEL);
                motor1.adc_ic = adc_inserted_data_read(ADC0, IW_INSERTED_CHANNEL);
                
                ia_sum += motor1.adc_ia;
                ib_sum += motor1.adc_ib;
                ic_sum += motor1.adc_ic;
            }
            else
            {
                motor1.ia_offset = ia_sum / motor1.adc_cal_cnt;
                motor1.ib_offset = ib_sum / motor1.adc_cal_cnt;
                motor1.ic_offset = ic_sum / motor1.adc_cal_cnt;
                motor1.state = START_DELAY;
            }
            break;
        case START_DELAY:
            if(motor1.start_delay_tick < START_DELAY_TICK)
            {
                motor1.start_delay_tick++;
            }
            else
            {
#ifndef OFFSET_ANGLE_CAL_DONE
                motor1.state = SENSOR_OFFSET_CAL;
#else
                motor1.state = RUN;
#endif
                motor_pwm_channel_enable(ENABLE);
            }
            break;
        case STOP:
            if(motor1.start == 1)
            {
                pmsm_foc_init();
                motor_pwm_channel_enable(ENABLE);
                motor1.state = RUN;
            }
            break;
        case SENSOR_OFFSET_CAL:
            {
                /* read ADC inserted group data register */
                motor1.adc_ia = adc_inserted_data_read(ADC0, IU_INSERTED_CHANNEL);
                motor1.adc_ib = adc_inserted_data_read(ADC0, IV_INSERTED_CHANNEL);
                motor1.adc_ic = adc_inserted_data_read(ADC0, IW_INSERTED_CHANNEL);
                
                pmsm_foc_param.ia = ((float)motor1.adc_ia - motor1.ia_offset) * ADC_TO_CURRENT_COEF;
                pmsm_foc_param.ib = ((float)motor1.adc_ib - motor1.ib_offset) * ADC_TO_CURRENT_COEF;
                pmsm_foc_param.ic = ((float)motor1.adc_ic - motor1.ic_offset) * ADC_TO_CURRENT_COEF;
                
                switch(motor1.sensor_cal_state)
                {
                    case WAIT_STABLE:
                        offset_angle_cal_delay++;
                        if(offset_angle_cal_delay >= OFFSET_ANGLE_CAL_DELAY_TICK)
                        {
                            motor1.sensor_cal_state = DIR_CAL;
                            offset_angle_cal_index = 0;
                        }
                        break;
                    case DIR_CAL:
                        delta_mech_angle_total += delta_mech_angle;
                        offset_angle_cal_index++;
                        if(offset_angle_cal_index >= OFFSET_ANGLE_CAL_COUNT)
                        {
                            offset_angle_cal_index = 0;
                            if(delta_mech_angle_total > 0)
                            {
                                sensor_cal_dir = SENSOR_DIR_POS;
                            }
                            else
                            {
                                sensor_cal_dir = SENSOR_DIR_REV;
                            }
                            sensor_dir = sensor_cal_dir;
                            motor1.sensor_cal_state = POLES_CAL;
                            last_foc_angle = pmsm_foc_param.angle;
                            rotor_turns = 0;
                        }
                        break;
                    case POLES_CAL:
                        if(rotor_turns >= 3)
                        {
                            if(pmsm_foc_param.angle < last_foc_angle)
                            {
                                /* 电角度换向了 */
                                motor_poles++;
                            }
                            last_foc_angle = pmsm_foc_param.angle;
                            if(rotor_turns >= 4)
                            {
                                motor1.sensor_cal_state = OFFSET_CAL;
                            }
                        }
                        else
                        {
                            last_foc_angle = pmsm_foc_param.angle;
                        }
                        
                        break;
                    case OFFSET_CAL:
                        sensor_elec_angle = sensor_mech_angle * motor_poles;
                        utils_norm_angle_rad(&sensor_elec_angle);
                        sensor_offset_angle = pmsm_foc_param.angle - sensor_elec_angle;
                        utils_norm_angle_rad(&sensor_offset_angle);

                        offset_total_angle += sensor_offset_angle;
                        offset_angle_cal_index++;
                        if(offset_angle_cal_index >= OFFSET_ANGLE_CAL_COUNT)
                        {
                            sensor_offset_angle = offset_total_angle / (float)OFFSET_ANGLE_CAL_COUNT;
                            sensor_offset_cal_done = 1;
                            motor1.sensor_cal_state = CAL_DONE;
                        }
                        break;
                    case CAL_DONE:
                        motor1.state = STOP;
                        break;
                    case CAL_ERROR:
                        break;
                    default:
                        break;
                }
                

                
                pmsm_mc_param.openloop = 1;
                pmsm_foc_run();
                
                temp1 = (float)sensor_mech_angle;
                memcpy(&uart_data[0],&temp1,4);
                temp2 = (float)pmsm_foc_param.angle;
                memcpy(&uart_data[4],&temp2,4);
                temp3 = (float)motor_poles;
                memcpy(&uart_data[8],&temp3,4);
                uart_data[sizeof(uart_data)-2] = 0x80;
                uart_data[sizeof(uart_data)-1] = 0x7f;
                usart_send_data(uart_data,sizeof(uart_data));
                
                motor_pwm_set_duty(pmsm_foc_param.pwma,pmsm_foc_param.pwmb,pmsm_foc_param.pwmc);
            }
            break;
        case RUN:
            {
                if(motor1.start == 0)
                {
                    motor_pwm_channel_enable(DISABLE);
                    motor1.state = STOP;
                    break;
                }
                
                /* read ADC inserted group data register */
                motor1.adc_ia = adc_inserted_data_read(ADC0, IU_INSERTED_CHANNEL);
                motor1.adc_ib = adc_inserted_data_read(ADC0, IV_INSERTED_CHANNEL);
                motor1.adc_ic = adc_inserted_data_read(ADC0, IW_INSERTED_CHANNEL);
                
                pmsm_foc_param.ia = ((float)motor1.adc_ia - motor1.ia_offset) * ADC_TO_CURRENT_COEF;
                pmsm_foc_param.ib = ((float)motor1.adc_ib - motor1.ib_offset) * ADC_TO_CURRENT_COEF;
                pmsm_foc_param.ic = ((float)motor1.adc_ic - motor1.ic_offset) * ADC_TO_CURRENT_COEF;
                
                pmsm_mc_param.openloop = 0;
                pmsm_foc_param.angle = sensor_elec_angle + sensor_offset_angle + 1.57f;
                utils_norm_angle_rad(&pmsm_foc_param.angle);
                pmsm_mc_param.actual_speed = motor_mech_omega;
                
                pmsm_foc_run();
                
                temp1 = (float)sensor_mech_angle;
                memcpy(&uart_data[0],&temp1,4);
                temp2 = (float)pmsm_foc_param.angle;
                memcpy(&uart_data[4],&temp2,4);
                temp3 = (float)motor_mech_omega;
                memcpy(&uart_data[8],&temp3,4);
                uart_data[sizeof(uart_data)-2] = 0x80;
                uart_data[sizeof(uart_data)-1] = 0x7f;
                usart_send_data(uart_data,sizeof(uart_data));
                
                motor_pwm_set_duty(pmsm_foc_param.pwma,pmsm_foc_param.pwmb,pmsm_foc_param.pwmc);
            }
            break;
        case FAULT:
            break;
        default:
            break;
    }
}

/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/

/**
  * @brief motor can frame transmit task
  * @param None
  * @retval None
  */
static void motor_can_transmit(void)
{
    uint8_t mosfet_temperature = 0;
    if(motor1.mosfet_temp < -40)
        mosfet_temperature = 0;
    else
        mosfet_temperature = motor1.mosfet_temp + 40;
    
    transmit_message.tx_data[0] = mosfet_temperature;
    transmit_message.tx_data[1] = (uint8_t)(motor1.idc * 10);
    transmit_message.tx_data[2] = (uint8_t)(motor1.vdc * 6);
    transmit_message.tx_data[3] = pmsm_foc_param.iq * 10;
    transmit_message.tx_data[4] = motor1.speed_rpm & 0xff;
    transmit_message.tx_data[5] = motor1.speed_rpm >> 8;
    transmit_message.tx_data[6] = 0;
    transmit_message.tx_data[7] = 0;
    
    can_message_transmit(CAN0, &transmit_message);
}

/**
  * @brief calculate motor speed
  * @param angle - motor angle
  * @retval None
  */
static float calc_mech_speed(float angle)
{
    delta_mech_angle = angle - sensor_last_mech_angle;
    sensor_last_mech_angle = angle;
    if(delta_mech_angle < -M_PI)
    {
        delta_mech_angle += ANGLE_2PI;
        rotor_turns++;
    }
    else if(delta_mech_angle > M_PI)
    {
        delta_mech_angle -= ANGLE_2PI;
        rotor_turns++;
    }
    return delta_mech_angle * MOTOR_PWM_FREQ_HZ;
}

/**
  * @brief calculate motor speed
  * @param angle - motor angle
  * @retval None
  */
static float calc_elec_speed(float angle)
{
    static float theta_pll = 0;
    static float pll_sum = 0;
    static float omega_pll = 0;
    static float omega_pll_filter = 0;
    
    // 1. 定义时间相关参数（需确保PWM_FREQ单位正确）
    #define PWM_PERIOD_SEC      (1.0f / MOTOR_PWM_FREQ_HZ)  // PWM周期（秒）
    // 2. 定义PLL参数（浮点数，显式标注单位）
    #define SpeedPllBandWidth   600.0f      // 环路带宽（rad/s）
    #define SpeedPllKp          (2.0f * SpeedPllBandWidth)                // Kp = 2*Wn
    #define SpeedPllKi          (SpeedPllBandWidth * SpeedPllBandWidth * PWM_PERIOD_SEC)  // Ki = Wn^2 * Ts
    // #define OmegaToTheta        (PWM_PERIOD_SEC * 16384.0f * 32768.0f / (2.0f * PI))       // 角速度→角度转换因子
	#define OmegaToTheta  (PWM_PERIOD_SEC)  // 直接使用PWM周期作为积分系数
    // 3. 霍尔信号积分（预测角度）
    //stcHallAngle.hallAngle += stcHallAngle.hallspeed;
    // 4. 生成实际角度的正弦/余弦信号
	float sin = arm_sin_f32(angle);
	float cos = arm_cos_f32(angle);
	float sin_pll = arm_sin_f32(theta_pll);
	float cos_pll = arm_cos_f32(theta_pll);
    // 6. 算相位误差（浮点数运算）
    float delta_theta_pll = sin * cos_pll - cos * sin_pll;
    // 7. 积分相位误差（PI控制器的积分项）
    pll_sum += delta_theta_pll;
    // 8. PI控制器计算角速度
    omega_pll = SpeedPllKp * delta_theta_pll + SpeedPllKi * pll_sum;
    // 9. 更新PLL角度（浮点数积分）
    theta_pll += omega_pll * OmegaToTheta;
    // 10. 角度归一化（防止溢出，可选）
    if (theta_pll > ANGLE_2PI) 
    {
        theta_pll -= ANGLE_2PI;
    } 
    else if (theta_pll < 0) 
    {
        theta_pll += ANGLE_2PI;
    }
    // 11. 低通滤波输出角速度（一阶滤波，时间常数≈4个PWM周期）
    omega_pll_filter += 0.005f * (omega_pll - omega_pll_filter);
    
    return omega_pll_filter;
}

/************************************************EOF************************************************/
