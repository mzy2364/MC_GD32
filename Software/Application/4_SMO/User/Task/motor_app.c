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

#include "pmsm.h"
#include "userparms.h"

/* DEFINES ------------------------------------------------------------------------------------------*/

/* VARIABLES ----------------------------------------------------------------------------------------*/
motor_control_t motor1 = {0};
static uint32_t ia_sum = 0,ib_sum = 0,ic_sum = 0;

can_trasnmit_message_struct transmit_message;

/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
static void motor_ui_init(void);
static void motor_ui_updata(void);
static void motor_can_transmit(void);

/* GLOBAL FUNCTION ----------------------------------------------------------------------------------*/

/**
  * @brief motor app init
  * @param None
  * @retval None
  */
void motor_app_init(void)
{
    motor1.state = INIT;
    
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
    
    motor_ui_init();
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
        motor1.speed_rpm = pmsm_mc_param.actual_speed * ((float)60 / ANGLE_2PI / MOTOR_NOPOLESPAIRS);
        if(encoder_code == ENCODER_INC)
        {
            if(pmsm_mc_param.vel_input < NOMINAL_SPEED_RAD_PER_SEC_ELEC)
            {
                pmsm_mc_param.vel_input = pmsm_mc_param.vel_input + 1.0f;
            }
        }
        else if(encoder_code == ENCODER_DEC)
        {      
            if(pmsm_mc_param.vel_input > END_SPEED_RADS_ELEC)
            {
                pmsm_mc_param.vel_input = pmsm_mc_param.vel_input - 1.0f;
            }
        }
    }
    
    if(key_code == KEY_0_RELEASE)
    {
        if(motor1.state == RUN)
        {
            motor_pwm_disable();
            motor1.state = STOP;
        }
        else if(motor1.state == STOP)
        {
            motor_pwm_enable();
            pmsm_foc_init();
            motor1.state = RUN;
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
    motor_ui_updata();
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
                motor1.state = RUN;
                motor_pwm_enable();
            }
            break;
        case STOP:
            break;
        case RUN:
            {
                /* read ADC inserted group data register */
                motor1.adc_ia = adc_inserted_data_read(ADC0, IU_INSERTED_CHANNEL);
                motor1.adc_ib = adc_inserted_data_read(ADC0, IV_INSERTED_CHANNEL);
                motor1.adc_ic = adc_inserted_data_read(ADC0, IW_INSERTED_CHANNEL);
                
                pmsm_foc_param.ia = ((float)motor1.adc_ia - motor1.ia_offset) * ADC_TO_CURRENT_COEF;
                pmsm_foc_param.ib = ((float)motor1.adc_ib - motor1.ib_offset) * ADC_TO_CURRENT_COEF;
                pmsm_foc_param.ic = ((float)motor1.adc_ic - motor1.ic_offset) * ADC_TO_CURRENT_COEF;
                pmsm_mc_param.max_phase_voltage = motor1.vdc * ONE_BY_SQRT3;
                
                pmsm_foc_run();
                
                temp1 = (float)smc1.ialpha;
                memcpy(&uart_data[0],&temp1,4);
                temp2 = (float)smc1.est_ialpha;
                memcpy(&uart_data[4],&temp2,4);
                temp3 = (float)smc1.ealpha;
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
  * @brief motor ui init
  * @param None
  * @retval None
  */
static void motor_ui_init(void)
{
    LCD_ShowString(0,0,(const uint8_t *)"tSpd",BLACK,WHITE,24,0);
    LCD_ShowIntNum(52,0,(uint16_t)pmsm_mc_param.vel_input,5,BLUE,WHITE,24);
    LCD_ShowString(120,0,(const uint8_t *)"aSpd",BLACK,WHITE,24,0);
    LCD_ShowIntNum(175,0,(uint16_t)pmsm_mc_param.actual_speed,5,BLUE,WHITE,24);
    
    LCD_ShowString(0,24,(const uint8_t *)"qRef",BLACK,WHITE,24,0);
    LCD_ShowFloatNum2(52,24,pmsm_mc_param.iq_ref,3,BLUE,WHITE,24);
    LCD_ShowString(120,24,(const uint8_t *)"qMea",BLACK,WHITE,24,0);
    LCD_ShowFloatNum2(175,24,pmsm_foc_param.iq,3,BLUE,WHITE,24);
    
    LCD_ShowString(0,48,(const uint8_t *)"dRef",BLACK,WHITE,24,0);
    LCD_ShowFloatNum2(52,48,pmsm_mc_param.id_ref,3,BLUE,WHITE,24);
    LCD_ShowString(120,48,(const uint8_t *)"dMea",BLACK,WHITE,24,0);
    LCD_ShowFloatNum2(175,48,pmsm_foc_param.id,3,BLUE,WHITE,24);
    
    LCD_ShowString(0,72,(const uint8_t *)"Vdc",BLACK,WHITE,24,0);
    LCD_ShowFloatNum2(52,72,motor1.vdc,3,BLUE,WHITE,24);
    LCD_ShowString(120,72,(const uint8_t *)"Idc",BLACK,WHITE,24,0);
    LCD_ShowFloatNum2(175,72,motor1.idc,3,BLUE,WHITE,24);
    
    LCD_ShowString(0,96,(const uint8_t *)"Temp",BLACK,WHITE,24,0);
    LCD_ShowFloatNum2(52,96,motor1.mosfet_temp,4,BLUE,WHITE,24);
    LCD_ShowString(120,96,(const uint8_t *)"STA",BLACK,WHITE,24,0);
    LCD_ShowString(175,96,(const uint8_t *)"INIT",BLACK,WHITE,24,0);
}

/**
  * @brief motor ui updata
  * @param None
  * @retval None
  */
static void motor_ui_updata(void)
{
    LCD_ShowIntNum(52,0,(uint16_t)(pmsm_mc_param.vel_input * ((float)60 / ANGLE_2PI / MOTOR_NOPOLESPAIRS)),5,BLUE,WHITE,24);
    LCD_ShowIntNum(175,0,(uint16_t)motor1.speed_rpm,5,BLUE,WHITE,24);
    
    LCD_ShowFloatNum2(52,24,pmsm_mc_param.iq_ref,3,BLUE,WHITE,24);
    LCD_ShowFloatNum2(175,24,pmsm_foc_param.iq,3,BLUE,WHITE,24);
    
    LCD_ShowFloatNum2(52,48,pmsm_mc_param.id_ref,3,BLUE,WHITE,24);
    LCD_ShowFloatNum2(175,48,pmsm_foc_param.id,3,BLUE,WHITE,24);
    
    LCD_ShowFloatNum2(52,72,motor1.vdc,3,BLUE,WHITE,24);
    LCD_ShowFloatNum2(175,72,motor1.idc,3,BLUE,WHITE,24);
    
    LCD_ShowFloatNum2(52,96,motor1.mosfet_temp,4,BLUE,WHITE,24);
    
    if(motor1.state == RUN)
    {
        LCD_ShowString(175,96,(const uint8_t *)"RUN  ",BLUE,WHITE,24,0);
    }
    else if(motor1.state == STOP)
    {
        LCD_ShowString(175,96,(const uint8_t *)"STOP",BLUE,WHITE,24,0);
    }
    else if(motor1.state == FAULT)
    {
        LCD_ShowString(175,96,(const uint8_t *)"FLT  ",BLUE,WHITE,24,0);
    }
    else
    {
        LCD_ShowString(175,96,(const uint8_t *)"NONE",BLUE,WHITE,24,0);
    }
}

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

/************************************************EOF************************************************/
