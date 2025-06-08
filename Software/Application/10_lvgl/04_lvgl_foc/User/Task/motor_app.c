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
#include "eeprom.h"
#include "drv8323rs.h"
#include "led.h"
#include "pmsm.h"
#include "userparms.h"

#include "ee_parameter.h"

#include "mainwindow.h"
#include "lvgl.h"
#include "lv_port_disp_mcgd32.h"

/* DEFINES ------------------------------------------------------------------------------------------*/

/* VARIABLES ----------------------------------------------------------------------------------------*/
motor_control_t motor1 = {0};
static uint32_t ia_sum = 0,ib_sum = 0,ic_sum = 0;

can_trasnmit_message_struct transmit_message;

uint8_t eeprom_read_buf[32] = {0};

/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
static void motor_can_transmit(void);

/* GLOBAL FUNCTION ----------------------------------------------------------------------------------*/

/**
  * @brief motor app init
  * @param None
  * @retval None
  */
void motor_app_init(void)
{
    lv_init();
	lv_port_disp_init();
    lvgl_main_app();
    
    eeprom_load_parameter();
    
    motor1.state = INIT;
    motor_set_speed(motor_normal_spd);
    
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
  * @brief motor app get Real time data
  * @param None
  * @retval None
  */
void motor_get_data(motor_control_t *info)
{
    memcpy(info,&motor1,sizeof(motor_control_t));
}

/**
  * @brief motor set speed cmd
  * @param None
  * @retval None
  */
void motor_set_speed(uint16_t speed)
{
    if(speed > motor_normal_spd)
    {
        motor1.speed_input = motor_normal_spd;
    }
    else
    {
        motor1.speed_input = speed;
    }
}

/**
  * @brief motor start running
  * @param None
  * @retval None
  */
void motor_start(void)
{
    if(motor1.state == STOP)
    {
        motor_pwm_enable();
        pmsm_foc_init();
        motor1.state = RUN;
    }
    else if(motor1.state == FAULT)
    {
        uint32_t drv8323_fault_status = 0;
        drv8323_fault_status = DRV8323RS_getFaults();
        DRV8323RS_clearFaults();
        drv8323_fault_status = DRV8323RS_getFaults();
        if(drv8323_fault_status == 0)
        {
            led_off(LED_FAULT);
            motor1.state = STOP;
            motor_timer_restart();
        }
    }
}

/**
  * @brief motor stop
  * @param None
  * @retval None
  */
void motor_stop(void)
{
    if(motor1.state == RUN)
    {
        motor_pwm_disable();
        motor1.state = STOP;
        motor1.speed_rpm = 0;
    }
}

/**
  * @brief 
  * @param None
  * @retval None
  * @note
  */
void motor_forced_stop(void)
{
    motor_pwm_pin_disable();
    motor1.state = FAULT;
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
    if(pmsm_mc_param.run_motor == 1)
    {
        motor1.speed_rpm = pmsm_mc_param.actual_speed * ((float)60 / ANGLE_2PI / MOTOR_NOPOLESPAIRS);
    }
    
    encoder_code = encoder_get();
    key_scan();
    key_code = key_get();
    if(encoder_code == ENCODER_INC)
    {
		ui_event_send(KEY_EVENT_ENC0_INC);
    }
    else if(encoder_code == ENCODER_DEC)
    {
		ui_event_send(KEY_EVENT_ENC0_DEC);
    }
    if(key_code == KEY_0_RELEASE)
    {
        ui_event_send(KEY_EVENT_KEY0_REL);
    }
    else if(key_code == KEY_0_LONG_PRESS)
    {
		ui_event_send(KEY_EVENT_KEY0_LONG_PRE);
    }
    
    pmsm_mc_param.vel_input = (float)motor1.speed_input*RPM_TO_RADS*motor_pole;
    
    lv_tick_inc(10);
    lv_task_handler();
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
    ui_monitor_update();
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
                
                temp1 = (float)pmsm_foc_param.ia;
                memcpy(&uart_data[0],&temp1,4);
                temp2 = (float)pmsm_foc_param.ib;
                memcpy(&uart_data[4],&temp2,4);
                temp3 = (float)pmsm_foc_param.ic;
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

/************************************************EOF************************************************/
