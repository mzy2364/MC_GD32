/*
*******************************************************************************************************
*
* File Name : motor_hardware.h
* Version   : V1.0
* Author    : mzy2364
* brief     : motor control hardware config file
* 
*******************************************************************************************************
*/
#ifndef _MOTOR_HARDWARE_H_
#define _MOTOR_HARDWARE_H_

#ifdef __cplusplus
extern "C"{
#endif

/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "gd32f30x.h"
#include "motor_config.h"


/* DEFINES ------------------------------------------------------------------------------------------*/
#define MOTOR_PWM_TIMER     TIMER0
#define PWM_U_CHANNEL       TIMER_CH_2
#define PWM_V_CHANNEL       TIMER_CH_1
#define PWM_W_CHANNEL       TIMER_CH_0

#define PWM_UH_GPIO_PORT    GPIOA
#define PWM_UH_GPIO_PIN     GPIO_PIN_10
#define PWM_VH_GPIO_PORT    GPIOA
#define PWM_VH_GPIO_PIN     GPIO_PIN_9
#define PWM_WH_GPIO_PORT    GPIOA
#define PWM_WH_GPIO_PIN     GPIO_PIN_8

#define PWM_UL_GPIO_PORT    GPIOB
#define PWM_UL_GPIO_PIN     GPIO_PIN_15
#define PWM_VL_GPIO_PORT    GPIOB
#define PWM_VL_GPIO_PIN     GPIO_PIN_14
#define PWM_WL_GPIO_PORT    GPIOB
#define PWM_WL_GPIO_PIN     GPIO_PIN_13

#define PWM_BKIN_GPIO_PORT    GPIOB
#define PWM_BKIN_GPIO_PIN     GPIO_PIN_12

#define MOTOR_CURRENT_ADC       ADC0
#define IU_INSERTED_CHANNEL     ADC_INSERTED_CHANNEL_0
#define IV_INSERTED_CHANNEL     ADC_INSERTED_CHANNEL_1
#define IW_INSERTED_CHANNEL     ADC_INSERTED_CHANNEL_2

#define IU_ADC_PORT         GPIOC
#define IU_ADC_PIN          GPIO_PIN_4
#define IU_ADC_CLK          RCU_GPIOC
#define IU_ADC_CHANNEL      ADC_CHANNEL_14

#define IV_ADC_PORT         GPIOC
#define IV_ADC_PIN          GPIO_PIN_5
#define IV_ADC_CLK          RCU_GPIOC
#define IV_ADC_CHANNEL      ADC_CHANNEL_15

#define IW_ADC_PORT         GPIOB
#define IW_ADC_PIN          GPIO_PIN_0
#define IW_ADC_CLK          RCU_GPIOB
#define IW_ADC_CHANNEL      ADC_CHANNEL_8

#define IDC_ADC_PORT        GPIOB
#define IDC_ADC_PIN         GPIO_PIN_1
#define IDC_ADC_CLK         RCU_GPIOB
#define IDC_ADC_CHANNEL     ADC_CHANNEL_9

#define IDC_AVER_ADC_PORT        GPIOA
#define IDC_AVER_ADC_PIN         GPIO_PIN_3
#define IDC_AVER_ADC_CLK         RCU_GPIOA
#define IDC_AVER_ADC_CHANNEL     ADC_CHANNEL_3

#define BEMFU_ADC_PORT      GPIOA
#define BEMFU_ADC_PIN       GPIO_PIN_0
#define BEMFU_ADC_CLK       RCU_GPIOA
#define BEMFU_ADC_CHANNEL   ADC_CHANNEL_0

#define BEMFV_ADC_PORT      GPIOA
#define BEMFV_ADC_PIN       GPIO_PIN_1
#define BEMFV_ADC_CLK       RCU_GPIOA
#define BEMFV_ADC_CHANNEL   ADC_CHANNEL_1

#define BEMFW_ADC_PORT      GPIOA
#define BEMFW_ADC_PIN       GPIO_PIN_2
#define BEMFW_ADC_CLK       RCU_GPIOA
#define BEMFW_ADC_CHANNEL   ADC_CHANNEL_2

#define USER_IO1_PORT       GPIOB
#define USER_IO1_PIN        GPIO_PIN_6
#define USER_IO1_CLK        RCU_GPIOB

#define USER_IO3_PORT       GPIOC
#define USER_IO3_PIN        GPIO_PIN_0
#define USER_IO3_CLK        RCU_GPIOC

#define USER_IO2_PORT       GPIOB
#define USER_IO2_PIN        GPIO_PIN_7
#define USER_IO2_CLK        RCU_GPIOB

#define HALLA_GPIO_PORT     GPIOC
#define HALLA_GPIO_PIN      GPIO_PIN_6
#define HALLA_GPIO_CLK      RCU_GPIOC

#define HALLB_GPIO_PORT     GPIOC
#define HALLB_GPIO_PIN      GPIO_PIN_7
#define HALLB_GPIO_CLK      RCU_GPIOC

#define HALLC_GPIO_PORT     GPIOC
#define HALLC_GPIO_PIN      GPIO_PIN_8
#define HALLC_GPIO_CLK      RCU_GPIOC

/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/
void motor_hardware_init(void);
void adc0_init(void);
void interrupt_init(void);
void comp_protect_init(void);
void hall_init(void);
uint8_t hall_get(void);
void motor_pwm_set_duty(uint16_t duty_u,uint16_t duty_v,uint16_t duty_w);
void motor_set_voltage_vector(int16_t vector);
void motor_pwm_pin_disable(void);
void motor_pwm_pin_enable(void);
void motor_pwm_disable(void);
void motor_pwm_enable(void);

#ifdef __cplusplus
extern "C"{
#endif

#endif /* _MOTOR_HARDWARE_H_ */

/***************************************** (END OF FILE) *********************************************/
