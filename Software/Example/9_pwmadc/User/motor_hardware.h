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
#define USER_IO1_PORT       GPIOB
#define USER_IO1_PIN        GPIO_PIN_6
#define USER_IO1_CLK        RCU_GPIOB

#define USER_IO3_PORT       GPIOC
#define USER_IO3_PIN        GPIO_PIN_0
#define USER_IO3_CLK        RCU_GPIOC

#define USER_IO2_PORT       GPIOB
#define USER_IO2_PIN        GPIO_PIN_7
#define USER_IO2_CLK        RCU_GPIOB



/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/
void motor_hardware_init(void);
void adc0_init(void);
void interrupt_init(void);
void motor_pwm_set_duty(uint16_t duty_u,uint16_t duty_v,uint16_t duty_w);

#ifdef __cplusplus
extern "C"{
#endif

#endif /* _MOTOR_HARDWARE_H_ */

/***************************************** (END OF FILE) *********************************************/
