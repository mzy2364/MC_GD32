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


/* DEFINES ------------------------------------------------------------------------------------------*/
#define LEDn                             3U

#define LED1_PIN                         GPIO_PIN_2
#define LED1_GPIO_PORT                   GPIOD
#define LED1_GPIO_CLK                    RCU_GPIOD
  
#define LED2_PIN                         GPIO_PIN_12
#define LED2_GPIO_PORT                   GPIOC
#define LED2_GPIO_CLK                    RCU_GPIOC
  
#define LED3_PIN                         GPIO_PIN_15
#define LED3_GPIO_PORT                   GPIOA
#define LED3_GPIO_CLK                    RCU_GPIOA

#define GPIO3_PIN                        GPIO_PIN_0
#define GPIO3_GPIO_PORT                  GPIOC
#define GPIO3_GPIO_CLK                   RCU_GPIOC

#define GPIO2_PIN                        GPIO_PIN_7
#define GPIO2_GPIO_PORT                  GPIOB
#define GPIO2_GPIO_CLK                   RCU_GPIOB

#define GPIO1_PIN                        GPIO_PIN_6
#define GPIO1_GPIO_PORT                  GPIOB
#define GPIO1_GPIO_CLK                   RCU_GPIOB

#define HALLA_GPIO_PORT                 GPIOC
#define HALLA_GPIO_PIN                  GPIO_PIN_6
#define HALLA_GPIO_CLK                  RCU_GPIOC

#define HALLB_GPIO_PORT                 GPIOC
#define HALLB_GPIO_PIN                  GPIO_PIN_7
#define HALLB_GPIO_CLK                  RCU_GPIOC

#define HALLC_GPIO_PORT                 GPIOC
#define HALLC_GPIO_PIN                  GPIO_PIN_8
#define HALLC_GPIO_CLK                  RCU_GPIOC


typedef enum 
{
    LED1 = 0,
    LED2 = 1,
    LED3 = 2
} led_typedef_enum;

#define LED_FAULT                       LED3
#define LED_SYS                         LED2

/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/
void pwm_init(void);
void interrupt_init(void);
void adc_init(void);
void usart_init(void);
void usart_send_data(uint8_t *buf,uint8_t len);
void led_init(void);
void led_on(led_typedef_enum lednum);
void led_off(led_typedef_enum lednum);
void led_toggle(led_typedef_enum lednum);
void dac_init(void);
void hall_init(void);
uint8_t hall_get(void);

uint16_t adc_get_vdc(void);
uint16_t adc_get_ntc(void);

void motor_get_phase_current_adc(int16_t *p_ia, int16_t *p_ib);
void motor_pwm_set_adc_trigger_point(uint16_t trigger_point0, uint16_t trigger_point1);

float calculate_temperature_float(uint16_t ntc_adc);

#ifdef __cplusplus
extern "C"{
#endif

#endif /* _MOTOR_HARDWARE_H_ */

/***************************************** (END OF FILE) *********************************************/
