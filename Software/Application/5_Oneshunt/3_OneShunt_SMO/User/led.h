/*
*******************************************************************************************************
*
* File Name : led.h
* Version   : V1.0
* Author    : mzy2364
* brief     : led header file
* 
*******************************************************************************************************
*/
#ifndef _LED_H_
#define _LED_H_

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

#define LED_FAULT                       LED3
#define LED_SYS                         LED2

typedef enum 
{
    LED1 = 0,
    LED2 = 1,
    LED3 = 2
} led_typedef_enum;

/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/
void led_init(void);
void led_on(led_typedef_enum lednum);
void led_off(led_typedef_enum lednum);
void led_toggle(led_typedef_enum lednum);

#ifdef __cplusplus
extern "C"{
#endif

#endif /* _LED_H_ */

/***************************************** (END OF FILE) *********************************************/
