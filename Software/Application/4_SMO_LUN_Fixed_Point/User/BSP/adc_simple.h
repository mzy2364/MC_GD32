/*
*******************************************************************************************************
*
* File Name : adc_simple.h
* Version   : V1.0
* Author    : mzy2364
* brief     : adc_simple header file
* 
*******************************************************************************************************
*/
#ifndef _ADC_SIMPLE_H_
#define _ADC_SIMPLE_H_

#ifdef __cplusplus
extern "C"{
#endif

/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "gd32f30x.h"


/* DEFINES ------------------------------------------------------------------------------------------*/
#define VDC_ADC_PORT        GPIOC
#define VDC_ADC_PIN         GPIO_PIN_3
#define VDC_ADC_CLK         RCU_GPIOC
#define VDC_ADC_CHANNEL     ADC_CHANNEL_13

#define NTC_ADC_PORT        GPIOC
#define NTC_ADC_PIN         GPIO_PIN_2
#define NTC_ADC_CLK         RCU_GPIOC
#define NTC_ADC_CHANNEL     ADC_CHANNEL_12

/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/
void adc1_init(void);
uint16_t adc_get_vdc(void);
uint16_t adc_get_ntc(void);
float calculate_temperature_float(uint16_t ntc_adc);

#ifdef __cplusplus
extern "C"{
#endif

#endif /* _ADC_SIMPLE_H_ */

/***************************************** (END OF FILE) *********************************************/
