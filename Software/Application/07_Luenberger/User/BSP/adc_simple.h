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

#define ADC_FILTER_COUNT_POWER      4
#define ADC_FILTER_COUNT            (1<<ADC_FILTER_COUNT_POWER)

#define ADC_SIMPLE_CHANNEL      3

/* TYPEDEF ------------------------------------------------------------------------------------------*/
typedef struct
{
    uint16_t count;
    uint16_t buffer[ADC_FILTER_COUNT];
}adc_filter_t;

typedef enum
{
    ADC_CH_NTC = 0,
    ADC_CH_VDC,
    ADC_CHIDC_AVER,
}adc_ch_t;

/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/
void adc1_init(void);
uint16_t adc_get_vdc(void);
uint16_t adc_get_ntc(void);
float calculate_temperature_float(uint16_t ntc_adc);
void adc_task(void);
uint16_t adc_read_data(adc_ch_t ch);

#ifdef __cplusplus
extern "C"{
#endif

#endif /* _ADC_SIMPLE_H_ */

/***************************************** (END OF FILE) *********************************************/
