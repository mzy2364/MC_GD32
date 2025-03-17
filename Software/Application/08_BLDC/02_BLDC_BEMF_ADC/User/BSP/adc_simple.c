/*
*******************************************************************************************************
*
* File Name : adc_simple.c
* Version   : V1.0
* Author    : mzy2364
* brief     : adc driver file
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "adc_simple.h"
#include "systick.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/


/**
  * @brief adc1 init
  * @param None
  * @retval None
  */
void adc1_init(void)
{
    /* gpio init */
    rcu_periph_clock_enable(VDC_ADC_CLK);
    rcu_periph_clock_enable(NTC_ADC_CLK);
    
    gpio_init(VDC_ADC_PORT, GPIO_MODE_AIN, GPIO_OSPEED_MAX, VDC_ADC_PIN);
    gpio_init(NTC_ADC_PORT, GPIO_MODE_AIN, GPIO_OSPEED_MAX, NTC_ADC_PIN);
    
    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_ADC1);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV2);
    
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);
    /* ADC channel length config */
    adc_channel_length_config(ADC1, ADC_REGULAR_CHANNEL, 1U);
    
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC1, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE); 
    /* ADC external trigger config */
    adc_external_trigger_config(ADC1, ADC_REGULAR_CHANNEL, ENABLE);

    /* enable ADC interface */
    adc_enable(ADC1);
    systick_delay(1U);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC1);
}

/**
  * @brief adc get dc bus voltage
  * @param None
  * @retval None
  */
uint16_t adc_get_vdc(void)
{
    /* ADC regular channel config */
    adc_regular_channel_config(ADC1, 0U, VDC_ADC_CHANNEL, ADC_SAMPLETIME_7POINT5);
    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC1, ADC_REGULAR_CHANNEL);

    /* wait the end of conversion flag */
    while(!adc_flag_get(ADC1, ADC_FLAG_EOC));
    /* clear the end of conversion flag */
    adc_flag_clear(ADC1, ADC_FLAG_EOC);
    /* return regular channel sample value */
    return (adc_regular_data_read(ADC1));
}

/**
  * @brief adc get ntc vaule
  * @param None
  * @retval None
  */
uint16_t adc_get_ntc(void)
{
    /* ADC regular channel config */
    adc_regular_channel_config(ADC1, 0U, NTC_ADC_CHANNEL, ADC_SAMPLETIME_7POINT5);
    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC1, ADC_REGULAR_CHANNEL);

    /* wait the end of conversion flag */
    while(!adc_flag_get(ADC1, ADC_FLAG_EOC));
    /* clear the end of conversion flag */
    adc_flag_clear(ADC1, ADC_FLAG_EOC);
    /* return regular channel sample value */
    return (adc_regular_data_read(ADC1));
}

/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
/**
  * @brief 
  * @param None
  * @retval None
  */


/***************************************** (END OF FILE) *********************************************/
