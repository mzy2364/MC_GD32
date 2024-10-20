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
#include "motor_hardware.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/
uint8_t channel_index = 0;
uint8_t adc_channel_buffer[ADC_SIMPLE_CHANNEL] = {NTC_ADC_CHANNEL,VDC_ADC_CHANNEL,IDC_AVER_ADC_CHANNEL};
adc_filter_t adc_filter_handler[ADC_SIMPLE_CHANNEL] = {0};

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
    rcu_periph_clock_enable(IDC_AVER_ADC_CLK);
    
    gpio_init(VDC_ADC_PORT, GPIO_MODE_AIN, GPIO_OSPEED_MAX, VDC_ADC_PIN);
    gpio_init(NTC_ADC_PORT, GPIO_MODE_AIN, GPIO_OSPEED_MAX, NTC_ADC_PIN);
    gpio_init(IDC_AVER_ADC_PORT, GPIO_MODE_AIN, GPIO_OSPEED_MAX, IDC_AVER_ADC_PIN);
    
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
    systick_delay(1U);
    
    /* ADC regular channel config */
    adc_regular_channel_config(ADC1, 0U, adc_channel_buffer[channel_index], ADC_SAMPLETIME_7POINT5);
    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC1, ADC_REGULAR_CHANNEL);
}

/**
  * @brief ADC conversion task
  * @param None
  * @retval None
  * @note Called in IDLE task
  */
void adc_task(void)
{
    if(adc_flag_get(ADC1, ADC_FLAG_EOC) == SET)
    {
        adc_flag_clear(ADC1, ADC_FLAG_EOC);
        adc_filter_handler[channel_index].buffer[adc_filter_handler[channel_index].count] = adc_regular_data_read(ADC1);
        adc_filter_handler[channel_index].count++;
        if(adc_filter_handler[channel_index].count >= ADC_FILTER_COUNT)
        {
            adc_filter_handler[channel_index].count = 0;
        }
        
        channel_index++;
        if(channel_index >= ADC_SIMPLE_CHANNEL)
        {
            channel_index = 0;
        }
        adc_regular_channel_config(ADC1, 0U, adc_channel_buffer[channel_index], ADC_SAMPLETIME_7POINT5);
        adc_software_trigger_enable(ADC1, ADC_REGULAR_CHANNEL);
    }
}

/**
  * @brief read adc result
  * @param ch-adc channel
  * @retval 12bit adc data
  * @note
  */
uint16_t adc_read_data(adc_ch_t ch)
{
    uint32_t sum = 0;
    uint16_t i = 0;
    if(ch < ADC_SIMPLE_CHANNEL)
    {
        for(i=0;i<ADC_FILTER_COUNT;i++)
        {
            sum += adc_filter_handler[ch].buffer[i];
        }
        return (sum >> ADC_FILTER_COUNT_POWER);
    }
    return 0;
}

/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
/**
  * @brief 
  * @param None
  * @retval None
  */


/***************************************** (END OF FILE) *********************************************/
