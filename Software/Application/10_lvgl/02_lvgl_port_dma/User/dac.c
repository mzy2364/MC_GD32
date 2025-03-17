/*
*******************************************************************************************************
*
* File Name : dac.c
* Version   : V1.0
* Author    : mzy2364
* brief     : dac file
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "dac.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/
/**
  * @brief DAC config
  * @param None
  * @retval None
  */
void dac_init(void)
{
    /* enable the clock of peripherals */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_DAC);
    
    /* once enabled the DAC, the corresponding GPIO pin is connected to the DAC converter automatically */
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_5);
    
    dac_deinit();
    /* configure the DAC0 */
    dac_wave_mode_config(DAC1, DAC_WAVE_MODE_LFSR);
    dac_lfsr_noise_config(DAC1, DAC_LFSR_BITS10_0);
    
    /* enable DAC0 and set data */
    dac_enable(DAC1);
    dac_data_set(DAC1, DAC_ALIGN_12B_R, 2048);
}


/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
/**
  * @brief 
  * @param None
  * @retval None
  */


/***************************************** (END OF FILE) *********************************************/
