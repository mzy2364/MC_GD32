/*
*******************************************************************************************************
*
* File Name : pwm_input.c
* Version   : V1.0
* Author    : mzy2364
* brief     : pwm input driver file
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "pwm_input.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/
uint32_t ic1value = 0,ic2value = 0;
__IO uint16_t dutycycle = 0;        /* 0-1000 */
__IO uint16_t frequency = 0;        /* unit Hz */
__IO uint8_t pwm_input_valid = 0;

/* FUNCTION -----------------------------------------------------------------------------------------*/
static void pwm_input_gpio_init(void);

/**
  * @brief pwm input capture init
  * @param None
  * @retval None
  */
void pwm_input_init(void)
{
    pwm_input_gpio_init();
    nvic_irq_enable(TIMER3_IRQn, 1, 1);
    
 /* TIMER3 configuration: PWM input mode ------------------------
     the external signal is connected to TIMER3 CH0 pin
     the rising edge is used as active edge
     the TIMER3 CH0CV is used to compute the frequency value 
     the TIMER3 CH1CV is used to compute the duty cycle value
  ------------------------------------------------------------ */
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER3);

    timer_deinit(TIMER3);

    /* TIMER2 configuration */
    timer_initpara.prescaler         = 119;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 65535;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3,&timer_initpara);

    /* TIMER2 configuration */
    /* TIMER2 CH0 PWM input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x0;
    timer_input_pwm_capture_config(TIMER3,TIMER_CH_0,&timer_icinitpara);

    /* slave mode selection: TIMER2 */
    timer_input_trigger_source_select(TIMER3,TIMER_SMCFG_TRGSEL_CI0FE0);
    timer_slave_mode_select(TIMER3,TIMER_SLAVE_MODE_RESTART);

    /* select the master slave mode */
    timer_master_slave_mode_config(TIMER3,TIMER_MASTER_SLAVE_MODE_ENABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER3);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_CH0);
    timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_UP);
    /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER3,TIMER_INT_CH0);
    timer_interrupt_enable(TIMER3,TIMER_INT_UP);

    /* TIMER2 counter enable */
    timer_enable(TIMER3);
}

/**
  * @brief pwm input capture init
  * @param None
  * @retval None
  */
uint8_t pwm_input_get_data(uint16_t *freq,uint16_t *duty)
{
    if(pwm_input_valid)
    {
        *freq = frequency;
        *duty = dutycycle;
    }
    else
    {
        *freq = 0;
        *duty = 0;
    }
    return pwm_input_valid;
}

/*!
    \brief      this function handles TIMER2 interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER3_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER3,TIMER_INT_FLAG_CH0))
    {
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_CH0);
        timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_UP);
        /* read channel 0 capture value */
        ic1value = timer_channel_capture_value_register_read(TIMER3,TIMER_CH_0)+1;

        if(0 != ic1value){
            /* read channel 1 capture value */
            ic2value = timer_channel_capture_value_register_read(TIMER3,TIMER_CH_1)+1;

            /* calculate the duty cycle value */
            dutycycle = (ic2value * 1000) / ic1value;
            /* calculate the frequency value */
            frequency = 1000000 / ic1value;
            
            pwm_input_valid = 1;

        }else{
            dutycycle = 0;
            frequency = 0;
        }
    }
    else if(SET == timer_interrupt_flag_get(TIMER3,TIMER_INT_FLAG_UP))
    {
        timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_UP);
        dutycycle = 0;
        frequency = 0;
        pwm_input_valid = 0;
    }
}

/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
/**
  * @brief 
  * @param None
  * @retval None
  */
static void pwm_input_gpio_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);

    /*configure PB6 (TIMER3 CH0) as alternate function*/
    gpio_init(GPIOB,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_6);
}

/***************************************** (END OF FILE) *********************************************/
