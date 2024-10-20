/*
*******************************************************************************************************
*
* File Name : encoder.c
* Version   : V1.0
* Author    : mzy2364
* brief     : dac file
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "encoder.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/
static encoder_code_t encoder_code = ENCODER_NONE;

/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
static void encoder_gpio_init(void);
static void encoder_timer_init(void);

/* FUNCTION -----------------------------------------------------------------------------------------*/
/**
  * @brief encoder timer init
  * @param None
  * @retval None
  */
void encoder_init(void)
{
    nvic_irq_enable(TIMER3_IRQn, 1, 1);
    encoder_gpio_init();
    encoder_timer_init();
}

encoder_code_t encoder_get(void)
{
    encoder_code_t code = encoder_code;
    encoder_code = ENCODER_NONE;
    return code;
}

void TIMER3_IRQHandler(void)
{
    int16_t cnt = 0;
    if(SET == timer_interrupt_flag_get(TIMER3,TIMER_INT_FLAG_TRG))
    {
        timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_TRG);
        cnt = timer_counter_read(TIMER3);
        timer_counter_value_config(TIMER3,0);
        if(cnt > 0)
        {
            encoder_code = ENCODER_INC;
        }
        else
        {
            encoder_code = ENCODER_DEC;
        }
    }
}


/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
/**
  * @brief 
  * @param None
  * @retval None
  */
static void encoder_gpio_init(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(ENCODERA_GPIO_CLK);
    rcu_periph_clock_enable(ENCODERB_GPIO_CLK);

    gpio_init(ENCODERA_GPIO_PORT,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,ENCODERA_GPIO_PIN);
    gpio_init(ENCODERB_GPIO_PORT,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,ENCODERB_GPIO_PIN);
}


static void encoder_timer_init(void)
{
    /* TIMER3 configuration: input capture mode */
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER3);

    timer_deinit(TIMER3);

    /* TIMER3 configuration */
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 0xFFFF;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3,&timer_initpara);

    /* TIMER3  configuration */
    /* TIMER3 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x10;
    timer_input_capture_config(TIMER3,TIMER_CH_0,&timer_icinitpara);
    timer_input_capture_config(TIMER3,TIMER_CH_1,&timer_icinitpara);

    timer_input_trigger_source_select(TIMER3,TIMER_SMCFG_TRGSEL_CI0FE0);
    timer_quadrature_decoder_mode_config(TIMER3,TIMER_ENCODER_MODE2,TIMER_IC_POLARITY_RISING,TIMER_IC_POLARITY_RISING);
    
    timer_interrupt_enable(TIMER3,TIMER_INT_TRG);

    /* TIMER2 counter enable */
    timer_enable(TIMER3);
}

/***************************************** (END OF FILE) *********************************************/
