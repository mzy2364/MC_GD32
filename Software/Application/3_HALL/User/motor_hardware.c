/*
*******************************************************************************************************
*
* File Name : motor_hardware.c
* Version   : V1.0
* Author    : mzy2364
* brief     : motor control hardware config file
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "motor_hardware.h"
#include "motor_config.h"
#include "systick.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/

/* FUNCTION -----------------------------------------------------------------------------------------*/
static void pwm_gpio_init(void);
static void pwm_timer_config(void);
static void adc_gpio_init(void);
static void adc0_config(void);
static void hall_timer_config(void);

/**
  * @brief gpio and timer config
  * @param None
  * @retval None
  */
void motor_hardware_init(void)
{
    comp_protect_init();
    interrupt_init();
    pwm_gpio_init();
    pwm_timer_config();
    hall_init();
    adc0_init();
}

/**
  * @brief nvic config
  * @param None
  * @retval None
  */
void interrupt_init(void)
{
    nvic_irq_enable(TIMER0_BRK_IRQn, 0, 0);
    nvic_irq_enable(ADC0_1_IRQn, 0, 2);
    nvic_irq_enable(TIMER2_IRQn, 1, 1);
}

/**
  * @brief update pwm duty
  * @param duty_u-phase U duty
  * @param duty_v-phase V duty
  * @param duty_w-phase W duty
  * @retval None
  */
void motor_pwm_set_duty(uint16_t duty_u,uint16_t duty_v,uint16_t duty_w)
{
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,duty_u);
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,duty_v);
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,duty_w);
}

/**
  * @brief adc config
  * @param None
  * @retval None
  */
void adc0_init(void)
{
    adc_gpio_init();
    adc0_config();
}

/**
  * @brief DAC comp config
  * @param None
  * @retval None
  */
void comp_protect_init(void)
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
    //dac_data_set(DAC1, DAC_ALIGN_12B_R, PHASE_OVER_CURRENT_DAC_VAULE);
    dac_data_set(DAC1, DAC_ALIGN_12B_R, PHASE_OVER_CURRENT_DAC_VAULE);
}

/**
  * @brief hall sensor init
  * @param None
  * @retval None
  */
void hall_init(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(HALLA_GPIO_CLK);
    rcu_periph_clock_enable(HALLB_GPIO_CLK);
    rcu_periph_clock_enable(HALLC_GPIO_CLK);
    rcu_periph_clock_enable(RCU_AF);
    
    gpio_pin_remap_config(GPIO_TIMER2_FULL_REMAP,ENABLE);
    
    gpio_init(HALLA_GPIO_PORT,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,HALLA_GPIO_PIN);
    gpio_init(HALLB_GPIO_PORT,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,HALLB_GPIO_PIN);
    gpio_init(HALLC_GPIO_PORT,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,HALLC_GPIO_PIN);
    
    hall_timer_config();
}

/**
  * @brief get hall sensor state
  * @param None
  * @retval None
  */
uint8_t hall_get(void)
{
    uint8_t ret = 0;
    
    ret |= gpio_input_bit_get(HALLA_GPIO_PORT,HALLA_GPIO_PIN);
    ret |= gpio_input_bit_get(HALLB_GPIO_PORT,HALLB_GPIO_PIN) << 1;
    ret |= gpio_input_bit_get(HALLC_GPIO_PORT,HALLC_GPIO_PIN) << 2;
    
    return ret;
}

/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/

/**
  * @brief timer gpio init
  * @param None
  * @retval None
  */
static void pwm_gpio_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);

    /*configure PA8 PA9 PA10(TIMER0 CH0 CH1 CH2) as alternate function*/
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_8);
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_9);
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_10);

    /*configure PB13 PB14 PB15(TIMER0 CH0N CH1N CH2N) as alternate function*/
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_13);
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_14);
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_15);

    /*configure PB12(TIMER0 BKIN) as alternate function*/
    gpio_init(GPIOB,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_12);
}

/**
  * @brief adc gpio init
  * @param None
  * @retval None
  */
static void adc_gpio_init(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
}

/**
  * @brief pwm timer init
  * @param None
  * @retval None
  */
static void pwm_timer_config(void)
{
    /* systemcoreclock = 120MHz 
    *  pwm freq = systemcoreclock / (prescaler+1) / period / 2
    *  deadtime = 1.2us
    */
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;
    timer_break_parameter_struct timer_breakpara;

    rcu_periph_clock_enable(RCU_TIMER0);

    timer_deinit(TIMER0);

    /* TIMER0 configuration */
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_UP;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = MOTOR_PWM_PERIOD;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER0,&timer_initpara);

     /* CH0/CH0N,CH1/CH1N and CH2/CH2N configuration in timing mode */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_ENABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER0,TIMER_CH_0,&timer_ocintpara);
    timer_channel_output_config(TIMER0,TIMER_CH_1,&timer_ocintpara);
    timer_channel_output_config(TIMER0,TIMER_CH_2,&timer_ocintpara);
    timer_channel_output_config(TIMER0,TIMER_CH_3,&timer_ocintpara);

    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,1000);
    timer_channel_output_mode_config(TIMER0,TIMER_CH_0,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0,TIMER_CH_0,TIMER_OC_SHADOW_ENABLE);

    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,1000);
    timer_channel_output_mode_config(TIMER0,TIMER_CH_1,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0,TIMER_CH_1,TIMER_OC_SHADOW_ENABLE);

    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,1000);
    timer_channel_output_mode_config(TIMER0,TIMER_CH_2,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0,TIMER_CH_2,TIMER_OC_SHADOW_ENABLE);
    
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,timer_initpara.period - 1);
    timer_channel_output_mode_config(TIMER0,TIMER_CH_3,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0,TIMER_CH_3,TIMER_OC_SHADOW_DISABLE);


    /* automatic output enable, break, dead time and lock configuration*/
    timer_breakpara.runoffstate      = TIMER_ROS_STATE_ENABLE;
    timer_breakpara.ideloffstate     = TIMER_IOS_STATE_ENABLE ;
    timer_breakpara.deadtime         = 0x88;        /* 1.2us */
    timer_breakpara.breakpolarity    = TIMER_BREAK_POLARITY_HIGH;
    timer_breakpara.outputautostate  = TIMER_OUTAUTO_ENABLE;
    timer_breakpara.protectmode      = TIMER_CCHP_PROT_OFF;
    timer_breakpara.breakstate       = TIMER_BREAK_ENABLE;
    timer_break_config(TIMER0,&timer_breakpara);
    
    /* TIMER0 primary output function enable */
    timer_primary_output_config(TIMER0,ENABLE);

    /* TIMER0 channel control update interrupt enable */
    timer_interrupt_enable(TIMER0,TIMER_INT_UP);
    /* TIMER0 break interrupt enable */
    timer_interrupt_enable(TIMER0,TIMER_INT_BRK);
    
    timer_interrupt_enable(TIMER0,TIMER_INT_CH3);

    /* TIMER0 counter enable */
    timer_enable(TIMER0);
}

/**
  * @brief adc0 init
  * @param None
  * @retval None
  * @note ADC Conversion time = (12.5+sample_time) * CLK_ADC
  *       CLK_ADC = APB / 4 = 30M
  *       sample_time = 1.5
  *       ADC Conversion time = (12.5+1.5) * (1/30M) = 0.466us (Single channel)
  */
static void adc0_config(void)
{
    /* enable ADC0 clock */
    rcu_periph_clock_enable(RCU_ADC0);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);
    
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE); 
    /* ADC special function config */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, DISABLE);  
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);

    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 4);
    /* ADC inserted channel config */
    adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_0, ADC_SAMPLETIME_1POINT5);
    adc_inserted_channel_config(ADC0, 1, ADC_CHANNEL_1, ADC_SAMPLETIME_1POINT5);
    adc_inserted_channel_config(ADC0, 2, ADC_CHANNEL_2, ADC_SAMPLETIME_1POINT5);
    adc_inserted_channel_config(ADC0, 3, ADC_CHANNEL_3, ADC_SAMPLETIME_1POINT5);
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T0_CH3); 
    /* ADC external trigger enable */
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);
    
    /* clear the ADC flag */
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOC);
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
    /* enable ADC interrupt */
    adc_interrupt_enable(ADC0, ADC_INT_EOIC);

    /* enable ADC interface */
    adc_enable(ADC0);
    systick_delay(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
}

/**
  * @brief hall timer2 init
  * @param None
  * @retval None
  */
static void hall_timer_config(void)
{
    /* TIMER2 configuration: input capture mode */
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER2);

    timer_deinit(TIMER2);

    /* TIMER2 configuration */
    timer_initpara.prescaler         = HALL_TIMER_PRESCALER;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 0xFFFF;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2,&timer_initpara);

    /* TIMER2  configuration */
    /* TIMER2 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_ITS;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x0;
    timer_input_capture_config(TIMER2,TIMER_CH_0,&timer_icinitpara);
    
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_ITS;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x0;
    timer_input_capture_config(TIMER2,TIMER_CH_1,&timer_icinitpara);
    
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_ITS;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x0;
    timer_input_capture_config(TIMER2,TIMER_CH_2,&timer_icinitpara);
    
    /* slave mode selection: TIMER2 */
    timer_input_trigger_source_select(TIMER2,TIMER_SMCFG_TRGSEL_CI0F_ED);
    timer_slave_mode_select(TIMER2,TIMER_SLAVE_MODE_RESTART);
    
    /* hall mode config */
    timer_hall_mode_config(TIMER2, TIMER_HALLINTERFACE_ENABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER2);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER2,TIMER_INT_FLAG_CH0);
    /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER2,TIMER_INT_CH0);

    /* TIMER2 counter enable */
    timer_enable(TIMER2);
}

/***************************************** (END OF FILE) *********************************************/
