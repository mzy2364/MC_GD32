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
#include <string.h>
#include "motor_hardware.h"
#include "motor_config.h"
#include "gd32f30x.h"
#include "gd32f30x_libopt.h"
#include "systick.h"
#include "ringbuffer.h"

/* DEFINES ------------------------------------------------------------------------------------------*/
#define ARRAYNUM(arr_name)      (uint32_t)(sizeof(arr_name) / sizeof(*(arr_name)))
#define UART3_DATA_ADDRESS      ((uint32_t)&USART_DATA(UART3))

/* VARIABLES ----------------------------------------------------------------------------------------*/
static uint32_t GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT,LED3_GPIO_PORT};
static uint32_t GPIO_PIN[LEDn] = {LED1_PIN, LED2_PIN, LED3_PIN};

uint8_t rxbuffer[32] = {0};
uint8_t txbuffer[32] = {0};

/* FUNCTION -----------------------------------------------------------------------------------------*/
static void pwm_gpio_init(void);
static void pwm_timer_config(void);
static void adc_gpio_init(void);
static void adc0_config(void);
static void adc1_config(void);
static void hall_timer_config(void);

/**
  * @brief gpio and timer config
  * @param None
  * @retval None
  */
void pwm_init(void)
{
    pwm_gpio_init();
    pwm_timer_config();
}

/**
  * @brief nvic config
  * @param None
  * @retval None
  */
void interrupt_init(void)
{
    nvic_irq_enable(TIMER0_Channel_IRQn, 1, 3);
    nvic_irq_enable(TIMER0_BRK_IRQn, 0, 0);
    
    nvic_irq_enable(TIMER0_UP_IRQn, 1, 2);
//    nvic_irq_enable(TIMER1_IRQn, 0, 2);
//    nvic_irq_enable(ADC0_1_IRQn, 0, 2);
//    nvic_irq_enable(TIMER2_IRQn, 1, 1);
}

/**
  * @brief adc config
  * @param None
  * @retval None
  */
void adc_init(void)
{
    adc_gpio_init();
    adc0_config();
    adc1_config();
}

/**
  * @brief adc get dc bus voltage
  * @param None
  * @retval None
  */
uint16_t adc_get_vdc(void)
{
    /* ADC regular channel config */
    adc_regular_channel_config(ADC1, 0U, ADC_CHANNEL_14, ADC_SAMPLETIME_7POINT5);
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
    adc_regular_channel_config(ADC1, 0U, ADC_CHANNEL_7, ADC_SAMPLETIME_7POINT5);
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
  * @brief led config
  * @param None
  * @retval None
  */
void led_init(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(LED1_GPIO_CLK);
    rcu_periph_clock_enable(LED2_GPIO_CLK);
    rcu_periph_clock_enable(LED3_GPIO_CLK);
    rcu_periph_clock_enable(RCU_AF);
    
    rcu_periph_clock_enable(GPIO1_GPIO_CLK);
    rcu_periph_clock_enable(GPIO2_GPIO_CLK);
    rcu_periph_clock_enable(GPIO3_GPIO_CLK);
    
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);

    /* configure led GPIO port */ 
    gpio_init(GPIO_PORT[LED1], GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN[LED1]);
    gpio_init(GPIO_PORT[LED2], GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN[LED2]);
    gpio_init(GPIO_PORT[LED3], GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN[LED3]);
    
    GPIO_BC(GPIO_PORT[LED1]) = GPIO_PIN[LED1];
    GPIO_BC(GPIO_PORT[LED2]) = GPIO_PIN[LED2];
    GPIO_BC(GPIO_PORT[LED3]) = GPIO_PIN[LED3];
    
    gpio_init(GPIO1_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO1_PIN);
    gpio_init(GPIO2_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO2_PIN);
    gpio_init(GPIO3_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO3_PIN);
    
    gpio_bit_write(GPIO1_GPIO_PORT,GPIO1_PIN,RESET);
    gpio_bit_write(GPIO2_GPIO_PORT,GPIO2_PIN,RESET);
    gpio_bit_write(GPIO3_GPIO_PORT,GPIO3_PIN,RESET);
}


/**
  * @brief turn on selected led
  * @param lednum: specify the led to be turned on
  * @retval None
  */
void led_on(led_typedef_enum lednum)
{
    GPIO_BC(GPIO_PORT[lednum]) = GPIO_PIN[lednum];
}

/**
  * @brief turn off selected led
  * @param lednum: specify the led to be turned on
  * @retval None
  */
void led_off(led_typedef_enum lednum)
{
    GPIO_BOP(GPIO_PORT[lednum]) = GPIO_PIN[lednum];
}

/**
  * @brief toggle led
  * @param lednum: specify the led to be turned on
  * @retval None
  */
void led_toggle(led_typedef_enum lednum)
{
    gpio_bit_write(GPIO_PORT[lednum], GPIO_PIN[lednum], 
                    (bit_status)(1-gpio_input_bit_get(GPIO_PORT[lednum], GPIO_PIN[lednum])));
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
    //dac_data_set(DAC1, DAC_ALIGN_12B_R, PHASE_OVER_CURRENT_DAC_VAULE);
    dac_data_set(DAC1, DAC_ALIGN_12B_R, (uint16_t)PHASE_OVER_CURRENT_DAC_VAULE);
}

/**
  * @brief uart init
  * @param None
  * @retval None
  */
void usart_init(void)
{
    dma_parameter_struct dma_init_struct;
    
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOC);
    /* enable DMA0 */
    rcu_periph_clock_enable(RCU_DMA1);
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_UART3);
    
//    nvic_irq_enable(DMA1_Channel2_IRQn, 0, 0);
//    nvic_irq_enable(DMA1_Channel3_Channel4_IRQn, 0, 1);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* connect port to USARTx_Rx */
    gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11);

    /* USART configure */
    usart_deinit(UART3);
    usart_baudrate_set(UART3, 921600U);
    usart_receive_config(UART3, USART_RECEIVE_ENABLE);
    usart_transmit_config(UART3, USART_TRANSMIT_ENABLE);
    usart_enable(UART3);
    
    /* deinitialize DMA channel4(USART3 tx) */
    dma_deinit(DMA1, DMA_CH4);
    dma_struct_para_init(&dma_init_struct);
    
    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = (uint32_t)txbuffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = ARRAYNUM(txbuffer);
    dma_init_struct.periph_addr = UART3_DATA_ADDRESS;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA1, DMA_CH4, &dma_init_struct);
    
    /* deinitialize DMA channel4 (UART3 rx) */
    dma_deinit(DMA1, DMA_CH2);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)rxbuffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = ARRAYNUM(rxbuffer);
    dma_init_struct.periph_addr = UART3_DATA_ADDRESS;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA1, DMA_CH2, &dma_init_struct);
    
    /* configure DMA mode */
    dma_circulation_disable(DMA1, DMA_CH4);
    dma_memory_to_memory_disable(DMA1, DMA_CH4);
    dma_circulation_disable(DMA1, DMA_CH2);
    dma_memory_to_memory_disable(DMA1, DMA_CH2);
    
    /* enable USART DMA for reception */
    usart_dma_receive_config(UART3, USART_RECEIVE_DMA_ENABLE);
    /* enable DMA0 channel4 transfer complete interrupt */
//    dma_interrupt_enable(DMA1, DMA_CH4, DMA_INT_FTF);
    /* enable DMA0 channel4 */
    dma_channel_enable(DMA1, DMA_CH4);
    /* enable USART DMA for transmission */
    usart_dma_transmit_config(UART3, USART_TRANSMIT_DMA_ENABLE);
    /* enable DMA0 channel3 transfer complete interrupt */
//    dma_interrupt_enable(DMA1, DMA_CH2, DMA_INT_FTF);
    /* enable DMA0 channel3 */
    dma_channel_enable(DMA1, DMA_CH2);
}

/**
  * @brief Asynchronous transmission
  * @param None
  * @retval None
  */
void usart_send_data(uint8_t *buf,uint8_t len)
{
    if(len < ARRAYNUM(txbuffer))
    {
        if(dma_flag_get(DMA1, DMA_CH4, DMA_INTF_FTFIF) == SET)
        {
            dma_parameter_struct dma_init_struct;
            
            memcpy(txbuffer,buf,len);
            
            dma_deinit(DMA1, DMA_CH4);
            usart_flag_clear(USART0, USART_FLAG_RBNE);
            usart_dma_receive_config(USART0, USART_RECEIVE_DMA_ENABLE);
            dma_struct_para_init(&dma_init_struct);
            dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
            dma_init_struct.memory_addr = (uint32_t)txbuffer;
            dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
            dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
            dma_init_struct.number = len;
            dma_init_struct.periph_addr = UART3_DATA_ADDRESS;
            dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
            dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
            dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
            dma_init(DMA1, DMA_CH4, &dma_init_struct);
            /* configure DMA mode */
            dma_circulation_disable(DMA1, DMA_CH4); 
            /* enable DMA channel4 */
            dma_channel_enable(DMA1, DMA_CH4);
        }
    }

}

/**
  * @brief 设置 ADC 的采样时间点
  * @param None
  * @retval None
  * @note
  */
void motor_pwm_set_adc_trigger_point(uint16_t trigger_point0, uint16_t trigger_point1)
{
    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_0,trigger_point0);
    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_1,trigger_point1);
}

/**
  * @brief 获取相电流的 ADC 值
  * @param None
  * @retval None
  * @note
  */
void motor_get_phase_current_adc(int16_t *p_ia, int16_t *p_ib)
{
	*p_ia = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_0);
	*p_ib = adc_regular_data_read(ADC0);
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
    rcu_periph_clock_enable(RCU_GPIOC);
    
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_7);
    gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_4);
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
    rcu_periph_clock_enable(RCU_TIMER1);
    
    /* TIMER1  configuration */
    timer_deinit(TIMER1);

    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_UP;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = MOTOR_PWM_PERIOD;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);

    /* CH1 configuration in PWM0 mode */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_ENABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER1,TIMER_CH_0,&timer_ocintpara);
    timer_channel_output_config(TIMER1,TIMER_CH_1,&timer_ocintpara);

    /* CHx 必须配置为 PWM1,这样在周期的前半段才能产生上升沿触发ADC */
    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_0,timer_initpara.period/2);
    timer_channel_output_mode_config(TIMER1,TIMER_CH_0,TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(TIMER1,TIMER_CH_0,TIMER_OC_SHADOW_ENABLE);
    
    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_1,timer_initpara.period/2);
    timer_channel_output_mode_config(TIMER1,TIMER_CH_1,TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(TIMER1,TIMER_CH_1,TIMER_OC_SHADOW_ENABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    /* select the master slave mode */
    timer_master_slave_mode_config(TIMER1,TIMER_MASTER_SLAVE_MODE_ENABLE);
    /* TIMER1 update event is used as trigger output */
    timer_master_output_trigger_source_select(TIMER1,TIMER_TRI_OUT_SRC_ENABLE);

    timer_interrupt_enable(TIMER1,TIMER_INT_CH0);
    timer_interrupt_enable(TIMER1,TIMER_INT_CH1);

    /* TIMER0 configuration */
    timer_deinit(TIMER0);

    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_DOWN;
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
    
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,300);
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
    
    /* slave mode selection: TIMER0 */
    timer_slave_mode_select(TIMER0,TIMER_SLAVE_MODE_EVENT);
    timer_input_trigger_source_select(TIMER0,TIMER_SMCFG_TRGSEL_ITI1);

    /* TIMER0 channel control update interrupt enable */
    timer_interrupt_enable(TIMER0,TIMER_INT_UP);
    /* TIMER0 break interrupt enable */
    timer_interrupt_enable(TIMER0,TIMER_INT_BRK);
    
    timer_interrupt_enable(TIMER0,TIMER_INT_CH3);

    /* TIMER counter enable */
//    timer_enable(TIMER0);
    timer_enable(TIMER1);
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
    adc_special_function_config(ADC0, ADC_SCAN_MODE, DISABLE);
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, DISABLE);  
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);

    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 1);
    /* ADC inserted channel config */
    adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_1, ADC_SAMPLETIME_1POINT5);
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T1_CH0); 
    /* ADC external trigger enable */
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);
    
    
    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 1);
    /* ADC regular channel config */
    adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_1, ADC_SAMPLETIME_1POINT5);
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_T1_CH1); 
    /* ADC external trigger config */
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);
    
    
    /* clear the ADC flag */
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOC);
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
    /* enable ADC interrupt */
    adc_interrupt_enable(ADC0, ADC_INT_FLAG_EOC);
    adc_interrupt_enable(ADC0, ADC_INT_FLAG_EOIC);

    /* enable ADC interface */
    adc_enable(ADC0);
    systick_delay(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
}

/**
  * @brief adc1 init
  * @param None
  * @retval None
  */
static void adc1_config(void)
{
    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_ADC1);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6);
    
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

/*!
    \brief      this function handles DMA0_Channel3_IRQHandler interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DMA1_Channel2_IRQHandler(void)
{
    if(dma_interrupt_flag_get(DMA1, DMA_CH4, DMA_INT_FLAG_FTF)) {
        dma_interrupt_flag_clear(DMA1, DMA_CH4, DMA_INT_FLAG_G);
    }
}

/*!
    \brief      this function handles DMA0_Channel4_IRQHandler interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DMA1_Channel3_4_IRQHandler(void)
{
    if(dma_interrupt_flag_get(DMA1, DMA_CH4, DMA_INT_FLAG_FTF)) {
        dma_interrupt_flag_clear(DMA1, DMA_CH4, DMA_INT_FLAG_G);
    }
}

/***************************************** (END OF FILE) *********************************************/
