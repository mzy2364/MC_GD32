/*
*******************************************************************************************************
*
* File Name : led.c
* Version   : V1.0
* Author    : mzy2364
* brief     : led file
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "led.h"
#include "motor_hardware.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/
static uint32_t GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT,LED3_GPIO_PORT};
static uint32_t GPIO_PIN[LEDn] = {LED1_PIN, LED2_PIN, LED3_PIN};

/* FUNCTION -----------------------------------------------------------------------------------------*/
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
    
    rcu_periph_clock_enable(USER_IO1_CLK);
    rcu_periph_clock_enable(USER_IO2_CLK);
    rcu_periph_clock_enable(USER_IO3_CLK);
    
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);

    /* configure led GPIO port */ 
    gpio_init(GPIO_PORT[LED1], GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN[LED1]);
    gpio_init(GPIO_PORT[LED2], GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN[LED2]);
    gpio_init(GPIO_PORT[LED3], GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN[LED3]);

    gpio_bit_write(GPIO_PORT[LED1], GPIO_PIN[LED1],SET);
    gpio_bit_write(GPIO_PORT[LED2], GPIO_PIN[LED2],SET);
    gpio_bit_write(GPIO_PORT[LED3], GPIO_PIN[LED3],SET);
    
    gpio_init(USER_IO1_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,USER_IO1_PIN);
    gpio_init(USER_IO2_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,USER_IO2_PIN);
    gpio_init(USER_IO3_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,USER_IO3_PIN);
    
    gpio_bit_write(USER_IO1_PORT,USER_IO1_PIN,RESET);
    gpio_bit_write(USER_IO2_PORT,USER_IO2_PIN,RESET);
    gpio_bit_write(USER_IO3_PORT,USER_IO3_PIN,RESET);
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
                    (bit_status)(1-gpio_output_bit_get(GPIO_PORT[lednum], GPIO_PIN[lednum])));
}


/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
/**
  * @brief 
  * @param None
  * @retval None
  */


/***************************************** (END OF FILE) *********************************************/
