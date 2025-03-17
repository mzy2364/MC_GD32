/*
*******************************************************************************************************
*
* File Name : can.c
* Version   : V1.0
* Author    : mzy2364
* brief     : can file
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "can.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/
static void can_gpio_config(void);
static void can_nvic_config(void);
static void can_config(void);

/**
  * @brief CAN INIT
  * @param None
  * @retval None
  */
void can_bus_init(void)
{
    can_gpio_config();
    can_nvic_config();
    can_config();

    can_interrupt_enable(CAN0, CAN_INT_RFNE0);
}



/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
/*!
    \brief      configure GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void can_gpio_config(void)
{
    /* enable clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    
    /* configure CAN0 GPIO */
    gpio_init(GPIOA,GPIO_MODE_IPU,GPIO_OSPEED_50MHZ,GPIO_PIN_11);
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_12);
}

/*!
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void can_nvic_config(void)
{
    /* configure CAN0 NVIC */
    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn,1,1);
}

/*!
    \brief      initialize CAN and filter
    \param[in]  can_parameter
      \arg        can_parameter_struct
    \param[in]  can_filter
      \arg        can_filter_parameter_struct
    \param[out] none
    \retval     none
*/
static void can_config(void)
{
    can_parameter_struct            can_parameter;
    can_filter_parameter_struct     can_filter;
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
    
    rcu_periph_clock_enable(RCU_CAN0);
    
    /* initialize CAN register */
    can_deinit(CAN0);
    
    /* initialize CAN parameters */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = ENABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = ENABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_7TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_2TQ;
    
    /* 1MBps */
#if CAN_BAUDRATE == 1000
    can_parameter.prescaler = 6;
    /* 500KBps */
#elif CAN_BAUDRATE == 500
    can_parameter.prescaler = 12;
    /* 250KBps */
#elif CAN_BAUDRATE == 250
    can_parameter.prescaler = 24;
    /* 125KBps */
#elif CAN_BAUDRATE == 125
    can_parameter.prescaler = 48;
    /* 100KBps */
#elif  CAN_BAUDRATE == 100
    can_parameter.prescaler = 60;
    /* 50KBps */
#elif  CAN_BAUDRATE == 50
    can_parameter.prescaler = 120;
    /* 20KBps */
#elif  CAN_BAUDRATE == 20
    can_parameter.prescaler = 300;
#else
    #error "please select list can baudrate in private defines in main.c "
#endif  
    /* initialize CAN */
    can_init(CAN0, &can_parameter);
    
    /* initialize filter */ 
    can_filter.filter_number=0;
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = 0x0000;
    can_filter.filter_list_low = 0x0000;
    can_filter.filter_mask_high = 0x0000;
    can_filter.filter_mask_low = 0x0000;
    can_filter.filter_fifo_number = CAN_FIFO0;
    can_filter.filter_enable = ENABLE;
    
    can_filter_init(&can_filter);
}

/***************************************** (END OF FILE) *********************************************/
