/*
*******************************************************************************************************
*
* File Name : uart.c
* Version   : V1.0
* Author    : mzy2364
* brief     : uart file
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include <string.h>
#include "uart.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/
static uint8_t rxbuffer[32] = {0};
static uint8_t txbuffer[32] = {0};

/* FUNCTION -----------------------------------------------------------------------------------------*/
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
    rcu_periph_clock_enable(RCU_DMA0);
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART2);

    rcu_periph_clock_enable(RCU_AF);
    
    gpio_pin_remap_config(GPIO_USART2_PARTIAL_REMAP,ENABLE);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* connect port to USARTx_Rx */
    gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11);

    /* USART configure */
    usart_deinit(USART2);
    usart_baudrate_set(USART2, 921600U);
    usart_receive_config(USART2, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);
    usart_enable(USART2);
    
    /* deinitialize DMA0 channel1(USART2 tx) */
    dma_deinit(DMA0, DMA_CH1);
    dma_struct_para_init(&dma_init_struct);
    
    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = (uint32_t)txbuffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = ARRAYNUM(txbuffer);
    dma_init_struct.periph_addr = UART2_DATA_ADDRESS;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH1, &dma_init_struct);
    
    /* deinitialize DMA0 channel2 (USART2 rx) */
    dma_deinit(DMA0, DMA_CH2);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)rxbuffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = ARRAYNUM(rxbuffer);
    dma_init_struct.periph_addr = UART2_DATA_ADDRESS;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH2, &dma_init_struct);
    
    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH1);
    dma_memory_to_memory_disable(DMA0, DMA_CH1);
    dma_circulation_disable(DMA0, DMA_CH2);
    dma_memory_to_memory_disable(DMA0, DMA_CH2);
    
    /* enable USART DMA for reception */
    usart_dma_receive_config(USART2, USART_RECEIVE_DMA_ENABLE);
    /* enable DMA0 channel4 transfer complete interrupt */
//    dma_interrupt_enable(DMA0, DMA_CH1, DMA_INT_FTF);
    /* enable DMA0 channel4 */
    dma_channel_enable(DMA0, DMA_CH1);
    /* enable USART DMA for transmission */
    usart_dma_transmit_config(USART2, USART_TRANSMIT_DMA_ENABLE);
    /* enable DMA0 channel3 transfer complete interrupt */
//    dma_interrupt_enable(DMA0, DMA_CH2, DMA_INT_FTF);
    /* enable DMA0 channel3 */
    dma_channel_enable(DMA0, DMA_CH2);
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
        if(dma_flag_get(DMA0, DMA_CH1, DMA_INTF_FTFIF) == SET)
        {
            dma_parameter_struct dma_init_struct;
            
            memcpy(txbuffer,buf,len);
            dma_deinit(DMA0, DMA_CH1);
            dma_struct_para_init(&dma_init_struct);
            dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
            dma_init_struct.memory_addr = (uint32_t)txbuffer;
            dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
            dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
            dma_init_struct.number = len;
            dma_init_struct.periph_addr = UART2_DATA_ADDRESS;
            dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
            dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
            dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
            dma_init(DMA0, DMA_CH1, &dma_init_struct);
            /* configure DMA mode */
            dma_circulation_disable(DMA0, DMA_CH1); 
            /* enable DMA channel4 */
            dma_channel_enable(DMA0, DMA_CH1);
        }
    }
}


/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
/**
  * @brief 
  * @param None
  * @retval None
  */


/***************************************** (END OF FILE) *********************************************/
