/*
*******************************************************************************************************
*
* File Name : RS485.c
* Version   : V1.0
* Author    : mzy2364
* brief     : RS485 file
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include <string.h>
#include "rs485.h"
#include "ring_buffer.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/
static uint8_t rs485_rxbuffer[64] = {0};
static uint8_t rs485_txbuffer[64] = {0};

static uint8_t rs485_usart_idle_status = 0;
static RingBuff_t rs485_rb_hadle;

/* FUNCTION -----------------------------------------------------------------------------------------*/
/**
  * @brief uart init
  * @param None
  * @retval None
  */
void rs485_usart_init(void)
{
    dma_parameter_struct dma_init_struct;
    
    nvic_irq_enable(RS485_UART_IRQn, 2, 1);
    
    /* enable GPIO clock */
    rcu_periph_clock_enable(RS485_UART_TX_GPIO_CLK_EN);
    rcu_periph_clock_enable(RS485_UART_RX_GPIO_CLK_EN);
    rcu_periph_clock_enable(RS485_EN_GPIO_CLK_EN);
    /* enable RS485_USART_TX_DMA */
    rcu_periph_clock_enable(RS485_USART_TX_DMA_CLK_EN);
    /* enable USART clock */
    rcu_periph_clock_enable(RS485_UART_CLK_EN);

    rcu_periph_clock_enable(RCU_AF);
    
#ifdef RS485_UART_GPIO_REMAP
    gpio_pin_remap_config(RS485_UART_GPIO_REMAP,ENABLE);
#endif

    /* connect port to RS485_EN */
    gpio_init(RS485_EN_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, RS485_EN_GPIO_PIN);
    gpio_bit_write(RS485_EN_GPIO_PORT,RS485_EN_GPIO_PIN,RESET);
    
    /* connect port to USARTx_Tx */
    gpio_init(RS485_UART_TX_GPIO_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, RS485_UART_TX_GPIO_PIN);

    /* connect port to USARTx_Rx */
    gpio_init(RS485_UART_RX_GPIO_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, RS485_UART_RX_GPIO_PIN);

    /* USART configure */
    usart_deinit(RS485_UART_PERIPH);
    usart_baudrate_set(RS485_UART_PERIPH, RS485_USART_BAUDRATE);
    usart_receive_config(RS485_UART_PERIPH, USART_RECEIVE_ENABLE);
    usart_transmit_config(RS485_UART_PERIPH, USART_TRANSMIT_ENABLE);
    usart_enable(RS485_UART_PERIPH);
    usart_flag_clear(RS485_UART_PERIPH, USART_FLAG_RBNE);
    usart_flag_clear(RS485_UART_PERIPH, USART_FLAG_IDLE);
    usart_flag_clear(RS485_UART_PERIPH, USART_FLAG_TC);
    usart_interrupt_enable(RS485_UART_PERIPH, USART_INT_RBNE);
    usart_interrupt_enable(RS485_UART_PERIPH, USART_INT_IDLE);
    usart_interrupt_enable(RS485_UART_PERIPH, USART_INT_TC);
    
    /* deinitialize RS485_USART_TX_DMA channel1(UART TX) */
    dma_deinit(RS485_USART_TX_DMA, RS485_USART_TX_DMA_CH);
    dma_struct_para_init(&dma_init_struct);
    
    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = (uint32_t)rs485_txbuffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = ARRAYNUM(rs485_txbuffer);
    dma_init_struct.periph_addr = RS485_UART_DATA_ADDRESS;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(RS485_USART_TX_DMA, RS485_USART_TX_DMA_CH, &dma_init_struct);
    
    /* configure DMA mode */
    dma_circulation_disable(RS485_USART_TX_DMA, RS485_USART_TX_DMA_CH);
    dma_memory_to_memory_disable(RS485_USART_TX_DMA, RS485_USART_TX_DMA_CH);
    
    /* enable USART DMA for reception */
    usart_dma_receive_config(RS485_UART_PERIPH, USART_RECEIVE_DMA_DISABLE);
    /* enable USART DMA for transmission */
    usart_dma_transmit_config(RS485_UART_PERIPH, USART_TRANSMIT_DMA_ENABLE);
    /* enable DMA channel transfer complete interrupt */
    dma_interrupt_enable(RS485_USART_TX_DMA, RS485_USART_TX_DMA_CH, DMA_INT_FTF);
    /* enable RS485_USART_TX_DMA channel3 */
    dma_channel_enable(RS485_USART_TX_DMA, RS485_USART_TX_DMA_CH);
    
    
    ring_buffer_create(&rs485_rb_hadle,rs485_rxbuffer,sizeof(rs485_rxbuffer));
}

/**
  * @brief Asynchronous transmission
  * @param None
  * @retval None
  */
void rs485_usart_send_data(uint8_t *buf,uint8_t len)
{
	gpio_bit_write(RS485_EN_GPIO_PORT,RS485_EN_GPIO_PIN,SET);

    if(len < ARRAYNUM(rs485_txbuffer))
    {
        memcpy(rs485_txbuffer,buf,len);
        
        dma_channel_disable(RS485_USART_TX_DMA, RS485_USART_TX_DMA_CH);
        dma_transfer_number_config(RS485_USART_TX_DMA, RS485_USART_TX_DMA_CH,len);
        dma_channel_enable(RS485_USART_TX_DMA, RS485_USART_TX_DMA_CH);
    }
}

/**
  * @brief 从RS485缓冲区读取数据
  * @param rbuff:存储读取到的数据的缓冲区
  *        len:需要读取的数据长度
  * @retval	读取到的真实数据长度
  * @note 
  */
uint32_t rs485_fifo_read(uint8_t *rbuff,uint32_t len)
{
    return ring_buffer_read(&rs485_rb_hadle,rbuff,len);
}

/**
  * @brief 获取RS485 FIFO的数据长度
  * @param None
  * @retval	FIFO的数据长度
  * @note 
  */
uint32_t rs485_fifo_get_data_length(void)
{
    return ring_buffer_can_read_length(&rs485_rb_hadle);
}

/**
  * @brief 获取RS485的串口IDLE状态
  * @param None
  * @retval	串口空闲状态 如果串口进入了空闲中断 返回1 否则返回0
  * @note 串口接收完成和发送完成都会触发空闲中断的
  */
uint8_t rs485_get_idle_status(void)
{
	return rs485_usart_idle_status;
}

/**
  * @brief 清除RS485的串口IDLE状态
  * @param None
  * @retval	None
  * @note 主程序调用该函数用于清除idle状态用于下次接收
  */
void rs485_clear_idle_status(void)
{
	rs485_usart_idle_status = 0;
}

/**
  * @brief UART Receive interrupt
  * @param None
  * @retval None
  */
void RS485_UART_IRQHandler(void)
{
    uint8_t uart_data = 0;
    if(usart_flag_get(RS485_UART_PERIPH,USART_FLAG_RBNE) != RESET)
    {
        usart_flag_clear(RS485_UART_PERIPH, USART_FLAG_RBNE);
        uart_data = usart_data_receive(RS485_UART_PERIPH);
        ring_buffer_write(&rs485_rb_hadle,&uart_data,1);
    }
    else if(usart_flag_get(RS485_UART_PERIPH,USART_FLAG_IDLE) != RESET)
    {
        usart_flag_clear(RS485_UART_PERIPH, USART_FLAG_IDLE);
        uart_data = usart_data_receive(RS485_UART_PERIPH);
        rs485_usart_idle_status = 1;
    }
    else if(usart_flag_get(RS485_UART_PERIPH,USART_FLAG_TC) != RESET)
    {
        usart_flag_clear(RS485_UART_PERIPH, USART_FLAG_TC);
        gpio_bit_write(RS485_EN_GPIO_PORT,RS485_EN_GPIO_PIN,RESET);
    }
}

/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
/**
  * @brief 
  * @param None
  * @retval None
  */


/***************************************** (END OF FILE) *********************************************/
