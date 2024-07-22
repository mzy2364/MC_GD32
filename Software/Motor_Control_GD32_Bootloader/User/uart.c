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
#include "gd32f30x.h"
#include "gd32f30x_libopt.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/
static uint16_t rxcount = 0; 
static uint8_t rxbuffer[2048] = {0};
static uint8_t uart_receive_finish = 0;

/* FUNCTION -----------------------------------------------------------------------------------------*/

/**
  * @brief uart init
  * @param None
  * @retval None
  */
void uart3_init(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOC);
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_UART3);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* connect port to USARTx_Rx */
    gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11);

    /* USART interrupt configuration */
    nvic_irq_enable(UART3_IRQn, 0, 1);
    
    /* USART configure */
    usart_deinit(UART3);
    usart_baudrate_set(UART3, 921600U);
    usart_receive_config(UART3, USART_RECEIVE_ENABLE);
    usart_transmit_config(UART3, USART_TRANSMIT_ENABLE);
    usart_interrupt_enable(UART3, USART_INT_RBNE);
    usart_interrupt_enable(UART3, USART_INT_IDLE);
    usart_enable(UART3);
}

/**
  * @brief uart deinit
  * @param None
  * @retval None
  */
void uart3_deinit(void)
{
    nvic_irq_disable(UART3_IRQn);
    usart_disable(UART3);
}

/**
  * @brief uart get receive data
  * @param buf-接收数据缓冲区
  * @param buf_len-缓冲区长度
  * @retval 返回收到的数据
  */
uint32_t uart3_get_receive_data(uint8_t *buf,uint32_t buf_len)
{
    uint32_t data_len = 0;
    if(uart_receive_finish)
    {
        uart_receive_finish = 0;
        if(buf_len >= rxcount)
        {
            data_len = rxcount;
            memcpy(buf,rxbuffer,data_len);
            rxcount = 0;
            return data_len;
        }
        else
        {
            data_len = buf_len;
            memcpy(buf,rxbuffer,buf_len);
            rxcount -= buf_len;
            return data_len;
        }
    }
    return 0;
}

/**
  * @brief uart3 transmit ata
  * @param buf-发送数据缓冲区
  * @param buf_len-缓冲区长度
  * @retval None
  */
void uart3_transmit_data(uint8_t *buf,uint32_t buf_len)
{
    uint32_t i = 0;
    while(buf_len--) 
    {
        /* wait until end of transmit */
        while(RESET == usart_flag_get(UART3, USART_FLAG_TBE)) {
        }
        usart_data_transmit(UART3, buf[i++]);
    }
}

/**
  * @brief uart3 transmit ata
  * @param buf-发送数据缓冲区
  * @param buf_len-缓冲区长度
  * @retval None
  */
void uart3_transmit_string(const char *buf)
{
    uint32_t i = 0;

    while (buf[i] != '\0')
    {
        /* wait until end of transmit */
        while(RESET == usart_flag_get(UART3, USART_FLAG_TBE)) {
        }
        usart_data_transmit(UART3, buf[i]);
        i++;
    }
}

/**
  * @brief uart3 transmit ata
  * @param buf-发送数据缓冲区
  * @param buf_len-缓冲区长度
  * @retval None
  */
void uart3_transmit_byte(uint8_t byte)
{
    /* wait until end of transmit */
    while(RESET == usart_flag_get(UART3, USART_FLAG_TBE)) {
    }
    usart_data_transmit(UART3, byte);
}

/*!
    \brief      this function handles USART RBNE interrupt request and TBE interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UART3_IRQHandler(void)
{
    if(RESET != usart_interrupt_flag_get(UART3, USART_INT_FLAG_RBNE))
    {
        usart_interrupt_flag_clear(UART3, USART_INT_FLAG_RBNE);
        if(!uart_receive_finish)
        {
            /* receive data */
            if(rxcount < sizeof(rxbuffer))
            {
                rxbuffer[rxcount++] = usart_data_receive(UART3);
            }
        }
    }
    if(RESET != usart_interrupt_flag_get(UART3, USART_INT_FLAG_IDLE))
    {
        /* clear IDLE flag */
        usart_data_receive(UART3);
        usart_interrupt_flag_clear(UART3, USART_INT_FLAG_IDLE);
        uart_receive_finish = 1;
    }
}

/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
/**
  * @brief 
  * @param None
  * @retval None
  */


/***************************************** (END OF FILE) *********************************************/
