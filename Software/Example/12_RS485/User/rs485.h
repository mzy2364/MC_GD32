/*
*******************************************************************************************************
*
* File Name : RS485.h
* Version   : V1.0
* Author    : mzy2364
* brief     : RS485 header file
* 
*******************************************************************************************************
*/
#ifndef _RS485_H_
#define _RS485_H_

#ifdef __cplusplus
extern "C"{
#endif

/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "gd32f30x.h"


/* DEFINES ------------------------------------------------------------------------------------------*/
#define ARRAYNUM(arr_name)      (uint32_t)(sizeof(arr_name) / sizeof(*(arr_name)))
#define UART2_DATA_ADDRESS      ((uint32_t)&USART_DATA(USART2))


#define RS485_USART_TX_DMA_CLK_EN   RCU_DMA0
#define RS485_USART_TX_DMA          DMA0
#define RS485_USART_TX_DMA_CH       DMA_CH3

#define RS485_UART_PERIPH           USART0
#define RS485_UART_DATA_ADDRESS     ((uint32_t)&USART_DATA(USART0))
#define RS485_UART_CLK_EN           RCU_USART0
#define RS485_UART_IRQn             USART0_IRQn
#define RS485_UART_IRQHandler       USART0_IRQHandler

#define RS485_UART_GPIO_REMAP       GPIO_USART0_REMAP

#define RS485_UART_TX_GPIO_PORT     GPIOB
#define RS485_UART_TX_GPIO_PIN      GPIO_PIN_6
#define RS485_UART_TX_GPIO_CLK_EN   RCU_GPIOB
#define RS485_UART_RX_GPIO_PORT     GPIOB
#define RS485_UART_RX_GPIO_PIN      GPIO_PIN_7
#define RS485_UART_RX_GPIO_CLK_EN   RCU_GPIOB
#define RS485_EN_GPIO_PORT          GPIOC
#define RS485_EN_GPIO_PIN           GPIO_PIN_0
#define RS485_EN_GPIO_CLK_EN        RCU_GPIOC

#define RS485_USART_BAUDRATE        9600

/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/
void rs485_usart_init(void);
void rs485_usart_send_data(uint8_t *buf,uint8_t len);

uint32_t rs485_fifo_read(uint8_t *rbuff,uint32_t len);
uint32_t rs485_fifo_get_data_length(void);
uint8_t rs485_get_idle_status(void);
void rs485_clear_idle_status(void);

#ifdef __cplusplus
extern "C"{
#endif

#endif /* _UART_H_ */

/***************************************** (END OF FILE) *********************************************/
