/*
*******************************************************************************************************
*
* File Name : uart.h
* Version   : V1.0
* Author    : mzy2364
* brief     : uart header file
* 
*******************************************************************************************************
*/
#ifndef _UART_H_
#define _UART_H_

#ifdef __cplusplus
extern "C"{
#endif

/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "gd32f30x.h"


/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/
void uart3_init(void);
void uart3_deinit(void);
void uart3_transmit_data(uint8_t *buf,uint32_t buf_len);
uint32_t uart3_get_receive_data(uint8_t *buf,uint32_t buf_len);
void uart3_transmit_byte(uint8_t byte);
void uart3_transmit_string(const char *buf);

#ifdef __cplusplus
extern "C"{
#endif

#endif /* _UART_H_ */

/***************************************** (END OF FILE) *********************************************/
