/*
*******************************************************************************************************
*
* File Name : iap.h
* Version   : V1.0
* Author    : mzy2364
* brief     : iap header file
* 
*******************************************************************************************************
*/
#ifndef _IAP_H_
#define _IAP_H_

#ifdef __cplusplus
extern "C"{
#endif

/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "gd32f30x.h"


/* DEFINES ------------------------------------------------------------------------------------------*/
#define APPLICATION_ADDRESS     (uint32_t)0x08004000
#define APP_MAX_LENGTH          (uint32_t)0x00078000        /* flash-512k boot-32k 512K-32K=480K */
#define FMC_PAGE_SIZE           ((uint16_t)0x800U)

#define YMODEM_SOH_DATA_LEN     128
#define YMODEM_STX_DATA_LEN     1024

/* TYPEDEF ------------------------------------------------------------------------------------------*/
typedef enum{
    WAIT_CONNECT = 0,
    DATA_RECEIVE,
    RECEIVE_FINISH,
    RECEIVE_END,
    JUMP_TO_APP,
}iap_status_t;

typedef  void (*pFunction)(void);

/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/
void iap_init(void);
void iap_task(void);
uint8_t jump_to_app(void);

#ifdef __cplusplus
extern "C"{
#endif

#endif /* _IAP_H_ */

/***************************************** (END OF FILE) *********************************************/
