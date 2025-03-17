/*
*******************************************************************************************************
*
* File Name : encoder.h
* Version   : V1.0
* Author    : mzy2364
* brief     : encoder header file
* 
*******************************************************************************************************
*/
#ifndef _ENCODER_H_
#define _ENCODER_H_

#ifdef __cplusplus
extern "C"{
#endif

/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "gd32f30x.h"


/* DEFINES ------------------------------------------------------------------------------------------*/
#define ENCODERA_GPIO_PORT                 GPIOB
#define ENCODERA_GPIO_PIN                  GPIO_PIN_6
#define ENCODERA_GPIO_CLK                  RCU_GPIOB

#define ENCODERB_GPIO_PORT                 GPIOB
#define ENCODERB_GPIO_PIN                  GPIO_PIN_7
#define ENCODERB_GPIO_CLK                  RCU_GPIOB

typedef enum{
    ENCODER_NONE = 0,
    ENCODER_INC,
    ENCODER_DEC,
}encoder_code_t;

/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/
void encoder_init(void);
encoder_code_t encoder_get(void);

#ifdef __cplusplus
extern "C"{
#endif

#endif /* _ENCODER_H_ */

/***************************************** (END OF FILE) *********************************************/
