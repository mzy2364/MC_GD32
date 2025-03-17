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
#define ENCODERA_GPIO_PORT                  GPIOB
#define ENCODERA_GPIO_PIN                   GPIO_PIN_6
#define ENCODERA_GPIO_CLK                   RCU_GPIOB

#define ENCODERB_GPIO_PORT                  GPIOB
#define ENCODERB_GPIO_PIN                   GPIO_PIN_7
#define ENCODERB_GPIO_CLK                   RCU_GPIOB

#define KEY_GPIO_PORT                       GPIOC
#define KEY_GPIO_PIN                        GPIO_PIN_0
#define KEY_GPIO_CLK                        RCU_GPIOC

/* �������� */
#define KEY_COUNT 1
/* �����˲�ʱ�� ��λ10ms */
#define KEY_FILTER_TIME   5
/* ��������ʱ�� ��λ10ms */
#define KEY_LONG_TIME     100
/* ���尴��FIFO��������С */
#define KEY_FIFO_SIZE	2

typedef enum{
    ENCODER_NONE = 0,
    ENCODER_INC,
    ENCODER_DEC,
}encoder_code_t;

/* �����Ƿ��� */
typedef enum
{
	KEY_ISDOWN = 0,
	KEY_NOTDOWN,
}KEY_DOWN;


/* �������� */
typedef enum
{
	KEY_NONE = 0,
	
	KEY_0_PRESS,
	KEY_0_RELEASE,
	KEY_0_LONG_PRESS,
	
}KEY_ENUM;


/* �����ṹ�� */
typedef struct
{
	uint8_t (*IsKeyDownFunc)(void);		//�����Ƿ��µĺ���ָ��
	
	uint8_t Count;	//�˲���������
	uint16_t LongCount;	//����������
	uint16_t LongTime;	//�������µ�ʱ��
	uint8_t State;		//�����ĵ�ǰ״̬
	uint8_t RepeatSpeed;	//������������
	uint8_t RepeatCount;		//��������������	
	uint8_t LongPressFlag;
	
	
}KEY_T;


/* fifo�ṹ�� */
typedef struct
{
	uint8_t Buf[KEY_FIFO_SIZE];        //����������
	uint8_t Read;                      //��ָ��
	uint8_t Write;                     //дָ��
	
}KEY_FIFO;

/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/
void encoder_init(void);
encoder_code_t encoder_get(void);

void key_clear(void);
void key_scan(void);
uint8_t key_get(void);

#ifdef __cplusplus
extern "C"{
#endif

#endif /* _ENCODER_H_ */

/***************************************** (END OF FILE) *********************************************/
