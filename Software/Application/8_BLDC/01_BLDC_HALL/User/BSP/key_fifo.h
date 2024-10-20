/*
*******************************************************************************************************
*
* �ļ����� : key_fifo.h
* ��    �� : V1.0
* ��    �� : mzy2364
* ˵    �� : ��������ͷ�ļ�
* 
*******************************************************************************************************
*/

#ifndef _KEY_FIFO_H
#define _KEY_FIFO_H



/* ͷ�ļ� -----------------------------------------------------------*/
#include "gd32f30x.h"



/* �궨�� -----------------------------------------------------------*/

/* �����˿ڶ��� */
#define KEY0_GPIO_PORT       GPIOC
#define KEY0_GPIO_PIN        GPIO_PIN_0
#define KEY0_GPIO_CLK        RCU_GPIOC

/* �������� */
#define KEY_COUNT 1
/* �����˲�ʱ�� ��λ10ms */
#define KEY_FILTER_TIME   5
/* ��������ʱ�� ��λ10ms */
#define KEY_LONG_TIME     100
/* ���尴��FIFO��������С */
#define KEY_FIFO_SIZE	4



/* typedef -----------------------------------------------------------*/

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
	
	
}KEY_T;


/* fifo�ṹ�� */
typedef struct
{
	uint8_t Buf[KEY_FIFO_SIZE];        //����������
	uint8_t Read;                      //��ָ��
	uint8_t Write;                     //дָ��
	
}KEY_FIFO;




/* �������� ----------------------------------------------------------*/


void key_fifo_init(void);
void key_clear(void);
uint8_t key_is_init(void);
void key_scan(void);
uint8_t key_get(void);


void key_put(uint8_t keyCode);
static void key_gpio_config(void);
static void key_var_init(void);
static void key_detect(uint8_t i);
 
#endif

/***************************** (END OF FILE) *********************************/
