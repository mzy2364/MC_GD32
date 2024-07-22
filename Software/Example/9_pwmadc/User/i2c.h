/*
*******************************************************************************************************
*
* �ļ����� : i2c.h
* ��    �� : V1.0
* ��    �� : mzy2364
* ˵    �� : I2C�����ļ�
* 
*******************************************************************************************************
*/

#ifndef _I2C_H
#define _I2C_H

#ifdef __cplusplus
extern "C"{
#endif

/* ͷ�ļ� -----------------------------------------------------------*/
#include "gd32f30x.h"

/* �궨�� -----------------------------------------------------------*/



/* ѡ�����I2C����Ӳ��I2C */
#define SOFTWARE_I2C
#ifndef SOFTWARE_I2C
#define HARDWARE_I2C
#endif

/* SOFTWARE_I2C��IO���� */
#ifdef SOFTWARE_I2C
#define SDA_OUT(n)	(n?gpio_bit_write(GPIOB,GPIO_PIN_9,SET):gpio_bit_write(GPIOB,GPIO_PIN_9,RESET))
#define SDA_IN		gpio_input_bit_get(GPIOB,GPIO_PIN_9)
#define SCL_OUT(n)	(n?gpio_bit_write(GPIOB,GPIO_PIN_8,SET):gpio_bit_write(GPIOB,GPIO_PIN_8,RESET))
#endif

/* I2C�洢���ĵ�ַ���� */
#define I2C_MEMADD_8BIT            0x00000001U
#define I2C_MEMADD_16BIT           0x00000010U

/* typedef -----------------------------------------------------------*/
typedef enum{SOFT_I2C_OK=0,SOFT_I2C_ERR}SOFT_I2C_StatusTypeDef;




/* ȫ�ֱ��� ----------------------------------------------------------*/
#ifdef HARDWARE_I2C

#endif

/* �������� ----------------------------------------------------------*/

void i2c_init(void);

uint8_t i2c_master_send_data(uint16_t dev_addr,uint8_t *pbuffer,uint16_t len,uint32_t time_out);
uint8_t i2c_master_receive_data(uint16_t dev_addr,uint8_t *pbuffer,uint16_t len,uint32_t time_out);
uint8_t i2c_mem_write(uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_add_size, uint8_t *pbuffer, uint16_t len, uint32_t time_out);
uint8_t i2c_mem_read(uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_add_size, uint8_t *pbuffer, uint16_t len, uint32_t time_out);


void soft_i2c_start(void);
void soft_i2c_stop(void);
uint8_t soft_i2c_wait_ack(uint32_t time_out);
void soft_i2c_ack(void);
void soft_i2c_no_ack(void);
void soft_i2c_send_byte(uint8_t byte);
uint8_t soft_i2c_receive_byte(void);

#ifdef __cplusplus
extern "C"{
#endif

#endif /* _I2C_H */

/***************************************** (END OF FILE) *********************************************/
