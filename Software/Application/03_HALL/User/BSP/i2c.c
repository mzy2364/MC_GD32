/*
*******************************************************************************************************
*
* �ļ����� : i2c.c
* ��    �� : V1.0
* ��    �� : mzy2364
* ˵    �� : I2C�����ļ�
* 
*******************************************************************************************************
*/



/* ͷ�ļ� -----------------------------------------------------------*/
#include "i2c.h"
#include "delay.h"

/* �궨�� -----------------------------------------------------------*/
#ifdef HARDWARE_I2C
I2C_HandleTypeDef hi2c;
#endif

/* ���� -------------------------------------------------------------*/


/* �������� ---------------------------------------------------------*/



/**
  * @brief I2C��ʼ������
  * @param None
  * @retval	None
  */
void i2c_init(void)
{
#ifdef HARDWARE_I2C

#else
	
    /**i2c GPIO Configuration    
    PB8     ------> I2C_SCL
    PB9     ------> I2C_SDA 
    */
	
    rcu_periph_clock_enable(RCU_GPIOB);

    gpio_init(GPIOB,GPIO_MODE_OUT_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_8);
    gpio_init(GPIOB,GPIO_MODE_OUT_OD,GPIO_OSPEED_50MHZ,GPIO_PIN_9);
	
	SDA_OUT(1);
	SCL_OUT(1);

#endif	
}


/**
  * @brief ����I2C��������
  * @param dev_addr-I2C���豸�ĵ�ַ
  * @param pbuffer-�����͵�����
  * @param len-���ݳ���
  * @param time_out-��ʱʱ��
  * @retval	0-���ͳɹ� other_value-����ʧ��
  */
uint8_t i2c_master_send_data(uint16_t dev_addr,uint8_t *pbuffer,uint16_t len,uint32_t time_out)
{
#ifdef HARDWARE_I2C
	
#else
	dev_addr &= ~(1<<0);	//���λ��0��IICд
	
	soft_i2c_start();
	
	soft_i2c_send_byte(dev_addr);
	if(soft_i2c_wait_ack(time_out) == SOFT_I2C_ERR)
		return SOFT_I2C_ERR;
	
	while(len)
	{
		soft_i2c_send_byte(*pbuffer);
		if(soft_i2c_wait_ack(time_out) == SOFT_I2C_ERR)
			return SOFT_I2C_ERR;
		pbuffer ++ ;
		len -- ;
	}
	soft_i2c_stop();
	return SOFT_I2C_OK;
#endif
}


/**
  * @brief ����I2C��������
  * @param dev_addr-I2C���豸�ĵ�ַ
  * @param pbuffer-�������ݻ�����
  * @param len-���ݳ���
  * @param time_out-��ʱʱ��
  * @retval	0-���ճɹ� other_value-����ʧ��
  */
uint8_t i2c_master_receive_data(uint16_t dev_addr,uint8_t *pbuffer,uint16_t len,uint32_t time_out)
{
#ifdef HARDWARE_I2C

#else
	dev_addr |= (1<<0);	//���λ��1��IIC��

	soft_i2c_start();
	
	soft_i2c_send_byte(dev_addr);
	if(soft_i2c_wait_ack(time_out) == SOFT_I2C_ERR)
		return SOFT_I2C_ERR;

	while(len)
	{
		*pbuffer = soft_i2c_receive_byte();
		pbuffer ++ ;
		len -- ;
		
		if(len == 0)
			soft_i2c_no_ack();
		else
			soft_i2c_ack();
	}
	soft_i2c_stop();
	
	return SOFT_I2C_OK;
	
#endif
}



/**
  * @brief ����I2C��洢��д��һ�����ȵ�����
  * @param dev_addr-I2C���豸�ĵ�ַ
  * @param mem_addr-�洢���ڲ���ַ
  * @param mem_add_size-�洢����ַ����(8λ����16λ)
  * @param pbuffer-��д�������
  * @param len-���ݳ���
  * @param time_out-��ʱʱ��
  * @retval	0-д��ɹ� other_value-д��ʧ��
  */
uint8_t i2c_mem_write(uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_add_size, uint8_t *pbuffer, uint16_t len, uint32_t time_out)
{
#ifdef HARDWARE_I2C

#else
	
	//dev_addr <<= 1;		//7λIIC��ַ
	dev_addr &= ~(1<<0);	//���λ��0��IICд
	
	soft_i2c_start();
	
	soft_i2c_send_byte(dev_addr);						//����IIC�豸��ַ
	if(soft_i2c_wait_ack(time_out) == SOFT_I2C_ERR)
		return SOFT_I2C_ERR;
	
	//����IIC�豸��MEM��ַ���߼Ĵ�����ַ
	if(mem_add_size == I2C_MEMADD_16BIT)
	{
		soft_i2c_send_byte((mem_addr>>8)&0X00FF);
		if(soft_i2c_wait_ack(time_out) == SOFT_I2C_ERR)
			return SOFT_I2C_ERR;
	}
	soft_i2c_send_byte(mem_addr & 0X00FF);						//����IIC�豸��MEM��ַ���߼Ĵ�����ַ
	if(soft_i2c_wait_ack(time_out) == SOFT_I2C_ERR)
		return SOFT_I2C_ERR;
	
	
	//��������
	while(len)
	{
		soft_i2c_send_byte(*pbuffer);						//����IIC�豸��MEM��ַ���߼Ĵ�����ַ
		if(soft_i2c_wait_ack(time_out) == SOFT_I2C_ERR)
			return SOFT_I2C_ERR;
		
		pbuffer ++ ;
		len -- ;
		
		delay_us(10);

	}

	
	soft_i2c_stop();
	
	return SOFT_I2C_OK;
	
#endif
}




/**
  * @brief ����I2C�Ӵ�����ȡһ�����ȵ�����
  * @param dev_addr-I2C���豸�ĵ�ַ
  * @param mem_addr-�洢���ڲ���ַ
  * @param mem_add_size-�洢����ַ����(8λ����16λ)
  * @param pbuffer-��ȡ���ݻ�����
  * @param len-���ݳ���
  * @param time_out-��ʱʱ��
  * @retval	0-��ȡ�ɹ� other_value-��ȡʧ��
  */
uint8_t i2c_mem_read(uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_add_size, uint8_t *pbuffer, uint16_t len, uint32_t time_out)
{
#ifdef HARDWARE_I2C

#else
	
	//dev_addr <<= 1;		//7λIIC��ַ
	dev_addr &= ~(1<<0);	//���λ��0��IICд
	//dev_addr |= (1<<0);	//���λ��1��IIC��
	
	soft_i2c_start();
	
	soft_i2c_send_byte(dev_addr);						//����IIC�豸��ַ
	if(soft_i2c_wait_ack(time_out) == SOFT_I2C_ERR)
		return SOFT_I2C_ERR;
	
	//����IIC�豸��MEM��ַ���߼Ĵ�����ַ
	if(mem_add_size == I2C_MEMADD_16BIT)
	{
		soft_i2c_send_byte((mem_addr>>8)&0X00FF);
		if(soft_i2c_wait_ack(time_out) == SOFT_I2C_ERR)
			return SOFT_I2C_ERR;
	}
	soft_i2c_send_byte(mem_addr & 0X00FF);						//����IIC�豸��MEM��ַ���߼Ĵ�����ַ
	if(soft_i2c_wait_ack(time_out) == SOFT_I2C_ERR)
		return SOFT_I2C_ERR;
	
	
	soft_i2c_start();
	
	dev_addr |= (1<<0);	//���λ��1��IIC��
	
	soft_i2c_send_byte(dev_addr);					//����IIC�豸��MEM��ַ���߼Ĵ�����ַ
	if(soft_i2c_wait_ack(time_out) == SOFT_I2C_ERR)
		return SOFT_I2C_ERR;
	
	while(len)
	{
		
		*pbuffer = soft_i2c_receive_byte();
		pbuffer ++ ;
		len -- ;
		if(len == 0)
			soft_i2c_no_ack();
		else
			soft_i2c_ack();
		
	}
	
	soft_i2c_stop();
	
	return SOFT_I2C_OK;
	
#endif
}



#ifdef SOFTWARE_I2C

/**
  * @brief I2C��ʼ�ź�
  * @param None
  * @retval	None
  */
void soft_i2c_start(void)
{
	SDA_OUT(1);
	SCL_OUT(1);

	delay_us(5);
	
	SDA_OUT(0);
	delay_us(5);
	SCL_OUT(0);
}

/**
  * @brief I2Cֹͣ�ź�
  * @param None
  * @retval	None
  */
void soft_i2c_stop(void)
{
	SDA_OUT(0);
	SCL_OUT(0);
	delay_us(5);
	
	SCL_OUT(1);
	delay_us(5);
	SDA_OUT(1);
	delay_us(5);
}

/**
  * @brief I2C�ȴ�Ӧ��
  * @param time_out-�ȴ���ʱ��,��λ5us
  * @retval	None
  */
uint8_t soft_i2c_wait_ack(uint32_t time_out)
{
	SDA_OUT(1);
	delay_us(5);
	SCL_OUT(1);
	delay_us(1);
	
	while(SDA_IN)	//�������SDA��Ϊ1����ȴ���Ӧ���ź�Ӧ��0
	{
		if(--time_out == 0)
		{
			soft_i2c_stop();	//��ʱδӦ��ֹͣ����
			return SOFT_I2C_ERR;
		}
		delay_us(5);
	}
	
	SCL_OUT(0);
	return SOFT_I2C_OK;
}


/**
  * @brief I2C����Ӧ��
  * @param None
  * @retval	None
  */
void soft_i2c_ack(void)
{
	SCL_OUT(0);
	SDA_OUT(0);
	delay_us(5);
	SCL_OUT(1);
	delay_us(5);
	SCL_OUT(0);
}

/**
  * @brief I2C���ͷ�Ӧ��
  * @param None
  * @retval	None
  */
void soft_i2c_no_ack(void)
{
	SCL_OUT(0);
	SDA_OUT(1);
	delay_us(5);
	SCL_OUT(1);
	delay_us(5);
	SCL_OUT(0);
}

/**
  * @brief I2C����һ�ֽ�����
  * @param byte-�����͵�����
  * @retval	None
  */
void soft_i2c_send_byte(uint8_t byte)
{
	unsigned char i=0;
	
	SCL_OUT(0);
	
	for(i=0;i<8;i++)
	{
		SDA_OUT((byte & 0X80) >> 7);
		
		byte <<= 1;
		
		delay_us(5);
		SCL_OUT(1);
		delay_us(5);
		SCL_OUT(0);
	}
}

/**
  * @brief I2C����һ�ֽ�����
  * @param None
  * @retval	���յ�������
  */
uint8_t soft_i2c_receive_byte(void)
{
	unsigned char i = 0 , byte = 0;
	
	SDA_OUT(1);		//��©״̬������SDA���Ա��ȡ��������
	
	for(i=0;i<8;i++)
	{
		SCL_OUT(0);
		delay_us(5);
		SCL_OUT(1);
		
		byte <<= 1;
		if(SDA_IN)
			byte ++ ;
		delay_us(5);	
	}
	return byte;
}
#endif



/***************************************** (END OF FILE) *********************************************/
