/*
*******************************************************************************************************
*
* 文件名称 : i2c.c
* 版    本 : V1.0
* 作    者 : mzy2364
* 说    明 : I2C驱动文件
* 
*******************************************************************************************************
*/



/* 头文件 -----------------------------------------------------------*/
#include "i2c.h"
#include "delay.h"

/* 宏定义 -----------------------------------------------------------*/
#ifdef HARDWARE_I2C
I2C_HandleTypeDef hi2c;
#endif

/* 变量 -------------------------------------------------------------*/


/* 函数声明 ---------------------------------------------------------*/



/**
  * @brief I2C初始化函数
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
  * @brief 启用I2C发送数据
  * @param dev_addr-I2C从设备的地址
  * @param pbuffer-待发送的数据
  * @param len-数据长度
  * @param time_out-超时时间
  * @retval	0-发送成功 other_value-发送失败
  */
uint8_t i2c_master_send_data(uint16_t dev_addr,uint8_t *pbuffer,uint16_t len,uint32_t time_out)
{
#ifdef HARDWARE_I2C
	
#else
	dev_addr &= ~(1<<0);	//最低位清0，IIC写
	
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
  * @brief 启用I2C接收数据
  * @param dev_addr-I2C从设备的地址
  * @param pbuffer-接收数据缓冲区
  * @param len-数据长度
  * @param time_out-超时时间
  * @retval	0-接收成功 other_value-接收失败
  */
uint8_t i2c_master_receive_data(uint16_t dev_addr,uint8_t *pbuffer,uint16_t len,uint32_t time_out)
{
#ifdef HARDWARE_I2C

#else
	dev_addr |= (1<<0);	//最低位置1，IIC读

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
  * @brief 启用I2C向存储器写入一定长度的数据
  * @param dev_addr-I2C从设备的地址
  * @param mem_addr-存储器内部地址
  * @param mem_add_size-存储器地址长度(8位或者16位)
  * @param pbuffer-待写入的数据
  * @param len-数据长度
  * @param time_out-超时时间
  * @retval	0-写入成功 other_value-写入失败
  */
uint8_t i2c_mem_write(uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_add_size, uint8_t *pbuffer, uint16_t len, uint32_t time_out)
{
#ifdef HARDWARE_I2C

#else
	
	//dev_addr <<= 1;		//7位IIC地址
	dev_addr &= ~(1<<0);	//最低位清0，IIC写
	
	soft_i2c_start();
	
	soft_i2c_send_byte(dev_addr);						//发送IIC设备地址
	if(soft_i2c_wait_ack(time_out) == SOFT_I2C_ERR)
		return SOFT_I2C_ERR;
	
	//发送IIC设备的MEM地址或者寄存器地址
	if(mem_add_size == I2C_MEMADD_16BIT)
	{
		soft_i2c_send_byte((mem_addr>>8)&0X00FF);
		if(soft_i2c_wait_ack(time_out) == SOFT_I2C_ERR)
			return SOFT_I2C_ERR;
	}
	soft_i2c_send_byte(mem_addr & 0X00FF);						//发送IIC设备的MEM地址或者寄存器地址
	if(soft_i2c_wait_ack(time_out) == SOFT_I2C_ERR)
		return SOFT_I2C_ERR;
	
	
	//发送数据
	while(len)
	{
		soft_i2c_send_byte(*pbuffer);						//发送IIC设备的MEM地址或者寄存器地址
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
  * @brief 启用I2C从储器读取一定长度的数据
  * @param dev_addr-I2C从设备的地址
  * @param mem_addr-存储器内部地址
  * @param mem_add_size-存储器地址长度(8位或者16位)
  * @param pbuffer-读取数据缓冲区
  * @param len-数据长度
  * @param time_out-超时时间
  * @retval	0-读取成功 other_value-读取失败
  */
uint8_t i2c_mem_read(uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_add_size, uint8_t *pbuffer, uint16_t len, uint32_t time_out)
{
#ifdef HARDWARE_I2C

#else
	
	//dev_addr <<= 1;		//7位IIC地址
	dev_addr &= ~(1<<0);	//最低位清0，IIC写
	//dev_addr |= (1<<0);	//最低位置1，IIC读
	
	soft_i2c_start();
	
	soft_i2c_send_byte(dev_addr);						//发送IIC设备地址
	if(soft_i2c_wait_ack(time_out) == SOFT_I2C_ERR)
		return SOFT_I2C_ERR;
	
	//发送IIC设备的MEM地址或者寄存器地址
	if(mem_add_size == I2C_MEMADD_16BIT)
	{
		soft_i2c_send_byte((mem_addr>>8)&0X00FF);
		if(soft_i2c_wait_ack(time_out) == SOFT_I2C_ERR)
			return SOFT_I2C_ERR;
	}
	soft_i2c_send_byte(mem_addr & 0X00FF);						//发送IIC设备的MEM地址或者寄存器地址
	if(soft_i2c_wait_ack(time_out) == SOFT_I2C_ERR)
		return SOFT_I2C_ERR;
	
	
	soft_i2c_start();
	
	dev_addr |= (1<<0);	//最低位置1，IIC读
	
	soft_i2c_send_byte(dev_addr);					//发送IIC设备的MEM地址或者寄存器地址
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
  * @brief I2C开始信号
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
  * @brief I2C停止信号
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
  * @brief I2C等待应答
  * @param time_out-等待的时间,单位5us
  * @retval	None
  */
uint8_t soft_i2c_wait_ack(uint32_t time_out)
{
	SDA_OUT(1);
	delay_us(5);
	SCL_OUT(1);
	delay_us(1);
	
	while(SDA_IN)	//如果读到SDA线为1，则等待。应答信号应是0
	{
		if(--time_out == 0)
		{
			soft_i2c_stop();	//超时未应答，停止总线
			return SOFT_I2C_ERR;
		}
		delay_us(5);
	}
	
	SCL_OUT(0);
	return SOFT_I2C_OK;
}


/**
  * @brief I2C发送应答
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
  * @brief I2C发送非应答
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
  * @brief I2C发送一字节数据
  * @param byte-待发送的数据
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
  * @brief I2C接收一字节数据
  * @param None
  * @retval	接收到的数据
  */
uint8_t soft_i2c_receive_byte(void)
{
	unsigned char i = 0 , byte = 0;
	
	SDA_OUT(1);		//开漏状态下拉高SDA线以便读取输入数据
	
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
