/*
*******************************************************************************************************
*
* 文件名称 : eeprom.c
* 版    本 : V1.0
* 作    者 : mzy2364
* 说    明 : 24C02-EEPROM驱动文件
* 
*******************************************************************************************************
*/



/* 头文件 -----------------------------------------------------------*/
#include "eeprom.h"
#include "systick.h"


/* 宏定义 -----------------------------------------------------------*/


/* 变量 -------------------------------------------------------------*/


/* 函数声明 ---------------------------------------------------------*/
static void eeprom_page_write(uint8_t mem_addr,uint8_t *pbuffer, uint8_t num_byte_to_write);


/**
  * @brief 向EEPROM写入指定长度数据
  * @param mem_addr-存储器内部地址
  * @param pbuffer-待写入的数据
  * @param num_byte_to_write-数据长度
  * @retval	None
  */
void eeprom_write(uint8_t mem_addr,uint8_t *pbuffer, uint16_t num_byte_to_write)
{
	uint8_t num_of_page = 0, num_of_single = 0, addr = 0, count = 0;

	addr = mem_addr % EEPROM_PAGESIZE;
	count = EEPROM_PAGESIZE - addr;
	num_of_page =  num_byte_to_write / EEPROM_PAGESIZE;
	num_of_single = num_byte_to_write % EEPROM_PAGESIZE;

	/* 写入地址和页对齐  */
	if(addr == 0) 
	{
		/* 写入数据长度小于页长度，直接按长度写 */
		if(num_of_page == 0) 
		{
			eeprom_page_write(mem_addr, pbuffer, num_of_single);
		}
		/* 写入数据长度大于页长度，先写满一页的，后写剩余的 */
		else  
		{
			while(num_of_page--)
			{
				eeprom_page_write(mem_addr, pbuffer, EEPROM_PAGESIZE); 
				mem_addr +=  EEPROM_PAGESIZE;
				pbuffer += EEPROM_PAGESIZE;
			}

			if(num_of_single!=0)
			{
				eeprom_page_write(mem_addr, pbuffer, num_of_single);
			}
		}
	}
	/* 写入地址和页不对齐  */
	else 
	{
		/* If NumByteToWrite < I2C_PageSize */
		if(num_of_page== 0) 
		{
			eeprom_page_write(mem_addr, pbuffer, num_of_single);
		}
		/* If NumByteToWrite > I2C_PageSize */
		else
		{
			num_byte_to_write -= count;
			num_of_page =  num_byte_to_write / EEPROM_PAGESIZE;
			num_of_single = num_byte_to_write % EEPROM_PAGESIZE;	

			if(count != 0)
			{  
				eeprom_page_write(mem_addr, pbuffer, count);
				mem_addr += count;
				pbuffer += count;
			} 

			while(num_of_page--)
			{
				eeprom_page_write(mem_addr, pbuffer, EEPROM_PAGESIZE);
				mem_addr +=  EEPROM_PAGESIZE;
				pbuffer += EEPROM_PAGESIZE;  
			}
			if(num_of_single != 0)
			{
				eeprom_page_write(mem_addr, pbuffer, num_of_single); 
			}
		}
	}  
}





/**
  * @brief 从EEPROM读取指定长度数据
  * @param mem_addr-存储器内部地址
  * @param pbuffer-读取数据缓冲区
  * @param num_byte_to_read-数据长度
  * @retval	None
  */
void eeprom_read(uint8_t mem_addr,uint8_t *pbuffer, uint16_t num_byte_to_read)
{
	i2c_mem_read(EEPROM_ADDRESS,mem_addr,I2C_MEMADD_8BIT,pbuffer,num_byte_to_read,1000);
}





/**
  * @brief 向EEPROM写入一页数据
  * @param mem_addr-存储器内部地址
  * @param pbuffer-待写入的数据
  * @param num_byte_to_write-数据长度
  * @retval	None
  * @note 数据长度不能超过EEPROM一页的大小
  */
static void eeprom_page_write(uint8_t mem_addr,uint8_t *pbuffer, uint8_t num_byte_to_write)
{
	i2c_mem_write(EEPROM_ADDRESS,mem_addr,I2C_MEMADD_8BIT,pbuffer,num_byte_to_write,1000);
    systick_delay(5);
}




/***************************************** (END OF FILE) *********************************************/
