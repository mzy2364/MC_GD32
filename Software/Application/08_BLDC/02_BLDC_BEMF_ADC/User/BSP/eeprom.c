/*
*******************************************************************************************************
*
* �ļ����� : eeprom.c
* ��    �� : V1.0
* ��    �� : mzy2364
* ˵    �� : 24C02-EEPROM�����ļ�
* 
*******************************************************************************************************
*/



/* ͷ�ļ� -----------------------------------------------------------*/
#include "eeprom.h"
#include "systick.h"


/* �궨�� -----------------------------------------------------------*/


/* ���� -------------------------------------------------------------*/


/* �������� ---------------------------------------------------------*/
static void eeprom_page_write(uint8_t mem_addr,uint8_t *pbuffer, uint8_t num_byte_to_write);


/**
  * @brief ��EEPROMд��ָ����������
  * @param mem_addr-�洢���ڲ���ַ
  * @param pbuffer-��д�������
  * @param num_byte_to_write-���ݳ���
  * @retval	None
  */
void eeprom_write(uint8_t mem_addr,uint8_t *pbuffer, uint16_t num_byte_to_write)
{
	uint8_t num_of_page = 0, num_of_single = 0, addr = 0, count = 0;

	addr = mem_addr % EEPROM_PAGESIZE;
	count = EEPROM_PAGESIZE - addr;
	num_of_page =  num_byte_to_write / EEPROM_PAGESIZE;
	num_of_single = num_byte_to_write % EEPROM_PAGESIZE;

	/* д���ַ��ҳ����  */
	if(addr == 0) 
	{
		/* д�����ݳ���С��ҳ���ȣ�ֱ�Ӱ�����д */
		if(num_of_page == 0) 
		{
			eeprom_page_write(mem_addr, pbuffer, num_of_single);
		}
		/* д�����ݳ��ȴ���ҳ���ȣ���д��һҳ�ģ���дʣ��� */
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
	/* д���ַ��ҳ������  */
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
  * @brief ��EEPROM��ȡָ����������
  * @param mem_addr-�洢���ڲ���ַ
  * @param pbuffer-��ȡ���ݻ�����
  * @param num_byte_to_read-���ݳ���
  * @retval	None
  */
void eeprom_read(uint8_t mem_addr,uint8_t *pbuffer, uint16_t num_byte_to_read)
{
	i2c_mem_read(EEPROM_ADDRESS,mem_addr,I2C_MEMADD_8BIT,pbuffer,num_byte_to_read,1000);
}





/**
  * @brief ��EEPROMд��һҳ����
  * @param mem_addr-�洢���ڲ���ַ
  * @param pbuffer-��д�������
  * @param num_byte_to_write-���ݳ���
  * @retval	None
  * @note ���ݳ��Ȳ��ܳ���EEPROMһҳ�Ĵ�С
  */
static void eeprom_page_write(uint8_t mem_addr,uint8_t *pbuffer, uint8_t num_byte_to_write)
{
	i2c_mem_write(EEPROM_ADDRESS,mem_addr,I2C_MEMADD_8BIT,pbuffer,num_byte_to_write,1000);
    systick_delay(5);
}




/***************************************** (END OF FILE) *********************************************/
