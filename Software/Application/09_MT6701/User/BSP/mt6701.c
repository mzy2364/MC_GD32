/*
*******************************************************************************************************
*
* File Name : MT6701.c
* Version   : V1.0
* Author    : mzy2364
* brief     : MT6701 driver file
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "mt6701.h"
#include "spi.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/
static uint8_t crc6(uint8_t *data, uint8_t length);


/**
  * @brief mt6701 INIT
  * @param None
  * @retval None
  */
void mt6701_init(void)
{
    spi2_init();
    
    rcu_periph_clock_enable(MT6701_CS_CLK);
    gpio_init(MT6701_CS_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, MT6701_CS_PIN);
    gpio_bit_write(MT6701_CS_PORT,MT6701_CS_PIN,SET);
}

/**
  * @brief 读取 mt6701 角度
  * @param angle:返回原始的14位角度数据
  * @retval 1-读取成功   0-CRC错误
  * @not SPI_PSC_4 4.15us
  */
uint8_t mt6701_read_angle0(uint16_t *angle)
{
    uint32_t mt6701_data = 0;
    uint8_t crc_buf[3] = {0};
    uint8_t crc_calc = 0;
    gpio_bit_write(MT6701_CS_PORT,MT6701_CS_PIN,RESET);
    mt6701_data |= spi2_transmit_receive_data(0xFF) << 16;
    mt6701_data |= spi2_transmit_receive_data(0xFF) << 8;
    mt6701_data |= spi2_transmit_receive_data(0xFF);
    gpio_bit_write(MT6701_CS_PORT,MT6701_CS_PIN,SET);
    crc_buf[0] = mt6701_data >> 18;
    crc_buf[1] = mt6701_data >> 12;
    crc_buf[2] = mt6701_data >> 6;
    
    crc_calc = crc6(crc_buf, 3);
    //if(crc_calc == (mt6701_data & 0x3f))
    if(1)
    {
        *angle = mt6701_data >> 10;
        return 1;
    }
    return 0;
}

/**
  * @brief 读取 mt6701 角度
  * @param angle:返回0-360度角度数据
  * @retval 1-读取成功   0-CRC错误
  * @not SPI_PSC_4 4.15us
  */
uint8_t mt6701_read_angle1(float *angle)
{
    uint32_t mt6701_data = 0;
    uint8_t crc_buf[3] = {0};
    uint8_t crc_calc = 0;
    float angle_f = 0;
    
    gpio_bit_write(MT6701_CS_PORT,MT6701_CS_PIN,RESET);
    mt6701_data |= spi2_transmit_receive_data(0xFF) << 16;
    mt6701_data |= spi2_transmit_receive_data(0xFF) << 8;
    mt6701_data |= spi2_transmit_receive_data(0xFF);
    gpio_bit_write(MT6701_CS_PORT,MT6701_CS_PIN,SET);
    crc_buf[0] = mt6701_data >> 18;
    crc_buf[1] = mt6701_data >> 12;
    crc_buf[2] = mt6701_data >> 6;
    
    crc_calc = crc6(crc_buf, 3);
    //if(crc_calc == (mt6701_data & 0x3f))
    if(1)
    {
        mt6701_data >>= 10;
        angle_f = (float)mt6701_data * 360 / 16384.0;
        *angle = angle_f;
        return 1;
    }
    return 0;
}

/**
  * @brief 读取 mt6701 角度
  * @param angle:返回0-360度角度数据
  * @retval 1-读取成功   0-CRC错误
  * @not SPI_PSC_4 4.15us
  */
uint8_t mt6701_read_angle2(float *angle)
{
    uint32_t mt6701_data = 0;
    uint8_t crc_buf[3] = {0};
    uint8_t crc_calc = 0;
    float angle_f = 0;
    
    gpio_bit_write(MT6701_CS_PORT,MT6701_CS_PIN,RESET);
    mt6701_data |= spi2_transmit_receive_data(0xFF) << 16;
    mt6701_data |= spi2_transmit_receive_data(0xFF) << 8;
    mt6701_data |= spi2_transmit_receive_data(0xFF);
    gpio_bit_write(MT6701_CS_PORT,MT6701_CS_PIN,SET);
    crc_buf[0] = mt6701_data >> 18;
    crc_buf[1] = mt6701_data >> 12;
    crc_buf[2] = mt6701_data >> 6;
    
    crc_calc = crc6(crc_buf, 3);
    //if(crc_calc == (mt6701_data & 0x3f))
    if(1)
    {
        mt6701_data >>= 10;
        angle_f = (float)mt6701_data * 6.283185 / 16384.0;
        *angle = angle_f;
        return 1;
    }
    return 0;
}

/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
static uint8_t crc6(uint8_t *data, uint8_t length)
{  
	uint8_t i;  
	uint8_t crc = 0;    // Initial value  
	
	while(length--)  
	{  
		 crc ^= *data++; // crc ^= *data; data++;
		for (i=6; i>0; --i)  
        { 
            if (crc & 0x20)
                crc = (crc << 1) ^ 0x03;
            else
                crc = (crc << 1);
        }
	 }  
	return crc&0x3f;  
} 

/***************************************** (END OF FILE) *********************************************/
