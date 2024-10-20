/*
*******************************************************************************************************
*
* File Name : spi.c
* Version   : V1.0
* Author    : mzy2364
* brief     : spi driver
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "spi.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/

/**
  * @brief spi0 init
  * @param None
  * @retval None
  */
void spi0_init(void)
{
    spi_parameter_struct spi_init_struct;
    
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_SPI0);
    
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_7);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

    /* SPI0 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_16BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_64;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI0, &spi_init_struct);
    
    spi_enable(SPI0);
}

/**
  * @brief spi2 init
  * @param None
  * @retval None
  */
void spi2_init(void)
{
    spi_parameter_struct spi_init_struct;
    
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_SPI2);
    
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
    
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

    /* SPI0 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_4;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI2, &spi_init_struct);
    
    spi_enable(SPI2);
}

/**
  * @brief spi2 transmit and receive data
  * @param None
  * @retval None
  */
uint8_t spi2_transmit_receive_data(uint8_t data)
{
    while(RESET == spi_i2s_flag_get(SPI2, SPI_FLAG_TBE)) {
    }
    spi_i2s_data_transmit(SPI2, data);
    
    while(RESET == spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE)) {
    }
    return spi_i2s_data_receive(SPI2);
}

/**
  * @brief spi2 transmit and receive data
  * @param None
  * @retval None
  */
uint16_t spi0_transmit_receive_data(uint16_t data)
{
    while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE)) {
    }
    spi_i2s_data_transmit(SPI0, data);
    
    while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE)) {
    }
    return spi_i2s_data_receive(SPI0);
}

/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
/**
  * @brief 
  * @param None
  * @retval None
  */


/***************************************** (END OF FILE) *********************************************/
