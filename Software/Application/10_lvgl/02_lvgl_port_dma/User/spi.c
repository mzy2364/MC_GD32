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
static uint16_t spi2_send_array[1] = {0};

/* FUNCTION -----------------------------------------------------------------------------------------*/
static void spi_dma_config(void);

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
    
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3 | GPIO_PIN_5);

    /* SPI0 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = 0;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI2, &spi_init_struct);
    
    spi_enable(SPI2);
    
    spi_dma_enable(SPI2, SPI_DMA_TRANSMIT);
    
    spi_dma_config();
    
    nvic_irq_enable(DMA1_Channel1_IRQn, 2, 2);
    
    dma_interrupt_flag_clear(DMA1,DMA_CH1,DMA_INT_FLAG_FTF);
    dma_interrupt_enable(DMA1, DMA_CH1, DMA_INT_FTF);
    
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

/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
/*!
    \brief      configure the DMA peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void spi_dma_config(void)
{
    dma_parameter_struct dma_init_struct;
    
    rcu_periph_clock_enable(RCU_DMA1);
    
    /* SPI0 transmit dma config:DMA0-DMA_CH2  */
    dma_deinit(DMA1, DMA_CH1);
    dma_init_struct.periph_addr  = (uint32_t)&SPI_DATA(SPI2);
    dma_init_struct.memory_addr  = (uint32_t)spi2_send_array;
    dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_init_struct.priority     = DMA_PRIORITY_LOW;
    dma_init_struct.number       = 1;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init(DMA1, DMA_CH1, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(DMA1, DMA_CH1);
    dma_memory_to_memory_disable(DMA1, DMA_CH1);
}

/***************************************** (END OF FILE) *********************************************/
