/*
*******************************************************************************************************
*
* File Name : main.c
* Version   : V1.0
* Author    : mzy2364
* brief     : main function file
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include <string.h>
#include "gd32f30x.h"
#include "gd32f30x_libopt.h"
#include "systick.h"
#include "can.h"

/* DEFINES ------------------------------------------------------------------------------------------*/

/* VARIABLES ----------------------------------------------------------------------------------------*/
can_receive_message_struct receive_message;
can_trasnmit_message_struct transmit_message;

/* FUNCTION -----------------------------------------------------------------------------------------*/


/**
  * @brief main function
  * @param None
  * @retval None
  */
int main(void)
{
    SCB->VTOR = FLASH_BASE | 0x4000;
    __enable_irq();
    
    systick_init();
    can_bus_init();
    
    /* initialize transmit message */
    transmit_message.tx_sfid = 0x200;
    transmit_message.tx_efid = 0x00;
    transmit_message.tx_ft = CAN_FT_DATA;
    transmit_message.tx_ff = CAN_FF_STANDARD;
    transmit_message.tx_dlen = 8;
    
    transmit_message.tx_data[0] = 0x00;
    transmit_message.tx_data[1] = 0xA1;
    transmit_message.tx_data[2] = 0xA2;
    transmit_message.tx_data[3] = 0xA3;
    transmit_message.tx_data[4] = 0xA4;
    transmit_message.tx_data[5] = 0xA5;
    transmit_message.tx_data[6] = 0xA6;
    transmit_message.tx_data[7] = 0xA7;
    
	while(1)
	{
        can_message_transmit(CAN0, &transmit_message);
        systick_delay(100);
	}
}

/*!
    \brief      this function handles CAN0 RX0 exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBD_LP_CAN0_RX0_IRQHandler(void)
{
    /* check the receive message */
    can_message_receive(CAN0, CAN_FIFO0, &receive_message);
    if((0x200 == receive_message.rx_sfid)&&(CAN_FF_STANDARD == receive_message.rx_ff) && (8 == receive_message.rx_dlen))
    {
        transmit_message.tx_data[0] = receive_message.rx_data[0];
        transmit_message.tx_data[1] = receive_message.rx_data[1];
        transmit_message.tx_data[2] = receive_message.rx_data[2];
        transmit_message.tx_data[3] = receive_message.rx_data[3];
        transmit_message.tx_data[4] = receive_message.rx_data[4];
        transmit_message.tx_data[5] = receive_message.rx_data[5];
        transmit_message.tx_data[6] = receive_message.rx_data[6];
        transmit_message.tx_data[7] = receive_message.rx_data[7];
    }
}


/***************************************** (END OF FILE) *********************************************/
