/*
*******************************************************************************************************
*
* File Name : iap.c
* Version   : V1.0
* Author    : mzy2364
* brief     : iap file
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include <string.h>
#include "iap.h"
#include "uart.h"
#include "systick.h"
#include "crc_check.h"

/* DEFINES ------------------------------------------------------------------------------------------*/
#define IAP_BUF_LEN     2048

#define YMODEM_CMD_SOH  0x01
#define YMODEM_CMD_STX  0x02
#define YMODEM_CMD_EOT  0x04
#define YMODEM_CMD_ACK  0x06
#define YMODEM_CMD_NAK  0x15
#define YMODEM_CMD_CA   0x18
#define YMODEM_CMD_C    0x43

#define CONNECT_DATA_CYCLE  1000
#define WAIT_CONNECT_TIMEOUT    15000

/* VARIABLES ----------------------------------------------------------------------------------------*/
static uint8_t iap_buf[IAP_BUF_LEN];
static uint16_t iap_data_len = 0;
static uint8_t ymodem_cmd = 0;
static uint32_t file_length = 0;
static uint8_t packet_number = 0;
static iap_status_t iap_status = WAIT_CONNECT;
static uint32_t iap_tick = 0;
static uint32_t jump_app_tick = 0;
static uint32_t program_buf[1024/4];
static uint32_t program_addr = 0;
pFunction JumpToApplication;

/* FUNCTION -----------------------------------------------------------------------------------------*/
static uint16_t update_crc16(uint16_t crc_in, uint8_t byte);
static uint16_t cal_crc16(const uint8_t* p_data, uint32_t size);
static void fmc_erase_pages(uint32_t addr,uint32_t length);
void fmc_program(uint32_t addr,uint32_t length,uint32_t *buf);

/**
  * @brief iap init
  * @param None
  * @retval None
  */
void iap_init(void)
{
    iap_tick = systick_get_current_tick();
    jump_app_tick = systick_get_current_tick();
}

/**
  * @brief iap loop task
  * @param None
  * @retval None
  */
void iap_task(void)
{
    uint16_t crc = 0;
    
    uart3_transmit_string("\r\nMotor Control GD32 Bootloader\r\n");
    uart3_transmit_string("Please send file with YMODEM\r\n");
    
    while(1)
    {
        iap_data_len = uart3_get_receive_data(iap_buf,IAP_BUF_LEN);
        
        switch(iap_status)
        {
            case WAIT_CONNECT:
                if(systick_time_diff(iap_tick) >= CONNECT_DATA_CYCLE)
                {
                    iap_tick = systick_get_current_tick();
                    uart3_transmit_byte(YMODEM_CMD_C);
                }
                if(systick_time_diff(jump_app_tick) >= WAIT_CONNECT_TIMEOUT)
                {
                    jump_to_app();
                    jump_app_tick = systick_get_current_tick();
                }
                if(iap_data_len == (YMODEM_SOH_DATA_LEN + 5))
                {
                    if((iap_buf[0] == YMODEM_CMD_SOH) && \
                       (iap_buf[1] == 0x00) && \
                       (iap_buf[2] == 0xFF))
                    {
                        packet_number = iap_buf[1];
                        crc = cal_crc16(&iap_buf[3],YMODEM_SOH_DATA_LEN);
                        if(crc == ((iap_buf[iap_data_len-2]<<8)|iap_buf[iap_data_len-1]))
                        {
                            uint16_t find_index = 3;
                            uint8_t file_name_end_flag = 0;
                            while(find_index < (iap_data_len - 5))
                            {
                                if(iap_buf[find_index] == 0x00)
                                {
                                    file_name_end_flag = 1;
                                    find_index++;
                                }
                                if(file_name_end_flag)
                                {
                                    if((iap_buf[find_index] >= 0x30) && (iap_buf[find_index] <= 0x39))
                                    {
                                        file_length *= 10;
                                        file_length += (iap_buf[find_index] - 0x30);
                                    }
                                    if(iap_buf[find_index] == 0x20)
                                    {
                                        break;
                                    }
                                }
                                find_index++;
                            }
                            if(file_length <= APP_MAX_LENGTH)
                            {
                                uart3_transmit_byte(YMODEM_CMD_ACK);
                                fmc_erase_pages(APPLICATION_ADDRESS,file_length);
                                packet_number++;
                                iap_status = DATA_RECEIVE;
                                program_addr = APPLICATION_ADDRESS;
                                uart3_transmit_byte(YMODEM_CMD_C);
                                iap_tick = systick_get_current_tick();
                                jump_app_tick = systick_get_current_tick();
                            }
                        }
                    }
                }
                break;
            case DATA_RECEIVE:
                if(iap_data_len)
                {
                    if(iap_buf[0] == YMODEM_CMD_SOH)
                    {
                        if(iap_data_len != (YMODEM_SOH_DATA_LEN + 5))
                        {
                            iap_status = WAIT_CONNECT;
                            break;
                        }
                        if(packet_number != iap_buf[1])
                        {
                            iap_status = WAIT_CONNECT;
                            break;
                        }
                        packet_number++;
                        crc = cal_crc16(&iap_buf[3],YMODEM_SOH_DATA_LEN);
                        if(crc != ((iap_buf[iap_data_len-2]<<8)|iap_buf[iap_data_len-1]))
                        {
                            iap_status = WAIT_CONNECT;
                            break;
                        }
                        if((program_addr + YMODEM_SOH_DATA_LEN - APPLICATION_ADDRESS) <= file_length)
                        {
                            memcpy(program_buf,&iap_buf[3],YMODEM_SOH_DATA_LEN);
                            fmc_program(program_addr,YMODEM_SOH_DATA_LEN,program_buf);
                            program_addr += YMODEM_SOH_DATA_LEN;
                        }
                        else
                        {
                            uint16_t program_len = file_length - (program_addr - APPLICATION_ADDRESS);
                            if(program_len)
                            {
                                memcpy(program_buf,&iap_buf[3],program_len);
                                fmc_program(program_addr,program_len,program_buf);
                                program_addr += program_len;
                            }
                        }
                        uart3_transmit_byte(YMODEM_CMD_ACK);
                        iap_tick = systick_get_current_tick();
                        jump_app_tick = systick_get_current_tick();
                    }
                    else if(iap_buf[0] == YMODEM_CMD_STX)
                    {
                        if(iap_data_len != (YMODEM_STX_DATA_LEN + 5))
                        {
                            iap_status = WAIT_CONNECT;
                            break;
                        }
                        if(packet_number != iap_buf[1])
                        {
                            iap_status = WAIT_CONNECT;
                            break;
                        }
                        packet_number++;
                        crc = cal_crc16(&iap_buf[3],YMODEM_STX_DATA_LEN);
                        if(crc != ((iap_buf[iap_data_len-2]<<8)|iap_buf[iap_data_len-1]))
                        {
                            iap_status = WAIT_CONNECT;
                            break;
                        }
                        if((program_addr + YMODEM_STX_DATA_LEN - APPLICATION_ADDRESS) <= file_length)
                        {
                            memcpy(program_buf,&iap_buf[3],YMODEM_STX_DATA_LEN);
                            fmc_program(program_addr,YMODEM_STX_DATA_LEN,program_buf);
                            program_addr += YMODEM_STX_DATA_LEN;
                        }
                        else
                        {
                            uint16_t program_len = file_length - (program_addr - APPLICATION_ADDRESS);
                            if(program_len)
                            {
                                memcpy(program_buf,&iap_buf[3],program_len);
                                fmc_program(program_addr,program_len,program_buf);
                                program_addr += program_len;
                            }
                        }
                        uart3_transmit_byte(YMODEM_CMD_ACK);
                        iap_tick = systick_get_current_tick();
                        jump_app_tick = systick_get_current_tick();
                    }
                    else if(iap_buf[0] == YMODEM_CMD_EOT)
                    {
                        uart3_transmit_byte(YMODEM_CMD_NAK);
                        iap_status = RECEIVE_FINISH;
                        iap_tick = systick_get_current_tick();
                        jump_app_tick = systick_get_current_tick();
                    }
                }
                break;
            case RECEIVE_FINISH:
                if(iap_data_len)
                {
                    if(iap_buf[0] == YMODEM_CMD_EOT)
                    {
                        uart3_transmit_byte(YMODEM_CMD_ACK);
                        systick_delay(100);
                        uart3_transmit_byte(YMODEM_CMD_C);
                        iap_status = RECEIVE_END;
                        iap_tick = systick_get_current_tick();
                        jump_app_tick = systick_get_current_tick();
                    }
                }
                break;
            case RECEIVE_END:
                if(iap_data_len == (YMODEM_SOH_DATA_LEN + 5))
                {
                    uart3_transmit_byte(YMODEM_CMD_ACK);
                    iap_status = JUMP_TO_APP;
                    systick_delay(1000);
                    uart3_transmit_string("\r\nProgramming Completed Successfully!\r\n");
                    iap_tick = systick_get_current_tick();
                }
                if(systick_time_diff(iap_tick) >= CONNECT_DATA_CYCLE)
                {
                    iap_tick = systick_get_current_tick();
                    iap_status = JUMP_TO_APP;
                }
                break;
            case JUMP_TO_APP:
                uart3_transmit_string("Jump to APP\r\n");
                jump_to_app();
                uart3_transmit_string("\r\nJump to APP Error!!!\r\n");
                iap_status = WAIT_CONNECT;
                break;
            default:
                break;
        }
    }
}

/**
  * @brief 
  * @param None
  * @retval None
  */
uint8_t jump_to_app(void)
{
    if (((*(__IO uint32_t*)APPLICATION_ADDRESS) & 0x2FFE0000  ) == 0x20000000)
    {
        systick_deinit();
        uart3_deinit();
        
        /* Jump to user application */
        JumpToApplication = (pFunction) *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);
        /* Initialize user application's Stack Pointer */
        __set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
        JumpToApplication();
        while(1);
    }
}

/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/

/**
  * @brief  Update CRC16 for input byte
  * @param  crc_in input value 
  * @param  input byte
  * @retval None
  */
static uint16_t update_crc16(uint16_t crc_in, uint8_t byte)
{
  uint32_t crc = crc_in;
  uint32_t in = byte | 0x100;

  do
  {
    crc <<= 1;
    in <<= 1;
    if(in & 0x100)
      ++crc;
    if(crc & 0x10000)
      crc ^= 0x1021;
  }
  
  while(!(in & 0x10000));

  return crc & 0xffffu;
}

/**
  * @brief  Cal CRC16 for YModem Packet
  * @param  data
  * @param  length
  * @retval None
  */
static uint16_t cal_crc16(const uint8_t* p_data, uint32_t size)
{
  uint32_t crc = 0;
  const uint8_t* dataEnd = p_data+size;

  while(p_data < dataEnd)
    crc = update_crc16(crc, *p_data++);
 
  crc = update_crc16(crc, 0);
  crc = update_crc16(crc, 0);

  return crc&0xffffu;
}

/*!
    \brief      erase fmc pages
    \param[in]  addr-erase addr
    \param[in]  length-erase length
    \param[out] none
    \retval     none
*/
static void fmc_erase_pages(uint32_t addr,uint32_t length)
{
    uint32_t erase_addr = addr;

    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

    /* erase the flash pages */
    for(erase_addr = addr; erase_addr < (addr+length); erase_addr += FMC_PAGE_SIZE)
    {
        fmc_page_erase(erase_addr);
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
    }

    /* lock the main FMC after the erase operation */
    fmc_lock();
}

/*!
    \brief      program fmc
    \param[in]  addr-program address
    \param[in]  length-program length for byte
    \param[in]  buf-program buf
    \param[out] none
    \retval     none
*/
void fmc_program(uint32_t addr,uint32_t length,uint32_t *buf)
{
    uint32_t program_addr = addr;
    uint32_t i = 0;
    
    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* program flash */
    while(program_addr < (addr+length))
    {
        fmc_word_program(program_addr, buf[i]);
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
        
        program_addr += 4;
        i++;
    }

    /* lock the main FMC after the program operation */
    fmc_lock();
}

/***************************************** (END OF FILE) *********************************************/
