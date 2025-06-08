/*
 * ee_parameter.c
 *
 *  Created on: 2025-3-17
 *      Author: mzy2364
 */

/*******************************************************************************
* include files
*******************************************************************************/
#include <string.h>
#include "ee_parameter.h"
#include "userparms.h"

/*******************************************************************************
* Defines
*******************************************************************************/

/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Global Constant definition
*******************************************************************************/

/*******************************************************************************
* Local Constant definition
*******************************************************************************/

/*******************************************************************************
* Global Variables definition
*******************************************************************************/
uint16_t motor_r = (MOTOR_PHASE_RESISTANCE * 1000);     /* unit mR */
uint16_t motor_l = (MOTOR_PHASE_INDUCTANCE * 1000000);  /* unit uH */
uint16_t motor_bemf = (MOTOR_BACK_EMF_Vpeak * 1000);    /* unit mV/KRPM */
uint8_t motor_pole = MOTOR_NOPOLESPAIRS;
uint16_t motor_normal_spd = NOMINAL_SPEED_RPM;          /* unit RPM */
uint8_t openloop_current = OPEN_LOOP_CURRENT * 10;      /* unit 0.1A */
uint8_t rotor_lock_time = LOCK_TIME_SEC * 10;           /* unit 0.1s */
uint16_t openloop_ramp_time = OPENLOOPTIMEINSEC * 10;   /* unit 0.1s */
uint16_t openloop_hold_time = OPENLOOP_HOLD_TIME_SEC *10;   /* unit 0.1s */
uint16_t open_loop_speed = OPEN_LOOP_END_SPEED_RPM;     /* unit RPM */
/*******************************************************************************
* Local Variables definition
*******************************************************************************/
static uint8_t eeprom_data[EEPROM_PARAMETER_LEN] = {0};

/*******************************************************************************
* Local Functions prototypes
*******************************************************************************/


/*******************************************************************************
*  Global Functions Body
*******************************************************************************/
void eeprom_load_parameter(void)
{
    eeprom_read(EEPROM_PARAMETER_ADDR, eeprom_data, EEPROM_PARAMETER_LEN);
    
    memcpy(&motor_r,&eeprom_data[MOTOR_R_ADDR],MOTOR_R_LEN);
    memcpy(&motor_l,&eeprom_data[MOTOR_L_ADDR],MOTOR_L_LEN);
    memcpy(&motor_bemf,&eeprom_data[MOTOR_BF_ADDR],MOTOR_BF_LEN);
    memcpy(&motor_pole,&eeprom_data[MOTOR_PL_ADDR],MOTOR_PL_LEN);
    memcpy(&motor_normal_spd,&eeprom_data[MOTOR_NML_SPD_ADDR],MOTOR_NML_SPD_LEN);
    memcpy(&openloop_current,&eeprom_data[OL_AMP_ADDR],OL_AMP_LEN);
    memcpy(&rotor_lock_time,&eeprom_data[ROTOR_LOCK_TIME_ADDR],ROTOR_LOCK_TIME_LEN);
    memcpy(&openloop_ramp_time,&eeprom_data[OL_RAMP_TIME_ADDR],OL_RAMP_TIME_LEN);
    memcpy(&openloop_hold_time,&eeprom_data[OL_HOLD_TIME_ADDR],OL_HOLD_TIME_LEN);
    memcpy(&open_loop_speed,&eeprom_data[OL_SPEED_ADDR],OL_SPEED_LEN);
}    


void eeprom_save_parameter(void)
{
    memset(eeprom_data,0,sizeof(eeprom_data));
    
    memcpy(&eeprom_data[MOTOR_R_ADDR],&motor_r,MOTOR_R_LEN);
    memcpy(&eeprom_data[MOTOR_L_ADDR],&motor_l,MOTOR_L_LEN);
    memcpy(&eeprom_data[MOTOR_BF_ADDR],&motor_bemf,MOTOR_BF_LEN);
    memcpy(&eeprom_data[MOTOR_PL_ADDR],&motor_pole,MOTOR_PL_LEN);
    memcpy(&eeprom_data[MOTOR_NML_SPD_ADDR],&motor_normal_spd,MOTOR_NML_SPD_LEN);
    memcpy(&eeprom_data[OL_AMP_ADDR],&openloop_current,OL_AMP_LEN);
    memcpy(&eeprom_data[ROTOR_LOCK_TIME_ADDR],&rotor_lock_time,ROTOR_LOCK_TIME_LEN);
    memcpy(&eeprom_data[OL_RAMP_TIME_ADDR],&openloop_ramp_time,OL_RAMP_TIME_LEN);
    memcpy(&eeprom_data[OL_HOLD_TIME_ADDR],&openloop_hold_time,OL_HOLD_TIME_LEN);
    memcpy(&eeprom_data[OL_SPEED_ADDR],&open_loop_speed,OL_SPEED_LEN);
    
    eeprom_write(EEPROM_PARAMETER_ADDR, eeprom_data, EEPROM_PARAMETER_LEN);
}

/*******************************************************************************
*  Local Functions Body
*******************************************************************************/


/***************************************** (END OF FILE) *********************************************/
