/**
*   @file           pmsm.h
*   @implements     pmsm.h_Artifact
*   @version        v1.0.0
*   @date           2023-12-03
*   @author         mzy2364
*
*   @brief          motor control core function
*   @details        API implementation for motor control.
*
*   @addtogroup     BSP
*   @log
*       data        author            notes
*   2023-12-03      mzy2364           Create
*/

#ifndef _PMSM_H_
#define _PMSM_H_

#ifdef __cplusplus
extern "C"{
#endif

/*==================================================================================================
*                                        INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
==================================================================================================*/
#include <stdint.h>
#include "foc.h"
#include "smcpos.h"
#include "sto_pll_speed_pos_fdbk.h"
#include "userparms.h"

/*==================================================================================================
*                              SOURCE FILE VERSION INFORMATION
==================================================================================================*/


/*==================================================================================================
*                                          CONSTANTS
==================================================================================================*/


/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/

/*==================================================================================================
*                                             ENUMS
==================================================================================================*/



/*==================================================================================================
*                                STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/
typedef struct{
	uint8_t openloop;
	uint8_t run_motor;
	uint8_t change_mode;
	
	/* Rotor positioning time */
	uint16_t startup_lock;
	/* Start up ramp in open loop */
	uint32_t startup_ramp;
	
	uint16_t lock_time;
	int32_t end_speed;
	
	int16_t iq_ref;
	int16_t id_ref;
	int16_t vel_ref;
    
    int32_t vd_squared;
	
}pmsm_mc_t;


/*==================================================================================================
*                                GLOBAL VARIABLE DECLARATIONS
==================================================================================================*/
extern pmsm_foc_t pmsm_foc_param;
extern pmsm_mc_t pmsm_mc_param;
extern SMC smc1;
extern STO_PLL_Handle_t STO_PLL_M1;

/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/
void pmsm_foc_init(void);
void pmsm_foc_run(void);

#ifdef __cplusplus
}
#endif

#endif /* _PMSM_H_ */

/************************************************EOF************************************************/
