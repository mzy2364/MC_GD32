/*
*******************************************************************************************************
*
* File Name : smcpos.h
* Version   : V1.0
* Author    : mzy2364
* brief     : PMSM SMO control file
* 
*******************************************************************************************************
*/

#ifndef _SMCPOS_H
#define _SMCPOS_H

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

/*==================================================================================================
*                              SOURCE FILE VERSION INFORMATION
==================================================================================================*/


/*==================================================================================================
*                                          CONSTANTS
==================================================================================================*/


/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/
#ifndef ABS
#define ABS(x)                    (((x)<0)? (-(x)):(x))
#endif

/*==================================================================================================
*                                             ENUMS
==================================================================================================*/
/**
* @brief          
* @details        
*/


/*==================================================================================================
*                                STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/
typedef struct
{
    /* Parameter: Motor dependent control gain */
    float gsmopos;
    /* Parameter: Motor dependent plant matrix */
    float fsmopos;
    /* Input: Stationary alfa-axis stator voltage */
    float valpha;
    /* Variable: Stationary alfa-axis back EMF */
    float ealpha;
    /* Variable: Filtered EMF for Angle calculation */
    float ealpha_final;
    /* Output: Stationary alfa-axis sliding control */
    float zalpha;
    /* Variable: Estimated stationary alfa-axis stator current */
    float est_ialpha;
    /* Input: Stationary alfa-axis stator current */
    float ialpha;
    /* Variable: Stationary alfa-axis current error */
    float ialpha_error;
    /* Input: Stationary beta-axis stator voltage */
    float vbeta;
    /* Variable: Stationary beta-axis back EMF */
    float ebeta;
    /* Variable: Filtered EMF for Angle calculation */
    float ebeta_final;
    /* Output: Stationary beta-axis sliding control */
    float zbeta;
    /* Variable: Estimated stationary beta-axis stator current */
    float est_ibeta;
    /* Input: Stationary beta-axis stator current */
    float ibeta;
    /* Variable: Stationary beta-axis current error */
    float ibeta_error;
    /* Parameter: Sliding control gain */
    float kslide;
    /* Parameter: Maximum current error for linear SMC */
    float max_smc_error;
    /* Parameter: Sliding control filter gain */
    float kslf;
    /* Parameter: BEMF Filter for angle calculation */
    float kslf_final;
    /* Parameter: Filter Coef for Omega filtered calc */
    float filt_omega_coef;
    /* Output: Offset used to compensate rotor angle */
    float theta_offset;
    /* Output: Compensated rotor angle */
    float theta;
    /* Output: Rotor speed */
    float omega;
    /* Output: Filtered Rotor speed for speed PI */
    float omega_fltred;
    /* This value is used to transition from open loop to closed loop */
    float theta_error;
    /* pll error */
    float pll_err;
    /* pll error sum */
    float pll_err_sum;
    /* Parameter : pll kp */
    float pll_kp;
    /* Parameter : pll ki */
    float pll_ki;
}smc_t;


/*==================================================================================================
*                                GLOBAL VARIABLE DECLARATIONS
==================================================================================================*/


/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/
void smc_init_parameters(smc_t *psmc);
void smc_position_estimation (smc_t *psmc);

#ifdef __cplusplus
}
#endif

#endif /* _SMCPOS_H */

/************************************************EOF************************************************/
