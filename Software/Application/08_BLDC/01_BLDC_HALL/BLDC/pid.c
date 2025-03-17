/*
*******************************************************************************************************
*
* File Name : pid.c
* Version   : V1.0
* Author    : mzy2364
* brief     : pid library
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "pid.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/

/**
  * @brief init pi 
  * @param None
  * @retval None
  * @note 
  */
void init_pi(pid_control_t *p_parm)
{
    p_parm->sum = 0;
    p_parm->out = 0;
}

/**
  * @brief calculate pi 
  * @param None
  * @retval None
  * @note 
  */
void calc_pi(pid_control_t *p_parm)
{
    float Err;
    float U;
    float Exc;
    
    Err  = p_parm->ref - p_parm->meas;
    p_parm->error =  Err; 
    U  = p_parm->sum + p_parm->kp * Err;
   
    if( U > p_parm->out_max)
    {
        p_parm->out = p_parm->out_max;
    }    
    else if( U < p_parm->out_min)
    {
        p_parm->out = p_parm->out_min;
    }
    else        
    {
        p_parm->out = U;  
    }
     
    Exc = U - p_parm->out;
    p_parm->sum = p_parm->sum + p_parm->ki * Err - p_parm->kc * Exc;
}

/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/


/***************************************** (END OF FILE) *********************************************/
