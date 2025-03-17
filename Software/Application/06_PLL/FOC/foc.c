/*
*******************************************************************************************************
*
* File Name : foc.c
* Version   : V1.0
* Author    : mzy2364
* brief     : foc library
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "foc.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/


/* FUNCTION -----------------------------------------------------------------------------------------*/

/**
  * @brief clark
  * @param None
  * @retval None
  * @note ialpha = ia
  *       ibeta = (ia+2ib)/sqrt(3)
  */
void clark(pmsm_foc_t *foc)
{
    foc->ialpha = foc->ia;
    foc->ibeta = (foc->ia * ONE_BY_SQRT3) + (foc->ib * 2 * ONE_BY_SQRT3);
}

/**
  * @brief park
  * @param None
  * @retval None
  * @note Id = ialpha*cos(theta) + ibeta*sin(theta)
  *       Iq = -ialpha*sin(theta) + ibeta*cos(theta)
  */
void park(pmsm_foc_t *foc)
{
    foc->id = (foc->ialpha * foc->cos) + (foc->ibeta * foc->sin);
    foc->iq = (foc->ibeta * foc->cos) - (foc->ialpha * foc->sin);
}

/**
  * @brief inv_park
  * @param None
  * @retval None
  * @note valpha = vd*cos(theta) - vq*sin(theta)
  *       vbeta  = vd*sin(theta) + vq*cos(theta)
  */
void inv_park(pmsm_foc_t *foc)
{
    foc->valpha = (foc->vd * foc->cos) - (foc->vq * foc->sin);
    foc->vbeta = (foc->vd * foc->sin) + (foc->vq * foc->cos);
}

/**
  * @brief sin cos calculate
  * @param None
  * @retval None
  * @note 
  */
void sin_cos(pmsm_foc_t *foc)
{
    foc->sin = arm_sin_f32(foc->angle);
    foc->cos = arm_cos_f32(foc->angle);
}

/**
  * @brief 
  * @param None
  * @retval None
  * @note for one shunt,Prevent duty cycles from being equal,tb must be increased by 1
  */
void svpwm(pmsm_foc_t *foc)
{
    float vr1 = 0, vr2 = 0, vr3 = 0;
    float t1w = 0,t2w = 0;
    float t1 = 0, t2 = 0, t3 = 0;

    vr1 = foc->vbeta;
    vr2 = (-foc->vbeta/2 + SQRT3_BY2 * foc->valpha);
    vr3 = (-foc->vbeta/2 - SQRT3_BY2 * foc->valpha);

    if(vr3 > 0)
    {
        if(vr2 > 0)
        {
            foc->sector = 5;

            t1w = foc->pwm_period * vr2;
            t2w = foc->pwm_period * vr3;

            t1 = (foc->pwm_period - t1w - t2w)/2;
            t2 = t1 + t1w;
            t3 = t2 + t2w;

            foc->pwma = t2;
            foc->pwmb = t1;
            foc->pwmc = t3;
        }
        else
        {
            if(vr1 > 0)
            {
                foc->sector = 3;

                t1w = foc->pwm_period * vr3;
                t2w = foc->pwm_period * vr1;

                t1 = (foc->pwm_period - t1w - t2w)/2;
                t2 = t1 + t1w;
                t3 = t2 + t2w;

                foc->pwma = t1;
                foc->pwmb = t3;
                foc->pwmc = t2;
            }
            else
            {
                foc->sector = 4;

                t1w = foc->pwm_period * -vr2;
                t2w = foc->pwm_period * -vr1;

                t1 = (foc->pwm_period - t1w - t2w)/2;
                t2 = t1 + t1w;
                t3 = t2 + t2w;

                foc->pwma = t1;
                foc->pwmb = t2;
                foc->pwmc = t3;
            }
        }
    }
    else
    {
        if(vr2 > 0)
        {
            if(vr1 > 0)
            {
                foc->sector = 1;

                t1w = foc->pwm_period * vr1;
                t2w = foc->pwm_period * vr2;

                t1 = (foc->pwm_period - t1w - t2w)/2;
                t2 = t1 + t1w;
                t3 = t2 + t2w;

                foc->pwma = t3;
                foc->pwmb = t2;
                foc->pwmc = t1;
            }
            else
            {
                foc->sector = 6;

                t1w = foc->pwm_period * -vr1;
                t2w = foc->pwm_period * -vr3;

                t1 = (foc->pwm_period - t1w - t2w)/2;
                t2 = t1 + t1w;
                t3 = t2 + t2w;

                foc->pwma = t3;
                foc->pwmb = t1;
                foc->pwmc = t2;
            }
        }
        else
        {
            foc->sector = 2;

            t1w = foc->pwm_period * -vr3;
            t2w = foc->pwm_period * -vr2;

            t1 = (foc->pwm_period - t1w - t2w)/2;
            t2 = t1 + t1w;
            t3 = t2 + t2w;

            foc->pwma = t2;
            foc->pwmb = t3;
            foc->pwmc = t1;
        }
    }
}

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
