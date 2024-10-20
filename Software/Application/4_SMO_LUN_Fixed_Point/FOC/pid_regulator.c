/**
  ******************************************************************************
  * @file    pid_regulator.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the PID regulator component of the Motor Control SDK:
  *
  *           * proportional, integral and derivative computation funcions
  *           * read and write gain functions
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pid_regulator.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup PIDRegulator PID Regulator
  * @brief PID regulator component of the Motor Control SDK
  *
  * The PID regulator component implements the following control function:
  *
  * @f[
  * u(t) = K_{p} e(t) + K_{i} \int_0^t e(\tau) \,d\tau + K_{d} \frac{de(t)}{dt}
  * @f]
  *
  * with the proportional, integral and derivative gains expressed as rational numbers, with a gain and a divisor parameter :
  *
  * @f[
  * K_{p} = \frac{K_{pg}}{K_{pd}}
  * @f]
  * @f[
  * K_{i} = \frac{K_{ig}}{K_{id}}
  * @f]
  * @f[
  * K_{d} = \frac{K_{dg}}{K_{dd}}
  * @f]
  *
  *  Each of the gain and divisor parameters, @f$K_{{p}g}@f$, @f$K_{{i}g}@f$, @f$K_{{d}g}@f$, @f$K_{{p}d}@f$,
  * @f$K_{id}@f$, @f$K_{dd}@f$, can be set independently. via the PID_SetKP(), PID_SetKPDivisorPOW2(), PID_SetKI(),
  * PID_SetKIDivisorPOW2(), PID_SetKD()
  *
  * @{
  */

/**
  * @brief  It initializes the handle
  * @param  pHandle: handler of the current instance of the PID component
  * @retval None
  */
__weak void PID_HandleInit(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hKpGain =  pHandle->hDefKpGain;
    pHandle->hKiGain =  pHandle->hDefKiGain;
    pHandle->hKdGain =  pHandle->hDefKdGain;
    pHandle->wIntegralTerm = 0;
    pHandle->wPrevProcessVarError = 0;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  It updates the Kp gain
  * @param  pHandle: handler of the current instance of the PID component
  * @param  hKpGain: new Kp gain
  * @retval None
  */
__weak void PID_SetKP(PID_Handle_t *pHandle, int16_t hKpGain)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hKpGain = hKpGain;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  It updates the Ki gain
  * @param  pHandle: handler of the current instance of the PID component
  * @param  hKiGain: new Ki gain
  * @retval None
  */
__weak void PID_SetKI(PID_Handle_t *pHandle, int16_t hKiGain)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hKiGain = hKiGain;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  It returns the Kp gain
  * @param  pHandle: handler of the current instance of the PID component
  * @retval Kp gain
  */
__weak int16_t PID_GetKP(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0 : pHandle->hKpGain);
#else
  return (pHandle->hKpGain);
#endif
}

/**
  * @brief  It returns the Ki gain
  * @param  pHandle: handler of the current instance of the PID component
  * @retval Ki gain
  */
__weak int16_t PID_GetKI(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0 : pHandle->hKiGain);
#else
  return (pHandle->hKiGain);
#endif
}

/**
  * @brief  It returns the Default Kp gain
  * @param  pHandle: handler of the current instance of the PID component
  * @retval default Kp gain
  */
__weak int16_t PID_GetDefaultKP(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0 : pHandle->hDefKpGain);
#else
  return (pHandle->hDefKpGain);
#endif
}

/**
  * @brief  It returns the Default Ki gain of the passed PI object
  * @param  pHandle: handler of the current instance of the PID component
  * @retval default Ki gain
  */
__weak int16_t PID_GetDefaultKI(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0 : pHandle->hDefKiGain);
#else
  return (pHandle->hDefKiGain);
#endif
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  It set a new value into the PI integral term
  * pHandle: handler of the current instance of the PID component
  * @param  wIntegralTermValue: new integral term value
  * @retval None
  */
__weak void PID_SetIntegralTerm(PID_Handle_t *pHandle, int32_t wIntegralTermValue)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->wIntegralTerm = wIntegralTermValue;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
  return;
}

/**
  * @brief  It returns the Kp gain divisor
  * @param  pHandle: handler of the current instance of the PID component
  * @retval Kp gain divisor
  */
__weak uint16_t PID_GetKPDivisor(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0U : pHandle->hKpDivisor);
#else
  return (pHandle->hKpDivisor);
#endif
}

/**
  * @brief  It returns the Kp gain divisor
  * @param  pHandle: handler of the current instance of the PID component
  * @retval hKpDivisorPOW2: Kp divisor expressed as power of 2
  */
__weak uint16_t PID_GetKPDivisorPOW2(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0U : pHandle->hKpDivisorPOW2);
#else
  return (pHandle->hKpDivisorPOW2);
#endif
}

/**
  * @brief  It updates the Kp divisor
  * @param  pHandle: handler of the current instance of the PID component
  * @param  hKpDivisorPOW2: new Kp divisor expressed as power of 2
  * @retval None
  */
__weak void PID_SetKPDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKpDivisorPOW2)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hKpDivisorPOW2 = hKpDivisorPOW2;
    pHandle->hKpDivisor = (((uint16_t)1) << hKpDivisorPOW2);
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  It returns the Ki gain divisor of the passed PI object
  * @param  pHandle: handler of the current instance of the PID component
  * @retval Ki gain divisor
  */
__weak uint16_t PID_GetKIDivisor(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0U : pHandle->hKiDivisor);
#else
  return (pHandle->hKiDivisor);
#endif
}

/**
  * @brief  It returns the Ki gain divisor of the passed PI object
  * @param  pHandle: handler of the current instance of the PID component
  * @retval hKiDivisorPOW2: Ki divisor expressed as power of 2
  */
__weak uint16_t PID_GetKIDivisorPOW2(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0U : pHandle->hKiDivisorPOW2);
#else
  return (pHandle->hKiDivisorPOW2);
#endif
}

/**
  * @brief  It updates the Ki divisor
  * @param  pHandle: handler of the current instance of the PID component
  * @param  hKiDivisorPOW2: new Ki divisor expressed as power of 2
  * @retval None
  */
__weak void PID_SetKIDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKiDivisorPOW2)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    uint32_t wKiDiv = (((uint32_t)1) << hKiDivisorPOW2);
    pHandle->hKiDivisorPOW2 = hKiDivisorPOW2;
    pHandle->hKiDivisor = (uint16_t)wKiDiv;
    PID_SetUpperIntegralTermLimit(pHandle, (int32_t)INT16_MAX * (int32_t)wKiDiv);
    PID_SetLowerIntegralTermLimit(pHandle, (int32_t)(-INT16_MAX) * (int32_t)wKiDiv);
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  It set a new value for lower integral term limit
  * @param  pHandle: handler of the current instance of the PID component
  * @param  wLowerLimit: new lower integral term limit value
  * @retval None
  */
__weak void PID_SetLowerIntegralTermLimit(PID_Handle_t *pHandle, int32_t wLowerLimit)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->wLowerIntegralLimit = wLowerLimit;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  It set a new value for upper integral term limit
  * @param  pHandle: handler of the current instance of the PID component
  * @param  wUpperLimit: new upper integral term limit value
  * @retval None
  */
__weak void PID_SetUpperIntegralTermLimit(PID_Handle_t *pHandle, int32_t wUpperLimit)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->wUpperIntegralLimit = wUpperLimit;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  It set a new value for lower output limit
  * @param  pHandle: handler of the current instance of the PID component
  * @param  hLowerLimit: new lower output limit value
  * @retval None
  */
__weak void PID_SetLowerOutputLimit(PID_Handle_t *pHandle, int16_t hLowerLimit)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hLowerOutputLimit = hLowerLimit;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  It set a new value for upper output limit
  * @param  pHandle: handler of the current instance of the PID component
  * @param  hUpperLimit: new upper output limit value
  * @retval None
  */
__weak void PID_SetUpperOutputLimit(PID_Handle_t *pHandle, int16_t hUpperLimit)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hUpperOutputLimit = hUpperLimit;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  It set a new value into the PID Previous error variable required to
  *         compute derivative term
  * @param  pHandle: handler of the current instance of the PID component
  * @param  wPrevProcessVarError: New previous error variable
  * @retval None
  */
__weak void PID_SetPrevError(PID_Handle_t *pHandle, int32_t wPrevProcessVarError)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->wPrevProcessVarError = wPrevProcessVarError;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
  return;
}

/**
  * @brief  It updates the Kd gain
  * @param  pHandle: handler of the current instance of the PID component
  * @param  hKdGain: new Kd gain
  * @retval None
  */
__weak void PID_SetKD(PID_Handle_t *pHandle, int16_t hKdGain)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hKdGain = hKdGain;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  It returns the Kd gain
  * @param  pHandle: handler of the current instance of the PID component
  * @retval Kd gain
  */
__weak int16_t PID_GetKD(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0 : pHandle->hKdGain);
#else
  return (pHandle->hKdGain);
#endif
}

/**
  * @brief  It returns the Kd gain divisor of the PID object passed
  * @param  pHandle: handler of the current instance of the PID component
  * @retval Kd gain divisor
  */
__weak uint16_t PID_GetKDDivisor(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0U : pHandle->hKdDivisor);
#else
  return (pHandle->hKdDivisor);
#endif
}

/**
  * @brief  It returns the Kd gain divisor of the PID object passed
  * @param  pHandle: handler of the current instance of the PID component
  * @retval hKdDivisorPOW2: Ki divisor expressed as power of 2
  */
__weak uint16_t PID_GetKDDivisorPOW2(PID_Handle_t * pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0U : pHandle->hKdDivisorPOW2);
#else
  return (pHandle->hKdDivisorPOW2);
#endif
}

/**
  * @brief Sets @f$K_{dd}@f$, the derivative divisor parameter of the PID component
  *
  * @param pHandle handle on the instance of the PID component to update
  * @param hKdDivisorPOW2
  */
__weak void PID_SetKDDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKdDivisorPOW2)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hKdDivisorPOW2 = hKdDivisorPOW2;
    pHandle->hKdDivisor = (((uint16_t)1) << hKdDivisorPOW2);
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  This function compute the output of a PI regulator sum of its
  *         proportional and integral terms
  * @param  pHandle: handler of the current instance of the PID component
  * @param  wProcessVarError: current process variable error, intended as the reference
  *         value minus the present process variable value
  * @retval computed PI output
  */
__weak int16_t PI_Controller(PID_Handle_t *pHandle, int32_t wProcessVarError)
{
  int16_t returnValue;
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    returnValue = 0;
  }
  else
  {
#endif
    int32_t wProportional_Term;
    int32_t wIntegral_Term;
    int32_t wOutput_32;
    int32_t wIntegral_sum_temp;
    int32_t wDischarge = 0;
    int16_t hUpperOutputLimit = pHandle->hUpperOutputLimit;
    int16_t hLowerOutputLimit = pHandle->hLowerOutputLimit;

    /* Proportional term computation*/
    wProportional_Term = pHandle->hKpGain * wProcessVarError;

    /* Integral term computation */
    if (0 == pHandle->hKiGain)
    {
      pHandle->wIntegralTerm = 0;
    }
    else
    {
      wIntegral_Term = pHandle->hKiGain * wProcessVarError;
      wIntegral_sum_temp = pHandle->wIntegralTerm + wIntegral_Term;

      if (wIntegral_sum_temp < 0)
      {
        if (pHandle->wIntegralTerm > 0)
        {
          if (wIntegral_Term > 0)
          {
            wIntegral_sum_temp = INT32_MAX;
          }
          else
          {
            /* Nothing to do */
          }
        }
        else
        {
          /* Nothing to do */
        }
      }
      else
      {
        if (pHandle->wIntegralTerm < 0)
        {
          if (wIntegral_Term < 0)
          {
            wIntegral_sum_temp = -INT32_MAX;
          }
          else
          {
            /* Nothing to do */
          }
        }
        else
        {
          /* Nothing to do */
        }
      }

      if (wIntegral_sum_temp > pHandle->wUpperIntegralLimit)
      {
        pHandle->wIntegralTerm = pHandle->wUpperIntegralLimit;
      }
      else if (wIntegral_sum_temp < pHandle->wLowerIntegralLimit)
      {
        pHandle->wIntegralTerm = pHandle->wLowerIntegralLimit;
      }
      else
      {
        pHandle->wIntegralTerm = wIntegral_sum_temp;
      }
    }

#ifndef FULL_MISRA_C_COMPLIANCY_PID_REGULATOR
    /* WARNING: the below instruction is not MISRA compliant, user should verify
               that Cortex-M3 assembly instruction ASR (arithmetic shift right)
               is used by the compiler to perform the shifts (instead of LSR
               logical shift right)*/
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    wOutput_32 = (wProportional_Term >> pHandle->hKpDivisorPOW2) + (pHandle->wIntegralTerm >> pHandle->hKiDivisorPOW2);
#else
    wOutput_32 = (wProportional_Term / (int32_t)pHandle->hKpDivisor)
              + (pHandle->wIntegralTerm / (int32_t)pHandle->hKiDivisor);
#endif

    if (wOutput_32 > hUpperOutputLimit)
    {
      wDischarge = hUpperOutputLimit - wOutput_32;
      wOutput_32 = hUpperOutputLimit;
    }
    else if (wOutput_32 < hLowerOutputLimit)
    {
      wDischarge = hLowerOutputLimit - wOutput_32;
      wOutput_32 = hLowerOutputLimit;
    }
    else
    {
      /* Nothing to do here */
    }

    pHandle->wIntegralTerm += wDischarge;
    returnValue = (int16_t)wOutput_32;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
  return (returnValue);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  This function compute the output of a PID regulator sum of its
  *         proportional, integral and derivative terms
  * @param  pHandle: handler of the current instance of the PID component
  * @param  wProcessVarError: current process variable error, intended as the
  *         reference value minus the present process variable value
  * @retval PID computed output
  */

__weak int16_t PID_Controller(PID_Handle_t *pHandle, int32_t wProcessVarError)
{
  int16_t returnValue;
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    returnValue = 0;
  }
  else
  {
#endif
    int32_t wDifferential_Term;
    int32_t wDeltaError;
    int32_t wTemp_output;

    if (0 == pHandle->hKdGain) /* derivative terms not used */
    {
      wTemp_output = PI_Controller(pHandle, wProcessVarError);
    }
    else
    {
      wDeltaError = wProcessVarError - pHandle->wPrevProcessVarError;
      wDifferential_Term = pHandle->hKdGain * wDeltaError;

#ifndef FULL_MISRA_C_COMPLIANCY_PID_REGULATOR
      /* WARNING: the below instruction is not MISRA compliant, user should verify
         that Cortex-M3 assembly instruction ASR (arithmetic shift right)
         is used by the compiler to perform the shifts (instead of LSR
         logical shift right)*/
      //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      wDifferential_Term >>= pHandle->hKdDivisorPOW2;
#else
      wDifferential_Term /= ((int32_t)pHandle->hKdDivisor);
#endif

      pHandle->wPrevProcessVarError = wProcessVarError;

      wTemp_output = PI_Controller(pHandle, wProcessVarError) + wDifferential_Term;

      if (wTemp_output > pHandle->hUpperOutputLimit)
      {
        wTemp_output = pHandle->hUpperOutputLimit;
      }
      else if (wTemp_output < pHandle->hLowerOutputLimit)
      {
        wTemp_output = pHandle->hLowerOutputLimit;
      }
      else
      {
        /* Nothing to do */
      }
    }
    returnValue = (int16_t) wTemp_output;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
  return (returnValue);
}
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
