/**
  * @ Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2022-2023. All rights reserved.
  * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
  * following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
  * disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
  * following disclaimer in the documentation and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
  * products derived from this software without specific prior written permission.
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
  * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  * @file      mcs_pll.c
  * @author    MCU Algorithm Team
  * @brief     This file provides function of phase-locked loop (PLL) module.
  */

#include "mcs_pll.h"
#include "mcs_math.h"
#include "mcs_math_const.h"
#include "mcs_assert.h"

/**
  * @brief Initialzer of Pll struct handle.
  * @param pllHandle pll struct handle.
  * @param ts control period (s).
  * @param bdw bandwidth (Hz).
  * @retval None.
  */
void PLL_Init(PLL_Handle *pllHandle, float ts, float bdw)
{
    MCS_ASSERT_PARAM(pllHandle != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    MCS_ASSERT_PARAM(bdw > 0.0f);
    /* Reset PLL PID. */
    PID_Reset(&pllHandle->pi);
    /* Initializing PLL Parameters. */
    pllHandle->ts = ts;
    pllHandle->pi.upperLimit = LARGE_FLOAT; /* The upper limit value of the pid comp output. */
    pllHandle->pi.lowerLimit = -pllHandle->pi.upperLimit;
    pllHandle->minAmp = 0.1f; /* Minimum value of the input value in case of the divergence of the PLL. */
    pllHandle->freq = 0.0f;
    pllHandle->angle = 0.0f;
    pllHandle->ratio = DOUBLE_PI * ts;
    pllHandle->pllBdw = bdw;
    pllHandle->pi.ts = pllHandle->ts;
    PLL_ParamUpdate(pllHandle, pllHandle->pllBdw);
}

/**
  * @brief Updating PLL PI Parameters.
  * @param pllHandle pll struct handle.
  * @param bdw bandwidth (Hz).
  * @retval None.
  */
void PLL_ParamUpdate(PLL_Handle *pllHandle, float bdw)
{
    MCS_ASSERT_PARAM(pllHandle != NULL);
    MCS_ASSERT_PARAM(bdw > 0.0f);
    float we = bdw * DOUBLE_PI; /* PLL bandwidth (unit: Hz) */
    pllHandle->pi.kp = 2.0f * we; /* 2.0f * we */
    pllHandle->pi.ki = we * we;
}

/**
  * @brief Reset the PLL handle, fill all parameters with zero.
  * @param pllHandle PLL struct handle.
  * @retval None.
  */
void PLL_Reset(PLL_Handle *pllHandle)
{
    MCS_ASSERT_PARAM(pllHandle != NULL);
    /* Reset PLL PID parameters */
    PID_Reset(&(pllHandle->pi));
    pllHandle->minAmp     = 0.0f;
    pllHandle->ts         = 0.0f;
    pllHandle->ratio      = 0.0f;
    pllHandle->freq       = 0.0f;
    pllHandle->angle      = 0;
}

/**
  * @brief Clear historical values of PLL controller.
  * @param pllHandle PLL struct handle.
  * @retval None.
  */
void PLL_Clear(PLL_Handle *pllHandle)
{
    MCS_ASSERT_PARAM(pllHandle != NULL);
    PID_Clear(&pllHandle->pi);
}

/**
  * @brief Calculation method of PLL controller.
  * @param pllHandle PLL struct handle.
  * @param sinVal Input sin value.
  * @param cosVal Input cos value.
  * @retval None.
  */
void PLL_Exec(PLL_Handle *pllHandle, float sinVal, float cosVal)
{
    MCS_ASSERT_PARAM(pllHandle != NULL);

    float amplitude = Sqrt(sinVal * sinVal + cosVal * cosVal);
    amplitude = (amplitude < pllHandle->minAmp) ? pllHandle->minAmp : amplitude; /* amplitude > minAmp > 0 */

    TrigVal localTrigVal;
    pllHandle->angle += pllHandle->freq * pllHandle->ratio;
    pllHandle->angle = Mod(pllHandle->angle, DOUBLE_PI);
    if (pllHandle->angle > ONE_PI) {
        pllHandle->angle -= DOUBLE_PI;
    }
    if (pllHandle->angle < -ONE_PI) {
        pllHandle->angle += DOUBLE_PI;
    }
    TrigCalc(&localTrigVal, pllHandle->angle);

    float err = sinVal * localTrigVal.cos - cosVal * localTrigVal.sin;
    pllHandle->pi.error = err / amplitude; /* amplitude != 0 */
    pllHandle->freq = PI_Exec(&pllHandle->pi);
}

/**
  * @brief Set ts of PLL controller.
  * @param pllHandle PLL struct handle.
  * @param ts control period (s).
  * @retval None.
  */
void PLL_SetTs(PLL_Handle *pllHandle, float ts)
{
    MCS_ASSERT_PARAM(pllHandle != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    /* It is need to modify the pid sample time, pll ratio when set PLL ts. */
    pllHandle->ts = ts;
    PID_SetTs(&pllHandle->pi, ts);
    pllHandle->ratio = DOUBLE_PI * ts;
}
