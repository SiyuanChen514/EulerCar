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
  * @file      mcs_if_ctrl.c
  * @author    MCU Algorithm Team
  * @brief     This file provides function of I/F control.
  */

#include "mcs_if_ctrl.h"
#include "mcs_assert.h"
#include "mcs_math_const.h"

/**
  * @brief Initialzer of I/F control struct handle.
  * @param ifHandle I/F handle.
  * @param targetAmp Target value of the I/F current (A).
  * @param currSlope Current slope.
  * @param stepAmpPeriod Step control period, using systick---spd_loop_ctrl_period (s).
  * @param anglePeriod Calculation period of the I/F angle---curr_loop_ctrl_period (s).
  * @retval None.
  */
void IF_Init(IF_Handle *ifHandle, float targetAmp, float currSlope, float stepAmpPeriod, float anglePeriod)
{
    MCS_ASSERT_PARAM(ifHandle != NULL);
    MCS_ASSERT_PARAM(targetAmp > 0.0f);
    MCS_ASSERT_PARAM(currSlope > 0.0f);
    MCS_ASSERT_PARAM(anglePeriod > 0.0f);
    MCS_ASSERT_PARAM(stepAmpPeriod > 0.0f);
    /* Initialize IF parameters. */
    ifHandle->targetAmp = targetAmp;
    ifHandle->stepAmp = currSlope * stepAmpPeriod;  /* current step increment */
    ifHandle->curAmp = 0.0f;
    /* Angle period. */
    ifHandle->anglePeriod = anglePeriod;
    ifHandle->angle = 0.0f;
}

/**
  * @brief Clear historical values of first-order filter handle.
  * @param ifHandle I/F control handle.
  * @retval None.
  */
void IF_Clear(IF_Handle *ifHandle)
{
    MCS_ASSERT_PARAM(ifHandle != NULL);
    ifHandle->curAmp = 0.0f;
    ifHandle->angle = 0;
}

/**
  * @brief I/F current amplitude calculation.
  * @param ifHandle I/F control handle.
  * @retval I/F current amplitude (A).
  */
float IF_CurrAmpCalc(IF_Handle *ifHandle)
{
    MCS_ASSERT_PARAM(ifHandle != NULL);
    /* Calculation of IF Current Amplitude */
    if (ifHandle->curAmp < ifHandle->targetAmp) {
        ifHandle->curAmp += ifHandle->stepAmp;
    } else {
        ifHandle->curAmp = ifHandle->targetAmp;
    }

    return ifHandle->curAmp;
}

/**
  * @brief I/F current angle calculation.
  * @param ifHandle I/F control handle.
  * @param spdRef Frequency of current vector.
  * @retval I/F output angle.
  */
float IF_CurrAngleCalc(IF_Handle *ifHandle, float spdRef)
{
    MCS_ASSERT_PARAM(ifHandle != NULL);
    /* Calculate IF angle. */
    ifHandle->angle += spdRef * DOUBLE_PI * ifHandle->anglePeriod;
    /* Limit the angle: [-pi, pi]. */
    if (ifHandle->angle > ONE_PI) {
        ifHandle->angle -= DOUBLE_PI;
    }
    if (ifHandle->angle < -ONE_PI) {
        ifHandle->angle += DOUBLE_PI;
    }

    return ifHandle->angle;
}

/**
  * @brief Set ts of I/F.
  * @param ifHandle I/F control handle.
  * @param ts Control period (s).
  * @retval None.
  */
void IF_SetAngleTs(IF_Handle *ifHandle, float ts)
{
    MCS_ASSERT_PARAM(ifHandle != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    ifHandle->anglePeriod = ts;
}