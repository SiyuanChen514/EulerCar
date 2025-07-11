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
  * @file      mcs_spd_ctrl.c
  * @author    MCU Algorithm Team
  * @brief     This file provides function of motor speed control.
  */

#include "typedefs.h"
#include "mcs_spd_ctrl.h"
#include "mcs_assert.h"

/**
  * @brief Initialzer of speed control struct handle.
  * @param spdHandle Speed control struct handle.
  * @param PID_Param PI controller parameter table.
  * @param ts Speed control period.
  * @retval None.
  */
void SPDCTRL_Init(SPDCTRL_Handle *spdHandle, MOTOR_Param *mtrParam, const PI_Param piParam, float ts)
{
    MCS_ASSERT_PARAM(spdHandle != NULL);
    MCS_ASSERT_PARAM(mtrParam != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    /* reset speed loop PI */
    PID_Reset(&spdHandle->spdPi);
    /* Parameter Initialization. */
    spdHandle->mtrParam = mtrParam;
    spdHandle->ts = ts;
    spdHandle->spdPi.ts = ts;
    spdHandle->spdPi.kp = piParam.kp;
    spdHandle->spdPi.ki = piParam.ki;
    spdHandle->outLimit = piParam.upperLim;
    spdHandle->spdPi.upperLimit = piParam.upperLim;
    spdHandle->spdPi.lowerLimit = piParam.lowerLim;
}


/**
  * @brief Simplified speed controller PI calculation.
  * @param spdHandle Speed controller struct handle.
  * @param spdTarget The target speed value (Hz).
  * @param spdFbk Motor electrical speed (Hz).
  * @retval None.
  */
float SPDCTRL_Exec(SPDCTRL_Handle *spdHandle, float spdTarget, float spdFbk)
{
    MCS_ASSERT_PARAM(spdHandle != NULL);
    float iqRef;
    /* Speed error calculation */
    spdHandle->spdPi.error = spdTarget - spdFbk;
    /* speed controller pid calculation */
    iqRef = PI_Exec(&spdHandle->spdPi);
    return iqRef;
}

/**
  * @brief Clear historical values of speed controller.
  * @param spdHandle Speed controller struct handle.
  * @retval None.
  */
void SPDCTRL_Clear(SPDCTRL_Handle *spdHandle)
{
    MCS_ASSERT_PARAM(spdHandle != NULL);
    PID_Clear(&spdHandle->spdPi);
}

/**
  * @brief Reset the speed controller, fill with zero, NULL.
  * @param spdHandle Speed controller struct handle.
  * @retval None.
  */
void SPDCTRL_Reset(SPDCTRL_Handle *spdHandle)
{
    MCS_ASSERT_PARAM(spdHandle != NULL);
    /* Reset speed ring PI */
    PID_Reset(&spdHandle->spdPi);
    /* Reset the speed controller, fill with zero, NULL. */
    spdHandle->outLimit = 0.0f;
    spdHandle->mtrParam = NULL;
    spdHandle->ts = 0.0f;
}