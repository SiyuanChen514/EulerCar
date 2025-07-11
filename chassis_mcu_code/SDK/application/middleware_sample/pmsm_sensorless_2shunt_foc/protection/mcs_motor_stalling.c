/**
  * @copyright Copyright (c) 2023, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      mcs_motor_stalling.c
  * @author    MCU Algorithm Team
  * @brief     This file contains motor stalling protection data struct and api definition.
  */

/* Includes ------------------------------------------------------------------------------------ */
#include "mcs_motor_stalling.h"
#include "mcs_prot_user_config.h"
#include "mcs_math.h"
#include "mcs_math_const.h"
#include "mcs_assert.h"

/**
  * @brief Initilization motor stalling protection function.
  * @param stall Motor stalling handle.
  * @param ts Ctrl period (s).
  * @param currLimit The current amplitude that triggers fault. (A).
  * @param spdLimit The speed amplitude that triggers fault. (Hz).
  * @param timeLimit The threshold time that current amplitude over the limit (s).
  * @retval None.
  */
void STP_Init(STP_Handle *stall, float ts, float currLimit, float spdLimit, float timeLimit)
{
    MCS_ASSERT_PARAM(stall != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    MCS_ASSERT_PARAM(currLimit > 0.0f);
    MCS_ASSERT_PARAM(spdLimit > 0.0f);
    MCS_ASSERT_PARAM(timeLimit > 0.0f);
    /* Configuring parameters for stalling detection. */
    stall->ts = ts;
    /* Current threshold and speed threshold for stalling fault. */
    stall->currAmpLimit = currLimit;
    stall->spdLimit = spdLimit;
    stall->timeLimit = timeLimit;
    stall->timer = 0.0f;
}

/**
  * @brief Motor stalling detection.
  * @param stall Motor stalling handle.
  * @param motorErrStatus Motor error status.
  * @param spd Speed feedback (Hz).
  * @param idq Dq-axis current feedback (A).
  * @retval None.
  */
void STP_Det_ByCurrSpd(STP_Handle *stall, MotorErrStatusReg *motorErrStatus, float spd, DqAxis idq)
{
    MCS_ASSERT_PARAM(stall != NULL);
    MCS_ASSERT_PARAM(motorErrStatus != NULL);
    MCS_ASSERT_PARAM(spd > 0.0f);
    /* Calculate current amplitude. */
    float currAmp = Sqrt(idq.d * idq.d + idq.q * idq.q);
    float spdAbs = Abs(spd);
    /* Check if value goes over threshold for continuous cycles. */
    if (currAmp < stall->currAmpLimit || spdAbs > stall->spdLimit) {
        stall->timer = 0.0f;
        return;
    }
    /* Time accumulation. */
    if (stall->timer < stall->timeLimit) {
        stall->timer += stall->ts;
        return;
    }
    motorErrStatus->Bit.motorStalling = 1;
}

/**
  * @brief Motor stalling protection execution.
  * @param motorErrStatus Motor error status.
  * @param aptAddr Three-phase APT address pointer.
  * @retval None.
  */
void STP_Exec(MotorErrStatusReg *motorErrStatus, APT_RegStruct **aptAddr)
{
    MCS_ASSERT_PARAM(motorErrStatus != NULL);
    MCS_ASSERT_PARAM(aptAddr != NULL);
    if (motorErrStatus->Bit.motorStalling == 0) {
        return;
    }

    /* Disable three-phase pwm output. */
    ProtSpo_Exec(aptAddr);
    return;
}

/**
  * @brief Motor stalling protection error status clear.
  * @param stall Motor stalling handle.
  * @retval None.
  */
void STP_Clear(STP_Handle *stall)
{
    MCS_ASSERT_PARAM(stall != NULL);
    stall->timer = 0.0f;
}