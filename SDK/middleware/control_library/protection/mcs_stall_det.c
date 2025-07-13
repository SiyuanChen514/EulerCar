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
  * @file      mcs_stall_det.c
  * @author    MCU Algorithm Team
  * @brief     This file contains motor stalling protection data struct and api declaration.
  */

#include "mcs_stall_det.h"
#include "mcs_assert.h"
#include "mcs_math.h"


/**
  * @brief Initilization motor stalling protection function.
  * @param stall Motor stalling handle.
  * @param ts Ctrl period (s).
  * @param currLimit The current amplitude that triggers fault. (A).
  * @param spdLimit The speed amplitude that triggers fault. (Hz).
  * @param timeLimit The threshold time that current amplitude over the limit (s).
  * @retval None.
  */
void STD_Init(STD_Handle *stall, float currLimit, float spdLimit, float timeLimit, float ts)
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
  * @retval Whether the motor is stalled, 1: motor stall, 0: no stall.
  */
bool STD_Exec_ByCurrSpd(STD_Handle *stall, float spdFbk, float currAmp)
{
    MCS_ASSERT_PARAM(stall != NULL);
    MCS_ASSERT_PARAM(currAmp >= 0.0f);
    /* Calculate current amplitude. */
    float spdAbs = Abs(spdFbk);
    /* Check if value goes over threshold for continuous cycles. */
    if (spdAbs > stall->spdLimit || currAmp < stall->currAmpLimit) {
        stall->timer = 0.0f;
        return false;
    }
    /* Time accumulation. */
    if (stall->timer < stall->timeLimit) {
        stall->timer += stall->ts;
        return false;
    }
    return true;
}

/**
  * @brief Clear stall history value.
  * @param stall Motor stalling handle.
  * @retval None.
  */
void STD_Clear(STD_Handle *stall)
{
    MCS_ASSERT_PARAM(stall != NULL);
    stall->timer = 0.0f;
}