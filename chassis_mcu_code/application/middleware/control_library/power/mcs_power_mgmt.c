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
  * @file      mcs_power_mgmt.c
  * @author    MCU Algorithm Team
  * @brief     This file provides functions of motor average power management.
  */


#include "mcs_power_mgmt.h"
#include "mcs_math.h"
#include "mcs_assert.h"

/**
  * @brief Init motor power management.
  * @param avgPower Pointer of motor power handle.
  * @param vdqRef Pointer of vdqRef handle.
  * @param idqFbk Pointer of idqFbk handle.
  * @retval None.
  */
void MotorPowerInit(POWER_Handle *avgPower, DqAxis *vdqRef, DqAxis *idqFbk)
{
    MCS_ASSERT_PARAM(avgPower != NULL);
    MCS_ASSERT_PARAM(vdqRef != NULL);
    MCS_ASSERT_PARAM(idqFbk != NULL);
    /* Initialization. */
    avgPower->avgPower = 0.0f;
    /* Initialization. */
    avgPower->vdqRef = vdqRef;
    avgPower->idqFbk = idqFbk;
}

/**
  * @brief Power result value.
  * @param avgPower Pointer of motor power handle.
  * @retval Motor power value (w).
  */
float MotorPowerCalc(POWER_Handle *avgPower)
{
    MCS_ASSERT_PARAM(avgPower != NULL);
    /* Calculate average power. */
    float activePower = 1.5f * (avgPower->idqFbk->d * avgPower->vdqRef->d + avgPower->idqFbk->q * avgPower->vdqRef->q);
    avgPower->avgPower = activePower;
    return activePower;
}

/**
  * @brief Clear motor power history value.
  * @param avgPower Pointer of motor power handle.
  * @retval None.
  */
void MotorPowerClear(POWER_Handle *avgPower)
{
    MCS_ASSERT_PARAM(avgPower != NULL);
    /* Clear history value. */
    avgPower->avgPower = 0.0f;
}