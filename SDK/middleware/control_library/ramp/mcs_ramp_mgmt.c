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
  * @file      mcs_ramp_mgmt.c
  * @author    MCU Algorithm Team
  * @brief     This file provides function of ramp function.
  */

#include "mcs_ramp_mgmt.h"
#include "mcs_assert.h"

/**
  * @brief Initializer of RMG handle.
  * @param rmg: Pointer of RMG handle.
  * @param ts: Control period of the RMG module.
  * @param slope: Target value divide time of variation.
  * @retval None.
  */
void RMG_Init(RMG_Handle *rmg, float ts, float slope)
{
    MCS_ASSERT_PARAM(rmg != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    /* Initializer of RMG handle. */
    rmg->slope = slope;
    rmg->yLast = 0.0f;
    rmg->ts = ts;
    rmg->delta = rmg->slope * rmg->ts;
}

/**
  * @brief Clear historical values of RMG handle.
  * @param rmg: Pointer of RMG handle.
  * @retval None.
  */
void RMG_Clear(RMG_Handle *rmg)
{
    MCS_ASSERT_PARAM(rmg != NULL);
    rmg->yLast = 0.0f;
}

/**
  * @brief Ramp generation and management.
  * @param rmg: Pointer of RMG handle.
  */
float RMG_Exec(RMG_Handle *rmg, float targetVal)
{
    MCS_ASSERT_PARAM(rmg != NULL);
    float out;
    /* Calculate the current output value based on the target value and slope. */
    if (rmg->yLast <= (targetVal - rmg->delta)) {
        out = rmg->yLast + rmg->delta;
    } else if (rmg->yLast >= (targetVal + rmg->delta)) {
        out = rmg->yLast - rmg->delta;
    } else {
        out = rmg->yLast = targetVal;
    }
    /* Recording and outputting slope calculation results. */
    rmg->yLast = out;
    return out;
}

/**
  * @brief Set ts for ramp.
  * @param rmg Pointer of RMG handle.
  * @retval The reference value which is ramped.
  */
void RMG_SetTs(RMG_Handle *rmg, float ts)
{
    MCS_ASSERT_PARAM(rmg != NULL);
    /* Set ts. */
    rmg->ts = ts;
    rmg->delta = rmg->slope * rmg->ts;
}

void RMG_SetSlope(RMG_Handle *rmg, float slope)
{
    MCS_ASSERT_PARAM(rmg != NULL);
    MCS_ASSERT_PARAM(slope > 0.0f);
    /* Set slope. */
    rmg->slope = slope;
    rmg->delta = rmg->slope * rmg->ts;
}