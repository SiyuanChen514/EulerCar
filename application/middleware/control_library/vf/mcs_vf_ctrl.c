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
  * @file      mcs_vf_ctrl.c
  * @author    MCU Algorithm Team
  * @brief     This file provides functions declaration of v/f control.
  */

#include "mcs_vf_ctrl.h"
#include "mcs_math.h"
#include "mcs_assert.h"
#include "mcs_math_const.h"

/**
  * @brief Init the vf control handle.
  * @param vf The vf control handle.
  * @param spdThr Minimum (spdThr[0]) and maximum(spdThr[1]) speed thresholds for ramp command.
  * @param voltThr Minimum (voltThr[0]) and maximum(voltThr[1]) voltage for thresholds ramp command.
  * @param ts Control period.
  * @param spdCmd Motor target speed frequency (Hz).
  * @param spdSlope Slope of motor speed reference.
  * @retval None.
  */
void VF_Init(VF_Handle *vf, const float *spdThr, const float *voltThr, float ts, float spdCmd, float spdSlope)
{
    MCS_ASSERT_PARAM(vf != NULL);
    MCS_ASSERT_PARAM(spdThr != NULL);
    MCS_ASSERT_PARAM(voltThr != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    VF_Clear(vf);
    RMG_Init(&vf->rmg, ts, spdSlope);
    vf->spdCmd = spdCmd;
    vf->ts = ts;
    /* Set voltage-speed curve. */
    vf->spdThr[0] = spdThr[0]; /* The minimum vf speed. */
    vf->spdThr[1] = spdThr[1]; /* The maximum vf speed. */
    vf->voltThr[0] = voltThr[0]; /* The minimum voltage. */
    vf->voltThr[1] = voltThr[1]; /* The maximum vf voltage. */
    /* Calculate vf slope. */
    vf->slope = (voltThr[1] - voltThr[0]) / (spdThr[1] - spdThr[0]);
    vf->ratio.d = 1.0f;
    vf->ratio.q = 0.0f;
}

/**
  * @brief Vf control Execution.
  * @param vf The vf control handle.
  * @param vdqRef Dq axis voltage reference vf control.
  * @retval None.
  */
void VF_Exec(VF_Handle *vf, DqAxis *vdqRef)
{
    MCS_ASSERT_PARAM(vf != NULL);
    MCS_ASSERT_PARAM(vdqRef != NULL);
    /* Vf speed generation. */
    vf->spdRef = RMG_Exec(&vf->rmg, vf->spdCmd);
    float vs = 0.0f;
    float lowSpdHz = vf->spdThr[0];
    float highSpdHz = vf->spdThr[1];
    float voltMin = vf->voltThr[0];
    float voltMax = vf->voltThr[1];
    /* When the vf reference speed is less than the minimum speed threshold, */
    /* the voltage is set to the minimum voltage threshold. */
    /* When the vf reference speed is greater than the maximum speed threshold, */
    /* the voltage is set to the maximum voltage threshold. */
    if (vf->spdRef < lowSpdHz) {
        vs = voltMin;
    } else if (vf->spdRef > highSpdHz) {
        vs = voltMax;
    } else {
        vs = voltMin + vf->slope * (vf->spdRef - lowSpdHz);
    }
    /* Sets dq voltage based on the dq axis proportion */
    vf->vdqRef.d = vs * vf->ratio.d;
    vf->vdqRef.q = vs * vf->ratio.q;
    vf->vfAngle += DOUBLE_PI * vf->spdRef * vf->ts;
    vf->vfAngle = Mod(vf->vfAngle, DOUBLE_PI);
    if (vf->vfAngle > ONE_PI) {
        vf->vfAngle -= DOUBLE_PI;
    }
    if (vf->vfAngle < -ONE_PI) {
        vf->vfAngle += DOUBLE_PI;
    }
    vdqRef->d = vf->vdqRef.d;
    vdqRef->q = vf->vdqRef.q;
}

/**
  * @brief Clear the vf control history value.
  * @param vf The vf control handle.
  * @retval None.
  */
void VF_Clear(VF_Handle *vf)
{
    MCS_ASSERT_PARAM(vf != NULL);
    /* Clear history value. */
    vf->vfAngle = 0.0f;
    vf->spdRef = 0.0f;
    vf->vdqRef.d = 0.0f;
    vf->vdqRef.q = 0.0f;
}

/**
  * @brief Set vf control period.
  * @param vf The vf control handle.
  * @param ts The updated vf control period.
  * @retval None.
  */
void VF_SetTs(VF_Handle *vf, float ts)
{
    MCS_ASSERT_PARAM(vf != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    RMG_SetTs(&vf->rmg, ts);
    vf->ts = ts;
}

/**
  * @brief Set the slope for the motor to accelerate to the target speed.
  * @param vf The vf control handle.
  * @param spdSlope The slope.
  * @retval None.
  */
void VF_SetSpdSlope(VF_Handle *vf, float spdSlope)
{
    MCS_ASSERT_PARAM(vf != NULL);
    RMG_SetSlope(&vf->rmg, spdSlope);
}

/**
  * @brief Set the voltage reference ratio of d, q axis.
  * @param vf The vf control handle.
  * @param dRatio D axis reference voltage ratio.
  * @retval None.
  */
void VF_SetDRatio(VF_Handle *vf, float dRatio)
{
    MCS_ASSERT_PARAM(vf != NULL);
    MCS_ASSERT_PARAM(dRatio >= 0.0f && dRatio <= 1.0f);
    vf->ratio.d = dRatio;
    vf->ratio.q = Sqrt(1.0f - dRatio * dRatio);
}