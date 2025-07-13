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
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
  * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  * @file      mcs_fw_ctrl.c
  * @author    MCU Algorithm Team
  * @brief     This file provides Flux-Weakening control for motor control.
  */
#include "mcs_fw_ctrl.h"
#include "mcs_math.h"
#include "mcs_math_const.h"
#include "mcs_assert.h"

/**
  * @brief Clear historical values of Flux-Weakening handle.
  * @param fw Flux-Weakening struct handle.
  * @retval None.
  */
static void FW_Clear(FW_Handle *fw)
{
    MCS_ASSERT_PARAM(fw != NULL);
    fw->idRef = 0.0f;
}

/**
  * @brief Flux-Weakening control Handle Initialization.
  * @param fw Flux-Weakening struct handle.
  * @param ts Control period (s).
  * @param enable Enable flux-weakening.
  * @param currMax Maximum phase current (A).
  * @param idDemag Demagnetizing d-axis current (A).
  * @param thr .
  * @retval None.
  */
void FW_Init(FW_Handle *fw, float ts, bool enable, float currMax, float idDemag, float thr, float slope)
{
    MCS_ASSERT_PARAM(fw != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    /* Indicates whether to enable the Flux-Weakening field function. */
    fw->enable = enable;
    fw->ts = ts;
    fw->udcThreshPer = thr * ONE_DIV_SQRT3;
    /* id control slope */
    fw->idSlope = slope;
    fw->idMaxAmp = (currMax < idDemag) ? currMax : idDemag;
    fw->currMaxSquare = currMax * currMax;
    FW_Clear(fw);
}

/**
  * @brief Flux-Weakening calculation execution function.
  * @param fw Flux-Weakening struct handle.
  * @param udqRef dq axis voltage reference.
  * @param udc bus voltage.
  * @param idqRefRaw  Command value of the d and q axis current.
  * @retval None.
  */
void FW_Exec(FW_Handle *fw, DqAxis udqRef, float udc, DqAxis *idqRefRaw)
{
    MCS_ASSERT_PARAM(fw != NULL);
    MCS_ASSERT_PARAM(udc > 0.0f);
    MCS_ASSERT_PARAM(idqRefRaw != NULL);
    float udcLimit = udc * fw->udcThreshPer;
    float voltRefAmp = Sqrt(udqRef.d * udqRef.d + udqRef.q * udqRef.q);
    float voltErr = udcLimit - voltRefAmp;
    float idRefRaw = idqRefRaw->d;
    float iqRefRaw = idqRefRaw->q;
    float iqRef;
    float dir = (idqRefRaw->q > 0.0f) ? 1.0f : -1.0f;

    /* Check whether the Flux-Weakening field function is enabled. */
    if (!fw->enable) {
        fw->idRef = idRefRaw;
        /* if fw is disabled, just return without any change. */
        return;
    }
    float idStep = fw->idSlope * fw->ts;
    /* Adjust the injection d-axis current based on the output voltage error. */
    /* When voltage error is positive, adjust id to idRefRaw, no need to fw. */
    if (voltErr >= 0.0f) {
        fw->idRef += idStep;
        if (fw->idRef > idRefRaw) {
            fw->idRef = idRefRaw;
        }
    } else {
        /* When voltage error is negative. Add negtive id to the motor, need to fw. */
        fw->idRef -= idStep;
        if (fw->idRef < -fw->idMaxAmp) {
            fw->idRef = -fw->idMaxAmp;
        }
    }
    
    /* Limit q-axis current output. */
    float idRefSquare = fw->idRef * fw->idRef;
    if (idRefSquare + iqRefRaw * iqRefRaw > fw->currMaxSquare) {
        iqRef = dir * Sqrt(fw->currMaxSquare - idRefSquare);
    } else {
        iqRef = iqRefRaw;
    }
    
    idqRefRaw->d = fw->idRef;
    idqRefRaw->q = iqRef;

    return;
}

/**
  * @brief Set ts of Flux-Weakening.
  * @param fw Flux-Weakening struct handle.
  * @param ts Control period (s).
  * @retval None.
  */
void FW_SetTs(FW_Handle *fw, float ts)
{
    MCS_ASSERT_PARAM(fw != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    fw->ts = ts;
}