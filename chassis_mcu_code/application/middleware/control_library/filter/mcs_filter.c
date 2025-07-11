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
  * @file      mcs_filter.c
  * @author    MCU Algorithm Team
  * @brief     This file provides functions of first-order filter.
  */

#include "mcs_filter.h"
#include "mcs_math_const.h"
#include "mcs_assert.h"

/**
  * @brief Initialzer of first-order low-pass filter handle.
  * @param lpfHandle First-order filter handle.
  * @param ts Control period (s).
  * @param fc Cut-off frequency (Hz).
  * @retval None.
  */
void FOLPF_Init(FOFLT_Handle *lpfHandle, float ts, float fc)
{
    MCS_ASSERT_PARAM(lpfHandle != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    MCS_ASSERT_PARAM(fc > 0.0f);
    lpfHandle->ts = ts;
    lpfHandle->fc = fc;

    FOLPF_Clear(lpfHandle);

    /* y(k) = (1/(1+wcTs)) * y(k-1) + (wcTs/(1+wcTs)) * u(k) */
    float wcTs = DOUBLE_PI * fc * ts;
    lpfHandle->a1 = 1.0f / (1.0f + wcTs); /* wcTs > 0 */
    lpfHandle->b1 = 1.0f - lpfHandle->a1;
}

/**
  * @brief Clear historical values of first-order filter handle.
  * @param FOFLT_Handle First-order filter handle.
  * @retval None.
  */
void FOLPF_Clear(FOFLT_Handle *lpfHandle)
{
    MCS_ASSERT_PARAM(lpfHandle != NULL);
    lpfHandle->uLast = 0.0f;
    lpfHandle->yLast = 0.0f;
}

/**lpfBkwd
  * @brief Calculation method of first-order filter.
  * @param lpfHandle First-order filter handle.
  * @param u The signal that wants to be filtered.
  * @retval The signal that is filtered.
  */
float FOLPF_Exec(FOFLT_Handle *lpfHandle, float u)
{
    MCS_ASSERT_PARAM(lpfHandle != NULL);
    float out;
    /* Transfer Func: G(s) = kw/(s+w), k = 1. */
    /* y(k) = (1/(1+wcTs)) * y(k-1) + (wcTs/(1+wcTs)) * u(k) */
    out = lpfHandle->a1 * lpfHandle->yLast + lpfHandle->b1 * u;
    lpfHandle->yLast = out;
    return out;
}

/**
  * @brief Set ts of first-order filter.
  * @param lpfHandle First-order filter handle.
  * @param ts Control period (s).
  * @retval None.
  */
void FOLPF_SetTs(FOFLT_Handle *lpfHandle, float ts)
{
    MCS_ASSERT_PARAM(lpfHandle != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    lpfHandle->ts = ts;

    float wcTs = DOUBLE_PI * lpfHandle->fc * ts;
    lpfHandle->a1 = 1.0f / (1.0f + wcTs); /* wcTs > 0 */
    lpfHandle->b1 = 1.0f - lpfHandle->a1;
}