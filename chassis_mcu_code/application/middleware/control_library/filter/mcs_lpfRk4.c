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
  * @file      mcs_LpfRk4.c
  * @author    MCU Algorithm Team
  * @brief     This file provides function of 4-order low-pass filter.
  */

#include "mcs_lpfRk4.h"
#include "mcs_math_const.h"
#include "mcs_assert.h"
#include "mcs_math.h"

#define RK4_GAIN             (0.5f)
#define RK4_COEFF            (0.1666667f)
#define LARGE_NUM            (100000000.0f)
#define SMALL_NUM            (-100000000.0f)

/**
  * @brief Clear historical values of 4-order low-pass filter handle.
  * @param LPF_RK4_Handle 4-order low-pass filter handle.
  * @retval None.
  */
void LPFRK4_Clear(LPF_RK4_Handle *lpf)
{
    MCS_ASSERT_PARAM(lpf != NULL);
    lpf->y1 = 0.0f;
}

/**
  * @brief Calculation method of 4-order low-pass filter.
  * @param LPF_RK4_Handle filter handle.
  * @param u The signal that wants to be filtered.
  * @param freq Cut-off frequency (Hz).
  * @param ts Control period (s).
  * @retval The signal that is filered.
  */
float LPFRK4_Exec(LPF_RK4_Handle *lpf, float u, float freq, float ts)
{
    MCS_ASSERT_PARAM(lpf != NULL);
    MCS_ASSERT_PARAM(freq > 0.0f);
    MCS_ASSERT_PARAM(ts > 0.0f);
    float wc = freq * DOUBLE_PI;
    float y1 = lpf->y1;
    float k1, k2, k3, k4, temp;

    /* Calculate K1. */
    float diff = wc * (u - y1);
    k1 = diff * ts;
    temp = y1 + k1 * RK4_GAIN;
    /* Calculate K2. */
    diff = wc * (u - temp);
    k2 = diff * ts;
    temp = y1 + k2 * RK4_GAIN;
    /* Calculate K3. */
    diff = wc * (u - temp);
    k3 = diff * ts;
    temp = y1 + k3;
    /* Calculate K4. */
    diff = wc * (u - temp);
    k4 = diff * ts;
    /* Calculate the final result. */
    y1 += (k1 + 2.0f * k2 + 2.0f * k3 + k4) * RK4_COEFF;
    y1 = Clamp(y1, LARGE_NUM, SMALL_NUM);
    lpf->y1 = y1;

    return y1;
}