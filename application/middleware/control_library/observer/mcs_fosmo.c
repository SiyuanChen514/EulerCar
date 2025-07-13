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
  * @file      mcs_fosmo.c
  * @author    MCU Algorithm Team
  * @brief     This file provides functions of position sliding mode observer (SMO) module.
  */

#include "mcs_fosmo.h"
#include "mcs_math_const.h"
#include "mcs_math.h"
#include "mcs_assert.h"


void FOSMO_Init(FOSMO_Handle *fosmo, const FOSMO_Param foSmoParam, const MOTOR_Param mtrParam, float ts)
{
    MCS_ASSERT_PARAM(fosmo != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    /* time sample, unit: s */
    fosmo->ts = ts;
    /* filter coefficient */
    fosmo->a1 = 1.0f - (fosmo->ts * mtrParam.mtrRs / mtrParam.mtrLd);
    fosmo->a2 = fosmo->ts / mtrParam.mtrLd;

    fosmo->kSmo = foSmoParam.gain;
    fosmo->lambda = foSmoParam.lambda; /* SMO coefficient of cut-off frequency = lambda * we, unit: rad/2. */
    /* smo angle  filcompAngle */
    fosmo->filCompAngle = Atan2(1.0f, 1.0f / fosmo->lambda);
    fosmo->pllBdw = foSmoParam.pllBdw;
    fosmo->fcLpf = foSmoParam.fcLpf;

    FOSMO_Clear(fosmo);

    fosmo->emfLpfMinFreq = foSmoParam.fcEmf; /* The minimum cutoff frequency of the back EMF filter is 2.0. */

    PLL_Init(&fosmo->pll, fosmo->ts, fosmo->pllBdw); // bdw

    /* low pass filter cutoff freqency for speed estimation is 40Hz */
    FOLPF_Init(&fosmo->spdFilter, fosmo->ts, fosmo->fcLpf);
}

/**
  * @brief Set parameters for fosmo.
  * @param fosmo The SMO handle.
  * @param gain The smo gain.
  * @param pllBdw The PLL bandwidth (Hz).
  * @param fc The first-order low pass filter cut-off frequency.
  * @retval None.
  */
void FOSMO_ParamUpdate(FOSMO_Handle *fosmo, float gain, float pllBdw, float fc)
{
    MCS_ASSERT_PARAM(fosmo != NULL);
    MCS_ASSERT_PARAM(pllBdw > 0.0f);
    MCS_ASSERT_PARAM(fc > 0.0f);

    fosmo->kSmo = gain;
    fosmo->fcLpf = fc;
    fosmo->pllBdw = pllBdw;

    /* Set PI parameters with given bandwidth */
    float we = DOUBLE_PI * pllBdw;
    fosmo->pll.pi.kp = 2.0f * we;
    fosmo->pll.pi.ki = we * we;

    /* Set LPF parameters with given fc */
    fosmo->spdFilter.fc = fc;
    float wcTs = DOUBLE_PI * fc * fosmo->spdFilter.ts;
    fosmo->spdFilter.a1 = 1.0f / (1.0f + wcTs); /* wcTs > 0 */
    fosmo->spdFilter.b1 = 1.0f - fosmo->spdFilter.a1;
}


/**
  * @brief Clear historical values of SMO handle.
  * @param fosmo SMO struct handle.
  * @retval None.
  */
void FOSMO_Clear(FOSMO_Handle *fosmo)
{
    MCS_ASSERT_PARAM(fosmo != NULL);
    /* Clear historical values of SMO handle */
    fosmo->ialbeEst.alpha     = 0.0f;
    fosmo->ialbeEst.beta      = 0.0f;
    fosmo->ialbeEstLast.alpha = 0.0f;
    fosmo->ialbeEstLast.beta  = 0.0f;
    fosmo->emfEstUnFil.alpha = 0.0f;
    fosmo->emfEstUnFil.beta  = 0.0f;
    fosmo->emfEstFil.alpha   = 0.0f;
    fosmo->emfEstFil.beta    = 0.0f;
    /* Clear historical values of PLL controller */
    PLL_Clear(&fosmo->pll);
    /* Clear historical values of first-order fosmo speed filter */
    FOLPF_Clear(&fosmo->spdFilter);
}

/**
  * @brief Calculation method of first-order SMO.
  * @param fosmo SMO struct handle.
  * @param ialbeFbk Feedback currents in the alpha-beta coordinate (A).
  * @param valbeRef FOC output voltages in alpha-beta coordinate (V).
  * @param refHz The reference frequency (Hz).
  * @retval None.
  */
void FOSMO_Exec(FOSMO_Handle *fosmo, const AlbeAxis *ialbeFbk, const AlbeAxis *valbeRef, float refHz)
{
    MCS_ASSERT_PARAM(fosmo != NULL);
    MCS_ASSERT_PARAM(ialbeFbk != NULL);
    MCS_ASSERT_PARAM(valbeRef != NULL);
    float err;
    float wcTs;
    float fcAbs = Abs(refHz);
    float filCompAngle; /* Compensation angle (rad) */
    float currAlpha = fosmo->ialbeEstLast.alpha;
    float currBeta  = fosmo->ialbeEstLast.beta;
    float emfUnAlpha = fosmo->emfEstUnFil.alpha;
    float emfUnBeta  = fosmo->emfEstUnFil.beta;
    /* Alpha beta current observation value */
    fosmo->ialbeEst.alpha =
        (fosmo->a1 * currAlpha) + (fosmo->a2 * (valbeRef->alpha - emfUnAlpha));
    fosmo->ialbeEst.beta =
        (fosmo->a1 * currBeta) + (fosmo->a2 * (valbeRef->beta - emfUnBeta));

    fosmo->ialbeEstLast.alpha = fosmo->ialbeEst.alpha;
    fosmo->ialbeEstLast.beta   = fosmo->ialbeEst.beta;

    /* Estmated back EMF by sign function. */
    err = fosmo->ialbeEst.alpha - ialbeFbk->alpha;
    fosmo->emfEstUnFil.alpha = fosmo->kSmo * ((err > 0.0f) ? 1.0f : -1.0f);
    err = fosmo->ialbeEst.beta - ialbeFbk->beta;
    fosmo->emfEstUnFil.beta  = fosmo->kSmo * ((err > 0.0f) ? 1.0f : -1.0f);

    /* Estmated back EMF is filtered by first-order LPF. */
    if (fcAbs <= fosmo->emfLpfMinFreq) {
        wcTs = fosmo->emfLpfMinFreq * DOUBLE_PI * fosmo->ts * fosmo->lambda;
    } else {
        wcTs = fcAbs * DOUBLE_PI * fosmo->ts * fosmo->lambda;
    }
    fosmo->emfEstFil.alpha = (fosmo->emfEstFil.alpha + wcTs * fosmo->emfEstUnFil.alpha) / (wcTs + 1.0f);
    fosmo->emfEstFil.beta  = (fosmo->emfEstFil.beta + wcTs * fosmo->emfEstUnFil.beta) / (wcTs + 1.0f);

    /* Get phase angle and frequency from BEMF by PLL. */
    PLL_Exec(&fosmo->pll, -fosmo->emfEstFil.alpha, fosmo->emfEstFil.beta);

    /* Compensation phase lag caused by the LPF. */
    filCompAngle = (refHz > 0.0f) ? (fosmo->filCompAngle) : AngleSub(ONE_PI, fosmo->filCompAngle);
    fosmo->elecAngle = Mod(fosmo->pll.angle + filCompAngle, DOUBLE_PI);
    if (fosmo->elecAngle > ONE_PI) {
        fosmo->elecAngle -= DOUBLE_PI;
    }
    if (fosmo->elecAngle < -ONE_PI) {
        fosmo->elecAngle += DOUBLE_PI;
    }
    /* Estmated speed is filtered by first-order LPF. */
    fosmo->spdEst = FOLPF_Exec(&fosmo->spdFilter, fosmo->pll.freq);
}

/**
  * @brief Set ts for first-order SMO.
  * @param fosmo SMO struct handle.
  * @param ts Control period (s).
  * @retval None.
  */
void FOSMO_SetTs(FOSMO_Handle *fosmo, float ts)
{
    MCS_ASSERT_PARAM(fosmo != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    fosmo->ts = ts;
    /* Set PLL ts and filter ts. */
    PLL_SetTs(&fosmo->pll, ts);
    FOLPF_SetTs(&fosmo->spdFilter, ts);
}

/**
  * @brief Set coefficient of cut-off frequency(lambda * we rad/2) for first-order SMO.
  * @param fosmo SMO struct handle.
  * @param lambda SMO filter coefficient.
  * @retval None.
  */
void FOSMO_SetLambda(FOSMO_Handle *fosmo, float lambda)
{
    MCS_ASSERT_PARAM(fosmo != NULL);
    MCS_ASSERT_PARAM(lambda > 0.0f);
    fosmo->lambda = lambda;
    fosmo->filCompAngle = Atan2(1.0f, 1.0f / fosmo->lambda);
}