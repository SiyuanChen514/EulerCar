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
  * @file      mcs_r1_svpwm.c
  * @author    MCU Algorithm Team
  * @brief     This file provides function of Space-vector pulse-width-modulation calculations
  *            in case of single shunt current sample and current reconstruction.
  */


#include "mcs_r1_svpwm.h"
#include "mcs_assert.h"


/**
  * @brief R1SVPWM handlel init.
  * @param r1svHandle The R1SVPWM handle.
  * @param voltPu Voltage per unit value.
  */
void R1SVPWM_Init(R1SVPWM_Handle *r1svHandle, float voltPu, float samplePointShift, float sampleWindow)
{
    MCS_ASSERT_PARAM(r1svHandle != NULL);
    MCS_ASSERT_PARAM(voltPu > 0.0f);
    MCS_ASSERT_PARAM(sampleWindow > 0.0f && sampleWindow < 1.0f);
    MCS_ASSERT_PARAM(samplePointShift > -1.0f && samplePointShift < 1.0f);
    /* Initialize the phase-shift sampling window size and sampling point offset. */
    r1svHandle->samplePointShift = samplePointShift;
    r1svHandle->sampleWindow     = sampleWindow;
    /* Initialize the Voltage per unit value */
    r1svHandle->voltPu           = voltPu;
    r1svHandle->oneDivVoltPu     = 1.0f / voltPu;
}

/**
  * @brief R1SVPWM clear.
  * @param r1svHandle The R1SVPWM handle.
  * @retval None.
  */
void R1SVPWM_Clear(R1SVPWM_Handle *r1svHandle)
{
    MCS_ASSERT_PARAM(r1svHandle != NULL);
    /* Clear the historical values calculated by the R1 SVPWM. */
    r1svHandle->voltIndex      = 0;
    r1svHandle->voltIndexLast  = 0;
    r1svHandle->samplePoint[SOCA] = 0.0f;
    r1svHandle->samplePoint[SOCB] = 0.0f;
}

/**
  * @brief Phase shift calculation for single resistance sampling.
  * @param r1SvCalc R1 svpwm calculation handle.
  * @param sampleWindow sample window.
  * @retval None.
  */
void R1SVPWM_PhaseShift(R1SVPWM_CALC_Handle *r1SvCalc, float sampleWindow)
{
    MCS_ASSERT_PARAM(r1SvCalc != NULL);
    MCS_ASSERT_PARAM(sampleWindow > 0.0f && sampleWindow < 1.0f);
    /* Pointer to the array of left and right comparison values. */
    float *compRight = r1SvCalc->compRight;
    float *compLeft  = r1SvCalc->compLeft;
    /* Comparison of three levels. */
    float compMax = r1SvCalc->svCalc.comp[SVPWM_COMP_VAL_MAX];
    float compMid = r1SvCalc->svCalc.comp[SVPWM_COMP_VAL_MID];
    float compMin = r1SvCalc->svCalc.comp[SVPWM_COMP_VAL_MIN];
    /* action time of two vectors */
    float t1 = r1SvCalc->svCalc.t1;
    float t2 = r1SvCalc->svCalc.t2;
    /**
      * PWM phase shift:
      * When the action time t1 of the first vector is less than the minimum sampling window,
      * the phase with the smallest comparison value(with the largest duty) shifts to the right.
      */
    if (t1 < sampleWindow) {
        compRight[SVPWM_COMP_VAL_MIN] = compMid - sampleWindow;
        compLeft[SVPWM_COMP_VAL_MIN] = compMin + sampleWindow - t1;
    } else {
        compRight[SVPWM_COMP_VAL_MIN] = compMin;
        compLeft[SVPWM_COMP_VAL_MIN] = compMin;
    }

    /**
      * When the action time t2 of the second vector is less than the minimum sampling window,
      * the phase with the largest comparison value (minimum duty) shifts to the left.
      */
    if (t2 < sampleWindow) {
        compRight[SVPWM_COMP_VAL_MAX] = compMid + sampleWindow;
        compLeft[SVPWM_COMP_VAL_MAX] = compMax - sampleWindow + t2;
    } else {
        compRight[SVPWM_COMP_VAL_MAX] = compMax;
        compLeft[SVPWM_COMP_VAL_MAX] = compMax;
    }
    /* intermediate large unshifted phase */
    compRight[SVPWM_COMP_VAL_MID] = compMid;
    compLeft[SVPWM_COMP_VAL_MID] = compMid;
}

/**
  * @brief The duty cycles of PWM wave of three-phase upper switches are
  *        calculated in the two-phase stationary coordinate system (albe).
  * @param r1svHandle R1SVPWM struct handle.
  * @param uAlbe Input voltage vector.
  * @param dutyUvwLeft Three-phase left duty cycle.
  * @param dutyUvwRight Three-phase right duty cycle.
  * @retval None.
  */
void R1SVPWM_Exec(R1SVPWM_Handle *r1svHandle, const AlbeAxis *uAlbe, UvwAxis *dutyUvwLeft, UvwAxis *dutyUvwRight)
{
    MCS_ASSERT_PARAM(r1svHandle != NULL);
    MCS_ASSERT_PARAM(uAlbe != NULL);
    MCS_ASSERT_PARAM(dutyUvwLeft != NULL);
    MCS_ASSERT_PARAM(dutyUvwRight != NULL);
    R1SVPWM_CALC_Handle r1SvCalc;
    float *samplePoint = r1svHandle->samplePoint;
    r1SvCalc.svCalc.vAlpha = uAlbe->alpha * r1svHandle->oneDivVoltPu;
    r1SvCalc.svCalc.vBeta = uAlbe->beta * r1svHandle->oneDivVoltPu;

    /* Sector Calculation */
    SVPWM_SectorCalc(&r1SvCalc.svCalc);
    /**
     * In control tick k, record the sector number of the voltage vector calculated in the k–1 tick.
     * For the next tick(k+1), it is the voltage vector to be applied.
     * Calculate the sector number of the voltage vector that actually acts on the (k+1)th tick.
     */
    r1svHandle->voltIndexLast = r1svHandle->voltIndex;
    r1svHandle->voltIndex = r1SvCalc.svCalc.sectorIndex;

    if (r1SvCalc.svCalc.sectorIndex < SVPWM_SECTOR_INDEX_MIN || r1SvCalc.svCalc.sectorIndex > SVPWM_SECTOR_INDEX_MAX) {
        dutyUvwLeft->u = 0.5f;
        dutyUvwLeft->v = 0.5f;
        dutyUvwLeft->w = 0.5f;
        dutyUvwRight->u = 0.5f;
        dutyUvwRight->v = 0.5f;
        dutyUvwRight->w = 0.5f;
        samplePoint[SOCA] = 0.5f;
        samplePoint[SOCB] = 0.5f;
        return;
    }
    /* Calculate three comparison values: max, medium, and min. */
    SVPWM_CompareValCalc(&r1SvCalc.svCalc);
    /* phase shift */
    R1SVPWM_PhaseShift(&r1SvCalc, r1svHandle->sampleWindow);

    /* Set sample point SOCA */
    samplePoint[SOCA] = r1SvCalc.compRight[SVPWM_COMP_VAL_MIN] + r1svHandle->samplePointShift;
    /* Set sample point SOCB */
    samplePoint[SOCB] = r1SvCalc.compRight[SVPWM_COMP_VAL_MID] + r1svHandle->samplePointShift;
    /*  Three-phase duty cycle data index based on sector convert */
    SVPWM_IndexConvert(&r1SvCalc.svCalc);

    dutyUvwLeft->u = r1SvCalc.compLeft[r1SvCalc.svCalc.indexU];
    dutyUvwLeft->v = r1SvCalc.compLeft[r1SvCalc.svCalc.indexV];
    dutyUvwLeft->w = r1SvCalc.compLeft[r1SvCalc.svCalc.indexW];
    dutyUvwRight->u = r1SvCalc.compRight[r1SvCalc.svCalc.indexU];
    dutyUvwRight->v = r1SvCalc.compRight[r1SvCalc.svCalc.indexV];
    dutyUvwRight->w = r1SvCalc.compRight[r1SvCalc.svCalc.indexW];
}

/**
  * @brief The stator current uvw is reconstructed from bus current according to the sector index
  *        of the output voltage vector.
  * @param sectorIndex Sector index of the output voltage vector.
  * @param currSocA Bus current at the sample point A.
  * @param currSocB Bus current at the sample point B.
  * @param curr The reconstructed stator current uvw.
  * @retval None.
  */
void R1CurrReconstruct(unsigned int sectorIndex, float currSocA, float currSocB, UvwAxis *curr)
{
    MCS_ASSERT_PARAM(curr != NULL);
    /* Reconstructed uvw three-phase current */
    float u;
    float v;
    float w;

    /*
     * The stator current uvw is reconstructed from bus current according to the sector index
     * of the output voltage vector.
     */
    switch (sectorIndex) {
        case SVPWM_ANGLE_0_TO_60_DEG: /* 0 ~ 60° Voltage vector sector */
            u = currSocA;
            w = -currSocB;
            v = -u - w;
            break;
        case SVPWM_ANGLE_60_TO_120_DEG: /* 60 ~ 120° Voltage vector sector */
            v = currSocA;
            w = -currSocB;
            u = -v - w;
            break;
        case SVPWM_ANGLE_120_TO_180_DEG: /* 120 ~ 180° Voltage vector sector */
            v = currSocA;
            u = -currSocB;
            w = -u - v;
            break;
        case SVPWM_ANGLE_180_TO_240_DEG: /* 180 ~ 240° Voltage vector sector */
            w = currSocA;
            u = -currSocB;
            v = -u - w;
            break;
        case SVPWM_ANGLE_240_TO_300_DEG: /* 240 ~ 300° Voltage vector sector */
            w = currSocA;
            v = -currSocB;
            u = -v - w;
            break;
        case SVPWM_ANGLE_300_TO_360_DEG: /* 300 ~ 360° Voltage vector sector */
            u = currSocA;
            v = -currSocB;
            w = -u - v;
            break;
        default:
            u = 0.0f;
            v = 0.0f;
            w = 0.0f;
            break;
    }
    curr->u = u;
    curr->v = v;
    curr->w = w;
}
