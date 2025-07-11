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
  * @file      mcs_svpwm.c
  * @author    MCU Algorithm Team
  * @brief     This file provides function of Space-Vector Pulse-Width-Modulation(SVPWM) calculations.
  */

#include "mcs_svpwm.h"
#include "mcs_math_const.h"
#include "mcs_assert.h"
#include "mcs_math.h"

/* Macro definitions ---------------------------------------------------------*/
/**
  * @brief Initialzer of SVPWM handle.
  * @param svHandle The SVPWM handle.
  * @param voltPu The per-unit voltage value.
  * @retval None.
  */
void SVPWM_Init(SVPWM_Handle *svHandle, float voltPu)
{
    MCS_ASSERT_PARAM(svHandle != NULL);
    MCS_ASSERT_PARAM(voltPu > 0.0f);
    svHandle->voltPu = voltPu;
    svHandle->oneDivVoltPu = 1.0f / voltPu;
}

/**
  * @brief Calculate svpwm sector.
  * @param svCalc The svpwm calc struct.
  * @retval None.
  */
void SVPWM_SectorCalc(SVPWM_CALC_Handle *svCalc)
{
    MCS_ASSERT_PARAM(svCalc != NULL);
    MCS_ASSERT_PARAM(svCalc->sectorIndex <= SVPWM_SECTOR_INDEX_MAX);
    /* The initial sector is 0. */
    svCalc->sectorIndex = 0;
    /* Three-level voltage calculation */
    svCalc->volt[SVPWM_VOLT_0] = svCalc->vBeta;
    svCalc->volt[SVPWM_VOLT_1] = SQRT3_DIV_TWO * svCalc->vAlpha - 0.5f * svCalc->vBeta;
    svCalc->volt[SVPWM_VOLT_2] = -SQRT3_DIV_TWO * svCalc->vAlpha - 0.5f * svCalc->vBeta;

    /* sector index calculate && calculate abs values (V) */
    if (svCalc->volt[SVPWM_VOLT_0] > 0.0f) {
        svCalc->sectorIndex += SVPWM_SECTOR_ADD_1;
    } else {
        svCalc->volt[SVPWM_VOLT_0] = -svCalc->volt[SVPWM_VOLT_0];
    }
    if (svCalc->volt[SVPWM_VOLT_1] > 0.0f) {
        svCalc->sectorIndex += SVPWM_SECTOR_ADD_2;
    } else {
        svCalc->volt[SVPWM_VOLT_1] = -svCalc->volt[SVPWM_VOLT_1];
    }
    if (svCalc->volt[SVPWM_VOLT_2] > 0.0f) {
        svCalc->sectorIndex += SVPWM_SECTOR_ADD_4;
    } else {
        svCalc->volt[SVPWM_VOLT_2] = -svCalc->volt[SVPWM_VOLT_2];
    }
}

/**
  * @brief Calculate three comparison values: max, medium, and min..
  * @param svCalc The svpwm calc struct.
  * @retval None.
  */
void SVPWM_CompareValCalc(SVPWM_CALC_Handle *svCalc)
{
    MCS_ASSERT_PARAM(svCalc != NULL);
    /* Calculate the action time of the two vectors based on the sector. */
    switch (svCalc->sectorIndex) {
        case SVPWM_ANGLE_0_TO_60_DEG: /* 0 ~ 60° Voltage vector sector */
            svCalc->t1 = svCalc->volt[SVPWM_VOLT_1];
            svCalc->t2 = svCalc->volt[SVPWM_VOLT_0];
            break;
        case SVPWM_ANGLE_60_TO_120_DEG: /* 60 ~ 120° Voltage vector sector */
            svCalc->t1 = svCalc->volt[SVPWM_VOLT_1];
            svCalc->t2 = svCalc->volt[SVPWM_VOLT_2];
            break;
        case SVPWM_ANGLE_120_TO_180_DEG: /* 120 ~ 180° Voltage vector sector */
            svCalc->t1 = svCalc->volt[SVPWM_VOLT_0];
            svCalc->t2 = svCalc->volt[SVPWM_VOLT_2];
            break;
        case SVPWM_ANGLE_180_TO_240_DEG: /* 180 ~ 240° Voltage vector sector */
            svCalc->t1 = svCalc->volt[SVPWM_VOLT_0];
            svCalc->t2 = svCalc->volt[SVPWM_VOLT_1];
            break;
        case SVPWM_ANGLE_240_TO_300_DEG: /* 240 ~ 300° Voltage vector sector */
            svCalc->t1 = svCalc->volt[SVPWM_VOLT_2];
            svCalc->t2 = svCalc->volt[SVPWM_VOLT_1];
            break;
        case SVPWM_ANGLE_300_TO_360_DEG: /* 300 ~ 360° Voltage vector sector */
            svCalc->t1 = svCalc->volt[SVPWM_VOLT_2];
            svCalc->t2 = svCalc->volt[SVPWM_VOLT_0];
            break;
        default:
            break;
    }
    
    /* The action time of two vectors is converted to three comparison values. */
    svCalc->comp[SVPWM_COMP_VAL_MIN] = (1.0f - svCalc->t1 - svCalc->t2) * 0.5f;
    svCalc->comp[SVPWM_COMP_VAL_MID] = svCalc->comp[SVPWM_COMP_VAL_MIN] + svCalc->t1;
    svCalc->comp[SVPWM_COMP_VAL_MAX] = svCalc->comp[SVPWM_COMP_VAL_MID] + svCalc->t2;
}

/**
  * @brief Three-phase duty cycle data index based on sector convert.
  * @param svCalc The svpwm calc struct.
  * @retval None.
  */
void SVPWM_IndexConvert(SVPWM_CALC_Handle *svCalc)
{
    MCS_ASSERT_PARAM(svCalc != NULL);
    /* Three-phase duty cycle data index based on sector convert */
    switch (svCalc->sectorIndex) {
        case SVPWM_ANGLE_0_TO_60_DEG: /* 0 ~ 60° Voltage vector sector */
            svCalc->indexU = SVPWM_COMP_VAL_MIN;
            svCalc->indexV = SVPWM_COMP_VAL_MID;
            svCalc->indexW = SVPWM_COMP_VAL_MAX;
            break;
        case SVPWM_ANGLE_60_TO_120_DEG: /* 60 ~ 120° Voltage vector sector */
            svCalc->indexU = SVPWM_COMP_VAL_MID;
            svCalc->indexV = SVPWM_COMP_VAL_MIN;
            svCalc->indexW = SVPWM_COMP_VAL_MAX;
            break;
        case SVPWM_ANGLE_120_TO_180_DEG: /* 120 ~ 180° Voltage vector sector */
            svCalc->indexU = SVPWM_COMP_VAL_MAX;
            svCalc->indexV = SVPWM_COMP_VAL_MIN;
            svCalc->indexW = SVPWM_COMP_VAL_MID;
            break;
        case SVPWM_ANGLE_180_TO_240_DEG: /* 180 ~ 240° Voltage vector sector */
            svCalc->indexU = SVPWM_COMP_VAL_MAX;
            svCalc->indexV = SVPWM_COMP_VAL_MID;
            svCalc->indexW = SVPWM_COMP_VAL_MIN;
            break;
        case SVPWM_ANGLE_240_TO_300_DEG: /* 240 ~ 300° Voltage vector sector */
            svCalc->indexU = SVPWM_COMP_VAL_MID;
            svCalc->indexV = SVPWM_COMP_VAL_MAX;
            svCalc->indexW = SVPWM_COMP_VAL_MIN;
            break;
        case SVPWM_ANGLE_300_TO_360_DEG: /* 300 ~ 360° Voltage vector sector */
            svCalc->indexU = SVPWM_COMP_VAL_MIN;
            svCalc->indexV = SVPWM_COMP_VAL_MAX;
            svCalc->indexW = SVPWM_COMP_VAL_MID;
            break;
        default:
            break;
    }
}

/**
  * @brief The duty cycles of PWM wave of three-phase upper switches are
  *        calculated in the two-phase stationary coordinate system (albe).
  * @param svHandle The SVPWM struct handle.
  * @param uAlbe    Input voltage vector.
  * @param dutyUvw  Three-phase A compare value.
  * @retval None.
  */
void SVPWM_Exec(const SVPWM_Handle *svHandle, const AlbeAxis *uAlbe, UvwAxis *dutyUvw)
{
    MCS_ASSERT_PARAM(svHandle != NULL);
    MCS_ASSERT_PARAM(uAlbe != NULL);
    MCS_ASSERT_PARAM(dutyUvw != NULL);
    SVPWM_CALC_Handle svCalc;

    /* Amplitude limited */
    float voltMax = 1.0f  / svHandle->oneDivVoltPu;
    float amp = Sqrt(uAlbe->alpha * uAlbe->alpha + uAlbe->beta * uAlbe->beta); /* Voltage amplitude. */
    AlbeAxis uAlbeLimited;
    float coeff;
    if (amp < 0.001f) {
        coeff = 1.0f;
    } else {
        coeff = voltMax / amp; /* Amplitude limit coefficient. */
    }
    if (amp > voltMax) {
        uAlbeLimited.alpha = uAlbe->alpha * coeff;
        uAlbeLimited.beta  = uAlbe->beta  * coeff;
    } else {
        uAlbeLimited.alpha = uAlbe->alpha;
        uAlbeLimited.beta  = uAlbe->beta;
    }
    svCalc.vAlpha = uAlbeLimited.alpha * svHandle->oneDivVoltPu;
    svCalc.vBeta  = uAlbeLimited.beta * svHandle->oneDivVoltPu;

    /* Voltage vector sector calculation */
    SVPWM_SectorCalc(&svCalc);
    /* Check whether the current sector is abnormal. */
    if (svCalc.sectorIndex < SVPWM_SECTOR_INDEX_MIN || svCalc.sectorIndex > SVPWM_SECTOR_INDEX_MAX) {
        dutyUvw->u = 0.5f;
        dutyUvw->v = 0.5f;
        dutyUvw->w = 0.5f;
        return;
    }
    /* Calculate three comparison values: max, medium, and min. */
    SVPWM_CompareValCalc(&svCalc);
    /*  Three-phase duty cycle data index based on sector convert */
    SVPWM_IndexConvert(&svCalc);
    /* Output UVW three-phase duty cycle */
    dutyUvw->u = svCalc.comp[svCalc.indexU];
    dutyUvw->v = svCalc.comp[svCalc.indexV];
    dutyUvw->w = svCalc.comp[svCalc.indexW];
}
