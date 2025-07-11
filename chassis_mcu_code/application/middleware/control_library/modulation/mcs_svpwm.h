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
  * @file      mcs_svpwm.h
  * @author    MCU Algorithm Team
  * @brief     This file provides functions declaration of Space-Vector Pulse-Width-Modulation(SVPWM) calculations.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_MCS_SVPWM_H
#define McuMagicTag_MCS_SVPWM_H

/* Includes ------------------------------------------------------------------*/
#include "mcs_typedef.h"

/** Voltage vector sector */
#define SVPWM_ANGLE_0_TO_60_DEG    3
#define SVPWM_ANGLE_60_TO_120_DEG  1
#define SVPWM_ANGLE_120_TO_180_DEG 5
#define SVPWM_ANGLE_180_TO_240_DEG 4
#define SVPWM_ANGLE_240_TO_300_DEG 6
#define SVPWM_ANGLE_300_TO_360_DEG 2

/** The U-V-W phase compare value's index of APT timers. */
#define SVPWM_COMP_VAL_MAX 2
#define SVPWM_COMP_VAL_MID 1
#define SVPWM_COMP_VAL_MIN 0
#define SVPWM_COMP_VAL_TOTAL 3

/** The three voltage level to compare, for sector index decision. */
#define SVPWM_VOLT_0     0
#define SVPWM_VOLT_1     1
#define SVPWM_VOLT_2     2
#define SVPWM_VOLT_TOTAL 3

/** Sector index calculate: N = A + 2B + 4C */
#define SVPWM_SECTOR_ADD_1 1
#define SVPWM_SECTOR_ADD_2 2
#define SVPWM_SECTOR_ADD_4 4

#define SVPWM_SECTOR_INDEX_MIN 1
#define SVPWM_SECTOR_INDEX_MAX 6

/**
  * @defgroup SVPWM_MODULE  SVPWM MODULE
  * @brief The Space-Vector Pulse-Width-Modulation(SVPWM) module.
  * @{
  */

/**
  * @defgroup SVPWM_STRUCT  SVPWM STRUCT
  * @brief The SVPWM module's data struct definition.
  * @{
  */

/* Typedef definitions -------------------------------------------------------*/
/**
  * @brief SVPWM struct members and parameters.
  */
typedef struct {
    float voltPu;       /**< Voltage per unit value. */
    float oneDivVoltPu; /**< Reciprocal of voltage unit value. */
} SVPWM_Handle;

/**
  * @brief Structure of temporary variables for SVPWM calculation.
  */
typedef struct {
    float vAlpha; /**< Voltage vector. */
    float vBeta; /**< Voltage vector. */
    float t1; /**< T1 are the action times of the sequential action vectors. */
    float t2; /**< T2 are the action times of the sequential action vectors. */
    unsigned short indexU; /**< U-phase duty cycle conversion index */
    unsigned short indexV; /**< V-phase duty cycle conversion index */
    unsigned short indexW; /**< W-phase duty cycle conversion index */
    unsigned int sectorIndex; /**< Sector index */
    float volt[SVPWM_VOLT_TOTAL]; /**< temporary voltage to calculate sector index */
    float comp[SVPWM_COMP_VAL_TOTAL]; /**< Duty cycle corresponding to the comparison value */
} SVPWM_CALC_Handle;

/**
  * @}
  */

/**
  * @defgroup SVPWM_API  SVPWM API
  * @brief The SVPWM module's API declaration.
  * @{
  */
void SVPWM_Init(SVPWM_Handle *svHandle, float voltPu);
void SVPWM_SectorCalc(SVPWM_CALC_Handle *svCalc);
void SVPWM_CompareValCalc(SVPWM_CALC_Handle *svCalc);
void SVPWM_IndexConvert(SVPWM_CALC_Handle *svCalc);
void SVPWM_Exec(const SVPWM_Handle *svHandle, const AlbeAxis *uAlbe, UvwAxis *dutyUvw);
/**
  * @}
  */

/**
  * @}
  */

#endif  /* McuMagicTag_MCS_SVPWM_H */
