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
  * @file      mcs_mtr_param.h
  * @author    MCU Algorithm Team
  * @brief     This file provides data structure define of motor parameters.
  */

/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_MTR_PARAM_H
#define McuMagicTag_MCS_MTR_PARAM_H

/* Typedef definitions ------------------------------------------------------------------------- */
#include "mcs_assert.h"
/**
  * @defgroup MOTOR_PARAMETER  MOTOR PARAMETER
  * @brief The motor parameter definitions.
  * @{
  */
/**
  * @brief motor parameters data structure
  */
typedef struct {
    unsigned short  mtrNp;      /**< Numbers of pole pairs. */
    float           mtrRs;      /**< Resistor of stator, Ohm. */
    float           mtrLd;      /**< Inductance of D-axis, H. */
    float           mtrLq;      /**< Inductance of Q-axis, H. */
    float           mtrLs;      /**< Average inductance, H. */
    float           mtrPsif;    /**< Permanent magnet flux, Wb. */
    float           mtrJ;       /**< Rotor inertia, Kg*m2. */
    float           maxElecSpd; /**< Max elec speed, Hz. */
    float           maxCurr;    /**< Max current, A. */
    float           maxTrq;     /**< Max torque, Nm. */
    /* Encoder parameters */
    unsigned int mtrPPMR;    /**< pulse per mechanical round */
    unsigned int zShift;     /**< pulse Z shift */
} MOTOR_Param;


void MtrParamInit(MOTOR_Param *handle, const MOTOR_Param motorTable);
/**
  * @}
  */

#endif
