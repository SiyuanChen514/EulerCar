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
  * @file      mcs_mtr_param.c
  * @author    MCU Algorithm Team
  * @brief     This file provides data structure define of motor parameters.
  */

#include "mcs_mtr_param.h"


/**
  * @brief Initialzer of motor parameters.
  * @param handle Motor parameters handle.
  * @param motorTable Motor parameters table.
  * @retval None.
  */
 void MtrParamInit(MOTOR_Param *handle, const MOTOR_Param motorTable)
{
    MCS_ASSERT_PARAM(handle != NULL);
    /* Initialzer of motor parameters */
    handle->mtrRs = motorTable.mtrRs; /* resistor of stator */
    handle->mtrLd = motorTable.mtrLd; /* inductance of D-axis */
    handle->mtrLq = motorTable.mtrLq; /* inductance of Q-axis */
    /* Average inductance, mtrLs = (mtrLd + mtrLq) * 0.5f */
    handle->mtrLs = (motorTable.mtrLd + motorTable.mtrLq) * 0.5f;
    handle->mtrPsif = motorTable.mtrPsif; /* permanent magnet flux */
    handle->mtrNp = motorTable.mtrNp; /* numbers of pole pairs */
    handle->mtrJ = motorTable.mtrJ;  /* rotor inertia */
    handle->maxElecSpd = motorTable.maxElecSpd; /* max elec speed */
    handle->maxCurr = motorTable.maxCurr; /* max current */
    handle->maxTrq = motorTable.maxTrq; /* max torque */
}