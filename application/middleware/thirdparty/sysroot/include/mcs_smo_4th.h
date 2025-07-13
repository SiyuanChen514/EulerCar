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
  * @file      mcs_smo_4th.h
  * @author    MCU Algorithm Team
  * @brief     This file provides functions declaration of 4th order smo module.
  */
#ifndef McuMagicTag_MCS_SMO_4TH_H
#define McuMagicTag_MCS_SMO_4TH_H

#include "mcs_typedef.h"
#include "mcs_pll.h"
#include "mcs_filter.h"
#include "mcs_mtr_param.h"


typedef struct {
    /* Model parameters */
    float ld;
    float lq;
    float rs;
    float ts;
    float kd;
    float kq;
    float pllBdw;
    float fcLpf;     /**< The cut-off frequency of First-order LPF for speed (Hz). */
    float elecAngle;
    float spdEst;
    /* Internal variable */
    AlbeAxis ialbeEst;
    AlbeAxis ealbeEst;
    PLL_Handle pll;
    FOFLT_Handle spdFilter;
} SMO4TH_Handle;

/**
 * @brief SMO4TH_Param
 */
typedef struct {
    float kd;
    float kq;
    float pllBdw;
    float fcLpf;
} SMO4TH_Param;


void SMO4TH_Init(SMO4TH_Handle *smo4th, const SMO4TH_Param smo4thParam, const MOTOR_Param mtrParam, float ts);

void SMO4TH_Exec(SMO4TH_Handle *smo4th, const AlbeAxis *ialbeFbk, const AlbeAxis *valbeRef);

void SMO4TH_ParamUpdate(SMO4TH_Handle *smo4th, float kd, float kq, float pllBdw, float fc);

void SMO4TH_Clear(SMO4TH_Handle *smo4th);

void SMO4TH_SetTs(SMO4TH_Handle *smo4th, float ts);

#endif