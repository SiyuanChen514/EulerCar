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
  * @file      mcs_unbalance_det.h
  * @author    MCU Algorithm Team
  * @brief     This file contains three-phase imbalance protection data struct and api declaration.
  */
 
/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef   McuMagicTag_MCS_UNBALANCE_DET_H
#define   McuMagicTag_MCS_UNBALANCE_DET_H

#include "typedefs.h"
#include "mcs_typedef.h"

typedef struct {
    unsigned int detCnt;
    unsigned int detCntLimit;
    unsigned int integralCnt;
    unsigned int timeCnt;
    unsigned int startupCnt;
    bool startFlag;
    bool startFlagLast;
    bool zeroFlag;
    bool calFlag;
    float unbalDegree;
    float unbalDegreeLimit;
    float delta;
    float ts;
    float ia;
    float ib;
    float ic;
} UNBAL_Handle;


void UNBAL_Init(UNBAL_Handle *unbal, float currDelta, float timeThr, float unbalDegreeLim, float ts);

bool UNBAL_Det(UNBAL_Handle *unbal, UvwAxis *iuvwFbk, float unbalFltCoeff);

void UNBAL_Clear(UNBAL_Handle *unbal);
#endif