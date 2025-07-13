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
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
  * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  * @file      mcs_fw_ctrl.h
  * @author    MCU Algorithm Team
  * @brief     This file provides functions declaration of Flux-Weakening control.
  */
#ifndef McuMagicTag_MCS_FW_CTRL_H
#define McuMagicTag_MCS_FW_CTRL_H

#include "typedefs.h"
#include "mcs_typedef.h"

typedef struct {
    bool  enable;
    float udcThreshPer;
    float ts;
    float idSlope;
    float idRef;    /* reference instruction value. */
    float idMaxAmp; /* Maximum id ingested */

    float currMax;  /* maximum phase current (A) */
    float currMaxSquare; /* square of maximum current. */
    float idDemag;  /* demagnetizing d-axis current (A) */
} FW_Handle;

void FW_Init(FW_Handle *fw, float ts, bool enable, float currMax, float idDemag, float thr, float slope);

void FW_Exec(FW_Handle *fw, DqAxis udqRef, float udc, DqAxis *idqRefRaw);

void FW_SetTs(FW_Handle *fw, float ts);
#endif