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
  * @file      mcs_openphs_det.h
  * @author    MCU Algorithm Team
  * @brief     This file contains self-check open phase fault detection function data struct and api declaration.
  */
 
/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_MCS_OPENPHS_DET_H
#define McuMagicTag_MCS_OPENPHS_DET_H

#include "typedefs.h"

typedef enum {
    OPD_U_V = 0,
    OPD_V_U,
    OPD_V_W,
    OPD_W_V,
    OPD_W_U,
    OPD_U_W,
    OPD_END
} OPD_Index;

typedef struct {
    float minOpenPhsCurr;  /* Minimum current for open-phase detection (A). */
    bool isOpenPhsU;
    bool isOpenPhsV;
    bool isOpenPhsW;
} OPD_Handle;

void OPD_Init(OPD_Handle *opd, float minOpenPhsCurr);

bool OPD_Exec(OPD_Handle *opd, const float *iuvw);

void OPD_Clear(OPD_Handle *opd);
#endif