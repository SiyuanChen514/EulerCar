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
  * @file      mcs_stall_det.h
  * @author    MCU Algorithm Team
  * @brief     This file contains motor stalling protection data struct and api declaration.
  */
 
/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_MCS_STALL_DET_H
#define McuMagicTag_MCS_STALL_DET_H

#include "typedefs.h"


typedef struct {
    float currAmpLimit; /**< Feedback current higher than this value triggers fault. (A). */
    float spdLimit;     /**< Feedback speed lower than this value triggers fault (Hz). */
    float timeLimit;    /**< The threshold time that current and speed feedback over ranges (s). */
    float timer;        /**< Timer to get speed and current over range time.  */
    float ts;           /**< Ctrl period (s). */
} STD_Handle;

void STD_Init(STD_Handle *stall, float currLimit, float spdLimit, float timeLimit, float ts);

bool STD_Exec_ByCurrSpd(STD_Handle *stall, float spdFbk, float currAmp);

void STD_Clear(STD_Handle *stall);

#endif