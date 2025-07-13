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
  * @file      mcs_startup.c
  * @author    MCU Algorithm Team
  * @brief     This file provides transition method from startup stage to run stage。
  */

#include "mcs_startup.h"
#include "mcs_math.h"
#include "mcs_assert.h"

/**
  * @brief Init the startup control handle.
  * @param startHandle The startup coontrol handle.
  * @param spdBegin    The begin speed for transition process.
  * @param spdEnd      The end speed for transition process.
  * @retval None.
  */
void STARTUP_Init(STARTUP_Handle *startHandle, float spdBegin, float spdEnd)
{
    MCS_ASSERT_PARAM(startHandle != NULL);
    MCS_ASSERT_PARAM(spdBegin > 0.0f);
    MCS_ASSERT_PARAM(spdEnd > 0.0f);
    MCS_ASSERT_PARAM(spdBegin < spdEnd);
    startHandle->stage     = STARTUP_STAGE_CURR;
    startHandle->spdBegin  = spdBegin;
    startHandle->spdEnd    = spdEnd;
    /* current AMP = slope * control period */
    startHandle->regionInv = 1.0f / (startHandle->spdEnd - startHandle->spdBegin);
}

/**
  * @brief Clear hisitory value, assign the stage to current change.
  * @param startHandle The startup control handle.
  * @retval None.
  */
void STARTUP_Clear(STARTUP_Handle *startHandle)
{
    MCS_ASSERT_PARAM(startHandle != NULL);
    startHandle->stage = STARTUP_STAGE_CURR;
}

/**
  * @brief Calculate the reference current in the startup stage.
  * @param startHandle The startup control handle.
  * @param refHz The speed reference in the startup stage.
  * @return The current AMP.
  */
float STARTUP_CurrCal(const STARTUP_Handle *startHandle, float refHz)
{
    MCS_ASSERT_PARAM(startHandle != NULL);
    float out;
    float tmp;
    /* Calculate the reference current in the startup stage */
    tmp = startHandle->spdEnd - Abs(refHz);
    tmp = tmp * startHandle->regionInv;
    out = tmp * startHandle->initCurr;

    return out;
}
