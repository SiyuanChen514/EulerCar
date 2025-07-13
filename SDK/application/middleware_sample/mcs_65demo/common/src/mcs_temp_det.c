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
  * @file      mcs_temp_det.c
  * @author    MCU Algorithm Team
  * @brief     This file contains current protection data struct and api declaration.
  */


#include "mcs_temp_det.h"
#include "mcs_assert.h"

/**
  * @brief Initilization over temperature detection function.
  * @param otd Over temperature dectection handle.
  * @param tempThr temperature value threshold.
  * @param timeThr Over current time threshold (s).
  * @param ts Ctrl period (s).
  * @retval None.
  */
void OTD_Init(OTD_Handle *otd, float tempThr, float timeThr, float ts)
{
    MCS_ASSERT_PARAM(otd != NULL);
    MCS_ASSERT_PARAM(timeThr > 0.0f);
    MCS_ASSERT_PARAM(ts > 0.0f);
    /* Time threshold and current threshold. */
    otd->tempThr = tempThr;
    otd->timeThr = timeThr;
    
    otd->ts = ts;
    /* Detection time limit calculation. */
    otd->detCntLimit = otd->timeThr / otd->ts;
}

/**
  * @brief Over temperature detection.
  * @param otd Over temperature dectection handle.
  * @param tempAmp Temperature vlaue.
  * @retval Whether the motor is over current.
  */
bool OTD_Exec(OTD_Handle *otd, float tempAmp)
{
    MCS_ASSERT_PARAM(otd != NULL);
    /* Current threshold judgment. */
    if (tempAmp < otd->tempThr) {
        otd->detCnt = 0;
        return false;
    }
    /* Over temperature detection time judgment. */
    if (otd->detCnt < otd->detCntLimit) {
        otd->detCnt++;
        return false;
    }
    return true;
}

/**
  * @brief Clear history value.
  * @param otd Over temperature dectection handle.
  * @retval None.
  */
void OTD_Clear(OTD_Handle *otd)
{
    MCS_ASSERT_PARAM(otd != NULL);
    otd->detCnt = 0;
}