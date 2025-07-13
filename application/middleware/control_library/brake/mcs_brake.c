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
  * @file      mcs_brake.c
  * @author    MCU Algorithm Team
  * @brief     This file provides functions of brake module.
  */
#include "mcs_brake.h"
#include "mcs_math.h"
#include "mcs_assert.h"


/**
  * @brief Initialize brake handle.
  * @param brake: Pointer of Brake Handle.
  * @param brkParam: Brake parameters.
  * @retval None.
  */
void BRAKE_Init(BRAKE_Handle *brake, BRAKE_Param *brkParam)
{
    MCS_ASSERT_PARAM(brake != NULL);
    MCS_ASSERT_PARAM(brkParam != NULL);
    MCS_ASSERT_PARAM(brkParam->ts > 0.0f);
    /* Set brake parameter. */
    brake->brkParam = brkParam;
    brake->sampleShiftDuty = brkParam->sampleWinTime / brkParam->ts;
    /* Brake count. */
    brake->tickNum = (unsigned int)(brkParam->brkTime / brkParam->ts);
}

/**
  * @brief Clear historical values of brake handle.
  * @param brake: Pointer of Brake Handle.
  * @retval None.
  */
void BRAKE_Clear(BRAKE_Handle *brake)
{
    MCS_ASSERT_PARAM(brake != NULL);
    brake->brkDuty = 0.0f; /* brake duty */
    /* counter for calculating brake time */
    brake->tickCnt = 0;
    /* brake status */
    brake->brkFlg = 0;

    brake->brkFlg = BRAKE_WAIT;
}

/**
  * @brief Brake execution.
  * @param brake: Pointer of Brake Handle.
  * @param brkCurr: Sampling result of current during brake condition (A).
  * @retval None.
  */
void BRAKE_Exec(BRAKE_Handle *brake, float brkCurr)
{
    MCS_ASSERT_PARAM(brake != NULL);
    MCS_ASSERT_PARAM(brkCurr > 0.0f);
    float dutyMax = 1.0f;
    float curr = Abs(brkCurr);
    float maxCurr = brake->brkParam->maxBrkCurr;
    float sampleShiftDuty = brake->sampleShiftDuty;
    unsigned int tickNum = brake->tickNum;

    if (brake->brkFlg == BRAKE_FINISHED) {
        return;
    }

    /* Collect statistics on the total braking duration */
    brake->tickCnt++;
    if (brake->tickCnt >= tickNum) {
        /* Time to push out the brakes  */
        brake->brkFlg = BRAKE_FINISHED;
    }

    if (curr < (maxCurr * brake->brkParam->fastBrkCurrCoeff)) {
        brake->brkDuty += brake->brkParam->maxBrkDutyStep;
    } else if (curr < maxCurr) {
        brake->brkDuty += brake->brkParam->minBrkDutyStep;
    } else {
        brake->brkDuty -= brake->brkParam->maxBrkDutyStep;
    }

    /* Reserved sampling window */
    brake->brkDuty = Clamp(brake->brkDuty, dutyMax - 2.0f * sampleShiftDuty, 0.0f);
    if (curr <= (maxCurr * brake->brkParam->openLoopBrkCurrCoeff) &&
        brake->brkDuty >= dutyMax - 3.0f * sampleShiftDuty) {
        /* Because the duty cycle is too large to collect the current, the brake open loop control */
        brake->brkDuty += brake->brkParam->openLoopBrkDutyStep;
        brake->brkDuty = Clamp(brake->brkDuty, dutyMax, 0.0f);
    }
}
