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

#ifndef McuMagicTag_MCS_BRAKE_H
#define McuMagicTag_MCS_BRAKE_H

/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief Brake Struct.
  */
typedef struct {
    float ts;                   /**< control period (s). */
    float brkTime;              /**< brake time (s). */
    float sampleWinTime;          /**< sample shift. */
    float maxBrkCurr;           /**< maximum brake current (A). */
    float minBrkDutyStep;       /**< small brake duty step, recommend value: 0.001f. */
    float maxBrkDutyStep;       /**< large brake duty step, recommend value: 0.005f. */
    float fastBrkCurrCoeff;     /**< current threshold coefficient for fast braking, recommend value: 0.5f. */
    float openLoopBrkDutyStep;  /**< open-loop brake duty step, recommend value: 0.0001f. */
    float openLoopBrkCurrCoeff; /**< current threshold coefficient for open-loop braking, recommend value: 0.2f. */
} BRAKE_Param;


typedef struct {
    float brkDuty;              /**< pwm duty ratio of lower switch during brake condition (0~1). */
    float sampleShiftDuty;      /**< phase shift duty of sample point for brake current (0~1). */
    unsigned int tickCnt;       /**< counter for calculating brake time. */
    unsigned int tickNum;       /**< count number corresponding to brake time. */
    unsigned char brkFlg;       /**< brake status. */
    BRAKE_Param *brkParam;
} BRAKE_Handle;

typedef enum {
    BRAKE_WAIT = 0,
    BRAKE_FINISHED
} BRAKE_Status;

void BRAKE_Init(BRAKE_Handle *brake, BRAKE_Param *brkParam);
void BRAKE_Clear(BRAKE_Handle *brake);
void BRAKE_Exec(BRAKE_Handle *brake, float brkCurr);

#endif
