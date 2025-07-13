/**
  * Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      mcs_posctrl.c
  * @author    MCU Algorithm Team
  * @brief     This file provides function of motor position control.
  */

#include "typedefs.h"
#include "mcs_math_const.h"
#include "mcs_math.h"
#include "mcs_pos_ctrl.h"

/**
  * @brief Reset the position controller, fill with zero, NULL.
  * @param posHandle position controller struct handle.
  * @retval None.
  */
void POSCTRL_Clear(POSCTRL_Handle *posHandle)
{
    /* PID controller history values clear */
    posHandle->posPi.error = 0.0f;
    posHandle->posPi.feedforward = 0.0f;
    posHandle->posPi.differ = 0.0f;
    posHandle->posPi.integral = 0.0f;
    posHandle->posPi.saturation = 0.0f;

    posHandle->posTarget = 0.0f;
    posHandle->posErr = 0.0f;
}

/**
 * @brief Position control initialization function.
 * @param posHandle position controller struct handle.
 * @param ts Control period.
 */
void POSCTRL_Init(POSCTRL_Handle *posHandle, const PID_Param *piCtrlTable, float ts)
{
    POSCTRL_Clear(posHandle);
    posHandle->ts = ts;
    
    /* position PID controller initialization */
    posHandle->posPi.ts = posHandle->ts;
    posHandle->posPi.kp = piCtrlTable->kp;
    posHandle->posPi.ki = piCtrlTable->ki;
    posHandle->posPi.kd = piCtrlTable->kd;
    posHandle->posPi.ns = piCtrlTable->ns;
    posHandle->posPi.ka = 1.0f / posHandle->posPi.kp;
    posHandle->posPi.upperLimit = piCtrlTable->upperLim;
    posHandle->posPi.lowerLimit = piCtrlTable->lowerLim;

    /* continuous mode: ramp controller initialization */
    RMG_Init(&posHandle->posRmg, posHandle->ts, posHandle->posRmg.slope * DOUBLE_PI);
    posHandle->posRmg.ts = posHandle->ts;
    
    /* position feedback history values clear */
    posHandle->angFbkLoop = 0;
    posHandle->angFbkPrev = 0.0f;

    /* position control mode configuration */
    posHandle->mode = POSCTRL_MODE_CONTINUOUS;
}

/**
 * @brief Position control mode settings.
 * @param posHandle position controller struct handle.
 * @param mode control mode.
 */
void POSCTRL_ModeSelect(POSCTRL_Handle *posHandle, POSCTRL_Mode mode)
{
    posHandle->mode = mode;
}

/**
  * @brief Set position change rate.
  * @param posHandle Position controller struct handle.
  * @param slope position change rate (Hz).
  * @retval None.
  */
void POSCTRL_SetSlope(POSCTRL_Handle *posHandle, float slope)
{
    posHandle->posRmg.slope = slope;
    posHandle->posRmg.delta = posHandle->posRmg.ts * posHandle->posRmg.slope * DOUBLE_PI;
}

/**
 * @brief Position ring target position setting.
 * @param posHandle Position controller struct handle.
 * @param posTarget Target location.
 */
void POSCTRL_SetTarget(POSCTRL_Handle *posHandle, float posTarget)
{
    posHandle->posTarget = posTarget;
    posHandle->posTargetShadow = posTarget;
}

/**
 * @brief Absolute position calculation.
 * @param posHandle Position controller struct handle.
 * @param angFbk Angle feedback.
 * @return float, Position feedback.
 */
float POSCTRL_AngleExpand(POSCTRL_Handle *posHandle, float angFbk)
{
    float angFbkPrevFloat = posHandle->angFbkPrev;
    int   loopPrev = posHandle->angFbkLoop;
    int   loop;

    /* unify feedback angle to ±2*pi */
    angFbk = Mod(angFbk, DOUBLE_PI);
    /* unify feedback angle to 0 ~ 2*pi */
    if (angFbk < 0.0f) {
        angFbk += DOUBLE_PI;
    }

    /* check if angle rotates one cycle */
    if (angFbkPrevFloat > THREE_PI_DIV_TWO && angFbkPrevFloat <= DOUBLE_PI && angFbk < HALF_PI) {
        loop = loopPrev + 1;
    } else if (angFbk > THREE_PI_DIV_TWO && THREE_PI_DIV_TWO <= DOUBLE_PI && angFbkPrevFloat < HALF_PI) {
        loop = loopPrev - 1;
    } else {
        loop = loopPrev;
    }

    /* update prev value */
    posHandle->angFbkLoop = loop;
    posHandle->angFbkPrev = angFbk;

    /* update output value */
    posHandle->posFbk = angFbk + loop * DOUBLE_PI;

    return posHandle->posFbk;
}

/**
 * @brief Position ring PID execution function.
 * @param posHandle Position controller struct handle.
 * @param posErr position error.
 * @return float
 */
float POSCTRL_PidExec(POSCTRL_Handle *posHandle, float posErr)
{
    float spdRef;
    posHandle->posPi.error = posErr;
    spdRef = PID_Exec(&posHandle->posPi);
    return spdRef;
}

/**
 * @brief position loop execution function.
 * @param posHandle Position controller struct handle.
 * @param posFbk Position feedback.
 * @return float, Speed reference value.
 */
float POSCTRL_Exec(POSCTRL_Handle *posHandle, float spdTarget, float posFbk)
{
    float posRef, spdRef;
    posRef = RMG_Exec(&posHandle->posRmg, spdTarget);
    posHandle->posRef = posRef;
    spdRef = POSCTRL_PidExec(posHandle, posRef - posFbk);
    spdRef *= ONE_DIV_DOUBLE_PI; /* transfer spdRef from rad/s to Hz */
    posHandle->spdRef = spdRef;
    return spdRef;
}

/**
 * @brief Set position loop kp parameter function.
 * @param posHandle Position controller struct handle.
 * @param kp PID-kp paramter.
 */
void POSCTRL_SetKp(POSCTRL_Handle *posHandle, float kp)
{
    posHandle->posPi.kp = kp;
}

/**
 * @brief Set position loop ki parameter function.
 * @param posHandle Position controller struct handle.
 * @param ki PID-ki paramter.
 */
void POSCTRL_SetKi(POSCTRL_Handle *posHandle, float ki)
{
    posHandle->posPi.ki = ki;
}

/**
 * @brief Set position loop kd parameter function.
 * @param posHandle Position controller struct handle.
 * @param kd PID-kd paramter.
 */
void POSCTRL_SetKd(POSCTRL_Handle *posHandle, float kd)
{
    posHandle->posPi.kd = kd;
}

/**
 * @brief Set position loop Ns parameter function.
 * @param posHandle Position controller struct handle.
 * @param ns ns paramter.
 */
void POSCTRL_SetNs(POSCTRL_Handle *posHandle, float ns)
{
    posHandle->posPi.ns = ns;
}