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
  * @file      mcs_curr_ctrl.c
  * @author    MCU Algorithm Team
  * @brief     This file provides function of motor current control.
  */

#include "typedefs.h"
#include "mcs_curr_ctrl.h"
#include "mcs_math_const.h"
#include "mcs_assert.h"
#include "mcs_curr_ff.h"


/**
  * @brief Initialzer of Current controller.
  * @param currHandle Current control handle.
  * @param pidTable Motor control handle.
  * @param mtrParam Motor parameters.
  * @param idqRef idqRef.
  * @param idqFbk idqFbk.
  * @param busVolt Bus voltage.
  * @param ts control period.
  * @retval None.
  */
void CURRCTRL_Init(CURRCTRL_Handle *currHandle, MOTOR_Param *mtrParam, DqAxis *idqRef, DqAxis *idqFbk,
                   const PI_Param dAxisPi, const PI_Param qAxisPi, float ts)
{
    MCS_ASSERT_PARAM(currHandle != NULL);
    MCS_ASSERT_PARAM(mtrParam != NULL);
    MCS_ASSERT_PARAM(idqRef != NULL);
    MCS_ASSERT_PARAM(idqFbk != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    /* Clear the control parameter. */
    CURRCTRL_Reset(currHandle);
    /* Current Pointer. */
    currHandle->idqRef = idqRef;
    currHandle->idqFbk = idqFbk;
    currHandle->mtrParam = mtrParam;
    /* The feedforward value is set to 0 by default. */
    currHandle->idqFf.d = 0.0f;
    currHandle->idqFf.q = 0.0f;
    
    /* Parameter initialization. */
    currHandle->ts = ts;
    currHandle->dAxisPi.ts = ts;
    currHandle->qAxisPi.ts = ts;

    currHandle->dAxisPi.kp = dAxisPi.kp;
    currHandle->dAxisPi.ki = dAxisPi.ki;
    currHandle->qAxisPi.kp = qAxisPi.kp;
    currHandle->qAxisPi.ki = qAxisPi.ki;
    /* output voltage limit. */
    currHandle->outLimit = qAxisPi.upperLim;
    currHandle->dAxisPi.upperLimit = dAxisPi.upperLim;
    currHandle->dAxisPi.lowerLimit = dAxisPi.lowerLim;
    currHandle->qAxisPi.upperLimit = qAxisPi.upperLim;
    currHandle->qAxisPi.lowerLimit = qAxisPi.lowerLim;
}

/**
  * @brief Reset the current control handle, fill with zero, NULL.
  * @param currHandle The current control handle.
  * @retval None.
  */
void CURRCTRL_Reset(CURRCTRL_Handle *currHandle)
{
    MCS_ASSERT_PARAM(currHandle != NULL);
    /* Reset the current control handle, fill with zero, NULL. */
    currHandle->idqRef    = NULL;
    currHandle->idqFbk   = NULL;
    currHandle->idqFf.d   = 0.0f;
    currHandle->idqFf.q   = 0.0f;
    currHandle->mtrParam   = NULL;
    currHandle->outLimit   = 0.0f;
    currHandle->ts = 0.0f;
    /* Reset Dq axis PID current control */
    PID_Reset(&currHandle->dAxisPi);
    PID_Reset(&currHandle->qAxisPi);
}

/**
  * @brief Clear historical values of current controller.
  * @param currHandle Current controller struct handle.
  * @retval None.
  */
void CURRCTRL_Clear(CURRCTRL_Handle *currHandle)
{
    MCS_ASSERT_PARAM(currHandle != NULL);
    PID_Clear(&currHandle->dAxisPi);
    PID_Clear(&currHandle->qAxisPi);
}


/**
  * @brief Simplified current controller PI calculation.
  * @param currHandle Current controller struct handle.
  * @param voltRef Dq-axis voltage reference which is the output of current controller.
  * @param spd speed (Hz).
  * @param ffEnable Feedforward compensation enable.
  * @retval None.
  */
void CURRCTRL_Exec(CURRCTRL_Handle *currHandle, DqAxis *vdqRef, float spd, int ffEnable)
{
    MCS_ASSERT_PARAM(currHandle != NULL);
    MCS_ASSERT_PARAM(vdqRef != NULL);
    DqAxis vdqFf;

    /* Calculate the current error of the dq axis. */
    currHandle->dAxisPi.error = currHandle->idqRef->d - currHandle->idqFbk->d;
    currHandle->qAxisPi.error = currHandle->idqRef->q - currHandle->idqFbk->q;
    CURRFF_Exec(&vdqFf, *currHandle->idqFbk, currHandle->mtrParam, spd, ffEnable);
    currHandle->dAxisPi.feedforward = vdqFf.d;
    currHandle->qAxisPi.feedforward = vdqFf.q;
    /* Calculation of the PI of the Dq axis current. */
    vdqRef->d = PI_Exec(&currHandle->dAxisPi);
    vdqRef->q = PI_Exec(&currHandle->qAxisPi);
}

/**
  * @brief Set ts of current controller.
  * @param currHandle Current controller struct handle.
  * @retval None.
  */
void CURRCTRL_SetTs(CURRCTRL_Handle *currHandle, float ts)
{
    MCS_ASSERT_PARAM(currHandle != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    currHandle->ts = ts;
    /* Set d and q axes pid sample time. */
    PID_SetTs(&currHandle->dAxisPi, ts);
    PID_SetTs(&currHandle->qAxisPi, ts);
}
