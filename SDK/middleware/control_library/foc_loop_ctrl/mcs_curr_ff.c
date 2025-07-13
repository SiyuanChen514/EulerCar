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
  * @file      mcs_curr_ff.c
  * @author    MCU Algorithm Team
  * @brief     This file provides current loop feedforward compensation declaration for motor control.
  */

#include "mcs_curr_ff.h"
#include "mcs_math_const.h"
#include "mcs_assert.h"

/**
  * @brief Current loop feedforward compensation execution function.
  * @param vdqFf DQ axis volt feedforward compensation value.
  * @param idqFbk DQ axis feedback current value.
  * @param param Motor parameters.
  * @param spd Speed (Hz).
  * @param enable Whether to enable feedforward compensation.
  * @retval None.
  */
void CURRFF_Exec(DqAxis *vdqFf, DqAxis idqFbk, MOTOR_Param *param, float spd, int enable)
{
    MCS_ASSERT_PARAM(vdqFf != NULL);
    MCS_ASSERT_PARAM(param != NULL);
    /* The unit is converted from Hz to rad. */
    float we = spd * DOUBLE_PI;
    if (enable) {
        /* Calculate the feedforward compensation value. */
        vdqFf->d = -param->mtrLq * we * idqFbk.q;
        vdqFf->q = we * (param->mtrLd * idqFbk.d + param->mtrPsif);
    } else {
        vdqFf->d = 0.0f;
        vdqFf->q = 0.0f;
    }
}