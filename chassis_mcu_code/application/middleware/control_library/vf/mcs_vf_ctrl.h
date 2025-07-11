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
  * @file      mcs_vf_ctrl.h
  * @author    MCU Algorithm Team
  * @brief     This file provides functions declaration of v/f control.
  *
  */

#ifndef McuMagicTag_MCS_VF_CTRL_H
#define McuMagicTag_MCS_VF_CTRL_H


#include "mcs_typedef.h"
#include "mcs_ramp_mgmt.h"


typedef struct {
    float spdCmd;      /**< Motor target speed frequency (Hz).  */
    float spdRef;      /**< Motor reference speed frequency (Hz).  */
    float vfAngle;       /**< Vf control angle.  */
    float ts;            /**< Control period.  */
    float spdThr[2];     /**< Minimum (spdThr[0]) and maximum(spdThr[1]) speed thresholds for ramp command. */
    float voltThr[2];    /**< Minimum (voltThr[0]) and maximum(voltThr[1]) voltage for thresholds ramp command. */
    float slope;         /**< Slope of the voltage-speed curve.  */
    DqAxis ratio;        /**< Proportion of dq-axis reference voltage. */
    DqAxis vdqRef;       /**< Dq-axis reference voltage. */
    RMG_Handle rmg;      /**< Ramp management structure */
} VF_Handle;


void VF_Init(VF_Handle *vf, const float *spdThr, const float *voltThr, float ts, float spdCmd, float spdSlope);
void VF_Exec(VF_Handle *vf, DqAxis *vdqRef);
void VF_Clear(VF_Handle *vf);
void VF_SetTs(VF_Handle *vf, float ts);
void VF_SetSpdSlope(VF_Handle *vf, float spdSlope);
void VF_SetDRatio(VF_Handle *vf, float dRatio);

#endif