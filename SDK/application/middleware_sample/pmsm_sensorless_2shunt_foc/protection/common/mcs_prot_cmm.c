/**
  * @copyright Copyright (c) 2023, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      mcs_prot_cmm.c
  * @author    MCU Algorithm Team
  * @brief     This file contains protection common api definition.
  */
 
/* Includes ------------------------------------------------------------------------------------ */
#include "mcs_prot_cmm.h"
#include "apt.h"
#include "mcs_assert.h"

/**
  * @brief Safty-pulse-off function execution to turn off all the power devices.
  * @param aptAddr Three-phase APT address pointer.
  * @retval None.
  */
void ProtSpo_Exec(APT_RegStruct **aptAddr)
{
    MCS_ASSERT_PARAM(aptAddr != NULL);
    /**< Turn off all the six power devices of the inverter. */
    for (unsigned int i = 0; i < MOTOR_PHASE_NUMBER; i++) {
        APT_RegStruct *aptx = (APT_RegStruct *)(aptAddr[i]);
        DCL_APT_EnableSwContPWMAction(aptx, APT_PWM_CHANNEL_A);
        DCL_APT_EnableSwContPWMAction(aptx, APT_PWM_CHANNEL_B);
        DCL_APT_ForcePWMOutputLow(aptx);
    }
}