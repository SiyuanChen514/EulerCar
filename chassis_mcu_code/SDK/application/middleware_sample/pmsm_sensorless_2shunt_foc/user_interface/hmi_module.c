/**
  * @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      hmi_module.c
  * @author    MCU Algorithm Team
  * @brief     This file provides functions declaration of human-machine interface.
  */

#include "hmi_module.h"
#include "mcs_assert.h"

/**
  * @brief HMI Initializatio.
  * @retval None.
  */
void HMI_Init(void)
{
    UartRecvInit();
}

/**
  * @brief HMI processing.
  * @param mtrCtrl  The motor control handle..
  * @retval None.
  */
void HMI_Process_Rx(MTRCTRL_Handle *mtrCtrl)
{
    /* Verify Parameters */
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    UartModuleProcess_Rx(mtrCtrl);
}

/**
  * @brief HMI processing.
  * @param mtrCtrl  The motor control handle..
  * @retval None.
  */
void HMI_Process_Tx(MTRCTRL_Handle *mtrCtrl)
{
    /* Verify Parameters */
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    UartModuleProcess_Tx(mtrCtrl);
}
