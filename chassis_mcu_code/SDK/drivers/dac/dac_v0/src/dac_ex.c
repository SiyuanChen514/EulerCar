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
  * @file    dac_ex.c
  * @author  MCU Driver Team
  * @brief   DAC module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the DAC.
  *           + DAC Sine mode setting.
  */
#include "dac_ex.h"

/**
  * @brief DAC sine wave mode configuration. The DAC automatically generates 8-bit sine wave digital signals,
           and outputs a sine wave sampling point every intervalValue clk_dac cycles. A complete sine wave period with
           100 sampling points.
  * @param dacHandle DAC handle.
  * @param intervalValue A sine wave sampling point is output every intervalValue clk_dac cycles.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_DAC_SetSineModeEx(DAC_Handle *dacHandle, unsigned short intervalValue)
{
    DAC_ASSERT_PARAM(dacHandle != NULL);
    DAC_ASSERT_PARAM(IsDACInstance(dacHandle->baseAddress));
    DAC_PARAM_CHECK_WITH_RET(IsDacSineIntervalNum(intervalValue), BASE_STATUS_ERROR);
    /* DAC sine mode configuration and setting. */
    DAC_CTRL_REG dacCtrlReg;
    dacCtrlReg.reg = dacHandle->baseAddress->DAC_CTRL.reg;  /* Sine mode and number setting. */
    dacCtrlReg.BIT.dac_test_num = intervalValue;
    dacCtrlReg.BIT.dac_test_en = BASE_CFG_ENABLE;
    dacHandle->baseAddress->DAC_CTRL.reg = dacCtrlReg.reg;
    return BASE_STATUS_OK;
}

/**
  * @brief Disable DAC sine mode.
  * @param dacHandle DAC handle.
  * @retval None.
  */
void HAL_DAC_DisableSineModeEx(DAC_Handle *dacHandle)
{
    DAC_ASSERT_PARAM(dacHandle != NULL);
    DAC_ASSERT_PARAM(IsDACInstance(dacHandle->baseAddress));
    dacHandle->baseAddress->DAC_CTRL.BIT.dac_test_en = BASE_CFG_DISABLE; /* Disable DAC sine mode. */
}