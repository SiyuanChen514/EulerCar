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
 * @file    dac.c
 * @author  MCU Driver Team.
 * @brief   DAC HAL level module driver.
 *          This file provides firmware functions to manage the following
 *          functionalities of the DAC and Comparator.
 *           + DAC's Initialization and de-initialization functions
 *           + Set DAC value function
 */
#include "dac.h"
#include "assert.h"

/**
  * @brief Set DAC value
  * @param dacHandle: DAC handle.
  * @param value: DAC value.
  * @retval None.
  */
void HAL_DAC_SetValue(DAC_Handle *dacHandle, unsigned int value)
{
    DAC_ASSERT_PARAM(dacHandle != NULL);
    DAC_ASSERT_PARAM(IsDACInstance(dacHandle->baseAddress));
    DAC_PARAM_CHECK_NO_RET(value <= DAC_MAX_OUT_VALUE);
    /* Change the conversion value of the DAC. */
    dacHandle->baseAddress->DAC_VALUE.BIT.dac_value = value;
}

/**
  * @brief DAC HAL Init
  * @param dacHandle: DAC handle.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_DAC_Init(DAC_Handle *dacHandle)
{
    DAC_ASSERT_PARAM(dacHandle != NULL);
    DAC_ASSERT_PARAM(IsDACInstance(dacHandle->baseAddress));
    DAC_PARAM_CHECK_WITH_RET(IsDacSineIntervalNum(dacHandle->handleEx.sineCountValue), BASE_STATUS_ERROR);
    /* Conversion value of the DAC. */
    dacHandle->baseAddress->DAC_VALUE.BIT.dac_value = dacHandle->dacValue;
    /* DAC sine mode configuration and setting. */
    DAC_CTRL_REG dacCtrlReg;
    dacCtrlReg.reg = dacHandle->baseAddress->DAC_CTRL.reg;
    dacCtrlReg.BIT.dac_test_num = dacHandle->handleEx.sineCountValue;
    dacHandle->baseAddress->DAC_CTRL.reg = dacCtrlReg.reg;
    dacHandle->baseAddress->DAC_CTRL.BIT.dac_test_en = dacHandle->handleEx.sineMode;
    /* Turn on the DAC. */
    dacHandle->baseAddress->DAC_CTRL.BIT.dac_en = BASE_CFG_ENABLE;
    return BASE_STATUS_OK;
}

/**
  * @brief DAC HAL DeInit
  * @param dacHandle: DAC handle.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_DAC_DeInit(DAC_Handle *dacHandle)
{
    DAC_ASSERT_PARAM(dacHandle != NULL);
    DAC_ASSERT_PARAM(IsDACInstance(dacHandle->baseAddress));
    dacHandle->baseAddress->DAC_CTRL.reg = BASE_CFG_DISABLE;  /* Disable DAC, clears the count value. */
    dacHandle->baseAddress->DAC_VALUE.reg = BASE_CFG_DISABLE; /* Clear DAC value. */
    return BASE_STATUS_OK;
}
