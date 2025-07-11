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
  * @file      timer_ex.c
  * @author    MCU Driver Team
  * @brief     TIMER module driver.
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the TIMER.
  *                + Implementation of extended functions of the timer module
  */

#include "timer_ex.h"

/**
  * @brief   Setting DMA request overflow interrupt.
  * @param   handle Timer Handle
  * @param   bool   enable or disable interrupt of DMA request overflow.
  * @retval  None
  */
void HAL_TIMER_DMARequestOverFlowEx(TIMER_Handle *handle, bool overFlowSet)
{
    TIMER_ASSERT_PARAM(handle != NULL);
    TIMER_ASSERT_PARAM(IsTIMERInstance(handle->baseAddress));
    handle->baseAddress->TIMERx_CONTROL.BIT.dmaovintenable = overFlowSet;
    return;
}

/**
  * @brief   Timer Trigger ADC Set
  * @param   handle  Timer Handle
  * @param   enable  0: disable 1: enable
  * @retval  BASE_STATUS_OK  Success
  * @retval  BASE_STATUS_ERROR Parameter check fail
  */
BASE_StatusType HAL_TIMER_TriggerAdcEx(TIMER_Handle *handle, bool enable)
{
    TIMER_ASSERT_PARAM(handle != NULL);
    TIMER_ASSERT_PARAM(IsTIMERInstance(handle->baseAddress));
    
    handle->baseAddress->TIMERx_CONTROLB.BIT.socen = enable;

    return BASE_STATUS_OK;
}