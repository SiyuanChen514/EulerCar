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
  * @file      wwdg.c
  * @author    MCU Driver Team
  * @brief     WWDG module driver
  * @details   This file provides firmware functions to manage the following functionalities of the WWDG.
  *             + Initialization functions.
  *             + WWDG Set And Get Functions.
  *             + Interrupt Handler Functions.
  */

/* Includes ------------------------------------------------------------------*/
#include "interrupt.h"
#include "wwdg.h"

/* Macro definitions ---------------------------------------------------------*/
#define WWDG_LOAD_VALUE_LIMIT      65535
#define WWDG_WINDOW_VALUE_LIMIT      65535
static unsigned int WWDG_CalculateRegTimeout(WWDG_RegStruct *baseAddress, float timeValue, WWDG_TimeType timeType);

/**
  * @brief Initializing WWDG values
  * @param handle Value of @ref WWDG_handle.
  * @retval BASE_StatusType: OK, ERROR
  */
BASE_StatusType HAL_WWDG_Init(WWDG_Handle *handle)
{
    WWDG_ASSERT_PARAM(handle != NULL);
    WWDG_ASSERT_PARAM(IsWWDGInstance(handle->baseAddress));
    WWDG_PARAM_CHECK_WITH_RET(IsWwdgTimeType(handle->timeType), BASE_STATUS_ERROR);
    WWDG_PARAM_CHECK_WITH_RET(IsWwdgTimeValue(handle->baseAddress, handle->timeValue, handle->timeType),
        BASE_STATUS_ERROR);
    /* The frequency divide value cannot exceed 8192. */
    unsigned int freqDivVal = (handle->freqDivValue > WWDG_FREQ_DIV_MAX) ? WWDG_FREQ_DIV_8192 : handle->freqDivValue;
    DCL_WWDG_SetFreqDivValue(handle->baseAddress, freqDivVal);
    HAL_WWDG_SetTimeValue(handle, handle->timeValue, handle->timeType);
    /* Window mode enable */
    if (handle->baseAddress->WWDOG_CONTROL.BIT.window_mode_en == BASE_CFG_ENABLE) {
        unsigned int value =
            WWDG_CalculateRegTimeout(handle->baseAddress, handle->windowValue, handle->timeType);
        unsigned int freqDiv = DCL_WWDG_GetFreqDivValue(handle->baseAddress);
        value = (value / (1 << freqDiv));
        /* The upper limit of the window value is determined. */
        value = (value <= WWDG_WINDOW_VALUE_LIMIT) ? value : WWDG_WINDOW_VALUE_LIMIT;
        /* window value only could be set litter than load value */
        value = (value < handle->baseAddress->WWDOG_LOAD.BIT.wwdg_load) ? value :
                                                                          handle->baseAddress->WWDOG_LOAD.BIT.wwdg_load;
        DCL_WWDG_SetWindowValue(handle->baseAddress, value);
    }
    DCL_WWDG_EnableInterrupt(handle->baseAddress); /* enable interrupt */
    DCL_WWDG_EnableReset(handle->baseAddress); /* enable reset */
    return BASE_STATUS_OK;
}

/**
  * @brief Calculate Reg Timeout.
  * @param timeValue Value to be load to wwdg.
  * @param timeType Value of @ref WWDG_TimeType.
  * @retval unsigned int timeout Value.
  */
static unsigned int WWDG_CalculateRegTimeout(WWDG_RegStruct *baseAddress, float timeValue, WWDG_TimeType timeType)
{
    float clockFreq = (float)HAL_CRG_GetIpFreq((void *)baseAddress);
    unsigned int timeoutValue = 0x00000000U;
    switch (timeType) {
        case WWDG_TIME_UNIT_TICK: /* If the time type is tick, calculate the timeout value. */
            timeoutValue = (unsigned int)timeValue;
            break;
        case WWDG_TIME_UNIT_S: /* If the time type is s, calculate the timeout value. */
            timeoutValue = (unsigned int)(timeValue * clockFreq);
            break;
        case WWDG_TIME_UNIT_MS: /* If the time type is ms, calculate the timeout value. */
            timeoutValue = (unsigned int)(timeValue * clockFreq / FREQ_CONVERT_MS_UNIT);
            break;
        case WWDG_TIME_UNIT_US: /* If the time type is us, calculate the timeout value. */
            timeoutValue = (unsigned int)(timeValue * clockFreq / FREQ_CONVERT_US_UNIT);
            break;
        default:
            break;
    }
    return timeoutValue;
}

/**
  * @brief Setting the load value of the WWDG counter.
  * @param handle Value of @ref WWDG_handle.
  * @param timeValue time value of the WWDG counter.
  * @param timeType WWDG time type.
  * @retval None.
  */
void HAL_WWDG_SetTimeValue(WWDG_Handle *handle, unsigned int timeValue, WWDG_TimeType timeType)
{
    WWDG_ASSERT_PARAM(handle != NULL);
    WWDG_ASSERT_PARAM(IsWWDGInstance(handle->baseAddress));
    WWDG_PARAM_CHECK_NO_RET(IsWwdgTimeType(timeType));
    WWDG_PARAM_CHECK_NO_RET(IsWwdgTimeValue(handle->baseAddress, timeValue, timeType));
    /* handle->baseAddress only could be configed WWDG */
    unsigned int value = WWDG_CalculateRegTimeout(handle->baseAddress, timeValue, timeType);
    unsigned int freqDiv = DCL_WWDG_GetFreqDivValue(handle->baseAddress);
    value = (value / (1 << freqDiv));
    /* The upper limit of the loaded value is determined. */
    value = (value <= WWDG_LOAD_VALUE_LIMIT) ? value : WWDG_LOAD_VALUE_LIMIT;
    DCL_WWDG_SetLoadValue(handle->baseAddress, value);
}

/**
  * @brief refresh the WWDG counter.
  * @param handle Value of @ref WWDG_handle.
  * @retval None.
  */
void HAL_WWDG_Refresh(WWDG_Handle *handle)
{
    WWDG_ASSERT_PARAM(handle != NULL);
    WWDG_ASSERT_PARAM(IsWWDGInstance(handle->baseAddress));
    DCL_WWDG_Refresh(handle->baseAddress);
}

/**
  * @brief obtain the load value.
  * @param handle Value of @ref WWDG_handle.
  * @retval unsigned int time value.
  */
unsigned int HAL_WWDG_GetLoadValue(WWDG_Handle *handle)
{
    WWDG_ASSERT_PARAM(handle != NULL);
    WWDG_ASSERT_PARAM(IsWWDGInstance(handle->baseAddress));
    return DCL_WWDG_GetLoadValue(handle->baseAddress);
}

/**
  * @brief Getting the window value of the WWDG counter.
  * @param handle Value of @ref WWDG_handle.
  * @retval unsigned int the value of window reg value.
  */
unsigned int HAL_WWDG_GetWindowValue(WWDG_Handle *handle)
{
    WWDG_ASSERT_PARAM(handle != NULL);
    WWDG_ASSERT_PARAM(IsWWDGInstance(handle->baseAddress));
    /* handle->baseAddress only could be configed WWDG */
    return DCL_WWDG_GetWindowValue(handle->baseAddress);
}

/**
  * @brief Refresh the WWDG counter value.
  * @param handle Value of @ref WWDG_handle.
  * @retval unsigned int Counter value.
  */
unsigned int HAL_WWDG_GetCounterValue(WWDG_Handle *handle)
{
    WWDG_ASSERT_PARAM(handle != NULL);
    WWDG_ASSERT_PARAM(IsWWDGInstance(handle->baseAddress));

    float res = (float)handle->baseAddress->WWDOG_VALUE.BIT.wwdg_value;
    if (res >= 65535) { /* 65535 is WWDG maximum current count */
        return handle->timeValue;
    }
    unsigned int freq = HAL_CRG_GetIpFreq((void *)handle->baseAddress);
    /* check clockFreq not equal zero */
    if (freq == 0) {
        return 0;
    }
    unsigned int freqDiv = DCL_WWDG_GetFreqDivValue(handle->baseAddress);
    switch (handle->timeType) {
        case WWDG_TIME_UNIT_TICK:
            /* Number of tick currently calculated */
            res = res * (1 << freqDiv);
            break;
        case WWDG_TIME_UNIT_S:
            /* Number of seconds currently calculated */
            res = res * (1 << freqDiv) / freq;
            break;
        case WWDG_TIME_UNIT_MS:
            res = res * (1 << freqDiv) * FREQ_CONVERT_MS_UNIT / freq;
            break;
        case WWDG_TIME_UNIT_US:
            /* Number of microseconds currently calculated */
            res = res * (1 << freqDiv) * FREQ_CONVERT_US_UNIT / freq;
            break;
        default:
            break;
    }
    return (unsigned int)res;
}

/**
  * @brief Start the WWDG count.
  * @param handle Value of @ref WWDG_handle.
  * @retval None.
  */
void HAL_WWDG_Start(WWDG_Handle *handle)
{
    WWDG_ASSERT_PARAM(handle != NULL);
    WWDG_ASSERT_PARAM(IsWWDGInstance(handle->baseAddress));
    DCL_WWDG_Start(handle->baseAddress);
}

/**
  * @brief Stop the WWDG count.
  * @param handle Value of @ref WWDG_handle.
  * @retval None.
  */
void HAL_WWDG_Stop(WWDG_Handle *handle)
{
    WWDG_ASSERT_PARAM(handle != NULL);
    WWDG_ASSERT_PARAM(IsWWDGInstance(handle->baseAddress));
    DCL_WWDG_Stop(handle->baseAddress);
}

/**
  * @brief   Register WWDG interrupt callback.
  * @param   handle Value of @ref WWDG_handle.
  * @param   callBackFunc Value of @ref WWDG_CallbackType.
  * @retval  None
  */
void HAL_WWDG_RegisterCallback(WWDG_Handle *handle, WWDG_CallbackType callBackFunc)
{
    WWDG_ASSERT_PARAM(handle != NULL);
    WWDG_ASSERT_PARAM(IsWWDGInstance(handle->baseAddress));
    if (callBackFunc != NULL) {
        /* Invoke the callback function. */
        handle->userCallBack.CallbackFunc = callBackFunc;
    }
}

/**
  * @brief Interrupt handler processing function.
  * @param handle WWDG_Handle.
  * @retval None.
  */
void HAL_WWDG_IrqHandler(void *handle)
{
    WWDG_Handle *wwdgHandle = (WWDG_Handle *)handle;
    WWDG_ASSERT_PARAM(wwdgHandle != NULL);
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgHandle->baseAddress));

    if (wwdgHandle->baseAddress->WWDOG_MIS.BIT.wwdogmis == 0x01) { /* Interrupt flag is set, fed dog in callback */
        if (wwdgHandle->userCallBack.CallbackFunc) {
            wwdgHandle->userCallBack.CallbackFunc(wwdgHandle);
        }
    }
}
