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
  * @file      cmm.c
  * @author    MCU Driver Team
  * @brief     CMM module driver.
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the CMM.
  *             + Initialization and de-initialization functions.
  *             + Config the register of CMM.
  *             + Interrupt callback function and user registration function.
  */

/* Includes ------------------------------------------------------------------*/
#include "interrupt.h"
#include "cmm.h"

/**
  * @brief Perform initial configuration based on the handle.
  * @param handle CMM handle.
  * @retval @ref BASE_StatusType.
  */
BASE_StatusType HAL_CMM_Init(CMM_Handle *handle)
{
    /* param check */
    CMM_ASSERT_PARAM(handle != NULL);
    CMM_ASSERT_PARAM(IsCMMInstance(handle->baseAddress));
    CMM_PARAM_CHECK_WITH_RET(handle->targetClockSource < CMM_TARGET_CLK_MAX, BASE_STATUS_ERROR);
    CMM_PARAM_CHECK_WITH_RET(handle->targetFreqDivision < CMM_TARGET_FREQ_DIV_MAX, BASE_STATUS_ERROR);
    CMM_PARAM_CHECK_WITH_RET(handle->refClockSource < CMM_REF_CLK_MAX, BASE_STATUS_ERROR);
    CMM_PARAM_CHECK_WITH_RET(handle->refFreqDivision < CMM_REF_FREQ_DIV_MAX, BASE_STATUS_ERROR);
    CMM_PARAM_CHECK_WITH_RET(handle->interruptType < CMM_INT_MAX, BASE_STATUS_ERROR);
    /* init handle value into register */
    /* Init CMM target clock. */
    DCL_CMM_SetTargetClockSource(handle->baseAddress, handle->targetClockSource);
    DCL_CMM_SetTargetClockFreqDivision(handle->baseAddress, handle->targetFreqDivision);
    /* Init CMM reference clock. */
    DCL_CMM_SetRefClockSource(handle->baseAddress, handle->refClockSource);
    DCL_CMM_SetRefClockFreqDivision(handle->baseAddress, handle->refFreqDivision);
    /* Init CMM UpperBound and LowerBound. */
    DCL_CMM_SetCmmWindowUpperBound(handle->baseAddress, handle->upperBound);
    DCL_CMM_SetCmmWindowLowerBound(handle->baseAddress, handle->lowerBound);
    DCL_CMM_EnableInterrupt(handle->baseAddress, handle->interruptType);
    return BASE_STATUS_OK;
}

/**
  * @brief Deinitialize configurations based on the handle.
  * @param handle CMM handle.
  * @retval @ref BASE_StatusType.
  */
BASE_StatusType HAL_CMM_DeInit(CMM_Handle *handle)
{
    CMM_ASSERT_PARAM(handle != NULL);
    CMM_ASSERT_PARAM(IsCMMInstance(handle->baseAddress));
    /* Clear interrupt callback function. */
    handle->userCallBack.FreqErrorCallback = NULL;
    handle->userCallBack.CheckEndCallback = NULL;
    handle->userCallBack.CountOverflowCallback = NULL;
    /* Disables the specified type of interrupt. */
    DCL_CMM_DisableInterrupt(handle->baseAddress, handle->interruptType);
    return BASE_STATUS_OK;
}

/**
  * @brief Set this parameter based on the configuration item parameters.
  * @param handle CMM handle.
  * @param cfgType Configurable item. @ref CMM_CFG_TYPE.
  * @retval @ref BASE_StatusType.
  */
BASE_StatusType HAL_CMM_Config(CMM_Handle *handle, CMM_CFG_TYPE cfgType)
{
    /* param check */
    CMM_ASSERT_PARAM(handle != NULL);
    CMM_ASSERT_PARAM(IsCMMInstance(handle->baseAddress));
    CMM_PARAM_CHECK_WITH_RET(handle->targetClockSource < CMM_TARGET_CLK_MAX, BASE_STATUS_ERROR);
    CMM_PARAM_CHECK_WITH_RET(handle->targetFreqDivision < CMM_TARGET_FREQ_DIV_MAX, BASE_STATUS_ERROR);
    CMM_PARAM_CHECK_WITH_RET(handle->refClockSource < CMM_REF_CLK_MAX, BASE_STATUS_ERROR);
    CMM_PARAM_CHECK_WITH_RET(handle->refFreqDivision < CMM_REF_FREQ_DIV_MAX, BASE_STATUS_ERROR);
    CMM_PARAM_CHECK_WITH_RET(handle->interruptType < CMM_INT_MAX, BASE_STATUS_ERROR);
    /* config register value with different type of cmm member */
    switch (cfgType) {
        case CMM_CFG_UPPER_BOUND:
            DCL_CMM_SetCmmWindowUpperBound(handle->baseAddress, handle->upperBound);  /* upperBound value */
            break;
        case CMM_CFG_LOWER_BOUND:
            DCL_CMM_SetCmmWindowLowerBound(handle->baseAddress, handle->lowerBound);  /* lowerBound value */
            break;
        case CMM_CFG_TARGET_SOURCE:
            DCL_CMM_SetTargetClockSource(handle->baseAddress, handle->targetClockSource); /* target Clock Source */
            break;
        case CMM_CFG_TARGET_FREQ_DIV:
            /* target Freq Division */
            DCL_CMM_SetTargetClockFreqDivision(handle->baseAddress, handle->targetFreqDivision);
            break;
        case CMM_CFG_REF_SOURCE:
            DCL_CMM_SetRefClockSource(handle->baseAddress, handle->refClockSource); /* ref Clock Source */
            break;
        case CMM_CFG_REF_FREQ_DIV:
            DCL_CMM_SetRefClockFreqDivision(handle->baseAddress, handle->refFreqDivision); /* ref Freq Division */
            break;
        case CMM_CFG_INT_TYPE:
            DCL_CMM_EnableInterrupt(handle->baseAddress, handle->interruptType); /* interrupt Type */
            break;
        default:
            return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Reads the register configuration value to the handle.
  * @param handle CMM handle.
  * @retval None.
  */
void HAL_CMM_GetConfig(CMM_Handle *handle)
{
    /* param check */
    CMM_ASSERT_PARAM(handle != NULL);
    CMM_ASSERT_PARAM(IsCMMInstance(handle->baseAddress));
    /* Get config of cmm member from register */
    handle->upperBound = DCL_CMM_GetCmmWindowUpperBound(handle->baseAddress);
    handle->lowerBound = DCL_CMM_GetCmmWindowLowerBound(handle->baseAddress);
    handle->targetClockSource = DCL_CMM_GetTargetClockSource(handle->baseAddress);
    handle->targetFreqDivision = DCL_CMM_GetTargetClockFreqDivision(handle->baseAddress);
    handle->refClockSource = DCL_CMM_GetRefClockSource(handle->baseAddress);
    handle->refFreqDivision = DCL_CMM_GetRefClockFreqDivision(handle->baseAddress);
}

/**
  * @brief Start CMM Module.
  * @param handle CMM handle.
  * @retval None.
  */
void HAL_CMM_Start(CMM_Handle *handle)
{
    CMM_ASSERT_PARAM(handle != NULL);
    CMM_ASSERT_PARAM(IsCMMInstance(handle->baseAddress));
    DCL_CMM_Enable(handle->baseAddress);
}

/**
  * @brief Stop CMM Module.
  * @param handle CMM handle.
  * @retval None.
  */
void HAL_CMM_Stop(CMM_Handle *handle)
{
    CMM_ASSERT_PARAM(handle != NULL);
    CMM_ASSERT_PARAM(IsCMMInstance(handle->baseAddress));
    DCL_CMM_Disable(handle->baseAddress);
}

/**
  * @brief Registers the interrupt function to the specified interrupt type.
  * @param handle CMM handle.
  * @param type Specified interrupt type.
  * @param callback Interrupt callback function.
  * @retval @ref BASE_StatusType.
  */
BASE_StatusType HAL_CMM_RegisterCallback(CMM_Handle *handle, CMM_Interrupt_Type type, CMM_CallBackFuncType callback)
{
    CMM_ASSERT_PARAM(handle != NULL);
    CMM_ASSERT_PARAM(callback != NULL);
    CMM_ASSERT_PARAM(IsCMMInstance(handle->baseAddress));

    switch (type) {
        case CMM_INT_FREQ_ERR_MASK:    /* Frequence error interrupt. */
            handle->userCallBack.FreqErrorCallback = callback;
            break;
        case CMM_INT_CHECK_END_MASK:   /* Check end interrupt. */
            handle->userCallBack.CheckEndCallback = callback;
            break;
        case CMM_INT_COUNTER_OVERFLOW_MASK:    /* Counter overflow interrupt. */
            handle->userCallBack.CountOverflowCallback = callback;
            break;
        default:
            return BASE_STATUS_ERROR;
    }

    return BASE_STATUS_OK;
}

/**
  * @brief Interrupt service processing function.
  * @param handle CMM Handle.
  * @retval None.
  */
void HAL_CMM_IrqHandler(void *handle)
{
    CMM_ASSERT_PARAM(handle != NULL);
    CMM_Handle *cmmHandle = (CMM_Handle *)handle;
    CMM_ASSERT_PARAM(IsCMMInstance(cmmHandle->baseAddress));

    /* Frequence error interrupt. */
    if (cmmHandle->baseAddress->CMINTSTS.BIT.freq_err_int == 0x01) {
        cmmHandle->baseAddress->CMINTRAW.BIT.freq_err_raw = BASE_CFG_SET;
        if (cmmHandle->userCallBack.FreqErrorCallback) {
            cmmHandle->userCallBack.FreqErrorCallback(cmmHandle);
        }
    }

    /* Check end interrupt. */
    if (cmmHandle->baseAddress->CMINTSTS.BIT.chk_end_int == 0x01) {
        cmmHandle->baseAddress->CMINTRAW.BIT.chk_end_raw = BASE_CFG_SET;
        if (cmmHandle->userCallBack.CheckEndCallback) {
            cmmHandle->userCallBack.CheckEndCallback(cmmHandle);
        }
    }

    /* Counter overflow interrupt. */
    if (cmmHandle->baseAddress->CMINTSTS.BIT.cnt_ovf_int == 0x01) {
        cmmHandle->baseAddress->CMINTRAW.BIT.cnt_ovf_raw = BASE_CFG_SET;
        if (cmmHandle->userCallBack.CountOverflowCallback) {
            cmmHandle->userCallBack.CountOverflowCallback(cmmHandle);
        }
    }
}