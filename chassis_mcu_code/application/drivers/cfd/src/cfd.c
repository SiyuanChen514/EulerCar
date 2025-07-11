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
  * @file      cfd.c
  * @author    MCU Driver Team
  * @brief     CFD module driver.
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the CFD.
  *             + Initialization and de-initialization functions.
  *             + Config the register of cfd.
  *             + Interrupt callback function and user registration function.
  */

/* Includes ------------------------------------------------------------------*/
#include "interrupt.h"
#include "cfd.h"

/**
  * @brief Perform initial configuration based on the handle.
  * @param handle CFD handle.
  * @retval @ref BASE_StatusType.
  */
BASE_StatusType HAL_CFD_Init(CFD_Handle *handle)
{
    CFD_ASSERT_PARAM(handle != NULL);
    CFD_ASSERT_PARAM(IsCFDInstance(handle->baseAddress));
    DCL_CFD_SetWindowUpperBound(handle->baseAddress, handle->upperBound);
    DCL_CFD_EnableInterrupt(handle->baseAddress, handle->interruptType);
    /* Set CFD clock. */
    if (handle->baseAddress->CMWDOH.BIT.cmwdoh == 0xFFFF) {
        DCL_CFD_SetCfdClock(handle->baseAddress);
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Deinitialize configurations based on the handle.
  * @param handle CFD handle.
  * @retval @ref BASE_StatusType.
  */
BASE_StatusType HAL_CFD_DeInit(CFD_Handle *handle)
{
    CFD_ASSERT_PARAM(handle != NULL);
    CFD_ASSERT_PARAM(IsCFDInstance(handle->baseAddress));
    /* Clear interrupt callback function. */
    handle->userCallBack.PllClockStopCallback = NULL;
    handle->userCallBack.CheckEndCallback = NULL;
    /* Clear register value. */
    DCL_CFD_DisableInterrupt(handle->baseAddress, BASE_CFG_DISABLE);
    return BASE_STATUS_OK;
}

/**
  * @brief Set this parameter based on the configuration item parameters.
  * @param handle CFD handle.
  * @param cfgType Configurable item. @ref CFD_CFG_TYPE.
  * @retval @ref BASE_StatusType.
  */
BASE_StatusType HAL_CFD_Config(CFD_Handle *handle, CFD_CFG_TYPE cfgType)
{
    CFD_ASSERT_PARAM(handle != NULL);
    CFD_ASSERT_PARAM(IsCFDInstance(handle->baseAddress));
    /* CFD config type. */
    switch (cfgType) {
        case CFD_CFG_UPPER_BOUND:    /* Config upperbound. */
            DCL_CFD_SetWindowUpperBound(handle->baseAddress, handle->upperBound);
            break;
        case CFD_CFG_INT_TYPE:       /* Config interrupt type. */
            DCL_CFD_EnableInterrupt(handle->baseAddress, handle->interruptType);
            break;
        default:
            return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Reads the register configuration value to the handle.
  * @param handle CFD handle.
  * @retval None.
  */
void HAL_CFD_GetConfig(CFD_Handle *handle)
{
    CFD_ASSERT_PARAM(handle != NULL);
    CFD_ASSERT_PARAM(IsCFDInstance(handle->baseAddress));
    handle->upperBound = DCL_CFD_GetWindowUpperBound(handle->baseAddress);
    handle->interruptType = DCL_CFD_GetInterruptType(handle->baseAddress);
}

/**
  * @brief Start CFD Module.
  * @param handle CFD handle.
  * @retval None.
  */
void HAL_CFD_Start(CFD_Handle *handle)
{
    CFD_ASSERT_PARAM(handle != NULL);
    CFD_ASSERT_PARAM(IsCFDInstance(handle->baseAddress));
    DCL_CFD_Enable(handle->baseAddress);
}

/**
  * @brief Stop CFD Module.
  * @param handle CFD handle.
  * @retval None.
  */
void HAL_CFD_Stop(CFD_Handle *handle)
{
    CFD_ASSERT_PARAM(handle != NULL);
    CFD_ASSERT_PARAM(IsCFDInstance(handle->baseAddress));
    DCL_CFD_Disable(handle->baseAddress);
}

/**
  * @brief Registers the interrupt function to the specified interrupt type.
  * @param handle CFD handle.
  * @param type Specified interrupt type.
  * @param callback Interrupt callback function.
  * @retval @ref BASE_StatusType.
  */
BASE_StatusType HAL_CFD_RegisterCallback(CFD_Handle *handle, CFD_Interrupt_Type type, CFD_CallBackFuncType callback)
{
    CFD_ASSERT_PARAM(handle != NULL);
    CFD_ASSERT_PARAM(callback != NULL);
    CFD_ASSERT_PARAM(IsCFDInstance(handle->baseAddress));
    /* Interrupt type. */
    switch (type) {
        case CFD_INT_PLL_REF_CLOCK_STOP_MASK:    /* Clock stop interrupt. */
            handle->userCallBack.PllClockStopCallback = callback;
            break;
        case CFD_INT_CHECK_END_MASK:             /* Check end interrupt. */
            handle->userCallBack.CheckEndCallback = callback;
            break;
        default:
            return BASE_STATUS_ERROR;
    }

    return BASE_STATUS_OK;
}

/**
  * @brief Interrupt service processing function.
  * @param handle CFD Handle.
  * @retval None.
  */
void HAL_CFD_IrqHandler(void *handle)
{
    CFD_ASSERT_PARAM(handle != NULL);
    CFD_Handle *cfdHandle = (CFD_Handle *)handle;
    CFD_ASSERT_PARAM(IsCFDInstance(cfdHandle->baseAddress));

    /* PLL clock stop interrupt. */
    if (cfdHandle->baseAddress->CMINTSTS.BIT.clk_fail_int == 0x01) {
        cfdHandle->baseAddress->CMINTRAW.BIT.clk_fail_raw = BASE_CFG_SET;
        if (cfdHandle->userCallBack.PllClockStopCallback) {
            cfdHandle->userCallBack.PllClockStopCallback(cfdHandle);
        }
    }

    /* Check end interrupt. */
    if (cfdHandle->baseAddress->CMINTSTS.BIT.chk_end_int == 0x01) {
        cfdHandle->baseAddress->CMINTRAW.BIT.chk_end_raw = BASE_CFG_SET;
        if (cfdHandle->userCallBack.CheckEndCallback) {
            cfdHandle->userCallBack.CheckEndCallback(cfdHandle);
        }
    }
}
