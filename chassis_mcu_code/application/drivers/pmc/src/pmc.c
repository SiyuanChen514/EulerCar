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
  * @file    pmc.c
  * @author  MCU Driver Team.
  * @brief   ACMP HAL level module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the DAC and Comparator.
  *           + PMC's initialization and de-initialization functions.
  *           + Enter sleep, deepsleep mode functions.
  */

#include "pmc_ip.h"
#include "pmc.h"

#define WAKEUP_ENABLE_OFFSET    0x8
#define WAKE_ACT_MODE_REG_WIDTH 0x2

/**
  * @brief Setting deepsleep wakeup source.
  * @param pmcHandle: PMC handle.
  * @retval None.
  */
static void PMC_SetDeepSleepWakeupSrc(PMC_Handle *pmcHandle)
{
    PMC_ASSERT_PARAM(pmcHandle != NULL);
    PMC_ASSERT_PARAM(IsPMCInstance(pmcHandle->baseAddress));
    PMC_PARAM_CHECK_NO_RET(IsWakeupSrc(pmcHandle->wakeupSrc));
    if (pmcHandle->wakeupSrc == PMC_WAKEUP_NONE) { /* No wakeup source. */
        return;
    }

    if (pmcHandle->wakeupSrc == PMC_WAKEUP_CNT) {
        pmcHandle->baseAddress->CNT32K_WAKE_CYC = pmcHandle->wakeupTime; /* Set wakeup time */
        pmcHandle->baseAddress->WAKEUP_CTRL.BIT.cnt32k_wakeup_en = BASE_CFG_ENABLE; /* Enable wakeup from timer */
    } else {
        PMC_PARAM_CHECK_NO_RET(IsActiveMode(pmcHandle->wakeupActMode));
        pmcHandle->baseAddress->WAKEUP_CTRL.reg |= (pmcHandle->wakeupActMode) \
                                                    << (pmcHandle->wakeupSrc * WAKE_ACT_MODE_REG_WIDTH);
        pmcHandle->baseAddress->WAKEUP_CTRL.reg |= ((0x1 << pmcHandle->wakeupSrc) << WAKEUP_ENABLE_OFFSET);
    }
}

/**
  * @brief Init PVD function.
  * @param pmcHandle: PMC handle.
  * @retval None.
  */
static void PMC_PvdInit(PMC_Handle *pmcHandle)
{
    PMC_ASSERT_PARAM(pmcHandle != NULL);
    PMC_ASSERT_PARAM(IsPMCInstance(pmcHandle->baseAddress));
    PMC_PARAM_CHECK_NO_RET(pmcHandle->pvdThreshold <= PMC_PVD_THRED_LEVEL7);
    if (pmcHandle->pvdEnable == BASE_CFG_ENABLE) { /* if PVD function is enable */
        DCL_PMC_EnablePvd();
        DCL_PMC_SetPvdThreshold(pmcHandle->pvdThreshold); /* set PVD threhold voltage */
    } else {
        DCL_PMC_DisablePvd();
    }
}

/**
  * @brief PMC initialize interface.
  * @param handle: PMC handle.
  * @retval None.
  */
void HAL_PMC_Init(PMC_Handle *handle)
{
    PMC_ASSERT_PARAM(handle != NULL);
    PMC_ASSERT_PARAM(IsPMCInstance(handle->baseAddress));
    PMC_PvdInit(handle);
    PMC_SetDeepSleepWakeupSrc(handle);
}

/**
  * @brief PMC deinitialize interface.
  * @param handle: PMC handle.
  * @retval None.
  */
void HAL_PMC_DeInit(PMC_Handle *handle)
{
    PMC_ASSERT_PARAM(handle != NULL);
    PMC_ASSERT_PARAM(IsPMCInstance(handle->baseAddress));
    DCL_PMC_DisablePvd();
    handle->baseAddress->WAKEUP_CTRL.reg = BASE_CFG_DISABLE; /* Disable all wakeup source. */
    handle->userCallBack.PmcCallBack = NULL;                 /* Clean interrupt callback functions. */
}

/**
  * @brief Enter sleep interface.
  * @param None.
  * @retval None.
  */
void HAL_PMC_EnterSleepMode(void)
{
#if defined(USER_MODE_ENABLE) && (USER_MODE_ENABLE == 1)
    /* If user mode is supported, make sure to execute WFI
       commands in machine mode */
    static unsigned int priv = RISCV_U_MODE;
    RISCV_PRIV_MODE_SWITCH(priv);
    __asm("wfi");
    RISCV_PRIV_MODE_SWITCH(priv);
#else
    __asm("wfi");
#endif
}

/**
  * @brief Enter deep sleep interface.
  * @param handle: PMC handle.
  * @retval None.
  */
void HAL_PMC_EnterDeepSleepMode(PMC_Handle *handle)
{
    PMC_ASSERT_PARAM(handle != NULL);
    PMC_ASSERT_PARAM(IsPMCInstance(handle->baseAddress));
    handle->baseAddress->LOWPOWER_MODE.BIT.deepsleep_req = BASE_CFG_ENABLE;
}

/**
  * @brief Enter shutdown interface.
  * @param handle: PMC handle.
  * @retval None.
  */
void HAL_PMC_EnterShutdownMode(PMC_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* The 3061M does not support this function. */
}

/**
  * @brief Get wakeup source type.
  * @param handle: PMC handle.
  * @retval Lowpower type.
  */
PMC_LowpowerType HAL_PMC_GetWakeupType(PMC_Handle *handle)
{
    PMC_ASSERT_PARAM(handle != NULL);
    PMC_ASSERT_PARAM(IsPMCInstance(handle->baseAddress));

    PMC_LowpowerType wakeupMode;
    bool deepsleepFlag = BASE_CFG_UNSET; /* Set as default. */
    deepsleepFlag = handle->baseAddress->LOWPOWER_STATUS.BIT.starup_from_deepsleep;
    if (deepsleepFlag == BASE_CFG_SET) { /* If deepsleep flag is set */
        wakeupMode = PMC_LP_DEEPSLEEP;
    } else {
        wakeupMode = PMC_LP_NONE;
    }
    return wakeupMode;
}

/**
  * @brief Interrupt handler function.
  * @param handle PMC module handle.
  * @retval None.
  */
void HAL_PMC_IrqHandler(void *handle)
{
    PMC_ASSERT_PARAM(handle != NULL);
    PMC_Handle *pmcHandle = (PMC_Handle *)handle;
    PMC_ASSERT_PARAM(IsPMCInstance(pmcHandle->baseAddress));

    SYSCTRL1_RegStruct *sysCtrl1x = SYSCTRL1_BASE;
    if (sysCtrl1x->PVD_STATUS.BIT.pvd_toggle == 1) { /* PVD interrupt */
        if (pmcHandle->userCallBack.PmcCallBack != NULL) {
            pmcHandle->userCallBack.PmcCallBack(pmcHandle); /* execute user's callback */
        }
    }
}

/**
  * @brief Interrupt callback functions registration interface.
  * @param handle PMC module handle.
  * @param callbackID base callback id
  * @param pCallback Pointer for the user callback function.
  * @retval None.
  */
void HAL_PMC_RegisterCallback(PMC_Handle *handle, PMC_CallBackID callbackID, PMC_CallbackType pCallback)
{
    PMC_ASSERT_PARAM(handle != NULL);
    PMC_ASSERT_PARAM(IsPMCInstance(handle->baseAddress));
    PMC_ASSERT_PARAM(pCallback != NULL);
    BASE_FUNC_UNUSED(callbackID); /* This parameter is not used to prevent compilation errors. */
    handle->userCallBack.PmcCallBack = pCallback;
}