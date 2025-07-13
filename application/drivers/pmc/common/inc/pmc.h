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
  * @file    pmc.h
  * @author  MCU Driver Team.
  * @brief   PMC module driver.
  *          This file provides functions declaration of PMC.
  *           + PMC's initialization and de-initialization functions.
  *           + Interface declaration of enter sleep, deepsleep and shutdowm mode.
  *           + PMC's register callback function.
  */

#ifndef __McuMagicTag_PMC_H__
#define __McuMagicTag_PMC_H__
#include "pmc_ip.h"

/**
  * @defgroup PMC PMC
  * @brief PMC module.
  * @{
  */

/**
  * @defgroup PMC_Common PMC Common
  * @brief PMC common external module.
  * @{
  */


/**
  * @defgroup PMC_Common_Param PMC Common Parameters
  * @{
  */

/**
  * @brief Definition of callback function type
  */
typedef void (* PMC_CallbackType)(void *pmcHandle);

/**
  * @brief PMC Handle
  */
typedef struct _PMC_Handle {
    PMC_RegStruct           *baseAddress;     /**< Register base address. */
    PMC_LowpowerWakeupSrc    wakeupSrc;       /**< Wakeup source of deep sleep. */
    PMC_ActMode              wakeupActMode;   /**< Wakeup pin level mode of PMC module. */
    unsigned int             wakeupTime;      /**< Wakeup time of deep sleep. */
    bool                     pvdEnable;       /**< PVD function enable. */
    PMC_PvdThreshold         pvdThreshold;    /**< PVD threshold voltage level. */
    PMC_UserCallBack         userCallBack;    /**< User-defined callback function. */
    PMC_ExtendHandle         handleEx;        /**< Extend handle, configuring some special parameters. */
} PMC_Handle;

/**
  * @}
  */

/**
  * @defgroup PMC_API_Declaration PMC HAL API
  * @{
  */
void HAL_PMC_Init(PMC_Handle *handle);
void HAL_PMC_DeInit(PMC_Handle *handle);
void HAL_PMC_EnterSleepMode(void);
void HAL_PMC_EnterDeepSleepMode(PMC_Handle *handle);
void HAL_PMC_EnterShutdownMode(PMC_Handle *handle);
PMC_LowpowerType HAL_PMC_GetWakeupType(PMC_Handle *handle);
void HAL_PMC_RegisterCallback(PMC_Handle *handle, PMC_CallBackID callbackID, PMC_CallbackType pCallback);
void HAL_PMC_IrqHandler(void *handle);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif