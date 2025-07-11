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
  * @file crg.h
  * @author   MCU Driver Team
  * @brief    CRG module driver
  * @details  This file provides firmware CRG Handle Structure and functions
  *           prototypes to manage the following functionalities of the CRG.
  *             + Config CRG
  *             + Config IP Clock
  *             + Get the Config of CRG
  *             + Get the frequency of cpu and IP
  */
#ifndef McuMagicTag_CRG_H
#define McuMagicTag_CRG_H

/* Includes ------------------------------------------------------------------*/
#include "crg_ip.h"

/* Macro definitions ---------------------------------------------------------*/

/**
 * @defgroup CRG CRG
 * @brief CRG module.
 * @{
 */

/**
 * @defgroup CRG_Common CRG Common
 * @brief CRG common external module.
 * @{
 */

/**
 * @defgroup CRG_Handle_Definition CRG Handle Definition
 * @{
 */
/**
 * @brief  Typedef callback function of CRG
 */
typedef void (*CRG_CallBackFunc)(void *param);

/**
  * @brief CRG Handle, include clock config and ip clock ip config
  */
typedef struct {
    CRG_RegStruct       *baseAddress;     /**< Base address of CLOCK register */
    CRG_PllRefClkSelect pllRefClkSelect;  /**< PLL Refer clock selection */
    CRG_PllPreDiv       pllPreDiv;        /**< PLL pre division */
    unsigned int        pllFbDiv;         /**< PLL loop divider ratio */
    CRG_PllPostDiv      pllPostDiv;       /**< PLL post ratio */
    bool                pllPd;            /**< Pll Power down or not */
    CRG_CoreClkSelect   coreClkSelect;    /**< Core clock selection */
    CRG_ExtendHandle    handleEx;         /**< CRG handle extra */
} CRG_Handle;
/**
  * @}
  */

/**
  * @defgroup CRG_API_Declaration CRG HAL API
  * @{
  */
BASE_StatusType HAL_CRG_Init(const CRG_Handle *handle);

BASE_StatusType HAL_CRG_DeInit(const CRG_Handle *handle);

BASE_StatusType HAL_CRG_GetConfig(CRG_Handle *handle);

BASE_StatusType HAL_CRG_SetCoreClockSelect(CRG_Handle *handle);

BASE_StatusType HAL_CRG_InitWithTargetFrequence(const CRG_Handle *handle, unsigned int targetFreq);

BASE_StatusType HAL_CRG_IpEnableSet(const void *baseAddress, unsigned int enable);

BASE_StatusType HAL_CRG_IpEnableGet(const void *baseAddress,  unsigned int *enable);

BASE_StatusType HAL_CRG_IpClkSelectSet(const void *baseAddress, unsigned int select);

BASE_StatusType HAL_CRG_IpClkSelectGet(const void *baseAddress, unsigned int *select);

BASE_StatusType HAL_CRG_IpClkResetSet(const void *baseAddress, unsigned int reset);

BASE_StatusType HAL_CRG_IpClkResetGet(const void *baseAddress, unsigned int *reset);

BASE_StatusType HAL_CRG_IpClkDivSet(const void *baseAddress, unsigned int div);

BASE_StatusType HAL_CRG_IpClkDivGet(const void *baseAddress, unsigned int *div);

void HAL_CRG_PvdResetEnable(bool enable);

unsigned int HAL_CRG_GetPllFreq(void);

unsigned int HAL_CRG_GetCoreClkFreq(void);

unsigned int HAL_CRG_GetIpFreq(const void *ipBaseAddr);
/**
  * @}
  */

/**
  * @}
  */

/**
 * @}
 */

#endif /* McuMagicTag_CRG_H */