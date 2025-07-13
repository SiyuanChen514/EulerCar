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
 * @file    acmp.h
 * @author  MCU Driver Team.
 * @brief   ACMP module driver.
 *          This file provides functions declaration of the Comparator.
 *           + Comparator's Initialization and de-initialization functions
 *           + Set Comparator's hysteresis voltage function
 *           + Set blocking function.
 *           + Interrupt handling and register.
 */
#ifndef McuMagicTag_ACMP_H
#define McuMagicTag_ACMP_H
#include "acmp_ip.h"

#ifdef ACMP_PARAM_CHECK
#define ACMP_ASSERT_PARAM BASE_FUNC_ASSERT_PARAM
#else
#define ACMP_ASSERT_PARAM(para) ((void)0U)
#endif

/**
  * @defgroup ACMP ACMP
  * @brief ACMP module.
  * @{
  */

/**
  * @defgroup ACMP_Common ACMP Common
  * @brief ACMP common external module.
  * @{
  */


/**
  * @defgroup ACMP_Handle_Definition ACMP Handle Definition
  * @{
  */

/**
  * @brief ACMP Handle
  */
typedef struct _ACMP_Handle {
    ACMP_RegStruct        *baseAddress;        /**< ACMP registers base address. */
    ACMP_InOutConfig       inOutConfig;        /**< ACMP input and output setting. */
    ACMP_FilterCtrl        filterCtrl;         /**< ACMP filter setting. */
    unsigned short         hysteresisVol;      /**< ACMP hysteresis voltage setting. */
    ACMP_UserCallBack      userCallBack;       /**< ACMP user callback function. */
    volatile bool          interruptEn;        /**< ACMP interrupt enable. */
    ACMP_ExtendHandle      handleEx;           /**< ACMP extended parameter. */
} ACMP_Handle;

typedef void (* ACMP_CallBackType)(void *handle);
/**
  * @}
  */

/**
  * @defgroup ACMP_API_Declaration ACMP HAL API
  * @{
  */
BASE_StatusType HAL_ACMP_Init(ACMP_Handle *acmpHandle);
BASE_StatusType HAL_ACMP_DeInit(ACMP_Handle *acmpHandle);
void HAL_ACMP_SetHystVol(ACMP_Handle *acmpHandle, ACMP_HystVol voltage);
void HAL_ACMP_BlkingValid(ACMP_Handle *acmpHandle);
void HAL_ACMP_BlkingInvalid(ACMP_Handle *acmpHandle);

/* ACMP output result selection. */
BASE_StatusType HAL_ACMP_ResultSelect(ACMP_Handle *acmpHandle, ACMP_ResultSelect resultSelect);

/* ACMP interrupt service function and user callback registration function. */
void HAL_ACMP_IrqHandler(void *handle);
BASE_StatusType HAL_ACMP_RegisterCallBack(ACMP_Handle *uartHandle, ACMP_CallBackFun_Type typeID,
                                          ACMP_CallBackType pCallback);
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