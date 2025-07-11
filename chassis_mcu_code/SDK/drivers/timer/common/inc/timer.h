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
  * @file      timer.h
  * @author    MCU Driver Team
  * @brief     TIMER module driver.
  * @details   This file provides firmware TIMER Handle structure and Functions
  *            prototypes to manage the following functionalities of the TIMER.
  *                + Initialization and de-initialization functions
  *                + config the register of timer
  */

#ifndef McuMagicTag_TIMER_H
#define McuMagicTag_TIMER_H

/* Includes ------------------------------------------------------------------*/
#include "timer_ip.h"

/* Macro definitions ---------------------------------------------------------*/

/**
 * @defgroup TIMER TIMER
 * @brief TIMER module.
 * @{
 */

/**
  * @defgroup TIMER_Common TIMER Common
  * @brief TIMER common external module.
  * @{
  */

/**
 * @defgroup TIMER_Handle_Definition TIMER Handle Definition
 * @{
 */

/**
 * @brief  Time base address and Configuration Structure definition
 */
typedef struct _TIMER_Handle {
    TIMER_RegStruct            *baseAddress;      /**< Base address of timer. */
    TIMER_CountMode            cntMode;           /**< Timer cnt Mode. */
    TIMER_Mode                 mode;              /**< Timer counting mode selection. */
    TIMER_PrescalerFactor      prescaler;         /**< Timer prescaler. */
    TIMER_Size                 size;              /**< Timer size 16 or 32 bits. */
    volatile unsigned int      load;              /**< Period, set the TIMERx_LOAD. */
    volatile unsigned int      bgLoad;            /**< Backgroud period, set the TIMEx_BGLOAD. */
    bool                       interruptEn;       /**< Interrupt enable or disable. */
    bool                       adcSocReqEnable;   /**< Trigger ADC Enable Sampling. */
    bool                       dmaReqEnable;      /**< Enable bit for DMA single request and DAM burst sampling. */
    TIMER_UserCallBack         userCallBack;      /**< Callback function of timer. */
    TIMER_ExtendHandle         handleEx;          /**< TIMER extend handle */
} TIMER_Handle;

/**
 * @brief  Typedef callback function of TIMER
 */
typedef void (*TIMER_CallBackFunc)(void *param);

/**
  * @}
  */

/**
  * @defgroup TIMER_API_Declaration TIMER HAL API
  * @{
  */
BASE_StatusType HAL_TIMER_Init(TIMER_Handle *handle);

void HAL_TIMER_DeInit(TIMER_Handle *handle);

void HAL_TIMER_Start(TIMER_Handle *handle);

void HAL_TIMER_Stop(TIMER_Handle *handle);

BASE_StatusType HAL_TIMER_Config(TIMER_Handle *handle, TIMER_CFG_TYPE cfgType);

BASE_StatusType HAL_TIMER_GetConfig(TIMER_Handle *handle);

BASE_StatusType HAL_TIMER_RegisterCallback(TIMER_Handle *handle, TIMER_InterruptType typeID,
                                           TIMER_CallBackFunc callBackFunc);

BASE_StatusType HAL_TIMER_UnRegisterCallback(TIMER_Handle *handle, TIMER_InterruptType typeID);

void HAL_TIMER_IrqHandler(void *handle);
/**
  * @}
  */

/**
  * @}
  */

/**
 * @}
 */
#endif /* McuMagicTag_TIMER_H */