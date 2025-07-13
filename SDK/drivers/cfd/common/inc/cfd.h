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
  * @file      cfd.h
  * @author    MCU Driver Team
  * @brief     CFD module driver.
  * @details   This file provides firmware CFD Handle structure and Functions
  *            prototypes to manage the following functionalities of the CFD module.
  *             + Initialization and de-initialization functions
  *             + config the register of CFD module
  */

#ifndef McuMagicTag_CFD_H
#define McuMagicTag_CFD_H

/* Includes ------------------------------------------------------------------ */
#include "cfd_ip.h"

/* Macro definitions ---------------------------------------------------------*/
/**
  * @defgroup CFD CFD
  * @brief CFD module.
  * @{
  */

/**
  * @defgroup CFD_Common CFD Common
  * @brief CFD common external module.
  * @{
  */

/**
  * @defgroup CFD_Handle_Definition CFD Handle Definition
  * @{
  */

/* Typedef definitions -------------------------------------------------------*/

/**
  * @brief CFD module configurable item.
  */
typedef enum {
    CFD_CFG_UPPER_BOUND = 0x00000001U,
    CFD_CFG_INT_TYPE = 0x00000002U,
    CFD_CFG_MAX
} CFD_CFG_TYPE;

/**
  * @brief CFD handle.
  */
typedef struct _CFD_Handle {
    CFD_RegStruct *baseAddress;   /**< CFD registers base address. */
    unsigned char upperBound;     /**< Upper boundary. */
    unsigned int interruptType;   /**< Enabled interrupt type. */
    CFD_UserCallBack  userCallBack;    /**< CFD Interrupt callback functions.*/
    CFD_ExtendHandle  handleEx;        /**< CFD extend parameter */
} CFD_Handle;

/**
  * @brief  Typedef callback function of CFD
  */
typedef void (*CFD_CallBackFuncType)(void *handle);

/**
  * @}
  */
 
/**
  * @defgroup CFD_API_Declaration CFD HAL API
  * @{
  */

/* Hardware abstraction layer functions -------------------------------------------------------- */
BASE_StatusType HAL_CFD_Init(CFD_Handle *handle);
BASE_StatusType HAL_CFD_DeInit(CFD_Handle *handle);
BASE_StatusType CFD_RspInit(CFD_Handle *handle);
BASE_StatusType CFD_RspDeInit(CFD_Handle *handle);
BASE_StatusType HAL_CFD_Config(CFD_Handle *handle, CFD_CFG_TYPE cfgType);
void HAL_CFD_GetConfig(CFD_Handle *handle);
void HAL_CFD_Start(CFD_Handle *handle);
void HAL_CFD_Stop(CFD_Handle *handle);
BASE_StatusType HAL_CFD_RegisterCallback(CFD_Handle *handle, CFD_Interrupt_Type type, CFD_CallBackFuncType callback);
void HAL_CFD_IrqHandler(void *handle);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* McuMagicTag_UART_H */