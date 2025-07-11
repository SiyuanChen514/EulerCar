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
  * @file    debug.h
  * @author  MCU Driver Team
  * @brief   Header file containing functions prototypes of DEBUG module.
  *          + Initialization and de-initialization functions
  *          + Format print function
  */

#ifndef McuMagicTag_DEBUG_H
#define McuMagicTag_DEBUG_H

#include "uart.h"

#ifdef DEBUG_PARAM_CHECK
#define DEBUG_ASSERT_PARAM BASE_FUNC_ASSERT_PARAM
#define DEBUG_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define DEBUG_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define DEBUG_ASSERT_PARAM(para) ((void)0U)
#define DEBUG_PARAM_CHECK_NO_RET(para)  ((void)0U)
#define DEBUG_PARAM_CHECK_WITH_RET(param, ret) ((void)0U)
#endif

/**
  * @defgroup DEBUG DEBUG
  * @brief DEBUG module.
  * @{
  */


/**
  * @defgroup DEBUG_Common DEBUG Common
  * @brief DEBUG common external module.
  * @{
  */

/* Macro definitions for enabling the function of DEBUG_PRINT submodule */
#define BAUDRATE 115200

#if (DBG_PRINTF_USE == DBG_USE_NO_PRINTF)
static inline int DBG_dummy(const char *format, ...)
{
    BASE_FUNC_UNUSED(format);
    return 0;
} /* dummy debug function */
#define DBG_PRINTF DBG_dummy /* Delete all print statement */
#endif

#if (DBG_PRINTF_USE == DBG_USE_UART_PRINTF)
#define DBG_PRINTF DBG_UartPrintf /**< Select the customized printf function */
#endif

/**
  * @defgroup DEBUG_API_Declaration DEBUG HAL API
  * @{
  */
BASE_StatusType DBG_UartPrintInit(unsigned int baudRate);
BASE_StatusType DBG_UartPrintDeInit(void);

/* Format print function */
int DBG_UartPrintf(const char *format, ...); /* Supported format: %c, %s, %d, %u, %x, %X, %p, %f */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* McuMagicTag_DEBUG_H */