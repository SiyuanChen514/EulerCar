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
  * @file    uart_ex.h
  * @author  MCU Driver Team
  * @brief   UART module driver.
  * @details This file provides functions declaration of the UART,
  *           + Initialization and de-initialization functions
  *           + Peripheral querying the state functions.
  *           + Peripheral transmit and abort functions.
  *           + Peripheral interrupt service and callback registration functions.
  *          This file also provides the definition of the UART handle structure.
  */

/* Includes ------------------------------------------------------------------*/
#ifndef McuMagicTag_UART_EX_H
#define McuMagicTag_UART_EX_H

#include "uart.h"

/**
  * @addtogroup UART_IP
  * @{
  */

/**
  * @defgroup UART_EX_API_Declaration UART HAL API EX
  * @{
  */
BASE_StatusType HAL_UART_OpenCharacterMatchEx(UART_Handle *uartHandle, unsigned char ch);

BASE_StatusType HAL_UART_CloseCharacterMatchEx(UART_Handle *uartHandle);

BASE_StatusType HAL_UART_EnableBaudDetectionEx(UART_Handle *uartHandle);

BASE_StatusType HAL_UART_DisableBaudDetectionEx(UART_Handle *uartHandle);

BASE_StatusType HAL_UART_SetRxWaiteTimeEx(UART_Handle *uartHandle, unsigned int cntOfBit);

BASE_StatusType HAL_UART_SetOversampleMultipleEx(UART_Handle *uartHandle, UART_OversampleMultiple multiple);

BASE_StatusType HAL_UART_SetDataSequenceModeEx(UART_Handle *uartHandle, UART_SequenceMode mode);

/**
  * @}
  */

/**
  * @}
  */
#endif  /* McuMagicTag_UART_EX_H */