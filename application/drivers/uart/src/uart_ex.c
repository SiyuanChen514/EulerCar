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
  * @file    uart_ex.c
  * @author  MCU Driver Team
  * @brief   UART module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the UART.
  *           + Initialization and de-initialization functions.
  *           + Peripheral send and receive functions in blocking mode.
  *           + Peripheral send and receive functions in interrupt mode.
  *           + Peripheral send and receive functions in DMA mode.
  *           + Peripheral stop sending and receiving functions in interrupt/DMA mode.
  *           + Interrupt callback function and user registration function.
  */

/* Includes ------------------------------------------------------------------*/
#include "uart_ex.h"
/* Macro definitions ---------------------------------------------------------*/

/**
  * @brief Open the character matching function of the UART RX and set the matching character.
  * @param uartHandle UART handle.
  * @param ch Characters to be matched.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_UART_OpenCharacterMatchEx(UART_Handle *uartHandle, unsigned char ch)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    uartHandle->baseAddress->UART_IMSC.BIT.cmim = BASE_CFG_ENABLE;
    uartHandle->baseAddress->UART_CHARMATCH.BIT.chamat = (unsigned int)ch; /* Sets the matching character. */
    uartHandle->baseAddress->UART_CHARMATCH.BIT.cmen = BASE_CFG_ENABLE;
    return BASE_STATUS_OK;
}

/**
  * @brief Close the character matching function of the UART RX.
  * @param uartHandle UART handle.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_UART_CloseCharacterMatchEx(UART_Handle *uartHandle)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    uartHandle->baseAddress->UART_CHARMATCH.BIT.cmen = BASE_CFG_DISABLE;
    uartHandle->baseAddress->UART_IMSC.BIT.cmim = BASE_CFG_DISABLE;  /* Turn off character matching */
    return BASE_STATUS_OK;
}

/**
  * @brief Enable the UART to automatically identify the baud rate and enable the corresponding interrupt.
  * @param uartHandle UART handle.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_UART_EnableBaudDetectionEx(UART_Handle *uartHandle)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    uartHandle->baseAddress->UART_CR.BIT.txe = BASE_CFG_DISABLE;
    uartHandle->baseAddress->UART_CR.BIT.rxe = BASE_CFG_DISABLE;      /* Disable TX and RX first */
    uartHandle->baseAddress->UART_IMSC.BIT.abdeim = BASE_CFG_ENABLE;
    uartHandle->baseAddress->UART_IMSC.BIT.abdcim = BASE_CFG_ENABLE;
    uartHandle->baseAddress->UART_ABDEN.BIT.abden = BASE_CFG_ENABLE;
    return BASE_STATUS_OK;
}

/**
  * @brief Disable the UART to automatically identify the baud rate.
  * @param uartHandle UART handle.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_UART_DisableBaudDetectionEx(UART_Handle *uartHandle)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    uartHandle->baseAddress->UART_ABDEN.BIT.abden = BASE_CFG_DISABLE;
    uartHandle->baseAddress->UART_IMSC.BIT.abdeim = BASE_CFG_DISABLE;
    uartHandle->baseAddress->UART_IMSC.BIT.abdcim = BASE_CFG_DISABLE;
    uartHandle->baseAddress->UART_CR.BIT.txe = BASE_CFG_ENABLE; /* Enable TX */
    uartHandle->baseAddress->UART_CR.BIT.rxe = BASE_CFG_ENABLE; /* Enable RX */
    return BASE_STATUS_OK;
}

/**
  * @brief Configuring the upper limit of Rx timeout.
  * @param uartHandle UART handle.
  * @param cntOfBit timeout is defined as the time spent in transmitting N bits, numer of N is cntOfBit.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_UART_SetRxWaiteTimeEx(UART_Handle *uartHandle, unsigned int cntOfBit)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    UART_PARAM_CHECK_WITH_RET(cntOfBit <= 0xFFFFFF, BASE_STATUS_ERROR);
    uartHandle->baseAddress->UART_RTCFG.reg = cntOfBit; /* Set wait time */
    return BASE_STATUS_OK;
}

/**
  * @brief Sets UART oversampling multiple.
  * @param uartHandle UART handle.
  * @param multiple Oversampling multiple, @ref UART_OversampleMultiple
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_UART_SetOversampleMultipleEx(UART_Handle *uartHandle, UART_OversampleMultiple multiple)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    UART_PARAM_CHECK_WITH_RET(IsUartOversampleMultiple(multiple), BASE_STATUS_ERROR);
    uartHandle->baseAddress->UART_SPCFG.BIT.spcfg = multiple;  /* Oversample setting */
    return BASE_STATUS_OK;
}

/**
  * @brief Sets the first bit of the character transmitted in the UART transmission.
  * @param uartHandle UART handle.
  * @param mode Sequence mode : LSB/MSB, @ref UART_SequenceMode
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_UART_SetDataSequenceModeEx(UART_Handle *uartHandle, UART_SequenceMode mode)
{
    UART_ASSERT_PARAM(uartHandle != NULL);
    UART_ASSERT_PARAM(IsUARTInstance(uartHandle->baseAddress));
    UART_PARAM_CHECK_WITH_RET(IsUartSequenceMode(mode), BASE_STATUS_ERROR);
    uartHandle->baseAddress->UART_DS.BIT.msbfirst = mode; /* Data sequence setting */
    return BASE_STATUS_OK;
}