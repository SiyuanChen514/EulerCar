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
  * @file    sample_uart_baud_detect.c
  * @author  MCU Driver Team
  * @brief   uart baud detection sample module.
  * @details This file provides sample code for users to help use
  *          the baud detection of the UART. Send char 'Z' to the UART to complete the baud rate detection.
  */
#include "sample_uart_baud_detect.h"

static bool baudCheck = BASE_CFG_DISABLE;   /* Flag of baud detection. */

/**
  * @brief UART baud detection success callback.
  * @param None.
  * @retval None.
  */
void UART_BaudDetectCallBack_Ok(void *handle)
{
    UART_Handle *uartHandle = (UART_Handle *)handle;
    BASE_FUNC_UNUSED(uartHandle);
    DBG_PRINTF("UART_BaudDetectCallBack_Ok\r\n");
    baudCheck = BASE_CFG_ENABLE;
}

/**
  * @brief UART baud detection error callback.
  * @param None.
  * @retval None.
  */
void UART_BaudDetectCallBack_Error(void *handle)
{
    UART_Handle *uartHandle = (UART_Handle *)handle;
    BASE_FUNC_UNUSED(uartHandle);
    /* Enable tx and rx. */
    uartHandle->baseAddress->UART_CR.BIT.txe = BASE_CFG_ENABLE;
    uartHandle->baseAddress->UART_CR.BIT.rxe = BASE_CFG_ENABLE;
    DBG_PRINTF("UART_BaudDetectCallBack_Error\r\n");
    baudCheck = BASE_CFG_DISABLE;
}

/**
  * @brief UART baud detection.
  * @param None.
  * @retval None.
  */
void UART_BaudDetection(void)
{
    SystemInit();
    /* Send char 'Z' to the UART to complete the baud rate detection. */
    return;
}