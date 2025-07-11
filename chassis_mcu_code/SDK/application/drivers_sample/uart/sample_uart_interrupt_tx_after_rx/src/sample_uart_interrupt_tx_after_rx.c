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
  * @file    sample_uart_interrupt_tx_after_rx.c
  * @author  MCU Driver Team
  * @brief   uart sample module.
  * @details In interrupt mode, when the UART receives data from the peer end, the UART sends the received data to the
  *          peer end. The UART interrupt is triggered during the entire process of receiving and transmitting data.
  *          (1) Receive: The user needs to open the memory space. The start address of the memory and the length of
  *          character to be received are used as input parameters and transferred to HAL_UART_ReadIT(). The received
  *          data is stored in the cache.
  *          (2) Transmit: After receiving an interrupt, the data in the buffer is used as the data to be transmitted
  *          and is filled in HAL_UART_WriteIT().
  */
#include "sample_uart_interrupt_tx_after_rx.h"

static unsigned char g_str[15] = {0};
static volatile unsigned int g_flag;
static unsigned int CountString(unsigned char *str);
static void ClearString(unsigned char *str);

/**
  * @brief User-defined read completion interrupt callback function.
  * @param UART_Handle UART handle.
  * @retval None.
  */
void ReadCallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("Read Finish: %s\r\n", g_str);
    g_flag = true;
    return;
}

/**
  * @brief User-defined write completion interrupt callback function.
  * @param UART_Handle UART handle.
  * @retval None.
  */
void WriteCallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("\r\nWrite Finish\r\n");
    return;
}

/**
  * @brief Count length of string.
  * @param str, String to be cleared.
  * @retval unsigned int, Character length.
  */
static unsigned int CountString(unsigned char *str)
{
    unsigned int ret;
    if (str == NULL) {
        return 0;
    }
    unsigned char *tmpStr = str;    /* Cycle Count */
    for (ret = 0; *tmpStr != 0; tmpStr++) {
        ret++;
    }
    return ret;
}

/**
  * @brief Clear string.
  * @param str, String to be cleared.
  * @retval None.
  */
static void ClearString(unsigned char *str)
{
    unsigned int len = CountString(str);
    for (unsigned int i = 0; i < len; ++i) {
        str[i] = 0;
    }
}

/**
  * @brief UART interrupt receive sample code.
  * @param None.
  * @retval None.
  */
void UART_InterruptTxAfterRx(void)
{
    SystemInit();
    DBG_PRINTF("UART Init finish, please enter characters(length no more than 10):\r\n");
    unsigned int rxDataLength = 10;  /* The receive length is 10 */
    g_flag = false;
    HAL_UART_ReadIT(&g_uart, g_str, rxDataLength);
    while (1) {
        if (g_flag == true) {
            g_flag = false;
            unsigned int txDataLength = CountString(g_str); /* string length of the data to be sent after receiving */
            HAL_UART_WriteIT(&g_uart, g_str, txDataLength);
            ClearString(g_str);
            HAL_UART_ReadIT(&g_uart, g_str, rxDataLength);
        }
    }
    return;
}