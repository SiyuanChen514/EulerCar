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
  * @file    sample_uart_interrupt_rx.c
  * @author  MCU Driver Team
  * @brief   uart sample module.
  * @details In interrupt mode, when the UART completes receiving data, it indicates that the data
  *          has been transferred to the preset memory. In this case, an interrupt is triggered.
  *          (1) Preset buffer: The user needs to create the memory space. Use the start address of the memory
  *          and the length of the character to be received as input parameters to HAL_UART_ReadIT().
  *          (2) Result reading: Read the result from the preset memory in the callback function of the UART.
  */
#include "sample_uart_interrupt_rx.h"

static unsigned char g_rxStr[20] = {0};
static volatile unsigned int g_flag;

/**
  * @brief User-defined read completion interrupt callback function.
  * @param UART_Handle UART handle.
  * @retval None.
  */
void ReadFinish(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("read_finish: %s\r\n", g_rxStr);
    g_flag = 1;
    return;
}

/**
  * @brief UART interrupt receive sample code.
  * @param None.
  * @retval None.
  */
void UART_InterruptRX(void)
{
    SystemInit();
    DBG_PRINTF("UART Init finish\r\n");
    unsigned int dataLength = 15;  /* The receive length is 15 */
    g_flag = 0;
    HAL_UART_ReadIT(&g_uart, g_rxStr, dataLength);
    while (1) {
        if (g_flag == 1) {
            g_flag = 0;
            HAL_UART_ReadIT(&g_uart, g_rxStr, dataLength);  /* Start receiving again */
        }
    }
    return;
}