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
  * @file    sample_uart_interrupt_tx.c
  * @author  MCU Driver Team
  * @brief   uart sample module.
  * @details In interrupt mode, an interrupt is triggered when the UART has finished transmitting data.
  *          (1) Data to be sent: The start address and character length to be sent are used as input parameters
  *          and transferred to HAL_UART_WriteIT().
  *          (2) Result determination: After the transfer is complete, an UART interrupt is reported. You can add
  *          a flag to the DMA interrupt callback function to determine whether the transfer is complete.
  */
#include "sample_uart_interrupt_tx.h"

static unsigned char g_txStr[] = "12345678987654321";

/**
  * @brief User-defined write completion interrupt callback function.
  * @param None.
  * @retval None.
  */
void WriteFinish(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("\r\nwrite_finish\r\n");
    return;
}

/**
  * @brief UART interrupt send sample code.
  * @param None.
  * @retval None.
  */
void UART_InterruptTX(void)
{
    SystemInit();
    DBG_PRINTF("UART Init finish\r\n");
    unsigned int dataLength = 17;   /* The receive length is 17 */
    HAL_UART_WriteIT(&g_uart, g_txStr, dataLength);
    while (1) {
        ;
    }
    return;
}