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
  * @file    sample_uart_dma_rx.c
  * @author  MCU Driver Team
  * @brief   uart sample module, DMA transfer direction: peripheral to memory.
  * @details In DMA mode, after receiving data from the UART, the DMA receives the transfer
  *          request from the UART and transfers the data to the preset memory.
  *          (1)Preset buffer: The user needs to create the memory space. Use the start address of the memory
  *          and the length of the character to be received as input parameters to HAL_UART_ReadDMA().
  *          (2) Result reading: Read the result from the preset memory in the callback function of the DMA.
  */

#include "sample_uart_dma_rx.h"

static unsigned char g_rxStr[20] = {0};
/**
  * @brief User-defined read completion DMAC callback function.
  * @param handle UART handle.
  * @retval None.
  */
void DMA_Channel2CallBack(void *handle)
{
    UART_Handle *uartHandle = (UART_Handle *)handle;
    DBG_PRINTF("read_finish: %s\r\n", g_rxStr);
    BASE_FUNC_UNUSED(uartHandle);
    return;
}

/**
  * @brief UART receive sample code in DMAC mode.
  * @param None.
  * @retval None.
  */
int UART_DMA_RX(void)
{
    SystemInit();
    DBG_PRINTF("UART and DMA Init Finish, please send characters to UART\r\n");
    unsigned int dataLength = 10;  /* The receive length is 10 */
    HAL_UART_ReadDMA(&g_uart, g_rxStr, dataLength);
    while (1) {
        HAL_UART_ReadDMA(&g_uart, g_rxStr, dataLength);
    }
    return 0;
}