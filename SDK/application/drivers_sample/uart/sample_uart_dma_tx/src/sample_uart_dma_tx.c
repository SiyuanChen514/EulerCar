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
  * @file    sample_uart_dma_tx.c
  * @author  MCU Driver Team
  * @brief   uart sample module, DMA transfer direction: memory to peripheral.
  * @details In DMA mode, the DMA receives the transfer request from the UART and directly transfers the data
  *          to be transmitted from the memory to the UART for transmission.
  *          (1) Data to be sent: The start address and character length to be sent are used as input parameters
  *          and transferred to HAL_UART_WriteDMA().
  *          (2) Result determination: After the DMA transfer is complete, an interrupt is reported. You can add
  *          a flag to the DMA interrupt callback function to determine whether the transfer is complete.
  */

#include "sample_uart_dma_tx.h"

static unsigned char g_txStr[] = "123456789A";


/**
  * @brief User-defined write completion DMAC callback function.
  * @param handle UART handle.
  * @retval None.
  */
void DMA_Channel3CallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("\r\nwrite_finish\r\n");
    return;
}

/**
  * @brief UART send sample code in DMAC mode.
  * @param None.
  * @retval None.
  */
void UART_DMA_TX(void)
{
    SystemInit();
    unsigned int dataLength = 10;
    DBG_PRINTF("UART and DMA Init finish\r\n");
    HAL_UART_WriteDMA(&g_uart, g_txStr, dataLength);    /* Sending timeout */
    return;
}