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
  * @file    sample_uart_dma_rx_cyclically_stored.c
  * @author  MCU Driver Team
  * @brief   uart sample module, UART example module, DMA cyclic write data, transfer direction: peripheral to memory.
  * @details In DMA cyclic write data mode, after receiving data from the UART, DMA transfers the data to
  *          preset memory.  If the memory is full, the data will be stored again.
  *          request from the UART and transfers the data to the preset memory.
  *          (1)Preset buffer: The user needs to create the memory space. Use the start address of the memory
  *          and length of character to be received as input parameters to HAL_UART_ReadDMAAndCyclicallyStored().
  *          (2) Result reading: Read the result from the preset memory.
  */

#include "sample_uart_dma_rx_cyclically_stored.h"

#define BUF_LEN 80

unsigned int g_pointRead = 0;  /* Read position  */
unsigned int g_pointWrite = 0; /* Write position  */

unsigned char g_buf[BUF_LEN]; /* Buf for saving data */

static DMA_LinkList g_Node; /* DMA linked list must be global */

int UART_DMA_RxCyclicallyStored(void)
{
    SystemInit();

    DBG_PRINTF("UART and DMA Init Finish\r\n UART DMA read and cyclically stored: please send characters to UART\r\n");
    g_pointRead = 0;

    /* Start DMA cyclically stored, The DMA can cyclically transfer data to the g_buf buffer */
    HAL_UART_ReadDMAAndCyclicallyStored(&g_uart, g_buf, &g_Node, BUF_LEN);

    while (1) {
        /* Obtains the destination address written by the DAM */
        g_pointWrite = HAL_UART_ReadDMAGetPos(&g_uart);

        /* Print data if read pointer is inconsistent with write pointer */
        while (g_pointRead != g_pointWrite) {
            DBG_PRINTF(
                "g_pointRead=0x%x, g_pointWrite=0x%x, receive:%c\r\n", g_pointRead, g_pointWrite, g_buf[g_pointRead]);
            g_pointRead++;
            if (g_pointRead >= BUF_LEN) {
                g_pointRead = 0;
            }
        }
    }
    return 0;
}
