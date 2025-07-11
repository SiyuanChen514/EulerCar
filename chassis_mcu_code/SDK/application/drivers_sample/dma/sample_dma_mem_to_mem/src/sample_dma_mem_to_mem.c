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
  * @file    sample_dma_mem_to_mem.c
  * @author  MCU Driver Team
  * @brief   dma sample module, memory-to-memory transfer.
  * @details The DMA is used to transfer data from the memory to the memory.
  *          After the transfer is complete, the DMA interrupt is triggered.
  *          (1) Transfer configuration: Transfer the source and destination addresses to the HAL_DMA_StartIT() API.
  *          Transfer the length of the transferred data and the DMA channel number as input parameters.
  *          (2) Result judgment: After the DMA transfer is complete, in the interrupt callback function, check whether
  *          the source and destination data are consistent.
  */
#include "sample_dma_mem_to_mem.h"

static unsigned char g_str1[10] = "12345678";
static unsigned char g_str2[10] = "";
void DMA_MemToMemCallBack(void *handle);
/**
  * @brief User-defined callback function for completing the transfer of memory to the memory.
  * @param handle callback handle.
  * @retval None.
  */
void DMA_MemToMemCallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("DMA_MemToMemCallBack\r\n");
    DBG_PRINTF("After transmission, src: %s\r\n", g_str1);
    DBG_PRINTF("After transmission, dest: %s\r\n", g_str2);
}

/**
  * @brief DMA sample code for the transfer of memory to the memory.
  * @param None.
  * @retval None.
  */
void DMA_MemoryToMemory(void)
{
    SystemInit();
    DBG_PRINTF("MemoryToMemory Begin: \r\n");
    DBG_PRINTF("Before transmission, src: %s\r\n", g_str1);
    DBG_PRINTF("Before transmission, dest: %s\r\n", g_str2);
    unsigned int channel = 2;  /* select transfer channel 2 */
    HAL_DMA_RegisterCallback(&g_dmac, DMA_CHANNEL_FINISH, channel, DMA_MemToMemCallBack);
    /* The transmission length is defined as 8 */
    HAL_DMA_StartIT(&g_dmac, (uintptr_t)(void *)g_str1, (uintptr_t)(void *)g_str2,  8, channel);
}