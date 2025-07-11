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
  * @file    sample_dma_list_transfer.c
  * @author  MCU Driver Team
  * @brief   dma sample module, scatter gather DMA.
  * @details This file provides sample code for users to help use
  *          the data transfer function of the dma in chain transmission mode.
  */
#include "sample_dma_list_transfer.h"

#define NUM 100
static unsigned char g_str1[NUM] = "1234567890123456789012345678901234567890123456789012345678901234567890";
static unsigned char g_str2[NUM] = {0};
static unsigned char g_str3[NUM] = {0};
static DMA_Handle g_dmac;
static DMA_ChannelParam g_param;

/**
  * @brief DMA controller initialization.
  * @param None.
  * @retval None.
  */
static void DMA_ControllerInit(void)
{
    g_dmac.baseAddress = DMA;
    g_dmac.srcByteOrder = DMA_BYTEORDER_SMALLENDIAN;
    g_dmac.destByteOrder = DMA_BYTEORDER_SMALLENDIAN;
    g_dmac.irqNumTc = IRQ_DMA_TC;
    g_dmac.irqNumError = IRQ_DMA_ERR;
    HAL_DMA_Init(&g_dmac);
}

/**
  * @brief DMA interrupt initialization.
  * @param handle DMA handle.
  * @retval None.
  */
static void DMA_InterruptInit(DMA_Handle *handle)
{
    IRQ_Enable();
    handle->irqNumTc = IRQ_DMA_TC;
    handle->irqNumError = IRQ_DMA_ERR;
    HAL_DMA_IRQService(handle);
    IRQ_EnableN(IRQ_DMA_TC);
    IRQ_EnableN(IRQ_DMA_ERR);
}

/**
  * @brief User-defined callback function for completing the transfer of memory to the memory.
  * @param handle callback handle.
  * @retval None.
  */
static void DMA_MemToMemFinishList(void *handle)
{
    DBG_PRINTF("g_str1->g_str2 Finish!\r\n");
    DBG_PRINTF("src_memory g_str1: %s\r\n", g_str1);
    DBG_PRINTF("dest_memory g_str2: %s\r\n", g_str2);
    DBG_PRINTF("dest_memory g_str3: %s\r\n", g_str3);
    if (handle == NULL) {
        DBG_PRINTF("handle is nullptr!\r\n");
    }
    BASE_FUNC_UNUSED(handle);
}

static DMA_LinkList g_firstNode;
static DMA_LinkList g_secondNode;
/**
  * @brief DMA sample code for the transfer of DMA chain transmission.
  * @param None.
  * @retval None.
  */
int DMA_MemoryToMemoryList(void)
{
    DBG_UartPrintInit(BAUDRATE);    /* baud rate is 115200 */
    DBG_PRINTF("MemoryToMemoryList Begin: \r\n");
    DBG_PRINTF("src_memory g_str1: %s\r\n", g_str1);
    DBG_PRINTF("dest_memory g_str2: %s\r\n", g_str2);
    DBG_PRINTF("dest_memory g_str3: %s\r\n", g_str3);

    unsigned int channel = 2;   /* select transfer channel 2 */
    DMA_ControllerInit();
    DMA_InterruptInit(&g_dmac);
    HAL_DMA_RegisterCallback(&g_dmac, DMA_CHANNEL_FINISH, channel, DMA_MemToMemFinishList);

    g_param.direction = DMA_MEMORY_TO_MEMORY_BY_DMAC;
    g_param.srcAddrInc = DMA_ADDR_UNALTERED;
    g_param.destAddrInc = DMA_ADDR_INCREASE;
    g_param.srcWidth = DMA_TRANSWIDTH_BYTE;
    g_param.destWidth = DMA_TRANSWIDTH_BYTE;
    g_param.srcBurst = DMA_BURST_LENGTH_1;
    g_param.destBurst = DMA_BURST_LENGTH_1;
    g_param.pHandle = &g_dmac;
    /* The transmission length is defined as 5 */
    HAL_DMA_InitNewNode(&g_firstNode, &g_param, (uintptr_t)(void *)g_str1, (uintptr_t)(void *)g_str2, 5);
    /* The transmission length is defined as 50 */
    HAL_DMA_InitNewNode(&g_secondNode, &g_param, (uintptr_t)(void *)g_str1, (uintptr_t)(void *)g_str3, 50);

    HAL_DMA_ListAddNode(&g_firstNode, &g_secondNode);
    HAL_DMA_StartListTransfer(&g_dmac, &g_firstNode, channel);
    DBG_PRINTF("End!\r\n");
    return 0;
}