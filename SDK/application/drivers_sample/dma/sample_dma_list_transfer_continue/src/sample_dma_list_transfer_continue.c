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
  * @file    sample_dma_list_transfer_continue.c
  * @author  MCU Driver Team
  * @brief   Single-node continuous transmission.
  * @details The DMA is used to transfer data from the memory to the memory.
  *          Single-node continuous transmission.
  */
#include "sample_dma_list_transfer_continue.h"

static unsigned char g_src[5] = "ABC";
static unsigned char g_dest[5] = "";
static DMA_LinkList g_node;
static DMA_ChannelParam g_param;
/**
  * @brief DMA sample code for the transfer of list node continue transmission.
  * @param None.
  * @retval None.
  */
void DMA_List_transfer_continue(void)
{
    SystemInit();
    DBG_PRINTF("ListTransferContinue Begin: \r\n");
    DBG_PRINTF("Before transmission, src: %s\r\n", g_src);
    DBG_PRINTF("Before transmission, dest: %s\r\n", g_dest);
    unsigned int channel = 2;       /* select transfer channel 2 */
    unsigned int dataLength = 3;    /* The length of the transferred characters is 3 */
    unsigned int count = 0;
    g_param.direction = DMA_MEMORY_TO_MEMORY_BY_DMAC;
    g_param.srcAddrInc = DMA_ADDR_INCREASE;
    g_param.destAddrInc = DMA_ADDR_INCREASE;
    g_param.srcWidth = DMA_TRANSWIDTH_BYTE;
    g_param.destWidth = DMA_TRANSWIDTH_BYTE;
    g_param.srcBurst = DMA_BURST_LENGTH_1;
    g_param.destBurst = DMA_BURST_LENGTH_1;
    g_param.pHandle = &g_dmac;
    HAL_DMA_InitNewNode(&g_node, &g_param, (uintptr_t)(void *)g_src, (uintptr_t)(void *)g_dest, dataLength);
    HAL_DMA_ListAddNode(&g_node, &g_node);
    HAL_DMA_StartListTransfer(&g_dmac, &g_node, channel);
    DBG_PRINTF("DMA Start Transport!\r\n");
    while (1) {
        BASE_FUNC_DelayUs(500);     /* delay 500 us */
        if (count % 100 == 0 && count <= 2000) {     /* 100 and 2000 are used for control printf */
            DBG_PRINTF("src: %s, dest: %s\r\n", g_src, g_dest);
            if (count == 1500) {  /* 1500 for turning off DMA transport */
                HAL_DMA_StopChannel(&g_dmac, channel);
                DBG_PRINTF("DMA Stop Transport!\r\n");
            }
            g_src[0] += 1;
            g_src[1] += 1;
            g_src[2] += 1;  /* Change the character of the subscript is 2 */
        }
        count++;
    }
}