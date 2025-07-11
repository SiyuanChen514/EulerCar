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
  * @file    dma_ex.c
  * @author  MCU Driver Team
  * @brief   DMA module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the DMA.
  *          + DMA Set Functions.
  */

/* Includes ------------------------------------------------------------------*/

#include "dma_ex.h"
/**
  * @brief Configuring the Transmission Channel Priority on the DMA.
  * @param dmaHandle DMA handle.
  * @param channel DMA channel num @ref DMA_ChannelNum.
  * @param priority DMA channel num @ref DMA_ChannelPriority.
  * @retval None.
  */
void HAL_DMA_SetChannelPriorityEx(DMA_Handle *dmaHandle, unsigned int channel, DMA_ChannelPriority priority)
{
    DMA_ASSERT_PARAM(dmaHandle != NULL);
    DMA_PARAM_CHECK_NO_RET(IsDmaChannelNum(channel));
    DMA_PARAM_CHECK_NO_RET(IsDmaPriority(priority));
    dmaHandle->DMA_Channels[channel].channelAddr->DMA_Cn_CONFIG.BIT.ch_priority = priority;
}