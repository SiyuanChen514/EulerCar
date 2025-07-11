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
  * @file    dma.c
  * @author  MCU Driver Team
  * @brief   DMA module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the DMA.
  *           + Initialization and de-initialization functions.
  *           + Start DMA transfer with interrupt mode.
  *           + Start DMA transfer without interrupt mode.
  *           + Stop DMA transfer and query the state of DMA.
  *           + Interrupt callback function and user registration function.
  */

/* Includes ------------------------------------------------------------------*/
#include "interrupt.h"
#include "dma.h"

static DMA_LinkList g_listTable[LISTNODE_MAX] = {0};
static unsigned int g_listIndex = 0;

static DMA_PeriphReq g_periphReqTable[] = {
    {DMA_REQLINEVAL_0, DMA_SYSCTRLSET_0, 0},    /* DMA_REQUEST_I2C_RX, system registers shift left 0 bit set */
    {DMA_REQLINEVAL_1, DMA_SYSCTRLSET_0, 1},    /* DMA_REQUEST_I2C_TX, system registers shift left 1 bit set */
    {DMA_REQLINEVAL_2, DMA_SYSCTRLSET_0, 0},    /* DMA_REQUEST_UART0_RX, system registers no use */
    {DMA_REQLINEVAL_3, DMA_SYSCTRLSET_0, 0},    /* DMA_REQUEST_UART0_TX, system registers no use */
    {DMA_REQLINEVAL_4, DMA_SYSCTRLSET_0, 0},    /* DMA_REQUEST_UART1_RX, system registers no use */
    {DMA_REQLINEVAL_5, DMA_SYSCTRLSET_0, 5},    /* DMA_REQUEST_UART1_TX, system registers shift left 5 bit set */
    {DMA_REQLINEVAL_6, DMA_SYSCTRLSET_1, 6},    /* DMA_REQUEST_SPI_RX, system registers shift left 6 bits set */
    {DMA_REQLINEVAL_7, DMA_SYSCTRLSET_1, 7},    /* DMA_REQUEST_SPI_TX, system registers shift left 7 bits set */
    {DMA_REQLINEVAL_8, DMA_SYSCTRLSET_0, 8},    /* DMA_REQUEST_CAPM0, system registers shift left 8 bits set */
    {DMA_REQLINEVAL_9, DMA_SYSCTRLSET_0, 9},    /* DMA_REQUEST_CAPM1, system registers shift left 9 bits set */
    {DMA_REQLINEVAL_10, DMA_SYSCTRLSET_0, 10},  /* DMA_REQUEST_CAPM2, system registers shift left 10 bits set */
    {DMA_REQLINEVAL_11, DMA_SYSCTRLSET_0, 11},  /* DMA_REQUEST_ADC0, system registers shift left 11 bits set */
    {DMA_REQLINEVAL_12, DMA_SYSCTRLSET_0, 12},  /* DMA_REQUEST_ADC1, system registers shift left 12 bits set */
    {DMA_REQLINEVAL_13, DMA_SYSCTRLSET_0, 13},  /* DMA_REQUEST_ADC2, system registers shift left 13 bits set */
    {DMA_REQLINEVAL_14, DMA_SYSCTRLSET_0, 14},  /* DMA_REQUEST_TIMER0, system registers shift left 14 bits set */
    {DMA_REQLINEVAL_15, DMA_SYSCTRLSET_0, 15},  /* DMA_REQUEST_TIMER1, system registers shift left 15 bits set */
    {DMA_REQLINEVAL_6, DMA_SYSCTRLSET_0, 6},    /* DMA_REQUEST_UART2_RX, system registers shift left 6 bits set */
    {DMA_REQLINEVAL_7, DMA_SYSCTRLSET_0, 7},    /* DMA_REQUEST_UART2_TX, system registers shift left 7 bits set */
    {DMA_REQLINEVAL_5, DMA_SYSCTRLSET_1, 5},    /* DMA_REQUEST_APT8, system registers shift left 5 bits set */
    {DMA_REQLINEVAL_8, DMA_SYSCTRLSET_1, 8},    /* DMA_REQUEST_APT0, system registers shift left 8 bits set */
    {DMA_REQLINEVAL_9, DMA_SYSCTRLSET_1, 9},    /* DMA_REQUEST_APT1, system registers shift left 9 bits set */
    {DMA_REQLINEVAL_10, DMA_SYSCTRLSET_1, 10},  /* DMA_REQUEST_APT2, system registers shift left 10 bits set */
    {DMA_REQLINEVAL_11, DMA_SYSCTRLSET_1, 11},  /* DMA_REQUEST_APT3, system registers shift left 11 bits set */
    {DMA_REQLINEVAL_12, DMA_SYSCTRLSET_1, 12},  /* DMA_REQUEST_APT4, system registers shift left 12 bits set */
    {DMA_REQLINEVAL_13, DMA_SYSCTRLSET_1, 13},  /* DMA_REQUEST_APT5, system registers shift left 13 bits set */
    {DMA_REQLINEVAL_14, DMA_SYSCTRLSET_1, 14},  /* DMA_REQUEST_APT6, system registers shift left 14 bits set */
    {DMA_REQLINEVAL_15, DMA_SYSCTRLSET_1, 15},  /* DMA_REQUEST_APT7, system registers shift left 15 bits set */
    {DMA_REQLINEVAL_0, DMA_SYSCTRLSET_1, 0},    /* DMA_REQUEST_TIMER2, system registers shift left 0 bit set */
    {DMA_REQLINEVAL_1, DMA_SYSCTRLSET_1, 1}     /* DMA_REQUEST_TIMER3, system registers shift left 1 bit set */
};
static BASE_StatusType DMA_SetChannelAndDirection(DMA_Handle *dmaHandle, unsigned int srcAddr, unsigned int destAddr,
                                                  unsigned int dataLength, unsigned int channel);
static BASE_StatusType DMA_SetDirection(DMA_Handle *dmaHandle, unsigned int channel);
static BASE_StatusType DMA_SetChannel(DMA_Handle *dmaHandle, unsigned int srcAddr, unsigned int destAddr,
                                      unsigned int dataLength, unsigned int channel);

static void DMA_SplitToBlock(DMA_LinkList *node, DMA_SplitParam *split);
/**
  * @brief Initialize the DMA hardware controller configuration.
  * @param dmaHandle DMA handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_DMA_Init(DMA_Handle *dmaHandle)
{
    DMA_ASSERT_PARAM(dmaHandle != NULL);
    DMA_ASSERT_PARAM(IsDMAInstance(dmaHandle->baseAddress));
    DMA_PARAM_CHECK_WITH_RET(IsDmaByteOrder(dmaHandle->handleEx.srcByteOrder) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaByteOrder(dmaHandle->handleEx.destByteOrder) == true, BASE_STATUS_ERROR);
    /* Sets the byte sequence of master1 and master2 */
    dmaHandle->baseAddress->DMAC_CONFIG.BIT.m1_endianness = dmaHandle->handleEx.srcByteOrder;
    dmaHandle->baseAddress->DMAC_CONFIG.BIT.m2_endianness = dmaHandle->handleEx.destByteOrder;
    dmaHandle->baseAddress->DMAC_CONFIG.BIT.dmac_enable = BASE_CFG_ENABLE; /* Enable the DMAC controller */
    dmaHandle->baseAddress->DMAC_INT_ERR_CLR.reg |= 0x0F;
    dmaHandle->baseAddress->DMAC_INT_TC_CLR.reg |= 0x0F;
    dmaHandle->baseAddress->DMAC_SYNC.reg = 0x00;
    dmaHandle->baseAddress->DMAC_C0_CONFIG.reg = 0x00;
    dmaHandle->baseAddress->DMAC_C1_CONFIG.reg = 0x00;
    dmaHandle->baseAddress->DMAC_C2_CONFIG.reg = 0x00;
    dmaHandle->baseAddress->DMAC_C3_CONFIG.reg = 0x00;
    dmaHandle->DMA_Channels[0].channelAddr = DMA_CHANNEL0;  /* Setting the base Address of channel 0 registers */
    dmaHandle->DMA_Channels[1].channelAddr = DMA_CHANNEL1;  /* Setting the base Address of channel 1 registers */
    dmaHandle->DMA_Channels[2].channelAddr = DMA_CHANNEL2;  /* Setting the base Address of channel 2 registers */
    dmaHandle->DMA_Channels[3].channelAddr = DMA_CHANNEL3;  /* Setting the base Address of channel 3 registers */
    return BASE_STATUS_OK;
}

/**
  * @brief DeInitialize the DMA, close all channels.
  * @param dmaHandle DMA handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_DMA_Deinit(DMA_Handle *dmaHandle)
{
    DMA_ASSERT_PARAM(dmaHandle != NULL);
    DMA_ASSERT_PARAM(IsDMAInstance(dmaHandle->baseAddress));
    dmaHandle->baseAddress->DMAC_INT_ERR_CLR.reg |= 0x0F;
    dmaHandle->baseAddress->DMAC_INT_TC_CLR.reg |= 0x0F;
    dmaHandle->baseAddress->DMAC_C0_CONFIG.reg = 0x00;
    dmaHandle->baseAddress->DMAC_C1_CONFIG.reg = 0x00;
    dmaHandle->baseAddress->DMAC_C2_CONFIG.reg = 0x00;
    dmaHandle->baseAddress->DMAC_C3_CONFIG.reg = 0x00;
    dmaHandle->baseAddress->DMAC_CONFIG.BIT.dmac_enable = BASE_CFG_DISABLE;
    /* Clean callback */
    for (unsigned int i = 0; i < CHANNEL_MAX_NUM; i++) {
        dmaHandle->userCallBack.DMA_CallbackFuns[i].ChannelFinishCallBack = NULL;
        dmaHandle->userCallBack.DMA_CallbackFuns[i].ChannelErrorCallBack = NULL;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Return the specified DMA channel state.
  * @param dmaHandle DMA handle.
  * @param channel ID of the selected DMA channel @ref DMA_ChannelNum.
  * @retval Channel state: BASE_STATUS_BUSY, BASE_STATUS_OK.
  */
BASE_StatusType HAL_DMA_GetChannelState(DMA_Handle *dmaHandle, unsigned int channel)
{
    DMA_ASSERT_PARAM(dmaHandle != NULL);
    DMA_ASSERT_PARAM(IsDMAInstance(dmaHandle->baseAddress));
    DMA_PARAM_CHECK_WITH_RET(IsDmaChannelNum(channel) == true, BASE_STATUS_ERROR);
    unsigned int chns = dmaHandle->baseAddress->DMAC_ENABLED_CHNS.reg; /* Obtains the channel enabling status */
    unsigned int channelStatus = chns & (1 << channel);
    if (channelStatus == (uintptr_t)(1 << channel)) {
        return BASE_STATUS_BUSY;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Modifying DMA channel parameters.
  * @param dmaHandle DMA handle.
  * @param channelParam DMA specific channel handle.
  * @param channel ID of the selected DMA channel @ref DMA_ChannelNum.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_DMA_InitChannel(DMA_Handle *dmaHandle, DMA_ChannelParam *channelParam, unsigned int channel)
{
    DMA_ASSERT_PARAM(dmaHandle != NULL);
    DMA_ASSERT_PARAM(channelParam != NULL);
    DMA_ASSERT_PARAM(IsDMAInstance(dmaHandle->baseAddress));
    DMA_PARAM_CHECK_WITH_RET(IsDmaDirection(channelParam->direction) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaReqPeriph(channelParam->srcPeriph) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaReqPeriph(channelParam->destPeriph) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaWidth(channelParam->srcWidth) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaWidth(channelParam->destWidth) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaBurstLength(channelParam->srcBurst) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaBurstLength(channelParam->destBurst) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaAddrMode(channelParam->srcAddrInc) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaAddrMode(channelParam->destAddrInc) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaChannelNum(channel) == true, BASE_STATUS_ERROR);
    dmaHandle->DMA_Channels[channel].direction = channelParam->direction;
    dmaHandle->DMA_Channels[channel].srcPeriph = channelParam->srcPeriph;
    dmaHandle->DMA_Channels[channel].destPeriph = channelParam->destPeriph;
    dmaHandle->DMA_Channels[channel].srcWidth = channelParam->srcWidth;
    dmaHandle->DMA_Channels[channel].destWidth = channelParam->destWidth;
    dmaHandle->DMA_Channels[channel].srcBurst = channelParam->srcBurst;
    dmaHandle->DMA_Channels[channel].destBurst = channelParam->destBurst;
    dmaHandle->DMA_Channels[channel].srcAddrInc = channelParam->srcAddrInc;
    dmaHandle->DMA_Channels[channel].destAddrInc = channelParam->destAddrInc;
    dmaHandle->DMA_Channels[channel].pHandle = channelParam->pHandle;
    return BASE_STATUS_OK;
}

/**
  * @brief Configuring the DMA source device.
  * @param dmaHandle DMA handle.
  * @param channel ID of the selected DMA channel @ref DMA_ChannelNum.
  * @retval None.
  */
static void DMA_SetSrcPeriph(DMA_Handle *dmaHandle, unsigned int channel)
{
    unsigned int periphNum = dmaHandle->DMA_Channels[channel].srcPeriph;
    if (periphNum  >=  DMA_REQUEST_MEM) {
        return;
    }
    /* Set the source device channel request line */
    unsigned int reqVal = g_periphReqTable[periphNum].reqLineVal;
    unsigned int sysVal = g_periphReqTable[periphNum].sysctrVal;
    unsigned int bitNum = g_periphReqTable[periphNum].shiftLeft;
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONFIG.BIT.src_periph = reqVal;
    if (periphNum > DMA_REQUEST_UART1_TX) {
        SYSCTRL1->DMA_REQ_SEL.reg |= (sysVal << bitNum);
    }
}

/**
  * @brief Configuring the DMA destination device.
  * @param dmaHandle DMA handle.
  * @param channel ID of the selected DMA channel @ref DMA_ChannelNum.
  * @retval None.
  */
static void DMA_SetDestPeriph(DMA_Handle *dmaHandle, unsigned int channel)
{
    unsigned int periphNum = dmaHandle->DMA_Channels[channel].destPeriph;
    if (periphNum  >=  DMA_REQUEST_MEM) {
        return;
    }
    /* Set the channel request line of the destination device */
    unsigned int reqVal = g_periphReqTable[periphNum].reqLineVal;
    unsigned int sysVal = g_periphReqTable[periphNum].sysctrVal;
    unsigned int bitNum = g_periphReqTable[periphNum].shiftLeft;
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONFIG.BIT.dest_periph = reqVal;
    if (periphNum > DMA_REQUEST_UART1_TX) {
        SYSCTRL1->DMA_REQ_SEL.reg |= (sysVal << bitNum);
    }
}

/**
  * @brief Configuring the transmission direction of the DMA channel.
  * @param dmaHandle DMA handle.
  * @param channel ID of the selected DMA channel @ref DMA_ChannelNum.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
static BASE_StatusType DMA_SetDirection(DMA_Handle *dmaHandle, unsigned int channel)
{
    unsigned int direction = dmaHandle->DMA_Channels[channel].direction;
    DMA_PARAM_CHECK_WITH_RET(IsDmaDirection(direction) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaReqPeriph(dmaHandle->DMA_Channels[channel].srcPeriph) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaReqPeriph(dmaHandle->DMA_Channels[channel].destPeriph) == true, BASE_STATUS_ERROR);
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONFIG.reg = 0x0000;
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONFIG.BIT.flow_ctrl = direction;
    switch (direction) {
        case DMA_MEMORY_TO_PERIPH_BY_DMAC:
            DMA_SetDestPeriph(dmaHandle, channel);
            break;
        /* The transfer type is peripheral to memory, flow control is controlled by DMAC */
        case DMA_PERIPH_TO_MEMORY_BY_DMAC:
            DMA_SetSrcPeriph(dmaHandle, channel);
            break;
        case DMA_PERIPH_TO_PERIPH_BY_DMAC:
            DMA_SetSrcPeriph(dmaHandle, channel);
            DMA_SetDestPeriph(dmaHandle, channel);
            break;
        /* The transfer type is peripheral to peripheral, flow control is controlled by destination periphera */
        case DMA_PERIPH_TO_PERIPH_BY_DES:
            DMA_SetSrcPeriph(dmaHandle, channel);
            DMA_SetDestPeriph(dmaHandle, channel);
            break;
        case DMA_MEMORY_TO_PERIPH_BY_DES:
            DMA_SetDestPeriph(dmaHandle, channel);
            break;
        /* The transfer type is peripheral to memory, flow control is controlled by source periphera */
        case DMA_PERIPH_TO_MEMORY_BY_SRC:
            DMA_SetSrcPeriph(dmaHandle, channel);
            break;
        case DMA_PERIPH_TO_PERIPH_BY_SRC:
            DMA_SetSrcPeriph(dmaHandle, channel);
            DMA_SetDestPeriph(dmaHandle, channel);
            break;
        default:
            break;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Calculate the configured value based on the channel configuration parameters.
  * @param dmaHandle DMA handle.
  * @param channel ID of the selected DMA channel @ref DMA_ChannelNum.
  * @retval val Calculation result.
  */
static unsigned int DMA_CalControlval(DMA_Handle *dmaHandle, unsigned int channel)
{
    unsigned int val = 0x82000000;    /* 0x82000000 indicates src use master1, dest use master2, int_tc_enable is set */
    val |= (dmaHandle->DMA_Channels[channel].srcBurst) << 12;     /* Shift left by 12 bits for source burst */
    val |= (dmaHandle->DMA_Channels[channel].destBurst) << 15;    /* Shift left by 15 bits for destination burst */
    val |= (dmaHandle->DMA_Channels[channel].srcWidth) << 18;     /* Shift left by 18 bits for source width */
    val |= (dmaHandle->DMA_Channels[channel].destWidth) << 21;    /* Shift left by 21 bits for destination width */
    val |= (dmaHandle->DMA_Channels[channel].srcAddrInc) << 26;   /* Shift left by 26 bits for source address */
    val |= (dmaHandle->DMA_Channels[channel].destAddrInc) << 27;  /* Shift left by 27 bits for destination address */
    return val;
}

/**
  * @brief Configuring Segmentation Parameters.
  * @param dmaHandle DMA handle.
  * @param srcAddr Data source address.
  * @param destAddr Data destination address
  * @param dataLength Length of data to be transferred.
  * @param channel ID of the selected DMA channel @ref DMA_ChannelNum.
  * @retval None.
  */
static void DMA_ConfigureSplit(DMA_Handle *dmaHandle, unsigned int srcAddr,
                               unsigned int destAddr, unsigned int dataLength, unsigned int channel)
{
    unsigned int val = DMA_CalControlval(dmaHandle, channel);
    DMA_SplitParam split;
    split.chnParam = val; /* Setting channel parameters by val */
    split.srcAddr = srcAddr;
    split.destAddr = destAddr;
    split.srcIn = dmaHandle->DMA_Channels[channel].srcAddrInc * (1 << dmaHandle->DMA_Channels[channel].srcWidth);
    split.destIn = dmaHandle->DMA_Channels[channel].destAddrInc * (1 << dmaHandle->DMA_Channels[channel].destWidth);
    split.totalSize = dataLength;
    DMA_LinkList *head = &(g_listTable[g_listIndex]);
    g_listIndex++;
    head->lliNext = NULL;
    val |= TRANS_BLOCK; /* Set the size of the data to be transferred, TRANS_BLOCK is 4092 */
    head->control.reg = val;
    DMA_SplitToBlock(head, &split);
    /* After DMA_SplitToBlock return, head->control.reg[31] int_tc_enable is set 0 */
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONTROL.reg = head->control.reg;
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_LLI.reg = (uintptr_t)(void *)head->lliNext;
}

/**
  * @brief Configuring DMA channel and direction.
  * @param dmaHandle DMA handle.
  * @param srcAddr Data source address.
  * @param destAddr Data destination address
  * @param dataLength Length of data to be transferred.
  * @param channel ID of the selected DMA channel @ref DMA_ChannelNum.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
static BASE_StatusType DMA_SetChannelAndDirection(DMA_Handle *dmaHandle, unsigned int srcAddr, unsigned int destAddr,
                                                  unsigned int dataLength, unsigned int channel)
{
    DMA_PARAM_CHECK_WITH_RET(IsDmaValidAddress(srcAddr), BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaValidAddress(destAddr), BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(dataLength > 0, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaValidAddress((unsigned long long)srcAddr + (unsigned long long)dataLength),
                             BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaValidAddress((unsigned long long)destAddr + (unsigned long long)dataLength),
                             BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaChannelNum(channel) == true, BASE_STATUS_ERROR);
    if (HAL_DMA_GetChannelState(dmaHandle, channel) != BASE_STATUS_OK) {
        return BASE_STATUS_BUSY;
    }
    /* Indicates whether to clear the corresponding channel interrupt */
    dmaHandle->baseAddress->DMAC_INT_ERR_CLR.reg |= (1 << channel);
    dmaHandle->baseAddress->DMAC_INT_TC_CLR.reg |= (1 << channel);
    if (DMA_SetChannel(dmaHandle, srcAddr, destAddr, dataLength, channel) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    /* Setting channel direction */
    if (DMA_SetDirection(dmaHandle, channel) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Configuring DMA channel transmission parameters.
  * @param dmaHandle DMA handle.
  * @param srcAddr Data source address.
  * @param destAddr Data destination address
  * @param dataLength Length of data to be transferred.
  * @param channel ID of the selected DMA channel @ref DMA_ChannelNum.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
static BASE_StatusType DMA_SetChannel(DMA_Handle *dmaHandle, unsigned int srcAddr, unsigned int destAddr,
                                      unsigned int dataLength, unsigned int channel)
{
    DMA_PARAM_CHECK_WITH_RET(IsDmaWidth(dmaHandle->DMA_Channels[channel].srcWidth) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaWidth(dmaHandle->DMA_Channels[channel].destWidth) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaBurstLength(dmaHandle->DMA_Channels[channel].srcBurst) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaBurstLength(dmaHandle->DMA_Channels[channel].destBurst) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaAddrMode(dmaHandle->DMA_Channels[channel].srcAddrInc) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaAddrMode(dmaHandle->DMA_Channels[channel].destAddrInc) == true, BASE_STATUS_ERROR);
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_SRC_ADDR.reg = srcAddr;
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_DEST_ADDR.reg = destAddr;
    /* If the data size is greater than 4095, data needs to be transferred in blocks */
    if (dataLength > TRANSIZE_MAX) {
        if (g_listIndex >= LISTNODE_MAX) {
            return BASE_STATUS_ERROR;
        }
        DMA_ConfigureSplit(dmaHandle, srcAddr, destAddr, dataLength, channel);
    } else {
        unsigned int val = DMA_CalControlval(dmaHandle, channel);
        val |= dataLength;
        /* Configure the corresponding channel control parameters based on the value */
        dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONTROL.reg = val;
        dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_LLI.reg = 0x00;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief DMA start data transfer without interrupt enable.
  * @param dmaHandle DMA handle.
  * @param srcAddr Data source address.
  * @param destAddr Data destination address
  * @param dataLength Length of data to be transferred
  * @param channel DMA channel num @ref DMA_ChannelNum.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_DMA_Start(DMA_Handle *dmaHandle, unsigned int srcAddr,
                              unsigned int destAddr, unsigned int dataLength, unsigned int channel)
{
    DMA_ASSERT_PARAM(dmaHandle != NULL);
    DMA_ASSERT_PARAM(IsDMAInstance(dmaHandle->baseAddress));
    BASE_StatusType status;
    /* Setting channel parameter */
    status = DMA_SetChannelAndDirection(dmaHandle, srcAddr, destAddr, dataLength, channel);
    if (status != BASE_STATUS_OK) {
        return status;
    }
    /* Mask completion interrupts and error interrupts, enable channels */
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONFIG.reg &= ~(0x0000C000);
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONFIG.BIT.ch_en = BASE_CFG_ENABLE;
#ifdef BASE_DEFINE_DMA_QUICKSTART
    dmaHandle->DMA_Channels[channel].srcAddr = dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_SRC_ADDR.reg;
    dmaHandle->DMA_Channels[channel].destAddr = dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_DEST_ADDR.reg;
    dmaHandle->DMA_Channels[channel].controlVal = dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONTROL.reg;
    dmaHandle->DMA_Channels[channel].configVal = dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONFIG.reg;
#endif
    return BASE_STATUS_OK;
}

/**
  * @brief DMA start data transfer with interrupt enable.
  * @param dmaHandle DMA handle.
  * @param srcAddr Data source address.
  * @param destAddr Data destination address
  * @param dataLength Length of data to be transferred
  * @param channel DMA channel num @ref DMA_ChannelNum.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_DMA_StartIT(DMA_Handle *dmaHandle, unsigned int srcAddr,
                                unsigned int destAddr, unsigned int dataLength, unsigned int channel)
{
    DMA_ASSERT_PARAM(dmaHandle != NULL);
    DMA_ASSERT_PARAM(IsDMAInstance(dmaHandle->baseAddress));
    BASE_StatusType status;
    /* Setting channel parameter */
    status = DMA_SetChannelAndDirection(dmaHandle, srcAddr, destAddr, dataLength, channel);
    if (status != BASE_STATUS_OK) {
        return status;
    }
    /* Set tc_int_msk, err_int_msk, ch_en */
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONFIG.reg |= 0xC001;
#ifdef BASE_DEFINE_DMA_QUICKSTART
    dmaHandle->DMA_Channels[channel].srcAddr = dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_SRC_ADDR.reg;
    dmaHandle->DMA_Channels[channel].destAddr = dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_DEST_ADDR.reg;
    dmaHandle->DMA_Channels[channel].controlVal = dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONTROL.reg;
    dmaHandle->DMA_Channels[channel].configVal = dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONFIG.reg;
#endif
    return BASE_STATUS_OK;
}

/**
  * @brief DMA specified channel stops transporting.
  * @param dmaHandle DMA handle.
  * @param channel ID of the selected DMA channel @ref DMA_ChannelNum.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_DMA_StopChannel(DMA_Handle *dmaHandle, unsigned int channel)
{
    DMA_ASSERT_PARAM(dmaHandle != NULL);
    DMA_ASSERT_PARAM(IsDMAInstance(dmaHandle->baseAddress));
    DMA_PARAM_CHECK_WITH_RET(IsDmaChannelNum(channel) == true, BASE_STATUS_ERROR);
    /* Ignore subsequent DMA requests */
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONFIG.BIT.ch_halt = BASE_CFG_ENABLE;
    unsigned int active;
    /* Processes the remaining data in the channel FIFO */
    do {
        active = dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONFIG.BIT.ch_active;
    } while (active != 0);
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONFIG.BIT.ch_en = BASE_CFG_DISABLE;
    return BASE_STATUS_OK;
}

/**
  * @brief DMA specified channel transfer complete interrupt service processing function.
  * @param dmaHandle DMA handle.
  * @param channel ID of the selected DMA channel @ref DMA_ChannelNum.
  * @retval None.
  */
static void DMA_ChannelIrqHandlerTc(DMA_Handle *dmaHandle, unsigned int channel)
{
    unsigned int finishStatus = dmaHandle->baseAddress->DMAC_INT_TC_STAT.reg;
    if ((finishStatus & (1 << channel)) != 0) {
        dmaHandle->baseAddress->DMAC_INT_TC_CLR.reg |= (1 << channel); /* Clear channel tc interrupt */
        if (dmaHandle->userCallBack.DMA_CallbackFuns[channel].ChannelFinishCallBack != NULL) {
            dmaHandle->userCallBack.DMA_CallbackFuns[channel].ChannelFinishCallBack(
                dmaHandle->DMA_Channels[channel].pHandle);
        }
    }
    return;
}

/**
  * @brief DMA specified channel error interrupt service processing function.
  * @param dmaHandle DMA handle.
  * @param channel ID of the selected DMA channel @ref DMA_ChannelNum.
  * @retval None.
  */
static void DMA_ChannelIrqHandlerError(DMA_Handle *dmaHandle, unsigned int channel)
{
    unsigned int errorStatus = dmaHandle->baseAddress->DMAC_INT_ERR_STAT.reg;
    if ((errorStatus & (1 << channel)) != 0) {
        dmaHandle->baseAddress->DMAC_INT_ERR_CLR.reg |= (1 << channel); /* Clear channel err interrupt */
        if (dmaHandle->userCallBack.DMA_CallbackFuns[channel].ChannelErrorCallBack != NULL) {
            dmaHandle->userCallBack.DMA_CallbackFuns[channel].ChannelErrorCallBack(
                dmaHandle->DMA_Channels[channel].pHandle);
        }
    }
    return;
}

/**
  * @brief DMA transfer complete interrupt service processing function.
  * @param handle DMA handle.
  * @retval None.
  */
void HAL_DMA_IrqHandlerTc(void *handle)
{
    DMA_ASSERT_PARAM(handle != NULL);
    DMA_Handle *dmaHandle = (DMA_Handle *)handle;
    DMA_ASSERT_PARAM(IsDMAInstance(dmaHandle->baseAddress));
    unsigned int intStatus = dmaHandle->baseAddress->DMAC_INT_STAT.reg;
    for (int i = 0; i < CHANNEL_MAX_NUM; i++) {
        if (intStatus & (1 << i)) { /* DMAC channel status */
            DMA_ChannelIrqHandlerTc(dmaHandle, i);
        }
    }
    return;
}

/**
  * @brief DMA error interrupt service processing function.
  * @param handle DMA handle.
  * @retval None.
  */
void HAL_DMA_IrqHandlerError(void *handle)
{
    DMA_ASSERT_PARAM(handle != NULL);
    DMA_Handle *dmaHandle = (DMA_Handle *)handle;
    DMA_ASSERT_PARAM(IsDMAInstance(dmaHandle->baseAddress));
    unsigned int intStatus = dmaHandle->baseAddress->DMAC_INT_STAT.reg;
    for (int i = 0; i < CHANNEL_MAX_NUM; i++) {
        if (intStatus & (1 << i)) { /* DMAC channel status */
            DMA_ChannelIrqHandlerError(dmaHandle, i);
        }
    }
    return;
}

/**
  * @brief User callback function registration interface.
  * @param dmaHandle DMA handle.
  * @param typeID Id of callback function type.
  * @param channel ID of the selected DMA channel @ref DMA_ChannelNum.
  * @param pCallback pointer of the specified callbcak function.
  * @retval None.
  */
void HAL_DMA_RegisterCallback(DMA_Handle *dmaHandle, DMA_CallbackFun_Type typeID,
                              DMA_ChannelNum channel, DMA_CallbackType pCallback)
{
    DMA_ASSERT_PARAM(dmaHandle != NULL);
    DMA_ASSERT_PARAM(IsDMAInstance(dmaHandle->baseAddress));
    DMA_PARAM_CHECK_NO_RET(IsDmaChannelNum(channel) == true);
    switch (typeID) {
        case DMA_CHANNEL_FINISH:
            dmaHandle->userCallBack.DMA_CallbackFuns[channel].ChannelFinishCallBack = pCallback;
            break;
        case DMA_CHANNEL_ERROR:
            dmaHandle->userCallBack.DMA_CallbackFuns[channel].ChannelErrorCallBack = pCallback;
            break;
        default:
            return;
    }
}

/**
  * @brief Find the last node in the linked list.
  * @param head Pointer to the transfer header of the linked list.
  * @retval retNode End node of the linked list.
  */
static DMA_LinkList* DMA_FindListEndNode(DMA_LinkList *head)
{
    DMA_LinkList* retNode = head;
    while (retNode->lliNext != NULL) {
        retNode = retNode->lliNext;
    }
    return retNode;
}

/**
  * @brief Add a new node to the end of the linked list.
  * @param head Pointer to the transfer header of the linked list.
  * @param newNode Node to be added.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_DMA_ListAddNode(DMA_LinkList *head, DMA_LinkList *newNode)
{
    DMA_ASSERT_PARAM(head != NULL);
    DMA_ASSERT_PARAM(newNode != NULL);
    DMA_LinkList *node = NULL;
    node = DMA_FindListEndNode(head);
    if (node != NULL) {
        node->lliNext = newNode;
        node->control.BIT.int_tc_enable = 0x0; /* current node does not trigger the transfer completion interrupt */
        newNode->control.BIT.int_tc_enable = 0x01; /* current node trigger the transfer completion interrupt */
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Create a new node and add it to the end of the linked list.
  * @param head Linked blocked head node.
  * @param split Argument handle that splits into small blocks.
  * @param index Sequence number of the new node in the linked list.
  * @param controlVal Channel control parameters for the new node.
  * @retval None.
  */
static void DMA_CreateNode(DMA_LinkList *head, DMA_SplitParam *split, unsigned int index, unsigned int controlVal)
{
    if (g_listIndex >= LISTNODE_MAX) {
        return;
    }
    DMA_LinkList *newNode = &(g_listTable[g_listIndex]);
    g_listIndex++;
    newNode->srcAddr = split->srcAddr + (index * TRANS_BLOCK * split->srcIn);
    newNode->destAddr = split->destAddr + (index * TRANS_BLOCK * split->destIn);
    newNode->lliNext = NULL;
    newNode->control.reg = controlVal; /* Channel parameters configured for the node */
    HAL_DMA_ListAddNode(head, newNode);
}

/**
  * @brief The upper limit of a DMA transfer is TRANSIZE_MAX. If the upper limit is greater than this value,
  * the DMA needs to be divided into small blocks, and each small block is linked for transmission.
  * @param head Linked blocked head node.
  * @param split Argument handle that splits into small blocks.
  * @retval None.
  */
static void DMA_SplitToBlock(DMA_LinkList *head, DMA_SplitParam *split)
{
    unsigned int totalSize = split->totalSize;
    unsigned remainSize = totalSize % TRANS_BLOCK;
    unsigned int index, controlVal;
    for (index = 1; index < totalSize / TRANS_BLOCK; index++) { /* Block transfer based on the 4092 size */
        controlVal = split->chnParam;
        controlVal |= TRANS_BLOCK;
        DMA_CreateNode(head, split, index, controlVal);
    }
    if (remainSize != 0) { /* The remaining data size is less than 4092 */
        controlVal = split->chnParam;
        controlVal |= remainSize;
        DMA_CreateNode(head, split, index, controlVal);
    }
}

/**
  * @brief In DMA chain transmission, initialize each node.
  * @param node Node to be initialized.
  * @param param Channel transmission parameters.
  * @param srcAddr Transport source address of this node.
  * @param destAddr Transport destnation address of this node.
  * @param tranSize Data transmitted by this node.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_DMA_InitNewNode(DMA_LinkList *node, const DMA_ChannelParam *param,
                                    unsigned int srcAddr, unsigned int destAddr, unsigned int tranSize)
{
    DMA_ASSERT_PARAM(node != NULL);
    DMA_ASSERT_PARAM(param != NULL);
    DMA_PARAM_CHECK_WITH_RET(IsDmaValidAddress(srcAddr), BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaValidAddress(destAddr), BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(tranSize > 0, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaValidAddress((unsigned long long)srcAddr + (unsigned long long)tranSize),
                             BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaValidAddress((unsigned long long)destAddr + (unsigned long long)tranSize),
                             BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaBurstLength(param->srcBurst) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaBurstLength(param->destBurst) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaWidth(param->srcWidth) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaWidth(param->destWidth) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaAddrMode(param->srcAddrInc) == true, BASE_STATUS_ERROR);
    DMA_PARAM_CHECK_WITH_RET(IsDmaAddrMode(param->destAddrInc) == true, BASE_STATUS_ERROR);
    node->srcAddr = srcAddr;
    node->destAddr = destAddr;
    node->lliNext = NULL;
    unsigned int val = 0x82000000;          /* 0x82000000 indicates src use master1, dest use master2, int_tc_enable */
    val |= (param->srcBurst) << 12;         /* Shift left by 12 bits for source burst */
    val |= (param->destBurst) << 15;        /* Shift left by 15 bits for destination burst */
    val |= (param->srcWidth) << 18;         /* Shift left by 18 bits for source width */
    val |= (param->destWidth) << 21;        /* Shift left by 21 bits for destination width */
    val |= (param->srcAddrInc) << 26;       /* Shift left by 26 bits for source address */
    val |= (param->destAddrInc) << 27;      /* Shift left by 27 bits for destination address */
    if (tranSize > TRANSIZE_MAX) {
        DMA_SplitParam split;
        split.chnParam = val;
        split.srcAddr = srcAddr;
        split.destAddr = destAddr;
        /* Source and destnation address single increment size */
        split.srcIn = param->srcAddrInc * (1 << param->srcWidth);
        split.destIn = param->destAddrInc * (1 << param->destWidth);
        split.totalSize = tranSize;
        val |= TRANS_BLOCK;
        node->control.reg = val;
        DMA_SplitToBlock(node, &split); /* Shift left by 27 bits for destination address */
    } else {
        val |= tranSize;
        node->control.reg = val;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Start DMA chain transmission. Chain transfer, which is used to transfer data to discontinuous
  * address spaces in memory. After the transmission task of the last node is complete, an interrupt is reported.
  * @param dmaHandle DMA handle.
  * @param head Pointer to the transfer header of the linked list.
  * @param channel DMA channel num @ref DMA_ChannelNum.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_DMA_StartListTransfer(DMA_Handle *dmaHandle, DMA_LinkList *head, unsigned int channel)
{
    DMA_ASSERT_PARAM(dmaHandle != NULL);
    DMA_ASSERT_PARAM(head != NULL);
    DMA_ASSERT_PARAM(IsDMAInstance(dmaHandle->baseAddress));
    DMA_PARAM_CHECK_WITH_RET(IsDmaChannelNum(channel) == true, BASE_STATUS_ERROR);
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_SRC_ADDR.reg = head->srcAddr;
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_DEST_ADDR.reg = head->destAddr;
    if (head->lliNext != NULL) {
        /* Configure the next node address of the linked list */
        dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_LLI.reg = (uintptr_t)(void *)head->lliNext;
    } else {
        dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_LLI.reg = 0x00;
    }
    if (head->lliNext == head) {
        head->control.BIT.int_tc_enable = 0;  /* current node does not trigger the transfer completion interrupt */
    }
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONTROL.reg = head->control.reg;
    /* Set tc_int_msk, ch_en */
    DMA_SetDirection(dmaHandle, channel);
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONFIG.BIT.tc_int_msk = BASE_CFG_ENABLE;
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONFIG.BIT.ch_en = BASE_CFG_ENABLE;
    return BASE_STATUS_OK;
}

#ifdef BASE_DEFINE_DMA_QUICKSTART
/**
  * @brief DMA start data transfer without parameter verification Use the parameters of the last DMA configuration.
  * @param dmaHandle DMA handle.
  * @param channel DMA channel num @ref DMA_ChannelNum.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
void HAL_DMA_QuickStart(DMA_Handle *dmaHandle, unsigned int channel)
{
    DMA_ASSERT_PARAM(dmaHandle != NULL);
    DMA_ASSERT_PARAM(IsDMAInstance(dmaHandle->baseAddress));
    DMA_PARAM_CHECK_NO_RET(IsDmaChannelNum(channel));
    /* Readback value configuration channel parameters */
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_SRC_ADDR.reg = dmaHandle->DMA_Channels[channel].srcAddr;
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_DEST_ADDR.reg = dmaHandle->DMA_Channels[channel].destAddr;
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONTROL.reg = dmaHandle->DMA_Channels[channel].controlVal;
    dmaHandle->DMA_Channels[channel].channelAddr->DMAC_Cn_CONFIG.reg = dmaHandle->DMA_Channels[channel].configVal;
}
#endif