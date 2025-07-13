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
  * @file      system_init.c
  * @author    MCU Driver Team
  * @brief     This file contains driver init functions.
  */

#include "main.h"
#include "ioconfig.h"

#define UART0_BAND_RATE 115200

#define SPI_FREQ_SCR 2
#define SPI_FREQ_CPSDVSR 10

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect)
{
    CRG_Handle crg;
    crg.baseAddress     = CRG;
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_HOSC;
    crg.pllPreDiv       = CRG_PLL_PREDIV_4;
    crg.pllFbDiv        = 32; /* PLL Multiplier 32 */
    crg.pllPostDiv      = CRG_PLL_POSTDIV_1;
    crg.coreClkSelect   = CRG_CORE_CLK_SELECT_PLL;
    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
}

static void DMA_Channel3Init(void *handle)
{
    DMA_ChannelParam dma_param;
    dma_param.direction = DMA_PERIPH_TO_MEMORY_BY_DMAC;
    dma_param.srcAddrInc = DMA_ADDR_UNALTERED;
    dma_param.destAddrInc = DMA_ADDR_UNALTERED;
    dma_param.srcPeriph = DMA_REQUEST_SPI_RX;
    dma_param.destPeriph = DMA_REQUEST_MEM;
    dma_param.srcWidth = DMA_TRANSWIDTH_BYTE;
    dma_param.destWidth = DMA_TRANSWIDTH_BYTE;
    dma_param.srcBurst = DMA_BURST_LENGTH_1;
    dma_param.destBurst = DMA_BURST_LENGTH_1;
    dma_param.pHandle = handle;
    HAL_DMA_InitChannel(&g_dmac, &dma_param, 3); /* Channel is 3 */
}

static void DMA_Channel2Init(void *handle)
{
    DMA_ChannelParam dma_param;
    dma_param.direction = DMA_MEMORY_TO_PERIPH_BY_DMAC;
    dma_param.srcAddrInc = DMA_ADDR_UNALTERED;
    dma_param.destAddrInc = DMA_ADDR_UNALTERED;
    dma_param.srcPeriph = DMA_REQUEST_MEM;
    dma_param.destPeriph = DMA_REQUEST_SPI_TX;
    dma_param.srcWidth = DMA_TRANSWIDTH_BYTE;
    dma_param.destWidth = DMA_TRANSWIDTH_BYTE;
    dma_param.srcBurst = DMA_BURST_LENGTH_1;
    dma_param.destBurst = DMA_BURST_LENGTH_1;
    dma_param.pHandle = handle;
    HAL_DMA_InitChannel(&g_dmac, &dma_param, 2); /* Channel is 2 */
}

static void DMA_Init(void)
{
    g_dmac.baseAddress = DMA;
    g_dmac.srcByteOrder = DMA_BYTEORDER_SMALLENDIAN;
    g_dmac.destByteOrder = DMA_BYTEORDER_SMALLENDIAN;
    g_dmac.irqNumTc = IRQ_DMA_TC;
    g_dmac.irqNumError = IRQ_DMA_ERR;
    HAL_DMA_IRQService(&g_dmac);
    IRQ_EnableN(IRQ_DMA_TC);
    IRQ_EnableN(IRQ_DMA_ERR);
    HAL_DMA_Init(&g_dmac);

    DMA_Channel3Init((void *)(&g_spiSampleHandle));
    DMA_Channel2Init((void *)(&g_spiSampleHandle));
}

__weak void TxSampleCallbackHandle(SPI_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN TxSampleCallbackHandle */
    /* USER CODE END TxSampleCallbackHandle */
}

__weak void RxSampleCallbackHandle(SPI_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN RxSampleCallbackHandle */
    /* USER CODE END RxSampleCallbackHandle */
}

__weak void TxRxSampleCallbackHandle(SPI_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN TxRxSampleCallbackHandle */
    /* USER CODE END TxRxSampleCallbackHandle */
}

__weak void ErrorSampleCallbackHandle(SPI_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN ErrorSampleCallbackHandle */
    /* USER CODE END ErrorSampleCallbackHandle */
}

__weak void SPICsCallback(SPI_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN SPICsCallback */
    /* USER CODE END SPICsCallback */
}

static void SPI_Init(void)
{
    HAL_CRG_IpEnableSet(SPI_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(SPI_BASE, CRG_PLL_NO_PREDV);

    g_spiSampleHandle.baseAddress = SPI;
    g_spiSampleHandle.irqNum = IRQ_SPI;

    g_spiSampleHandle.mode = HAL_SPI_MASTER;
    g_spiSampleHandle.csMode = SPI_CHIP_SELECT_MODE_INTERNAL;
    g_spiSampleHandle.xFerMode = HAL_XFER_MODE_DMA;
    g_spiSampleHandle.clkPolarity = 0;
    g_spiSampleHandle.clkPhase = 0;
    g_spiSampleHandle.endian = HAL_SPI_BIG_ENDIAN;
    g_spiSampleHandle.frameFormat = HAL_SPI_MODE_MOTOROLA;
    g_spiSampleHandle.dataWidth = SPI_DATA_WIDTH_16BIT;
    g_spiSampleHandle.freqScr = SPI_FREQ_SCR;
    g_spiSampleHandle.freqCpsdvsr = SPI_FREQ_CPSDVSR;
    g_spiSampleHandle.waitVal = 127; /* Wait time is 127 clock */
    g_spiSampleHandle.rxBuff = NULL;
    g_spiSampleHandle.txBuff = NULL;
    g_spiSampleHandle.transferSize = 0;
    g_spiSampleHandle.txCount = 0;
    g_spiSampleHandle.rxCount = 0;
    g_spiSampleHandle.state = HAL_SPI_STATE_RESET;
    g_spiSampleHandle.rxIntSize =  SPI_RX_INTERRUPT_SIZE_1;
    g_spiSampleHandle.txIntSize =  SPI_TX_INTERRUPT_SIZE_1;
    g_spiSampleHandle.rxDMABurstSize =  SPI_RX_DMA_BURST_SIZE_1;
    g_spiSampleHandle.txDMABurstSize =  SPI_TX_DMA_BURST_SIZE_1;
    g_spiSampleHandle.dmaHandle = &g_dmac;
    g_spiSampleHandle.txDmaCh = 2; /* Channel 2 */
    g_spiSampleHandle.rxDmaCh = 3; /* Channel 3 */

    HAL_SPI_Init(&g_spiSampleHandle);

    HAL_SPI_RegisterCallback(&g_spiSampleHandle, SPI_TX_COMPLETE_CB_ID, TxSampleCallbackHandle);
    HAL_SPI_RegisterCallback(&g_spiSampleHandle, SPI_RX_COMPLETE_CB_ID, RxSampleCallbackHandle);
    HAL_SPI_RegisterCallback(&g_spiSampleHandle, SPI_TX_RX_COMPLETE_CB_ID, TxRxSampleCallbackHandle);
    HAL_SPI_RegisterCallback(&g_spiSampleHandle, SPI_ERROR_CB_ID, ErrorSampleCallbackHandle);
    HAL_SPI_RegisterCallback(&g_spiSampleHandle, SPI_CS_CB_ID, SPICsCallback);
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART0_BASE, CRG_PLL_NO_PREDV);

    g_uart0.baseAddress = UART0;
    g_uart0.irqNum = IRQ_UART0;

    g_uart0.baudRate = UART0_BAND_RATE;
    g_uart0.dataLength = UART_DATALENGTH_8BIT;
    g_uart0.stopBits = UART_STOPBITS_ONE;
    g_uart0.parity = UART_PARITY_NONE;
    g_uart0.txMode = UART_MODE_BLOCKING;
    g_uart0.rxMode = UART_MODE_BLOCKING;
    g_uart0.fifoMode = BASE_CFG_ENABLE;
    g_uart0.fifoTxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.fifoRxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0);
}

static void IOConfig(void)
{
    IOConfig_RegStruct *iconfig = IOCONFIG;

    iconfig->iocmg_22.BIT.func = 0x5; /* 0x5 is SPI_CLK */
    iconfig->iocmg_22.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_22.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_22.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_22.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_22.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_18.BIT.func = 0x5; /* 0x5 is SPI_CSN0 */
    iconfig->iocmg_18.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_18.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_18.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_18.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_18.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_21.BIT.func = 0x5; /* 0x5 is SPI_CSN1 */
    iconfig->iocmg_21.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_21.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_21.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_21.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_21.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_20.BIT.func = 0x5; /* 0x5 is SPI_RXD */
    iconfig->iocmg_20.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_20.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_20.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_20.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_20.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_19.BIT.func = 0x5; /* 0x5 is SPI_TXD */
    iconfig->iocmg_19.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_19.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_19.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_19.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_19.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_7.BIT.func = 0x4; /* 0x4 is UART0_RXD */
    iconfig->iocmg_7.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_7.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_7.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_7.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_7.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_6.BIT.func = 0x4; /* 0x4 is UART0_TXD */
    iconfig->iocmg_6.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_6.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_6.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_6.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_6.BIT.se = BASE_CFG_DISABLE;
}

void SystemInit(void)
{
    IOConfig();
    UART0_Init();
    DMA_Init();
    SPI_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}