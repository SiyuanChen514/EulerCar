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
#include "iocmg.h"

#define UART0_BAND_RATE 115200

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect)
{
    CRG_Handle crg;
    crg.baseAddress     = CRG;
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_HOSC;
    crg.pllPreDiv       = CRG_PLL_PREDIV_4;
    crg.pllFbDiv        = 48; /* PLL Multiplier 48 */
    crg.pllPostDiv      = CRG_PLL_POSTDIV_2;
    crg.coreClkSelect   = CRG_CORE_CLK_SELECT_HOSC;
    crg.handleEx.clk1MSelect   = CRG_1M_CLK_SELECT_HOSC;
    crg.handleEx.pllPostDiv2   = CRG_PLL_POSTDIV2_3;

    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
}

static void DMA_Channel3Init(void *handle)
{
    DMA_ChannelParam dma_param;
    dma_param.direction = DMA_MEMORY_TO_PERIPH_BY_DMAC;
    dma_param.srcAddrInc = DMA_ADDR_INCREASE;
    dma_param.destAddrInc = DMA_ADDR_UNALTERED;
    dma_param.srcPeriph = DMA_REQUEST_MEM;
    dma_param.destPeriph = DMA_REQUEST_UART0_TX;
    dma_param.srcWidth = DMA_TRANSWIDTH_BYTE;
    dma_param.destWidth = DMA_TRANSWIDTH_BYTE;
    dma_param.srcBurst = DMA_BURST_LENGTH_1;
    dma_param.destBurst = DMA_BURST_LENGTH_1;
    dma_param.pHandle = handle;
    HAL_DMA_InitChannel(&g_dmac, &dma_param, DMA_CHANNEL_THREE);
}

static void DMA_Channel2Init(void *handle)
{
    DMA_ChannelParam dma_param;
    dma_param.direction = DMA_PERIPH_TO_MEMORY_BY_DMAC;
    dma_param.srcAddrInc = DMA_ADDR_UNALTERED;
    dma_param.destAddrInc = DMA_ADDR_INCREASE;
    dma_param.srcPeriph = DMA_REQUEST_UART0_RX;
    dma_param.destPeriph = DMA_REQUEST_MEM;
    dma_param.srcWidth = DMA_TRANSWIDTH_BYTE;
    dma_param.destWidth = DMA_TRANSWIDTH_BYTE;
    dma_param.srcBurst = DMA_BURST_LENGTH_1;
    dma_param.destBurst = DMA_BURST_LENGTH_1;
    dma_param.pHandle = handle;
    HAL_DMA_InitChannel(&g_dmac, &dma_param, DMA_CHANNEL_TWO);
}

static void DMA_Init(void)
{
    HAL_CRG_IpEnableSet(DMA_BASE, IP_CLK_ENABLE);
    g_dmac.baseAddress = DMA;
    IRQ_Register(IRQ_DMA_TC, HAL_DMA_IrqHandlerTc, &g_dmac);
    IRQ_Register(IRQ_DMA_ERR, HAL_DMA_IrqHandlerError, &g_dmac);
    IRQ_EnableN(IRQ_DMA_TC);
    IRQ_EnableN(IRQ_DMA_ERR);
    HAL_DMA_Init(&g_dmac);
    DMA_Channel3Init((void *)(&g_uart));
    HAL_DMA_SetChannelPriorityEx(&g_dmac, DMA_CHANNEL_THREE, DMA_PRIORITY_LOW);
    DMA_Channel2Init((void *)(&g_uart));
    HAL_DMA_SetChannelPriorityEx(&g_dmac, DMA_CHANNEL_TWO, DMA_PRIORITY_MEDIUM);
}

__weak void DMA_Channel3CallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART0_WRITE_DMA_FINISH */
    /* USER CODE END UART0_WRITE_DMA_FINISH */
}

__weak void DMA_Channel2CallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART0_READ_DMA_FINISH */
    /* USER CODE END UART0_READ_DMA_FINISH */
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART0_BASE, CRG_AHB_CLK_NO_PREDV);

    g_uart.baseAddress = UART0;

    g_uart.baudRate = UART0_BAND_RATE;
    g_uart.dataLength = UART_DATALENGTH_8BIT;
    g_uart.stopBits = UART_STOPBITS_ONE;
    g_uart.parity = UART_PARITY_NONE;
    g_uart.txMode = UART_MODE_DMA;
    g_uart.rxMode = UART_MODE_DMA;
    g_uart.fifoMode = BASE_CFG_ENABLE;
    g_uart.fifoTxThr = UART_FIFODEPTH_SIZE8;
    g_uart.fifoRxThr = UART_FIFODEPTH_SIZE8;
    g_uart.hwFlowCtr = BASE_CFG_DISABLE;
    g_uart.handleEx.overSampleMultiple = UART_OVERSAMPLING_16X;
    g_uart.handleEx.msbFirst = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart);

    g_uart.dmaHandle = &g_dmac;
    g_uart.uartDmaTxChn = 3;      /* Rx Dma Channel is 3. */
    HAL_UART_RegisterCallBack(&g_uart, UART_WRITE_DMA_FINISH, DMA_Channel3CallBack);
    g_uart.uartDmaRxChn = 2;   /* DMA channel is 2. */
    HAL_UART_RegisterCallBack(&g_uart, UART_READ_DMA_FINISH, DMA_Channel2CallBack);
}

static void IOConfig(void)
{
    HAL_IOCMG_SetPinAltFuncMode(GPIO2_2_AS_UART0_TXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_2_AS_UART0_TXD, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_2_AS_UART0_TXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_2_AS_UART0_TXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_2_AS_UART0_TXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO2_3_AS_UART0_RXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_3_AS_UART0_RXD, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_3_AS_UART0_RXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_3_AS_UART0_RXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_3_AS_UART0_RXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */
}

void SystemInit(void)
{
    IOConfig();
    DMA_Init();
    UART0_Init();
    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}