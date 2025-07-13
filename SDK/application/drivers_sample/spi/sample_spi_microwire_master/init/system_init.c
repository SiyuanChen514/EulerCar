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

static void SPI_Init(void)
{
    HAL_CRG_IpEnableSet(SPI_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(SPI_BASE, CRG_PLL_NO_PREDV);

    g_spiSampleHandle.baseAddress = SPI;
    g_spiSampleHandle.irqNum = IRQ_SPI;

    g_spiSampleHandle.mode = HAL_SPI_MASTER;
    g_spiSampleHandle.csMode = SPI_CHIP_SELECT_MODE_INTERNAL;
    g_spiSampleHandle.xFerMode = HAL_XFER_MODE_BLOCKING;
    g_spiSampleHandle.clkPolarity = 0;
    g_spiSampleHandle.clkPhase = 0;
    g_spiSampleHandle.endian = HAL_SPI_BIG_ENDIAN;
    g_spiSampleHandle.frameFormat = HAL_SPI_MODE_MICROWIRE;
    g_spiSampleHandle.dataWidth = SPI_DATA_WIDTH_8BIT;
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

    HAL_SPI_Init(&g_spiSampleHandle);
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
}

void SystemInit(void)
{
    IOConfig();
    SPI_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}