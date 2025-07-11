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

static void DMA_Channel1Init(void *handle)
{
    DMA_ChannelParam dma_param;
    dma_param.direction = DMA_PERIPH_TO_MEMORY_BY_DMAC;  /* DMA transmission direction */
    dma_param.srcAddrInc = DMA_ADDR_UNALTERED;
    dma_param.destAddrInc = DMA_ADDR_UNALTERED;
    dma_param.srcPeriph = DMA_REQUEST_ADC1;   /* ADC as DMA Request Source */
    dma_param.destPeriph = DMA_REQUEST_MEM;
    dma_param.srcWidth = DMA_TRANSWIDTH_BYTE;
    dma_param.destWidth = DMA_TRANSWIDTH_BYTE;
    dma_param.srcBurst = DMA_BURST_LENGTH_1;
    dma_param.destBurst = DMA_BURST_LENGTH_1;
    dma_param.pHandle = handle;
    HAL_DMA_InitChannel(&g_dmac, &dma_param, DMA_CHANNEL_ONE);   /* Initializing the DMA Channel */
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
    HAL_DMA_Init(&g_dmac);  /* Initializing the DMA controller */

    DMA_Channel1Init((void *)(&g_adc));
}

__weak void ADC1DMACallback(ADC_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN ADC1DMACallback */
    /* USER CODE END ADC1DMACallback */
}

static void ADC1_Init(void)
{
    HAL_CRG_IpEnableSet(ADC1_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(ADC1_BASE, CRG_ADC_CLK_SELECT_PLL_DIV);
    HAL_CRG_IpClkDivSet(ADC1_BASE, CRG_ADC_DIV_5);

    g_adc.baseAddress = ADC1;
    g_adc.socPriority = ADC_PRIMODE_ALL_ROUND;
    g_adc.vrefBuf = ADC_VREF_2P5V;
    g_adc.irqNumOver = IRQ_ADC1_OVINT;
    g_adc.ADC_IntxParam[0].irqNum = IRQ_ADC1_INT1;     /* interrupt 0 */
    g_adc.ADC_IntxParam[1].irqNum = IRQ_ADC1_INT2;     /* interrupt 1 */
    g_adc.ADC_IntxParam[2].irqNum = IRQ_ADC1_INT3;     /* interrupt 2 */
    g_adc.ADC_IntxParam[3].irqNum = IRQ_ADC1_INT4;     /* interrupt 3 */

    g_adc.dmaHandle = &g_dmac;
    g_adc.adcDmaChn = 1; /* DMA Channel 1 */
    HAL_ADC_RegisterCallBack(&g_adc, ADC_CALLBACK_DMA, ADC1DMACallback);

    HAL_ADC_Init(&g_adc);

    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINA1;

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 127; /* adc sample total time 127 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_SOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_NONEPERIPH;
    socParam.finishMode = ADC_SOCFINISH_DMA;
    HAL_ADC_ConfigureSoc(&g_adc, ADC_SOC_NUM1, &socParam);
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

    iconfig->iocmg_21.BIT.func = 0x8; /* 0x8 is ADC1_ANA_A1 */
    iconfig->iocmg_21.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_21.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_21.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_21.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_21.BIT.se = BASE_CFG_DISABLE;

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
    DMA_Init();
    ADC1_Init();
    UART0_Init();
    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}