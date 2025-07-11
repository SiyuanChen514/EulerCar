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

static void CAPM0_Init(void)
{
    HAL_CRG_IpEnableSet(CAPM0_BASE, IP_CLK_ENABLE);

    g_capmAConfig.baseAddress = CAPM0;
    g_capmAConfig.evtIrqNum = IRQ_CAPM0;

    g_capmAConfig.deburrNum = 0; /* deburr value */
    g_capmAConfig.capMode = CAPM_CONTINUECAP; /* capture mode */
    g_capmAConfig.preScale = 1;
    g_capmAConfig.capRegConfig[0].capEvent = CAPM_RISING; /* capture rising edge */
    g_capmAConfig.capRegConfig[0].regReset = CAPM_RESET;
    g_capmAConfig.capRegConfig[1].capEvent = CAPM_FALLING; /* capture falling edge */
    g_capmAConfig.capRegConfig[1].regReset = CAPM_RESET;
    g_capmAConfig.useCapNum = 2; /* 2: use 2 ECR */
    g_capmAConfig.tscntDiv = 1 - 1;
    g_capmAConfig.inputSrc = CAPM_INPUT_SRC1; /* input source 1 */

    HAL_CAPM_Init(&g_capmAConfig);
}

static void CAPM1_Init(void)
{
    HAL_CRG_IpEnableSet(CAPM1_BASE, IP_CLK_ENABLE);

    g_capmBConfig.baseAddress = CAPM1;
    g_capmBConfig.evtIrqNum = IRQ_CAPM1;

    g_capmBConfig.deburrNum = 0; /* deburr value */
    g_capmBConfig.capMode = CAPM_CONTINUECAP; /* capture mode */
    g_capmBConfig.preScale = 1;
    g_capmBConfig.capRegConfig[0].capEvent = CAPM_RISING; /* capture rising edge */
    g_capmBConfig.capRegConfig[0].regReset = CAPM_RESET;
    g_capmBConfig.capRegConfig[1].capEvent = CAPM_FALLING; /* capture falling edge */
    g_capmBConfig.capRegConfig[1].regReset = CAPM_RESET;
    g_capmBConfig.useCapNum = 2; /* 2: use 2 ECR */
    g_capmBConfig.tscntDiv = 1 - 1;
    g_capmBConfig.inputSrc = CAPM_INPUT_SRC0; /* input source 0 */

    HAL_CAPM_Init(&g_capmBConfig);
}

static void CAPM2_Init(void)
{
    HAL_CRG_IpEnableSet(CAPM2_BASE, IP_CLK_ENABLE);

    g_capmCConfig.baseAddress = CAPM2;
    g_capmCConfig.evtIrqNum = IRQ_CAPM2;

    g_capmCConfig.deburrNum = 0; /* deburr value */
    g_capmCConfig.capMode = CAPM_CONTINUECAP; /* capture mode */
    g_capmCConfig.preScale = 1;
    g_capmCConfig.capRegConfig[0].capEvent = CAPM_RISING; /* capture rising edge */
    g_capmCConfig.capRegConfig[0].regReset = CAPM_RESET;
    g_capmCConfig.capRegConfig[1].capEvent = CAPM_FALLING; /* capture falling edge */
    g_capmCConfig.capRegConfig[1].regReset = CAPM_RESET;
    g_capmCConfig.useCapNum = 2; /* 2: use 2 ECR */
    g_capmCConfig.tscntDiv = 1 - 1;
    g_capmCConfig.inputSrc = CAPM_INPUT_SRC1; /* input source 1 */

    HAL_CAPM_Init(&g_capmCConfig);
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE); /* enable uart's clock */
    HAL_CRG_IpClkSelectSet(UART0_BASE, CRG_PLL_NO_PREDV);

    g_uart0.baseAddress = UART0; /* uart0  base address */
    g_uart0.irqNum = IRQ_UART0;

    g_uart0.baudRate = UART0_BAND_RATE; /* uart bandrate */
    g_uart0.dataLength = UART_DATALENGTH_8BIT;
    g_uart0.stopBits = UART_STOPBITS_ONE;
    g_uart0.parity = UART_PARITY_NONE;
    g_uart0.txMode = UART_MODE_BLOCKING; /* TX mode */
    g_uart0.rxMode = UART_MODE_BLOCKING; /* RX mode */
    g_uart0.fifoMode = BASE_CFG_ENABLE;
    g_uart0.fifoTxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.fifoRxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0);
}

static void IOConfig(void)
{
    IOConfig_RegStruct *iconfig = IOCONFIG;

    iconfig->iocmg_20.BIT.func = 0x3; /* 0x3 is CAPM0_SRC */
    iconfig->iocmg_20.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_20.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_20.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_20.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_20.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_8.BIT.func = 0x3; /* 0x3 is CAPM1_SRC */
    iconfig->iocmg_8.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_8.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_8.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_8.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_8.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_19.BIT.func = 0x3; /* 0x3 is CAPM2_SRC */
    iconfig->iocmg_19.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_19.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_19.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_19.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_19.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_6.BIT.func = 0x4; /* 0x4 is UART0_TXD */
    iconfig->iocmg_6.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_6.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_6.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_6.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_6.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_7.BIT.func = 0x4; /* 0x4 is UART0_RXD */
    iconfig->iocmg_7.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_7.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_7.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_7.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_7.BIT.se = BASE_CFG_DISABLE;
}

void SystemInit(void)
{
    IOConfig();
    CAPM0_Init();
    CAPM1_Init();
    CAPM2_Init();
    UART0_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}