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

static void GPT1_Init(void)
{
    HAL_CRG_IpEnableSet(GPT1_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(GPT1_BASE, CRG_PLL_NO_PREDV);

    g_gptHandle.baseAddress = GPT1;
    g_gptHandle.period      = 10000 * 1000;  /* Period counter, Period = 10000us, 1000ns in 1us */
    g_gptHandle.duty        = 10000 * 10 * 10; /* Duty counter, duty = 10000 / 10 us */
    g_gptHandle.pwmPolarity = BASE_CFG_ENABLE; /* Continuous output */
    g_gptHandle.pwmKeep     = BASE_CFG_ENABLE; /* Continuous output */
    HAL_GPT_Init(&g_gptHandle);
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART0_BASE, CRG_PLL_NO_PREDV);

    g_uart0Handle.baseAddress = UART0;
    g_uart0Handle.irqNum = IRQ_UART0;

    g_uart0Handle.baudRate = UART0_BAND_RATE;
    g_uart0Handle.dataLength = UART_DATALENGTH_8BIT;
    g_uart0Handle.stopBits = UART_STOPBITS_ONE;
    g_uart0Handle.parity = UART_PARITY_NONE;
    g_uart0Handle.txMode = UART_MODE_BLOCKING;
    g_uart0Handle.rxMode = UART_MODE_BLOCKING;
    g_uart0Handle.fifoMode = BASE_CFG_ENABLE;
    g_uart0Handle.fifoTxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0Handle.fifoRxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0Handle.hwFlowCtr = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0Handle);
}

static void IOConfig(void)
{
    IOConfig_RegStruct *iconfig = IOCONFIG;

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

    iconfig->iocmg_14.BIT.func = 0x2; /* 0x2 is GPT1_PWM */
    iconfig->iocmg_14.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_14.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_14.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_14.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_14.BIT.se = BASE_CFG_DISABLE;
}

void SystemInit(void)
{
    IOConfig();
    UART0_Init();
    GPT1_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}