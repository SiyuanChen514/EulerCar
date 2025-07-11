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
#define UART1_BAND_RATE 800
#define UART2_BAND_RATE 800

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect)
{
    CRG_Handle crg;
    crg.baseAddress     = CRG;
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_HOSC;
    crg.pllPreDiv       = CRG_PLL_PREDIV_4;
    crg.pllFbDiv        = 32; /* PLL Multiplier 32 */
    crg.pllPostDiv      = CRG_PLL_POSTDIV_1;
    crg.coreClkSelect   = CRG_CORE_CLK_SELECT_PLL;
    /* USER CODE BEGIN CRG ITCallBackFunc */
    /* USER CODE END CRG ITCallBackFunc */
    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    /* USER CODE BEGIN crg system_init */
    /* USER CODE END crg system_init */
    return BASE_STATUS_OK;
}

__weak void TIMER2CallbackFunction(void *handle)
{
    HAL_TIMER_IrqClear((TIMER_Handle *)handle);
    /* USER CODE BEGIN TIMER2 ITCallBackFunc */
    /* USER CODE END TIMER2 ITCallBackFunc */
}
static void TIMER2_Init(void)
{
    unsigned int load = HAL_CRG_GetIpFreq((void *)TIMER2) / 1000000u * 500;  /* 500us timer counting period. */

    HAL_CRG_IpEnableSet(TIMER2_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(TIMER2_BASE, CRG_PLL_NO_PREDV);

    g_timer2.baseAddress = TIMER2;
    g_timer2.irqNum = IRQ_TIMER2;
    /* USER CODE BEGIN timer2 system_init */
    /* USER CODE END timer2 system_init */
    g_timer2.load        = load - 1; /* Set timer value immediately */
    g_timer2.bgLoad      = load - 1; /* Set timer value */
    g_timer2.mode        = TIMER_MODE_RUN_PERIODIC; /* Run in period mode */
    g_timer2.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer2.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer2.dmaAdcSingleReqEnable = BASE_CFG_DISABLE;
    g_timer2.dmaBurstReqEnable = BASE_CFG_DISABLE;
    g_timer2.interruptEn = BASE_CFG_ENABLE;
    HAL_TIMER_Init(&g_timer2);
    /* USER CODE BEGIN timer2 system_init */
    /* USER CODE END timer2 system_init */
    HAL_TIMER_RegisterCallback(&g_timer2, TIMER2CallbackFunction);  /* Timer callback funtion register. */
    IRQ_SetPriority(g_timer2.irqNum, 1);
    IRQ_EnableN(g_timer2.irqNum);
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART0_BASE, CRG_PLL_NO_PREDV);
    /* USER CODE BEGIN uart0 system_init */
    /* USER CODE END uart0 system_init */
    g_uart0.baseAddress = UART0;
    g_uart0.irqNum = IRQ_UART0;
    /* USER CODE BEGIN uart0 system_init */
    /* USER CODE END uart0 system_init */
    g_uart0.baudRate = UART0_BAND_RATE;
    g_uart0.dataLength = UART_DATALENGTH_8BIT;
    g_uart0.stopBits = UART_STOPBITS_ONE;
    g_uart0.parity = UART_PARITY_NONE;
    g_uart0.txMode = UART_MODE_BLOCKING;
    g_uart0.rxMode = UART_MODE_BLOCKING;
    g_uart0.fifoMode = BASE_CFG_ENABLE;   /* FIFO enable. */
    g_uart0.fifoTxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.fifoRxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0);
    /* USER CODE BEGIN uart0 system_init */
    /* USER CODE END uart0 system_init */
}

__weak void UART1WriteInterruptCallback(UART_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART1_WRITE_IT_FINISH */
    /* USER CODE END UART1_WRITE_IT_FINISH */
}

__weak void UART1ReadInterruptCallback(UART_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART1_READ_IT_FINISH */
    /* USER CODE END UART1_READ_IT_FINISH */
}

__weak void UART1InterruptErrorCallback(UART_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART1_TRNS_IT_ERROR */
    /* USER CODE END UART1_TRNS_IT_ERROR */
}

static void UART1_Init(void)
{
    HAL_CRG_IpEnableSet(UART1_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART1_BASE, CRG_PLL_NO_PREDV);
    /* USER CODE BEGIN uart1 system_init */
    /* USER CODE END uart1 system_init */
    g_uart1.baseAddress = UART1;
    g_uart1.irqNum = IRQ_UART1;

    g_uart1.baudRate = UART1_BAND_RATE;    /* Baud rate setting. */
    g_uart1.dataLength = UART_DATALENGTH_8BIT;
    g_uart1.stopBits = UART_STOPBITS_ONE;
    g_uart1.parity = UART_PARITY_NONE;
    g_uart1.txMode = UART_MODE_INTERRUPT;
    g_uart1.rxMode = UART_MODE_INTERRUPT;
    g_uart1.fifoMode = BASE_CFG_DISABLE;    /* FIFO disable. */
    g_uart1.fifoTxThr = UART_FIFOFULL_ONE_TWO;
    g_uart1.fifoRxThr = UART_FIFOFULL_ONE_TWO;
    g_uart1.hwFlowCtr = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart1);
    /* UART1 user callback register. */
    HAL_UART_RegisterCallBack(&g_uart1, UART_WRITE_IT_FINISH, UART1WriteInterruptCallback);
    HAL_UART_RegisterCallBack(&g_uart1, UART_READ_IT_FINISH, UART1ReadInterruptCallback);
    HAL_UART_RegisterCallBack(&g_uart1, UART_TRNS_IT_ERROR, UART1InterruptErrorCallback);
    HAL_UART_IRQService(&g_uart1);      /* Interrupt Enable */
    IRQ_SetPriority(g_uart1.irqNum, 1);
    IRQ_EnableN(g_uart1.irqNum);
}

__weak void UART2WriteInterruptCallback(UART_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART2_WRITE_IT_FINISH */
    /* USER CODE END UART2_WRITE_IT_FINISH */
}

__weak void UART2ReadInterruptCallback(UART_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART2_READ_IT_FINISH */
    /* USER CODE END UART2_READ_IT_FINISH */
}

__weak void UART2InterruptErrorCallback(UART_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART2_TRNS_IT_ERROR */
    /* USER CODE END UART2_TRNS_IT_ERROR */
}

static void UART2_Init(void)
{
    HAL_CRG_IpEnableSet(UART2_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART2_BASE, CRG_PLL_NO_PREDV);
    /* USER CODE BEGIN uart2 system_init */
    /* USER CODE END uart3 system_init */
    g_uart2.baseAddress = UART2;
    g_uart2.irqNum = IRQ_UART2;

    g_uart2.baudRate = UART2_BAND_RATE;
    g_uart2.dataLength = UART_DATALENGTH_8BIT;
    g_uart2.stopBits = UART_STOPBITS_ONE;
    g_uart2.parity = UART_PARITY_NONE;
    g_uart2.txMode = UART_MODE_INTERRUPT;
    g_uart2.rxMode = UART_MODE_INTERRUPT;
    g_uart2.fifoMode = BASE_CFG_DISABLE;  /* FIFO disable. */
    g_uart2.fifoTxThr = UART_FIFOFULL_ONE_TWO;
    g_uart2.fifoRxThr = UART_FIFOFULL_ONE_TWO;
    g_uart2.hwFlowCtr = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart2);
    /* UART2 user callback register  */
    HAL_UART_RegisterCallBack(&g_uart2, UART_WRITE_IT_FINISH, UART2WriteInterruptCallback);
    HAL_UART_RegisterCallBack(&g_uart2, UART_READ_IT_FINISH, UART2ReadInterruptCallback);
    HAL_UART_RegisterCallBack(&g_uart2, UART_TRNS_IT_ERROR, UART2InterruptErrorCallback);
    /* USER CODE BEGIN uart2 system_init */
    /* USER CODE END uart2 system_init */
    HAL_UART_IRQService(&g_uart2);
    IRQ_SetPriority(g_uart2.irqNum, 1);
    IRQ_EnableN(g_uart2.irqNum);
}

void IOConfig(void)
{
    IOConfig_RegStruct *iconfig = IOCONFIG;    /* Handle of ioconfig. */
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode = 0;
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode_clear = 1;

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

    iconfig->iocmg_15.BIT.func = 0x4; /* 0x4 is UART1_TXD */
    iconfig->iocmg_15.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_15.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_15.BIT.pu = BASE_CFG_ENABLE;                /* Pull-up resistor of UART1_Tx.  */
    iconfig->iocmg_15.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_15.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_14.BIT.func = 0x4; /* 0x4 is UART1_RXD */
    iconfig->iocmg_14.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_14.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_14.BIT.pu = BASE_CFG_ENABLE;               /* Pull-up resistor of UART1_Rx.  */
    iconfig->iocmg_14.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_14.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_51.BIT.func = 0x3; /* 0x3 is UART2_TXD */
    iconfig->iocmg_51.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_51.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_51.BIT.pu = BASE_CFG_ENABLE;               /* Pull-up resistor of UART1_Tx.  */
    iconfig->iocmg_51.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_51.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_52.BIT.func = 0x3; /* 0x3 is UART2_RXD */
    iconfig->iocmg_52.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_52.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_52.BIT.pu = BASE_CFG_ENABLE;               /* Pull-up resistor of UART2_Rx.  */
    iconfig->iocmg_52.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_52.BIT.se = BASE_CFG_DISABLE;
}

void SystemInit(void)
{
    IOConfig();
    UART0_Init();
    UART1_Init();
    UART2_Init();
    TIMER2_Init();
    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}