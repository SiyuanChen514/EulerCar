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

#define I2C_FREQ_SCR 100000
#define I2C_HOLD_DURATION 10

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

__weak void Stlm75TxCallbackHandle(I2C_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN Stlm75TxCallbackHandle */
    /* USER CODE END Stlm75TxCallbackHandle */
}

__weak void Stlm75RxCallbackHandle(I2C_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN Stlm75RxCallbackHandle */
    /* USER CODE END Stlm75RxCallbackHandle */
}

__weak void Stlm75ErrCallbackHandle(I2C_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN Stlm75ErrCallbackHandle */
    /* USER CODE END Stlm75ErrCallbackHandle */
}

static void I2C_Init(void)
{
    HAL_CRG_IpEnableSet(I2C_BASE, IP_CLK_ENABLE);

    g_i2cSampleHandle.baseAddress = I2C;
    g_i2cSampleHandle.irqNum = IRQ_I2C;

    g_i2cSampleHandle.addrMode = I2C_7_BITS;
    g_i2cSampleHandle.sdaHoldTime = I2C_HOLD_DURATION;
    g_i2cSampleHandle.freq = I2C_FREQ_SCR;
    g_i2cSampleHandle.rxBuff = NULL;
    g_i2cSampleHandle.txBuff = NULL;
    g_i2cSampleHandle.ignoreAckFlag =  BASE_CFG_DISABLE;
    g_i2cSampleHandle.msgStopFlag =  BASE_CFG_ENABLE;
    g_i2cSampleHandle.state = I2C_STATE_RESET;
    g_i2cSampleHandle.rxWaterMark = 32; /* Rx water mark is 32 */
    g_i2cSampleHandle.txWaterMark = 32; /* Tx water mark is 32 */
    HAL_I2C_Init(&g_i2cSampleHandle);

    HAL_I2C_RegisterCallback(&g_i2cSampleHandle, I2C_MASTER_TX_COMPLETE_CB_ID, Stlm75TxCallbackHandle);
    HAL_I2C_RegisterCallback(&g_i2cSampleHandle, I2C_MASTER_RX_COMPLETE_CB_ID, Stlm75RxCallbackHandle);
    HAL_I2C_RegisterCallback(&g_i2cSampleHandle, I2C_ERROR_CB_ID, Stlm75ErrCallbackHandle);
    HAL_I2C_IRQService(&g_i2cSampleHandle);
    IRQ_SetPriority(g_i2cSampleHandle.irqNum, 1);
    IRQ_EnableN(IRQ_I2C);
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

    iconfig->iocmg_14.BIT.func = 0x5; /* 0x5 is I2C0_SCL */
    iconfig->iocmg_14.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_14.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_14.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_14.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_14.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_15.BIT.func = 0x5; /* 0x5 is I2C0_SDA */
    iconfig->iocmg_15.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_15.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_15.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_15.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_15.BIT.se = BASE_CFG_DISABLE;

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
    I2C_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}