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
#include "iocmg_ip.h"

#define UART0_BAND_RATE 115200

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect)
{
    /* Clock-related parameter configuration. */
    CRG_Handle crg;
    crg.baseAddress     = CRG;
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_HOSC;
    crg.pllPreDiv       = CRG_PLL_PREDIV_4;
    crg.pllFbDiv        = 32; /* PLL Multiplier 32 */
    crg.pllPostDiv      = CRG_PLL_POSTDIV_1;
    crg.coreClkSelect   = CRG_CORE_CLK_SELECT_HOSC;
    crg.handleEx.clk1MSelect   = CRG_1M_CLK_SELECT_HOSC;
    crg.handleEx.pllPostDiv2   = CRG_PLL_POSTDIV2_1;
    /* Initialize the clock configuration. */
    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
}

static void I2C0_Init(void)
{
    HAL_CRG_IpEnableSet(I2C0_BASE, IP_CLK_ENABLE);
    g_i2c0.baseAddress = I2C0;

    g_i2c0.functionMode = I2C_MODE_SELECT_SLAVE_ONLY;
    g_i2c0.addrMode = I2C_7_BITS;
    g_i2c0.sdaHoldTime = 10; /* I2C SDA hold time 10. */
    g_i2c0.freq = 400000; /* I2C SCL rate 400000 bit/s. */
    g_i2c0.transferBuff = NULL;
    g_i2c0.ignoreAckFlag = BASE_CFG_DISABLE;
    g_i2c0.handleEx.spikeFilterTime = 0;
    g_i2c0.handleEx.sdaDelayTime = 0;
    g_i2c0.timeout = 10000; /* The value of timeout is 10000ms; */
    g_i2c0.slaveOwnAddress = 82; /* The own address of I2C is 82. */
    g_i2c0.handleEx.slaveOwnXmbAddressEnable = BASE_CFG_DISABLE;
    g_i2c0.handleEx.slaveOwnXmbAddress = 0;
    g_i2c0.generalCallMode = BASE_CFG_DISABLE;
    g_i2c0.state = I2C_STATE_RESET;
    HAL_I2C_Init(&g_i2c0);
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART0_BASE, CRG_PLL_NO_PREDV);
    /* UART-related parameter configuration. */
    g_uart0.baseAddress = UART0;

    g_uart0.baudRate = UART0_BAND_RATE;  /* Setting the UART Baud Rate. */
    g_uart0.dataLength = UART_DATALENGTH_8BIT;
    g_uart0.stopBits = UART_STOPBITS_ONE;
    g_uart0.parity = UART_PARITY_NONE;
    g_uart0.txMode = UART_MODE_BLOCKING;
    g_uart0.rxMode = UART_MODE_BLOCKING;
    g_uart0.fifoMode = BASE_CFG_ENABLE; /* Disable uart fifo mode. */
    g_uart0.fifoTxThr = UART_FIFODEPTH_SIZE8;
    g_uart0.fifoRxThr = UART_FIFODEPTH_SIZE8;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    g_uart0.handleEx.overSampleMultiple = UART_OVERSAMPLING_16X;
    g_uart0.handleEx.msbFirst = BASE_CFG_DISABLE;
    /* Initializing the UART0. */
    HAL_UART_Init(&g_uart0);
}

static void IOConfig(void)
{
    HAL_IOCMG_SetPinAltFuncMode(GPIO4_5_AS_I2C0_SCL);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO4_5_AS_I2C0_SCL, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO4_5_AS_I2C0_SCL, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO4_5_AS_I2C0_SCL, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO4_5_AS_I2C0_SCL, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO4_6_AS_I2C0_SDA);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO4_6_AS_I2C0_SDA, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO4_6_AS_I2C0_SDA, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO4_6_AS_I2C0_SDA, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO4_6_AS_I2C0_SDA, DRIVER_RATE_2);  /* Output signal edge fast/slow */

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
    UART0_Init();
    I2C0_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}