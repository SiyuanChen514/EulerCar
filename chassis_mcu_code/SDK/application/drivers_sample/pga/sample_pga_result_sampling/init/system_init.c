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
    BASE_FUNC_ASSERT_PARAM(coreClkSelect != NULL);

    CRG_Handle crg;
    crg.baseAddress     = CRG;
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_HOSC;
    crg.pllFbDiv        = 0x30;  /* PLL loop divider ratio = 0x30 */
    crg.pllPreDiv       = CRG_PLL_PREDIV_4;
    crg.pllPostDiv      = CRG_PLL_POSTDIV_2;
    crg.coreClkSelect   = 0;           /* Inner clock HOSC select. */
    crg.handleEx.pllPostDiv2 = CRG_PLL_POSTDIV2_3;
    crg.handleEx.clk1MSelect = CRG_1M_CLK_SELECT_HOSC; /* Set the 1MHz clock select. */
    crg.handleEx.clk1MDiv = 0x29;   /* Default frequency divider of the 1 MHz clock = 0x29 */
    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;          /* CRG init error. */
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
}

static void PGA0_Init(void)
{
    HAL_CRG_IpEnableSet(PGA0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(PGA0_BASE, 0);

    g_pga0.baseAddress = PGA0_BASE;
    g_pga0.gain = PGA_GAIN_2X;        /* PGA gain value. */
    g_pga0.externalResistorMode = BASE_CFG_DISABLE;   /* Inner resistor mode. */
    HAL_PGA_Init(&g_pga0);
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART0_BASE, CRG_AHB_CLK_NO_PREDV);

    g_uart0.baseAddress = UART0;
    g_uart0.baudRate = UART0_BAND_RATE;
    g_uart0.dataLength = UART_DATALENGTH_8BIT;   /* Word length 8 bit. */
    g_uart0.stopBits = UART_STOPBITS_ONE;
    g_uart0.parity = UART_PARITY_NONE;
    g_uart0.txMode = UART_MODE_BLOCKING;        /* UART blocking mode. */
    g_uart0.rxMode = UART_MODE_BLOCKING;
    g_uart0.fifoMode = BASE_CFG_ENABLE;
    g_uart0.fifoTxThr = UART_FIFODEPTH_SIZE8;   /* UART FIFO depth. */
    g_uart0.fifoRxThr = UART_FIFODEPTH_SIZE8;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    g_uart0.handleEx.overSampleMultiple = UART_OVERSAMPLING_16X;  /* Oversamping 16x. */
    g_uart0.handleEx.msbFirst = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0);
}

static void IOConfig(void)
{
    HAL_IOCMG_SetPinAltFuncMode(GPIO2_7_AS_PGA0_OUT);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_7_AS_PGA0_OUT, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_7_AS_PGA0_OUT, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_7_AS_PGA0_OUT, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_7_AS_PGA0_OUT, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO2_6_AS_PGA0_N0);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_6_AS_PGA0_N0, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_6_AS_PGA0_N0, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_6_AS_PGA0_N0, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_6_AS_PGA0_N0, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO2_5_AS_PGA0_P0);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_5_AS_PGA0_P0, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_5_AS_PGA0_P0, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_5_AS_PGA0_P0, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_5_AS_PGA0_P0, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO0_3_AS_UART0_TXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO0_3_AS_UART0_TXD, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO0_3_AS_UART0_TXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO0_3_AS_UART0_TXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO0_3_AS_UART0_TXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO0_4_AS_UART0_RXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO0_4_AS_UART0_RXD, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO0_4_AS_UART0_RXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO0_4_AS_UART0_RXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO0_4_AS_UART0_RXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */
}


static void ADC0_Init(void)
{
    HAL_CRG_IpEnableSet(ADC0_BASE, IP_CLK_ENABLE);     /* ADC clock configure. */
    HAL_CRG_IpClkSelectSet(ADC0_BASE, CRG_ADC_CLK_ASYN_PLL_DIV);
    HAL_CRG_IpClkDivSet(ADC0_BASE, CRG_ADC_DIV_2);

    g_adc.baseAddress = ADC0;
    g_adc.socPriority = ADC_PRIMODE_ALL_ROUND;

    HAL_ADC_Init(&g_adc);      /* ADC init configure. */

    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINA0; /* PIN47(ADC AIN2) */
    socParam.sampleTotalTime = ADC_SOCSAMPLE_5CLK; /* adc sample total time 5 adc_clk */
    socParam.trigSource = ADC_TRIGSOC_SOFT;
    socParam.continueMode = BASE_CFG_DISABLE;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc, ADC_SOC_NUM1, &socParam); /* SOC configure. */
}

void SystemInit(void)
{
    IOConfig();
    UART0_Init();
    ADC0_Init();
    PGA0_Init();
    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}