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
    CRG_Handle crg;
    crg.baseAddress     = CRG;  /* crg address */
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_HOSC;
    crg.pllPreDiv       = CRG_PLL_PREDIV_4;
    crg.pllFbDiv        = 48; /* PLL Multiplier 48 */
    crg.pllPostDiv      = CRG_PLL_POSTDIV_2;
    crg.coreClkSelect   = CRG_CORE_CLK_SELECT_HOSC;
    crg.handleEx.clk1MSelect   = CRG_1M_CLK_SELECT_HOSC;
    crg.handleEx.pllPostDiv2   = CRG_PLL_POSTDIV2_3;

    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) { /* crg initialization */
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
}

__weak void ADC_Int1Finish(ADC_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN ADC0_CALLBACK_INT1 */
    /* USER CODE END ADC0_CALLBACK_INT1 */
}

__weak void ADCOversampingCallback(ADC_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN ADCOversampingCallback */
    /* USER CODE END ADCOversampingCallback */
}

static void ADC0_Init(void)
{
    HAL_CRG_IpEnableSet(ADC0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(ADC0_BASE, CRG_ADC_CLK_ASYN_PLL_DIV);
    HAL_CRG_IpClkDivSet(ADC0_BASE, CRG_ADC_DIV_2);

    g_adc.baseAddress = ADC0;
    g_adc.socPriority = ADC_PRIMODE_ALL_ROUND;

    HAL_ADC_Init(&g_adc); /* adc initialization */

    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINA2; /* PIN47(ADC AIN2) */
    socParam.sampleTotalTime = ADC_SOCSAMPLE_5CLK; /* adc sample total time 5 adc_clk */
    socParam.trigSource = ADC_TRIGSOC_SOFT;
    socParam.continueMode = BASE_CFG_DISABLE;
    socParam.finishMode = ADC_SOCFINISH_INT1;
    HAL_ADC_ConfigureSoc(&g_adc, ADC_SOC_NUM1, &socParam);

    HAL_ADC_RegisterCallBack(&g_adc, ADC_CALLBACK_INT1, (ADC_CallbackType)ADC_Int1Finish);
    HAL_ADC_RegisterCallBack(&g_adc, ADC_CALLBACK_EVENT_OVERSAMPLING, (ADC_CallbackType)ADCOversampingCallback);

    IRQ_Register(IRQ_ADC0_INT1, HAL_ADC_IrqHandlerDataB, &g_adc);
    IRQ_SetPriority(IRQ_ADC0_INT1, 7); /* The priority is 7 */
    IRQ_EnableN(IRQ_ADC0_INT1);
    IRQ_Register(IRQ_ADC0_EVENT, HAL_ADC_IrqHandlerAllEvent, &g_adc);
    IRQ_SetPriority(IRQ_ADC0_EVENT, 6); /* The priority is 6 */
    IRQ_EnableN(IRQ_ADC0_EVENT);
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART0_BASE, CRG_AHB_CLK_NO_PREDV);

    g_uart0.baseAddress = UART0;  /* uart address */
    /* UART parameter settings */
    g_uart0.baudRate = UART0_BAND_RATE;           /* uart baud rate */
    g_uart0.dataLength = UART_DATALENGTH_8BIT;    /* uart datalength */
    g_uart0.stopBits = UART_STOPBITS_ONE;         /* uart stop bit */
    g_uart0.parity = UART_PARITY_NONE;
    g_uart0.txMode = UART_MODE_BLOCKING;
    g_uart0.rxMode = UART_MODE_BLOCKING;
    g_uart0.fifoMode = BASE_CFG_ENABLE;
    g_uart0.fifoTxThr = UART_FIFODEPTH_SIZE8;
    g_uart0.fifoRxThr = UART_FIFODEPTH_SIZE8;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    g_uart0.handleEx.overSampleMultiple = UART_OVERSAMPLING_16X;
    g_uart0.handleEx.msbFirst = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0); /* uart initialization */
}

static void IOConfig(void)
{
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode = 0;
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode_clear = 1;
    HAL_IOCMG_SetPinAltFuncMode(GPIO0_5_AS_ADC_AIN2);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO0_5_AS_ADC_AIN2, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO0_5_AS_ADC_AIN2, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO0_5_AS_ADC_AIN2, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO0_5_AS_ADC_AIN2, DRIVER_RATE_2);  /* Output signal edge fast/slow */

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
    ADC0_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}