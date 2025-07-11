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

__weak void ADC_Init2FromApt(ADC_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN ADC_Init2FromApt */
    /* USER CODE END ADC_Init2FromApt */
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

    HAL_ADC_Init(&g_adc);

    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINA1;

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 127; /* adc sample total time 127 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT0_SOCA;
    socParam.finishMode = ADC_SOCFINISH_INT2;
    HAL_ADC_ConfigureSoc(&g_adc, ADC_SOC_NUM1, &socParam);

    HAL_ADC_RegisterCallBack(&g_adc, ADC_CALLBACK_INT2, ADC_Init2FromApt);
    IRQ_SetPriority(IRQ_ADC1_INT2, 4);  /* Set the priority to level 4 */
    IRQ_EnableN(IRQ_ADC1_INT2);
    HAL_ADC_IrqService(&g_adc);
}

static void APT0_Init(void)
{
    HAL_CRG_IpEnableSet(APT0_BASE, IP_CLK_ENABLE);

    g_hAptU.baseAddress = APT0;
    g_hAptU.irqNumEvt = IRQ_APT0_EVT;
    g_hAptU.irqNumTmr = IRQ_APT0_TMR;

    /* Clock Settings */
    g_hAptU.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_hAptU.waveform.timerPeriod = 20000;   /* timerPeriod is 20000 */
    g_hAptU.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_hAptU.waveform.basicType = APT_PWM_BASIC_A_LOW_B_HIGH;
    g_hAptU.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_hAptU.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_hAptU.waveform.divInitVal = 0U;
    g_hAptU.waveform.cntInitVal = 0U;
    g_hAptU.waveform.cntCmpLeftEdge = 1;
    g_hAptU.waveform.cntCmpRightEdge = 19999;   /* cntCmpRightEdge is 19999 */
    g_hAptU.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_hAptU.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_hAptU.waveform.deadBandCnt = 60;  /* deadBandCnt is 60 */

    /* ADC Triggle SOCA */
    g_hAptU.adcTrg.trgEnSOCA = BASE_CFG_ENABLE;
    g_hAptU.adcTrg.cntCmpSOCA = 1;
    g_hAptU.adcTrg.trgSrcSOCA = APT_CS_SRC_CNTR_CMPA_UP;
    g_hAptU.adcTrg.trgScaleSOCA = 1;

    /* ADC Triggle SOCB */
    g_hAptU.adcTrg.trgEnSOCB = BASE_CFG_ENABLE;
    g_hAptU.adcTrg.cntCmpSOCB =  1;
    g_hAptU.adcTrg.trgSrcSOCB = APT_CS_SRC_CNTR_CMPB_UP;
    g_hAptU.adcTrg.trgScaleSOCB = 1;

    g_hAptU.adcTrg.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_hAptU.adcTrg.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;

    /* Timer Triggle */
    g_hAptU.tmrInterrupt.tmrInterruptEn = BASE_CFG_ENABLE;
    g_hAptU.tmrInterrupt.tmrInterruptSrc = APT_INT_SRC_CNTR_ZERO_PERIOD;
    g_hAptU.tmrInterrupt.tmrInterruptScale = 1;

    /* Output Protect */
    g_hProtectU.ocEventEn = BASE_CFG_ENABLE;
    g_hProtectU.ocEventMode = APT_OUT_CTRL_ONE_SHOT;
    g_hProtectU.evtPolarity = APT_EM_EVENT_POLARITY_INVERT;
    g_hProtectU.ocEvent = APT_OC_GPIO_EVENT_1;
    g_hProtectU.cbcClrMode = APT_CLEAR_CBC_ON_CNTR_ZERO;
    g_hProtectU.ocAction = APT_OUT_CTRL_ACTION_LOW;
    g_hProtectU.ocEvtInterruptEn = true;
    HAL_APT_ProtectInit(&g_hAptU, &g_hProtectU);

    HAL_APT_PWMInit(&g_hAptU);
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

    iconfig->iocmg_32.BIT.func = 0x3; /* 0x3 is APT0_PWMA */
    iconfig->iocmg_32.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_32.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_32.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_32.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_32.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_35.BIT.func = 0x3; /* 0x3 is APT0_PWMB */
    iconfig->iocmg_35.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_35.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_35.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_35.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_35.BIT.se = BASE_CFG_DISABLE;

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
    APT0_Init();
    ADC1_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}