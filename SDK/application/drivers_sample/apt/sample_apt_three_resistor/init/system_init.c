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
    crg.baseAddress     = CRG;
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_XTAL;
    crg.pllPreDiv       = CRG_PLL_PREDIV_4;
    crg.pllFbDiv        = 32; /* PLL Multiplier 32 */
    crg.pllPostDiv      = CRG_PLL_POSTDIV_1;
    crg.coreClkSelect   = CRG_CORE_CLK_SELECT_PLL;
    crg.handleEx.clk1MSelect   = CRG_1M_CLK_SELECT_HOSC;
    crg.handleEx.pllPostDiv2   = CRG_PLL_POSTDIV2_1;

    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
}

__weak void APTUEventCallback(void *aptHandle)
{
    BASE_FUNC_UNUSED(aptHandle);
    /* USER CODE BEGIN APT0_EVENT_INTERRUPT */
    /* USER CODE END APT0_EVENT_INTERRUPT */
}

__weak void APTUTimerCallback(void *aptHandle)
{
    BASE_FUNC_UNUSED(aptHandle);
    /* USER CODE BEGIN APT0_TIMER_INTERRUPT */
    /* USER CODE END APT0_TIMER_INTERRUPT */
}

static void APT0_ProtectInit(void)
{
    /* APT protection action. */
    APT_OutCtrlProtectEx protectApt = {0};
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.cbcClrModeEx = APT_CLEAR_CBC_ON_CNTR_ZERO;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;   /* APT protection action. */
    protectApt.ocEvtInterruptEnEx = BASE_CFG_ENABLE;    /* Enable APT even interrupt. */
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_POE0;
    protectApt.filterCycleNumEx = 0;
    /* Configure APT protection. */
    HAL_APT_ProtectInitEx(&g_hAptU, &protectApt);
}

static void APT0_Init(void)
{
    HAL_CRG_IpEnableSet(APT0_BASE, IP_CLK_ENABLE);

    g_hAptU.baseAddress = APT0;

    /* Clock Settings */
    g_hAptU.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_hAptU.waveform.timerPeriod = 20000;      /* timerPeriod is 20000 */
    g_hAptU.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_hAptU.waveform.basicType = APT_PWM_BASIC_A_LOW_B_HIGH;
    g_hAptU.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_hAptU.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_hAptU.waveform.divInitVal = 0;
    g_hAptU.waveform.cntInitVal = 0;
    g_hAptU.waveform.cntCmpLeftEdge = 1;
    g_hAptU.waveform.cntCmpRightEdge = 19999;   /* cntCmpRightEdge is 19999 */
    g_hAptU.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_hAptU.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_hAptU.waveform.deadBandCnt = 60;   /* deadBandCnt is 60 */

    /* ADC Trigger SOCA */
    g_hAptU.adcTrg.trgEnSOCA = BASE_CFG_ENABLE;
    g_hAptU.adcTrg.cntCmpSOCA = 1;
    g_hAptU.adcTrg.trgSrcSOCA = APT_CS_SRC_CNTR_CMPA_UP;
    g_hAptU.adcTrg.trgScaleSOCA = 1;

    /* ADC Trigger SOCB */
    g_hAptU.adcTrg.trgEnSOCB = BASE_CFG_ENABLE;
    g_hAptU.adcTrg.cntCmpSOCB =  1;
    g_hAptU.adcTrg.trgSrcSOCB = APT_CS_SRC_CNTR_CMPB_UP;
    g_hAptU.adcTrg.trgScaleSOCB = 1;

    g_hAptU.adcTrg.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_hAptU.adcTrg.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;

    /* Timer Trigger */
    g_hAptU.tmrInterrupt.tmrInterruptEn = BASE_CFG_ENABLE;
    g_hAptU.tmrInterrupt.tmrInterruptSrc = APT_INT_SRC_CNTR_ZERO_PERIOD;
    g_hAptU.tmrInterrupt.tmrInterruptScale = 1;

    APT0_ProtectInit();

    HAL_APT_PWMInit(&g_hAptU);
    HAL_APT_RegisterCallBack(&g_hAptU, APT_EVENT_INTERRUPT, APTUEventCallback);
    IRQ_SetPriority(IRQ_APT0_TMR, 1);
    IRQ_Register(IRQ_APT0_TMR, HAL_APT_TimerIrqHandler, &g_hAptU);
    IRQ_EnableN(IRQ_APT0_TMR);
    HAL_APT_RegisterCallBack(&g_hAptU, APT_TIMER_INTERRUPT, APTUTimerCallback);
    IRQ_SetPriority(IRQ_APT0_EVT, 1);
    IRQ_Register(IRQ_APT0_EVT, HAL_APT_EventIrqHandler, &g_hAptU);
    IRQ_EnableN(IRQ_APT0_EVT);
}

static void APT1_ProtectInit(void)
{
    /* APT protection action. */
    APT_OutCtrlProtectEx protectApt = {0};
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;  /* enable event protect */
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.cbcClrModeEx = APT_CLEAR_CBC_ON_CNTR_ZERO;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = BASE_CFG_DISABLE;
    /* config systerm error event protect */
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_POE0;
    protectApt.filterCycleNumEx = 0;
    /* Configure protection. */
    HAL_APT_ProtectInitEx(&g_hAptV, &protectApt);
}

static void APT1_Init(void)
{
    HAL_CRG_IpEnableSet(APT1_BASE, IP_CLK_ENABLE);

    g_hAptV.baseAddress = APT1;

    /* Clock Settings */
    g_hAptV.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_hAptV.waveform.timerPeriod = 20000;      /* APT timer period is 20000. */
    g_hAptV.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_hAptV.waveform.basicType = APT_PWM_BASIC_A_LOW_B_HIGH;
    g_hAptV.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_hAptV.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_hAptV.waveform.divInitVal = 0;
    g_hAptV.waveform.cntInitVal = 0;
    g_hAptV.waveform.cntCmpLeftEdge = 1;
    g_hAptV.waveform.cntCmpRightEdge = 19999;  /* APT compare right edge is 19999. */
    g_hAptV.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_hAptV.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_hAptV.waveform.deadBandCnt = 60;      /* deadBandCnt is 60 */

    /* ADC Trigger SOCA */
    g_hAptV.adcTrg.trgEnSOCA = BASE_CFG_ENABLE;
    g_hAptV.adcTrg.cntCmpSOCA = 1;
    g_hAptV.adcTrg.trgSrcSOCA = APT_CS_SRC_CNTR_CMPA_UP;
    g_hAptV.adcTrg.trgScaleSOCA = 1;

    g_hAptV.adcTrg.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_hAptV.adcTrg.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;

    APT1_ProtectInit();

    HAL_APT_PWMInit(&g_hAptV);
}

static void APT2_ProtectInit(void)
{
    /* APT protection action. */
    APT_OutCtrlProtectEx protectApt = {0};
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;  /* enable event protect */
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.cbcClrModeEx = APT_CLEAR_CBC_ON_CNTR_ZERO;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = BASE_CFG_DISABLE;
    /* config systerm error event protect */
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_POE0;
    protectApt.filterCycleNumEx = 0;
    /* Configure protection. */
    HAL_APT_ProtectInitEx(&g_hAptW, &protectApt);
}

static void APT2_Init(void)
{
    HAL_CRG_IpEnableSet(APT2_BASE, IP_CLK_ENABLE);

    g_hAptW.baseAddress = APT2;
    /* Clock Settings */
    g_hAptW.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_hAptW.waveform.timerPeriod = 20000;    /* APT timer period is 20000. */
    g_hAptW.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_hAptW.waveform.basicType = APT_PWM_BASIC_A_LOW_B_HIGH;
    g_hAptW.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_hAptW.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_hAptW.waveform.divInitVal = 0;
    g_hAptW.waveform.cntInitVal = 0;
    g_hAptW.waveform.cntCmpLeftEdge = 1;
    g_hAptW.waveform.cntCmpRightEdge = 19999;     /* APT compare right edge is 19999. */
    g_hAptW.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_hAptW.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_hAptW.waveform.deadBandCnt = 60;          /* APT dead band count is 60. */

    /* ADC Trigger SOCA */
    g_hAptW.adcTrg.trgEnSOCA = BASE_CFG_ENABLE;
    g_hAptW.adcTrg.cntCmpSOCA = 1;
    g_hAptW.adcTrg.trgSrcSOCA = APT_CS_SRC_CNTR_CMPA_UP;
    g_hAptW.adcTrg.trgScaleSOCA = 1;

    g_hAptW.adcTrg.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_hAptW.adcTrg.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;

    APT2_ProtectInit();

    HAL_APT_PWMInit(&g_hAptW);
}

static void GPIO_Init(void)
{
    HAL_CRG_IpEnableSet(GPIO0_BASE, IP_CLK_ENABLE);
    g_gpio0.baseAddress = GPIO0;
    /* Setting GPIO_PIN_0_1 */
    g_gpio0.pins = GPIO_PIN_1;
    HAL_GPIO_Init(&g_gpio0);
    HAL_GPIO_SetDirection(&g_gpio0, g_gpio0.pins, GPIO_OUTPUT_MODE); /* Setting GPIO_PIN_0_1 is out mode. */
    HAL_GPIO_SetValue(&g_gpio0, g_gpio0.pins, GPIO_LOW_LEVEL);       /* Setting GPIO_PIN_0_1 drive class is low. */
    HAL_GPIO_SetIrqType(&g_gpio0, g_gpio0.pins, GPIO_INT_TYPE_NONE);  /* Sets GPIO_PIN_0_1 interrupt type. */
    return;
}

__weak void UART_CharacterMatchCallBack(UART_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART_CharacterMatchCallBack */
    /* USER CODE END UART_CharacterMatchCallBack */
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART0_BASE, CRG_PLL_NO_PREDV);

    g_uart0Handle.baseAddress = UART0;

    g_uart0Handle.baudRate = UART0_BAND_RATE;
    g_uart0Handle.dataLength = UART_DATALENGTH_8BIT;
    g_uart0Handle.stopBits = UART_STOPBITS_ONE;
    g_uart0Handle.parity = UART_PARITY_NONE;
    g_uart0Handle.txMode = UART_MODE_BLOCKING;
    g_uart0Handle.rxMode = UART_MODE_BLOCKING;
    g_uart0Handle.fifoMode = BASE_CFG_ENABLE;
    g_uart0Handle.fifoTxThr = UART_FIFODEPTH_SIZE8;
    g_uart0Handle.fifoRxThr = UART_FIFODEPTH_SIZE8;
    g_uart0Handle.hwFlowCtr = BASE_CFG_DISABLE;
    g_uart0Handle.handleEx.overSampleMultiple = UART_OVERSAMPLING_16X;
    g_uart0Handle.handleEx.msbFirst = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0Handle);
}

static void IOConfig(void)
{
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode = 0;
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode_clear = 1;
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

    HAL_IOCMG_SetPinAltFuncMode(GPIO3_0_AS_APT0_PWMA);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO3_0_AS_APT0_PWMA, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO3_0_AS_APT0_PWMA, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO3_0_AS_APT0_PWMA, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO3_0_AS_APT0_PWMA, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO4_0_AS_APT0_PWMB);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO4_0_AS_APT0_PWMB, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO4_0_AS_APT0_PWMB, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO4_0_AS_APT0_PWMB, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO4_0_AS_APT0_PWMB, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO3_2_AS_APT2_PWMA);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO3_2_AS_APT2_PWMA, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO3_2_AS_APT2_PWMA, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO3_2_AS_APT2_PWMA, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO3_2_AS_APT2_PWMA, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO4_2_AS_APT2_PWMB);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO4_2_AS_APT2_PWMB, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO4_2_AS_APT2_PWMB, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO4_2_AS_APT2_PWMB, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO4_2_AS_APT2_PWMB, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO3_1_AS_APT1_PWMA);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO3_1_AS_APT1_PWMA, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO3_1_AS_APT1_PWMA, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO3_1_AS_APT1_PWMA, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO3_1_AS_APT1_PWMA, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO4_1_AS_APT1_PWMB);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO4_1_AS_APT1_PWMB, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO4_1_AS_APT1_PWMB, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO4_1_AS_APT1_PWMB, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO4_1_AS_APT1_PWMB, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO0_7_AS_POE0);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO0_7_AS_POE0, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO0_7_AS_POE0, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO0_7_AS_POE0, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO0_7_AS_POE0, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO0_1_AS_GPIO0_1);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO0_1_AS_GPIO0_1, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO0_1_AS_GPIO0_1, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO0_1_AS_GPIO0_1, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO0_1_AS_GPIO0_1, DRIVER_RATE_2);  /* Output signal edge fast/slow */
}

void SystemInit(void)
{
    IOConfig();
    UART0_Init();
    APT0_Init();
    APT1_Init();
    APT2_Init();
    GPIO_Init();
    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}