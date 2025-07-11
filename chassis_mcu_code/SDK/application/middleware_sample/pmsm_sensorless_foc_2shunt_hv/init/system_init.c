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

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect)
{
    CRG_Handle crg;
    crg.baseAddress     = CRG;
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_HOSC;
    crg.pllPreDiv       = CRG_PLL_PREDIV_4;
    crg.pllFbDiv        = 48; /* PLL Multiplier 48 */
    crg.pllPostDiv      = CRG_PLL_POSTDIV_2;
    crg.coreClkSelect   = CRG_CORE_CLK_SELECT_PLL;
    crg.handleEx.pllPostDiv2   = CRG_PLL_POSTDIV2_3;
    crg.handleEx.clk1MSelect   = CRG_1M_CLK_SELECT_HOSC;
    /* The 1 MHz freq is equal to the input clock frequency / (clk_1m_div + 1). */
    crg.handleEx.clk1MDiv = (25 - 1); /* The 25 - 1 is div coffecient */

    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
}

static void ACMP0_Init(void)
{
    /* enable acmp input capture function */
    HAL_CRG_IpEnableSet(ACMP0_BASE, IP_CLK_ENABLE);                /* ACMP clock bit reset. */
    HAL_CRG_IpClkSelectSet(ACMP0_BASE, 0);
    g_acmp0.baseAddress =  ACMP0_BASE;
    g_acmp0.inOutConfig.inputNNum = ACMP_INPUT_N_SELECT4;
    g_acmp0.inOutConfig.inputPNum = ACMP_INPUT_P_SELECT4;
    /* set acmp polarity */
    g_acmp0.inOutConfig.polarity = ACMP_OUT_NOT_INVERT;
    g_acmp0.filterCtrl.filterMode = ACMP_FILTER_FILTER;
    g_acmp0.filterCtrl.filterStep = 100; /* 100 is filter step size of the comparator. */
    g_acmp0.hysteresisVol = ACMP_HYS_VOL_ZERO;
    g_acmp0.interruptEn = BASE_CFG_UNSET;
    /* acmp0 init */
    HAL_ACMP_Init(&g_acmp0);
}

static void ADC0_Init(void)
{
    HAL_CRG_IpEnableSet(ADC0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(ADC0_BASE, CRG_ADC_CLK_ASYN_PLL_DIV);
    HAL_CRG_IpClkDivSet(ADC0_BASE, CRG_ADC_DIV_1);

    g_adc0.baseAddress = ADC0;
    g_adc0.socPriority = ADC_PRIMODE_ALL_ROUND;

    HAL_ADC_Init(&g_adc0);

    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINA0; /* PGA0_OUT(ADC AIN0) */
    socParam.sampleTotalTime = ADC_SOCSAMPLE_22CLK; /* adc sample total time 22 adc_clk */
    socParam.trigSource = ADC_TRIGSOC_APT0_SOCA;
    socParam.continueMode = BASE_CFG_DISABLE;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc0, ADC_SOC_NUM1, &socParam);

    socParam.adcInput = ADC_CH_ADCINA1; /* PGA1_OUT(ADC AIN1) */
    socParam.sampleTotalTime = ADC_SOCSAMPLE_22CLK; /* adc sample total time 22 adc_clk */
    socParam.trigSource = ADC_TRIGSOC_APT0_SOCA;
    socParam.continueMode = BASE_CFG_DISABLE;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc0, ADC_SOC_NUM6, &socParam);
}

__weak void MotorSysErrCallback(void *aptHandle)
{
    BASE_FUNC_UNUSED(aptHandle);
    /* USER CODE BEGIN APT0_EVENT_INTERRUPT */
    /* USER CODE END APT0_EVENT_INTERRUPT */
}

__weak void MotorCarrierProcessCallback(void *aptHandle)
{
    BASE_FUNC_UNUSED(aptHandle);
    /* USER CODE BEGIN APT0_TIMER_INTERRUPT */
    /* USER CODE END APT0_TIMER_INTERRUPT */
}

static void APT0_ProtectInit(void)
{
    /* enable apt0 event interupt protection function */
    APT_OutCtrlProtectEx protectApt = {0};
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.cbcClrModeEx = APT_CLEAR_CBC_ON_CNTR_ZERO;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;    /* low action protection */
    protectApt.ocEvtInterruptEnEx = BASE_CFG_ENABLE;
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_ACMP0;
    protectApt.evtPolarityMaskEx = APT_EM_ACMP0_INVERT_BIT;
    protectApt.filterCycleNumEx = 0;
    /* set apt protect register */
    HAL_APT_ProtectInitEx(&g_apt0, &protectApt);
}

static void APT0_Init(void)
{
    HAL_CRG_IpEnableSet(APT0_BASE, IP_CLK_ENABLE);

    g_apt0.baseAddress = APT0;

    /* Clock Settings */
    g_apt0.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt0.waveform.timerPeriod = 7500; /* apt init timer period is 7500 */
    g_apt0.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_apt0.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt0.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt0.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt0.waveform.divInitVal = 0;
    g_apt0.waveform.cntInitVal = 0;
    g_apt0.waveform.cntCmpLeftEdge = 500;   /* apt init left edge count period is 500 */
    g_apt0.waveform.cntCmpRightEdge = 4000; /* apt init right edge count period is 4000 */
    g_apt0.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt0.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_apt0.waveform.deadBandCnt = 225;  /* apt dead band count value is 225 */

    /* ADC Trigger SOCA */
    g_apt0.adcTrg.trgEnSOCA = BASE_CFG_ENABLE;
    g_apt0.adcTrg.cntCmpSOCA = 375; /* apt trig sample count value is 375 */
    g_apt0.adcTrg.trgSrcSOCA = APT_CS_SRC_CNTR_CMPA_DOWN;
    g_apt0.adcTrg.trgScaleSOCA = 1;

    /* ADC Trigger SOCB */
    g_apt0.adcTrg.trgEnSOCB = BASE_CFG_ENABLE;
    g_apt0.adcTrg.cntCmpSOCB =  1;
    g_apt0.adcTrg.trgSrcSOCB = APT_CS_SRC_CNTR_CMPB_DOWN;
    g_apt0.adcTrg.trgScaleSOCB = 1;

    g_apt0.adcTrg.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt0.adcTrg.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;

    /* Timer Trigger */
    g_apt0.tmrInterrupt.tmrInterruptEn = BASE_CFG_ENABLE;
    g_apt0.tmrInterrupt.tmrInterruptSrc = APT_INT_SRC_CNTR_ZERO;
    g_apt0.tmrInterrupt.tmrInterruptScale = 1;

    APT0_ProtectInit();

    HAL_APT_PWMInit(&g_apt0);
    HAL_APT_RegisterCallBack(&g_apt0, APT_EVENT_INTERRUPT, MotorSysErrCallback);
    IRQ_SetPriority(IRQ_APT0_EVT, 7); /* apt event interupt level is 7 */
    IRQ_Register(IRQ_APT0_EVT, HAL_APT_EventIrqHandler, &g_apt0);
    IRQ_EnableN(IRQ_APT0_EVT);
    HAL_APT_RegisterCallBack(&g_apt0, APT_TIMER_INTERRUPT, MotorCarrierProcessCallback);
    IRQ_SetPriority(IRQ_APT0_TMR, 6); /* apt timer interupt level is 6 */
    IRQ_Register(IRQ_APT0_TMR, HAL_APT_TimerIrqHandler, &g_apt0);
    IRQ_EnableN(IRQ_APT0_TMR);
}

static void APT1_ProtectInit(void)
{
    APT_OutCtrlProtectEx protectApt = {0};
    /* enable apt1 event interupt protection function */
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.cbcClrModeEx = APT_CLEAR_CBC_ON_CNTR_ZERO;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    /* disable apt1 event interupt protection function */
    protectApt.ocEvtInterruptEnEx = BASE_CFG_DISABLE;
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_ACMP0;
    protectApt.evtPolarityMaskEx = APT_EM_ACMP0_INVERT_BIT;
    protectApt.filterCycleNumEx = 0;
    /* init APT config module */
    HAL_APT_ProtectInitEx(&g_apt1, &protectApt);
}

static void APT1_Init(void)
{
    HAL_CRG_IpEnableSet(APT1_BASE, IP_CLK_ENABLE);

    g_apt1.baseAddress = APT1;

    /* Clock Settings */
    g_apt1.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt1.waveform.timerPeriod = 7500; /* apt init timer period is 7500 */
    g_apt1.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_apt1.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt1.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt1.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt1.waveform.divInitVal = 0;
    g_apt1.waveform.cntInitVal = 0;
    g_apt1.waveform.cntCmpLeftEdge = 500;   /* apt init left edge count period is 500 */
    g_apt1.waveform.cntCmpRightEdge = 4000; /* apt init right edge count period is 4000 */
    g_apt1.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt1.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_apt1.waveform.deadBandCnt = 225; /* apt dead band count value is 225 */
    APT1_ProtectInit();
    HAL_APT_PWMInit(&g_apt1);
}

static void APT2_ProtectInit(void)
{
    APT_OutCtrlProtectEx protectApt = {0};
    /* enable apt2 event interupt protection function */
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.cbcClrModeEx = APT_CLEAR_CBC_ON_CNTR_ZERO;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    /* disable apt2 event interupt protection function */
    protectApt.ocEvtInterruptEnEx = BASE_CFG_DISABLE;
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_ACMP0;
    protectApt.evtPolarityMaskEx = APT_EM_ACMP0_INVERT_BIT;
    protectApt.filterCycleNumEx = 0;
    /* init APT config module */
    HAL_APT_ProtectInitEx(&g_apt2, &protectApt);
}

static void APT2_Init(void)
{
    HAL_CRG_IpEnableSet(APT2_BASE, IP_CLK_ENABLE);

    g_apt2.baseAddress = APT2;

    /* Clock Settings */
    g_apt2.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt2.waveform.timerPeriod = 7500; /* apt init timer period is 7500 */
    g_apt2.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_apt2.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt2.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt2.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt2.waveform.divInitVal = 0;
    g_apt2.waveform.cntInitVal = 0;
    g_apt2.waveform.cntCmpLeftEdge = 500;   /* apt init left edge count period is 500 */
    g_apt2.waveform.cntCmpRightEdge = 4000; /* apt init right edge count period is 4000 */
    g_apt2.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt2.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_apt2.waveform.deadBandCnt = 225;  /* apt dead band count value is 225 */
    APT2_ProtectInit();
    HAL_APT_PWMInit(&g_apt2);
}

__weak void MotorStartStopKeyCallback(void *param)
{
    /* gpio interupt callback function */
    GPIO_Handle *handle = (GPIO_Handle *)param;
    BASE_FUNC_UNUSED(handle);
}

static void GPIO_Init(void)
{
    HAL_CRG_IpEnableSet(GPIO0_BASE, IP_CLK_ENABLE);
    g_gpio0.baseAddress = GPIO0;
    /* set gpio0_6 config mode */
    g_gpio0.pins = GPIO_PIN_6;
    HAL_GPIO_Init(&g_gpio0);
    HAL_GPIO_SetDirection(&g_gpio0, g_gpio0.pins, GPIO_OUTPUT_MODE);
    HAL_GPIO_SetValue(&g_gpio0, g_gpio0.pins, GPIO_LOW_LEVEL);
    HAL_GPIO_SetIrqType(&g_gpio0, g_gpio0.pins, GPIO_INT_TYPE_NONE);
    /* enable gpio2 config */
    HAL_CRG_IpEnableSet(GPIO2_BASE, IP_CLK_ENABLE);
    g_gpio2.baseAddress = GPIO2;
    /* set gpio2_4 config mode */
    g_gpio2.pins = GPIO_PIN_4;
    HAL_GPIO_Init(&g_gpio2);
    HAL_GPIO_SetDirection(&g_gpio2, g_gpio2.pins, GPIO_OUTPUT_MODE);
    HAL_GPIO_SetValue(&g_gpio2, g_gpio2.pins, GPIO_LOW_LEVEL);
    HAL_GPIO_SetIrqType(&g_gpio2, g_gpio2.pins, GPIO_INT_TYPE_NONE);
    /* set gpio2_3 config mode */
    g_gpio2.pins = GPIO_PIN_3;
    HAL_GPIO_Init(&g_gpio2);
    HAL_GPIO_SetDirection(&g_gpio2, g_gpio2.pins, GPIO_INPUT_MODE);
    HAL_GPIO_SetValue(&g_gpio2, g_gpio2.pins, GPIO_HIGH_LEVEL);
    HAL_GPIO_SetIrqType(&g_gpio2, g_gpio2.pins, GPIO_INT_TYPE_LOW_LEVEL);
    /* set gpio2_3 config callback function */
    HAL_GPIO_RegisterCallBack(&g_gpio2, GPIO_PIN_3, MotorStartStopKeyCallback);
    IRQ_Register(IRQ_GPIO2, HAL_GPIO_IrqHandler, &g_gpio2);
    IRQ_SetPriority(IRQ_GPIO2, 1); /* set gpio1 interrupt priority to 1, 1~15 */
    IRQ_EnableN(IRQ_GPIO2); /* gpio interrupt enable */

    return;
}

static void PGA0_Init(void)
{
    HAL_CRG_IpEnableSet(PGA0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(PGA0_BASE, 0);
    /* congfig pga0 base address */
    g_pga0.baseAddress = PGA0_BASE;
    g_pga0.gain = PGA_GAIN_2X;
    g_pga0.externalResistorMode = BASE_CFG_ENABLE;
    g_pga0.handleEx.extCapCompensation = PGA_EXT_COMPENSATION_2X;
    /* init pga0 module */
    HAL_PGA_Init(&g_pga0);
}

static void PGA1_Init(void)
{
    /* enable pga1 */
    HAL_CRG_IpEnableSet(PGA1_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(PGA1_BASE, 0);
    /* config pga1 base address */
    g_pga1.baseAddress = PGA1_BASE;
    g_pga1.gain = PGA_GAIN_2X;
    g_pga1.externalResistorMode = BASE_CFG_ENABLE;
    g_pga1.handleEx.extCapCompensation = PGA_EXT_COMPENSATION_2X;
    /* init pga1 module */
    HAL_PGA_Init(&g_pga1);
}

__weak void MotorStatemachineCallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN MotorStatemachineCallBack */
    /* USER CODE END MotorStatemachineCallBack */
}

static void TIMER1_Init(void)
{
    HAL_CRG_IpEnableSet(TIMER1_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(TIMER1_BASE, CRG_AHB_CLK_NO_PREDV);
    unsigned int load = (HAL_CRG_GetIpFreq((void *)TIMER1) / (1u << (TIMERPRESCALER_NO_DIV * 4)) / 1000000u) * 500;
    g_timer1.baseAddress = TIMER1;
    g_timer1.load        = load - 1; /* Set timer value immediately */
    g_timer1.bgLoad      = load - 1; /* Set timer value */
    g_timer1.mode        = TIMER_MODE_RUN_PERIODIC; /* Run in period mode */
    g_timer1.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer1.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer1.interruptEn = BASE_CFG_ENABLE;
    g_timer1.adcSocReqEnable = BASE_CFG_DISABLE;
    g_timer1.dmaReqEnable = BASE_CFG_DISABLE;
    HAL_TIMER_Init(&g_timer1);
    IRQ_Register(IRQ_TIMER1, HAL_TIMER_IrqHandler, &g_timer1);

    HAL_TIMER_RegisterCallback(&g_timer1, TIMER_PERIOD_FIN, MotorStatemachineCallBack);
    IRQ_SetPriority(IRQ_TIMER1, 1);
    IRQ_EnableN(IRQ_TIMER1);
}

static void IOConfig(void)
{
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode = 0;
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode_clear = 1;
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

    HAL_IOCMG_SetPinAltFuncMode(GPIO2_7_AS_PGA0_OUT);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_7_AS_PGA0_OUT, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_7_AS_PGA0_OUT, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_7_AS_PGA0_OUT, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_7_AS_PGA0_OUT, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO1_5_AS_PGA1_P0);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO1_5_AS_PGA1_P0, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO1_5_AS_PGA1_P0, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO1_5_AS_PGA1_P0, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO1_5_AS_PGA1_P0, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO1_6_AS_PGA1_N0);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO1_6_AS_PGA1_N0, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO1_6_AS_PGA1_N0, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO1_6_AS_PGA1_N0, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO1_6_AS_PGA1_N0, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO1_7_AS_PGA1_OUT);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO1_7_AS_PGA1_OUT, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO1_7_AS_PGA1_OUT, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO1_7_AS_PGA1_OUT, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO1_7_AS_PGA1_OUT, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO0_6_AS_GPIO0_6);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO0_6_AS_GPIO0_6, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO0_6_AS_GPIO0_6, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO0_6_AS_GPIO0_6, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO0_6_AS_GPIO0_6, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO2_4_AS_GPIO2_4);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_4_AS_GPIO2_4, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_4_AS_GPIO2_4, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_4_AS_GPIO2_4, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_4_AS_GPIO2_4, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO2_3_AS_GPIO2_3);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_3_AS_GPIO2_3, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_3_AS_GPIO2_3, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_3_AS_GPIO2_3, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_3_AS_GPIO2_3, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO3_7_AS_ACMP0_OUT);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO3_7_AS_ACMP0_OUT, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO3_7_AS_ACMP0_OUT, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO3_7_AS_ACMP0_OUT, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO3_7_AS_ACMP0_OUT, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO3_6_AS_ACMP_N4);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO3_6_AS_ACMP_N4, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO3_6_AS_ACMP_N4, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO3_6_AS_ACMP_N4, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO3_6_AS_ACMP_N4, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO3_5_AS_ACMP_P4);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO3_5_AS_ACMP_P4, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO3_5_AS_ACMP_P4, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO3_5_AS_ACMP_P4, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO3_5_AS_ACMP_P4, DRIVER_RATE_2);  /* Output signal edge fast/slow */
}

void SystemInit(void)
{
    /* init system module */
    IOConfig();
    ACMP0_Init();
    /* init APT module */
    APT0_Init();
    APT1_Init();
    APT2_Init();
    /* init ADC module */
    ADC0_Init();
    /* init PGA module */
    PGA0_Init();
    PGA1_Init();
    /* init TIMER module */
    TIMER1_Init();
    GPIO_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}