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
#include "iocmg.h"

#define UART0_BAND_RATE 921600

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

static void DMA_Channel0Init(void *handle)
{
    /* enable DMA function for uart data transfer */
    DMA_ChannelParam dma_param;
    dma_param.direction = DMA_MEMORY_TO_PERIPH_BY_DMAC;
    dma_param.srcAddrInc = DMA_ADDR_INCREASE;
    dma_param.destAddrInc = DMA_ADDR_UNALTERED;
    dma_param.srcPeriph = DMA_REQUEST_MEM;
    /* enable UART0 tx mode DMA */
    dma_param.destPeriph = DMA_REQUEST_UART0_TX;
    dma_param.srcWidth = DMA_TRANSWIDTH_BYTE;
    dma_param.destWidth = DMA_TRANSWIDTH_BYTE;
    dma_param.srcBurst = DMA_BURST_LENGTH_1;
    dma_param.destBurst = DMA_BURST_LENGTH_1;
    dma_param.pHandle = handle;
    /* init DMA config module */
    HAL_DMA_InitChannel(&g_dmac, &dma_param, DMA_CHANNEL_ZERO);
}

static void DMA_Init(void)
{
    HAL_CRG_IpEnableSet(DMA_BASE, IP_CLK_ENABLE);
    g_dmac.baseAddress = DMA;
    g_dmac.srcByteOrder = DMA_BYTEORDER_SMALLENDIAN;
    g_dmac.destByteOrder = DMA_BYTEORDER_SMALLENDIAN;
    g_dmac.irqNumTc = IRQ_DMA_TC;
    g_dmac.irqNumError = IRQ_DMA_ERR;
    HAL_DMA_IRQService(&g_dmac);
    IRQ_EnableN(IRQ_DMA_TC);
    IRQ_EnableN(IRQ_DMA_ERR);
    HAL_DMA_Init(&g_dmac);

    DMA_Channel0Init((void *)(&g_uart0));
}

static void ACMP1_Init(void)
{
    HAL_CRG_IpEnableSet(ACMP1_BASE, BASE_CFG_ENABLE);

    g_acmp1.baseAddress =  ACMP1_BASE;  /* set acmp base address */

    g_acmp1.enable = true;  /* enable acmp input capture function */
    g_acmp1.syncEn = false;
    g_acmp1.inOutConfig.vinNNum = ACMP_VIN_MUX3;
    g_acmp1.inOutConfig.vinPNum = ACMP_VIN_MUX3;
    g_acmp1.inOutConfig.swVinPNum = ACMP_SW_VIN3;
    g_acmp1.inOutConfig.swVinNNum = ACMP_SW_VIN3;
    /* set acmp polarity */
    g_acmp1.inOutConfig.polarity = ACMP_OUT_NOT_INVERT;
    g_acmp1.filterCtrl.filterMode = ACMP_FILTER_NONE;
    g_acmp1.hysteresisVol = ACMP_HYS_VOL_ZERO;
    HAL_ACMP_Init(&g_acmp1);
}

static void ADC0_Init(void)
{
    HAL_CRG_IpEnableSet(ADC0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(ADC0_BASE, CRG_ADC_CLK_SELECT_PLL_DIV);
    HAL_CRG_IpClkDivSet(ADC0_BASE, CRG_ADC_DIV_5);

    g_adc0.baseAddress = ADC0;
    g_adc0.socPriority = ADC_PRIMODE_ALL_ROUND;
    g_adc0.vrefBuf = ADC_VREF_2P5V;
    g_adc0.irqNumOver = IRQ_ADC0_OVINT;
    g_adc0.ADC_IntxParam[0].irqNum = IRQ_ADC0_INT1;     /* interrupt 0 */
    g_adc0.ADC_IntxParam[1].irqNum = IRQ_ADC0_INT2;     /* interrupt 1 */
    g_adc0.ADC_IntxParam[2].irqNum = IRQ_ADC0_INT3;     /* interrupt 2 */
    g_adc0.ADC_IntxParam[3].irqNum = IRQ_ADC0_INT4;     /* interrupt 3 */

    HAL_ADC_Init(&g_adc0);

    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINA0; /* PGA0_OUT(ADC INA0) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT0_SOCA;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc0, ADC_SOC_NUM8, &socParam);
}

static void ADC1_Init(void)
{
    HAL_CRG_IpEnableSet(ADC1_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(ADC1_BASE, CRG_ADC_CLK_SELECT_PLL_DIV);
    HAL_CRG_IpClkDivSet(ADC1_BASE, CRG_ADC_DIV_5);

    g_adc1.baseAddress = ADC1;
    g_adc1.socPriority = ADC_PRIMODE_ALL_ROUND;
    g_adc1.vrefBuf = ADC_VREF_2P5V;
    g_adc1.irqNumOver = IRQ_ADC1_OVINT;
    g_adc1.ADC_IntxParam[0].irqNum = IRQ_ADC1_INT1;     /* interrupt 0 */
    g_adc1.ADC_IntxParam[1].irqNum = IRQ_ADC1_INT2;     /* interrupt 1 */
    g_adc1.ADC_IntxParam[2].irqNum = IRQ_ADC1_INT3;     /* interrupt 2 */
    g_adc1.ADC_IntxParam[3].irqNum = IRQ_ADC1_INT4;     /* interrupt 3 */

    HAL_ADC_Init(&g_adc1);

    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINA0; /* PGA1_OUT(ADC INA0) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT0_SOCA;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc1, ADC_SOC_NUM8, &socParam);

    socParam.adcInput = ADC_CH_ADCINB6; /* TSENSOR(ADC INB6) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_SOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_NONEPERIPH;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc1, ADC_SOC_NUM0, &socParam);
}

static void ADC2_Init(void)
{
    HAL_CRG_IpEnableSet(ADC2_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(ADC2_BASE, CRG_ADC_CLK_SELECT_PLL_DIV);
    HAL_CRG_IpClkDivSet(ADC2_BASE, CRG_ADC_DIV_5);

    g_adc2.baseAddress = ADC2;
    g_adc2.socPriority = ADC_PRIMODE_ALL_ROUND;
    g_adc2.vrefBuf = ADC_VREF_2P5V;
    g_adc2.irqNumOver = IRQ_ADC2_OVINT;
    g_adc2.ADC_IntxParam[0].irqNum = IRQ_ADC2_INT1;     /* interrupt 0 */
    g_adc2.ADC_IntxParam[1].irqNum = IRQ_ADC2_INT2;     /* interrupt 1 */
    g_adc2.ADC_IntxParam[2].irqNum = IRQ_ADC2_INT3;     /* interrupt 2 */
    g_adc2.ADC_IntxParam[3].irqNum = IRQ_ADC2_INT4;     /* interrupt 3 */

    HAL_ADC_Init(&g_adc2);

    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINB1; /* PIN31(ADC INB1) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_SOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_NONEPERIPH;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc2, ADC_SOC_NUM0, &socParam);

    socParam.adcInput = ADC_CH_ADCINB2; /* PIN40(ADC INB2) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_SOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_NONEPERIPH;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc2, ADC_SOC_NUM1, &socParam);

    socParam.adcInput = ADC_CH_ADCINA7; /* PIN36(ADC INA7) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_SOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_NONEPERIPH;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc2, ADC_SOC_NUM14, &socParam);
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
    protectApt.ocEventModeEx =APT_OUT_CTRL_ONE_SHOT;
    protectApt.cbcClrModeEx = APT_CLEAR_CBC_ON_CNTR_ZERO;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;    /* low action protection */
    protectApt.ocActionBEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = BASE_CFG_ENABLE;
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_ACMP1;
    protectApt.evtPolarityMaskEx = APT_EM_ACMP1_INVERT_BIT;
    protectApt.filterCycleNumEx = 0;
    /* set apt protect register */
    HAL_APT_ProtectInitEx(&g_apt0, &protectApt);
}

static void APT0_Init(void)
{
    HAL_CRG_IpEnableSet(APT0_BASE, IP_CLK_ENABLE);

    g_apt0.baseAddress = APT0;
    g_apt0.irqNumEvt = IRQ_APT0_EVT;
    g_apt0.irqNumTmr = IRQ_APT0_TMR;

    /* Clock Settings */
    g_apt0.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt0.waveform.timerPeriod = 10000; /* apt timer count period is 10000 */
    g_apt0.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_apt0.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt0.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt0.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt0.waveform.divInitVal = 0;
    g_apt0.waveform.cntInitVal = 0;
    g_apt0.waveform.cntCmpLeftEdge = 5000;  /* apt left edge count period is 5000 */
    g_apt0.waveform.cntCmpRightEdge = 5000; /* apt right edge count period is 5000 */
    g_apt0.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt0.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_apt0.waveform.deadBandCnt = 300; /* apt dead band count value is 300 */

    /* ADC Trigger SOCA */
    g_apt0.adcTrg.trgEnSOCA = BASE_CFG_ENABLE;
    g_apt0.adcTrg.cntCmpSOCA = 1;
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
    IRQ_SetPriority(g_apt0.irqNumEvt, 7); /* apt event interupt level is 7 */
    HAL_APT_IRQService(&g_apt0);
    IRQ_EnableN(g_apt0.irqNumEvt);
    HAL_APT_RegisterCallBack(&g_apt0, APT_TIMER_INTERRUPT, MotorCarrierProcessCallback);
    IRQ_SetPriority(g_apt0.irqNumTmr, 6); /* apt timer interupt level is 6 */
    HAL_APT_IRQService(&g_apt0);
    IRQ_EnableN(g_apt0.irqNumTmr);
}

static void APT1_ProtectInit(void)
{
    APT_OutCtrlProtectEx protectApt = {0};
    /* enable apt1 event interupt protection function */
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.cbcClrModeEx = APT_CLEAR_CBC_ON_CNTR_ZERO;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocActionBEx = APT_OUT_CTRL_ACTION_LOW;
    /* disable apt1 event interupt protection function */
    protectApt.ocEvtInterruptEnEx = BASE_CFG_DISABLE;
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_ACMP1;
    protectApt.evtPolarityMaskEx = APT_EM_ACMP1_INVERT_BIT;
    protectApt.filterCycleNumEx = 0;
    /* init APT config module */
    HAL_APT_ProtectInitEx(&g_apt1, &protectApt);
}

static void APT1_Init(void)
{
    HAL_CRG_IpEnableSet(APT1_BASE, IP_CLK_ENABLE);

    g_apt1.baseAddress = APT1;
    g_apt1.irqNumEvt = IRQ_APT1_EVT;
    g_apt1.irqNumTmr = IRQ_APT1_TMR;

    /* Clock Settings */
    g_apt1.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt1.waveform.timerPeriod = 10000; /* apt timer count period is 10000 */
    g_apt1.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_apt1.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt1.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt1.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt1.waveform.divInitVal = 0;
    g_apt1.waveform.cntInitVal = 0;
    g_apt1.waveform.cntCmpLeftEdge = 5000;  /* apt left edge count period is 5000 */
    g_apt1.waveform.cntCmpRightEdge = 5000; /* apt right edge count period is 5000 */
    g_apt1.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt1.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_apt1.waveform.deadBandCnt = 300;      /* apt dead band count value is 300 */

    APT1_ProtectInit();

    HAL_APT_PWMInit(&g_apt1);
}

static void APT2_ProtectInit(void)
{
    APT_OutCtrlProtectEx protectApt = {0};
    /* enable apt2 event interupt protection function */
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx =APT_OUT_CTRL_ONE_SHOT;
    protectApt.cbcClrModeEx = APT_CLEAR_CBC_ON_CNTR_ZERO;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocActionBEx = APT_OUT_CTRL_ACTION_LOW;
    /* disable apt2 event interupt protection function */
    protectApt.ocEvtInterruptEnEx = BASE_CFG_DISABLE;
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_ACMP1;
    protectApt.evtPolarityMaskEx = APT_EM_ACMP1_INVERT_BIT;
    protectApt.filterCycleNumEx = 0;
    /* init APT config module */
    HAL_APT_ProtectInitEx(&g_apt2, &protectApt);
}

static void APT2_Init(void)
{
    HAL_CRG_IpEnableSet(APT2_BASE, IP_CLK_ENABLE);

    g_apt2.baseAddress = APT2;
    g_apt2.irqNumEvt = IRQ_APT2_EVT;
    g_apt2.irqNumTmr = IRQ_APT2_TMR;

    /* Clock Settings */
    g_apt2.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt2.waveform.timerPeriod = 10000;    /* apt timer count period is 10000 */
    g_apt2.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_apt2.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt2.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt2.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt2.waveform.divInitVal = 0;
    g_apt2.waveform.cntInitVal = 0;
    g_apt2.waveform.cntCmpLeftEdge = 5000;  /* apt left edge count period is 5000 */
    g_apt2.waveform.cntCmpRightEdge = 5000; /* apt right edge count period is 5000 */
    g_apt2.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt2.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_apt2.waveform.deadBandCnt = 300;  /* apt dead band count value is 300 */

    APT2_ProtectInit();

    HAL_APT_PWMInit(&g_apt2);
}

__weak void MotorStartStopKeyCallback(void *param)
{
    /* gpio input interupt function */
    GPIO_Handle *handle = (GPIO_Handle *)param;
    BASE_FUNC_UNUSED(handle);
}

static void GPIO_Init(void)
{
    HAL_CRG_IpEnableSet(GPIO0_BASE, IP_CLK_ENABLE);
    g_gpio0.baseAddress = GPIO0;
    /* set gpio0_6_7 output mode */
    g_gpio0.dir = GPIO_OUTPUT_MODE;
    g_gpio0.value = GPIO_HIGH_LEVEL;
    g_gpio0.interruptMode = GPIO_INT_TYPE_NONE;
    g_gpio0.pins = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(&g_gpio0);
    /* enable  GPIO clk */
    HAL_CRG_IpEnableSet(GPIO2_BASE, IP_CLK_ENABLE);
    g_gpio2.baseAddress = GPIO2;
    /* set gpio2_6 output mode */
    g_gpio2.dir = GPIO_OUTPUT_MODE;
    g_gpio2.value = GPIO_HIGH_LEVEL;
    g_gpio2.interruptMode = GPIO_INT_TYPE_NONE;
    g_gpio2.pins = GPIO_PIN_6;
    HAL_GPIO_Init(&g_gpio2);
    /* set gpio2_2 input mode */
    g_gpio2.dir = GPIO_INPUT_MODE;
    g_gpio2.value = GPIO_HIGH_LEVEL;
    g_gpio2.interruptMode = GPIO_INT_TYPE_LOW_LEVEL;
    g_gpio2.pins = GPIO_PIN_2;
    HAL_GPIO_Init(&g_gpio2);
    /* enable GPIO interupt function */
    HAL_GPIO_RegisterCallBack(&g_gpio2, GPIO_PIN_2, MotorStartStopKeyCallback);
    g_gpio2.irqNum = IRQ_GPIO2;
    HAL_GPIO_IRQService(&g_gpio2);
    IRQ_SetPriority(g_gpio2.irqNum, 1); /* set gpio1 interrupt priority to 1, 1~7 */
    IRQ_EnableN(g_gpio2.irqNum); /* gpio interrupt enable */

    return;
}

static void PGA0_Init(void)
{
    HAL_CRG_IpEnableSet(PGA0_BASE, IP_CLK_ENABLE);

    g_pga0.baseAddress = PGA0_BASE;
    g_pga0.enable = BASE_CFG_ENABLE; /* enable pga0 */
    g_pga0.extLoopbackEn = BASE_CFG_DISABLE;
    g_pga0.pgaMux = PGA_EXT_RES_VI0;
    g_pga0.gain = PGA_GAIN_1X;
    /* init pga0 module */
    HAL_PGA_Init(&g_pga0);
}

static void PGA1_Init(void)
{
    HAL_CRG_IpEnableSet(PGA1_BASE, IP_CLK_ENABLE);

    g_pga1.baseAddress = PGA1_BASE; /* enable pga1 */
    g_pga1.enable = BASE_CFG_ENABLE;
    g_pga1.extLoopbackEn = BASE_CFG_DISABLE;
    g_pga1.pgaMux = PGA_EXT_RES_VI3;
    g_pga1.gain = PGA_GAIN_1X;
    /* init pga1 module */
    HAL_PGA_Init(&g_pga1);
}

__weak void CheckPotentiometerValueCallback(void *handle)
{
    HAL_TIMER_IrqClear((TIMER_Handle *)handle);
    /* USER CODE BEGIN TIMER0 ITCallBackFunc */
    /* USER CODE END TIMER0 ITCallBackFunc */
}

static void TIMER0_Init(void)
{
    unsigned int load = (HAL_CRG_GetIpFreq((void *)TIMER0) / (1u << (TIMERPRESCALER_NO_DIV * 4)) / 1000000u) * 1000000;

    HAL_CRG_IpEnableSet(TIMER0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(TIMER0_BASE, CRG_PLL_NO_PREDV);

    g_timer0.baseAddress = TIMER0;
    g_timer0.irqNum = IRQ_TIMER0;

    g_timer0.load        = load - 1; /* Set timer value immediately */
    g_timer0.bgLoad      = load - 1; /* Set timer value */
    g_timer0.mode        = TIMER_MODE_RUN_PERIODIC; /* Run in period mode */
    g_timer0.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer0.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer0.dmaAdcSingleReqEnable = BASE_CFG_DISABLE;
    g_timer0.dmaBurstReqEnable = BASE_CFG_DISABLE;
    g_timer0.interruptEn = BASE_CFG_ENABLE;
    HAL_TIMER_Init(&g_timer0);

    HAL_TIMER_RegisterCallback(&g_timer0, CheckPotentiometerValueCallback);
    IRQ_SetPriority(g_timer0.irqNum, 1);
    IRQ_EnableN(g_timer0.irqNum);
}

__weak void MotorStatemachineCallBack(void *handle)
{
    HAL_TIMER_IrqClear((TIMER_Handle *)handle);
    /* USER CODE BEGIN TIMER1 ITCallBackFunc */
    /* USER CODE END TIMER1 ITCallBackFunc */
}

static void TIMER1_Init(void)
{
    unsigned int load = (HAL_CRG_GetIpFreq((void *)TIMER1) / (1u << (TIMERPRESCALER_NO_DIV * 4)) / 1000000u) * 500;

    HAL_CRG_IpEnableSet(TIMER1_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(TIMER1_BASE, CRG_PLL_NO_PREDV);

    g_timer1.baseAddress = TIMER1;
    g_timer1.irqNum = IRQ_TIMER1;

    g_timer1.load        = load - 1; /* Set timer value immediately */
    g_timer1.bgLoad      = load - 1; /* Set timer value */
    g_timer1.mode        = TIMER_MODE_RUN_PERIODIC; /* Run in period mode */
    g_timer1.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer1.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer1.dmaAdcSingleReqEnable = BASE_CFG_DISABLE;
    g_timer1.dmaBurstReqEnable = BASE_CFG_DISABLE;
    g_timer1.interruptEn = BASE_CFG_ENABLE;
    HAL_TIMER_Init(&g_timer1);

    HAL_TIMER_RegisterCallback(&g_timer1, MotorStatemachineCallBack);
    IRQ_SetPriority(g_timer1.irqNum, 1);
    IRQ_EnableN(g_timer1.irqNum);
}

__weak void UART0WriteInterruptCallback(UART_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART0_WRITE_IT_FINISH */
    /* USER CODE END UART0_WRITE_IT_FINISH */
}

__weak void UART0ReadInterruptCallback(UART_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART0_READ_IT_FINISH */
    /* USER CODE END UART0_READ_IT_FINISH */
}

__weak void UART0InterruptErrorCallback(UART_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART0_TRNS_IT_ERROR */
    /* USER CODE END UART0_TRNS_IT_ERROR */
}

__weak void UART0_TXDMACallback(UART_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART0_WRITE_DMA_FINISH */
    /* USER CODE END UART0_WRITE_DMA_FINISH */
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
    g_uart0.txMode = UART_MODE_DMA;
    g_uart0.rxMode = UART_MODE_INTERRUPT;
    g_uart0.fifoMode = BASE_CFG_ENABLE;
    g_uart0.fifoTxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.fifoRxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0);
    HAL_UART_RegisterCallBack(&g_uart0, UART_WRITE_IT_FINISH, UART0WriteInterruptCallback);
    HAL_UART_RegisterCallBack(&g_uart0, UART_READ_IT_FINISH, UART0ReadInterruptCallback);
    HAL_UART_RegisterCallBack(&g_uart0, UART_TRNS_IT_ERROR, UART0InterruptErrorCallback);
    HAL_UART_IRQService(&g_uart0);
    IRQ_SetPriority(g_uart0.irqNum, 1);
    IRQ_EnableN(g_uart0.irqNum);
    g_uart0.dmaHandle = &g_dmac;
    g_uart0.uartDmaTxChn = 0;
    HAL_UART_RegisterCallBack(&g_uart0, UART_WRITE_DMA_FINISH, UART0_TXDMACallback);
}

static void IOConfig(void)
{
    IOConfig_RegStruct *iconfig = IOCONFIG;

    iconfig->iocmg_26.BIT.func = 0x2; /* 0x2 is ACMP1_OUT */
    iconfig->iocmg_26.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_26.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_26.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_26.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_26.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_27.BIT.func = 0x9; /* 0x9 is ACMP1_ANA_N */
    iconfig->iocmg_27.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_27.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_27.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_27.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_27.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_28.BIT.func = 0x9; /* 0x9 is ACMP1_ANA_P */
    iconfig->iocmg_28.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_28.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_28.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_28.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_28.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_48.BIT.func = 0x8; /* 0x8 is ADC2_ANA_B1 */
    iconfig->iocmg_48.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_48.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_48.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_48.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_48.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_55.BIT.func = 0x8; /* 0x8 is ADC2_ANA_B2 */
    iconfig->iocmg_55.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_55.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_55.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_55.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_55.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_51.BIT.func = 0x8; /* 0x8 is ADC2_ANA_A7 */
    iconfig->iocmg_51.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_51.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_51.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_51.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_51.BIT.se = BASE_CFG_DISABLE;

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

    iconfig->iocmg_33.BIT.func = 0x3; /* 0x3 is APT1_PWMA */
    iconfig->iocmg_33.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_33.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_33.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_33.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_33.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_36.BIT.func = 0x3; /* 0x3 is APT1_PWMB */
    iconfig->iocmg_36.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_36.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_36.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_36.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_36.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_34.BIT.func = 0x3; /* 0x3 is APT2_PWMA */
    iconfig->iocmg_34.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_34.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_34.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_34.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_34.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_37.BIT.func = 0x3; /* 0x3 is APT2_PWMB */
    iconfig->iocmg_37.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_37.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_37.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_37.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_37.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_9.BIT.func = 0x0; /* 0x0 is GPIO0_6 */
    iconfig->iocmg_9.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_9.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_9.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_9.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_9.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_10.BIT.func = 0x0; /* 0x0 is GPIO0_7 */
    iconfig->iocmg_10.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_10.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_10.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_10.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_10.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_25.BIT.func = 0x0; /* 0x0 is GPIO2_6 */
    iconfig->iocmg_25.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_25.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_25.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_25.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_25.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_21.BIT.func = 0x0; /* 0x0 is GPIO2_2 */
    iconfig->iocmg_21.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_21.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_21.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_21.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_21.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_38.BIT.func = 0x9; /* 0x9 is PGA0_ANA_P */
    iconfig->iocmg_38.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_38.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_38.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_38.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_38.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_39.BIT.func = 0x9; /* 0x9 is PGA0_ANA_N */
    iconfig->iocmg_39.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_39.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_39.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_39.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_39.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_40.BIT.func = 0x8; /* 0x8 is PGA0_ANA_EXT */
    iconfig->iocmg_40.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_40.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_40.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_40.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_40.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_43.BIT.func = 0x9; /* 0x9 is PGA1_ANA_EXT */
    iconfig->iocmg_43.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_43.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_43.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_43.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_43.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_42.BIT.func = 0x9; /* 0x9 is PGA1_ANA_N */
    iconfig->iocmg_42.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_42.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_42.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_42.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_42.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_41.BIT.func = 0x9; /* 0x9 is PGA1_ANA_P */
    iconfig->iocmg_41.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_41.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_41.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_41.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_41.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_6.BIT.func = 0x4; /* 0x4 is UART0_TXD */
    iconfig->iocmg_6.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_6.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_6.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_6.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_6.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_7.BIT.func = 0x4; /* 0x4 is UART0_RXD */
    iconfig->iocmg_7.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_7.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_7.BIT.pu = BASE_CFG_ENABLE;
    iconfig->iocmg_7.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_7.BIT.se = BASE_CFG_DISABLE;
}

void SystemInit(void)
{
    /* init system module */
    IOConfig();
    DMA_Init();
    UART0_Init();
    ACMP1_Init();
    /* init APT module */
    APT0_Init();
    APT1_Init();
    APT2_Init();
    /* init ADC module */
    ADC0_Init();
    ADC1_Init();
    ADC2_Init();
    /* init PGA module */
    PGA0_Init();
    PGA1_Init();
    /* init TIMER module */
    TIMER0_Init();
    TIMER1_Init();
    GPIO_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}