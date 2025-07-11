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
#include "debug.h"

#define UART0_BAND_RATE 115200
#define UART2_BAND_RATE 115200
#define UART3_BAND_RATE 9600

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
    crg.handleEx.clk1MDiv = (25 - 1); /* The 1 MHz freq is equal to the input clock frequency / (clk_1m_div + 1). */

    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
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
    socParam.adcInput = ADC_CH_ADCINA12; /* PIN10(ADC AIN12) */
    socParam.sampleTotalTime = ADC_SOCSAMPLE_5CLK; /* adc sample total time 5 adc_clk */
    socParam.trigSource = ADC_TRIGSOC_SOFT;
    socParam.continueMode = BASE_CFG_DISABLE;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc0, ADC_SOC_NUM1, &socParam);
}

static void APT0_Init(void)
{
    HAL_CRG_IpEnableSet(APT0_BASE, IP_CLK_ENABLE);

    g_apt0.baseAddress = APT0;

    /* Clock Settings */
    g_apt0.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt0.waveform.timerPeriod = 1500;
    g_apt0.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_apt0.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt0.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt0.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt0.waveform.divInitVal = 0;
    g_apt0.waveform.cntInitVal = 0;
    g_apt0.waveform.cntCmpLeftEdge = 250;
    g_apt0.waveform.cntCmpRightEdge = 250;
    g_apt0.waveform.cntCmpLoadMode = APT_BUFFER_DISABLE;
    g_apt0.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_apt0.waveform.deadBandCnt = 10;

    HAL_APT_PWMInit(&g_apt0);
}

static void APT1_Init(void)
{
    HAL_CRG_IpEnableSet(APT1_BASE, IP_CLK_ENABLE);

    g_apt1.baseAddress = APT1;

    /* Clock Settings */
    g_apt1.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt1.waveform.timerPeriod = 1500;
    g_apt1.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_apt1.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt1.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt1.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt1.waveform.divInitVal = 0;
    g_apt1.waveform.cntInitVal = 0;
    g_apt1.waveform.cntCmpLeftEdge = 250;
    g_apt1.waveform.cntCmpRightEdge = 250;
    g_apt1.waveform.cntCmpLoadMode = APT_BUFFER_DISABLE;
    g_apt1.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_apt1.waveform.deadBandCnt = 10;

    HAL_APT_PWMInit(&g_apt1);
}

static void CAN_Init(void){
    HAL_CRG_IpEnableSet(CAN_BASE, IP_CLK_ENABLE);

    g_can.baseAddress = CAN;

    g_can.typeMode = CAN_MODE_NORMAL;
    g_can.seg1Phase = CAN_SEG1_5TQ;
    g_can.seg2Phase = CAN_SEG2_4TQ;
    g_can.sjw = CAN_SJW_1TQ; // 2
    g_can.prescalser = 5;  /* 25 is frequency division coefficient */
    g_can.rxFIFODepth = 4;  /* A maximum of 4 packet objects are in RX FIFO */ 
    g_can.autoRetrans = BASE_CFG_ENABLE;
    HAL_CAN_Init(&g_can);

    HAL_CAN_RegisterCallBack(&g_can, CAN_WRITE_FINISH, (CAN_CallbackType)CANWriteCallbackFunc);
    HAL_CAN_RegisterCallBack(&g_can, CAN_READ_FINISH, (CAN_CallbackType)CANReadCallbackFunc);
    HAL_CAN_RegisterCallBack(&g_can, CAN_TRNS_ERROR, (CAN_CallbackType)CANTransCallbackFunc);

    IRQ_Register(IRQ_CAN, HAL_CAN_IrqHandler, &g_can);
    IRQ_SetPriority(IRQ_CAN, 2);
    IRQ_EnableN(IRQ_CAN);
}

__weak void GPIO1_0_CallbackFunc(void *param)
{
    GPIO_Handle *handle = (GPIO_Handle *)param;
    BASE_FUNC_UNUSED(handle);
}
__weak void GPIO1_1_CallbackFunc(void *param)
{
    GPIO_Handle *handle = (GPIO_Handle *)param;
    BASE_FUNC_UNUSED(handle);
}

__weak void CANWriteCallbackFunc(void* handle){
    BASE_FUNC_UNUSED(handle);
}

__weak void CANReadCallbackFunc(void* handle){
    BASE_FUNC_UNUSED(handle);
}

__weak void CANTransCallbackFunc(void* handle){
    BASE_FUNC_UNUSED(handle);
}

static void GPIO_Init(void)
{
    HAL_CRG_IpEnableSet(GPIO1_BASE, IP_CLK_ENABLE);
    g_gpio1.baseAddress = GPIO1;

    g_gpio1.pins = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(&g_gpio1);
    HAL_GPIO_SetDirection(&g_gpio1, g_gpio1.pins, GPIO_INPUT_MODE);
    HAL_GPIO_SetValue(&g_gpio1, g_gpio1.pins, GPIO_LOW_LEVEL);
    HAL_GPIO_SetIrqType(&g_gpio1, g_gpio1.pins, GPIO_INT_TYPE_RISE_EDGE);

    HAL_CRG_IpEnableSet(GPIO2_BASE, IP_CLK_ENABLE);
    g_gpio2.baseAddress = GPIO2;

    g_gpio2.pins = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(&g_gpio2);
    HAL_GPIO_SetDirection(&g_gpio2, g_gpio2.pins, GPIO_OUTPUT_MODE);
    HAL_GPIO_SetValue(&g_gpio2, g_gpio2.pins, GPIO_HIGH_LEVEL);
    HAL_GPIO_SetIrqType(&g_gpio2, g_gpio2.pins, GPIO_INT_TYPE_NONE);

    HAL_CRG_IpEnableSet(GPIO5_BASE, IP_CLK_ENABLE);
    g_gpio5.baseAddress = GPIO5;

    g_gpio5.pins = GPIO_PIN_1;
    HAL_GPIO_Init(&g_gpio5);
    HAL_GPIO_SetDirection(&g_gpio5, g_gpio5.pins, GPIO_OUTPUT_MODE);
    HAL_GPIO_SetValue(&g_gpio5, g_gpio5.pins, GPIO_LOW_LEVEL);
    HAL_GPIO_SetIrqType(&g_gpio5, g_gpio5.pins, GPIO_INT_TYPE_NONE);

    HAL_GPIO_RegisterCallBack(&g_gpio1, GPIO_PIN_0, GPIO1_0_CallbackFunc);
    HAL_GPIO_RegisterCallBack(&g_gpio1, GPIO_PIN_1, GPIO1_1_CallbackFunc);
    IRQ_Register(IRQ_GPIO1, HAL_GPIO_IrqHandler, &g_gpio1);
    IRQ_SetPriority(IRQ_GPIO1, 1); /* set gpio1 interrupt priority to 1, 1~15 */
    IRQ_EnableN(IRQ_GPIO1); /* gpio interrupt enable */

    return;
}

static void GPT0_Init(void)
{
    HAL_CRG_IpEnableSet(GPT0_BASE, IP_CLK_ENABLE);

    g_gpt0.baseAddress = GPT0;
    g_gpt0.clockDiv = 500 - 1;  /* 1000 is the internal frequency division of GPT */
    g_gpt0.period = 1000;  /* 2996 is the number of GPT counting cycles. */
    g_gpt0.refA0.refdot = 1;  /* 1 is the value of PWM reference point A. */
    g_gpt0.refA0.refAction = GPT_ACTION_OUTPUT_HIGH;  /* GPT Action High */
    g_gpt0.refB0.refdot = 76;  /* 76 is the value of PWM reference point B. */
    g_gpt0.refB0.refAction = GPT_ACTION_OUTPUT_LOW; /* GPT Action Low */
    g_gpt0.bufLoad = BASE_CFG_ENABLE;
    g_gpt0.pwmKeep = BASE_CFG_ENABLE;
    g_gpt0.handleEx.periodIntEnable = BASE_CFG_DISABLE;
    g_gpt0.handleEx.outputFinIntEnable = BASE_CFG_DISABLE;
    g_gpt0.triggleAdcOutFinish = BASE_CFG_DISABLE;
    g_gpt0.triggleAdcPeriod = BASE_CFG_DISABLE;

    HAL_GPT_Init(&g_gpt0);
}

static void GPT1_Init(void)
{
    HAL_CRG_IpEnableSet(GPT1_BASE, IP_CLK_ENABLE);

    g_gpt1.baseAddress = GPT1;
    g_gpt1.clockDiv = 500 - 1;  /* 1000 is the internal frequency division of GPT */
    g_gpt1.period = 1000 - 1;  /* 49999 is the number of GPT counting cycles. */
    g_gpt1.refA0.refdot = 1;  /* 10000 is the value of PWM reference point A. */
    g_gpt1.refA0.refAction = GPT_ACTION_OUTPUT_HIGH;  /* GPT Action High */
    g_gpt1.refB0.refdot = 2;  /* 30000 is the value of PWM reference point B. */
    g_gpt1.refB0.refAction = GPT_ACTION_OUTPUT_LOW; /* GPT Action Low */
    g_gpt1.bufLoad = BASE_CFG_ENABLE;
    g_gpt1.pwmKeep = BASE_CFG_ENABLE;
    g_gpt1.handleEx.periodIntEnable = BASE_CFG_DISABLE;
    g_gpt1.handleEx.outputFinIntEnable = BASE_CFG_DISABLE;
    g_gpt1.triggleAdcOutFinish = BASE_CFG_DISABLE;
    g_gpt1.triggleAdcPeriod = BASE_CFG_DISABLE;

    HAL_GPT_Init(&g_gpt1);
}

static void GPT2_Init(void)
{
    HAL_CRG_IpEnableSet(GPT2_BASE, IP_CLK_ENABLE);

    g_gpt2.baseAddress = GPT2;
    g_gpt2.clockDiv = 500 - 1;  /* 1000 is the internal frequency division of GPT */
    g_gpt2.period = 1000 - 1;  /* 49999 is the number of GPT counting cycles. */
    g_gpt2.refA0.refdot = 1;  /* 10000 is the value of PWM reference point A. */
    g_gpt2.refA0.refAction = GPT_ACTION_OUTPUT_HIGH;  /* GPT Action High */
    g_gpt2.refB0.refdot = 2;  /* 30000 is the value of PWM reference point B. */
    g_gpt2.refB0.refAction = GPT_ACTION_OUTPUT_LOW; /* GPT Action Low */
    g_gpt2.bufLoad = BASE_CFG_ENABLE;
    g_gpt2.pwmKeep = BASE_CFG_ENABLE;
    g_gpt2.handleEx.periodIntEnable = BASE_CFG_DISABLE;
    g_gpt2.handleEx.outputFinIntEnable = BASE_CFG_DISABLE;
    g_gpt2.triggleAdcOutFinish = BASE_CFG_DISABLE;
    g_gpt2.triggleAdcPeriod = BASE_CFG_DISABLE;

    HAL_GPT_Init(&g_gpt2);
}

static void GPT3_Init(void)
{
    HAL_CRG_IpEnableSet(GPT3_BASE, IP_CLK_ENABLE);

    g_gpt3.baseAddress = GPT3;
    g_gpt3.clockDiv = 500 - 1;  /* 1000 is the internal frequency division of GPT */
    g_gpt3.period = 1000 - 1;  /* 300 is the number of GPT counting cycles. */
    g_gpt3.refA0.refdot = 1;  /* 100 is the value of PWM reference point A. */
    g_gpt3.refA0.refAction = GPT_ACTION_OUTPUT_HIGH;  /* GPT Action High */
    g_gpt3.refB0.refdot = 200;  /* 200 is the value of PWM reference point B. */
    g_gpt3.refB0.refAction = GPT_ACTION_OUTPUT_LOW; /* GPT Action Low */
    g_gpt3.bufLoad = BASE_CFG_ENABLE;
    g_gpt3.pwmKeep = BASE_CFG_ENABLE;
    g_gpt3.handleEx.periodIntEnable = BASE_CFG_DISABLE;
    g_gpt3.handleEx.outputFinIntEnable = BASE_CFG_DISABLE;
    g_gpt3.triggleAdcOutFinish = BASE_CFG_DISABLE;
    g_gpt3.triggleAdcPeriod = BASE_CFG_DISABLE;

    HAL_GPT_Init(&g_gpt3);
}

static void I2C0_Init(void)
{
    HAL_CRG_IpEnableSet(I2C0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(I2C0_BASE, CRG_AHB_CLK_NO_PREDV);

    g_i2c0.baseAddress = I2C0;

    g_i2c0.functionMode = I2C_MODE_SELECT_MASTER_ONLY;
    g_i2c0.addrMode = I2C_7_BITS;
    g_i2c0.sdaHoldTime = 10;
    g_i2c0.freq = 400000;
    g_i2c0.transferBuff = NULL;
    g_i2c0.ignoreAckFlag = BASE_CFG_DISABLE;
    g_i2c0.handleEx.spikeFilterTime = 0;
    g_i2c0.handleEx.sdaDelayTime = 0;
    g_i2c0.timeout = 10000;
    g_i2c0.state = I2C_STATE_RESET;
    HAL_I2C_Init(&g_i2c0);
}

static void I2C1_Init(void)
{
    HAL_CRG_IpEnableSet(I2C1_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(I2C1_BASE, CRG_AHB_CLK_NO_PREDV);

    g_i2c1.baseAddress = I2C1;

    g_i2c1.functionMode = I2C_MODE_SELECT_MASTER_ONLY;
    g_i2c1.addrMode = I2C_7_BITS;
    g_i2c1.sdaHoldTime = 10;
    g_i2c1.freq = 200000;
    g_i2c1.transferBuff = NULL;
    g_i2c1.ignoreAckFlag = BASE_CFG_DISABLE;
    g_i2c1.handleEx.spikeFilterTime = 0;
    g_i2c1.handleEx.sdaDelayTime = 0;
    g_i2c1.timeout = 10000;
    g_i2c1.state = I2C_STATE_RESET;
    HAL_I2C_Init(&g_i2c1);
}

__weak void QDM0PTUCycleCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN QDM0_TSU_CYCLE */
    /* USER CODE END QDM0_TSU_CYCLE */
}

__weak void QDM0ZIndexLockedCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN QDM0_INDEX_LOCKED */
    /* USER CODE END QDM0_INDEX_LOCKED */
}

__weak void QDM0QuadraturePhaseErrorCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE QDM0_PHASE_ERROR */
    /* USER CODE QDM0_PHASE_ERROR */
}

static void QDM0_Init(void)
{
    HAL_CRG_IpEnableSet(QDM0_BASE, IP_CLK_ENABLE);

    g_qdm0.baseAddress = QDM0_BASE;

    /* emulation config */
    g_qdm0.emuMode = QDM_EMULATION_MODE_STOP_IMMEDIATELY;
    /* input config */
    g_qdm0.ctrlConfig.decoderMode = QDM_QUADRATURE_COUNT;
    g_qdm0.ctrlConfig.polarity = 0;
    g_qdm0.ctrlConfig.resolution = QDM_1X_RESOLUTION;
    g_qdm0.ctrlConfig.trgLockMode = QDM_TRG_BY_CYCLE;
    g_qdm0.ctrlConfig.swap = QDM_SWAP_DISABLE;
    g_qdm0.ctrlConfig.ptuMode = QDM_PTU_MODE_CYCLE;
    /* filter config */
    g_qdm0.inputFilter.qdmAFilterLevel = 0;
    g_qdm0.inputFilter.qdmBFilterLevel = 0;
    g_qdm0.inputFilter.qdmZFilterLevel = 0;
    /* other config */
    g_qdm0.lock_mode = QDM_LOCK_RESERVE;
    g_qdm0.pcntMode = QDM_PCNT_MODE_BY_DIR;
    //g_qdm0.pcntRstMode = QDM_PCNT_RST_BY_PTU;
    g_qdm0.pcntRstMode = QDM_PCNT_RST_OVF; // 单独调试脉冲数
    g_qdm0.pcntIdxInitMode = QDM_IDX_INIT_DISABLE;
    g_qdm0.qcMax = 4294967295;
    g_qdm0.subModeEn = true;
    g_qdm0.tsuPrescaler = 0;
    g_qdm0.cevtPrescaler = QDM_CEVT_PRESCALER_DIVI1;
    //g_qdm0.posMax = 4294967295;
    g_qdm0.posMax = g_motorLineNum;    
    g_qdm0.posInit = 0;
    g_qdm0.period = 150000000;

    g_qdm0.motorLineNum = g_motorLineNum;  /* 设置编码器线数 */      
    //DBG_PRINTF("g_qdm0.motorLineNum:%d\r\n", g_qdm0.motorLineNum);
    g_qdm0.interruptEn = QDM_INT_WATCHDOG | 
        QDM_INT_INDEX_EVNT_LATCH | 
        QDM_INT_UNIT_TIME_OUT;

    HAL_QDM_Init(&g_qdm0);

    HAL_QDM_RegisterCallback(&g_qdm0, QDM_TSU_CYCLE, QDM0PTUCycleCallback);
    HAL_QDM_RegisterCallback(&g_qdm0, QDM_INDEX_LOCKED, QDM0ZIndexLockedCallback);
    HAL_QDM_RegisterCallback(&g_qdm0, QDM_PHASE_ERROR, QDM0QuadraturePhaseErrorCallback);
    IRQ_Register(IRQ_QDM0, HAL_QDM_IrqHandler, &g_qdm0);
    IRQ_SetPriority(IRQ_QDM0, 1);
    IRQ_EnableN(IRQ_QDM0);
}

__weak void QDM1PTUCycleCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN QDM1_TSU_CYCLE */
    /* USER CODE END QDM1_TSU_CYCLE */
}

__weak void QDM1ZIndexLockedCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN QDM1_INDEX_LOCKED */
    /* USER CODE END QDM1_INDEX_LOCKED */
}

__weak void QDM1QuadraturePhaseErrorCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE QDM1_PHASE_ERROR */
    /* USER CODE QDM1_PHASE_ERROR */
}

static void QDM1_Init(void)
{
    HAL_CRG_IpEnableSet(QDM1_BASE, IP_CLK_ENABLE);

    g_qdm1.baseAddress = QDM1_BASE;

    /* emulation config */
    g_qdm1.emuMode = QDM_EMULATION_MODE_STOP_IMMEDIATELY;
    /* input config */
    g_qdm1.ctrlConfig.decoderMode = QDM_QUADRATURE_COUNT;
    g_qdm1.ctrlConfig.polarity = 0;
    g_qdm1.ctrlConfig.resolution = QDM_1X_RESOLUTION; /* 1倍频：只统计A相上升沿，4倍频：统计A、B两项上升沿和下降沿 */
    g_qdm1.ctrlConfig.trgLockMode = QDM_TRG_BY_CYCLE;
    g_qdm1.ctrlConfig.swap = QDM_SWAP_DISABLE;
    g_qdm1.ctrlConfig.ptuMode = QDM_PTU_MODE_CYCLE;
    /* filter config */
    g_qdm1.inputFilter.qdmAFilterLevel = 0;
    g_qdm1.inputFilter.qdmBFilterLevel = 0;
    g_qdm1.inputFilter.qdmZFilterLevel = 0;
    /* other config */
    g_qdm1.lock_mode = QDM_LOCK_RESERVE;
    g_qdm1.pcntMode = QDM_PCNT_MODE_BY_DIR;
    //g_qdm1.pcntRstMode = QDM_PCNT_RST_BY_PTU;
    g_qdm1.pcntRstMode = QDM_PCNT_RST_OVF;
    g_qdm1.pcntIdxInitMode = QDM_IDX_INIT_DISABLE;
    g_qdm1.qcMax = 4294967295;
    g_qdm1.subModeEn = true;
    g_qdm1.tsuPrescaler = 0U;
    g_qdm1.cevtPrescaler = QDM_CEVT_PRESCALER_DIVI1;
    //g_qdm1.posMax = 4294967295;
    g_qdm1.posMax = g_motorLineNum;
    g_qdm1.posInit = 0;
    g_qdm1.period = 150000000;

    g_qdm1.motorLineNum = g_motorLineNum;   /* 设置编码器线数 */
    //DBG_PRINTF("g_qdm0.motorLineNum:%d\r\n", g_qdm0.motorLineNum);

    g_qdm1.interruptEn = QDM_INT_WATCHDOG | 
        QDM_INT_INDEX_EVNT_LATCH | 
        QDM_INT_UNIT_TIME_OUT;

    HAL_QDM_Init(&g_qdm1);

    HAL_QDM_RegisterCallback(&g_qdm1, QDM_TSU_CYCLE, QDM1PTUCycleCallback);
    HAL_QDM_RegisterCallback(&g_qdm1, QDM_INDEX_LOCKED, QDM1ZIndexLockedCallback);
    HAL_QDM_RegisterCallback(&g_qdm1, QDM_PHASE_ERROR, QDM1QuadraturePhaseErrorCallback);
    IRQ_Register(IRQ_QDM1, HAL_QDM_IrqHandler, &g_qdm1);
    IRQ_SetPriority(IRQ_QDM1, 1);
    IRQ_EnableN(IRQ_QDM1);
}

__weak void TIMER0_InterruptProcess(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN TIMER0_InterruptProcess */
    /* USER CODE END TIMER0_InterruptProcess */
}

static void TIMER0_Init(void)
{

    HAL_CRG_IpEnableSet(TIMER0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(TIMER0_BASE, CRG_AHB_CLK_NO_PREDV);

    unsigned int load = (HAL_CRG_GetIpFreq((void *)TIMER0) / (1u << (TIMERPRESCALER_NO_DIV * 4)) / 1000000u) * 10000;

    g_timer0.baseAddress = TIMER0;
    g_timer0.load        = load - 1; /* Set timer value immediately */
    g_timer0.bgLoad      = load - 1; /* Set timer value */
    g_timer0.mode        = TIMER_MODE_RUN_PERIODIC; /* Run in period mode */
    g_timer0.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer0.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer0.interruptEn = BASE_CFG_ENABLE;
    g_timer0.adcSocReqEnable = BASE_CFG_DISABLE;
    g_timer0.dmaReqEnable = BASE_CFG_DISABLE;
    HAL_TIMER_Init(&g_timer0);
    IRQ_Register(IRQ_TIMER0, HAL_TIMER_IrqHandler, &g_timer0);

    HAL_TIMER_RegisterCallback(&g_timer0, TIMER_PERIOD_FIN, TIMER0_InterruptProcess);
    IRQ_SetPriority(IRQ_TIMER0, 1);
    IRQ_EnableN(IRQ_TIMER0);
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART0_BASE, CRG_AHB_CLK_NO_PREDV);

    g_uart0.baseAddress = UART0;

    g_uart0.baudRate = UART0_BAND_RATE;
    g_uart0.dataLength = UART_DATALENGTH_8BIT;
    g_uart0.stopBits = UART_STOPBITS_ONE;
    g_uart0.parity = UART_PARITY_NONE;
    g_uart0.txMode = UART_MODE_BLOCKING;
    g_uart0.rxMode = UART_MODE_BLOCKING;
    g_uart0.fifoMode = BASE_CFG_ENABLE;
    g_uart0.fifoTxThr = UART_FIFODEPTH_SIZE8;
    g_uart0.fifoRxThr = UART_FIFODEPTH_SIZE8;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    g_uart0.handleEx.overSampleMultiple = UART_OVERSAMPLING_16X;
    g_uart0.handleEx.msbFirst = BASE_CFG_DISABLE;   
    HAL_UART_Init(&g_uart0);
}

__weak void UART2InterruptErrorCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART2_TRNS_IT_ERROR */
    /* USER CODE END UART2_TRNS_IT_ERROR */
}

__weak void UART2WriteInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART2_WRITE_IT_FINISH */
    /* USER CODE END UART2_WRITE_IT_FINISH */
}

__weak void UART2ReadInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART2_READ_IT_FINISH */
    /* USER CODE END UART2_READ_IT_FINISH */
}

static void UART2_Init(void)
{
    HAL_CRG_IpEnableSet(UART2_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART2_BASE, CRG_AHB_CLK_NO_PREDV);

    g_uart2.baseAddress = UART2;

    g_uart2.baudRate = UART2_BAND_RATE;
    g_uart2.dataLength = UART_DATALENGTH_8BIT;
    g_uart2.stopBits = UART_STOPBITS_ONE;
    g_uart2.parity = UART_PARITY_NONE;
    g_uart2.txMode = UART_MODE_INTERRUPT;
    g_uart2.rxMode = UART_MODE_INTERRUPT;
    g_uart2.fifoMode = BASE_CFG_ENABLE;
    g_uart2.fifoTxThr = UART_FIFODEPTH_SIZE4;
    g_uart2.fifoRxThr = UART_FIFODEPTH_SIZE4;
    g_uart2.hwFlowCtr = BASE_CFG_DISABLE;
    g_uart2.handleEx.overSampleMultiple = UART_OVERSAMPLING_16X;
    g_uart2.handleEx.msbFirst = BASE_CFG_DISABLE;   
    HAL_UART_Init(&g_uart2);
    HAL_UART_RegisterCallBack(&g_uart2, UART_TRNS_IT_ERROR, (UART_CallbackType)UART2InterruptErrorCallback);
    HAL_UART_RegisterCallBack(&g_uart2, UART_WRITE_IT_FINISH, (UART_CallbackType)UART2WriteInterruptCallback);
    HAL_UART_RegisterCallBack(&g_uart2, UART_READ_IT_FINISH, (UART_CallbackType)UART2ReadInterruptCallback);

    IRQ_Register(IRQ_UART2, HAL_UART_IrqHandler, &g_uart2);
    IRQ_SetPriority(IRQ_UART2, 1);
    IRQ_EnableN(IRQ_UART2);
}

__weak void UART3InterruptErrorCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART3InterruptErrorCallback */
    /* USER CODE END UART3InterruptErrorCallback */
}

__weak void UART3WriteInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART3WriteInterruptCallback */
    /* USER CODE END UART3WriteInterruptCallback */
}

__weak void UART3ReadInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART3ReadInterruptCallback */
    /* USER CODE END UART3ReadInterruptCallback */
}

static void UART3_Init(void)
{
    HAL_CRG_IpEnableSet(UART3_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART3_BASE, CRG_AHB_CLK_NO_PREDV);

    g_uart3.baseAddress = UART3;

    g_uart3.baudRate = UART3_BAND_RATE;
    g_uart3.dataLength = UART_DATALENGTH_8BIT;
    g_uart3.stopBits = UART_STOPBITS_ONE;
    g_uart3.parity = UART_PARITY_NONE;
    g_uart3.txMode = UART_MODE_INTERRUPT;
    g_uart3.rxMode = UART_MODE_INTERRUPT;
    g_uart3.fifoMode = BASE_CFG_ENABLE;
    g_uart3.fifoTxThr = UART_FIFODEPTH_SIZE4;
    g_uart3.fifoRxThr = UART_FIFODEPTH_SIZE4;
    g_uart3.hwFlowCtr = BASE_CFG_DISABLE;
    g_uart3.handleEx.overSampleMultiple = UART_OVERSAMPLING_16X;
    g_uart3.handleEx.msbFirst = BASE_CFG_DISABLE;   
    HAL_UART_Init(&g_uart3);
    HAL_UART_RegisterCallBack(&g_uart3, UART_TRNS_IT_ERROR, (UART_CallbackType)UART3InterruptErrorCallback);
    HAL_UART_RegisterCallBack(&g_uart3, UART_WRITE_IT_FINISH, (UART_CallbackType)UART3WriteInterruptCallback);
    HAL_UART_RegisterCallBack(&g_uart3, UART_READ_IT_FINISH, (UART_CallbackType)UART3ReadInterruptCallback);

    IRQ_Register(IRQ_UART3, HAL_UART_IrqHandler, &g_uart3);
    IRQ_SetPriority(IRQ_UART3, 1);
    IRQ_EnableN(IRQ_UART3);
}

static void IOConfig(void)
{
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

    HAL_IOCMG_SetPinAltFuncMode(GPIO2_0_AS_QDM1_A);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_0_AS_QDM1_A, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_0_AS_QDM1_A, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_0_AS_QDM1_A, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_0_AS_QDM1_A, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO2_1_AS_QDM1_B);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_1_AS_QDM1_B, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_1_AS_QDM1_B, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_1_AS_QDM1_B, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_1_AS_QDM1_B, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO5_0_AS_QDM1_INDEX);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO5_0_AS_QDM1_INDEX, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO5_0_AS_QDM1_INDEX, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO5_0_AS_QDM1_INDEX, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO5_0_AS_QDM1_INDEX, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO1_7_AS_I2C1_SCL);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO1_7_AS_I2C1_SCL, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO1_7_AS_I2C1_SCL, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO1_7_AS_I2C1_SCL, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO1_7_AS_I2C1_SCL, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO4_7_AS_I2C1_SDA);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO4_7_AS_I2C1_SDA, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO4_7_AS_I2C1_SDA, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO4_7_AS_I2C1_SDA, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO4_7_AS_I2C1_SDA, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO1_5_AS_UART2_TXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO1_5_AS_UART2_TXD, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO1_5_AS_UART2_TXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO1_5_AS_UART2_TXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO1_5_AS_UART2_TXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO1_6_AS_UART2_RXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO1_6_AS_UART2_RXD, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO1_6_AS_UART2_RXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO1_6_AS_UART2_RXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO1_6_AS_UART2_RXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO5_2_AS_GPT2_PWM);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO5_2_AS_GPT2_PWM, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO5_2_AS_GPT2_PWM, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO5_2_AS_GPT2_PWM, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO5_2_AS_GPT2_PWM, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO0_5_AS_GPT1_PWM);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO0_5_AS_GPT1_PWM, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO0_5_AS_GPT1_PWM, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO0_5_AS_GPT1_PWM, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO0_5_AS_GPT1_PWM, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO2_4_AS_GPT0_PWM);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_4_AS_GPT0_PWM, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_4_AS_GPT0_PWM, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_4_AS_GPT0_PWM, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_4_AS_GPT0_PWM, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO1_0_AS_GPIO1_0);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO1_0_AS_GPIO1_0, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO1_0_AS_GPIO1_0, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO1_0_AS_GPIO1_0, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO1_0_AS_GPIO1_0, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO1_1_AS_GPIO1_1);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO1_1_AS_GPIO1_1, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO1_1_AS_GPIO1_1, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO1_1_AS_GPIO1_1, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO1_1_AS_GPIO1_1, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO2_5_AS_GPIO2_5);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_5_AS_GPIO2_5, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_5_AS_GPIO2_5, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_5_AS_GPIO2_5, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_5_AS_GPIO2_5, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO2_6_AS_GPIO2_6);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_6_AS_GPIO2_6, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_6_AS_GPIO2_6, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_6_AS_GPIO2_6, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_6_AS_GPIO2_6, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO2_7_AS_GPIO2_7);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_7_AS_GPIO2_7, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_7_AS_GPIO2_7, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_7_AS_GPIO2_7, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_7_AS_GPIO2_7, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO5_1_AS_GPIO5_1);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO5_1_AS_GPIO5_1, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO5_1_AS_GPIO5_1, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO5_1_AS_GPIO5_1, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO5_1_AS_GPIO5_1, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO1_4_AS_UART3_RXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO1_4_AS_UART3_RXD, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO1_4_AS_UART3_RXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO1_4_AS_UART3_RXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO1_4_AS_UART3_RXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO1_3_AS_UART3_TXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO1_3_AS_UART3_TXD, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO1_3_AS_UART3_TXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO1_3_AS_UART3_TXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO1_3_AS_UART3_TXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO4_2_AS_I2C0_SCL);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO4_2_AS_I2C0_SCL, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO4_2_AS_I2C0_SCL, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO4_2_AS_I2C0_SCL, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO4_2_AS_I2C0_SCL, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO4_3_AS_I2C0_SDA);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO4_3_AS_I2C0_SDA, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO4_3_AS_I2C0_SDA, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO4_3_AS_I2C0_SDA, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO4_3_AS_I2C0_SDA, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO3_7_AS_GPT3_PWM);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO3_7_AS_GPT3_PWM, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO3_7_AS_GPT3_PWM, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO3_7_AS_GPT3_PWM, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO3_7_AS_GPT3_PWM, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO5_3_AS_ADC_AIN12);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO5_3_AS_ADC_AIN12, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO5_3_AS_ADC_AIN12, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO5_3_AS_ADC_AIN12, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO5_3_AS_ADC_AIN12, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO3_6_AS_CAN_RX);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO3_6_AS_CAN_RX, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO3_6_AS_CAN_RX, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO3_6_AS_CAN_RX, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO3_6_AS_CAN_RX, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO3_5_AS_CAN_TX);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO3_5_AS_CAN_TX, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO3_5_AS_CAN_TX, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO3_5_AS_CAN_TX, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO3_5_AS_CAN_TX, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO4_5_AS_QDM0_A);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO4_5_AS_QDM0_A, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO4_5_AS_QDM0_A, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO4_5_AS_QDM0_A, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO4_5_AS_QDM0_A, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO4_6_AS_QDM0_B);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO4_6_AS_QDM0_B, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO4_6_AS_QDM0_B, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO4_6_AS_QDM0_B, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO4_6_AS_QDM0_B, DRIVER_RATE_2);  /* Output signal edge fast/slow */

}

void SystemInit(void)
{
    IOConfig();
    UART0_Init();
    UART2_Init();
    UART3_Init();
    APT0_Init();
    APT1_Init();
    ADC0_Init();
    CAN_Init();
    GPT0_Init();
    GPT1_Init();
    GPT2_Init();
    GPT3_Init();
    TIMER0_Init();
    I2C0_Init();
    I2C1_Init();
    QDM0_Init();
    QDM1_Init();
    GPIO_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}