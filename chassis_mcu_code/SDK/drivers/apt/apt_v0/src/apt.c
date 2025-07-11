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
  * @file    apt.c
  * @author  MCU Driver Team
  * @brief   APT module driver.
  * @details This file provides firmware functions to manage the following functionalities of APT module.
  *          + Initialization and de-initialization functions
  *          + APT module synchronization functions.
  *          + PWM waveform configuration and ADC trigger time configuration functions.
  *          + Interrupt callback function and user registration function
  */

#include "apt.h"
#include "crg.h"
#define MAX_DUTY 100
#define ALL_EVT_INT_FLAGS 0xf770000U
#define RERF   4
/**
  * @brief The parameters of PWM waveform.
  */
typedef struct {
    APT_PWMAction leftEdgeActA;     /**< Action on the left edge of PWM channel A. */
    APT_PWMAction rightEdgeActA;    /**< Action on the right edge of PWM channel A. */
    APT_PWMAction leftEdgeActB;     /**< Action on the left edge of PWM channel B. */
    APT_PWMAction rightEdgeActB;    /**< Action on the right edge of PWM channel B. */
    APT_REDInput redInput;          /**< Input source of Dead-Band rising edge delay counter. */
    APT_REDOutMode redOutMode;      /**< Output mode of Dead-Band rising edge delay counter. */
    APT_FEDInput fedInput;          /**< Input source of Dead-Band falling edge delay counter. */
    APT_FEDOutMode fedOutMode;      /**< Output mode of Dead-Band falling edge delay counter. */
} APT_WaveformPara;

/**
  * @brief Initialize the time-base counter of APT module.
  * @param aptHandle APT module handle.
  * @retval None.
  */
static void APT_TimeBaseInit(APT_Handle *aptHandle)
{
    aptHandle->baseAddress->TC_MODE.BIT.rg_cnt_mode = aptHandle->waveform.cntMode;
    aptHandle->baseAddress->TC_MODE.BIT.rg_div_fac = aptHandle->waveform.dividerFactor;
    /* Disable buffer mode of TC_PRD */
    aptHandle->baseAddress->TC_BUF_EN.reg &= (~(0b11 << 0));
    aptHandle->baseAddress->TC_PRD.BIT.rg_cnt_prd = aptHandle->waveform.timerPeriod;
    /* Set the override value of divier and timebase counter */
    aptHandle->baseAddress->TC_OVRID.BIT.rg_div_ovrid = aptHandle->waveform.divInitVal;
    aptHandle->baseAddress->TC_OVRID.BIT.rg_cnt_ovrid = aptHandle->waveform.cntInitVal;
    aptHandle->baseAddress->TC_OVRID.BIT.rg_cnt_ovrid_en = 1;
}

/**
  * @brief Initialize the count compare points for PWM waveform generation.
  * @param aptHandle APT module handle.
  * @retval None.
  */
static void APT_SetPWMCompareVal(APT_Handle *aptHandle)
{
    /* Do not use divider compare value for PWM wareform generation  */
    aptHandle->baseAddress->TC_REFC.BIT.rg_cnt_refcl = aptHandle->waveform.dividerFactor;
    aptHandle->baseAddress->TC_REFD.BIT.rg_cnt_refdl = aptHandle->waveform.dividerFactor;
    /* Configure the compare point along the left and right edges of PWM waveform */
    TC_REFC_REG tmpC;
    TC_REFD_REG tmpD;
    /* Set the value of the active register of CMPC and CMPD */
    tmpC = aptHandle->baseAddress->TC_REFC;
    tmpC.BIT.rg_cnt_refch = aptHandle->waveform.cntCmpLeftEdge;
    aptHandle->baseAddress->TC_REFC = tmpC;
    tmpD = aptHandle->baseAddress->TC_REFD;
    tmpD.BIT.rg_cnt_refdh = aptHandle->waveform.cntCmpRightEdge;
    aptHandle->baseAddress->TC_REFD = tmpD;
    /* Set the buffer load mode of CMPC and CMPD */
    if (aptHandle->waveform.cntCmpLoadMode == APT_BUFFER_DISABLE) {
        aptHandle->baseAddress->TC_BUF_EN.BIT.rg_refc_buf_en = 0;
        aptHandle->baseAddress->TC_BUF_EN.BIT.rg_refd_buf_en = 0;
    } else {
        aptHandle->baseAddress->TC_BUF_EN.BIT.rg_refc_buf_en = 1;
        aptHandle->baseAddress->TC_BUF_EN.BIT.rg_refd_buf_en = 1;
        unsigned int gldLdEn = (aptHandle->waveform.cntCmpLoadMode == APT_BUFFER_GLOBAL_LOAD) ? 1 : 0;
        aptHandle->baseAddress->TC_BUF_EN.BIT.rg_refc_gld_en = gldLdEn;
        aptHandle->baseAddress->TC_BUF_EN.BIT.rg_refd_gld_en = gldLdEn;
        /* Set buffer load event */
        unsigned int refBufField = 8;
        aptHandle->baseAddress->TC_REF_LOAD.reg &= (~(0x1F << (APT_COMPARE_REFERENCE_C * refBufField)));
        aptHandle->baseAddress->TC_REF_LOAD.reg &= (~(0x1F << (APT_COMPARE_REFERENCE_D * refBufField)));
        aptHandle->baseAddress->TC_REF_LOAD.reg |=
            (aptHandle->waveform.cntCmpLoadEvt << (APT_COMPARE_REFERENCE_C * refBufField));
        aptHandle->baseAddress->TC_REF_LOAD.reg |=
            (aptHandle->waveform.cntCmpLoadEvt << (APT_COMPARE_REFERENCE_D * refBufField));
        /* Set the value of the buffer register of CMPC and CMPD */
        tmpC = aptHandle->baseAddress->TC_REFC;
        tmpC.BIT.rg_cnt_refch = aptHandle->waveform.cntCmpLeftEdge;
        aptHandle->baseAddress->TC_REFC = tmpC;
        tmpD = aptHandle->baseAddress->TC_REFD;
        tmpD.BIT.rg_cnt_refdh = aptHandle->waveform.cntCmpRightEdge;
        aptHandle->baseAddress->TC_REFD = tmpD;
    }
}

/**
  * @brief Configure the basic PWM A waveform output according to the waveform parameters.
  * @param aptHandle APT module handle.
  * @param wavePara PWM waveform parameter.
  * @retval None.
  */
static void APT_SetOutputABasicType(APT_Handle *aptHandle, const APT_WaveformPara *wavePara)
{
    switch (aptHandle->waveform.cntMode) {
        case APT_COUNT_MODE_UP:
            aptHandle->baseAddress->PG_ACT_A.BIT.rg_pga_act_refc_inc = wavePara->leftEdgeActA;
            aptHandle->baseAddress->PG_ACT_A.BIT.rg_pga_act_refd_inc = wavePara->rightEdgeActA;
            break;
        case APT_COUNT_MODE_DOWN:
            aptHandle->baseAddress->PG_ACT_A.BIT.rg_pga_act_refc_dec = wavePara->rightEdgeActA;
            aptHandle->baseAddress->PG_ACT_A.BIT.rg_pga_act_refd_dec = wavePara->leftEdgeActA;
            break;
        case APT_COUNT_MODE_UP_DOWN:
            aptHandle->baseAddress->PG_ACT_A.BIT.rg_pga_act_refc_inc = wavePara->leftEdgeActA;
            aptHandle->baseAddress->PG_ACT_A.BIT.rg_pga_act_refd_dec = wavePara->rightEdgeActA;
            break;
        default:
            break;
    }
    return;
}

/**
  * @brief Configure the basic PWM B waveform output according to the waveform parameters.
  * @param aptHandle APT module handle.
  * @param wavePara PWM waveform parameter.
  * @retval None.
  */
static void APT_SetOutputBBasicType(APT_Handle *aptHandle, const APT_WaveformPara *wavePara)
{
    switch (aptHandle->waveform.cntMode) {
        case APT_COUNT_MODE_UP:
            aptHandle->baseAddress->PG_ACT_B.BIT.rg_pgb_act_refc_inc = wavePara->leftEdgeActB;
            aptHandle->baseAddress->PG_ACT_B.BIT.rg_pgb_act_refd_inc = wavePara->rightEdgeActB;
            break;
        case APT_COUNT_MODE_DOWN:
            aptHandle->baseAddress->PG_ACT_B.BIT.rg_pgb_act_refc_dec = wavePara->rightEdgeActB;
            aptHandle->baseAddress->PG_ACT_B.BIT.rg_pgb_act_refd_dec = wavePara->leftEdgeActB;
            break;
        case APT_COUNT_MODE_UP_DOWN:
            aptHandle->baseAddress->PG_ACT_B.BIT.rg_pgb_act_refc_inc = wavePara->leftEdgeActB;
            aptHandle->baseAddress->PG_ACT_B.BIT.rg_pgb_act_refd_dec = wavePara->rightEdgeActB;
            break;
        default:
            break;
    }
    return;
}

/**
  * @brief Configure the basic PWM waveform output according to the waveform parameters.
  * @param aptHandle APT module handle.
  * @param wavePara PWM waveform parameter.
  * @retval None.
  */
static void APT_SetPWMBasicType(APT_Handle *aptHandle, const APT_WaveformPara *wavePara)
{
    /* Configure PWM waveform of PWM channel A */
    if (aptHandle->waveform.chAOutType == APT_PWM_OUT_BASIC_TYPE) {
        APT_SetOutputABasicType(aptHandle, wavePara);
    }
    /* Configure PWM waveform of PWM channel B */
    if (aptHandle->waveform.chBOutType == APT_PWM_OUT_BASIC_TYPE) {
        APT_SetOutputBBasicType(aptHandle, wavePara);
    }
    /* Configure dead band of PWM channel A and channel B */
    if (aptHandle->waveform.chAOutType == APT_PWM_OUT_BASIC_TYPE &&
        aptHandle->waveform.chBOutType == APT_PWM_OUT_BASIC_TYPE) {
        aptHandle->baseAddress->DG_CFG.BIT.rg_dg_red_isel = wavePara->redInput;
        aptHandle->baseAddress->DG_CFG.BIT.rg_dg_red_osel = wavePara->redOutMode;
        aptHandle->baseAddress->DG_RED.BIT.rg_dg_red = aptHandle->waveform.deadBandCnt;
        aptHandle->baseAddress->DG_CFG.BIT.rg_dg_fed_isel = wavePara->fedInput;
        aptHandle->baseAddress->DG_CFG.BIT.rg_dg_fed_osel = wavePara->fedOutMode;
        aptHandle->baseAddress->DG_FED.BIT.rg_dg_fed = aptHandle->waveform.deadBandCnt;
    }
}

/**
  * @brief Set the actual outputs of PWM channelA and channelB when basic PWM waveform type is not used.
  * @param aptHandle APT module handle.
  * @retval None.
  */
static void APT_SetContWaveform(APT_Handle *aptHandle)
{
    if (aptHandle->waveform.chAOutType != APT_PWM_OUT_BASIC_TYPE) {
        unsigned int contActA = (aptHandle->waveform.chAOutType == APT_PWM_OUT_ALWAYS_LOW) ? 0b01 : 0b10;
        aptHandle->baseAddress->PG_ACT_A.BIT.rg_pga_act_zro = contActA;
    }
    if (aptHandle->waveform.chBOutType != APT_PWM_OUT_BASIC_TYPE) {
        unsigned int contActB = (aptHandle->waveform.chBOutType == APT_PWM_OUT_ALWAYS_LOW) ? 0b01 : 0b10;
        aptHandle->baseAddress->PG_ACT_B.BIT.rg_pgb_act_zro = contActB;
    }
}

/**
  * @brief Initialize the PWM waveform parameters according to the selected PWM basic type.
  * @param aptHandle APT module handle.
  * @retval None.
  */
static void APT_SetPWMWaveform(APT_Handle *aptHandle)
{
    APT_SetContWaveform(aptHandle);
    /* Configure the basic type of PWM waveform */
    APT_WaveformPara wavePara = {0, 0, 0, 0, 0, 0, 0, 0};
    switch (aptHandle->waveform.basicType) {
        case APT_PWM_BASIC_A_HIGH_B_LOW:
            wavePara.leftEdgeActA = APT_PWM_ACTION_HIGH;
            wavePara.rightEdgeActA = APT_PWM_ACTION_LOW;
            wavePara.leftEdgeActB = APT_PWM_ACTION_HIGH;
            wavePara.rightEdgeActB = APT_PWM_ACTION_LOW;
            wavePara.redInput = APT_DB_RED_INPUT_PWM_A;
            wavePara.redOutMode = APT_DB_RED_OUTPUT_NOT_INVERT;
            wavePara.fedInput = APT_DB_FED_INPUT_PWM_B;
            wavePara.fedOutMode = APT_DB_FED_OUTPUT_INVERT;
            break;
        case APT_PWM_BASIC_A_LOW_B_HIGH:
            wavePara.leftEdgeActA = APT_PWM_ACTION_LOW;
            wavePara.rightEdgeActA = APT_PWM_ACTION_HIGH;
            wavePara.leftEdgeActB = APT_PWM_ACTION_LOW;
            wavePara.rightEdgeActB = APT_PWM_ACTION_HIGH;
            wavePara.fedInput = APT_DB_FED_INPUT_PWM_A;
            wavePara.fedOutMode = APT_DB_FED_OUTPUT_INVERT;
            wavePara.redInput = APT_DB_RED_INPUT_PWM_B;
            wavePara.redOutMode = APT_DB_RED_OUTPUT_NOT_INVERT;
            break;
        case APT_PWM_BASIC_A_HIGH_B_HIGH:
            wavePara.leftEdgeActA = APT_PWM_ACTION_HIGH;
            wavePara.rightEdgeActA = APT_PWM_ACTION_LOW;
            wavePara.leftEdgeActB = APT_PWM_ACTION_HIGH;
            wavePara.rightEdgeActB = APT_PWM_ACTION_LOW;
            wavePara.redInput = APT_DB_RED_INPUT_PWM_A;
            wavePara.redOutMode = APT_DB_RED_OUTPUT_NOT_INVERT;
            wavePara.fedInput = APT_DB_FED_INPUT_PWM_B;
            wavePara.fedOutMode = APT_DB_FED_OUTPUT_NOT_INVERT;
            break;
        case APT_PWM_BASIC_A_LOW_B_LOW:
            wavePara.leftEdgeActA = APT_PWM_ACTION_LOW;
            wavePara.rightEdgeActA = APT_PWM_ACTION_HIGH;
            wavePara.leftEdgeActB = APT_PWM_ACTION_LOW;
            wavePara.rightEdgeActB = APT_PWM_ACTION_HIGH;
            wavePara.fedInput = APT_DB_FED_INPUT_PWM_A;
            wavePara.fedOutMode = APT_DB_FED_OUTPUT_NOT_INVERT;
            wavePara.redInput = APT_DB_RED_INPUT_PWM_B;
            wavePara.redOutMode = APT_DB_RED_OUTPUT_NOT_INVERT;
            break;
        default:
            break;
    }
    APT_SetPWMBasicType(aptHandle, &wavePara);
}

/**
  * @brief Initialize the count compare points for triggering ADC sampling.
  * @param aptHandle APT module handle.
  * @retval None.
  */
static void APT_SetADCTrgCompareVal(APT_Handle *aptHandle)
{
    /* Do not use divider compare value for ADC trigger source SOCA and SOCB  */
    aptHandle->baseAddress->TC_REFA.BIT.rg_cnt_refal = aptHandle->waveform.dividerFactor;
    aptHandle->baseAddress->TC_REFB.BIT.rg_cnt_refbl = aptHandle->waveform.dividerFactor;
    /* Configure the count compare point for triggering SOCA and SOCB */
    TC_REFA_REG tmpA;
    TC_REFB_REG tmpB;
    /* Set the value of active register for CMPA and CMPB */
    tmpA = aptHandle->baseAddress->TC_REFA;
    tmpA.BIT.rg_cnt_refah = aptHandle->adcTrg.cntCmpSOCA;
    aptHandle->baseAddress->TC_REFA = tmpA;
    tmpB = aptHandle->baseAddress->TC_REFB;
    tmpB.BIT.rg_cnt_refbh = aptHandle->adcTrg.cntCmpSOCB;
    aptHandle->baseAddress->TC_REFB = tmpB;
    /* Set the buffer load mode of CMPA and CMPB */
    if (aptHandle->adcTrg.cntCmpLoadMode == APT_BUFFER_DISABLE) {
        aptHandle->baseAddress->TC_BUF_EN.BIT.rg_refa_buf_en = 0;
        aptHandle->baseAddress->TC_BUF_EN.BIT.rg_refb_buf_en = 0;
    } else {
        aptHandle->baseAddress->TC_BUF_EN.BIT.rg_refa_buf_en = 1;
        aptHandle->baseAddress->TC_BUF_EN.BIT.rg_refb_buf_en = 1;
        unsigned int gldLdEn = (aptHandle->adcTrg.cntCmpLoadMode == APT_BUFFER_GLOBAL_LOAD) ? 1 : 0;
        aptHandle->baseAddress->TC_BUF_EN.BIT.rg_refa_gld_en = gldLdEn;
        aptHandle->baseAddress->TC_BUF_EN.BIT.rg_refb_gld_en = gldLdEn;
        /* Set buffer load event */
        unsigned int refBufField = 8;
        aptHandle->baseAddress->TC_REF_LOAD.reg &= (~(0x1F << (APT_COMPARE_REFERENCE_A * refBufField)));
        aptHandle->baseAddress->TC_REF_LOAD.reg &= (~(0x1F << (APT_COMPARE_REFERENCE_B * refBufField)));
        aptHandle->baseAddress->TC_REF_LOAD.reg |=
            (aptHandle->adcTrg.cntCmpLoadEvt << (APT_COMPARE_REFERENCE_A * refBufField));
        aptHandle->baseAddress->TC_REF_LOAD.reg |=
            (aptHandle->adcTrg.cntCmpLoadEvt << (APT_COMPARE_REFERENCE_B * refBufField));
        /* Set the value of buffer register for CMPA and CMPB */
        tmpA = aptHandle->baseAddress->TC_REFA;
        tmpA.BIT.rg_cnt_refah = aptHandle->adcTrg.cntCmpSOCA;
        aptHandle->baseAddress->TC_REFA = tmpA;
        tmpB = aptHandle->baseAddress->TC_REFB;
        tmpB.BIT.rg_cnt_refbh = aptHandle->adcTrg.cntCmpSOCB;
        aptHandle->baseAddress->TC_REFB = tmpB;
    }
}

/**
  * @brief Initialize the ADC trigger function of APT module.
  * @param aptHandle APT module handle
  * @retval None.
  */
static void APT_SetADCTrigger(APT_Handle *aptHandle)
{
    APT_PARAM_CHECK_NO_RET(aptHandle->adcTrg.trgScaleSOCA <= ADC_CONVERSION_START_CNT_MAX);
    APT_PARAM_CHECK_NO_RET(aptHandle->adcTrg.trgScaleSOCB <= ADC_CONVERSION_START_CNT_MAX);
    /* Configure ADC trigger source SOCA */
    aptHandle->baseAddress->CS_TMR_SELA.BIT.rg_csa_tmr_sel = aptHandle->adcTrg.trgSrcSOCA;
    aptHandle->baseAddress->CS_PRSCA_CFG.BIT.rg_csa_prsc_prd = aptHandle->adcTrg.trgScaleSOCA;
    aptHandle->baseAddress->CS_TMR_SELA.BIT.rg_csa_en_cs = aptHandle->adcTrg.trgEnSOCA;
    /* Configure ADC trigger source SOCB */
    aptHandle->baseAddress->CS_TMR_SELB.BIT.rg_csb_tmr_sel = aptHandle->adcTrg.trgSrcSOCB;
    aptHandle->baseAddress->CS_PRSCB_CFG.BIT.rg_csb_prsc_prd = aptHandle->adcTrg.trgScaleSOCB;
    aptHandle->baseAddress->CS_TMR_SELB.BIT.rg_csb_en_cs = aptHandle->adcTrg.trgEnSOCB;
}

/**
  * @brief Initialize the timer interrupt of APT module.
  * @param aptHandle APT module handle.
  * @retval None.
  */
static void APT_SetTimerInterrupt(APT_Handle *aptHandle)
{
    APT_PARAM_CHECK_NO_RET(aptHandle->tmrInterrupt.tmrInterruptScale <= TIMER_INTERRUPT_CNT_MAX);
    aptHandle->baseAddress->INT_TMR_SEL.BIT.rg_int_tmr_sel = aptHandle->tmrInterrupt.tmrInterruptSrc;
    aptHandle->baseAddress->INT_PRSC_CFG.BIT.rg_int_prsc_prd = aptHandle->tmrInterrupt.tmrInterruptScale;
    aptHandle->baseAddress->INT_TMR_EN.BIT.rg_int_en_tmr = aptHandle->tmrInterrupt.tmrInterruptEn;
}

/**
  * @brief Initialize the APT hardware configuration based on the APT module handle.
  * @param aptHandle APT module handle.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_APT_PWMInit(APT_Handle *aptHandle)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(IsAPTInstance(aptHandle->baseAddress));
    APT_PARAM_CHECK_WITH_RET(aptHandle->waveform.dividerFactor <= DIVIDER_FACTOR_MAX, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(aptHandle->waveform.divInitVal <= aptHandle->waveform.dividerFactor, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(aptHandle->waveform.cntInitVal < aptHandle->waveform.timerPeriod, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(aptHandle->waveform.cntCmpLeftEdge > 0, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(aptHandle->waveform.cntCmpLeftEdge < aptHandle->waveform.timerPeriod, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(aptHandle->waveform.cntCmpRightEdge > 0, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(aptHandle->waveform.cntCmpRightEdge < aptHandle->waveform.timerPeriod, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(aptHandle->adcTrg.cntCmpSOCA >= 0, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(aptHandle->adcTrg.cntCmpSOCA < aptHandle->waveform.timerPeriod, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(aptHandle->adcTrg.cntCmpSOCB >= 0, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(aptHandle->adcTrg.cntCmpSOCB < aptHandle->waveform.timerPeriod, BASE_STATUS_ERROR);
    APT_TimeBaseInit(aptHandle);
    APT_SetPWMCompareVal(aptHandle);
    APT_SetPWMWaveform(aptHandle);
    APT_SetADCTrgCompareVal(aptHandle);
    APT_SetADCTrigger(aptHandle);
    APT_SetTimerInterrupt(aptHandle);
    return BASE_STATUS_OK;
}

/**
  * @brief Deinitialize the APT hardware configuration.
  * @param aptHandle APT module handle.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_APT_PWMDeInit(APT_Handle *aptHandle)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    aptHandle->userCallBack.EvtInterruptCallBack = NULL;  /* The callback function is left blank. */
    aptHandle->userCallBack.TmrInterruptCallBack = NULL;
    aptHandle->baseAddress->INT_TMR_EN.BIT.rg_int_en_tmr = BASE_CFG_UNSET; /* Interrupt reset. */
    aptHandle->baseAddress->CS_TMR_SELA.BIT.rg_csa_en_cs = BASE_CFG_UNSET;
    aptHandle->baseAddress->CS_TMR_SELB.BIT.rg_csb_en_cs = BASE_CFG_UNSET;
    aptHandle->baseAddress->TC_BUF_EN.reg = 0x0;  /* APT Buff register reset. */
    aptHandle->baseAddress->TC_REFA.reg = 0x0;    /* APT reference dots reset. */
    aptHandle->baseAddress->TC_REFB.reg = 0x0;
    aptHandle->baseAddress->TC_REFC.reg = 0x0;
    aptHandle->baseAddress->TC_REFD.reg = 0x0;
    aptHandle->baseAddress->TC_PRD.BIT.rg_cnt_prd = 0x2710; /* 0x2710: default value */
    aptHandle->baseAddress->PG_ACT_A.reg = 0x0;     /* Clear action register. */
    aptHandle->baseAddress->PG_ACT_B.reg = 0x0;
    
    return BASE_STATUS_OK;
}

/**
  * @brief Configure output control protection mode.
  * @param aptHandle APT module handle.
  * @param protect Output control protection event handle.
  * @retval None.
  */
static void APT_SetOutCtrlProtectMode(APT_Handle *aptHandle, APT_OutCtrlProtect *protect)
{
    /* Set output control protect mode */
    unsigned int ocModeOffset = 16;
    unsigned int cbcClrOffsetPrd = 16;
    if (protect->ocEventMode == APT_OUT_CTRL_ONE_SHOT) {
        aptHandle->baseAddress->OC_MODE.reg &= (~(protect->ocEvent << ocModeOffset));
    } else if (protect->ocEventMode == APT_OUT_CTRL_CYCLE_BY_CYBLE) {
        aptHandle->baseAddress->OC_MODE.reg |= (protect->ocEvent << ocModeOffset);
        if ((protect->cbcClrMode & APT_CLEAR_CBC_ON_CNTR_ZERO) ==APT_CLEAR_CBC_ON_CNTR_ZERO) {
            aptHandle->baseAddress->OC_PRD_CLR.reg |= protect->ocEvent;
        }
        if ((protect->cbcClrMode & APT_CLEAR_CBC_ON_CNTR_PERIOD) == APT_CLEAR_CBC_ON_CNTR_PERIOD) {
            aptHandle->baseAddress->OC_PRD_CLR.reg |= (protect->ocEvent << cbcClrOffsetPrd);
        }
    }
}

/**
  * @brief Output control protection action selection.
  * @param aptHandle APT module handle.
  * @param ocAction Out control action.
  * @param protect Output control protection event handle.
  * @param outCtrlEvent output settings.
  * @retval None.
  */
static void APT_SetOutCtrlAction(APT_Handle *aptHandle, APT_OutCtrlAction ocAction, APT_OutCtrlEventDir outCtrlEvent)
{
    /* Set output control action when counting up */
    aptHandle->baseAddress->OC_ACT_A.reg &= (~(0b111 << outCtrlEvent));
    aptHandle->baseAddress->OC_ACT_A.reg |= (ocAction << outCtrlEvent);
    aptHandle->baseAddress->OC_ACT_B.reg &= (~(0b111 << outCtrlEvent));
    aptHandle->baseAddress->OC_ACT_B.reg |= (ocAction << outCtrlEvent);
}

/**
  * @brief Change APT's OC Event to EM Event.
  * @param ocEvent OC Event.
  * @param emEvent EM Event.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
static BASE_StatusType APT_ChangeOcEventToEmEvent(APT_OutCtrlEvent ocEvent, APT_EMIOSysEvent *emEvent)
{
    APT_ASSERT_PARAM(emEvent != NULL);
    switch (ocEvent) {
        case APT_OC_GPIO_EVENT_1:
            *emEvent = APT_EM_GPIO_EVENT_1;
            break;
        case APT_OC_GPIO_EVENT_2:
            *emEvent = APT_EM_GPIO_EVENT_2;
            break;
        case APT_OC_GPIO_EVENT_3:
            *emEvent = APT_EM_GPIO_EVENT_3;
            break;
        case APT_OC_SYSTEM_EVENT_1:
            *emEvent = APT_EM_SYSTEM_EVENT_1;
            break;
        case APT_OC_SYSTEM_EVENT_2:
            *emEvent = APT_EM_SYSTEM_EVENT_2;
            break;
        case APT_OC_SYSTEM_EVENT_3:
            *emEvent = APT_EM_SYSTEM_EVENT_3;
            break;
        default:
            return BASE_STATUS_ERROR;
    }
    
    return BASE_STATUS_OK;
}

/**
  * @brief Set combine event out control action.
  * @param aptHandle APT module handle.
  * @param protect Output control protection event handle.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
static BASE_StatusType APT_SetCombieEvtOutCtrl(APT_Handle *aptHandle, APT_OutCtrlProtect *protect)
{
    switch (protect->ocEvent) {
        case APT_OC_COMBINE_EVENT_A1:
            APT_SetOutCtrlAction(aptHandle, protect->ocAction, APT_OC_EVT_COMBINE_EVENT_A1_UP);
            APT_SetOutCtrlAction(aptHandle, protect->ocAction, APT_OC_EVT_COMBINE_EVENT_A1_DOWN);
            break;
        case APT_OC_COMBINE_EVENT_A2:
            APT_SetOutCtrlAction(aptHandle, protect->ocAction, APT_OC_EVT_COMBINE_EVENT_A2_UP);
            APT_SetOutCtrlAction(aptHandle, protect->ocAction, APT_OC_EVT_COMBINE_EVENT_A2_DOWN);
            break;
        case APT_OC_COMBINE_EVENT_B1:
            APT_SetOutCtrlAction(aptHandle, protect->ocAction, APT_OC_EVT_COMBINE_EVENT_B1_UP);
            APT_SetOutCtrlAction(aptHandle, protect->ocAction, APT_OC_EVT_COMBINE_EVENT_B1_DOWN);
            break;
        case APT_OC_COMBINE_EVENT_B2:
            APT_SetOutCtrlAction(aptHandle, protect->ocAction, APT_OC_EVT_COMBINE_EVENT_B2_UP);
            APT_SetOutCtrlAction(aptHandle, protect->ocAction, APT_OC_EVT_COMBINE_EVENT_B2_DOWN);
            break;
        default:
            return BASE_STATUS_ERROR;
    }

    return BASE_STATUS_OK;
}

/**
  * @brief Setting emulation mode of APT module.
  * @param aptHandle APT module handle.
  * @param protect Output control protection event handle.
  * @retval None.
  */
static void APT_OcSetEmulation(APT_Handle *aptHandle, APT_OutCtrlProtect *protect)
{
    aptHandle->baseAddress->TC_MODE.BIT.rg_emu_stop = protect->emMode;
    if (protect->emMode > APT_EMULATION_NO_STOP) {
        aptHandle->baseAddress->OC_MODE.reg |= APT_OC_SYSTEM_EVENT_1;
    }
}

/**
  * @brief Initialize the output control protection event of APT module.
  * @param aptHandle APT module handle.
  * @param protect Output control protection event handle.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_APT_ProtectInit(APT_Handle *aptHandle, APT_OutCtrlProtect *protect)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(protect != NULL);
    APT_ASSERT_PARAM(IsAPTInstance(aptHandle->baseAddress));
    APT_PARAM_CHECK_WITH_RET(protect->ocEvent >= APT_OC_GPIO_EVENT_1, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(protect->ocEvent <= APT_OC_COMBINE_EVENT_B2, BASE_STATUS_ERROR);
    APT_SetOutCtrlProtectMode(aptHandle, protect);
    /* Emultion settings */
    APT_OcSetEmulation(aptHandle, protect);

    if ((protect->ocEvent >= APT_OC_COMBINE_EVENT_A1) && (protect->ocEvent <= APT_OC_COMBINE_EVENT_B2)) {
        if (APT_SetCombieEvtOutCtrl(aptHandle, protect) == BASE_STATUS_ERROR) {
            return BASE_STATUS_ERROR;
        }
    } else {
        /* Set IO event polarity */
        APT_EMIOSysEvent ioSysEvt;
        if (APT_ChangeOcEventToEmEvent(protect->ocEvent, &ioSysEvt) == BASE_STATUS_OK) {
            aptHandle->baseAddress->EM_EVTIO_PSEL.reg &= (~(0b11 << ioSysEvt));
            aptHandle->baseAddress->EM_EVTIO_PSEL.reg |= (protect->evtPolarity << ioSysEvt);
        }
        /* Set output control action when counting up */
        APT_SetOutCtrlAction(aptHandle, protect->ocAction, APT_OC_EVT_GPIO_OR_SYSTEM_UP);
        /* Set output control action when counting down */
        APT_SetOutCtrlAction(aptHandle, protect->ocAction, APT_OC_EVT_GPIO_OR_SYSTEM_DOWN);
    }
    if (protect->ocEventEn == BASE_CFG_ENABLE) {
        aptHandle->baseAddress->OC_MODE.reg |= (protect->ocEvent);
    } else {
        aptHandle->baseAddress->OC_MODE.reg &= (~(protect->ocEvent));
    }
    if (protect->ocEvtInterruptEn == BASE_CFG_ENABLE) {
        aptHandle->baseAddress->INT_EVT_EN.reg |= (protect->ocEvent);
    } else {
        aptHandle->baseAddress->INT_EVT_EN.reg &= (~(protect->ocEvent));
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Setting protect source event filter, only support the same filter value.
  * @param filterNum filter cycle number.
  * @retval None.
  */
static void APT_SetEMEventFilterEx(unsigned char filterNum)
{
    unsigned int srcEvent;
    unsigned int enableOffset = 24;
    unsigned int valueShift = 8;
    unsigned int maxEventNum = 3; /* every register can config 3 event's filer */
    for (srcEvent = 0; srcEvent < maxEventNum; srcEvent++) {
        SYSCTRL1->APT_POE_FILTER.reg |= 0x1 << (enableOffset + srcEvent);
        SYSCTRL1->APT_POE_FILTER.reg |= (((unsigned int)filterNum & 0xff) << (valueShift * srcEvent));
        SYSCTRL1->APT_EVTMP_FILTER.reg |=  0x1 << (enableOffset + srcEvent);
        SYSCTRL1->APT_EVTMP_FILTER.reg |= (((unsigned int)filterNum & 0xff) << (valueShift * srcEvent));
    }
}

/**
  * @brief Set protect source event polarity.
  * @param aptHandle APT module handle.
  * @param polarityMask polarity bit mask.
  * @retval None.
  */
static void APT_SetProtectSrcEventPolarityEx(APT_Handle *aptHandle, unsigned polarityMask)
{
    unsigned int curEvent;
    unsigned int curPolarity;
    unsigned int curMpEventNum;
    unsigned int curIoEventNum;

    for (int i = 0; i <= APT_EM_COMBINE_SRC_EVT_MP_6; i++) {
        curEvent = i;
        curPolarity = (polarityMask >> curEvent) & 0x01;
        if (curEvent >= APT_EM_COMBINE_SRC_EVT_MP_1) {
            curMpEventNum = (curEvent - APT_EM_COMBINE_SRC_EVT_MP_1) << 1;
            /* set ACMP0~2 and EVTMP4~6 event polarity */
            aptHandle->baseAddress->EM_EVTMP_PSEL.reg &= (~(0b11 << curMpEventNum));
            aptHandle->baseAddress->EM_EVTMP_PSEL.reg |= (curPolarity << curMpEventNum);
        } else {
            /* set IO event polarity */
            curIoEventNum = curEvent << 1;
            aptHandle->baseAddress->EM_EVTIO_PSEL.reg &= (~(0b11 << curIoEventNum));
            aptHandle->baseAddress->EM_EVTIO_PSEL.reg |= (curPolarity << curIoEventNum);
        }
    }
}

/**
  * @brief Configure output control protection mode.
  * @param aptHandle APT module handle.
  * @param protect Output control protection event handle.
  * @retval None.
  */
static void APT_SetSysEventProtectModeEx(APT_Handle *aptHandle, APT_OutCtrlProtectEx *protect)
{
    /* Set output control protect mode */
    unsigned int ocModeOffset = 16;
    unsigned int cbcClrOffsetPrd = 16;
    if (protect->ocEventModeEx == APT_OUT_CTRL_ONE_SHOT) {
        aptHandle->baseAddress->OC_MODE.reg &= (~(protect->ocSysEvent << ocModeOffset));
    } else if (protect->ocEventModeEx == APT_OUT_CTRL_CYCLE_BY_CYBLE) {
        aptHandle->baseAddress->OC_MODE.reg |= (protect->ocSysEvent << ocModeOffset);
        if ((protect->cbcClrModeEx & APT_CLEAR_CBC_ON_CNTR_ZERO) ==APT_CLEAR_CBC_ON_CNTR_ZERO) {
            aptHandle->baseAddress->OC_PRD_CLR.reg |= protect->ocSysEvent;
        }
        if ((protect->cbcClrModeEx & APT_CLEAR_CBC_ON_CNTR_PERIOD) == APT_CLEAR_CBC_ON_CNTR_PERIOD) {
            aptHandle->baseAddress->OC_PRD_CLR.reg |= (protect->ocSysEvent << cbcClrOffsetPrd);
        }
    }
}

/**
  * @brief System event protect initialize.
  * @param aptHandle APT module handle.
  * @param protect Output control protection event data.
  * @retval None.
  */
static void APT_SysProtectInitEx(APT_Handle *aptHandle, APT_OutCtrlProtectEx *protect)
{
    APT_SetSysEventProtectModeEx(aptHandle, protect);
    if (protect->ocEventEnEx == BASE_CFG_ENABLE) {
        aptHandle->baseAddress->OC_MODE.reg |= protect->ocSysEvent;
    } else {
        aptHandle->baseAddress->OC_MODE.reg &= (~(protect->ocSysEvent));
    }
    if (protect->ocEvtInterruptEnEx == BASE_CFG_ENABLE) {
        aptHandle->baseAddress->INT_EVT_EN.reg |= protect->ocSysEvent;
    } else {
        aptHandle->baseAddress->INT_EVT_EN.reg &= (~(protect->ocSysEvent));
    }
}

/**
  * @brief Initialize the output control protection event of APT module (Extended interface).
  * @param aptHandle APT module handle.
  * @param protect Output control protection event data.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_APT_ProtectInitEx(APT_Handle *aptHandle, APT_OutCtrlProtectEx *protect)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(protect != NULL);
    APT_ASSERT_PARAM(IsAPTInstance(aptHandle->baseAddress));
    APT_PARAM_CHECK_WITH_RET(protect->originalEvtEx >= 0, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(protect->originalEvtEx <= 0x1FF, BASE_STATUS_ERROR); /* 0x1FF : all event enable */
    unsigned int cbcClrOffsetPrd = 16;
    aptHandle->baseAddress->OC_MODE.reg = 0x0; /* clear OC_MODE resgiter */
    aptHandle->baseAddress->TC_MODE.BIT.rg_emu_stop = 0x0; /* don't stop APT when emulation */
    aptHandle->baseAddress->OC_PRD_CLR.reg = 0x0; /* clear OC_PRD_CLR register */
    APT_SysProtectInitEx(aptHandle, protect);
    /* event management configuration */
    aptHandle->baseAddress->EM_MRG_SEL.BIT.rg_em_a1_sel = EM_COMBINE_A1_SRC_ENABLE_ALL; /* open logic OR */
    aptHandle->baseAddress->EM_AOR_EN.BIT.rg_em_a1_oren = protect->originalEvtEx; /* open selected event */
    APT_SetProtectSrcEventPolarityEx(aptHandle, protect->evtPolarityMaskEx);
    APT_SetEMEventFilterEx(protect->filterCycleNumEx);
    aptHandle->baseAddress->EM_MRG_SEL.BIT.rg_evta1t_sel= APT_EM_COMBINE_EVT1; /* all event input to combine event A1 */
    /* out control configuration */
    APT_SetOutCtrlAction(aptHandle, protect->ocActionEx, APT_OC_EVT_COMBINE_EVENT_A1_UP);
    APT_SetOutCtrlAction(aptHandle, protect->ocActionEx, APT_OC_EVT_COMBINE_EVENT_A1_DOWN);
    aptHandle->baseAddress->OC_MODE.BIT.rg_oc_mode_evta1 = protect->ocEventModeEx; /* set protect mode */
    if ((protect->cbcClrModeEx & APT_CLEAR_CBC_ON_CNTR_ZERO) ==APT_CLEAR_CBC_ON_CNTR_ZERO) {
        aptHandle->baseAddress->OC_PRD_CLR.reg |= APT_OC_COMBINE_EVENT_A1; /* set CBC clear mode */
    }
    if ((protect->cbcClrModeEx & APT_CLEAR_CBC_ON_CNTR_PERIOD) == APT_CLEAR_CBC_ON_CNTR_PERIOD) {
        aptHandle->baseAddress->OC_PRD_CLR.reg |= (APT_OC_COMBINE_EVENT_A1 << cbcClrOffsetPrd);
    }
    if (protect->ocEventEnEx == BASE_CFG_ENABLE) {
        aptHandle->baseAddress->OC_MODE.reg |= APT_OC_COMBINE_EVENT_A1; /* OC input combine event A1 */
    } else {
        aptHandle->baseAddress->OC_MODE.reg &= (~(APT_OC_COMBINE_EVENT_A1));
    }
    if (protect->ocEvtInterruptEnEx == BASE_CFG_ENABLE) {
        aptHandle->baseAddress->INT_EVT_EN.reg |= (APT_OC_COMBINE_EVENT_A1);
    } else {
        aptHandle->baseAddress->INT_EVT_EN.reg &= (~(APT_OC_COMBINE_EVENT_A1)); /* enable combine event A1 interrupt */
    }
    return BASE_STATUS_OK;
}

/**
  * @brief De-initialize the output control protection event of APT module (Extended interface).
  * @param aptHandle APT module handle.
  * @param protect Output control protection event handle.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_APT_ProtectDeInitEx(APT_Handle *aptHandle, APT_OutCtrlProtectEx *protect)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(protect != NULL);
    protect->ocEventEnEx = BASE_CFG_DISABLE;
    aptHandle->baseAddress->OC_MODE.reg = 0x700070; /* 0x7000070: default value */

    return BASE_STATUS_OK;
}

/**
  * @brief De-initialize the output control protection event of APT module.
  * @param aptHandle APT module handle.
  * @param protect Output control protection event handle.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_APT_ProtectDeInit(APT_Handle *aptHandle, APT_OutCtrlProtect *protect)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(protect != NULL);
    protect->ocEventEn = BASE_CFG_DISABLE;
    aptHandle->baseAddress->OC_MODE.reg = 0x700070; /* 0x7000070: default value */

    return BASE_STATUS_OK;
}

/**
  * @brief Set event management's source events polarity.
  * @param aptHandle APT module handle.
  * @param emEvtSrc Source event selection.
  * @param emEvtPolar Event polarity.
  * @retval None.
  */
static void APT_SetEMInputEvtPolarity(APT_Handle *aptHandle, APT_EMCombineEvtSrc emEvtSrc,
                                      APT_EMEventPolarity emEvtPolar)
{
    unsigned int eventPolarity;
    if (emEvtSrc >= APT_EM_COMBINE_SRC_EVT_MP_1) {
        /* set multiplex event polarity */
        eventPolarity = (emEvtSrc - APT_EM_COMBINE_SRC_EVT_MP_1) << 1;
        aptHandle->baseAddress->EM_EVTMP_PSEL.reg &= (~(0b11 << eventPolarity));
        aptHandle->baseAddress->EM_EVTMP_PSEL.reg |= (emEvtPolar << eventPolarity);
    } else {
        /* set io event polarity */
        eventPolarity = (emEvtSrc) << 1;
        aptHandle->baseAddress->EM_EVTIO_PSEL.reg &= (~(0b11 << eventPolarity));
        aptHandle->baseAddress->EM_EVTIO_PSEL.reg |= (emEvtPolar << eventPolarity);
    }
}


/**
  * @brief Set event management's source events input and event combine.
  * (if enable logic or function, it do not support setting polarity, need use DCL to set polarity.)
  * @param aptHandle APT module handle.
  * @param emEvent EM event handle.
  * @retval None.
  */
static void APT_EMCombineEventInit(APT_Handle *aptHandle, APT_CombineEvt *emEvent)
{
    unsigned int evtNum;
    for (evtNum = 0; evtNum < EM_CMB_EVT_NUM; evtNum++) {
        /* if select logical or */
        aptHandle->baseAddress->EM_MRG_SEL.reg |= emEvent[evtNum].emEvtSrc << (evtNum * EM_CMB_SRC_SEL_INTERVAL);
        if (emEvent[evtNum].emEvtSrc == APT_EM_COMBINE_SRC_ALL_EVENT_OR) {
            /* enable logical or events */
            if (evtNum < APT_EM_COMBINE_EVENT_B1) {
                aptHandle->baseAddress->EM_AOR_EN.reg |= (emEvent[evtNum].emEvtOrEnBits << (evtNum * EM_OR_INTERVAL));
            } else {
                aptHandle->baseAddress->EM_BOR_EN.reg |= (emEvent[evtNum].emEvtOrEnBits << \
                ((evtNum - APT_EM_COMBINE_EVENT_B1) * EM_OR_INTERVAL));
            }
        } else {
            /* set input event's polarity */
            APT_SetEMInputEvtPolarity(aptHandle, emEvent[evtNum].emEvtSrc, emEvent[evtNum].emEvtPolar);
        }
        aptHandle->baseAddress->EM_MRG_SEL.reg |= (emEvent[evtNum].emEvtCombineMode << \
        (evtNum * EM_CMB_MODE_INTERVAL)) << EM_CMB_MODE_OFFSET;
    }
}

/**
  * @brief Initialize mask window and capture function of event management.
  * @param aptHandle APT module handle.
  * @param emWdAndCp Mask window and capture configuration handle.
  * @retval None.
  */
static void APT_EMWdAndCapInit(APT_Handle *aptHandle, APT_WdAndCap *emWdAndCap)
{
    if (emWdAndCap->wdEnable == true) {
        /* filter source select */
        aptHandle->baseAddress->EM_OUT_SEL.BIT.rg_evtfilt_sel = emWdAndCap->eventSel;
        /* set window offset */
        aptHandle->baseAddress->EM_WD_CNT.BIT.rg_mskwd_offset = emWdAndCap->wdOffset;
        /* set window width */
        aptHandle->baseAddress->EM_WD_CNT.BIT.rg_mskwd_width = emWdAndCap->wdWidth;
        /* window polarit select */
        aptHandle->baseAddress->EM_WD_EN.BIT.rg_mskwd_psel = emWdAndCap->wdPolar;
        /* capture clear mode */
        aptHandle->baseAddress->EM_WD_EN.BIT.rg_mskwd_alg_zroen = emWdAndCap->wdStartAndCapClr & 0x1;
        aptHandle->baseAddress->EM_WD_EN.BIT.rg_mskwd_alg_prden = (emWdAndCap->wdStartAndCapClr >> 0x1) & 0x1;
        /* enable capture function */
        if (emWdAndCap->emCapEnable == true) {
            aptHandle->baseAddress->EM_TCAP_CFG.BIT.rg_tcap_en = BASE_CFG_SET;
        }
    }
}

/**
  * @brief Initialize vally switch function of event management.
  * @param aptHandle APT module handle.
  * @param emVallySw Valley switch configuration handle.
  * @retval None.
  */
static void APT_EMVallySwInit(APT_Handle *aptHandle, APT_ValleySw *emValleySw)
{
    if (emValleySw->vsEnable == true) {
        /* filter edge */
        aptHandle->baseAddress->EM_FILT_CFG.BIT.rg_filt_edg_sel = emValleySw->vsFilerEdgeSel;
        /* filter count */
        aptHandle->baseAddress->EM_FILT_CFG.BIT.rg_filt_edg_cnt = emValleySw->vsFilterCnt;
        /* clear setting */
        aptHandle->baseAddress->EM_VCAP_CFG.BIT.rg_vcap_trig_sel = emValleySw->vsClrType;
        /* capture edge */
        aptHandle->baseAddress->EM_VCAP_CFG.BIT.rg_vcap_edg_sel = emValleySw->vsCapEdgeSel;
        /* capture start edge */
        aptHandle->baseAddress->EM_VCAP_CFG.BIT.rg_vcap_sta_edg = emValleySw->vsCapStartEdge;
        /* capture end edge */
        aptHandle->baseAddress->EM_VCAP_CFG.BIT.rg_vcap_sta_edg = emValleySw->vsCapStartEdge;
        /* capture delay mode */
        aptHandle->baseAddress->EM_VCAP_DLY.BIT.rg_vcap_dly_mode = emValleySw->vsCapDelayMode;
        /* Calibrate delay */
        aptHandle->baseAddress->EM_VCAP_DLY.BIT.rg_vcap_swdly = emValleySw->vsCapSoftDelay;
    }
}

/**
  * @brief Event management initialization interface.
  * @param aptHandle APT module handle.
  * @param eventManage Event management handle.
  * @retval None.
  */
BASE_StatusType HAL_APT_EMInit(APT_Handle *aptHandle, APT_EventManage *eventManage)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(eventManage != NULL);
    if (eventManage->emEnable == true) { /* event manage enable */
        APT_EMCombineEventInit(aptHandle, eventManage->emEvt); /* init combine event */
        if ((eventManage->emValleySw.vsEnable == true) || (eventManage->emWdAndCap.wdEnable == true)) {
            aptHandle->baseAddress->EM_OUT_SEL.reg |= EM_OUT_EVT_FILTER_EN;
            APT_EMWdAndCapInit(aptHandle, &(eventManage->emWdAndCap)); /* init window and capture function */
            APT_EMVallySwInit(aptHandle, &(eventManage->emValleySw)); /* init valley switch */
        }
        return BASE_STATUS_OK;
    }
    return BASE_STATUS_ERROR;
}


/**
  * @brief Get capture value of Event management.
  * @param aptHandle APT module handle.
  * @retval None.
  */
unsigned short HAL_APT_EMGetCapValue(APT_Handle *aptHandle)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(IsAPTInstance(aptHandle->baseAddress));
    unsigned short capValue = 0;
    /* if capture valid */
    if (aptHandle->baseAddress->EM_TCAP_CFG.BIT.rg_tcap_en == BASE_CFG_ENABLE) {
        /* read capture value */
        capValue = aptHandle->baseAddress->EM_TCAP_VAL.BIT.ro_tcap_cnt_rt;
    }
    
    return capValue;
}

/**
  * @brief Set window's offset and width of Event management.
  * @param aptHandle APT module handle.
  * @param offset Window's offset.
  * @param width Window's width.
  * @retval None.
  */
void HAL_APT_EMSetWdOffsetAndWidth(APT_Handle *aptHandle, unsigned short offset, unsigned short width)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(IsAPTInstance(aptHandle->baseAddress));
    aptHandle->baseAddress->EM_WD_CNT.BIT.rg_mskwd_offset = offset;
    aptHandle->baseAddress->EM_WD_CNT.BIT.rg_mskwd_width = width;
}

/**
  * @brief Set vallet switch's software calibrate of Event management.
  * @param aptHandle APT module handle.
  * @retval None.
  */
void HAL_APT_EMSetValleySwithSoftDelay(APT_Handle *aptHandle, unsigned short calibrate)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(IsAPTInstance(aptHandle->baseAddress));
    aptHandle->baseAddress->EM_VCAP_DLY.BIT.rg_vcap_swdly = calibrate;
}

/**
  * @brief Disable PWMA and PWMB output. PWMA and PWMB output low level.
  * @param aptHandle APT module handle.
  * @retval None.
  */
void HAL_APT_ForcePWMOutputLow(APT_Handle *aptHandle)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(IsAPTInstance(aptHandle->baseAddress));
    DCL_APT_ForcePWMOutputLow(aptHandle->baseAddress);
    
    return;
}

/**
  * @brief Initialize the master APT module when using multiple sync-out mode.
  * @param aptHandle APT module handle.
  * @param masterSyncOut Master APT module synchronization handle.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_APT_MasterSyncInit(APT_Handle *aptHandle, unsigned short syncOutSrc)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(IsAPTInstance(aptHandle->baseAddress));
    APT_PARAM_CHECK_WITH_RET(syncOutSrc <= SYNC_OUT_SOURCE_MAX, BASE_STATUS_ERROR);
    /* Configure the sync-out pulse source of APT module synchronization */
    aptHandle->baseAddress->SYNO_CFG.reg &= (~(0xFF << 0));
    aptHandle->baseAddress->SYNO_CFG.reg |= (syncOutSrc << 0);
    aptHandle->baseAddress->SYNO_CFG.BIT.rg_mode_syno = APT_SYNCOUT_MULTIPLE_MODE;
    return BASE_STATUS_OK;
}

/**
  * @brief Initialize the slave APT module.
  * @param aptHandle APT module handle.
  * @param slaveSyncIn Slave APT module synchronization handle.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_APT_SlaveSyncInit(APT_Handle *aptHandle, APT_SlaveSyncIn *slaveSyncIn)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(slaveSyncIn != NULL);
    APT_ASSERT_PARAM(IsAPTInstance(aptHandle->baseAddress));
    APT_PARAM_CHECK_WITH_RET(slaveSyncIn->divPhase <= aptHandle->waveform.dividerFactor, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(slaveSyncIn->cntPhase < aptHandle->waveform.timerPeriod, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(slaveSyncIn->cntrSyncSrc <= CNTR_SYNC_SOURCE_MAX, BASE_STATUS_ERROR);

    aptHandle->baseAddress->TC_PHS.BIT.rg_cnt_dir = slaveSyncIn->syncCntMode;
    aptHandle->baseAddress->TC_PHS.BIT.rg_div_phs = slaveSyncIn->divPhase;
    TC_PHS_REG tmp = aptHandle->baseAddress->TC_PHS;
    tmp.BIT.rg_cnt_phs = slaveSyncIn->cntPhase;
    aptHandle->baseAddress->TC_PHS = tmp;

    aptHandle->baseAddress->SYNI_CFG.BIT.rg_syni_sel = slaveSyncIn->syncInSrc;
    aptHandle->baseAddress->SYNCNT_CFG.reg = slaveSyncIn->cntrSyncSrc;
    return BASE_STATUS_OK;
}

/**
  * @brief Start all of the used APT modules simultaneously.
  * @param aptRunMask A logical OR of valid values that can be passed as the aptRunMask.
  *        Valid values for aptRunMask are:
  *            RUN_APT0 - apt0_run bit in SYSCTRL1 register.
  *            RUN_APT1 - apt1_run bit in SYSCTRL1 register.
  *            RUN_APT2 - apt2_run bit in SYSCTRL1 register.
  *            RUN_APT3 - apt3_run bit in SYSCTRL1 register.
  *            RUN_APT4 - apt4_run bit in SYSCTRL1 register.
  *            RUN_APT5 - apt5_run bit in SYSCTRL1 register.
  *            RUN_APT6 - apt6_run bit in SYSCTRL1 register.
  *            RUN_APT7 - apt7_run bit in SYSCTRL1 register.
  *            RUN_APT8 - apt8_run bit in SYSCTRL1 register.
  * @retval None.
  */
void HAL_APT_StartModule(unsigned int aptRunMask)
{
    SYSCTRL1->APT_RUN.reg |= aptRunMask;
}

/**
  * @brief Stop all of the used APT modules simultaneously.
  * @param aptRunMask A logical OR of valid values that can be passed as the aptRunMask.
  *        Valid values for aptRunMask are:
  *            RUN_APT0 - apt0_run bit in SYSCTRL1 register.
  *            RUN_APT1 - apt1_run bit in SYSCTRL1 register.
  *            RUN_APT2 - apt2_run bit in SYSCTRL1 register.
  *            RUN_APT3 - apt3_run bit in SYSCTRL1 register.
  *            RUN_APT4 - apt4_run bit in SYSCTRL1 register.
  *            RUN_APT5 - apt5_run bit in SYSCTRL1 register.
  *            RUN_APT6 - apt6_run bit in SYSCTRL1 register.
  *            RUN_APT7 - apt7_run bit in SYSCTRL1 register.
  *            RUN_APT8 - apt8_run bit in SYSCTRL1 register.
  * @retval None.
  */
void HAL_APT_StopModule(unsigned int aptRunMask)
{
    SYSCTRL1->APT_RUN.reg &= (~aptRunMask);
}

/**
  * @brief Set the count compare points along the left and right edges of PWM waveform.
  * @param aptHandle APT module handle.
  * @param cntCmpLeftEdge The count compare point of the left edge of PWM waveform. Pull High on left edge.
  * @param cntCmpRightEdge The count compare point of the right edge of PWM waveform. Pull Low on right edge.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_APT_SetPWMDuty(APT_Handle *aptHandle, unsigned short cntCmpLeftEdge, \
                                   unsigned short cntCmpRightEdge)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(IsAPTInstance(aptHandle->baseAddress));
    APT_PARAM_CHECK_WITH_RET(cntCmpLeftEdge > 0, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(cntCmpLeftEdge < aptHandle->waveform.timerPeriod, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(cntCmpRightEdge > 0, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(cntCmpRightEdge < aptHandle->waveform.timerPeriod, BASE_STATUS_ERROR);
    TC_REFC_REG tmpC;
    TC_REFD_REG tmpD;
    tmpC = aptHandle->baseAddress->TC_REFC;
    tmpC.BIT.rg_cnt_refch = cntCmpLeftEdge;
    aptHandle->baseAddress->TC_REFC = tmpC;
    tmpD = aptHandle->baseAddress->TC_REFD;
    tmpD.BIT.rg_cnt_refdh = cntCmpRightEdge;
    aptHandle->baseAddress->TC_REFD = tmpD;
    return BASE_STATUS_OK;
}

/**
  * @brief Set the count compare points along the left and right edges of PWM waveform.
  * @param aptHandle APT module handle.
  * @param duty PWM duty. Range: 1 ~ 99.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_APT_SetPWMDutyByNumber(APT_Handle *aptHandle, unsigned int duty)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(IsAPTInstance(aptHandle->baseAddress));
    APT_PARAM_CHECK_WITH_RET(duty < MAX_DUTY, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(duty > 0, BASE_STATUS_ERROR);

    unsigned int cntCmpLeftEdge, cntCmpRightEdge;
    TC_REFC_REG tmpC;
    TC_REFD_REG tmpD;

    if (aptHandle->waveform.cntMode == APT_COUNT_MODE_UP_DOWN) {
        cntCmpLeftEdge = aptHandle->waveform.timerPeriod - \
                         (int)(((float)aptHandle->waveform.timerPeriod / MAX_DUTY) * duty);
        cntCmpRightEdge = cntCmpLeftEdge;
    } else {
        cntCmpLeftEdge = 1;
        cntCmpRightEdge = (int)(((float)aptHandle->waveform.timerPeriod / MAX_DUTY) * duty + cntCmpLeftEdge);
    }
    tmpC = aptHandle->baseAddress->TC_REFC;
    tmpC.BIT.rg_cnt_refch = cntCmpLeftEdge;
    aptHandle->baseAddress->TC_REFC = tmpC;
    tmpD = aptHandle->baseAddress->TC_REFD;
    tmpD.BIT.rg_cnt_refdh = cntCmpRightEdge;
    aptHandle->baseAddress->TC_REFD = tmpD;
    return BASE_STATUS_OK;
}

/**
  * @brief Set the count compare points to trigger the ADC sampling.
  * @param aptHandle APT module handle.
  * @param cntCmpSOCA The count compare point for triggering SOCA.
  * @param cntCmpSOCB The count compare point for triggering SOCB.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_APT_SetADCTriggerTime(APT_Handle *aptHandle, unsigned short cntCmpSOCA, unsigned short cntCmpSOCB)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(IsAPTInstance(aptHandle->baseAddress));
    APT_PARAM_CHECK_WITH_RET(cntCmpSOCA > 0, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(cntCmpSOCA < aptHandle->waveform.timerPeriod, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(cntCmpSOCB > 0, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(cntCmpSOCB < aptHandle->waveform.timerPeriod, BASE_STATUS_ERROR);
    TC_REFA_REG tmpA;
    TC_REFB_REG tmpB;
    tmpA = aptHandle->baseAddress->TC_REFA;
    tmpA.BIT.rg_cnt_refah = cntCmpSOCA;
    aptHandle->baseAddress->TC_REFA = tmpA;
    tmpB = aptHandle->baseAddress->TC_REFB;
    tmpB.BIT.rg_cnt_refbh = cntCmpSOCB;
    aptHandle->baseAddress->TC_REFB = tmpB;
    return BASE_STATUS_OK;
}

/**
  * @brief set outputs of channelA  when use APT_PWM_BASIC_A_HIGH_B_HIGH.
  * @param aptHandle APT module handle.
  * @param aptAction output action type.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
static BASE_StatusType APT_SetActionChannelA(APT_Handle *aptHandle, APT_PWMChannelOutType aptAction)
{
    switch (aptAction) {
        case APT_PWM_OUT_BASIC_TYPE:
            aptHandle->baseAddress->PG_OUT_FRC.BIT.rg_pga_frc_en = BASE_CFG_UNSET; /* disable force action */
            break;
        case APT_PWM_OUT_ALWAYS_LOW:
            aptHandle->baseAddress->PG_OUT_FRC.BIT.rg_pga_frc_act = APT_PWM_OUT_ALWAYS_LOW; /* force output low */
            aptHandle->baseAddress->PG_OUT_FRC.BIT.rg_pga_frc_en = BASE_CFG_SET;
            break;
        case APT_PWM_OUT_ALWAYS_HIGH:
            aptHandle->baseAddress->PG_OUT_FRC.BIT.rg_pga_frc_act = APT_PWM_OUT_ALWAYS_HIGH; /* force output high */
            aptHandle->baseAddress->PG_OUT_FRC.BIT.rg_pga_frc_en = BASE_CFG_SET;
            break;
        default:
            return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief set outputs of channelB  when use APT_PWM_BASIC_A_HIGH_B_HIGH.
  * @param aptHandle APT module handle.
  * @param aptAction output action type.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
static BASE_StatusType APT_SetActionChannelB(APT_Handle *aptHandle, APT_PWMChannelOutType aptAction)
{
    switch (aptAction) {
        case APT_PWM_OUT_BASIC_TYPE:
            aptHandle->baseAddress->PG_OUT_FRC.BIT.rg_pgb_frc_en = BASE_CFG_UNSET; /* disable force action */
            break;
        case APT_PWM_OUT_ALWAYS_LOW:
            aptHandle->baseAddress->PG_OUT_FRC.BIT.rg_pgb_frc_act = APT_PWM_OUT_ALWAYS_LOW; /* force output low */
            aptHandle->baseAddress->PG_OUT_FRC.BIT.rg_pgb_frc_en = BASE_CFG_SET;
            break;
        case APT_PWM_OUT_ALWAYS_HIGH:
            aptHandle->baseAddress->PG_OUT_FRC.BIT.rg_pgb_frc_act = APT_PWM_OUT_ALWAYS_HIGH; /* force output high */
            aptHandle->baseAddress->PG_OUT_FRC.BIT.rg_pgb_frc_en = BASE_CFG_SET;
            break;
        default:
            return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Change outputs of channelA and channelB when use APT_PWM_BASIC_A_HIGH_B_HIGH.
  * @param aptHandle APT module handle.
  * @param channel channel number.
  * @param aptAction output action type.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_APT_ChangeOutputType(APT_Handle *aptHandle, APT_PWMChannel channel, APT_PWMChannelOutType aptAction)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(aptHandle->baseAddress != NULL);
    APT_ASSERT_PARAM(IsAPTInstance(aptHandle->baseAddress));
    APT_PARAM_CHECK_WITH_RET(channel >= APT_PWM_CHANNEL_A, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(channel <= APT_PWM_CHANNEL_B, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(aptAction >= APT_PWM_OUT_BASIC_TYPE, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(aptAction <= APT_PWM_OUT_ALWAYS_HIGH, BASE_STATUS_ERROR);
    /* only use in APT_PWM_BASIC_A_HIGH_B_HIGH mode */
    if (aptHandle->waveform.basicType != APT_PWM_BASIC_A_HIGH_B_HIGH) {
        return BASE_STATUS_ERROR;
    }
    if (channel == APT_PWM_CHANNEL_A) {
        return APT_SetActionChannelA(aptHandle, aptAction); /* set channnelA's action */
    } else if (channel == APT_PWM_CHANNEL_B) {
        return APT_SetActionChannelB(aptHandle, aptAction); /* set channelB's action */
    } else {
        return BASE_STATUS_ERROR; /* error channnel number */
    }
}

/**
  * @brief APT event interrupt service processing function.
  * @param handle APT module handle.
  * @retval None.
  */
void HAL_APT_EventIrqHandler(void *handle)
{
    APT_ASSERT_PARAM(handle != NULL);
    APT_Handle *aptHandle = (APT_Handle *)handle;
    /* Continuous protection cannot clear the event flag. Clear the event flag by users. */
    if (aptHandle->baseAddress->OC_MODE.BIT.rg_oc_mode_evta1 == 0x1) {  /* Protection by period. */
        /* Interrupt of the periodic protection clear event. */
        aptHandle->baseAddress->OC_EVT_FLAG.reg |= ALL_EVT_INT_FLAGS;
    }
    aptHandle->baseAddress->OC_EVT_FLAG.BIT.rg_int_clr_evt = 1;  /* clear event flag */
    if (aptHandle->userCallBack.EvtInterruptCallBack != NULL) {
        aptHandle->userCallBack.EvtInterruptCallBack(aptHandle);
    }
}

/**
  * @brief APT timer interrupt service processing function.
  * @param handle APT module handle.
  * @retval None.
  */
void HAL_APT_TimerIrqHandler(void *handle)
{
    APT_ASSERT_PARAM(handle != NULL);
    APT_Handle *aptHandle = (APT_Handle *)handle;
    aptHandle->baseAddress->INT_TMR_FLAG.BIT.rg_int_clr_tmr = 1; /* clear timer interrupt flag */
    if (aptHandle->userCallBack.TmrInterruptCallBack != NULL) {
        aptHandle->userCallBack.TmrInterruptCallBack(aptHandle);
    }
}

/**
  * @brief Interrupt callback functions registration interface.
  * @param aptHandle APT module handle.
  * @param typeID ID of callback function type.
  * @param pCallback Pointer for the user callback function.
  * @retval None.
  */
void HAL_APT_RegisterCallBack(APT_Handle *aptHandle, APT_InterruputType typeID, APT_CallbackType pCallback)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    switch (typeID) {
        case APT_TIMER_INTERRUPT: /* timer interrupt */
            aptHandle->userCallBack.TmrInterruptCallBack = pCallback;
            break;
        case APT_EVENT_INTERRUPT: /* event interrupt */
            aptHandle->userCallBack.EvtInterruptCallBack = pCallback;
            break;
        default:
            break;
    }
}

/**
  * @brief Attribute configuration of the reference point.
  * @param aptHandle APT module handle.
  * @param refDotParameters Attribute structure of a reference point.
  * @retval BASE_StatusType: OK, ERROR.
  */
static BASE_StatusType APT_ConfigAction(APT_Handle *aptHandle, APT_RefDotParameters *refDotParameters)
{
    /* Action configuration of the reference point of channel B. */
    if (refDotParameters->pwmChannel == APT_PWM_CHANNEL_B) {
        aptHandle->baseAddress->PG_ACT_B.reg &= (~(0b11 << refDotParameters->actionEvent)); /* Reset configuration */
        aptHandle->baseAddress->PG_ACT_B.reg |=  (refDotParameters->action << refDotParameters->actionEvent);
        return BASE_STATUS_OK;
    }
    /* Action configuration of the reference point of channel A. */
    if (refDotParameters->pwmChannel == APT_PWM_CHANNEL_A) {
        aptHandle->baseAddress->PG_ACT_A.reg &= (~(0b11 << refDotParameters->actionEvent)); /* Reset configuration */
        aptHandle->baseAddress->PG_ACT_A.reg |=  (refDotParameters->action << refDotParameters->actionEvent);
        return BASE_STATUS_OK;
    }
    return BASE_STATUS_ERROR;
}

/**
  * @brief Configure the value and action of the reference point A.
  * @param aptHandle APT module handle.
  * @param refDotParameters Reference point A configuration property set.
  * @retval BASE_StatusType: OK, ERROR.
  */
BASE_StatusType APT_ConfigRefA(APT_Handle *aptHandle, APT_RefDotParameters *refDotParameters)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(aptHandle->baseAddress != NULL);
    APT_ASSERT_PARAM(IsAPTInstance(aptHandle->baseAddress));
    /* Check the attributes of the reference point A: PWM Channel. */
    APT_PARAM_CHECK_WITH_RET(refDotParameters->pwmChannel >= APT_PWM_CHANNEL_A, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(refDotParameters->pwmChannel <= APT_PWM_CHANNEL_B, BASE_STATUS_ERROR);
    /* Check the attributes of the reference point A: triggle action event. */
    APT_PARAM_CHECK_WITH_RET(refDotParameters->actionEvent >= APT_PWM_ACTION_ON_TIMEBASE_ZERO, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(refDotParameters->actionEvent <= APT_PWM_ACTION_ON_C2_COUNT_DOWN, BASE_STATUS_ERROR);
    /* Check the attributes of the reference point A: triggle action. */
    APT_PARAM_CHECK_WITH_RET(refDotParameters->action <= APT_PWM_ACTION_TOGGLE, BASE_STATUS_ERROR);
    /* Change reference dot A value and division value. */
    aptHandle->baseAddress->TC_REFA.BIT.rg_cnt_refah = refDotParameters->refDotValue;
    aptHandle->baseAddress->TC_REFA.BIT.rg_cnt_refal = refDotParameters->refDotDivValue;
    /* Reference dot A triggle event and action */
    return APT_ConfigAction(aptHandle, refDotParameters);
}

/**
  * @brief Configure the value and action of the reference point B.
  * @param aptHandle APT module handle.
  * @param refDotParameters Reference point B configuration property set.
  * @retval BASE_StatusType: OK, ERROR.
  */
BASE_StatusType APT_ConfigRefB(APT_Handle *aptHandle, APT_RefDotParameters *refDotParameters)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(aptHandle->baseAddress != NULL);
    APT_ASSERT_PARAM(IsAPTInstance(aptHandle->baseAddress));
    /* Check the attributes of the reference point B: PWM Channel. */
    APT_PARAM_CHECK_WITH_RET(refDotParameters->pwmChannel >= APT_PWM_CHANNEL_A, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(refDotParameters->pwmChannel <= APT_PWM_CHANNEL_B, BASE_STATUS_ERROR);
    /* Check the attributes of the reference point B: triggle action event. */
    APT_PARAM_CHECK_WITH_RET(refDotParameters->actionEvent >= APT_PWM_ACTION_ON_TIMEBASE_ZERO, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(refDotParameters->actionEvent <= APT_PWM_ACTION_ON_C2_COUNT_DOWN, BASE_STATUS_ERROR);
    /* Check the attributes of the reference point B: triggle action. */
    APT_PARAM_CHECK_WITH_RET(refDotParameters->action <= APT_PWM_ACTION_TOGGLE, BASE_STATUS_ERROR);
    /* Change reference dot B value and division value. */
    aptHandle->baseAddress->TC_REFB.BIT.rg_cnt_refbh = refDotParameters->refDotValue;
    aptHandle->baseAddress->TC_REFB.BIT.rg_cnt_refbl = refDotParameters->refDotDivValue;
    /* Reference dot B triggle event and action. */
    return APT_ConfigAction(aptHandle, refDotParameters);
}

/**
  * @brief Configure the value and action of the reference point C.
  * @param aptHandle APT module handle.
  * @param refDotParameters Reference point C configuration property set.
  * @retval BASE_StatusType: OK, ERROR.
  */
BASE_StatusType APT_ConfigRefC(APT_Handle *aptHandle, APT_RefDotParameters *refDotParameters)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(aptHandle->baseAddress != NULL);
    APT_ASSERT_PARAM(IsAPTInstance(aptHandle->baseAddress));
    /* Check the attributes of the reference point C: PWM Channel. */
    APT_PARAM_CHECK_WITH_RET(refDotParameters->pwmChannel >= APT_PWM_CHANNEL_A, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(refDotParameters->pwmChannel <= APT_PWM_CHANNEL_B, BASE_STATUS_ERROR);
    /* Check the attributes of the reference point C: triggle action event. */
    APT_PARAM_CHECK_WITH_RET(refDotParameters->actionEvent >= APT_PWM_ACTION_ON_TIMEBASE_ZERO, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(refDotParameters->actionEvent <= APT_PWM_ACTION_ON_C2_COUNT_DOWN, BASE_STATUS_ERROR);
    /* Check the attributes of the reference point C: triggle action. */
    APT_PARAM_CHECK_WITH_RET(refDotParameters->action <= APT_PWM_ACTION_TOGGLE, BASE_STATUS_ERROR);
    /* Change reference dot C value and division value. */
    aptHandle->baseAddress->TC_REFC.BIT.rg_cnt_refch = refDotParameters->refDotValue;
    aptHandle->baseAddress->TC_REFC.BIT.rg_cnt_refcl = refDotParameters->refDotDivValue;
    /* Reference dot C triggle event and action. */
    return APT_ConfigAction(aptHandle, refDotParameters);
}

/**
  * @brief Configure the value and action of the reference point D.
  * @param aptHandle APT module handle.
  * @param refDotParameters Reference point D configuration property set.
  * @retval BASE_StatusType: OK, ERROR.
  */
BASE_StatusType APT_ConfigRefD(APT_Handle *aptHandle, APT_RefDotParameters *refDotParameters)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(aptHandle->baseAddress != NULL);
    APT_ASSERT_PARAM(IsAPTInstance(aptHandle->baseAddress));
    /* Check the attributes of the reference point D: PWM Channel. */
    APT_PARAM_CHECK_WITH_RET(refDotParameters->pwmChannel >= APT_PWM_CHANNEL_A, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(refDotParameters->pwmChannel <= APT_PWM_CHANNEL_B, BASE_STATUS_ERROR);
    /* Check the attributes of the reference point D: triggle action event. */
    APT_PARAM_CHECK_WITH_RET(refDotParameters->actionEvent >= APT_PWM_ACTION_ON_TIMEBASE_ZERO, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(refDotParameters->actionEvent <= APT_PWM_ACTION_ON_C2_COUNT_DOWN, BASE_STATUS_ERROR);
    /* Check the attributes of the reference point D: triggle action. */
    APT_PARAM_CHECK_WITH_RET(refDotParameters->action <= APT_PWM_ACTION_TOGGLE, BASE_STATUS_ERROR);
    /* Change reference dot D value and division value. */
    aptHandle->baseAddress->TC_REFD.BIT.rg_cnt_refdh = refDotParameters->refDotValue;
    aptHandle->baseAddress->TC_REFD.BIT.rg_cnt_refdl = refDotParameters->refDotDivValue;
    /* Reference dot D triggle event and action. */
    return APT_ConfigAction(aptHandle, refDotParameters);
}


/**
  * @brief Attribute configuration of any reference point.
  * @param aptHandle APT module handle.
  * @param refDotSelect Selection of reference points.
  * @param refDotParameters The properties of the reference point.
  * @retval BASE_StatusType: OK, ERROR.
  */
BASE_StatusType HAL_APT_ConfigRefDot(APT_Handle *aptHandle, APT_RefDotSelect refDotSelect,
                                     APT_RefDotParameters *refDotParameters)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(aptHandle->baseAddress != NULL);
    APT_ASSERT_PARAM(IsAPTInstance(aptHandle->baseAddress));
    /* Reference point configuration, which must be point A, point B, point C, and point D. */
    APT_PARAM_CHECK_WITH_RET(refDotSelect >= APT_REFERENCE_DOTA, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(refDotSelect <= APT_REFERENCE_DOTD, BASE_STATUS_ERROR);
    /* Channels A and B are optional. */
    APT_PARAM_CHECK_WITH_RET(refDotParameters->pwmChannel >= APT_PWM_CHANNEL_A, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(refDotParameters->pwmChannel <= APT_PWM_CHANNEL_B, BASE_STATUS_ERROR);
    /* Trigger event type check. */
    APT_PARAM_CHECK_WITH_RET(refDotParameters->actionEvent >= APT_PWM_ACTION_ON_TIMEBASE_ZERO, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(refDotParameters->actionEvent <= APT_PWM_ACTION_ON_C2_COUNT_DOWN, BASE_STATUS_ERROR);
    /* There are four types of trigger actions. */
    APT_PARAM_CHECK_WITH_RET(refDotParameters->action <= APT_PWM_ACTION_TOGGLE, BASE_STATUS_ERROR);
    /* Transfer table for setting reference dot. */
    BASE_StatusType (* APT_RefDotConfigTable[RERF])(APT_Handle *, APT_RefDotParameters *) = {APT_ConfigRefA,
                                                                                             APT_ConfigRefB,
                                                                                             APT_ConfigRefC,
                                                                                             APT_ConfigRefD};
    return APT_RefDotConfigTable[refDotSelect](aptHandle, refDotParameters); /* Configure reference point. */
}