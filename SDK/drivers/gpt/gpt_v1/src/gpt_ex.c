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
  * @file      gpt_ex.c
  * @author    MCU Driver Team
  * @brief     GPT module driver.
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the GPT.
  *                + Initialization function of GPT
  *                + Clock Configuration of GPT
  *                + Get GPT State and Apply GPT
  */

#include "gpt_ex.h"

/**
 * @brief   Get GPT Counter Value
 * @param   handle   GPT Handle
 * @retval  counter  The current count value of the counter
 */
unsigned int HAL_GPT_GetCounterValueEx(GPT_Handle *handle)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));
    /* Returns count value of the counter. */
    GPT_TC_STS_REG tcValue;
    tcValue.reg = handle->baseAddress->GPT_TC_STS.reg;
    return tcValue.BIT.ro_cnt_val;
}

/**
 * @brief   GPT Trigger DMA Enable
 * @param   handle   GPT Handle
 * @param   triggerDMAType Trigger DMA Type Mask, @ref GPT_TriggerDMAType
 * @retval  BASE_STATUS_OK    Setting succeeded.
 * @retval  BASE_STATUS_ERROR Setting failed.
 */
BASE_StatusType HAL_GPT_TriggerDMAEnableEx(GPT_Handle *handle, GPT_TriggerDMAType triggerDMAType)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));

    GPT_SOCDR_EN_REG socdrEn;
    /* Set GPT trigger DMA enable flags */
    socdrEn.reg = handle->baseAddress->GPT_SOCDR_EN.reg;
    switch (triggerDMAType) {
        case GPT_PWM0_TRIGGER_DMA:     /* DMA request is triggered when the PWM output finish. */
            socdrEn.BIT.rg_dbr_pwm0_en = BASE_CFG_ENABLE;
            socdrEn.BIT.rg_dsr_pwm0_en = BASE_CFG_ENABLE;
            break;
        case GPT_PERIOD_TRIGGER_DMA:  /* DMA request is triggered when the PWM period output finish. */
            socdrEn.BIT.rg_dbr_prd_en = BASE_CFG_ENABLE;
            socdrEn.BIT.rg_dsr_prd_en = BASE_CFG_ENABLE;
            break;
        /* DMA request is triggered when the PWM output finish or period output finish. */
        case GPT_PWM0_PERIOD_TRIGGER_DMA:
            socdrEn.BIT.rg_dbr_pwm0_en = BASE_CFG_ENABLE;
            socdrEn.BIT.rg_dsr_pwm0_en = BASE_CFG_ENABLE;
            socdrEn.BIT.rg_dbr_prd_en  = BASE_CFG_ENABLE;
            socdrEn.BIT.rg_dsr_prd_en  = BASE_CFG_ENABLE;
            break;
        default:
            return BASE_STATUS_ERROR; /* Failed to set the DMA trigger. */
    }
    handle->baseAddress->GPT_SOCDR_EN.reg = socdrEn.reg;
    return BASE_STATUS_OK;
}

/**
 * @brief   GPT Trigger DMA Disable
 * @param   handle   GPT Handle
 * @param   triggerDMAType Trigger DMA Type Mask, @ref GPT_TriggerDMAType
 * @retval  BASE_STATUS_OK    succeeded.
 * @retval  BASE_STATUS_ERROR failed.
 */
BASE_StatusType HAL_GPT_TriggerDMADisableEx(GPT_Handle *handle, GPT_TriggerDMAType triggerDMAType)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));

    GPT_SOCDR_EN_REG socdrEn;
    /* Set GPT trigger DMA enable flags */
    socdrEn.reg = handle->baseAddress->GPT_SOCDR_EN.reg;
    switch (triggerDMAType) {
        case GPT_PWM0_TRIGGER_DMA:  /* Disables triggering DMA request when the PWM output finish. */
            socdrEn.BIT.rg_dbr_pwm0_en = BASE_CFG_DISABLE;
            socdrEn.BIT.rg_dsr_pwm0_en = BASE_CFG_DISABLE;
            break;
        case GPT_PERIOD_TRIGGER_DMA: /* Disables triggering DMA request when the PWM period out finish. */
            socdrEn.BIT.rg_dbr_prd_en = BASE_CFG_DISABLE;
            socdrEn.BIT.rg_dsr_prd_en = BASE_CFG_DISABLE;
            break;
        /* Disables triggering DMA request when the PWM output finish or period output finish. */
        case GPT_PWM0_PERIOD_TRIGGER_DMA:
            socdrEn.BIT.rg_dbr_pwm0_en = BASE_CFG_DISABLE;
            socdrEn.BIT.rg_dsr_pwm0_en = BASE_CFG_DISABLE;
            socdrEn.BIT.rg_dbr_prd_en  = BASE_CFG_DISABLE;
            socdrEn.BIT.rg_dsr_prd_en  = BASE_CFG_DISABLE;
            break;
        default:
            return BASE_STATUS_ERROR; /* Failed to set the DMA trigger. */
    }
    handle->baseAddress->GPT_SOCDR_EN.reg = socdrEn.reg;
    return BASE_STATUS_OK;
}

/**
 * @brief   GPT Trigger ADC Enable
 * @param   handle   GPT Handle
 * @param   triggerADCType ADC Type Mask, @ref GPT_TriggerADCType
 * @retval  BASE_STATUS_OK    succeeded.
 * @retval  BASE_STATUS_ERROR failed.
 */
BASE_StatusType HAL_GPT_TriggerADCEnableEx(GPT_Handle *handle, GPT_TriggerADCType triggerADCType)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));

    GPT_SOCDR_EN_REG socdrEn;
    /* Set GPT trigger ADC enable flags */
    socdrEn.reg = handle->baseAddress->GPT_SOCDR_EN.reg;
    switch (triggerADCType) {
        case GPT_PWM0_TRIGGER_ADC:   /* Enable triggering ADC request when the PWM output finish. */
            socdrEn.BIT.rg_soc_pwm0_en = BASE_CFG_ENABLE;
            break;
        case GPT_PERIOD_TRIGGER_ADC: /* Enable triggering ADC request when the PWM period output finish. */
            socdrEn.BIT.rg_soc_prd_en = BASE_CFG_ENABLE;
            break;
        /* Enable triggering ADC request when the PWM period output finish and PWM output finish. */
        case GPT_PWM0_PERIOD_TRIGGER_ADC:
            socdrEn.BIT.rg_soc_pwm0_en = BASE_CFG_ENABLE;
            socdrEn.BIT.rg_soc_prd_en = BASE_CFG_ENABLE;
            break;
        default:
            return BASE_STATUS_ERROR; /* Failed to set the ADC trigger. */
    }
    handle->baseAddress->GPT_SOCDR_EN.reg = socdrEn.reg;
    return BASE_STATUS_OK;
}

/**
 * @brief   GPT Trigger ADC Disable
 * @param   handle   GPT Handle
 * @param   triggerADCType Trigger DMA Type Mask, @ref GPT_TriggerADCType
 * @retval  BASE_STATUS_OK    succeeded.
 * @retval  BASE_STATUS_ERROR failed.
 */
BASE_StatusType HAL_GPT_TriggerADCDisableEx(GPT_Handle *handle, GPT_TriggerADCType triggerADCType)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));

    GPT_SOCDR_EN_REG socdrEn;
    /* Set GPT trigger ADC enable flags */
    socdrEn.reg = handle->baseAddress->GPT_SOCDR_EN.reg;
    switch (triggerADCType) {
        case GPT_PWM0_TRIGGER_ADC:    /* Disable triggering ADC request when the PWM output finish. */
            socdrEn.BIT.rg_soc_pwm0_en     =  BASE_CFG_DISABLE;
            break;
        case GPT_PERIOD_TRIGGER_ADC:  /* Disable triggering ADC request when the PWM period output finish. */
            socdrEn.BIT.rg_soc_prd_en      =  BASE_CFG_DISABLE;
            break;
        /* Disable triggering ADC request when the PWM output finish and period output finish. */
        case GPT_PWM0_PERIOD_TRIGGER_ADC:
            socdrEn.BIT.rg_soc_pwm0_en     =  BASE_CFG_DISABLE;
            socdrEn.BIT.rg_soc_prd_en      =  BASE_CFG_DISABLE;
            break;
        default:
            return BASE_STATUS_ERROR; /* Failed to set the ADC trigger. */
    }
    handle->baseAddress->GPT_SOCDR_EN.reg = socdrEn.reg;
    return BASE_STATUS_OK;
}

/**
 * @brief   Get Current PWM0 Number
 * @param   handle     GPT Handle
 * @retval  pwmNumber  Current PWM0 Number, Only valid when PWM0_CFG.rg_pwm0_keep = 1
 */
unsigned int HAL_GPT_GetCurrentPWM0NumberEx(GPT_Handle *handle)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));

    GPT_PWM0_STA_REG pwm0Stat;
    /* Clear GPT trigger ADC enable flags */
    pwm0Stat.reg = handle->baseAddress->GPT_PWM0_STA.reg;
    return pwm0Stat.BIT.ro_pwm0_num_sta;
}

/**
 * @brief   Injected PWM output completion interrupt, which takes effect only when PWM waves are output.
 * @param   handle   GPT Handle
 * @param   softInjOutFin  1: enable 0: disable @ref GPT_SetOption
 * @retval  BASE_STATUS_OK   Success
 * @retval  BASE_STATUS_ERROR Parameter check fail
 */
BASE_StatusType HAL_GPT_SoftInjOutFinIntEx(GPT_Handle *handle, GPT_SetOption softInjOutFin)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));
    GPT_PARAM_CHECK_WITH_RET(IsGptSetOption(softInjOutFin), BASE_STATUS_ERROR);
    /* Software injection PWM period output finish interrupt. */
    GPT_INT_INJ_REG  intInj;
    intInj.reg = handle->baseAddress->GPT_INT_INJ.reg;
    intInj.BIT.rg_pwm0_int_inj = softInjOutFin;
    handle->baseAddress->GPT_INT_INJ.reg = intInj.reg;
    return BASE_STATUS_OK;
}

/**
 * @brief  Injected PWM period finish interrupt, which takes effect only when PWM waves are output.
 * @param   handle   GPT Handle
 * @param   softInjPeriod  1: enable 0: disable, @ref GPT_SetOption
 * @retval  BASE_STATUS_OK   Success
 * @retval  BASE_STATUS_ERROR Parameter check fail
 */
BASE_StatusType HAL_GPT_SoftInjPeriodFinIntEx(GPT_Handle *handle, GPT_SetOption softInjPeriod)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));
    GPT_PARAM_CHECK_WITH_RET(IsGptSetOption(softInjPeriod), BASE_STATUS_ERROR);
    /* Software injection PWM output finish interrupt. */
    GPT_INT_INJ_REG  intInj;
    intInj.reg = handle->baseAddress->GPT_INT_INJ.reg;
    intInj.BIT.rg_prd_int_inj = softInjPeriod;
    handle->baseAddress->GPT_INT_INJ.reg = intInj.reg;
    return BASE_STATUS_OK;
}