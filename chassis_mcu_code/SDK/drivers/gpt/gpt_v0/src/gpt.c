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
  * @file      gpt.c
  * @author    MCU Driver Team
  * @brief     GPT module driver.
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the GPT.
  *                + Initialization function of GPT
  *                + Clock Configuration of GPT
  *                + Get GPT State and Apply GPT
  */

#include "gpt.h"

#define FREQ_1000M         (1000 * 1000 * 1000)

/**
 * @brief   Init the GPT.
 * @param   handle   GPT Handle.
 * @retval  BASE_STATUS_OK   Success
 * @retval  BASE_STATUS_ERROR Parameter check fail
 */
BASE_StatusType HAL_GPT_Init(GPT_Handle *handle)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));

    if (HAL_GPT_Config(handle) == BASE_STATUS_ERROR) {
        return BASE_STATUS_ERROR;  /* Configuration failed. */
    }
    handle->baseAddress->PWM_CTRL.BIT.pwm_enable = 0;

    return BASE_STATUS_OK;
}

/**
 * @brief   Start GPT
 * @param   handle   GPT Handle.
 * @retval  None
 */
void HAL_GPT_Start(GPT_Handle *handle)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));
    handle->baseAddress->PWM_CTRL.BIT.pwm_enable = BASE_CFG_SET;
}

/**
 * @brief   Stop GPT
 * @param   handle   GPT Handle.
 * @retval  None
 */
void HAL_GPT_Stop(GPT_Handle *handle)
{
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));
    handle->baseAddress->PWM_CTRL.BIT.pwm_enable = BASE_CFG_UNSET;
}

/**
 * @brief   Set basic GPT parameters.
 * @param   handle   GPT Handle.
 * @retval  BASE_StatusType: BASE_STATUS_OK    Success.
 * @retval                   BASE_STATUS_ERROR Configuration failed.
 */
BASE_StatusType HAL_GPT_Config(GPT_Handle *handle)
{
    unsigned long long pwmPeriod;
    unsigned long long pwmDuty;
    unsigned int freq;
    /* Input parameter macro check. */
    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));
    GPT_PARAM_CHECK_WITH_RET((handle->period > 0), BASE_STATUS_ERROR);
    GPT_PARAM_CHECK_WITH_RET((handle->duty > 0), BASE_STATUS_ERROR);
    GPT_PARAM_CHECK_WITH_RET(handle->duty <= handle->period, BASE_STATUS_ERROR); /* Duty is must less than period. */
    GPT_PARAM_CHECK_WITH_RET(IsGptPwmNum(handle->pwmNum), BASE_STATUS_ERROR);
    GPT_PARAM_CHECK_WITH_RET((handle->pwmPolarity == BASE_CFG_SET) || \
                             (handle->pwmPolarity == BASE_CFG_UNSET), BASE_STATUS_ERROR);
    GPT_PARAM_CHECK_WITH_RET((handle->pwmKeep == BASE_CFG_SET) || \
                             (handle->pwmKeep == BASE_CFG_UNSET), BASE_STATUS_ERROR);
    /* Clock frequency of the GPT. */
    freq = HAL_CRG_GetIpFreq((void*)handle->baseAddress);
    pwmPeriod = handle->period;
    pwmPeriod = (freq * pwmPeriod) / FREQ_1000M;   /* The period(us) is converted to the counting period. */
    if (pwmPeriod > GPT_PWM_PERIOD_MAX_VALUE) {
        pwmPeriod = GPT_PWM_PERIOD_INVALID_VALUE;
    }
    /* The value of the duty cycle (percentage) in period. */
    pwmDuty = handle->duty;
    pwmDuty = (freq * pwmDuty) / FREQ_1000M;
    if (pwmDuty > GPT_PWM_DUTY_MAX_VALUE) {
        pwmDuty = GPT_PWM_DUTY_INVALID_VALUE;
    }
    /* Check the duty and counting period. */
    if (!IsGptPeriod((unsigned int)pwmPeriod) || !IsGptDuty((unsigned int)pwmDuty) || (pwmDuty >= pwmPeriod)) {
        return BASE_STATUS_ERROR;
    }
    /* Writing duty cycle and count period values */
    handle->baseAddress->pwm_period = (unsigned int)pwmPeriod;
    handle->baseAddress->pwm_duty = (unsigned int)pwmDuty;
    handle->baseAddress->PWM_CFG2.pwm_num = handle->pwmNum;  /* Number of output PWM. */
    handle->baseAddress->PWM_CTRL.BIT.pwm_inv = handle->pwmPolarity;
    handle->baseAddress->PWM_CTRL.BIT.pwm_keep = handle->pwmKeep;
    return BASE_STATUS_OK;
}

/**
 * @brief   Get State of GPT.
 * @param   handle   GPT Handle.
 * @retval  BASE_STATUS_OK   Success
 * @retval  BASE_STATUS_ERROR Parameter check fail
 */
BASE_StatusType HAL_GPT_GetConfig(GPT_Handle *handle)
{
    unsigned long long period;
    unsigned long long duty;
    unsigned int freq;

    GPT_ASSERT_PARAM(handle != NULL);
    GPT_ASSERT_PARAM(IsGPTInstance(handle->baseAddress));

    freq = HAL_CRG_GetIpFreq((void *)handle->baseAddress);
    if (freq == 0) {
        return BASE_STATUS_ERROR;
    }
    period = handle->baseAddress->pwm_period;
    period = (period * FREQ_1000M + freq - 1) / freq;
    handle->period      = period;
    duty = handle->baseAddress->pwm_duty;
    duty = (duty * FREQ_1000M + freq - 1) / freq;
    handle->duty        = duty;
    handle->pwmEnable   = handle->baseAddress->PWM_CTRL.BIT.pwm_enable;
    handle->pwmPolarity = handle->baseAddress->PWM_CTRL.BIT.pwm_inv;
    handle->pwmKeep     = handle->baseAddress->PWM_CTRL.BIT.pwm_keep;
    handle->pwmNum      = handle->baseAddress->PWM_CFG2.pwm_num;

    return BASE_STATUS_OK;
}
