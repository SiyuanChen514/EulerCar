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
  * @file    sample_adc_associative_trigger_apt.c
  * @author  MCU Driver Team
  * @brief   adc sample sampling by apt trigger.
  * @details Use APT to trigger ADC sampling, trigger an ADC interrupt after sampling, and read the ADC conversion
  *          result in the interrupt callback function.
  *          (1) ADC strigger source is APT0_SOCA. Select the ADC trigger source in "socParam.periphTrigSource"
  *          from SystemInit() and modify the initialization interface of the APT.
  *          (2) ADC sample source is ADC1_SOC1. Select sample source in "g_adc.baseAddress" and "ADC_SOC_NUM1"
  *          from SystemInit(). External input source: GPIO2_2/GPIO2_3
  *          (3) The ADC conversion result is read from the interrupt callback function ADC_Init2FromApt interface.
  *          If interrupt is not used, after the conversion is complete, use HAL_ADC_GetConvResult() to obtain result.
  *          Check whether the ADC conversion is complete through the HAL_ADC_CheckSocFinish() interface.
  */
#include "sample_adc_associative_trigger_apt.h"

/**
  * @brief ADC sampling interrupt callback triggered by APT.
  * @param None.
  * @retval None.
  */
void ADC_Init2FromApt(ADC_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    unsigned int ret = HAL_ADC_GetConvResult(&g_adc, ADC_SOC_NUM1);  /* Read ADC sample result */
    DBG_PRINTF("APT trigger sampling completed, result: %x\r\n", ret);
    float voltage = (float)ret / (float)4096 * 3.3;  /* 4096 and 3.3 are for Sample Value Conversion */
    DBG_PRINTF("voltage: %f\r\n", voltage);
}

/**
  * @brief ADC single channel sample by trigger from apt.
  * @param None.
  * @retval None.
  */
void ADC_AptTrigger(void)
{
    SystemInit();
    DBG_PRINTF("ADC_AptTrigger begin\r\n");
    HAL_ADC_StartIt(&g_adc);
    HAL_APT_StartModule(RUN_APT0);  /* APT0 is enable and generates the APT0_SOCA trigger signal */
    return;
}