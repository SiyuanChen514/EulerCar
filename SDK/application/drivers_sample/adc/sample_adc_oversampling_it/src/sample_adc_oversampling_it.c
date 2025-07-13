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
  * @file    sample_adc_oversampling_it.c
  * @author  MCU Driver Team
  * @brief   adc sample module.
  * @details In oversampling mode, the ADC uses software triggers oversampling on the same channel.
  *          the ADC interrupt is triggered and conversion result is read in Oversampling event interrupt function.
  *          (1) ADC strigger source is software. Use HAL_ADC_SoftTrigSample() to configure software tirgger SOC1.
  *          (2) ADC sample source is SOC1. Select sample source in "g_adc.baseAddress" of SystemInit(),
  *           "ADC_SOC_NUM1" can be Modified.
  *          (3) The ADC conversion result is read from interrupt callback function ADC_OversamplingCalBack interface.
  *          If interrupt is not used, after the conversion is complete, use HAL_ADC_GetOversamplingResultEx() to obtain
  *          result. Check whether ADC conversion is complete through the HAL_ADC_CheckOversamplingFinishEx() interface.
  */
#include "adc_ex.h"
#include "sample_adc_oversampling_it.h"

/**
  * @brief ADC Oversampling callback function.
  * @param handle.
  * @retval None.
  */
void ADC_OversamplingCalBack(ADC_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    unsigned int ret = HAL_ADC_GetOversamplingResultEx(&g_adc); /* Software trigger ADC sampling */
    DBG_PRINTF("ADC_OversamplingCalBack, result: %x\r\n", ret);
    float voltage = ((float)ret / (float)4096 * 3.3) / (float)(64 >> 3);  /* 4096 , 3.3, 64 , 3 are for Conversion */
    DBG_PRINTF("voltage: %f\r\n", voltage);
}

/**
  * @brief ADC single channel sample without DMA and interrupt.
  * @param None.
  * @retval None.
  */
void ADC_OverSamplingIt(void)
{
    SystemInit();
    DBG_PRINTF("ADC_OverSamplingIt begin\r\n");
    HAL_ADC_SoftTrigSample(&g_adc, ADC_SOC_NUM1);  /* Software trigger ADC sampling */
    return;
}