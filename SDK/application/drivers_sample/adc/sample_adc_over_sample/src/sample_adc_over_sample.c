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
  * @file    sample_adc_over_sample.c
  * @author  MCU Driver Team
  * @brief   adc over sample module.
  * @details In the oversampling example, the ADC samples the same channel twice and uses software to trigger ADC
  *          sampling. After the sampling is triggered, determines the ADC conversion is complete and reads the result.
  *          (1) ADC strigger source is software. Use HAL_ADC_SoftTrigMultiSample() to configure software tirgger SOC3
  *          and SOC5.
  *          (2) ADC sample source is ADC1_SOC3 and ADC1_SOC5. Select sample source in "g_adc.baseAddress" of
  *           SystemInit(), "ADC_SOC_NUM3" and "ADC_SOC_NUM5" can be Modified. External input source: GPIO2_2/GPIO2_3.
  *          (3) The ADC conversion result is read from HAL_ADC_GetConvResult interface.
  *          If interrupt is not used, after the conversion is complete, use HAL_ADC_GetConvResult() to obtain result.
  *          Check whether the ADC conversion is complete through the HAL_ADC_CheckSocFinish() interface.
  */

#include "sample_adc_over_sample.h"
/**
  * @brief ADC over sample.
  * @param None.
  * @retval None.
  */
void ADC_OverSample(void)
{
    SystemInit();
    DBG_PRINTF("ADC_overSample begin\r\n");

    ADC_SoftMultiTrig softTrig = {0};
    softTrig.BIT.trigSoc3 = BASE_CFG_ENABLE;
    softTrig.BIT.trigSoc5 = BASE_CFG_ENABLE;
    HAL_ADC_SoftTrigMultiSample(&g_adc, softTrig);  /* Trigger both SOC3 and SOC5 */

    BASE_FUNC_DELAY_MS(5);  /* delay 5 ms */
    if (HAL_ADC_CheckSocFinish(&g_adc, ADC_SOC_NUM3) == BASE_STATUS_ERROR) {
        DBG_PRINTF("SOC3 did not complete sampling and conversion\r\n");
        return;
    }
    if (HAL_ADC_CheckSocFinish(&g_adc, ADC_SOC_NUM5) == BASE_STATUS_ERROR) {
        DBG_PRINTF("SOC5 did not complete sampling and conversion\r\n");
        return;
    }
    /* Read sampling results */
    unsigned ret1 = HAL_ADC_GetConvResult(&g_adc, ADC_SOC_NUM3);
    unsigned ret2 = HAL_ADC_GetConvResult(&g_adc, ADC_SOC_NUM5);
    DBG_PRINTF("ret1: %x, ret2: %x\r\n", ret1, ret2);
}