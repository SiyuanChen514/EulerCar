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
  * @file    sample_adc_single_trigger_dma.c
  * @author  MCU Driver Team
  * @brief   adc sample module.
  * @details In single sampling mode, the ADC sampling is triggered by software. After the sampling is complete,
  *          the DMA is triggered to transfer the result.
  *          (1) ADC strigger source is software. Use HAL_ADC_SoftTrigSample() to configure software tirgger SOC1.
  *          (2) ADC sample source is ADC1_SOC1. Select sample source in "g_adc.baseAddress" of SystemInit(),
  *           "ADC_SOC_NUM3" can be Modified. External input source: GPIO2_2/GPIO2_3
  *          (3) The ADC conversion result is read from DMA interrupt callback function ADC1DMACallback().
  */

#include "sample_adc_single_trigger_dma.h"

static unsigned int g_adcRet[10] = {0};
/**
  * @brief User callback function of ADC interrupt one.
  * @param adcHandle ADC handle.
  * @retval None.
  */
void ADC_DMACallback(ADC_Handle *adcHandle)
{
    BASE_FUNC_UNUSED(adcHandle);
    DBG_PRINTF("DMACallback\r\n");
    DBG_PRINTF("result: %d\r\n", g_adcRet[0]);
    float voltage = (float)g_adcRet[0] / (float)4096 * 3.3;  /* 4096 and 3.3 are for Sample Value Conversion */
    DBG_PRINTF("voltage: %f\r\n", voltage);
}

/**
  * @brief ADC channels sample with DMA. Transfers four SOC conversion results in one DMA request.
  * @param None.
  * @retval None.
  */
void ADC_SingleTriggerDma(void)
{
    SystemInit();
    DBG_PRINTF("ADC_SingleTriggerDma begin\r\n");

    HAL_ADC_StartDma(&g_adc, ADC_SOC_NUM1, ADC_SOC_NUM1, g_adcRet);   /* Transfer converted data of SOC1 */

    HAL_ADC_SoftTrigSample(&g_adc, ADC_SOC_NUM1);         /* Software triggers SOC1 */
}