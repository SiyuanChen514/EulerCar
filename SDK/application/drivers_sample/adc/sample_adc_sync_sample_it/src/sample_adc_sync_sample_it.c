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
  * @file    sample_adc_sync_sample_it.c
  * @author  MCU Driver Team
  * @brief   adc sample module.
  * @details In synchronous sampling mode, software triggers simultaneous sampling of the two ADC channels. After
  *          sampling, an ADC interrupt is triggered and conversion result is read in the interrupt callback function.
  *          (1) ADC strigger source is software. Use HAL_ADC_SoftTrigMultiSample() to configure software tirgger SOC4
  *          and SOC5.
  *          (2) ADC sample source is ADC1_SOC4 and ADC1_SOC5. Select sample source in "g_adc.baseAddress" of
  *          SystemInit(), SOC4 and SOC5 are determined by the ADC_SYNCSAMPLE_GROUP_3 in SystemInit().
  *          (3) The ADC conversion result is read from ADC interrupt callback function ADC1Interrupt4Callback().
  */
#include "sample_adc_sync_sample_it.h"

/**
  * @brief User callback function of ADC interrupt one.
  * @param adcHandle ADC handle.
  * @retval None.
  */
void ADC1Interrupt4Callback(ADC_Handle *adcHandle)
{
    BASE_FUNC_UNUSED(adcHandle);
    DBG_PRINTF("ADC1Interrupt4Callback\r\n");   /* ADC Synchronous Sampling Complete */
    unsigned ret1 = HAL_ADC_GetConvResult(&g_adc, ADC_SOC_NUM4);
    unsigned ret2 = HAL_ADC_GetConvResult(&g_adc, ADC_SOC_NUM5);
    DBG_PRINTF("ret1: %x, ret2: %x\r\n", ret1, ret2);
}

/**
  * @brief ADC Dual-Channel synchronous sample with interrupt.
  * @param None.
  * @retval None.
  */
void ADC_SyncSampleWithIt(void)
{
    SystemInit();
    DBG_PRINTF("ADC_SyncSampleWithIt begin\r\n");
    ADC_SoftMultiTrig softTrig = {0};
    softTrig.BIT.trigSoc4 = BASE_CFG_ENABLE;         /* Group3 -- Synchronous sample group: SOC4 and SOC5 */
    softTrig.BIT.trigSoc5 = BASE_CFG_ENABLE;
    HAL_ADC_SoftTrigMultiSample(&g_adc, softTrig);   /* Trigger both SOC4 and SOC5 */
}