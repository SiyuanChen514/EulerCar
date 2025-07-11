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
  * @file    sample_adc_sync_sample.c
  * @author  MCU Driver Team
  * @brief   adc sample module.
  * @details In synchronous sampling mode, the ADC uses software to trigger simultaneous sampling of the two ADC
  *          channels. After sampling is triggered, the ADC determines that the ADC conversion is complete and reads
  *          the result ADC Dual-Channel synchronous sample, the trigger sources of the two channels must be the same.
  *          (1) ADC strigger source is software. Use HAL_ADC_SoftTrigMultiSample() to configure software tirgger SOC0
  *          and SOC1.
  *          (2) ADC sample source is ADC1_SOC0 and ADC1_SOC1. Select sample source in "g_adc.baseAddress" of
  *          SystemInit(), SOC0 and SOC1 are determined by the ADC_SYNCSAMPLE_GROUP_1 in SystemInit().
  *          (3) The ADC conversion result is read from HAL_ADC_GetConvResult().
  *          After the conversion is complete, use HAL_ADC_GetConvResult() to obtain result.
  *          Check whether the ADC conversion is complete through the HAL_ADC_CheckSocFinish() interface.
  */

#include "sample_adc_sync_sample.h"

/**
  * @brief ADC Dual-Channel synchronous sample.
  * @param None.
  * @retval None.
  */
void ADC_SyncSample(void)
{
    SystemInit();
    DBG_PRINTF("ADC_SyncSample begin\r\n");

    ADC_SoftMultiTrig softTrig = {0};
    softTrig.BIT.trigSoc0 = BASE_CFG_ENABLE;
    softTrig.BIT.trigSoc1 = BASE_CFG_ENABLE;
    HAL_ADC_SoftTrigMultiSample(&g_adc, softTrig);  /* Trigger both SOC0 and SOC1 */

    BASE_FUNC_DELAY_MS(5);  /* delay 5 ms */
    if (HAL_ADC_CheckSocFinish(&g_adc, ADC_SOC_NUM0) == BASE_STATUS_ERROR) {
        DBG_PRINTF("SOC0 did not complete sampling and conversion\r\n");
        return;
    }
    if (HAL_ADC_CheckSocFinish(&g_adc, ADC_SOC_NUM1) == BASE_STATUS_ERROR) {
        DBG_PRINTF("SOC1 did not complete sampling and conversion\r\n");
        return;
    }
    /* Read sampling results */
    unsigned ret1 = HAL_ADC_GetConvResult(&g_adc, ADC_SOC_NUM0);
    unsigned ret2 = HAL_ADC_GetConvResult(&g_adc, ADC_SOC_NUM1);
    DBG_PRINTF("ret1: %x, ret2: %x\r\n", ret1, ret2);
}