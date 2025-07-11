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
  * @file      adc_tsensor.c
  * @author    MCU Driver Team
  * @brief     tsensor init module.
  * @details   tsensor initialization verification parameter table during startup
  */

#include "fotp_info_read.h"
#include "adc_tsensor.h"

TSENSOR_VrefList g_tsensor[1] = {0};
unsigned int g_hosc_ctrim = 0x1FF;

/**
 * @brief ADC initialize vref power.
 * @param None.
 * @retval None.
 */
void TSENSOR_InitVrefList(void)
{
    FOTP_INFO_RGN0_NUMBER_1 tsensorRef;
    FOTP_InfoGet(FOTP_INFO_RNG0, 1, &tsensorRef.comData);   /* 1 is the index of tsensorRef in otp */
    unsigned int t = tsensorRef.REG.data3.ts_ref_t0_ft_rt - 57;  /* offset temperature is -57 */
    float v = tsensorRef.REG.data3.ts_ref_v0_ft_rt * 0.0031f + 0.65f;  /* offset Voltage is 0.65, degree is 0.0031 */
    float slope = (v) / (float)(t + 273);  /* reference temperature is -273 */
    g_tsensor->vrefTemp = t;
    g_tsensor->vrefVoltage = v;
    g_tsensor->slope = slope;
    FOTP_INFO_RGN0_NUMBER_1 trimDate;
    FOTP_InfoGet(FOTP_INFO_RNG0, 1, &trimDate.comData);   /* 1 is the number of adc_vref in otp */
    g_hosc_ctrim = trimDate.REG.data2.hosc_ctrim_ft_rt;
    return;
}