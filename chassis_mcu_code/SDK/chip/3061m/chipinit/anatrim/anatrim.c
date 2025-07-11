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
  * @file      anatrim.c
  * @author    MCU Driver Team
  * @brief     Chip Init modlue.
  * @details   Calibration of analog module parameters.
  */
#include "anatrim.h"

float g_tsensorGain = 0.00041f;

/**
 * @brief Calculate the conversion gain of the tsensor.
 * @param data, original data.
 * @retval None
 */
static void CalculateGain(unsigned int data)
{
    g_tsensorGain = ((float)(data) / 10000000.0f);
}

/**
 * @brief Obtains the chip ID.
 * @param None
 * @retval None
 */
static bool CHIP_GetInfo(void)
{
    FOTP_INFO_RGN0_NUMBER_4 emptyData;
    FOTP_INFO_RGN0_NUMBER_2 idData;
    FOTP_InfoGet(FOTP_INFO_RNG0, 4U, (void *)&emptyData.comData);   /* 4 is the number of fotp_empty_flag in otp */
    FOTP_InfoGet(FOTP_INFO_RNG0, 2U, (void *)&idData.comData);      /* 2 is the number of idData in otp */
    if (emptyData.REG.fotp_empty_flag != 0x5AA59669 || idData.REG.chip_id == 0xFFFFFFFF) {
        return false;
    }
    return true;
}

/**
 * @brief Analog module trim.
 * @param None
 * @retval None
 */
static void CHIP_AnalogTrim(void)
{
    FOTP_INFO_RGN0_NUMBER_20 trimData20;
    FOTP_InfoGet(FOTP_INFO_RNG0, 20U, (void *)&trimData20.comData); /* 20 is the number of trim data in otp */
    /* VREF */
    VREF->VREF_TRIM0.BIT.da_iref_trim = trimData20.REG.data0.da_iref_trim;
    VREF->VREF_TRIM0.BIT.da_ref_vref_trim = trimData20.REG.data0.da_ref_vref_trim;
    VREF->VREF_TRIM0.BIT.da_ref_vbg_trim = trimData20.REG.data0.da_ref_vbg_trim;
    unsigned int value = trimData20.REG.data1.da_ref_temp_trim3;
    value |= (trimData20.REG.data1.da_ref_temp_trim2 << 8U);    /* Shift left by 8 bits */
    value |= (trimData20.REG.data1.da_ref_temp_trim1 << 16U);   /* Shift left by 16 bits */
    value |= (trimData20.REG.data0.da_ref_temp_trim0 << 24U);   /* Shift left by 24 bits */
    VREF->VREF_TRIM1.reg = value;

    FOTP_INFO_RGN0_NUMBER_21 trimData21;
    FOTP_InfoGet(FOTP_INFO_RNG0, 21U, (void *)&trimData21.comData); /* 21 is the number of trim data in otp */
    /* ADC */
    ADC0->ADC_OEGE_TRIM.BIT.cfg_gain_cali_trim = trimData21.REG.data1.saradc_gain;
    ADC0->ADC_OEGE_TRIM.BIT.cfg_ofst_cali_trim = trimData21.REG.data1.saradc_offset;
   
    /* TSENSOR */
    TSENSOR->TSENSOR_TRIM.reg = trimData20.REG.data1.da_ref_vptat_trim;
    ADC0->ADC_TSENSOR_TRIM.BIT.cfg_tsensor_ofst_trim = trimData21.REG.data2.ts_offset;
    CalculateGain(trimData21.REG.data3.ts_gain);

    /* PGA */
    PGA0->PGA_TRIM.BIT.da_pga_vos_trim = trimData21.REG.data0.da_pga0_vos_trim;
    PGA1->PGA_TRIM.BIT.da_pga_vos_trim = trimData21.REG.data0.da_pga1_vos_trim;

    FOTP_INFO_RGN0_NUMBER_22 trimData22;
    FOTP_InfoGet(FOTP_INFO_RNG0, 22U, (void *)&trimData22.comData); /* 22 is the number of trim data in otp */
    ADC0->ADC_PGA0_OEGE_TRIM0.BIT.cfg_pga0_gain_trim2 = trimData22.REG.data0.pga0_gain2;
    ADC0->ADC_PGA0_OEGE_TRIM0.BIT.cfg_pga0_ofst_trim2 = trimData22.REG.data0.pga0_offset2;
    ADC0->ADC_PGA0_OEGE_TRIM1.BIT.cfg_pga0_gain_trim4 = trimData22.REG.data1.pga0_gain4;
    ADC0->ADC_PGA0_OEGE_TRIM1.BIT.cfg_pga0_ofst_trim4 = trimData22.REG.data1.pga0_offset4;
    ADC0->ADC_PGA0_OEGE_TRIM2.BIT.cfg_pga0_gain_trim8 = trimData22.REG.data2.pga0_gain8;
    ADC0->ADC_PGA0_OEGE_TRIM2.BIT.cfg_pga0_ofst_trim8 = trimData22.REG.data2.pga0_offset8;
    ADC0->ADC_PGA0_OEGE_TRIM3.BIT.cfg_pga0_gain_trim16 = trimData22.REG.data3.pga0_gain16;
    ADC0->ADC_PGA0_OEGE_TRIM3.BIT.cfg_pga0_ofst_trim16 = trimData22.REG.data3.pga0_offset16;

    FOTP_INFO_RGN0_NUMBER_23 trimData23;
    FOTP_InfoGet(FOTP_INFO_RNG0, 23U, (void *)&trimData23.comData); /* 23 is the number of trim data in otp */
    ADC0->ADC_PGA1_OEGE_TRIM0.BIT.cfg_pga1_gain_trim2 = trimData23.REG.data0.pga1_gain2;
    ADC0->ADC_PGA1_OEGE_TRIM0.BIT.cfg_pga1_ofst_trim2 = trimData23.REG.data0.pga1_offset2;
    ADC0->ADC_PGA1_OEGE_TRIM1.BIT.cfg_pga1_gain_trim4 = trimData23.REG.data1.pga1_gain4;
    ADC0->ADC_PGA1_OEGE_TRIM1.BIT.cfg_pga1_ofst_trim4 = trimData23.REG.data1.pga1_offset4;
    ADC0->ADC_PGA1_OEGE_TRIM2.BIT.cfg_pga1_gain_trim8 = trimData23.REG.data2.pga1_gain8;
    ADC0->ADC_PGA1_OEGE_TRIM2.BIT.cfg_pga1_ofst_trim8 = trimData23.REG.data2.pga1_offset8;
    ADC0->ADC_PGA1_OEGE_TRIM3.BIT.cfg_pga1_gain_trim16 = trimData23.REG.data3.pga1_gain16;
    ADC0->ADC_PGA1_OEGE_TRIM3.BIT.cfg_pga1_ofst_trim16 = trimData23.REG.data3.pga1_offset16;
}

/**
 * @brief Parameter calibration entry of the analog module.
 * @param None
 * @retval None
 */
void ANATRIM_Entry(void)
{
    if (CHIP_GetInfo() == false) { /* If the chip information is incorrect, calibration is not performed */
        return;
    }
    HAL_CRG_IpEnableSet((void *)ADC0, IP_CLK_ENABLE); /* Enable the clock for calibration */
    HAL_CRG_IpEnableSet((void *)PGA0, IP_CLK_ENABLE);
    HAL_CRG_IpEnableSet((void *)PGA1, IP_CLK_ENABLE);
    CHIP_AnalogTrim();
    HAL_CRG_IpEnableSet((void *)ADC0, IP_CLK_DISABLE); /* The clock is disabled after calibration */
    HAL_CRG_IpEnableSet((void *)PGA0, IP_CLK_DISABLE);
    HAL_CRG_IpEnableSet((void *)PGA1, IP_CLK_DISABLE);
}