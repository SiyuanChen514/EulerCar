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
  * @file      adcinit.c
  * @author    MCU Driver Team
  * @brief     adc init module.
  * @details   adc initialization function during startup
  */

#include "fotp_info_read.h"
#include "adc_ip.h"
#include "dac_ip.h"
#include "adcinit.h"

ADC_TrimValue g_adcTrimList[LIST_NUM] = {0};
ADC_ParameterList g_adcParmList[4][3][2] = {0};
bool g_trimEnable = false;
unsigned int g_versionId = 0xFF;
static void ADC_CreateTrimList(void);
static void ADC_CreateParmList(void);

/**
 * @brief ADC initialize vref power.
 * @param None.
 * @retval None.
 */
void ADC_InitVref(void)
{
    FOTP_INFO_RGN0_NUMBER_4 trimDate;
    FOTP_InfoGet(FOTP_INFO_RNG0, 4, &trimDate.comData);   /* 4 is the number of fotp_empty_flag in otp */
    if (trimDate.REG.fotp_empty_flag == 0x5AA59669) {     /* fotp_empty_flag is 0x5AA59669 */
        g_trimEnable = true;
    }
    FOTP_INFO_RGN0_NUMBER_2 idInfo;
    FOTP_InfoGet(FOTP_INFO_RNG0, 2, &idInfo.comData);   /* 2 is the number of in otp */
    g_versionId = idInfo.REG.data2.version_id;
    FOTP_INFO_RGN0_NUMBER_21 adcVrefTrim;
    if (g_trimEnable == true) {
        FOTP_InfoGet(FOTP_INFO_RNG0, 21, &adcVrefTrim.comData);   /* 21 is the number of adc_vref in otp */
    } else {
        adcVrefTrim.REG.data1.adcvref_bg_trim = 0x10;             /* Use the default value 0x10 */
        adcVrefTrim.REG.data1.adcvref_adcldo_trim = 0x10;
        adcVrefTrim.REG.data2.adcvref_refbuf0_trim_2p0v = 0x10;
        adcVrefTrim.REG.data3.adcvref_refbuf0_trim_2p5v = 0x10;
        adcVrefTrim.REG.data2.adcvref_refbuf1_trim_2p0v = 0x10;
        adcVrefTrim.REG.data3.adcvref_refbuf1_trim_2p5v = 0x10;
        adcVrefTrim.REG.data2.adcvref_refbuf2_trim_2p0v = 0x10;
        adcVrefTrim.REG.data3.adcvref_refbuf2_trim_2p5v = 0x10;
    }
    SYSCTRL1->ADCVREF_CTRL0.BIT.adcvref_bg_trim = adcVrefTrim.REG.data1.adcvref_bg_trim;
    SYSCTRL1->ADCVREF_CTRL0.BIT.adcvref_bg_en = BASE_CFG_ENABLE;
    SYSCTRL1->ADCVREF_CTRL1.BIT.adcvref_adcldo_trim = adcVrefTrim.REG.data1.adcvref_adcldo_trim;
    SYSCTRL1->ADCVREF_CTRL1.BIT.adcvref_adcldo_en = BASE_CFG_ENABLE;
    SYSCTRL1->ADC0_VREF_CTRL.BIT.adcvref_refbuf_trim0_2p0v = adcVrefTrim.REG.data2.adcvref_refbuf0_trim_2p0v;
    SYSCTRL1->ADC0_VREF_CTRL.BIT.adcvref_refbuf_trim0_2p5v = adcVrefTrim.REG.data3.adcvref_refbuf0_trim_2p5v;
    SYSCTRL1->ADC1_VREF_CTRL.BIT.adcvref_refbuf_trim1_2p0v = adcVrefTrim.REG.data2.adcvref_refbuf1_trim_2p0v;
    SYSCTRL1->ADC1_VREF_CTRL.BIT.adcvref_refbuf_trim1_2p5v = adcVrefTrim.REG.data3.adcvref_refbuf1_trim_2p5v;
    SYSCTRL1->ADC2_VREF_CTRL.BIT.adcvref_refbuf_trim2_2p0v = adcVrefTrim.REG.data2.adcvref_refbuf2_trim_2p0v;
    SYSCTRL1->ADC2_VREF_CTRL.BIT.adcvref_refbuf_trim2_2p5v = adcVrefTrim.REG.data3.adcvref_refbuf2_trim_2p5v;
    BASE_FUNC_DELAY_MS(10);     /* Wait for 10 ms until the LDO becomes stable */
    unsigned int ldoStatu = SYSCTRL1->ADCVREF_CTRL1.BIT.adcvref_adcldo_ok;
    if (ldoStatu == BASE_CFG_ENABLE) {
        SYSCTRL1->ADCVREF_CTRL6.reg |= 0x800000;
    }
    if (g_trimEnable == true) {
        ADC_CreateTrimList();
        ADC_CreateParmList();
    }
}

/**
 * @brief Establish ADC calibration parameter list.
 * @param None.
 * @retval None.
 */
static void ADC_CreateTrimList(void)
{
    unsigned int index = 0;
    FOTP_INFO_RGN0_NUMBER_22 adc0Sh0;
    FOTP_InfoGet(FOTP_INFO_RNG0, 22, &adc0Sh0.comData);   /* 22 is the number of adc calibration trim in otp */
    FOTP_INFO_RGN0_NUMBER_23 adc0Sh1;
    FOTP_InfoGet(FOTP_INFO_RNG0, 23, &adc0Sh1.comData);   /* 23 is the number of adc calibration trim in otp */
    /* ADC0 vref2.0 SH0 and SH1 */
    g_adcTrimList[index].gain = adc0Sh0.REG.data0.adc0_sh0_g0p6_ge_trim;
    g_adcTrimList[index++].offset = adc0Sh0.REG.data0.adc0_sh0_g0p6_oe_trim;
    g_adcTrimList[index].gain = adc0Sh1.REG.data0.adc0_sh1_g0p6_ge_trim;
    g_adcTrimList[index++].offset = adc0Sh1.REG.data0.adc0_sh1_g0p6_oe_trim;
    /* ADC0 vref2.5 SH0 and SH1 */
    g_adcTrimList[index].gain = adc0Sh0.REG.data2.adc0_sh0_g0p75_ge_trim;
    g_adcTrimList[index++].offset = adc0Sh0.REG.data2.adc0_sh0_g0p75_oe_trim;
    g_adcTrimList[index].gain = adc0Sh1.REG.data2.adc0_sh1_g0p75_ge_trim ;
    g_adcTrimList[index++].offset = adc0Sh1.REG.data2.adc0_sh1_g0p75_oe_trim;

    FOTP_INFO_RGN0_NUMBER_24 adc1Sh0;
    FOTP_InfoGet(FOTP_INFO_RNG0, 24, &adc1Sh0.comData);   /* 24 is the number of adc calibration trim in otp */
    FOTP_INFO_RGN0_NUMBER_25 adc1Sh1;
    FOTP_InfoGet(FOTP_INFO_RNG0, 25, &adc1Sh1.comData);   /* 25 is the number of adc calibration trim in otp */
    /* ADC1 vref2.0 SH0 and SH1 */
    g_adcTrimList[index].gain = adc1Sh0.REG.data0.adc1_sh0_g0p6_ge_trim;
    g_adcTrimList[index++].offset = adc1Sh0.REG.data0.adc1_sh0_g0p6_oe_trim;
    g_adcTrimList[index].gain = adc1Sh1.REG.data0.adc1_sh1_g0p6_ge_trim;
    g_adcTrimList[index++].offset = adc1Sh1.REG.data0.adc1_sh1_g0p6_oe_trim;
    /* ADC1 vref2.5 SH0 and SH1 */
    g_adcTrimList[index].gain = adc1Sh0.REG.data2.adc1_sh0_g0p75_ge_trim ;
    g_adcTrimList[index++].offset = adc1Sh0.REG.data2.adc1_sh0_g0p75_oe_trim;
    g_adcTrimList[index].gain = adc1Sh1.REG.data2.adc1_sh1_g0p75_ge_trim ;
    g_adcTrimList[index++].offset = adc1Sh1.REG.data2.adc1_sh1_g0p75_oe_trim;

    FOTP_INFO_RGN0_NUMBER_26 adc2Sh0;
    FOTP_InfoGet(FOTP_INFO_RNG0, 26, &adc2Sh0.comData);   /* 26 is the number of adc calibration trim in otp */
    FOTP_INFO_RGN0_NUMBER_27 adc2Sh1;
    FOTP_InfoGet(FOTP_INFO_RNG0, 27, &adc2Sh1.comData);   /* 27 is the number of adc calibration trim in otp */
    /* ADC2 vref2.0 SH0 and SH1 */
    g_adcTrimList[index].gain = adc2Sh0.REG.data0.adc2_sh0_g0p6_ge_trim;
    g_adcTrimList[index++].offset = adc2Sh0.REG.data0.adc2_sh0_g0p6_oe_trim;
    g_adcTrimList[index].gain = adc2Sh1.REG.data0.adc2_sh1_g0p6_ge_trim;
    g_adcTrimList[index++].offset = adc2Sh1.REG.data0.adc2_sh1_g0p6_oe_trim;
    /* ADC2 vref2.5 SH0 and SH1 */
    g_adcTrimList[index].gain = adc2Sh0.REG.data2.adc2_sh0_g0p75_ge_trim ;
    g_adcTrimList[index++].offset = adc2Sh0.REG.data2.adc2_sh0_g0p75_oe_trim;
    g_adcTrimList[index].gain = adc2Sh1.REG.data2.adc2_sh1_g0p75_ge_trim ;
    g_adcTrimList[index++].offset = adc2Sh1.REG.data2.adc2_sh1_g0p75_oe_trim;
}

/**
 * @brief Obtains the adc gain value.
 * @param None.
 * @retval gain value.
 */
float ADC_GetGainTrim(unsigned int index)
{
    unsigned int temp = g_adcTrimList[index].gain;
    float ret = (float)temp / (float)4096;  /* 4096 for decimal conversion */
    return ret;
}

/**
 * @brief Obtains the adc offset value.
 * @param None.
 * @retval offset value.
 */
float ADC_GetOffsetTrim(unsigned int index)
{
    unsigned temp = g_adcTrimList[index].offset;
    float ret;
    if ((temp & 0x2000) == 0x2000) {      /* determine the sign bit[13] */
        temp |= 0xFFFFC000;
        unsigned int tmp = ~(temp - 1);
        ret = (0 - (int)tmp) / (float)2;  /* 2 for decimal conversion */
    } else {
        temp &= 0x1FFF;
        ret = (float)temp / (float)2;  /* 2 for decimal conversion */
    }
    return ret;
}

/**
 * @brief Establish ADC calibration parameter list.
 * @param None.
 * @retval None.
 */
static void ADC_CreateParmList(void)
{
    float gain, offset, tmpk2;
    unsigned int addrIndex, vrefIndex, shIndex, tmpIndex;
    for (unsigned int index = 0; index < LIST_NUM; ++index) {
        addrIndex = index / 4;   /* 4 used to create list */
        tmpIndex = index % 4;    /* 4 used to create list */
        vrefIndex = ((tmpIndex / 2) == 1) ? 1 : 2;  /* index of vref2.0 is 2, index of vref2.0 is 1 */
        shIndex = index % 2;     /* 2 used to create list */
        gain = ADC_GetGainTrim(index);
        offset = ADC_GetOffsetTrim(index);
        tmpk2 = (float)(2048 - (2048 - offset) * gain);     /* 2048 is formula parameters */
        g_adcParmList[addrIndex][vrefIndex][shIndex].k1 = gain;
        g_adcParmList[addrIndex][vrefIndex][shIndex].k2 = tmpk2;
    }
}