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
  * @file    tsensor.c
  * @author  MCU Driver Team
  * @brief   tsensor module driver
  * @details This file provides functions to manage tsensor and definition of
  *          specific parameters.
  */

/* Includes ------------------------------------------------------------------*/
#include "crg.h"
#include "adc.h"
#include "adc_tsensor.h"
#include "fotp_info_read.h"
#include "tsensor.h"

#define SAMPLE_MAX 4096
#define NUM 16
#define TSENSOR_SOC_NUM ADC_SOC_NUM15  /* This parameter can be modified according to the actual situation */

/**
  * @brief ADC for tsensor clock initialization.
  * @param None.
  * @retval None.
  */
static void ADC_ClkEnable(void)
{
    HAL_CRG_IpEnableSet(ADCX_TSENSOR_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(ADCX_TSENSOR_BASE, CRG_ADC_CLK_SELECT_PLL_DIV);
    HAL_CRG_IpClkDivSet(ADCX_TSENSOR_BASE, CRG_ADC_DIV_5);
}

/**
  * @brief ADC for tsensor sample configuration.
  * @param None.
  * @retval None.
  */
static void TSENSOR_SampleConfigure(void)
{
    ADC_Handle adcHandle = {0};
    adcHandle.baseAddress = ADCX_TSENSOR;
    adcHandle.socPriority = ADC_PRIMODE_ALL_ROUND;
    adcHandle.handleEx.vrefBuf = ADC_VREF_2P5V;
    HAL_ADC_Init(&adcHandle);

    SOC_Param socParam = {0};
    socParam.adcInput = TSENSOR_SAMPLE_CH;
    socParam.sampleHoldTime = 2;                    /* hold time is set as default value 2 */
    socParam.sampleTotalTime = 127;                 /* charge time is set as default value 127 */
    socParam.finishMode = ADC_SOCFINISH_NONE;
    socParam.softTrigSource = ADC_TRIGSOC_SOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_NONEPERIPH;
    unsigned int soc = TSENSOR_SOC_NUM;
    HAL_ADC_ConfigureSoc(&adcHandle, soc, &socParam);

    TSENSOR_RegStruct *tsensor;
    tsensor = TSENSOR;
    tsensor->TSENSOR_CTRL.BIT.tsen_ana_en = BASE_CFG_ENABLE;
    BASE_FUNC_DELAY_US(50);                          /* waite for 50us until stable */
}

/**
  * @brief ADC Results Converted to Temperature.
  * @param digital digital parameter of tsensor.
  * @retval Temperature type: float, temperature of MCU, unit: ℃.
  */
static float TSENSOR_Conversion(unsigned int digital)
{
    float curV = ((float)digital / 4096.0f) * 3.33333f;           /* 4096.0 and 3.33333 for voltage conversion */
    float curTemp = g_tsensor[0].vrefTemp + (curV - g_tsensor[0].vrefVoltage) / g_tsensor[0].slope;
    return curTemp;
}

/**
  * @brief Configuration of tsensor.
  * @param None.
  * @retval None.
  */
void HAL_TSENSOR_Init(void)
{
    ADC_ClkEnable();
    TSENSOR_SampleConfigure();
}

/**
  * @brief Deinitialize of tsensor.
  * @param None.
  * @retval None.
  */
void HAL_TSENSOR_Deinit(void)
{
    TSENSOR_RegStruct *tsensor;
    tsensor = TSENSOR;
    tsensor->TSENSOR_CTRL.BIT.tsen_ana_en = BASE_CFG_DISABLE;
}


/**
  * @brief Get the result from the tsensor.
  * @param None.
  * @retval result of tsensor.
  */
unsigned int HAL_TSENSOR_GetResult(void)
{
    unsigned int ret = 0;
    unsigned int count = 0;
    ADC_RegStruct *adcAddr = ADCX_TSENSOR;
    for (unsigned int i = 0; i < NUM; i++) {
        unsigned int socRet;
        DCL_ADC_SOCxSoftTrigger(adcAddr, TSENSOR_SOC_NUM);
        BASE_FUNC_DELAY_MS(1);                      /* waite for 1ms until conversion finish */
        if (adcAddr->ADC_EOC_FLAG.BIT.eoc15_flag == BASE_CFG_ENABLE) {
            socRet = DCL_ADC_ReadSOCxResult(adcAddr, TSENSOR_SOC_NUM);
            ret += socRet;
            count++;
        }
    }
    if (count == 0) {
        return 0xFFF;
    }
    return (ret / count);  /* Average the results */
}

/**
  * @brief Get the temperature from the tsensor.
  * @param None.
  * @retval Temperature type: float, temperature of MCU, unit: ℃.
  */
float HAL_TSENSOR_GetTemperature(void)
{
    unsigned int result = HAL_TSENSOR_GetResult();
    float temp = TSENSOR_Conversion(result);
    return temp;
}