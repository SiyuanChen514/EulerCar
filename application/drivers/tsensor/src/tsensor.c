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
#include "fotp_info_read.h"
#include "anatrim.h"
#include "tsensor.h"

#define NUM 16
#define TSENSOR_SOC_NUM ADC_SOC_NUM15  /* This parameter can be modified according to the actual situation */

/**
  * @brief ADC for tsensor clock initialization.
  * @param None.
  * @retval None.
  */
static void ADC_ClkEnable(void)
{
    unsigned int status = BASE_CFG_UNSET;
    HAL_CRG_IpEnableGet(ADC0_BASE, &status); /* Check whether the ADC clock is enabled */
    if (status != IP_CLK_ENABLE) {
        HAL_CRG_IpEnableSet(ADC0_BASE, IP_CLK_ENABLE);
        HAL_CRG_IpClkSelectSet(ADC0_BASE, CRG_ADC_CLK_ASYN_PLL_DIV);
        HAL_CRG_IpClkDivSet(ADC0_BASE, CRG_ADC_DIV_2);
    }
}

/**
  * @brief ADC for tsensor sample configuration.
  * @param None.
  * @retval None.
  */
static void TSENSOR_SampleConfigure(void)
{
    ADC_Handle adcHandle = {0};
    adcHandle.baseAddress = ADC0;
    adcHandle.socPriority = ADC_PRIMODE_ALL_ROUND;
    HAL_ADC_Init(&adcHandle); /* ADC ADC initialization */

    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINA16;
    socParam.sampleTotalTime = ADC_SOCSAMPLE_5CLK;
    socParam.trigSource = ADC_TRIGSOC_SOFT;
    socParam.continueMode = BASE_CFG_DISABLE;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    unsigned int soc = TSENSOR_SOC_NUM;
    HAL_ADC_ConfigureSoc(&adcHandle, soc, &socParam); /* ADC_CH_ADCINA16 Sampling Configuration */

    TSENSOR_RegStruct *tsensor;
    tsensor = TSENSOR;
    tsensor->TSENSOR_CTRL.BIT.cfg_tsensor_pd = 0x0;
    BASE_FUNC_DELAY_US(40);                          /* waite for 40us until stable */
}

/**
  * @brief ADC Results Converted to Temperature.
  * @param digital digital parameter of tsensor.
  * @retval Temperature type: float, temperature of MCU, unit: ℃.
  */
static float TSENSOR_Conversion(unsigned int digital)
{
    float curV = ((float)digital / 4096.0f) * 3.3f;  /* 4096.0 and 3.3 for voltage conversion */
    /* 1.228f and 25.0f are used as parameters to calculate result */
    float curTemp = (curV - 1.228f) / g_tsensorGain + 25.0f;
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
    tsensor->TSENSOR_CTRL.BIT.cfg_tsensor_pd = 0x1;
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
    for (unsigned int i = 0; i < NUM; i++) {
        unsigned int socRet;
        DCL_ADC_SOCxSoftTrigger(ADC0, TSENSOR_SOC_NUM);
        BASE_FUNC_DELAY_MS(1);                      /* waite for 1ms until conversion finish */
        DCL_ADC_GetConvState(ADC0, TSENSOR_SOC_NUM);
        if (DCL_ADC_GetConvState(ADC0, TSENSOR_SOC_NUM) != BASE_CFG_UNSET) {
            socRet = DCL_ADC_ReadSOCxResult(ADC0, TSENSOR_SOC_NUM);
            ret += socRet;
            count++;
            DCL_ADC_ResetConvState(ADC0, TSENSOR_SOC_NUM); /* Set the sampling completion flag */
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