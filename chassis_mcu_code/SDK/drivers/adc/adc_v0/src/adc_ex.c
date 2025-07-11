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
  * @file    adc_ex.c
  * @author  MCU Driver Team
  * @brief   ADC module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the ADC.
  *           + ADC Synchronous Sampling.
  *           + ADC Software Calibration.
  */

/* Includes ------------------------------------------------------------------*/

#include "adcinit.h"
#include "adc_ex.h"

/**
  * @brief Enable ADC synchronous sample.Use one ADC with two different SOCs, configure the same trigger source, and
  * sample both inputs simultaneously. When used, the inputs in group A need to be configured with one SOC, and
  * the inputs in group B need to be configured with another SOC.
  * @param adcHandle ADC handle.
  * @param syncParam Param struct of synchronous sample.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_ADC_StartSyncSampleEx(ADC_Handle *adcHandle, SOC_SyncParam *syncParam)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_ASSERT_PARAM(syncParam != NULL);
    ADC_PARAM_CHECK_WITH_RET(IsADCGroupAChannel(syncParam->ChannelA) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCGroupBChannel(syncParam->ChannelB) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCChargeTime(syncParam->sampleTotalTime) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCHodeTime(syncParam->sampleHoldTime) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCSoftTrig(syncParam->softTrigSource) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCPeriphTrig(syncParam->periphTrigSource) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCIntTrig(syncParam->intTrigSource) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCFinishMode(syncParam->finishMode) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCSyncGroup(syncParam->group) == true, BASE_STATUS_ERROR);
    unsigned int socA = 0;
    unsigned int socB = 0;
    unsigned int tmp;
    SOC_Param param;                /* Configuring SOC Parameters */
    param.adcInput = syncParam->ChannelA;
    param.sampleHoldTime = syncParam->sampleHoldTime;
    param.sampleTotalTime = syncParam->sampleTotalTime;
    param.finishMode = ADC_SOCFINISH_NONE;
    param.softTrigSource = syncParam->softTrigSource;
    param.intTrigSource = syncParam->intTrigSource;
    param.periphTrigSource = syncParam->periphTrigSource;
    for (unsigned int i = 0; i < SYNCGROUP_NUM; i++) {
        tmp = (1 << i);
        if (tmp & syncParam->group) {
            socA = 2 * i;           /* 2 for converting from group number to SOC number */
            socB = 2 * i + 1;       /* 2 for converting from group number to SOC number */
            break;
        }
    }
    DCL_ADC_SetSyncSample(adcHandle->baseAddress, syncParam->group);
    /* Group A does not report DMA and interrupts by default. Group B reports DMA and interrupts after sampling */
    HAL_ADC_ConfigureSoc(adcHandle, socA, &param); /* Configuring group A input */
    param.finishMode = syncParam->finishMode;
    param.adcInput = syncParam->ChannelB;
    HAL_ADC_ConfigureSoc(adcHandle, socB, &param); /* Configuring group B input */
    if (syncParam->finishMode >= ADC_SOCFINISH_INT1) {
        HAL_ADC_StartIt(adcHandle);  /* Configuring Interrupts During Synchronous Sampling */
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Obtains the sample result after SOC conversion.
  * @param adcx ADC Base Pointer.
  * @param soc ID of SOC.
  * @param originalRet Sampling original data.
  * @retval unsigned int, Calibrated data.
  */
unsigned int HAL_ADC_ActiveCalibrateRetEx(ADC_RegStruct * const adcx, unsigned int soc, unsigned int originalRet)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(soc) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(originalRet <= 0x1000, BASE_STATUS_ERROR);
    uintptr_t adcAddr = (uintptr_t)(void *)adcx;
    unsigned int addrIndex = (adcAddr & 0x3000) >> 12;  /* 0x3000 and 12 are used to convert addresses to index */
    unsigned int socNum = DCL_ADC_GetSOCxInputChannel(adcx, soc);
    unsigned int vrefIndex = adcx->ADC_ANA_CTRL.BIT.adc_ana_gsh0;
    unsigned int shIndex = ((socNum & 0x18) == 0) ? 0 : 1;  /* 0x18 is used to convert sh0 or sh1 to index */
    float k1 = g_adcParmList[addrIndex][vrefIndex][shIndex].k1;
    float k2 = g_adcParmList[addrIndex][vrefIndex][shIndex].k2;
    int result = (int)((float)originalRet * k1 + k2);
    if (result < 0) {
        return 0;        /* Limit the range of results after calibration */
    } else if ((unsigned int)result > 0xFFF) {
        return 0xFFF;    /* Limit the range of results after calibration */
    }
    return (unsigned int)result;
}