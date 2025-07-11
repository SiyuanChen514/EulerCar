/**
  * @ Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2022-2023. All rights reserved.
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
  * @file      mcs_adcCalibr.c
  * @author    MCU Algorithm Team
  * @brief     This file provides adc bias calibration function. 0722-7
  */
#include "mcs_adcCalibr.h"
#include "mcs_assert.h"


/**
  * @brief Get ADC result when ADC conversion completes.
  * @param adcCalibr ADC calibration value.
  * @param soc ID of SOC.
  * @retval None.
  */
static unsigned int ADCCALIBR_GetSocResult(ADC_Handle *adcHandle, unsigned int soc)
{
    MCS_ASSERT_PARAM(adcHandle != NULL);
    /* wait for ADC conversion complete */
    while (1) {
        /* Check ADC conversion if completes. */
        if (HAL_ADC_CheckSocFinish(adcHandle, soc) == BASE_STATUS_OK) {
            break;
        }
    }
    return DCL_ADC_ReadSOCxResult(adcHandle->baseAddress, soc);
}

/**
  * @brief ADC calibration initialization.
  * @param adcCalibr ADC calibration value.
  * @retval None.
  */
void ADCCALIBR_Init(ADC_CALIBR_Handle *adcCalibr)
{
    MCS_ASSERT_PARAM(adcCalibr != NULL);
    adcCalibr->adcShiftAccu = 0;
    adcCalibr->cnt = 0;
    adcCalibr->state = ADC_CALIBR_NOT_FINISH;
}

/**
  * @brief Compute current sampling adc offset.
  * @param adcCalibr ADC calibration handle.
  * @param adcHandle ADC handle.
  * @param soc ID of SOC.
  * @retval offset val.
  */
unsigned int ADCCALIBR_Exec(ADC_CALIBR_Handle *adcCalibr, ADC_Handle *adcHandle, unsigned int soc)
{
    MCS_ASSERT_PARAM(adcCalibr != NULL);
    MCS_ASSERT_PARAM(adcHandle != NULL);
    MCS_ASSERT_PARAM(adcCalibr->adcShiftAccu <= 4096 * ADC_CNT_POINTS); /* 4096: 12-bit adc precision. */
    adcCalibr->cnt++;
    /* sum of 50 points value */
    if (adcCalibr->cnt > ADC_CNT_POINTS) {
        adcCalibr->cnt = ADC_CNT_POINTS;
        adcCalibr->state = ADC_CALIBR_FINISH;
        return (unsigned int)((float)(adcCalibr->adcShiftAccu) / (float)ADC_CNT_POINTS);
    } else {
        adcCalibr->adcShiftAccu += ADCCALIBR_GetSocResult(adcHandle, soc);
        adcCalibr->state = ADC_CALIBR_NOT_FINISH;
        return 0;
    }
    /* Returns the offset sampling average */
    return 0;
}

/**
  * @brief Returns the motor control status based on whether the calibration is complete or not.
  * @param adcCalibr ADC calibration handle.
  * @retval bool.
  */
bool ADCCALIBR_IsFinish(ADC_CALIBR_Handle *adcCalibr)
{
    MCS_ASSERT_PARAM(adcCalibr != NULL);
    if (adcCalibr->state == ADC_CALIBR_FINISH) {
        return true;
    }

    return false;
}