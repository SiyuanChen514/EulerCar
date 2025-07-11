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
  * @details This file provides firmware functions to manage the following extend function.
  *           + ADC Oversampling Function Configuration and Usage Definition.
  *           + ADC PPB Function Configuration and Usage Definition.
  */

/* Includes ------------------------------------------------------------------*/

#include "adc_ex.h"

/**
  * @brief Enable SOC for continuous conversion.
  * @param adcHandle ADC handle.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_ADC_EnableSocCotinueModeEx(ADC_Handle *adcHandle, ADC_SOCNumber soc)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(soc) == true, BASE_STATUS_ERROR);
    DCL_ADC_EnableSOCxContinue(adcHandle->baseAddress, soc); /* Enable continuous conversion */
    return BASE_STATUS_OK;
}

/**
  * @brief Disable SOC for continuous conversion.
  * @param adcHandle ADC handle.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_ADC_DisableSocCotinueModeEx(ADC_Handle *adcHandle, ADC_SOCNumber soc)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(soc) == true, BASE_STATUS_ERROR);
    DCL_ADC_DisableSOCxContinue(adcHandle->baseAddress, soc); /* Disbale continuous conversion */
    return BASE_STATUS_OK;
}

/**
  * @brief Obtaining ADC Controller Status.
  * @param adcHandle ADC handle.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_ADC_GetControllerStatusEx(ADC_Handle *adcHandle)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    unsigned int status = adcHandle->baseAddress->ADC_STATUS.reg;
    return (status == 0 ? BASE_STATUS_OK : BASE_STATUS_BUSY);
}

/**
  * @brief Obtaining ADC Controller Status.
  * @param adcHandle ADC handle.
  * @retval BASE status type. OK, SOC has completed, ERROR, SOC does not complete.
  */
BASE_StatusType HAL_ADC_CheckOversamplingFinishEx(ADC_Handle *adcHandle)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    if (DCL_ADC_GetOversamplingState(adcHandle->baseAddress) == 0) {
        return BASE_STATUS_ERROR;       /* The SOC does not complete the conversion */
    }
    DCL_ADC_ResetOversamplingState(adcHandle->baseAddress);  /* Clear flag bit */
    return BASE_STATUS_OK;
}

/**
  * @brief Obtaining ADC Controller Status.
  * @param adcHandle ADC handle.
  * @retval unsigned int, result of ADC oversampling result.
  */
unsigned int HAL_ADC_GetOversamplingResultEx(ADC_Handle *adcHandle)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    return DCL_ADC_ReadOversamplingResult(adcHandle->baseAddress);
}

/**
  * @brief Configuring ADC Oversampling Parameters.
  * @param adcHandle ADC handle.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @param param Configure param of oversampling.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_ADC_ConfigureOversamplingEx(ADC_Handle *adcHandle, ADC_SOCNumber soc, ADC_OversamplingParam *param)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(param != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(soc) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCOversamplingMultiple(param->multiple), BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCOversamplingRightShift(param->rightShift), BASE_STATUS_ERROR);
    unsigned int accuracy = 12 + param->multiple - param->rightShift; /* ADC default sampling precision is 12 bits */
    if (accuracy < 12 || accuracy > 16) {   /* oversampling effective accuracy: 12 ~ 16 bits */
        return BASE_STATUS_ERROR;
    }
    unsigned int value = 1;
    value |= ((unsigned int)soc << 4U);                  /* Shift left 4 bits to configure the soc */
    value |= ((unsigned int)param->multiple << 8U);      /* Shift left 8 bits to configure the multiple */
    value |= ((unsigned int)param->rightShift << 12U);   /* Shift left 12 bits to configure the rightShift */
    adcHandle->baseAddress->ADC_OVERSAMP.reg = value;
    if (param->oversamplingInt == true) {
        DCL_ADC_EnableOversamplingInt(adcHandle->baseAddress);
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Configuring the Working Mode of the ADC Controller.
  * @param adcHandle ADC handle.
  * @param mode Work mode of ADC.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_ADC_ConfigureWorkModeEx(ADC_Handle *adcHandle, ADC_WorkMode mode)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_PARAM_CHECK_WITH_RET(IsADCWorkMode(mode) == true, BASE_STATUS_ERROR);
    adcHandle->baseAddress->ADC_MODE.reg = mode; /* Configuring the Working Mode */
    return BASE_STATUS_OK;
}

/**
  * @brief Enable PPB interrupt.
  * @param adcHandle ADC handle.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_ADC_EnablePPBxEventIntEx(ADC_Handle *adcHandle)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    adcHandle->baseAddress->ADC_EVENT_INT_MASK.reg |= 0xFFFF;
    return BASE_STATUS_OK;
}

/**
  * @brief Disbale PPB interrupt.
  * @param adcHandle ADC handle.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_ADC_DisablePPBxEventIntEx(ADC_Handle *adcHandle)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    unsigned int value = adcHandle->baseAddress->ADC_EVENT_INT_MASK.reg;
    value = (value & 0xFFFF0000);   /* The lower 16 bits of the PPB event interrupt is disabled. */
    adcHandle->baseAddress->ADC_EVENT_INT_MASK.reg = value;
    return BASE_STATUS_OK;
}

/**
  * @brief Event interrupt callback function registration interface.
  * @param adcHandle ADC handle.
  * @param soc ID of SOC.
  * @param ppb Number of PPB.
  * @param fun PPB function configuration.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_ADC_ConfigurePPBxEx(ADC_Handle *adcHandle, ADC_SOCNumber soc, ADC_PPBNumber ppb, PPB_Function *fun)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(soc), BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCPostProcessingBlock(ppb), BASE_STATUS_ERROR);
    ADC_ASSERT_PARAM(fun != NULL);
    DCL_ADC_SOCxSelectPPBx(adcHandle->baseAddress, ppb, soc);   /* Selecting SOC that needs to use the PPB function */
    DCL_ADC_SetPPBxFunction(adcHandle->baseAddress, ppb, fun);  /* Configuring the PPB Function */
    return BASE_STATUS_OK;
}

/**
  * @brief Set the upper and down thresholds.
  * @param adcHandle ADC handle.
  * @param ppb Number of PPB.
  * @param up Upper threshold.
  * @param dn Down threshold.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_ADC_SetPPBxThresholdEx(ADC_Handle *adcHandle, ADC_PPBNumber ppb, int up, int dn)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_PARAM_CHECK_WITH_RET(IsADCPostProcessingBlock(ppb), BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(-4096 <= up && up <= 4095, BASE_STATUS_ERROR); /* Threshold Range: -4096 ~ 4095 */
    ADC_PARAM_CHECK_WITH_RET(-4096 <= dn && dn <= 4095, BASE_STATUS_ERROR); /* Threshold Range: -4096 ~ 4095 */
    if (up < dn) {
        return BASE_STATUS_ERROR;
    }
    unsigned int upTemp = (0x1FFF & (unsigned int)up); /* The lower 13 bits are valid */
    unsigned int dnTemp = (0x1FFF & (unsigned int)dn); /* The lower 13 bits are valid */
    DCL_ADC_SetPPBxThreshold(adcHandle->baseAddress, ppb, upTemp, dnTemp);
    return BASE_STATUS_OK;
}

/**
  * @brief Set the compensation offset.
  * @param adcHandle ADC handle.
  * @param ppb Number of PPB.
  * @param offset Offset compensation value.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_ADC_SetPPBxOffsetEx(ADC_Handle *adcHandle, ADC_PPBNumber ppb, int offset)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_PARAM_CHECK_WITH_RET(IsADCPostProcessingBlock(ppb), BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(-2048 <= offset && offset <= 2047, BASE_STATUS_ERROR); /* Offset Range: -2048 ~ 2047 */
    unsigned int temp = (0xFFF & (unsigned int)offset); /* The lower 12 bits are valid */
    DCL_ADC_SetPPBxOffset(adcHandle->baseAddress, ppb, temp);
    return BASE_STATUS_OK;
}

/**
  * @brief Setting the Error reference value.
  * @param adcHandle ADC handle.
  * @param ppb Number of PPB.
  * @param ref Error reference value.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_ADC_SetPPBxErrorRefEx(ADC_Handle *adcHandle, ADC_PPBNumber ppb, unsigned int ref)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_PARAM_CHECK_WITH_RET(IsADCPostProcessingBlock(ppb), BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(ref <= 0xFFF, BASE_STATUS_ERROR);
    DCL_ADC_SetPPBxErrorRef(adcHandle->baseAddress, ppb, ref); /* Setting the Error reference value */
    return BASE_STATUS_OK;
}

/**
  * @brief Get the error calculation result.
  * @param adcHandle ADC handle.
  * @param ppb Number of PPB.
  * @retval unsigned int, Error Result.
  */
int HAL_ADC_GetPPBxErrorResultEx(ADC_Handle *adcHandle, ADC_PPBNumber ppb)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_PARAM_CHECK_WITH_RET(IsADCPostProcessingBlock(ppb), BASE_STATUS_ERROR);
    unsigned int ret = DCL_ADC_GetPPBxErrorResult(adcHandle->baseAddress, ppb);
    if ((ret & 0x1000) == 0x1000) { /* Check whether the value is negative */
        ret |= 0xFFFFE000;
        ret = ~(ret - 1);
        return (0 - (int)ret);
    }
    return (int)ret;
}

/**
  * @brief Get the error calculation result.
  * @param adcHandle ADC handle.
  * @param ppb Number of PPB.
  * @retval unsigned int, delay count value. The unit is the system frequency period.
  */
unsigned int HAL_ADC_GetPPBxDelayCntEx(ADC_Handle *adcHandle, ADC_PPBNumber ppb)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_PARAM_CHECK_WITH_RET(IsADCPostProcessingBlock(ppb), BASE_STATUS_ERROR);
    return DCL_ADC_GetPPBxDelayCnt(adcHandle->baseAddress, ppb);
}