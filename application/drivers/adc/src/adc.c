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
  * @file    adc.c
  * @author  MCU Driver Team
  * @brief   ADC module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the ADC.
  *           + ADC initialization function.
  *           + Start ADC sample and conversion.
  *           + Start ADC sample and conversion with interrupt.
  *           + Start ADC sample and conversion with DMA.
  *           + Query the ADC conversion result.
  *           + Single and multichannel software trigger functions.
  *           + Interrupt callback function and user registration function.
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/**
  * @brief Initialize the ADC hardware controller.After the controller is initialized, the ADC sampling is
  * triggered at least 100 us later.
  * @param adcHandle ADC handle.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_ADC_Init(ADC_Handle *adcHandle)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_PARAM_CHECK_WITH_RET(IsADCPriorityMode(adcHandle->socPriority) == true, BASE_STATUS_ERROR);
    DCL_ADC_SOCxSetPriority(adcHandle->baseAddress, adcHandle->socPriority);
    adcHandle->baseAddress->ADC_ANA_CTRL0.BIT.cfg_sar_samp_cap_sel = 0x4; /* Set the Number of Sampling Capacitors */
    adcHandle->baseAddress->ADC_EN.reg = BASE_CFG_ENABLE; /* Enable ADC Controller */
    BASE_FUNC_DelayUs(100);  /* Wait for 100 us until the ADC controller is stable */
    return BASE_STATUS_OK;
}

/**
  * @brief DeInitialize the ADC hardware controller.
  * @param adcHandle ADC handle.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_ADC_Deinit(ADC_Handle *adcHandle)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    adcHandle->baseAddress->ADC_EN.reg = BASE_CFG_DISABLE;
    return BASE_STATUS_OK;
}

/**
  * @brief configurating the specified SOC parameters.
  * @param adcHandle ADC handle.
  * @param soc ID of SOC(start of conversion), managing the specific sample inputs.
  * @param socParam Param struct of SOC. This is related to the peripheral circuit design, @ref SOC_Param.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_ADC_ConfigureSoc(ADC_Handle *adcHandle, ADC_SOCNumber soc, SOC_Param *socParam)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(soc) == true, BASE_STATUS_ERROR);
    ADC_ASSERT_PARAM(socParam != NULL);
    ADC_PARAM_CHECK_WITH_RET(IsADCSampleChannel(socParam->adcInput) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCTotalTime(socParam->sampleTotalTime) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCTrigSource(socParam->trigSource) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCFinishMode(socParam->finishMode) == true, BASE_STATUS_ERROR);
    DCL_ADC_SOCxSelcetChannel(adcHandle->baseAddress, soc, socParam->adcInput);      /* Set channel */
    DCL_ADC_SOCxSetAcqps(adcHandle->baseAddress, soc, socParam->sampleTotalTime);    /* Set sampling time */
    DCL_ADC_SOCxSelcetTrigSource(adcHandle->baseAddress, soc, socParam->trigSource); /* Set trigger source */
    if (socParam->continueMode == true) { /* Continuous Mode Judgment */
        DCL_ADC_EnableSOCxContinue(adcHandle->baseAddress, soc);
    } else {
        DCL_ADC_DisableSOCxContinue(adcHandle->baseAddress, soc);
    }
    adcHandle->ADC_SOCxParam[soc].finishMode = socParam->finishMode;
    return BASE_STATUS_OK;
}

/**
  * @brief Callback function that ADC completes the sample conversion and uses the DMA to complete the transmission.
  * @param handle ADC handle.
  * @retval None.
  */
static void ADC_DMATransFinish(void *handle)
{
    ADC_ASSERT_PARAM(handle != NULL);
    ADC_Handle *adcHandle = (ADC_Handle *)(handle);
    if (adcHandle->userCallBack.DmaFinishCallBack != NULL) {
        adcHandle->userCallBack.DmaFinishCallBack(adcHandle); /* Callback User Registration Function */
    }
    return;
}

/**
  * @brief Callback function that ADC falis to use DMA.
  * @param handle ADC handle.
  * @retval None.
  */
static void ADC_DMATransError(void *handle)
{
    ADC_ASSERT_PARAM(handle != NULL);
    ADC_Handle *adcHandle = (ADC_Handle *)(handle);
    if (adcHandle->userCallBack.DmaErrorCallBack != NULL) {
        adcHandle->userCallBack.DmaErrorCallBack(adcHandle); /* Callback User Registration Function */
    }
    return;
}

/**
  * @brief Start the ADC conversion and enable ADC DMA. After the SOC conversion using the DMA is complete, use the DMA
  * to transfer data The DMA can transfer the sampling results of consecutive SOCs. The start and end of DMA transfer
  * are determined by startSoc and endSoc.
  * @param adcHandle ADC handle.
  * @param startSoc First SOC result for DMA transfer.
  * @param endSoc Last SOC result for DMA transfer.
  * @param saveData Address where the converted result is saved.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_ADC_StartDma(ADC_Handle *adcHandle, unsigned int startSoc,
                                 unsigned int endSoc, unsigned int *saveData)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(startSoc) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(endSoc) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(startSoc <= endSoc, BASE_STATUS_ERROR);
    ADC_ASSERT_PARAM(saveData != NULL);
    ADC_ASSERT_PARAM(adcHandle->dmaHandle != NULL);
    ADC_PARAM_CHECK_WITH_RET(IsDmaChannelNum(adcHandle->adcDmaChn) == true, BASE_STATUS_ERROR);
    unsigned int dmaSOCx = 0;
    unsigned int dataLength = endSoc - startSoc + 1;
    for (int i = 0; i < SOC_MAX_NUM; i++) { /* The DMA request is generated by the last SOC */
        if (adcHandle->ADC_SOCxParam[i].finishMode == ADC_SOCFINISH_DMA) {
            dmaSOCx = i;
        }
    }
    DCL_ADC_DMARequestSource(adcHandle->baseAddress, dmaSOCx); /* Enable the DMA function of the ADC */
    DCL_ADC_EnableDMABurstReq(adcHandle->baseAddress);         /* Enable the DMA burst request */
    DCL_ADC_EnableDMASingleReq(adcHandle->baseAddress);        /* Enable the DMA single request */
    uintptr_t srcAddr = (uintptr_t)(void *)(adcHandle->baseAddress);
    srcAddr = srcAddr + 4 * startSoc;   /* The base address difference of adjacent SOC result registers is 4 */
    adcHandle->dmaHandle->userCallBack.DMA_CallbackFuns[adcHandle->adcDmaChn].ChannelFinishCallBack =
        ADC_DMATransFinish;
    adcHandle->dmaHandle->userCallBack.DMA_CallbackFuns[adcHandle->adcDmaChn].ChannelErrorCallBack = ADC_DMATransError;
    BASE_StatusType ret = HAL_DMA_StartIT(adcHandle->dmaHandle, srcAddr, (uintptr_t)(void *)(saveData),
                                          dataLength, adcHandle->adcDmaChn);
    return ret;
}

/**
  * @brief Start the ADC conversion and enable ADC interrupt. After the SOC completes sample conversion, the ADC
  * interrupt is reported.
  * @param adcHandle ADC handle.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_ADC_StartIt(ADC_Handle *adcHandle)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    unsigned int intVal = 0;
    for (int i = 0; i < SOC_MAX_NUM; i++) {
        intVal = adcHandle->ADC_SOCxParam[i].finishMode;
        switch (intVal) {
            case ADC_SOCFINISH_INT0:
                DCL_ADC_SetSOCxBlindInt0(adcHandle->baseAddress, i); /* The SOC selects to use interrupt 0 */
                break;
            case ADC_SOCFINISH_INT1:
                DCL_ADC_SetSOCxBlindInt1(adcHandle->baseAddress, i); /* The SOC selects to use interrupt 1 */
                break;
            case ADC_SOCFINISH_INT2:
                DCL_ADC_SetSOCxBlindInt2(adcHandle->baseAddress, i); /* The SOC selects to use interrupt 2 */
                break;
            case ADC_SOCFINISH_INT3:
                DCL_ADC_SetSOCxBlindInt3(adcHandle->baseAddress, i); /* The SOC selects to use interrupt 3 */
                break;
            default:
                break;
        }
    } /* Enable ADC Interrupt */
    DCL_ADC_EnableIntx(adcHandle->baseAddress, ADC_INT_NUMBER0);
    DCL_ADC_EnableIntx(adcHandle->baseAddress, ADC_INT_NUMBER1);
    DCL_ADC_EnableIntx(adcHandle->baseAddress, ADC_INT_NUMBER2);
    DCL_ADC_EnableIntx(adcHandle->baseAddress, ADC_INT_NUMBER3);
    return BASE_STATUS_OK;
}

/**
  * @brief The software triggers multiple SCOs for sampling at the same time.
  * @param adcHandle ADC handle.
  * @param syncTrig Triggering Parameters. The lower 16 bits correspond to one SOC. If this bit is set to 1, the
  * software triggers the SOC.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_ADC_SoftTrigMultiSample(ADC_Handle *adcHandle, ADC_SoftMultiTrig syncTrig)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    unsigned int val = syncTrig.softTrigVal;
    ADC_PARAM_CHECK_WITH_RET(val <= 0xFFFF, BASE_STATUS_ERROR);
    DCL_ADC_SOCxMultiSoftTrigger(adcHandle->baseAddress, val); /* Software triggering for multiple SOC */
    return BASE_STATUS_OK;
}

/**
  * @brief The software triggers only one soc.
  * @param adcHandle ADC handle.
  * @param soc ID of SOC.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_ADC_SoftTrigSample(ADC_Handle *adcHandle, unsigned int soc)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(soc) == true, BASE_STATUS_ERROR);
    DCL_ADC_SOCxSoftTrigger(adcHandle->baseAddress, soc); /* Software triggers a single SOC */
    return BASE_STATUS_OK;
}

/**
  * @brief Obtains the sample result after SOC conversion.
  * @param adcHandle ADC handle.
  * @param soc ID of SOC.
  * @retval BASE status type: OK, ERROR.
  */
unsigned int HAL_ADC_GetConvResult(ADC_Handle *adcHandle, unsigned int soc)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(soc) == true, BASE_STATUS_ERROR);
    return DCL_ADC_ReadSOCxResult(adcHandle->baseAddress, soc);
}


/**
  * @brief Check the SOC completion flag.
  * @param adcHandle ADC handle.
  * @param soc ID of SOC.
  * @retval BASE_STATUS_ERROR: The SOC does not complete the data sampling conversion.
  * @retval BASE_STATUS_OK: The SOC has completed data sampling conversion.
  */
BASE_StatusType HAL_ADC_CheckSocFinish(ADC_Handle *adcHandle, unsigned int soc)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(soc) == true, BASE_STATUS_ERROR);
    if (DCL_ADC_GetConvState(adcHandle->baseAddress, soc) == 0) {
        return BASE_STATUS_ERROR;       /* The SOC does not complete the conversion */
    }
    DCL_ADC_ResetConvState(adcHandle->baseAddress, soc);  /* Clear flag bit */
    return BASE_STATUS_OK;
}

/**
  * @brief The ADC completes the interrupt processing.
  * @param adcHandle ADC handle.
  * @param intx ADC interrupt type number @ref ADC_IntNumber.
  * @retval None.
  */
static void ADC_IntxClearEoc(ADC_Handle *adcHandle, unsigned int intx)
{
    unsigned int eocFlag = adcHandle->baseAddress->ADC_EOC_FLAG.reg;
    ADC_INT_DATA_0_REG intData0;
    ADC_INT_DATA_1_REG intData1;
    unsigned int eocMask = 0;
    switch (intx) {
        case ADC_INT_NUMBER0:   /* Read Interrupt Configuration */
            intData0.reg = adcHandle->baseAddress->ADC_INT_DATA_0.reg;
            eocMask = intData0.BIT.cfg_intr_data_sel0;
            break;
        case ADC_INT_NUMBER1:   /* Read Interrupt Configuration */
            intData0.reg = adcHandle->baseAddress->ADC_INT_DATA_0.reg;
            eocMask = intData0.BIT.cfg_intr_data_sel1;
            break;
        case ADC_INT_NUMBER2:   /* Read Interrupt Configuration */
            intData1.reg = adcHandle->baseAddress->ADC_INT_DATA_1.reg;
            eocMask = intData1.BIT.cfg_intr_data_sel2;
            break;
        case ADC_INT_NUMBER3:   /* Read Interrupt Configuration */
            intData1.reg = adcHandle->baseAddress->ADC_INT_DATA_1.reg;
            eocMask = intData1.BIT.cfg_intr_data_sel3;
            break;
        default:
            break;
    }
    unsigned int eoc = eocFlag & eocMask;
    adcHandle->ADC_IntxParam[intx].socxFinish = eoc;
    for (int i = 0; i < SOC_MAX_NUM; i++) {
        unsigned int val = (1 << i);
        if (eoc & val) {
            adcHandle->baseAddress->ADC_EOC_FLAG.reg = val;  /* Clear the EOC flag */
        }
    }
}

/**
  * @brief The ADC overflow interrupt processing
  * @param adcHandle ADC handle.
  * @param overFlag Overflow interrupt flag bit.
  * @retval None.
  */
void HAL_ADC_IrqHandlerOver(void *handle)
{
    ADC_ASSERT_PARAM(handle != NULL);
    ADC_Handle *adcHandle = (ADC_Handle *)handle;
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    unsigned int overState = adcHandle->baseAddress->ADC_ERR_INT_MSK.reg;
    unsigned int overFlag = adcHandle->baseAddress->ADC_ERR_INT.reg;
    adcHandle->baseAddress->ADC_ERR_INT.reg = overFlag;
    adcHandle->overState.trigOver = (overState & 0xFFFF);   /* Save trigger overflow status */
    adcHandle->overState.dmaReqOver = (overState & 0x10000); /* Save dma request overflow status */
    if (adcHandle->userCallBack.TrigOverCallBack != NULL  && adcHandle->overState.trigOver != 0) {
        adcHandle->userCallBack.TrigOverCallBack(handle); /* Callback User Registration Function */
    }
    if (adcHandle->userCallBack.DmaOverCallBack != NULL  && adcHandle->overState.dmaReqOver != 0) {
        adcHandle->userCallBack.DmaOverCallBack(handle); /* Callback User Registration Function */
    }
}

/**
  * @brief ADC Interrupt0 service processing function.
  * @param handle ADC handle.
  * @retval None.
  */
void HAL_ADC_IrqHandlerInt0(void *handle)
{
    ADC_ASSERT_PARAM(handle != NULL);
    ADC_Handle *adcHandle = (ADC_Handle *)handle;
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_IntxClearEoc(adcHandle, ADC_INT_NUMBER0); /* Clear conversion completion flag */
    DCL_ADC_ClearIntx(adcHandle->baseAddress, ADC_INT_NUMBER0);
    if (adcHandle->userCallBack.Int0FinishCallBack != NULL) {
        adcHandle->userCallBack.Int0FinishCallBack(handle);
    }
}

/**
  * @brief ADC Interrupt1 service processing function.
  * @param handle ADC handle.
  * @retval None.
  */
void HAL_ADC_IrqHandlerInt1(void *handle)
{
    ADC_ASSERT_PARAM(handle != NULL);
    ADC_Handle *adcHandle = (ADC_Handle *)handle;
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_IntxClearEoc(adcHandle, ADC_INT_NUMBER1);  /* Clear conversion completion flag */
    DCL_ADC_ClearIntx(adcHandle->baseAddress, ADC_INT_NUMBER1);
    if (adcHandle->userCallBack.Int1FinishCallBack != NULL) {
        adcHandle->userCallBack.Int1FinishCallBack(handle);
    }
}

/**
  * @brief ADC Interrupt2 service processing function.
  * @param handle ADC handle.
  * @retval None.
  */
void HAL_ADC_IrqHandlerInt2(void *handle)
{
    ADC_ASSERT_PARAM(handle != NULL);
    ADC_Handle *adcHandle = (ADC_Handle *)handle;
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_IntxClearEoc(adcHandle, ADC_INT_NUMBER2);  /* Clear conversion completion flag */
    DCL_ADC_ClearIntx(adcHandle->baseAddress, ADC_INT_NUMBER2);
    if (adcHandle->userCallBack.Int2FinishCallBack != NULL) {
        adcHandle->userCallBack.Int2FinishCallBack(handle);
    }
}

/**
  * @brief ADC Interrupt3 service processing function.
  * @param handle ADC handle.
  * @retval None.
  */
void HAL_ADC_IrqHandlerInt3(void *handle)
{
    ADC_ASSERT_PARAM(handle != NULL);
    ADC_Handle *adcHandle = (ADC_Handle *)handle;
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_IntxClearEoc(adcHandle, ADC_INT_NUMBER3);  /* Clear conversion completion flag */
    DCL_ADC_ClearIntx(adcHandle->baseAddress, ADC_INT_NUMBER3);
    if (adcHandle->userCallBack.Int3FinishCallBack != NULL) {
        adcHandle->userCallBack.Int3FinishCallBack(handle);
    }
}

/**
  * @brief Event interrupt callback processing.
  * @param adcHandle ADC handle.
  * @param ppb Work mode of ADC.
  * @param eventStatus Status of the event interrupt.
  * @retval None.
  */
static void ADC_EventCallBack(ADC_Handle *handle, unsigned int ppb, unsigned int eventStatus)
{
    for (unsigned int i = 0; i < 4; ++i) {  /* Each PPB has 4 interrupt types */
        unsigned int index = 4 * ppb + i;   /* Each PPB has 4 interrupt types */
        if (handle->userCallBack.PPBEventCallBack[index] != NULL && (eventStatus & (1U << index)) != 0) {
            handle->userCallBack.PPBEventCallBack[index](handle);
        }
    }
}

/**
  * @brief ADC extended interrupt service processing function.
  * @param handle Event handle.
  * @retval None.
  */
void HAL_ADC_IrqHandlerAllEvent(void *handle)
{
    ADC_ASSERT_PARAM(handle != NULL);
    ADC_Handle *adcHandle = (ADC_Handle *)handle;
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    unsigned int eventStatus = adcHandle->baseAddress->ADC_EVENT_INT_MSK.reg;
    if (adcHandle->baseAddress->ADC_EVENT_INT_MSK.BIT.intr_oversamp_data_vld_msk == BASE_CFG_ENABLE) {
        adcHandle->baseAddress->ADC_EVENT_INT.BIT.intr_oversamp_data_vld = BASE_CFG_SET;
        if (adcHandle->userCallBack.OverSamplingFinishCallBack != NULL) {
            adcHandle->userCallBack.OverSamplingFinishCallBack(handle); /* Oversampling callback function */
        }
        return;
    }
    for (unsigned int i = 0; i < 4; ++i) {       /* Each ADC has 4 PPB  */
        unsigned int tmp = 0xF << (4U * i);      /* Each PPB has 4 interrupt types */
        if ((tmp & eventStatus) != 0) {
            adcHandle->baseAddress->ADC_EVENT_INT.reg = tmp;
            ADC_EventCallBack(handle, i, eventStatus);  /* PPB event callback function */
            break;
        }
    }
    return;
}

/**
  * @brief User callback function registration interface for event type.
  * @param adcHandle ADC handle.
  * @param typeID Id of callback function type.
  * @param pCallback Pointer of the specified callbcak function.
  * @retval None.
  */
static void ADC_RegieterEventCallBack(ADC_Handle *adcHandle, ADC_CallbackFunType typeID, ADC_CallbackType pCallback)
{
    if (typeID > ADC_CALLBACK_EVENT_PPB3_ERROR || typeID < ADC_CALLBACK_EVENT_PPB0_ZERO) {
        return;
    }
    unsigned int index = ((unsigned int)typeID & 0xF);
    adcHandle->userCallBack.PPBEventCallBack[index] = pCallback;
}

/**
  * @brief User callback function registration interface.
  * @param adcHandle ADC handle.
  * @param typeID Id of callback function type.
  * @param pCallback Pointer of the specified callbcak function.
  * @retval None.
  */
void HAL_ADC_RegisterCallBack(ADC_Handle *adcHandle, ADC_CallbackFunType typeID, ADC_CallbackType pCallback)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(pCallback != NULL);
    switch (typeID) {   /* Register the callback function based on the interrupt type */
        case ADC_CALLBACK_INT0:
            adcHandle->userCallBack.Int0FinishCallBack = pCallback; /* Sampling finsish interrupt 0 callback function */
            break;
        case ADC_CALLBACK_INT1:
            adcHandle->userCallBack.Int1FinishCallBack = pCallback; /* Sampling finsish interrupt 1 callback function */
            break;
        case ADC_CALLBACK_INT2:
            adcHandle->userCallBack.Int2FinishCallBack = pCallback; /* Sampling finsish interrupt 2 callback function */
            break;
        case ADC_CALLBACK_INT3:
            adcHandle->userCallBack.Int3FinishCallBack = pCallback; /* Sampling finsish interrupt 3 callback function */
            break;
        case ADC_CALLBACK_DMA:
            adcHandle->userCallBack.DmaFinishCallBack = pCallback;  /* Dma transfer finish callback function */
            break;
        case ADC_CALLBACK_DMAERROR:
            adcHandle->userCallBack.DmaErrorCallBack = pCallback;   /* Dma transfer error callback function */
            break;
        case ADC_CALLBACK_DMAOVER:
            adcHandle->userCallBack.DmaOverCallBack = pCallback;   /* Dma request over callback function */
            break;
        case ADC_CALLBACK_TRIGOVER:
            adcHandle->userCallBack.TrigOverCallBack = pCallback;  /* trigger over callback function */
            break;
        case ADC_CALLBACK_EVENT_OVERSAMPLING:                      /* Oversampling callback function */
            adcHandle->userCallBack.OverSamplingFinishCallBack = pCallback;
            break;
        default:
            ADC_RegieterEventCallBack(adcHandle, typeID, pCallback); /* PPB Function Callback Function */
            break;
    }
}
