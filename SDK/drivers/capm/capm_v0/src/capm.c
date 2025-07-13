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
  * @file    capm.c
  * @author  MCU Driver Team.
  * @brief   CAPM HAL level module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the CAPM.
  *           + Initialization and de-initialization functions.
  *           + Get CAPM ECR value and next load ECR number.
  *           + Get CAPM CRT edge.
  *           + Enable/Disable CAPM sync function.
  *           + Get/Set CAPM sync phase(TSR) value.
  *           + Config CAPM interrupt function.
  */
#include "capm.h"
#include "assert.h"
#include "interrupt.h"

/**
  * @brief Config whether the ECR capture event need reset TSR.
  * @param handle: CAPM handle.
  * @param number: ECR number.
  * @retval None.
  */
static inline void CAPM_SetCapReset(CAPM_Handle *handle, unsigned int number)
{
    CAPM_ASSERT_PARAM(handle != NULL);
    CAPM_ASSERT_PARAM(IsCAPMInstance(handle->baseAddress));
    CAPM_PARAM_CHECK_NO_RET(number <= CAPM_MAX_CAP_REG_NUM);
    if (handle->capRegConfig[number].regReset == CAPM_RESET) {
        /* Enable ECR capture event need reset TSR */
        DCL_CAPM_EnableCapReset(handle->baseAddress, number);
    } else {
        /* Disable ECR capture event need reset TSR */
        DCL_CAPM_DisableCapReset(handle->baseAddress, number);
    }
    return;
}

/**
  * @brief Config triggle ECR capture event source.
  * @param handle: CAPM handle.
  * @retval BASE status type: BASE_STATUS_OK, BASE_STATUS_ERROR.
  */
static BASE_StatusType CAPM_SetRegCaptureEvent(CAPM_Handle *handle)
{
    CAPM_ASSERT_PARAM(handle != NULL);
    CAPM_PARAM_CHECK_WITH_RET(handle->useCapNum <= CAPM_MAX_CAP_REG_NUM, BASE_STATUS_ERROR);
    unsigned int i;
    for (i = 0; i < handle->useCapNum; i++) {
        if (handle->capRegConfig[i].capEvent == CAPM_RISING) {    /* CAPM rising capture. */
            DCL_CAPM_RisingCap(handle->baseAddress, i);
            CAPM_SetCapReset(handle, i);
        } else if (handle->capRegConfig[i].capEvent == CAPM_FALLING) {   /* CAPM falling capture. */
            DCL_CAPM_FallingCap(handle->baseAddress, i);
            CAPM_SetCapReset(handle, i);
        } else {
            return BASE_STATUS_ERROR;
        }
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Set deburr number.
  * @param handle: CAPM handle.
  * @retval BASE status type: BASE_STATUS_OK, BASE_STATUS_ERROR.
  */
static BASE_StatusType CAPM_SetDeburrNum(CAPM_Handle *handle)
{
    CAPM_ASSERT_PARAM(handle != NULL);
    if ((handle->deburrNum > 0) && (handle->deburrNum <= CAPM_MAX_FILTER_LEVEL)) {
        DCL_CAPM_EnableFilter(handle->baseAddress);
        DCL_CAPM_SetFilterLevel(handle->baseAddress, handle->deburrNum - 1);
    } else {
        /* deburrNum = 0: Disable filter. */
        DCL_CAPM_DisableFilter(handle->baseAddress);
    }
    return BASE_STATUS_OK;
}

/**
  * @brief IRQ Handler
  * @param handle: CAPM handle.
  * @retval None
  */
void HAL_CAPM_IrqHandler(void *handle)
{
    CAPM_ASSERT_PARAM(handle != NULL);
    CAPM_Handle *useHandle = (CAPM_Handle *)handle;
    CAPM_ASSERT_PARAM(IsCAPMInstance(useHandle->baseAddress));
    if (useHandle->userCallBack.EvtFinishCallback != NULL) {
        /* Get interrupt flag. */
        unsigned int intMask = DCL_CAPM_GetInterFlag(useHandle->baseAddress);
        unsigned int intBit;
        for (unsigned int i = 0; i <= CAPM_INTDMAREQOVF; i++) {
            if (((intMask >> i) & 0x1) == 0x1) {
                intBit = (intMask & (0x1 << i));
                /* Clear interrupt. */
                DCL_CAPM_ClearInter(useHandle->baseAddress, intBit);
                useHandle->userCallBack.EvtFinishCallback(useHandle, i);
            }
        }
    }
    return;
}

/**
  * @brief Register IRQ  callback functions
  * @param capmHandle: CAPM handle.
  * @param typeID: callback function type ID.
  * @param pCallback: pointer of callback function.
  * @retval None
  */
void HAL_CAPM_RegisterCallback(CAPM_Handle *capmHandle, CAPM_CallbackFuncType typeID, CAPM_CallbackType pCallback)
{
    CAPM_ASSERT_PARAM(capmHandle != NULL);
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmHandle->baseAddress));
    CAPM_ASSERT_PARAM(pCallback != NULL);

    switch (typeID) {
        case CAPM_EVT_FINISH:
            capmHandle->userCallBack.EvtFinishCallback = (EvtCallbackType)pCallback;  /**< Event finish callback. */
            break;
        case CAPM_DMA_ERROR:
            capmHandle->userCallBack.DmaErrorCallback = pCallback;      /**< DMA error callback function. */
            break;
        case CAPM_DMA_FINISH:
            capmHandle->userCallBack.DmaFinishCallback = pCallback;     /**< DMA finish callback function. */
        default:
            return;
    }
}

/**
  * @brief DMA error interrupt service routine.
  * @param handle: CAPM handle.
  * @retval None.
  */
static void CAPM_DmaErrorIRQService(void *handle)
{
    CAPM_ASSERT_PARAM(handle != NULL);

    CAPM_Handle *useHandle = (CAPM_Handle *)handle;
    if (useHandle->userCallBack.DmaErrorCallback != NULL) { /* if callback not equal to null */
        useHandle->userCallBack.DmaErrorCallback(useHandle);
    }
    return;
}

/**
  * @brief DMA finish interrupt service routine.
  * @param handle: CAPM handle.
  * @retval None.
  */
static void CAPM_DmaFinishIRQService(void *handle)
{
    CAPM_ASSERT_PARAM(handle != NULL);

    CAPM_Handle *useHandle = (CAPM_Handle *)handle;
    if (useHandle->userCallBack.DmaFinishCallback != NULL) { /* if callback not equal to null */
        useHandle->userCallBack.DmaFinishCallback(useHandle);
    }
    return;
}

/**
  * @brief Get camp number.
  * @param handle: CAPM handle.
  * @retval camp number.
  */
static unsigned char CAPM_GetCapmNumber(CAPM_Handle *capmHandle)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmHandle->baseAddress));
    if (capmHandle->baseAddress == CAPM0_BASE) {
        return CAPM_NUM_0; /* capm0 */
    } else if (capmHandle->baseAddress == CAPM1_BASE) {
        return CAPM_NUM_1; /* capm1 */
    } else if (capmHandle->baseAddress == CAPM2_BASE) {
        return CAPM_NUM_2; /* capm2 */
    } else {
        return CAPM_NUM_0;
    }
}

/**
  * @brief Setting camp sync.
  * @param capmHandle: CAPM handle.
  * @param capmNum: capm number.
  * @retval camp number.
  */
static void CAPM_SyncSetByNumber(CAPM_Handle *capmHandle, unsigned char capmNum)
{
    switch (capmNum) {
        case CAPM_NUM_0:
            DCL_CAPM_EnableSyncIn0(CAPM_COMM); /* enable capm0 sync */
            DCL_CAPM_SetSyncInput0(CAPM_COMM, capmHandle->syncSrc);
            break;
        case CAPM_NUM_1:
            DCL_CAPM_EnableSyncIn1(CAPM_COMM); /* enable capm1 sync */
            DCL_CAPM_SetSyncInput1(CAPM_COMM, capmHandle->syncSrc);
            break;
        case CAPM_NUM_2:
            DCL_CAPM_EnableSyncIn2(CAPM_COMM); /* enable capm2 sync */
            DCL_CAPM_SetSyncInput2(CAPM_COMM, capmHandle->syncSrc);
            break;
        default:
            break;
    }
}

/**
  * @brief Disable  sync by capm number.
  * @param capmNum: CAPM number.
  * @retval None.
  */
static void CAPM_SyncDisableByNumber(unsigned char capmNum)
{
    switch (capmNum) {
        case CAPM_NUM_0:
            DCL_CAPM_DisableSyncIn0(CAPM_COMM); /* disable capm0 sync */
            break;
        case CAPM_NUM_1:
            DCL_CAPM_DisableSyncIn1(CAPM_COMM); /* disable camp1 sync */
            break;
        case CAPM_NUM_2:
            DCL_CAPM_DisableSyncIn2(CAPM_COMM); /* disable capm2 sync */
            break;
        default:
            break;
    }
}

/**
  * @brief Capm sync initialize.
  * @param capmHandle: CAPM handle.
  * @retval None.
  */
static void CAPM_SyncInit(CAPM_Handle *capmHandle)
{
    unsigned char capmNum;
    CAPM_ASSERT_PARAM(capmHandle != NULL);
    capmNum = CAPM_GetCapmNumber(capmHandle);
    if (capmHandle->enableSync == true) { /* if enable sync */
        CAPM_SyncSetByNumber(capmHandle, capmNum);
    } else { /* if do not enable sync */
        CAPM_SyncDisableByNumber(capmNum);
    }
}

/**
  * @brief Capm select input.
  * @param capmHandle: CAPM handle.
  * @retval None.
  */
static BASE_StatusType CAPM_InputSel(CAPM_Handle *capmHandle)
{
    CAPM_ASSERT_PARAM(capmHandle != NULL);
    if (capmHandle->baseAddress == CAPM0_BASE) {
        DCL_CAPM_SetInputSEL0(CAPM_COMM, capmHandle->inputSrc); /* set capm0 input selection */
    } else if (capmHandle->baseAddress == CAPM1_BASE) {
        DCL_CAPM_SetInputSEL1(CAPM_COMM, capmHandle->inputSrc); /* set capm1 input selection */
    } else if (capmHandle->baseAddress == CAPM2_BASE) {
        DCL_CAPM_SetInputSEL2(CAPM_COMM, capmHandle->inputSrc); /* set capm2 input selection */
    } else { /* error value */
        return BASE_STATUS_ERROR;
    }

    return BASE_STATUS_OK;
}

/**
  * @brief CAPM initialize function.
  * @param handle: CAPM handle.
  * @retval BASE status type: BASE_STATUS_OK, BASE_STATUS_ERROR.
  */
BASE_StatusType HAL_CAPM_Init(CAPM_Handle *handle)
{
    CAPM_ASSERT_PARAM(handle != NULL);
    CAPM_ASSERT_PARAM(IsCAPMInstance(handle->baseAddress));
    CAPM_PARAM_CHECK_WITH_RET(handle->useCapNum <= CAPM_MAX_CAP_REG_NUM, BASE_STATUS_ERROR);
    CAPM_PARAM_CHECK_WITH_RET(handle->preScale <= CAPM_MAX_PRESCALE, BASE_STATUS_ERROR);
    /* Init CAPM TSR division. */
    DCL_CAPM_SetTSRDiv(handle->baseAddress, handle->tscntDiv);
    /* Init CAPM capture mode. */
    DCL_CAPM_SetCapMode(handle->baseAddress, handle->capMode);
    DCL_CAPM_SetStopSeq(handle->baseAddress, handle->useCapNum - 1);
    CAPM_SetDeburrNum(handle);
    /* Init CAPM prescale. */
    DCL_CAPM_SetPreScale(handle->baseAddress, handle->preScale);
    DCL_CAPM_SetDMATriggleReg(handle->baseAddress, handle->useCapNum - 1);
    CAPM_SetRegCaptureEvent(handle);
    CAPM_SyncInit(handle);
    if (CAPM_InputSel(handle) == BASE_STATUS_ERROR) {
        return BASE_STATUS_ERROR;
    }
    DCL_CAPM_DisableTSRStop((CAPM_COMM_RegStruct *) CAPM_COMM);
    /* Enable CAPM interrupt. */
    DCL_CAPM_EnableInter(handle->baseAddress, handle->enableIntFlags);
    DCL_CAPM_EnableCapRegLoad(handle->baseAddress);

    return BASE_STATUS_OK;
}

/**
  * @brief CAPM deinitialize function.
  * @param handle: CAPM handle.
  * @retval BASE status type: BASE_STATUS_OK, BASE_STATUS_ERROR.
  */
BASE_StatusType HAL_CAPM_DeInit(CAPM_Handle *handle)
{
    CAPM_ASSERT_PARAM(handle != NULL);
    CAPM_ASSERT_PARAM(IsCAPMInstance(handle->baseAddress));
    /* Clear interrupt callback function. */
    handle->userCallBack.EvtFinishCallback = NULL;
    handle->userCallBack.DmaErrorCallback = NULL;
    handle->userCallBack.DmaFinishCallback = NULL;
 
    /* Clear enable operations. */
    DCL_CAPM_DisableInter(handle->baseAddress, handle->enableIntFlags);
    DCL_CAPM_DisableCapRegLoad(handle->baseAddress);
    return BASE_STATUS_OK;
}

/**
  * @brief Get ECR value.
  * @param handle: CAPM handle.
  * @param ecrNum: ECR number.
  * @retval ECR value.
  */
unsigned int HAL_CAPM_GetECRValue(CAPM_Handle *handle, CAPM_ECRNum ecrNum)
{
    CAPM_ASSERT_PARAM(handle != NULL);
    CAPM_PARAM_CHECK_WITH_RET(ecrNum > 0, BASE_STATUS_ERROR);
    CAPM_PARAM_CHECK_WITH_RET(ecrNum < CAPM_MAX_CAP_REG_NUM, BASE_STATUS_ERROR);
    switch (ecrNum) {
        case CAPM_ECR_NUM1:
            return DCL_CAPM_GetECR1(handle->baseAddress);    /* Get ECR1 value. */
        case CAPM_ECR_NUM2:
            return DCL_CAPM_GetECR2(handle->baseAddress);    /* Get ECR2 value. */
        case CAPM_ECR_NUM3:
            return DCL_CAPM_GetECR3(handle->baseAddress);    /* Get ECR3 value. */
        case CAPM_ECR_NUM4:
            return DCL_CAPM_GetECR4(handle->baseAddress);    /* Get ECR4 value. */
        default:
            return BASE_STATUS_OK;
    }
}

/**
  * @brief Get current signal level.
  * @param handle: CAPM handle.
  * @retval Current signal level: CAPM_LOW_LEVEL, CAPM_UP_EDGE, CAPM_DOWN_EDGE, CAPM_HIGH_LEVEL.
  */
unsigned char HAL_CAPM_GetCrtEdge(CAPM_Handle *handle)
{
    CAPM_ASSERT_PARAM(handle != NULL);
    return DCL_CAPM_GetCRTEdge(handle->baseAddress);
}

/**
  * @brief Get the number of next ECR to be loaded.
  * @param handle: CAPM handle.
  * @retval Next ECR number:NEXT_LOAD_ECR1, NEXT_LOAD_ECR2, NEXT_LOAD_ECR3, NEXT_LOAD_ECR4.
  */
unsigned char HAL_CAPM_GetNextLoadECRNum(CAPM_Handle *handle)
{
    CAPM_ASSERT_PARAM(handle != NULL);
    return DCL_CAPM_GetNextECRNum(handle->baseAddress);
}

/**
  * @brief Set sync phase value.
  * @param handle: CAPM handle.
  * @param phase: Default sync phase value.
  * @retval None.
  */
void HAL_CAPM_SetSyncPhs(CAPM_Handle *handle, unsigned int phase)
{
    CAPM_ASSERT_PARAM(handle != NULL);
    DCL_CAPM_SetSyncPhase(handle->baseAddress, phase);
    return;
}

/**
  * @brief Get sync phase value.
  * @param handle: CAPM handle.
  * @retval Sync phase value.
  */
unsigned int HAL_CAPM_GetSyncPhs(CAPM_Handle *handle)
{
    CAPM_ASSERT_PARAM(handle != NULL);
    return DCL_CAPM_GetSyncPhase(handle->baseAddress);
}

/**
  * @brief Get ECR register value by DMA.
  * @param handle: CAPM handle.
  * @param distAddr: Distance address.
  * @param dataLength: CAPM handle.
  * @retval BASE status type: BASE_STATUS_OK, BASE_STATUS_ERROR.
  */
BASE_StatusType HAL_CAPM_GetECRValueDMA(CAPM_Handle *handle, unsigned int *distAddr,
                                        unsigned int dataLength)
{
    CAPM_ASSERT_PARAM(handle != NULL);
    CAPM_ASSERT_PARAM(handle->dmaHandle != NULL);
    CAPM_ASSERT_PARAM(distAddr != NULL);
    CAPM_PARAM_CHECK_WITH_RET(dataLength > 0, BASE_STATUS_ERROR);
    unsigned int channel;
    channel = handle->dmaChannel;
    if (channel >= CHANNEL_MAX_NUM) {
        return BASE_STATUS_ERROR;
    }
    /* Config DMA callback. */
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[channel].ChannelFinishCallBack = CAPM_DmaFinishIRQService;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[channel].ChannelErrorCallBack = CAPM_DmaErrorIRQService;
    /* Get ECR value by DMA. */
    if (HAL_DMA_StartIT(handle->dmaHandle, (unsigned int)(uintptr_t)(void *)&(handle->baseAddress->ECR1),
        (unsigned int)(uintptr_t)(void *)distAddr, dataLength, channel) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}
