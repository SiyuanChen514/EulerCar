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
  * @file      crc.c
  * @author    MCU Driver Team
  * @brief     CRC module driver
  * @details   This file provides firmware functions to manage the following functionalities of the GPIO.
  *             + Initialization functions.
  *             + CRC Set And Get Functions.
  *             + Interrupt Handler Functions.
  */

/* Includes ------------------------------------------------------------------*/
#include "interrupt.h"
#include "crc.h"

#define WORD_DIV_BYTE_SIZE      4
#define WORD_DIV_DOUBLE_SIZE    2

#define OFFSET_ONE_BYTE       1
#define OFFSET_TWO_BYTE       2
#define OFFSET_THREE_BYTE     3

#define BIT_SHIFT24    24
#define BIT_SHIFT16    16
#define BIT_SHIFT8     8

#define REMAINDER_SIZE_ONE    1
#define REMAINDER_SIZE_TWO    2
#define REMAINDER_SIZE_THREE  3
#define REMAINDER_RANGE_THREE  3
#define REMAINDER_RANGE_ONE    1

static void CRC_Handle_8(CRC_Handle *handle, const unsigned char *pData, unsigned int length);
static void CRC_Handle_16(CRC_Handle *handle, const unsigned short *pData, unsigned int length);
static void CRC_Handle_32(CRC_Handle *handle, const unsigned int *pData, unsigned int length);
static void CRC_SetPolynomialModeByAlgorithm(CRC_RegStruct *crcx, CRC_AlgorithmMode algorithmMode);
static void CRC_SetInitValueByAlgorithm(CRC_RegStruct *crcx, CRC_AlgorithmMode algorithmMode);
static void CRC_SetPolynomialMode(CRC_Handle *handle, CRC_PolynomialMode polynomialMode);
static void CRC_SetInitValue(CRC_Handle *handle, CRC_InitValueType type);

/**
  * @brief Initializing CRC register values.
  * @param handle Value of @ref CRC_Handle.
  * @retval BASE_StatusType: OK, ERROR.
  */
BASE_StatusType HAL_CRC_Init(CRC_Handle *handle)
{
    CRC_ASSERT_PARAM(handle != NULL);
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    CRC_PARAM_CHECK_WITH_RET(IsCrcInputDataFormat(handle->inputDataFormat), BASE_STATUS_ERROR);
    DCL_CRC_SoftReset(handle->baseAddress);
    DCL_CRC_SetByteMode(handle->baseAddress, handle->inputDataFormat);
    /* Set the polynomial and initial value by algorithm mode */
    if (IsCrcAlgorithm(handle->handleEx.algoMode)) {
        CRC_SetInitValueByAlgorithm(handle->baseAddress, handle->handleEx.algoMode);
        CRC_SetPolynomialModeByAlgorithm(handle->baseAddress, handle->handleEx.algoMode);
    } else { /* Set the polynomial and initial value by algorithm attribute */
        CRC_SetPolynomialMode(handle, handle->polyMode);
        CRC_SetInitValue(handle, handle->initValueType);
    }
    DCL_CRC_SetTimeOut(handle->baseAddress, handle->handleEx.timeout);
    /* enable CRC interrupt */
    handle->baseAddress->CRC_CTRL_INTMASK.BIT.pready_timeout_int_mask = handle->handleEx.enableIT ? \
                                                                        BASE_CFG_UNSET : BASE_CFG_SET;
    /* enable CRC error inject */
    handle->baseAddress->CRC_CTRL_ERRINJ.BIT.time_out_err_inj = handle->handleEx.enableErrInject ? \
                                                                    BASE_CFG_SET : BASE_CFG_UNSET;
    return BASE_STATUS_OK;
}

/**
  * @brief DeInitializing CRC register values.
  * @param handle Value of @ref CRC_Handle.
  * @retval None.
  */
void HAL_CRC_DeInit(CRC_Handle *handle)
{
    CRC_ASSERT_PARAM(handle != NULL);
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    /* Reset CRC calculation data */
    handle->baseAddress->crc_data_in = 0x00000000;
    /* Reset CRC init value */
    handle->baseAddress->CRC_CTRL_CFG1.BIT.crc_init = BASE_CFG_SET; /* Reset CRC init value. */
    handle->baseAddress->CRC_CTRL_CFG2.BIT.crc_ld = BASE_CFG_SET; /* load CRC init value. */
    handle->userCallBack.PreadyTimeoutCallback = NULL; /* Clean callback */
}

/**
  * @brief Set CRC input data and get CRC output.
  * @param handle Value of @ref CRC_Handle.
  * @param data CRC input data.
  * @retval unsigned int CRC output data.
  */
unsigned int HAL_CRC_SetInputDataGetCheck(CRC_Handle *handle, unsigned int data)
{
    CRC_ASSERT_PARAM(handle != NULL);
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    handle->baseAddress->crc_data_in = data; /* Set CRC input data */
    DCL_CRC_WaitComplete(handle->baseAddress); /* wait crc cpmplete */
    return handle->baseAddress->crc_out;
}

/**
  * @brief Compute the 8, 16 or 32-bit CRC value of an 8, 16 or
           32-bit data buffer starting with the previously computed CRC as initialization value.
  * @param handle Value of @ref CRC_Handle.
  * @param pData Pointer to the input data buffer.
  * @param length pData array length.
  * @retval unsigned int CRC output data.
  */
unsigned int HAL_CRC_Accumulate(CRC_Handle *handle, const void *pData, unsigned int length)
{
    CRC_ASSERT_PARAM(handle != NULL);
    CRC_ASSERT_PARAM(pData != NULL);
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    switch (handle->inputDataFormat) {
        case CRC_MODE_BIT8:
            CRC_Handle_8(handle, (unsigned char *)pData, length); /* Input register to compute 8-bit data value */
            break;
        case CRC_MODE_BIT16:
            CRC_Handle_16(handle, (unsigned short *)pData, length); /* Input register to compute 16-bit data value */
            break;
        case CRC_MODE_BIT32:
            CRC_Handle_32(handle, (unsigned int *)pData, length); /* Input register to compute 32-bit data value */
            break;
        default:
            break;
    }
    DCL_CRC_WaitComplete(handle->baseAddress);
    return handle->baseAddress->crc_out;
}

/**
  * @brief Compute the 8, 16 or 32-bit CRC value of an 8, 16 or
           32-bit data buffer starting with default initialization value.
  * @param handle Value of @ref CRC_Handle.
  * @param pData Pointer to the input data buffer.
  * @param length pData array length.
  * @retval unsigned int CRC output data.
  */
unsigned int HAL_CRC_Calculate(CRC_Handle *handle, const void *pData, unsigned int length)
{
    CRC_ASSERT_PARAM(handle != NULL);
    CRC_ASSERT_PARAM(pData != NULL);
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    DCL_CRC_SoftReset(handle->baseAddress); /* Reset CRC init value. */
    DCL_CRC_LoadInitValue(handle->baseAddress); /* load CRC init value. */
    return HAL_CRC_Accumulate(handle, pData, length);
}

/**
  * @brief Compute the 8-bit input data to the CRC calculator.
  * @param handle Value of @ref CRC_Handle.
  * @param pData Pointer to the input data buffer.
  * @param length pData array length.
  * @retval unsigned int CRC output data.
  */
static void CRC_Handle_8(CRC_Handle *handle, const unsigned char *pData, unsigned int length)
{
    CRC_ASSERT_PARAM(handle != NULL);
    CRC_ASSERT_PARAM(pData != NULL);
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    unsigned int i = 0;
    for (i = 0; i < (length / WORD_DIV_BYTE_SIZE); i++) {
        DCL_CRC_SetByteMode(handle->baseAddress, CRC_MODE_BIT32);
        /* Data concatenation */
        handle->baseAddress->crc_data_in = \
            ((unsigned int)pData[WORD_DIV_BYTE_SIZE * i] << BIT_SHIFT24) | \
            ((unsigned int)pData[WORD_DIV_BYTE_SIZE * i + OFFSET_ONE_BYTE] << BIT_SHIFT16) | \
            ((unsigned int)pData[WORD_DIV_BYTE_SIZE * i + OFFSET_TWO_BYTE] << BIT_SHIFT8) | \
             (unsigned int)pData[WORD_DIV_BYTE_SIZE * i + OFFSET_THREE_BYTE];
    }
    if ((length & REMAINDER_RANGE_THREE) != 0) {
        if ((length & REMAINDER_RANGE_THREE) == REMAINDER_SIZE_ONE) { /* remainder : 1 */
            DCL_CRC_SetByteMode(handle->baseAddress, CRC_MODE_BIT8);
            handle->baseAddress->crc_data_in = pData[WORD_DIV_BYTE_SIZE * i];
        }
        if ((length & REMAINDER_RANGE_THREE) == REMAINDER_SIZE_TWO) { /* remainder : 2 */
            DCL_CRC_SetByteMode(handle->baseAddress, CRC_MODE_BIT16);
            /* Data concatenation */
            handle->baseAddress->crc_data_in = \
                ((unsigned int)pData[WORD_DIV_BYTE_SIZE * i] << BIT_SHIFT8) | \
                 (unsigned int)pData[WORD_DIV_BYTE_SIZE * i + OFFSET_ONE_BYTE];
        }
        if ((length & REMAINDER_RANGE_THREE) == REMAINDER_SIZE_THREE) { /* remainder : 3 */
            DCL_CRC_SetByteMode(handle->baseAddress, CRC_MODE_BIT24);
            /* Data concatenation */
            handle->baseAddress->crc_data_in = \
                ((unsigned int)pData[WORD_DIV_BYTE_SIZE * i] << BIT_SHIFT16) | \
                ((unsigned int)pData[WORD_DIV_BYTE_SIZE * i + OFFSET_ONE_BYTE] << BIT_SHIFT8) | \
                 (unsigned int)pData[WORD_DIV_BYTE_SIZE * i + OFFSET_TWO_BYTE];
        }
    }
    DCL_CRC_SetByteMode(handle->baseAddress, CRC_MODE_BIT8);
}

/**
  * @brief Compute the 16-bit input data to the CRC calculator.
  * @param handle Value of @ref CRC_Handle.
  * @param pData Pointer to the input data buffer.
  * @param length pData array length.
  * @retval unsigned int CRC output data.
  */
static void CRC_Handle_16(CRC_Handle *handle, const unsigned short *pData, unsigned int length)
{
    CRC_ASSERT_PARAM(handle != NULL);
    CRC_ASSERT_PARAM(pData != NULL);
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    unsigned int i = 0;
    for (i = 0; i < (length / WORD_DIV_DOUBLE_SIZE); i++) {
        DCL_CRC_SetByteMode(handle->baseAddress, CRC_MODE_BIT32);
        /* Data concatenation */
        handle->baseAddress->crc_data_in = \
            ((unsigned int)pData[WORD_DIV_DOUBLE_SIZE * i] << BIT_SHIFT16) | \
             (unsigned int)pData[WORD_DIV_DOUBLE_SIZE * i + OFFSET_ONE_BYTE];
    }
    if ((length & REMAINDER_RANGE_ONE) != 0) { /* remainder bot equal zero */
        DCL_CRC_SetByteMode(handle->baseAddress, CRC_MODE_BIT16);
        handle->baseAddress->crc_data_in = (unsigned int)pData[WORD_DIV_DOUBLE_SIZE * i];
    }
    DCL_CRC_SetByteMode(handle->baseAddress, CRC_MODE_BIT16);
}

/**
  * @brief Compute the 32-bit input data to the CRC calculator.
  * @param handle Value of @ref CRC_Handle.
  * @param pData Pointer to the input data buffer.
  * @param length pData array length.
  * @retval unsigned int CRC output data.
  */
static void CRC_Handle_32(CRC_Handle *handle, const unsigned int *pData, unsigned int length)
{
    CRC_ASSERT_PARAM(handle != NULL);
    CRC_ASSERT_PARAM(pData != NULL);
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    for (unsigned int i = 0; i < length; i++) {
        handle->baseAddress->crc_data_in = pData[i];   /* input crc data */
    }
}

/**
  * @brief Check whether the received data CRC value is the same as the expected value.
  * @param handle Value of @ref CRC_Handle.
  * @param pData Pointer to the input data buffer,
                 exact input data byte mode is provided by handle->inputDataFormat.
  * @param length pData array length.
  * @param crcValue CRC check value.
  * @retval unsigned int CRC check result
  */
bool HAL_CRC_CheckInputData(CRC_Handle *handle, const void *pData, unsigned int length, unsigned int crcValue)
{
    CRC_ASSERT_PARAM(handle != NULL);
    CRC_ASSERT_PARAM(pData != NULL);
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    return (HAL_CRC_Calculate(handle, pData, length) == crcValue);
}

/**
  * @brief Set CRC check_in data to register.
  * @param handle Value of @ref CRC_Handle.
  * @retval None.
  */
void HAL_CRC_SetCheckInData(CRC_Handle *handle, unsigned int data)
{
    CRC_ASSERT_PARAM(handle != NULL);
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    handle->baseAddress->crc_check_in = data;
}

/**
  * @brief Load CRC check_in register data to crc_out register.
  * @param handle Value of @ref CRC_Handle.
  * @retval unsigned int Reversed check_in data.
  */
unsigned int HAL_CRC_LoadCheckInData(CRC_Handle *handle)
{
    CRC_ASSERT_PARAM(handle != NULL);
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    handle->baseAddress->CRC_CTRL_CFG2.BIT.crc_ld = BASE_CFG_SET;
    return handle->baseAddress->crc_out;
}

/**
  * @brief   Register CRC interrupt callback.
  * @param   handle Value of @ref CRC_handle.
  * @param   callBackFunc Value of @ref CRC_CallbackType.
  * @retval  None
  */
void HAL_CRC_RegisterCallback(CRC_Handle *handle, CRC_CallbackType callBackFunc)
{
    CRC_ASSERT_PARAM(handle != NULL);
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    handle->userCallBack.PreadyTimeoutCallback = callBackFunc;
}

/**
  * @brief Interrupt handler processing function.
  * @param handle CRC_Handle.
  * @retval None.
  */
void HAL_CRC_IrqHandler(void *handle)
{
    CRC_Handle *crcHandle = (CRC_Handle *)handle;
    CRC_ASSERT_PARAM(crcHandle != NULL);
    CRC_ASSERT_PARAM(IsCRCInstance(crcHandle->baseAddress));

    if (crcHandle->baseAddress->CRC_CTRL_INT.BIT.pready_timeout == 0x01) {
        crcHandle->baseAddress->CRC_CTRL_INT.BIT.pready_timeout = BASE_CFG_SET; /* Clear pready_timeout IRQ */
        if (crcHandle->userCallBack.PreadyTimeoutCallback) {
            crcHandle->userCallBack.PreadyTimeoutCallback(crcHandle);
        }
    }
}

/**
  * @brief Set crc polynomial mode by algorithm.
  * @param crcx Value of @ref CRC_Handle.
  * @param algorithmMode CRC algorithm mode.
  * @retval None.
  */
static void CRC_SetPolynomialModeByAlgorithm(CRC_RegStruct *crcx, CRC_AlgorithmMode algorithmMode)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    CRC_PARAM_CHECK_NO_RET(algorithmMode < CRC_ALG_MODE_MAX);
    /* Matching enumerated values with register configuration values */
    unsigned int polyMode = CRC_POLY_MODE_MAX;
    if (algorithmMode == CRC8_ROHC) {
        polyMode = CRC8_07_POLY_MODE;
    } else if (algorithmMode == CRC16_XMODEM || algorithmMode == CRC16_CCITT_FALSE) {
        polyMode = CRC16_1021_POLY_MODE;
    } else if (algorithmMode == CRC16_IBM || algorithmMode == CRC16_MODBUS) {
        polyMode = CRC16_8005_POLY_MODE;
    } else if (algorithmMode == CRC32) {
        polyMode = CRC32_04C11D87_POLY_MODE;
    }
    /* config register */
    DCL_CRC_SetPolynomialMode(crcx, polyMode);
}

/**
  * @brief Set CRC init value by algorithmMode.
  * @param crc Value of @ref CRC_RegStruct.
  * @param algorithmMode value of CRC algorithm.
  * @retval None.
  */
static void CRC_SetInitValueByAlgorithm(CRC_RegStruct *crcx, CRC_AlgorithmMode algorithmMode)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcx));
    CRC_PARAM_CHECK_NO_RET(algorithmMode < CRC_ALG_MODE_MAX && algorithmMode >= CRC8_ROHC);
    switch (algorithmMode) {
        case CRC8_ROHC:
            /* CRC8_ROHC init value is FF */
            *(unsigned char *)(void *)(&crcx->crc_check_in) = CRC_INIT_VALUE_FF;
            break;
        case CRC16_IBM:
            *(unsigned short *)(void *)(&crcx->crc_check_in) = CRC_INIT_VALUE_0000;
            break;
        case CRC16_MODBUS:
            /* CRC16_MODBUS init value is FFFF */
            *(unsigned short *)(void *)(&crcx->crc_check_in) = CRC_INIT_VALUE_FFFF;
            break;
        case CRC16_CCITT_FALSE:
            *(unsigned short *)(void *)(&crcx->crc_check_in) = CRC_INIT_VALUE_FFFF;
            break;
        case CRC16_XMODEM:
            /* CRC16_XMODEM init value is 0000 */
            *(unsigned short *)(void *)(&crcx->crc_check_in) = CRC_INIT_VALUE_0000;
            break;
        case CRC32:
            *(unsigned int *)(void *)(&crcx->crc_check_in) = CRC_INIT_VALUE_FFFFFFFF;
            break;
        default:
            break;
    }
    DCL_CRC_LoadInitValue(crcx); /* load init value */
}

/**
  * @brief Set CRC Polynomial Mode by algorithmMode.
  * @param handle Value of @ref CRC_Handle.
  * @param polynomialMode CRC_PolynomialMode.
  * @retval None.
  */
static void CRC_SetPolynomialMode(CRC_Handle *handle, CRC_PolynomialMode polynomialMode)
{
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    CRC_PARAM_CHECK_NO_RET(IsCrcPolynomial(polynomialMode));
    DCL_CRC_SetPolynomialMode(handle->baseAddress, polynomialMode);
}

/**
  * @brief Set CRC init value.
  * @param handle Value of @ref CRC_Handle.
  * @param type Value of @ref CRC_InitValueType.
  * @retval None.
  */
static void CRC_SetInitValue(CRC_Handle *handle, CRC_InitValueType type)
{
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    CRC_PARAM_CHECK_NO_RET(IsCrcInitValueType(type));
    switch (type) {
        case TYPE_CRC_INIT_VALUE_FF: /* init value type 00 */
            *(unsigned char *)(void *)(&handle->baseAddress->crc_check_in) = CRC_INIT_VALUE_FF;
            break;
        case TYPE_CRC_INIT_VALUE_0000: /* init value type 0000 */
            *(unsigned short *)(void *)(&handle->baseAddress->crc_check_in) = CRC_INIT_VALUE_0000;
            break;
        case TYPE_CRC_INIT_VALUE_FFFF:
            *(unsigned short *)(void *)(&handle->baseAddress->crc_check_in) = CRC_INIT_VALUE_FFFF;
            break;
        case TYPE_CRC_INIT_VALUE_FFFFFFFF: /* init value type FFFFFFFF */
            *(unsigned int *)(void *)(&handle->baseAddress->crc_check_in) = CRC_INIT_VALUE_FFFFFFFF;
            break;
        default:
            break;
    }
}