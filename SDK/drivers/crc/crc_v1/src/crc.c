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

/* Includes ------------------------------------------------------------------ */
#include "interrupt.h"
#include "crc.h"

#define WORD_DIV_BYTE_SIZE 4
#define WORD_DIV_DOUBLE_SIZE 2

#define OFFSET_ONE_BYTE 1
#define OFFSET_TWO_BYTE 2
#define OFFSET_THREE_BYTE 3

#define BIT_SHIFT24 24
#define BIT_SHIFT16 16
#define BIT_SHIFT8 8

#define REMAINDER_SIZE_ONE 1
#define REMAINDER_SIZE_TWO 2
#define REMAINDER_SIZE_THREE 3
#define REMAINDER_RANGE_THREE 3
#define REMAINDER_RANGE_ONE 1

#define CRC8_MODE_07_REG_VALUE 0
#define CRC16_MODE_8005_REG_VALUE 2
#define CRC16_MODE_1021_REG_VALUE 3
#define CRC32_MODE_04C11D87_REG_VALUE 4

static void CRC_Handle_8(CRC_Handle *handle, const unsigned char *pData, unsigned int length);
static void CRC_Handle_16(CRC_Handle *handle, const unsigned short *pData, unsigned int length);
static void CRC_Handle_32(CRC_Handle *handle, const unsigned int *pData, unsigned int length);
static void CRC_SetPolynomialModeByAlgorithm(CRC_Handle *handle, CRC_AlgorithmMode algorithmMode);
static void CRC_SetXorEndianReverseEnableByAlgorithm(CRC_Handle *handle, CRC_AlgorithmMode algorithmMode);
static void CRC_SetXorValueByAlgorithm(CRC_Handle *handle, CRC_AlgorithmMode algorithmMode);
static void CRC_SetInitValueByAlgorithm(CRC_Handle *handle, CRC_AlgorithmMode algorithmMode);
static void CRC_SetResultXorValue(CRC_Handle *handle, CRC_ResultXorValueType type);
static void CRC_SetInitValue(CRC_Handle *handle, CRC_InitValueType type);

/**
  * @brief Initializing CRC register values.
  * @param handle Value of @ref CRC_Handle.
  * @retval BASE_StatusType BASE Status.
  */
BASE_StatusType HAL_CRC_Init(CRC_Handle *handle)
{
    /* PARAM check */
    CRC_ASSERT_PARAM(handle != NULL);
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    CRC_PARAM_CHECK_WITH_RET(IsCrcInputDataFormat(handle->inputDataFormat), BASE_STATUS_ERROR);
    DCL_CRC_SoftReset(handle->baseAddress);
    if (IsCrcAlgorithm(handle->handleEx.algoMode)) {
        /* Algorithm Mode Parameter Configuration */
        CRC_SetPolynomialModeByAlgorithm(handle, handle->handleEx.algoMode);
        CRC_SetXorEndianReverseEnableByAlgorithm(handle, handle->handleEx.algoMode);
        CRC_SetXorValueByAlgorithm(handle, handle->handleEx.algoMode);
        CRC_SetInitValueByAlgorithm(handle, handle->handleEx.algoMode);
    } else {
        /* CRC PARAM check */
        CRC_PARAM_CHECK_WITH_RET(IsCrcPolynomial(handle->polyMode), BASE_STATUS_ERROR);
        CRC_PARAM_CHECK_WITH_RET(IsCrcInitValueType(handle->initValueType), BASE_STATUS_ERROR);
        CRC_PARAM_CHECK_WITH_RET(IsCrcResultXorValueType(handle->resultXorValueType), BASE_STATUS_ERROR);
        CRC_PARAM_CHECK_WITH_RET(IsCrcReverseEnableType(handle->reverseEnableType), BASE_STATUS_ERROR);
        CRC_PARAM_CHECK_WITH_RET(IsCrcXorEndianEnableType(handle->xorEndianEnbaleType), BASE_STATUS_ERROR);
        unsigned int polyMode = handle->polyMode; /* algorithmic polynomial mode */
        bool inputEndianMode = ((handle->xorEndianEnbaleType & TYPE_ENDIAN_MSB_BIT) == TYPE_ENDIAN_MSB_BIT);
        bool inputByteReverse =
            ((handle->reverseEnableType & TYPE_BYTE_REVERSE_ENABLE_BIT) == TYPE_BYTE_REVERSE_ENABLE_BIT);
        bool outputReverse =
            ((handle->reverseEnableType & TYPE_OUTPUT_REVERSE_ENABLE_BIT) == TYPE_OUTPUT_REVERSE_ENABLE_BIT);
        bool resultXorEnable = ((handle->xorEndianEnbaleType & TYPE_XOR_ENABLE_BIT) == TYPE_XOR_ENABLE_BIT);
        /* DCL CRC set parameters */
        DCL_CRC_SetPolynomialMode(handle->baseAddress, polyMode);
        DCL_CRC_SetEndianMode(handle->baseAddress, inputEndianMode);
        DCL_CRC_SetByteReverseMode(handle->baseAddress, inputByteReverse);
        DCL_CRC_SetOutputReverseMode(handle->baseAddress, outputReverse);
        DCL_CRC_SetXorResultMode(handle->baseAddress, resultXorEnable);
        /* Extended Interface Parameter Settings */
        if (handle->baseAddress->CRC_POST_CFG.BIT.crc_post_xor_enable == BASE_CFG_ENABLE) {
            CRC_SetResultXorValue(handle, handle->resultXorValueType);
        }
        CRC_SetInitValue(handle, handle->initValueType);
        DCL_CRC_LoadInitValue(handle->baseAddress);
    }
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
    DCL_CRC_SoftReset(handle->baseAddress);
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
    DCL_CRC_SetInputData32(handle->baseAddress, data); /* Set CRC input data */
    return DCL_CRC_GetOutputData(handle->baseAddress);
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
    return DCL_CRC_GetOutputData(handle->baseAddress);
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
    DCL_CRC_LoadInitValue(handle->baseAddress); /* load init value */
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
    volatile unsigned char *crcData8 = (unsigned char *)(void *)(&handle->baseAddress->crc_data_in);
    for (unsigned int i = 0; i < length; i++) {
        *(crcData8) = pData[i]; /* input crc data */
    }
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
    volatile unsigned short *crcData16 = (unsigned short *)(void *)(&handle->baseAddress->crc_data_in);
    for (unsigned int i = 0; i < length; i++) {
        *(crcData16) = pData[i]; /* input crc data */
    }
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
    volatile unsigned int *crcData32 = (unsigned int *)(void *)(&handle->baseAddress->crc_data_in);
    for (unsigned int i = 0; i < length; i++) {
        *(crcData32) = pData[i]; /* input crc data */
    }
}

/**
  * @brief Check whether the recived data CRC value is the same as the expected value.
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
    handle->baseAddress->crc_calc_init_value = data;
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
    DCL_CRC_LoadInitValue(handle->baseAddress);
    return DCL_CRC_GetInitValue(handle->baseAddress);
}

/**
  * @brief  Register CRC interrupt callback.
  * @param  handle Value of @ref CRC_handle.
  * @param  callBackFunc Value of @ref CRC_CallbackType.
  * @retval None
  */
void HAL_CRC_RegisterCallback(CRC_Handle *handle, CRC_CallbackType callBackFunc)
{
    BASE_FUNC_UNUSED(handle);
    BASE_FUNC_UNUSED(callBackFunc);
}

/**
  * @brief Interrupt handler processing function.
  * @param handle CRC_Handle.
  * @retval None.
  */
void HAL_CRC_IrqHandler(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    return;
}

/**
  * @brief Set CRC init value by algorithmMode.
  * @param handle Value of @ref CRC_Handle.
  * @param algorithmMode value of CRC algorithm.
  * @retval None.
  */
static void CRC_SetInitValueByAlgorithm(CRC_Handle *handle, CRC_AlgorithmMode algorithmMode)
{
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    /* 0x000000FF : MASK of initValueType in crc algorithm */
    unsigned int initValueType = (algorithmMode & TYPE_INIT_MASK);
    CRC_SetInitValue(handle, initValueType);
    DCL_CRC_LoadInitValue(handle->baseAddress);
}


/**
  * @brief Set CRC xor value by algorithmMode.
  * @param handle Value of @ref CRC_Handle.
  * @param algorithmMode value of CRC algorithm.
  * @retval None.
  */
static void CRC_SetXorValueByAlgorithm(CRC_Handle *handle, CRC_AlgorithmMode algorithmMode)
{
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    unsigned int xorValueType = (algorithmMode & TYPE_XOR_VALUE_MASK);
    if (handle->baseAddress->CRC_POST_CFG.BIT.crc_post_xor_enable == BASE_CFG_ENABLE) {
        /* Setting result xor value */
        CRC_SetResultXorValue(handle, xorValueType);
    }
}

/**
  * @brief Set CRC xor endian reverse enable type by algorithmMode.
  * @param handle Value of @ref CRC_Handle.
  * @param algorithmMode value of CRC algorithm.
  * @retval None.
  */
static void CRC_SetXorEndianReverseEnableByAlgorithm(CRC_Handle *handle, CRC_AlgorithmMode algorithmMode)
{
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    /* 0x000000FF : MASK of initValueType in crc algorithm */
    unsigned int xorEndianEnableType = (algorithmMode & TYPE_XOR_ENDIAN_ENABLE_MASK);
    unsigned int reverseEnableType = (algorithmMode & TYPE_REVERSE_ENABLE_MASK);
    if ((IsCrcXorEndianEnableType(xorEndianEnableType))) {
        /* config register */
        DCL_CRC_SetEndianMode(handle->baseAddress,
            ((xorEndianEnableType & TYPE_ENDIAN_MSB_BIT) == TYPE_ENDIAN_MSB_BIT));
        DCL_CRC_SetXorResultMode(handle->baseAddress,
            ((xorEndianEnableType & TYPE_XOR_ENABLE_BIT) == TYPE_XOR_ENABLE_BIT));
    }
    if (IsCrcReverseEnableType(reverseEnableType)) {
        /* config register */
        DCL_CRC_SetByteReverseMode(handle->baseAddress,
            ((reverseEnableType & TYPE_BYTE_REVERSE_ENABLE_BIT) == TYPE_BYTE_REVERSE_ENABLE_BIT));
        DCL_CRC_SetOutputReverseMode(handle->baseAddress,
            ((reverseEnableType & TYPE_OUTPUT_REVERSE_ENABLE_BIT) == TYPE_OUTPUT_REVERSE_ENABLE_BIT));
    }
}

/**
  * @brief Set CRC Polynomial Mode by algorithmMode.
  * @param handle Value of @ref CRC_Handle.
  * @param algorithmMode value of CRC algorithm.
  * @retval None.
  */
static void CRC_SetPolynomialModeByAlgorithm(CRC_Handle *handle, CRC_AlgorithmMode algorithmMode)
{
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    unsigned int polynomialMode = (algorithmMode & TYPE_POLY_MASK);
    if (IsCrcPolynomial(polynomialMode)) {
        DCL_CRC_SetPolynomialMode(handle->baseAddress, polynomialMode);
    }
}

/**
  * @brief Set CRC xor value mode.
  * @param handle Value of @ref CRC_Handle.
  * @param type Value of @ref CRC_ResultXorValueType
  * @retval None.
  */
static void CRC_SetResultXorValue(CRC_Handle *handle, CRC_ResultXorValueType type)
{
    CRC_ASSERT_PARAM(IsCRCInstance(handle->baseAddress));
    CRC_PARAM_CHECK_NO_RET(IsCrcResultXorValueType(type));
    switch (type) {
        case TYPE_CRC_XOR_VALUE_00: /* xor value tyep 00 */
            *(unsigned char *)(void *)(&handle->baseAddress->crc_post_xor_value) = CRC_XOR_VALUE_00;
            break;
        case TYPE_CRC_XOR_VALUE_55:
            *(unsigned char *)(void *)(&handle->baseAddress->crc_post_xor_value) = CRC_XOR_VALUE_55;
            break;
        case TYPE_CRC_XOR_VALUE_0000: /* xor value type 0000 */
            *(unsigned short *)(void *)(&handle->baseAddress->crc_post_xor_value) = CRC_XOR_VALUE_0000;
            break;
        case TYPE_CRC_XOR_VALUE_FFFF:
            *(unsigned short *)(void *)(&handle->baseAddress->crc_post_xor_value) = CRC_XOR_VALUE_FFFF;
            break;
        case TYPE_CRC_XOR_VALUE_00000000: /* xor value type 00000000 */
            *(unsigned int *)(void *)(&handle->baseAddress->crc_post_xor_value) = CRC_XOR_VALUE_00000000;
            break;
        case TYPE_CRC_XOR_VALUE_FFFFFFFF: /* xor value type FFFFFFFF */
            *(unsigned int *)(void *)(&handle->baseAddress->crc_post_xor_value) = CRC_XOR_VALUE_FFFFFFFF;
            break;
        default:
            break;
    }
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
        case TYPE_CRC_INIT_VALUE_00: /* init value type 00 */
            *(unsigned char *)(void *)(&handle->baseAddress->crc_calc_init_value) = CRC_INIT_VALUE_00;
            break;
        case TYPE_CRC_INIT_VALUE_FF: /* init value type FF */
            *(unsigned char *)(void *)(&handle->baseAddress->crc_calc_init_value) = CRC_INIT_VALUE_FF;
            break;
        case TYPE_CRC_INIT_VALUE_0000: /* init value type 0000 */
            *(unsigned short *)(void *)(&handle->baseAddress->crc_calc_init_value) = CRC_INIT_VALUE_0000;
            break;
        case TYPE_CRC_INIT_VALUE_FFFF: /* init value type FFFF */
            *(unsigned short *)(void *)(&handle->baseAddress->crc_calc_init_value) = CRC_INIT_VALUE_FFFF;
            break;
        case TYPE_CRC_INIT_VALUE_FFFFFFFF: /* init value type FFFFFFFF */
            *(unsigned int *)(void *)(&handle->baseAddress->crc_calc_init_value) = CRC_INIT_VALUE_FFFFFFFF;
            break;
        default:
            break;
    }
}
