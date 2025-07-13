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
  * @file      sample_crc_gen_algo.c
  * @author    MCU Driver Team
  * @brief     Generates the CRC value.
  * @details   Setting a group of unsigned short values increasing from 0 and performing CRC accumulation \
  *            operation on the grouping values to generate a CRC value;
  */

/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "crc.h"
#include "main.h"
#include "sample_crc_gen_algo.h"

#define TABLE_SIZE  1024

static unsigned short g_crcTempData[TABLE_SIZE] = {0};

/**
  * @brief To test the function of generating a CRC value By Algorithm.
  * @param None
  * @retval Value of @ref BASE_StatusType.
  */
static BASE_StatusType CRC_GenerateByAlgorithm(void)
{
    CRC_Handle genAlgo = {0};
    genAlgo.handleEx.algoMode = CRC16_XMODEM; /* crc algorithm mode CRC16_XMODEM */
    genAlgo.baseAddress = CRC;
    genAlgo.inputDataFormat = CRC_MODE_BIT16; /* crc data size */
    HAL_CRC_Init(&genAlgo);
    unsigned int res = HAL_CRC_Accumulate(&genAlgo, g_crcTempData, TABLE_SIZE);
    DBG_PRINTF("\r\n res %x size %d \r\n", res, TABLE_SIZE);
    return BASE_STATUS_OK;
}

/**
  * @brief To test the function of generating a CRC value By Algorithm Attribute.
  * @param None
  * @retval Value of @ref BASE_StatusType.
  */
static BASE_StatusType CRC_GenerateByAlgorithmAttr(void)
{
    CRC_Handle genAlgoAttr = {0};
    genAlgoAttr.baseAddress = CRC;
    genAlgoAttr.inputDataFormat = CRC_MODE_BIT16; /* crc input data size */
    genAlgoAttr.initValueType = TYPE_CRC_INIT_VALUE_0000;
    genAlgoAttr.polyMode = CRC16_1021_POLY_MODE;
    genAlgoAttr.xorEndianEnbaleType = ENABLE_XOR_ENABLE_LSB; /* enabel xor and enable lsb */
    genAlgoAttr.reverseEnableType = REVERSE_INPUT_FALSE_OUTPUT_FALSE;
    genAlgoAttr.resultXorValueType = TYPE_CRC_XOR_VALUE_0000;
    HAL_CRC_Init(&genAlgoAttr);
    unsigned int res = HAL_CRC_Accumulate(&genAlgoAttr, g_crcTempData, TABLE_SIZE);
    DBG_PRINTF("\r\n res %x size %d \r\n", res, TABLE_SIZE);
    return BASE_STATUS_OK;
}

/**
  * @brief To test the function of generating a CRC value.
  * @param None
  * @retval Value of @ref BASE_StatusType.
  */
BASE_StatusType CRC_GenerateSample(void)
{
    SystemInit();
    HAL_CRG_IpEnableSet(CRC_BASE, IP_CLK_ENABLE);
    for (unsigned int i = 0 ; i < TABLE_SIZE; i++) {
        g_crcTempData[i] = i;
    }
    DBG_PRINTF("GenerateByAlgorithm:------ \r\n");
    CRC_GenerateByAlgorithm(); /* crc gen by algorithm */
    DBG_PRINTF("GenerateByAlgorithmAttr:------ \r\n");
    CRC_GenerateByAlgorithmAttr(); /* crc gen by algorithm attr */
    return BASE_STATUS_OK;
}

