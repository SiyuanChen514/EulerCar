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
  * @file      crc_gen_sample.c
  * @author    MCU Driver Team
  * @brief     Generates the CRC value.
  * @details   Setting a group of unsigned short values increasing from 0 and performing CRC accumulation \
  *            operation on the grouping values to generate a CRC value;
  */

/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "crc.h"
#include "crc_gen_sample.h"
#include "main.h"

#define TABLE_SIZE  1024

static unsigned short g_crcTempData[TABLE_SIZE] = {0};

/**
  * @brief To test the function of generating a CRC value.
  * @param None
  * @retval Value of @ref BASE_StatusType.
  */
BASE_StatusType CRC_GenerateSample(void)
{
    SystemInit();
    DBG_UartPrintInit(115200); /* 115200 : baud rate */
    for (unsigned int i = 0 ; i < TABLE_SIZE; i++) {
        g_crcTempData[i] = i;
    }
    unsigned int res = HAL_CRC_Accumulate(&g_genCrcHandle, g_crcTempData, TABLE_SIZE);
    DBG_PRINTF("\r\n res %x size %d \r\n", res, TABLE_SIZE);
    return BASE_STATUS_OK;
}

/**
  * @brief register callback function.
  * @param param Value of @ref CRC_Handle.
  * @retval None
  */
void CRC_CallbackFunc(void *param)
{
    /* User Add code here */
    BASE_FUNC_UNUSED(param);
    DBG_PRINTF("CRC pready timeout! \r\n");
}