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
  * @file      crc_load_sample.c
  * @author    MCU Driver Team
  * @brief     CRC module load init data config sample
  * @details   Perform the CRC16_XMODEM algorithm operation on the 16-bit data of 0x5678 to obtain a CRC value. \
  *            Compare the CRC value with the standard value to determine whether the value is correct. Then load \
  *            the initial value 0x0000 to the CRC module. Therefore, the CRC16_XMODEM algorithm is configured as \
  *            the CRC16_CCIT-FALSE algorithm, and then the value 0x5678 is calculated. The CRC value generated \
  *            after the calculation is compared with the standard value to determine whether the value of the \
  *            initial load value is correct.
  */

/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "crc.h"
#include "crc_load_sample.h"
#include "main.h"

#define CRC_XMODEM_REF_VALUE      0x5b86
#define CRC_CCITFALSE_REF_VALUE   0x4689

void CRC_CallbackFunc(void *handle);

/**
  * @brief To test the function of loading the check_in value to crc_data register.
  *        The load value is read from crc_out.
  *        The value is reversed and the original crc_out is overwritten.
  * @param None
  * @retval Value of @ref BASE_StatusType.
  */
BASE_StatusType CRC_LoadSample(void)
{
    SystemInit();
    DBG_UartPrintInit(115200); /* 115200 : baud rate */
    /* CRC16_XMODEM Algrithem, init data = 0x0000 */
    unsigned int preLoadCrcValue = HAL_CRC_SetInputDataGetCheck(&g_loadCrcHandle, 0x5678);
    DBG_PRINTF("preLoadCrcValue: 0x%x \r\n", preLoadCrcValue);
    if (preLoadCrcValue == CRC_XMODEM_REF_VALUE) {
        DBG_PRINTF("CRC Algrithem is CRC16_XMODEM, inputData is 0x5678 \r\n");
    } else {
        DBG_PRINTF("CRC Algrithem is not right for this sample! \r\n");
    }
    /* load init data 0xffff, crc algrithem from CRC16_XMODEM to CRC16_CCIT-FALSE */
    HAL_CRC_SetCheckInData(&g_loadCrcHandle, 0xFFFF);
    unsigned int initData = HAL_CRC_LoadCheckInData(&g_loadCrcHandle);
    /* CRC16_CCIT-FALSE Algrithem, init data = 0xffff */
    unsigned int afterLoadCrcValue = HAL_CRC_SetInputDataGetCheck(&g_loadCrcHandle, 0x5678);
    DBG_PRINTF("initData: 0x%x \r\n", initData);
    DBG_PRINTF("afterLoadCrcValue: 0x%x \r\n", afterLoadCrcValue);
    if (afterLoadCrcValue == CRC_CCITFALSE_REF_VALUE) {
        DBG_PRINTF("CRC Algrithem is CRC16_CCIT-FALSE, load init data success! \r\n");
    } else {
        DBG_PRINTF("load init data fail ! \r\n");
    }
    return BASE_STATUS_OK;
}

/**
  * @brief crc timeout callback function
  * @param handle CRC handle
  * @retval None.
  */
void CRC_CallbackFunc(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("CRC pready timeout! \r\n");
}
