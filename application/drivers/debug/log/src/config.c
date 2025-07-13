/**
  * @copyright Copyright (c) 2023, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      config.c
  * @author    MCU Driver Team
  * @brief     config module driver
  * @details   The header file contains the following declaration:
  *             + Miniaturized logs are written based on addresses.
  *             + Abnormal event reporting
  */
#include <stdlib.h>
#include "config.h"
#include "common.h"
#include "console.h"
#include "type.h"
#include "string.h"
#include "securec.h"

/**
 * @brief read information by address
 * @param add : register address of the information to be read
 * @param value : storage array of read information
 * @param len ： length of the information to be read
 * @retval None.
 */
void ExtLoadRead(uintptr_t add, char *value, int len)
{
    /* Check whether the address is out of range */
    if (add > REGISTER_END || add < REGISTER_START) {
        EXT_PRINT("The address is out of range");
        return;
    }
    /* check param vaild */
    if (value == NULL) {
        EXT_PRINT("read data is null, please check value\n");
        return;
    }
    /* Read information cyclically */
    for (int i = 0; i < len; i++) {
        *(value + i) = *(volatile char *)(add + i);
    }
}

/**
 * @brief write information by address
 * @param add : register address of the information to write
 * @param value : storage array of write information
 * @param len ： length of the information to write
 * @retval None.
 */
void ExtLoadWrite(uintptr_t add, const char *value, int len)
{
    /* Check whether the address is out of range */
    if (add > REGISTER_END || add < REGISTER_START) {
        EXT_PRINT("The address is out of range");
        return;
    }
    /* check param vaild */
    if (value == NULL) {
        EXT_PRINT("write data is null, please check value\n");
        return;
    }
    /* Write information cyclically */
    for (int i = 0; i < len; i++) {
        *(volatile char *)(add + i) = *(value + i);
    }
}

char g_dataItem[DATA_ITEM_NUM_MAX][DATA_ITEM_MAX_LEN];

/**
 * @brief read information by config
 * @param item : event that starts to read data
 * @param value : storage array of read information
 * @param len ： length of the information to be read
 * @retval None.
 */
void ExtConfigRead(enum DataItem item, char *value, int len)
{
    /* Exceeded the maximum scenario value */
    if (item > DATA_ITEM_NUM_MAX) {
        EXT_PRINT("The config has exceeded max vaule");
        return;
    }
    /* check param vaild */
    if (len >= DATA_ITEM_MAX_LEN) {
        EXT_PRINT("The length of the read data exceeds 256\n");
        return;
    }
    if (value == NULL) {
        EXT_PRINT("The read content is empty, read err");
        return;
    }
    /* Reads the data stored in the register */
    if (memcpy_s(value, len, &g_dataItem[item], len) != EXT_SUCCESS) {
        EXT_PRINT("config read memcpy failed");
    }
    return;
}

/**
 * @brief write information by config
 * @param add : register address of the information to write
 * @param value : storage array of write information
 * @param len ： length of the information to write
 * @retval None.
 */
void ExtConfigWrite(enum DataItem item, const char *value, int len)
{
    /* Exceeded the maximum scenario value */
    if (item > DATA_ITEM_NUM_MAX) {
        EXT_PRINT("The config has exceeded max vaule");
        return;
    }
    /* check param vaild */
    if (len >= DATA_ITEM_MAX_LEN) {
        EXT_PRINT("The length of the write data exceeds 256\n");
        return;
    }
    if (value == NULL) {
        EXT_PRINT("The written content is empty, write err");
        return;
    }
    /* Writes data to a register for storage */
    if (memcpy_s(&g_dataItem[item], DATA_ITEM_MAX_LEN, value, len) != EXT_SUCCESS) {
        EXT_PRINT("config write memcpy failed");
    }
    return;
}
