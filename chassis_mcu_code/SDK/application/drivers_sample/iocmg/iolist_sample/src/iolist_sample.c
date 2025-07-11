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
  * @file      iolist_sample.c
  * @author    MCU Driver Team
  * @brief     Calculate and verify the CRC value of the data.
  * @details   IOCMG Config in IOListTable, init the iolist g_resultTable, printf the config of the g_resultTable \
  *            and Get config after init ,verify the value has been configed success or not, and \
  *            set config by HAL_IOCMG_SetConfig API, and get congfig by HAL_IOCMG_GetConfig API, \
  *            Compare the config value is match or not.
  */

/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "iocmg.h"
#include "iolist_sample.h"

/* for sample normalize in different chip, \
   when user use it, needn't define this macro, just need find the io pin in the iomap.h */
#define UART0_TXD_PIN   IOCMG_PIN_MUX(iocmg_6, FUNC_MODE_4, 0x0000)
#define UART0_RXD_PIN   IOCMG_PIN_MUX(iocmg_7, FUNC_MODE_4, 0x0000)
#define I2C0_SCL_PIN    IOCMG_PIN_MUX(iocmg_14, FUNC_MODE_5, 0x0211)
#define I2C0_SDA_PIN    IOCMG_PIN_MUX(iocmg_15, FUNC_MODE_5, 0x0001)

static IOCMG_Handle g_ioListTable[] = {
    {UART0_TXD_PIN, PULL_BOTH, SCHMIDT_ENABLE, LEVEL_SHIFT_RATE_SLOW, DRIVER_RATE_2},
    {UART0_RXD_PIN, PULL_BOTH, SCHMIDT_ENABLE, LEVEL_SHIFT_RATE_SLOW, DRIVER_RATE_2},
    {I2C0_SCL_PIN,  PULL_UP,   SCHMIDT_ENABLE, LEVEL_SHIFT_RATE_FAST, DRIVER_RATE_1},
    {I2C0_SDA_PIN,  PULL_UP,   SCHMIDT_ENABLE, LEVEL_SHIFT_RATE_FAST, DRIVER_RATE_1},
};

const unsigned int IOLIST_SIZE = sizeof(g_ioListTable) / sizeof(g_ioListTable[0]);

/**
  * @brief Init IOLIST by HAL_IOCMG_Init API.
  * @param None
  * @retval None.
  */
void IOCMG_IOListInitSample(void)
{
    /* init method 1 */
    for (unsigned int index = 0; index < IOLIST_SIZE; ++index) {
        HAL_IOCMG_Init(&g_ioListTable[index]);  /* Use the HAL_IOCMG_Init function to init uart. */
    }
    DBG_UartPrintInit(115200); /* 115200 : baud rate */
    BASE_FUNC_DELAY_MS(10); /* 10 : delay 10ms */
    while (1) {
        DBG_PRINTF("**************************************************************\r\n");
        /* test API */
        for (unsigned int index = 0; index < IOLIST_SIZE; ++index) {
            DBG_PRINTF("\r\nindex = %d Func Num = %d \r\n", index, \
                HAL_IOCMG_GetPinAltFuncMode(g_ioListTable[index].pinTypedef));
            DBG_PRINTF("Pull Mode = %d \r\n", HAL_IOCMG_GetPinPullMode(g_ioListTable[index].pinTypedef));
            DBG_PRINTF("Level Shift Rate = %d \r\n", HAL_IOCMG_GetPinLevelShiftRate(g_ioListTable[index].pinTypedef));
            DBG_PRINTF("Schmidt Mode = %d \r\n", HAL_IOCMG_GetPinSchmidtMode(g_ioListTable[index].pinTypedef));
            DBG_PRINTF("Drive Rate = %d \r\n", HAL_IOCMG_GetPinDriveRate(g_ioListTable[index].pinTypedef));
            BASE_FUNC_DELAY_MS(10); /* 10: Prevents the printing rate of the serial port from being too high. */
        }

        /* init method 2 */
        for (unsigned int index = 2; index < IOLIST_SIZE; ++index) {
            HAL_IOCMG_SetPinAltFuncMode(g_ioListTable[index].pinTypedef);
            HAL_IOCMG_SetPinPullMode(g_ioListTable[index].pinTypedef, PULL_BOTH);
            HAL_IOCMG_SetPinLevelShiftRate(g_ioListTable[index].pinTypedef, LEVEL_SHIFT_RATE_SLOW);
            HAL_IOCMG_SetPinSchmidtMode(g_ioListTable[index].pinTypedef, SCHMIDT_ENABLE);
            HAL_IOCMG_SetPinDriveRate(g_ioListTable[index].pinTypedef, DRIVER_RATE_2);
        }
        /* test API */
        for (unsigned int index = 2; index < IOLIST_SIZE; ++index) {
            DBG_PRINTF("\r\nindex = %d Func Num = %d \r\n", index, \
                HAL_IOCMG_GetPinAltFuncMode(g_ioListTable[index].pinTypedef));
            DBG_PRINTF("Pull Mode = %d \r\n", HAL_IOCMG_GetPinPullMode(g_ioListTable[index].pinTypedef));
            DBG_PRINTF("Level Shift Rate = %d \r\n", HAL_IOCMG_GetPinLevelShiftRate(g_ioListTable[index].pinTypedef));
            DBG_PRINTF("Schmidt Mode = %d \r\n", HAL_IOCMG_GetPinSchmidtMode(g_ioListTable[index].pinTypedef));
            DBG_PRINTF("Drive Rate = %d \r\n", HAL_IOCMG_GetPinDriveRate(g_ioListTable[index].pinTypedef));
            BASE_FUNC_DELAY_MS(10); /* 10 : Prevents the printing rate of the serial port from being too high. */
        }

        /* test osc pin function */
        HAL_IOCMG_SetOscClkFuncMode(BASE_CFG_ENABLE);
        HAL_IOCMG_SetOscClkOutputMode(BASE_CFG_ENABLE);
        HAL_IOCMG_SetOscClkDriveRate(DRIVER_RATE_2);
        /* test API */
        DBG_PRINTF("\r\nOsc func mode = %d \r\n", HAL_IOCMG_GetOscClkFuncMode());
        DBG_PRINTF("Osc clk output mode = %d \r\n", HAL_IOCMG_GetOscClkOutputMode());
        DBG_PRINTF("Osc drive rate = %d \r\n", HAL_IOCMG_GetOscClkDriveRate());
        BASE_FUNC_DELAY_MS(1000U); /* delay 1s */
    }
}
