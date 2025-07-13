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
  * @file    sample_spi_microwire_master.c
  * @author  MCU Driver Team
  * @brief   Sample for SPI Module microwire master.
  * @details This sample demonstrates how to use the HAL interface to perform operations on the microwire master.
  *          The microwire master sends different commands to the microwire slave to obtain different data:
  *          0xFF->0xFFFF
  *          0x1C->0x1C20
  *          0x83->0x1183
  *          0x85->0x1285
  *          0x40->0x1240
  *          0xc0->0x12C0
  *          0x13->0x1340
  *          This sample must be used together with sample_spi_microwire_slave. One device functions as the
  *          master device and the other as the slave device.
  */
#include "main.h"
#include "spi.h"
#include "debug.h"
#include "sample_spi_microwire_master.h"

#define USER_TIMEOUT    0x400

#define MANUAL_MODE_SET_CH0 0xFF
#define MANUAL_MODE_SET_CH1 0x1c
#define MANUAL_MODE_SET_CH2 0x83
#define MANUAL_MODE_SET_CH3 0x85
#define MANUAL_MODE_SET_CH4 0x40
#define MANUAL_MODE_SET_CH5 0xC0
#define MANUAL_MODE_SET_CH6 0x13

#define MAX_TIMEOUT_VAL    5000

/**
  * @brief Spi microwire master sample processing.
  * @param None.
  * @retval None.
  */
void MicroWireMasterTestSampleProcessing(void)
{
    unsigned short tempWdata[] = {
        MANUAL_MODE_SET_CH0,
        MANUAL_MODE_SET_CH1,
        MANUAL_MODE_SET_CH2,
        MANUAL_MODE_SET_CH3,
        MANUAL_MODE_SET_CH4,
        MANUAL_MODE_SET_CH5,
        MANUAL_MODE_SET_CH6,
        MANUAL_MODE_SET_CH2,
        MANUAL_MODE_SET_CH3,
        MANUAL_MODE_SET_CH4
    };
    unsigned short tempRdata[10] = {0};

    SystemInit();
    while (1) {
        for (int i = 0; i < 7; i++) { /* Test 7 times */
            HAL_SPI_WriteBlocking(&g_spiSampleHandle, (unsigned char *)&tempWdata[i],
                                  sizeof(tempWdata[i]), MAX_TIMEOUT_VAL);
            HAL_SPI_ReadBlocking(&g_spiSampleHandle, (unsigned char *)&tempRdata[i],
                                 sizeof(tempRdata[i]), MAX_TIMEOUT_VAL);
            BASE_FUNC_DELAY_MS(100); /* Delay 100ms */
            DBG_PRINTF("tempWdata[%d] = 0x%x \r\n", i, tempWdata[i]);
            DBG_PRINTF("tempRdata[%d] = 0x%x \r\n", i, tempRdata[i]);
        }
        BASE_FUNC_DELAY_MS(1000); /* Delay 1000ms */
    }
}