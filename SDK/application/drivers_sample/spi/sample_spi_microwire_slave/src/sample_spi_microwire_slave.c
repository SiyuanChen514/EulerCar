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
  * @file    sample_spi_microwire_slave.c
  * @author  MCU Driver Team
  * @brief   Sample for SPI Module microwire slave.
  * @details This sample demonstrates how to use the HAL interface to perform operations on the microwire master.
  *          The microwire slave receives different commands sent by the master and responds with different data.:
  *          0xFF->0xFFFF
  *          0x1C->0x1C20
  *          0x83->0x1183
  *          0x85->0x1285
  *          0x40->0x1240
  *          0xc0->0x12C0
  *          0x13->0x1340
  *          This sample must be used together with sample_spi_microwire_master. One device functions as the
  *          master device and the other as the slave device.
  */
#include "main.h"
#include "spi.h"
#include "debug.h"
#include "sample_spi_microwire_slave.h"

#define USER_TIMEOUT    0x400

#define MANUAL_MODE_SET_CH0 0xFFFF
#define MANUAL_MODE_SET_CH1 0x1c20
#define MANUAL_MODE_SET_CH2 0x1183
#define MANUAL_MODE_SET_CH3 0x1285
#define MANUAL_MODE_SET_CH4 0x1240
#define MANUAL_MODE_SET_CH5 0x12C0
#define MANUAL_MODE_SET_CH6 0x1340

#define MANUAL_MODE_SET_CH0_RX 0xFF
#define MANUAL_MODE_SET_CH1_RX 0x1c
#define MANUAL_MODE_SET_CH2_RX 0x83
#define MANUAL_MODE_SET_CH3_RX 0x85
#define MANUAL_MODE_SET_CH4_RX 0x40
#define MANUAL_MODE_SET_CH5_RX 0xC0
#define MANUAL_MODE_SET_CH6_RX 0x13

#define MAX_TIMEOUT_VAL        5000

/**
  * @brief Spi microwire slave sample processing.
  * @param None.
  * @retval None.
  */
void MicroWireSlaveTestSampleProcessing(void)
{
    unsigned short tempWdata[] = {
        MANUAL_MODE_SET_CH0, MANUAL_MODE_SET_CH1,
        MANUAL_MODE_SET_CH2, MANUAL_MODE_SET_CH3,
        MANUAL_MODE_SET_CH4, MANUAL_MODE_SET_CH5,
        MANUAL_MODE_SET_CH6, MANUAL_MODE_SET_CH2,
        MANUAL_MODE_SET_CH3, MANUAL_MODE_SET_CH4
    };
    unsigned short tempRxData = 0;

    SystemInit();
    while (1) {
        HAL_SPI_ReadBlocking(&g_spiSampleHandle, (unsigned char *)&tempRxData, sizeof(tempRxData), MAX_TIMEOUT_VAL);
        switch (tempRxData) {
            case MANUAL_MODE_SET_CH0_RX :
                /* Send the 0 piece of data. */
                HAL_SPI_WriteBlocking(&g_spiSampleHandle, (unsigned char *)&tempWdata[0],
                                      sizeof(tempWdata[0]), MAX_TIMEOUT_VAL);     /* Send the 0 piece of data. */
                break;
            case MANUAL_MODE_SET_CH1_RX :
                /* Send the 1 piece of data. */
                HAL_SPI_WriteBlocking(&g_spiSampleHandle, (unsigned char *)&tempWdata[1],
                                      sizeof(tempWdata[1]), MAX_TIMEOUT_VAL);     /* Send the 1 piece of data. */
                break;
            case MANUAL_MODE_SET_CH2_RX :
                /* Send the 2 piece of data. */
                HAL_SPI_WriteBlocking(&g_spiSampleHandle, (unsigned char *)&tempWdata[2],
                                      sizeof(tempWdata[2]), MAX_TIMEOUT_VAL);     /* Send the 2 piece of data. */
                break;
            case MANUAL_MODE_SET_CH3_RX :
                /* Send the 3 piece of data. */
                HAL_SPI_WriteBlocking(&g_spiSampleHandle, (unsigned char *)&tempWdata[3],
                                      sizeof(tempWdata[3]), MAX_TIMEOUT_VAL);      /* Send the 3 piece of data. */
                break;
            case MANUAL_MODE_SET_CH4_RX :
                /* Send the 4 piece of data. */
                HAL_SPI_WriteBlocking(&g_spiSampleHandle, (unsigned char *)&tempWdata[4],
                                      sizeof(tempWdata[4]), MAX_TIMEOUT_VAL);     /* Send the 4 piece of data. */
                break;
            case MANUAL_MODE_SET_CH5_RX :
                /* Send the 5 piece of data. */
                HAL_SPI_WriteBlocking(&g_spiSampleHandle, (unsigned char *)&tempWdata[5],
                                      sizeof(tempWdata[5]), MAX_TIMEOUT_VAL);     /* Send the 5 piece of data. */
                break;
            case MANUAL_MODE_SET_CH6_RX :
                /* Send the 6 piece of data. */
                HAL_SPI_WriteBlocking(&g_spiSampleHandle, (unsigned char *)&tempWdata[6],
                                      sizeof(tempWdata[6]), MAX_TIMEOUT_VAL);     /* Send the 6 piece of data. */
                break;
            default :
                break;
        }
        DBG_PRINTF("tempRxData = 0x%x \r\n", tempRxData);
    }
}