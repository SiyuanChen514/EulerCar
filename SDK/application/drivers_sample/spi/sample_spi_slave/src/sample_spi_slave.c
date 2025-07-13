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
  * @file    sample_spi_slave.c
  * @author  MCU Driver Team
  * @brief   Sample for SPI Module slave.
  * @details This sample demonstrates the use of HAL interfaces in the salve mode. This sample uses the blocking mode.
  *            This sample must be connected to the master device.
  *          This sample sends 0x1105, 0x1c20, 0x1183, 0x1285, 0x1240, 0x12c0, and 0x1340 in polling mode.
  *           Print the received and sent data through the serial port.
  */
#include "main.h"
#include "spi.h"
#include "debug.h"
#include "sample_spi_slave.h"

#define SLAVE_READ_TESE
#define SLAVE_READ_WRITE_TESE
#define SLAVE_WRITE_TESE

#define USER_TIMEOUT    0x400

#define MANUAL_MODE_SET_CH0 0x1105
#define MANUAL_MODE_SET_CH1 0x1c20
#define MANUAL_MODE_SET_CH2 0x1183
#define MANUAL_MODE_SET_CH3 0x1285
#define MANUAL_MODE_SET_CH4 0x1240
#define MANUAL_MODE_SET_CH5 0x12C0
#define MANUAL_MODE_SET_CH6 0x1340

#define MAX_TIMEOUT_VAL     5000


/**
  * @brief Spi slave sample processing.
  * @param None.
  * @retval None.
  */
void SlaveTestSampleProcessing(void)
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
#ifdef SLAVE_READ_TESE
        HAL_SPI_ReadBlocking(&g_spiSampleHandle, (unsigned char *)tempRdata, sizeof(tempRdata), MAX_TIMEOUT_VAL);
        for (int i = 0; i < 10; i++) { /* Print the test data for 10 times. */
            DBG_PRINTF("tempRdata[%d] = 0x%x \r\n", i, tempRdata[i]);
            BASE_FUNC_DELAY_MS(10); /* Delay 10ms */
        }
#endif

#ifdef SLAVE_WRITE_TESE
        for (int i = 0; i < 10; i++) { /* Print the test data for 10 times. */
            DBG_PRINTF("tempWdata[%d] = 0x%x \r\n", i, tempWdata[i]);
            BASE_FUNC_DELAY_MS(10); /* Delay 10ms */
        }
        HAL_SPI_WriteBlocking(&g_spiSampleHandle, (unsigned char *)tempWdata, sizeof(tempWdata), MAX_TIMEOUT_VAL);
#endif

#ifdef SLAVE_READ_WRITE_TESE
        HAL_SPI_WriteReadBlocking(&g_spiSampleHandle, (unsigned char *)tempRdata,
                                  (unsigned char *)tempWdata,
                                  sizeof(tempWdata), MAX_TIMEOUT_VAL);
        for (int i = 0; i < 10; i++) { /* Print the test data for 10 times. */
            DBG_PRINTF("tempRdata[%d] = 0x%x  tempWdata[%d] = 0x%x \r\n", i, tempRdata[i], i, tempWdata[i]);
            BASE_FUNC_DELAY_MS(10); /* Delay 10ms */
        }
#endif
    }
}
