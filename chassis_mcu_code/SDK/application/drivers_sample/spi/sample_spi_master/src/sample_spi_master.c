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
  * @file    sample_spi_master.c
  * @author  MCU Driver Team
  * @brief   Sample for SPI Module master.
  * @details This sample demonstrates the use of HAL interfaces in the master mode. This sample uses the blocking mode.
  *          This sample must be connected to the slave device.
  *          This sample code corresponds to sample_spi_slave.c code.
  *          This sample sends 0x1001, 0x1002, 0x1003, 0x1004, 0x1005, 0x1006, and 0x1007 in polling mode.
  *          Print the received and sent data through the serial port.
  */
#include "main.h"
#include "spi.h"
#include "debug.h"
#include "sample_spi_master.h"

#define MASTER_READ_TESE
#define MASTER_READ_WRITE_TESE
#define MASTER_WRITE_TESE

#define USER_TIMEOUT    0x400

#define MANUAL_MODE_SET_CH0 0x1001
#define MANUAL_MODE_SET_CH1 0x1002
#define MANUAL_MODE_SET_CH2 0x1003
#define MANUAL_MODE_SET_CH3 0x1004
#define MANUAL_MODE_SET_CH4 0x1005
#define MANUAL_MODE_SET_CH5 0x1006
#define MANUAL_MODE_SET_CH6 0x1007

#define MAX_TIMEOUT_VAL     5000


/**
  * @brief Spi master sample processing.
  * @param None.
  * @retval None.
  */
void MasterTestSampleProcessing(void)
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
#ifdef MASTER_WRITE_TESE
        for (int i = 0; i < 10; i++) { /* Print the test data for 10 times. */
            DBG_PRINTF("tempWdata[%d] = 0x%x \r\n", i, tempWdata[i]);
        }
        HAL_SPI_WriteBlocking(&g_spiSampleHandle, (unsigned char *)tempWdata, sizeof(tempWdata), MAX_TIMEOUT_VAL);
#endif
        BASE_FUNC_DELAY_MS(300); /* Delay 300ms */
#ifdef MASTER_READ_TESE
        HAL_SPI_ReadBlocking(&g_spiSampleHandle, (unsigned char *)tempRdata, sizeof(tempRdata), MAX_TIMEOUT_VAL);
        for (int i = 0; i < 10; i++) { /* Print the test data for 10 times. */
            DBG_PRINTF("tempRdata[%d] = 0x%x \r\n", i, tempRdata[i]);
        }
#endif
        BASE_FUNC_DELAY_MS(20); /* Delay 20ms */
#ifdef MASTER_READ_WRITE_TESE
        HAL_SPI_WriteReadBlocking(&g_spiSampleHandle, (unsigned char *)tempRdata,
                                  (unsigned char *)tempWdata,
                                  sizeof(tempWdata), MAX_TIMEOUT_VAL);
        for (int i = 0; i < 10; i++) { /* Print the test data for 10 times. */
            DBG_PRINTF("tempRdata[%d] = 0x%x  tempWdata[%d] = 0x%x \r\n", i, tempRdata[i], i, tempWdata[i]);
            BASE_FUNC_DELAY_MS(10); /* Delay 10ms */
        }
#endif
        BASE_FUNC_DELAY_MS(20); /* Delay 20ms */
    }
}
