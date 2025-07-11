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
  * @file    sample_spi_blocking_kta7953.c
  * @author  MCU Driver Team
  * @brief   Sample for SPI Module Blocking.
  * @details This sample is connected to the ADC chip kta7953 to demonstrate how to operate the HAL interface
  *          in blocking mode in the SPI driver.
  *          This sample operates channels 0 to 6 of ADC chip kta7953 in manual mode.
  *          This sample outputs the sampling values of each channel of the ADC chip kta7953 through the serial port.
  *          To use this sample, connect the external ADC chip kta7953.
  */
#include "main.h"
#include "spi.h"
#include "debug.h"
#include "sample_spi_blocking_kta7953.h"

#define USER_TIMEOUT    0x400

#define MANUAL_MODE_SET_CH0 0x1040
#define MANUAL_MODE_SET_CH1 0x10C0
#define MANUAL_MODE_SET_CH2 0x1140
#define MANUAL_MODE_SET_CH3 0x11C0
#define MANUAL_MODE_SET_CH4 0x1240
#define MANUAL_MODE_SET_CH5 0x12C0
#define MANUAL_MODE_SET_CH6 0x1340

#define MAX_TIMEOUT_VAL     5000

/**
  * @brief Spi blocking sample kta7953 processing.
  * @param None.
  * @retval None.
  */
void Kta7953SampleProcessing(void)
{
    unsigned short temp = MANUAL_MODE_SET_CH0;
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
    unsigned short tempRdata[10];
    unsigned short outCh = 0;
    unsigned short outData = 0;

    SystemInit();
    HAL_SPI_WriteBlocking(&g_spiSampleHandle, (unsigned char *)&temp, sizeof(temp), MAX_TIMEOUT_VAL);
    while (1) {
        for (int i = 0; i < 10; i++) { /* Test 10 times */
            if (i < 7) { /* First 7 */
                HAL_SPI_WriteReadBlocking(&g_spiSampleHandle,
                                          (unsigned char *)&tempRdata[i],
                                          (unsigned char *)&tempWdata[i],
                                          sizeof(tempWdata[i]),
                                          MAX_TIMEOUT_VAL);
            } else {
                HAL_SPI_WriteBlocking(&g_spiSampleHandle,
                                      (unsigned char *)&tempWdata[i],
                                      sizeof(tempWdata[i]),
                                      MAX_TIMEOUT_VAL);
                HAL_SPI_ReadBlocking(&g_spiSampleHandle,
                                     (unsigned char *)&tempRdata[i],
                                     sizeof(tempRdata[i]),
                                     MAX_TIMEOUT_VAL);
            }
            outCh = tempRdata[i] >> 12; /* Shift right by 12 bits to obtain the lower four bits. */
            outData = tempRdata[i] & 0x0FFF; /* Mask 0x0FFF */
            DBG_PRINTF("[%d], CH:%d = %d\r\n", i, outCh, outData);
        }

        BASE_FUNC_DELAY_MS(2000); /* Delay 2000ms */
    }
}
