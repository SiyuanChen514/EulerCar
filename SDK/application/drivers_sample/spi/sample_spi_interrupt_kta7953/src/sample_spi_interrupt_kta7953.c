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
  * @file    sample_spi_interrupt_kta7953.c
  * @author  MCU Driver Team
  * @brief   Sample for SPI Module Interrupt.
  * @details This sample is connected to the ADC chip kta7953 to demonstrate how to operate the HAL interface
  *          in interrupt mode in the SPI driver.
  *          This sample operates channels 0 to 6 of ADC chip kta7953 in manual mode.
  *          This sample outputs the sampling values of each channel of the ADC chip kta7953 through the serial port.
  *          To use this sample, connect the external ADC chip kta7953.
  */
#include "main.h"
#include "spi.h"
#include "debug.h"
#include "interrupt.h"
#include "sample_spi_interrupt_kta7953.h"

#define USER_TIMEOUT    0x400

#define MANUAL_MODE_SET_CH0 0x1040
#define MANUAL_MODE_SET_CH1 0x10C0
#define MANUAL_MODE_SET_CH2 0x1140
#define MANUAL_MODE_SET_CH3 0x11C0
#define MANUAL_MODE_SET_CH4 0x1240
#define MANUAL_MODE_SET_CH5 0x12C0
#define MANUAL_MODE_SET_CH6 0x1340


static volatile unsigned int g_txrxFlag = 0;
static unsigned short g_tempRdata[10];
static volatile unsigned short g_outCh = 0;
static volatile unsigned short g_outData = 0;

/**
  * @brief Spi interrupt sample kta7953 buffer printf.
  * @param None.
  * @retval None.
  */
static void Kta7953BuffPrintf(void)
{
    for (int i = 0; i < 10; i++) { /* Print 10 groups of test data. */
        g_outCh = g_tempRdata[i] >> 12; /* Shift right by 12 bits to obtain the lower four bits. */
        g_outData = g_tempRdata[i] & 0x0FFF; /* Mask 0x0FFF */
        DBG_PRINTF("[%d] = CH:%d = %d\r\n", i, g_outCh, g_outData);
    }
}

/**
  * @brief Spi interrupt sample kta7953 TxRx callback handle.
  * @param handle SPI handle.
  * @retval None.
  */
void TxRxSampleCallbackHandle(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    g_txrxFlag = 1;
    DBG_PRINTF("TxRxCpltCallbackHandle\r\n");
}

/**
  * @brief Spi interrupt sample kta7953 Tx callback handle.
  * @param handle SPI handle.
  * @retval None.
  */
void TxSampleCallbackHandle(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("txCpltCallbackHandle\r\n");
}

/**
  * @brief Spi interrupt sample kta7953 Rx callback handle.
  * @param handle SPI handle.
  * @retval None.
  */
void RxSampleCallbackHandle(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("RxCpltCallbackHandle\r\n");
}

/**
  * @brief Spi interrupt sample kta7953 Error callback handle.
  * @param handle SPI handle.
  * @retval None.
  */
void ErrorSampleCallbackHandle(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("ErrorCallbackHandle\r\n");
}

/**
  * @brief Spi interrupt sample kta7953 processing.
  * @param None.
  * @retval None.
  */
void Kta7953InterruptSampleProcessing(void)
{
    static unsigned short cnt = 0;
    unsigned short temp = MANUAL_MODE_SET_CH0;
    unsigned short tempWdata[] = {
        MANUAL_MODE_SET_CH0,
        MANUAL_MODE_SET_CH1,
        MANUAL_MODE_SET_CH2,
        MANUAL_MODE_SET_CH3,
        MANUAL_MODE_SET_CH4,
        MANUAL_MODE_SET_CH5,
        MANUAL_MODE_SET_CH6,
        3,
        0,
        0
    };

    SystemInit();
    HAL_SPI_WriteIT(&g_spiSampleHandle, (unsigned char *)&temp, sizeof(temp));
    while (1) {
        if (g_txrxFlag > 0) {
            g_txrxFlag = 0;

            if (cnt >= 10) { /* Test 10 times in total */
                cnt = 0;
                g_txrxFlag = 1;
                Kta7953BuffPrintf();
            }

            if (cnt > 7 && cnt < 10) { /* 7 to 10 */
                HAL_SPI_ReadIT(&g_spiSampleHandle, (unsigned char *)&g_tempRdata[cnt], sizeof(g_tempRdata[cnt]));
                cnt++;
            }

            if (cnt == 7) { /*  7th */
                HAL_SPI_WriteIT(&g_spiSampleHandle, (unsigned char *)&tempWdata[cnt], sizeof(tempWdata[cnt]));
                g_tempRdata[cnt] = cnt;
                cnt++;
            }

            if (cnt < 7) { /* First 7 */
                HAL_SPI_WriteReadIT(&g_spiSampleHandle,
                                    (unsigned char *)&g_tempRdata[cnt],
                                    (unsigned char *)&tempWdata[cnt],
                                    sizeof(tempWdata[cnt]));
                cnt++;
            }
        }
        BASE_FUNC_DELAY_MS(300); /* Delay 300ms */
    }
}
