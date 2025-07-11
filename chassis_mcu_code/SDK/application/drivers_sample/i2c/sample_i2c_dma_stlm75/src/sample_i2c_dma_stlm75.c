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
  * @file    sample_i2c_dma_stlm75.c
  * @author  MCU Driver Team
  * @brief   Sample for I2C Module DMA.
  * @details This sample is connected to the temperature sensor stlm75 to demonstrate how to use the HAL interface that
  *          operates in DMA mode in the I2C driver.
  *          This sample must be connected to the stlm75 temperature sensor.
  *          Run the sample and print the temperature value through the serial port.
  */
#include "main.h"
#include "i2c.h"
#include "debug.h"
#include "sample_i2c_dma_stlm75.h"

#define DEV_ADDR_WRITE    0x90
#define DEV_ADDR_READ     0x91


static volatile unsigned char g_txFlang = 0;
static volatile unsigned char g_rxFlang = 0;

static unsigned char g_rxBuff[3];
static unsigned char g_txBuff[3];

/**
  * @brief I2c dma sample stlm75 error handle.
  * @param handle I2c handle.
  * @retval None.
  */
void Stlm75ErrSampleHandle(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("errHandle\r\n");
}

/**
  * @brief I2c dma sample stlm75 tx handle.
  * @param handle I2c handle.
  * @retval None.
  */
void Stlm75TxSampleHandle(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("txHandle\r\n");
    g_txFlang = 1;
}

/**
  * @brief I2c dma sample stlm75 rx handle.
  * @param handle I2c handle.
  * @retval None.
  */
void Stlm75RxSampleHandle(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("rxHandle\r\n");
    g_rxFlang = 1;
}

/**
  * @brief I2c dma sample stlm75 init.
  * @param None.
  * @retval None.
  */
void Stlm75DmaSampleProcessing(void)
{
    unsigned short temp = 0;
    float ftemp = 0;
    g_txBuff[0] = 0;

    SystemInit();
    HAL_I2C_MasterWriteDMA(&g_i2cSampleHandle, DEV_ADDR_WRITE, g_txBuff, 1);
    while (1) {
        if (g_rxFlang > 0) {
            g_rxFlang = 0;
            temp = g_rxBuff[0];
            temp <<= 8; /* Shift left by 8 bits */
            temp = temp | g_rxBuff[1];
            temp >>= 5; /* Move right by 5 digits */
            if ((temp & 0x0400) == 0x0400) { /* Determine the 10th digit. temp & 0x0400 */
                ftemp = -temp * 0.125; /* The formula is temp x 0.125. */
            } else {
                ftemp = temp * 0.125; /* The formula is temp x 0.125. */
            }
            DBG_PRINTF("ftemp = %f\r\n", ftemp);
            HAL_I2C_MasterWriteDMA(&g_i2cSampleHandle, DEV_ADDR_WRITE, g_txBuff, 1);
        }
        if (g_txFlang > 0) {
            g_txFlang = 0;
            HAL_I2C_MasterReadDMA(&g_i2cSampleHandle, DEV_ADDR_READ, g_rxBuff, 2); /* Read 2 data */
        }
        BASE_FUNC_DELAY_MS(500); /* Delay 500ms */
    }
}