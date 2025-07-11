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
  * @file    sample_i2c_blocking_stlm75.c
  * @author  MCU Driver Team
  * @brief   Sample for I2C Module Blocking.
  * @details This sample is connected to the temperature sensor stlm75 to demonstrate how to use the HAL interface that
  *          operates in blocking mode in the I2C driver.
  *          This sample must be connected to the stlm75 temperature sensor.
  *          Run the sample and print the temperature value through the serial port.
  */
#include "main.h"
#include "i2c.h"
#include "debug.h"
#include "sample_i2c_blocking_stlm75.h"

#define STLM75_DEV_ADDR_WRITE       0x90
#define STLM75_DEV_ADDR_READ        0x91

#define MAX_TIMEOUT_VAL             5000

/**
  * @brief I2c blocking sample lm75 read two byte.
  * @param addr stlm75 address.
  * @retval unsigned short.
  */
static unsigned short Stlm75ReadtwoByte(unsigned char addr)
{
    unsigned short temp;
    unsigned char opAddr = addr;
    unsigned char rxBuff[3];

    HAL_I2C_MasterWriteBlocking(&g_i2cSampleHandle, STLM75_DEV_ADDR_WRITE, &opAddr, 1, MAX_TIMEOUT_VAL);
    HAL_I2C_MasterReadBlocking(&g_i2cSampleHandle, STLM75_DEV_ADDR_READ, rxBuff, 2, MAX_TIMEOUT_VAL); /* Read 2 data */

    temp = rxBuff[0];
    temp <<= 8; /* Shift left by 8 bits */
    temp = temp | rxBuff[1];
    return temp;
}

/**
  * @brief I2c blocking sample processing.
  * @param addr lm75 address.
  * @retval unsigned int, 0 is OK, other is fail.
  */
unsigned int Stlm75SampleProcessing(void)
{
    unsigned short temp;
    float ftemp;

    SystemInit();
    while (1) {
        temp = Stlm75ReadtwoByte(0x00);
        temp >>= 5; /* Move the data by 5 bits to the right to obtain the lower 10 bits. */
        if ((temp & 0x0400) == 0x0400) { /* Determine the 10th digit. temp & 0x0400 */
            ftemp = -temp * 0.125; /* The formula is temp x 0.125. */
        } else {
            ftemp = temp * 0.125; /* The formula is temp x 0.125. */
        }
        DBG_PRINTF("ftemp = %f\r\n", ftemp);
        BASE_FUNC_DELAY_MS(1000); /* Delay 1000ms */
    }
    return 0;
}

