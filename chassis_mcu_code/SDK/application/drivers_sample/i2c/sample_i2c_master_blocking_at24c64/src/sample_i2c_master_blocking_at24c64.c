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
  * @file    sample_i2c_master_blocking_at24c64.c
  * @author  MCU Driver Team
  * @brief   Sample for I2C module blocking as master.
  * @details This sample demonstrates how to use the I2C master blocking interface to read and write the EEPROM.
  *          To use this sample, the I2C interface must be connected to the AT24C64 EEPROM chip.
  */
#include "i2c.h"
#include "main.h"
#include "debug.h"
#include "sample_i2c_master_blocking_at24c64.h"

#define DEV_24C64_ADDRESS_WRITE            0xA0
#define DEV_24C64_ADDRESS_READ             0xA1
#define I2C_SAMPLE_24C64_OPT_LEN           8
#define I2C_SAMPLE_24C64_OPT_START_ADDR    0x0
#define I2C_SAMPLE_24C64_DATA_OFFSET       2
#define I2C_SAMPLE_24C64_PAGE_SIZE         32
#define I2C_SAMPLE_24C64_ADDR_SIZE         2
#define I2C_SAMPLE_24C64_OPT_ONCE_LEN      255
#define I2C_SAMPLE_24C64_ADDRESS_POS       8
#define I2C_SAMPLE_24C64_ADDRESS_MASK      0xFF

#define I2C_SAMPLE_24C64_TEST_NUM          200
#define I2C_SAMPLE_MAX_TIMEOUT             10000

/**
  * @brief Copy data.
  * @param destBuffer dest buffer.
  * @param srcBuffer source buffer.
  * @param len Number of the data to be copy.
  * @retval None.
  */
static void CopyData(unsigned char *destBuffer, unsigned char *srcBuffer, unsigned int len)
{
    for (unsigned int i = 0; i < len; i++) {
        destBuffer[i] = srcBuffer[i]; /* Copy the srcBuffer data to the destBuffer. */
    }
}

/**
  * @brief Read data from eeprom.
  * @param addr The memory address of eeprom.
  * @param buffer Address of buff to be receive data.
  * @param len Number of the data to be read.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
static BASE_StatusType Sample24c64ReadData(unsigned int addr, unsigned char *buffer, unsigned int len)
{
    BASE_StatusType ret = BASE_STATUS_OK;
    /* Define variables for internal use. */
    unsigned char tempAddr[I2C_SAMPLE_24C64_ADDR_SIZE];
    unsigned int currentLen = len;
    unsigned int currentAddr = addr;
    unsigned int tempReadLen;
    unsigned char *tempBuffer = buffer;

    /* Start read data from the 24c64 eeprom. */
    while (1) {
        if (currentLen == 0) {
            break;
        }
        /* Set the memory address of eeprom. */
        tempAddr[0] = (currentAddr >> I2C_SAMPLE_24C64_ADDRESS_POS) & I2C_SAMPLE_24C64_ADDRESS_MASK;
        tempAddr[1] = currentAddr & I2C_SAMPLE_24C64_ADDRESS_MASK;

        tempReadLen = currentLen;
        if (currentLen > I2C_SAMPLE_24C64_OPT_ONCE_LEN) {
            tempReadLen = I2C_SAMPLE_24C64_OPT_ONCE_LEN;
        }
        /* Send the memory address of eeprom. */
        ret = HAL_I2C_MasterWriteBlocking(&g_i2c0, DEV_24C64_ADDRESS_WRITE, tempAddr,
                                          I2C_SAMPLE_24C64_ADDR_SIZE, I2C_SAMPLE_MAX_TIMEOUT);
        if (ret != BASE_STATUS_OK) {
            DBG_PRINTF("LINE:%d,Read Data Fail,ret:%d\r\n", __LINE__, ret);
            return ret;
        }
        /* Read data from eeprom. */
        ret = HAL_I2C_MasterReadBlocking(&g_i2c0, DEV_24C64_ADDRESS_READ, tempBuffer,
                                         tempReadLen, I2C_SAMPLE_MAX_TIMEOUT);
        if (ret != BASE_STATUS_OK) {
            DBG_PRINTF("LINE:%d,Read Data Fail,ret:%d\r\n", __LINE__, ret);
            return ret;
        }
        /* Updata the destAddress, srcAddress and len. */
        currentAddr += tempReadLen;
        currentLen -= tempReadLen;
        tempBuffer += tempReadLen;
    }
    return ret;
}

/**
  * @brief Read data from eeprom.
  * @param addr The memory address of eeprom.
  * @param buffer Address of buff to be send.
  * @param len Number of the data to be send.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
static BASE_StatusType Sample24c64WriteData(unsigned int addr, unsigned char *buffer, unsigned int len)
{
    unsigned char tempWrite[I2C_SAMPLE_24C64_PAGE_SIZE + I2C_SAMPLE_24C64_ADDR_SIZE];
    unsigned int currentLen = len;
    unsigned int currentAddr = addr;
    unsigned int tempWriteLen;
    unsigned char *tempBuffer = buffer;
    BASE_StatusType ret = BASE_STATUS_OK;

    /* Start send data to eeprom. */
    while (1) {
        if (currentLen == 0) {
            break;
        }
        tempWriteLen = I2C_SAMPLE_24C64_PAGE_SIZE - (currentAddr % I2C_SAMPLE_24C64_PAGE_SIZE);
        if (tempWriteLen > currentLen) {
            tempWriteLen = currentLen;
        }
        /* Set the memory address of eeprom. */
        tempWrite[0] = (currentAddr >> I2C_SAMPLE_24C64_ADDRESS_POS) & I2C_SAMPLE_24C64_ADDRESS_MASK;
        tempWrite[1] = currentAddr & I2C_SAMPLE_24C64_ADDRESS_MASK;
        CopyData(&tempWrite[I2C_SAMPLE_24C64_ADDR_SIZE], tempBuffer, tempWriteLen);
        /* Send data to eeprom. */
        ret = HAL_I2C_MasterWriteBlocking(&g_i2c0, DEV_24C64_ADDRESS_WRITE, tempWrite,
                                          tempWriteLen + I2C_SAMPLE_24C64_ADDR_SIZE, I2C_SAMPLE_MAX_TIMEOUT);
        if (ret != BASE_STATUS_OK) {
            DBG_PRINTF("LINE:%d,Write Data Fail!,ret:%d\r\n", __LINE__, ret);
            break;
        }
        /* Updata the destAddress, srcAddress and len. */
        currentAddr += tempWriteLen;
        currentLen -= tempWriteLen;
        tempBuffer += tempWriteLen;
    }
    return ret;
}

/**
  * @brief Send and receive data with the 24c64 eeprom in interrupt mode as master.
  * @retval None.
  */
void I2cBlocking24c64Processing(void)
{
    BASE_StatusType ret;
    unsigned int i;
    unsigned int dataFlag;
    unsigned char tempWriteBuff[I2C_SAMPLE_24C64_OPT_LEN] = {0};
    unsigned int successCnt = 0;

    SystemInit();
    DBG_PRINTF("I2C Blocking 24C64 Start\r\n");
    for (i = 0; i < I2C_SAMPLE_24C64_OPT_LEN; i++) {
        tempWriteBuff[i] = 0x5A; /* The written data. */
    }
    BASE_FUNC_DELAY_MS(20); /* Delay 20 ms. */
    for (int j = 0; j < I2C_SAMPLE_24C64_TEST_NUM; j++) {
        unsigned char tempReadBuff[I2C_SAMPLE_24C64_OPT_LEN] = {0};
        /* Write 24c64 data */
        ret = Sample24c64WriteData(I2C_SAMPLE_24C64_OPT_START_ADDR, tempWriteBuff, I2C_SAMPLE_24C64_OPT_LEN);
        if (ret != BASE_STATUS_OK) {
            DBG_PRINTF("LINE:%d,Write Data Fail,ret:%d\r\n", __LINE__, ret);
        }
        BASE_FUNC_DELAY_MS(20); /* Delay 20 ms. */
        /* Read 24c64 data */
        ret = Sample24c64ReadData(I2C_SAMPLE_24C64_OPT_START_ADDR, tempReadBuff, I2C_SAMPLE_24C64_OPT_LEN);
        if (ret != BASE_STATUS_OK) {
            DBG_PRINTF("LINE:%d,Write Data Fail,ret:%d\r\n", __LINE__, ret);
        }
        BASE_FUNC_DELAY_MS(20); /* Delay 20 ms. */
        /* Compare read and write data */
        dataFlag = 0;
        for (i = 0; i < I2C_SAMPLE_24C64_OPT_LEN; i++) {
            if (tempReadBuff[i] != tempWriteBuff[i]) {
                DBG_PRINTF("I2C Data error! offset[%d]\r\nReadData:0x%x\r\nWriteData:0x%x\r\n", i,
                           tempReadBuff[i], tempWriteBuff[i]);
                dataFlag = 1;
                break;
            }
        }
        /* The read data is exactly the same as the written data. */
        if (dataFlag == 0) {
            successCnt++;
            DBG_PRINTF("I2C Data Success\r\n");
        }
    }
    DBG_PRINTF("I2C sample End!\r\nsuccessCnt:%d\r\n", successCnt);
}