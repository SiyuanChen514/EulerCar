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
  * @file    sample_spi_blocking_w25q32.c
  * @author  MCU Driver Team
  * @brief   Sample for SPI Module Blocking.
  * @details This sample is used to operate the HAL interface on the W25Q32 chip of the flash.
  *          Write data and read data to address 0 of W25Q32
  */
#include "main.h"
#include "spi.h"
#include "debug.h"
#include "sample_spi_blocking_w25q32.h"

#define SPI_W25X_WRITE_ENABLE 0x06
#define SPI_W25X_SECTOR_ERASE 0x20
#define SPI_W25X_PAGE_PROGRAM 0x02
#define SPI_W25X_READ_DATA 0x03

#define MAX_TIMEOUT_VAL 5000

static unsigned int g_flashAddr = 0;
static unsigned char g_writeBuf[120] = {0};
static unsigned char g_recvBuf[120] = {0};

void SPICsCallback(void *handle);
/**
  * @brief spi chip select callback function.
  * @param handle SPI handle.
  * @retval None
  */
void SPICsCallback(void *handle)
{
    SPI_Handle *spiHandle = (SPI_Handle *)handle;
    /* USER CODE BEGIN SPICsCallback */
    if (g_spiSampleHandle.csCtrl == SPI_CHIP_SELECT) {
        HAL_SPI_SetChipConfigSelectEx(spiHandle, HAL_SPI_CHIP_CONFIG_SOFR_UNSET);
    } else {
        HAL_SPI_SetChipConfigSelectEx(spiHandle, HAL_SPI_CHIP_CONFIG_SOFR_SET);
    }
    /* USER CODE END SPICsCallback */
}

/**
  * @brief w25q32 enable the write function.
  * @param None.
  * @retval BASE_StatusType：OK, ERROR.
  */
static BASE_StatusType SPI_W25Q32_WriteEnable(void)
{
    unsigned char temp = SPI_W25X_WRITE_ENABLE; /* Write enable command */
    BASE_StatusType ret;
    ret = HAL_SPI_WriteBlocking(&g_spiSampleHandle, (unsigned char *)&temp, sizeof(temp), MAX_TIMEOUT_VAL);
    if (ret != BASE_STATUS_OK) {
        DBG_PRINTF("enable error!\r\n");
        return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief w25q32 sector erase function.
  * @param sectorEraseAddr sector erase address.
  * @retval BASE_StatusType：OK, ERROR.
  */
static BASE_StatusType SectorErase(unsigned int sectorEraseAddr)
{
    BASE_StatusType ret;
    ret = SPI_W25Q32_WriteEnable();
    if (ret != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    BASE_FUNC_DELAY_MS(20); /* delay 20 ms */

    unsigned char buffer[4] = {0};
    buffer[0] = SPI_W25X_SECTOR_ERASE;               /* Sector erase command */
    buffer[1] = (sectorEraseAddr & 0xFF0000) >> 16;  /* Shift right by 16 bits */
    buffer[2] = (sectorEraseAddr & 0xFF00) >> 8;     /* Shift right by 8 bits, 2 array subscript  */
    buffer[3] = (sectorEraseAddr & 0xFF);            /* Lower 8 bits of the sector erase address, 3 array subscript */
    ret = HAL_SPI_WriteBlocking(&g_spiSampleHandle, buffer, sizeof(buffer), MAX_TIMEOUT_VAL);
    if (ret != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief w25q32 single page write, maximum 256 bytes.
  * @param writeBuf write data buffer.
  * @param writeAddr write address.
  * @param len data length.
  * @retval BASE_StatusType：OK, ERROR.
  */
static BASE_StatusType WritePage(unsigned char *writeBuf, unsigned int writeAddr, unsigned int len)
{
    BASE_StatusType ret;
    ret = SPI_W25Q32_WriteEnable(); /* enable the write */
    if (ret != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    BASE_FUNC_DELAY_MS(20); /* Delay 20ms */

    unsigned char buffer[124] = {0}; /* transmit 124 bytes: 4-byte command and 120-byte data */

    buffer[0] = SPI_W25X_PAGE_PROGRAM;         /* Page Program command */
    buffer[1] = (writeAddr & 0xFF0000) >> 16;  /* Shift right by 16 bits */
    buffer[2] = (writeAddr & 0xFF00) >> 8;     /* Shift right by 8 bits, 2 array subscript */
    buffer[3] = (writeAddr & 0xFF);            /* Lower 8 bits of the sector erase address, 3 array subscript */
    for (unsigned int count = 0; count < len; count++) {
        buffer[count + 4] = writeBuf[count];   /* data is stored from array subscript 4 */
    }
    ret = HAL_SPI_WriteBlocking(&g_spiSampleHandle, buffer, (len + 4), MAX_TIMEOUT_VAL); /* len + 4 is total size */
    if (ret != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Read data in the address.
  * @param recvBuf write data buffer.
  * @param recvAddr write address.
  * @param len data length.
  * @retval BASE_StatusType：OK, ERROR.
  */
static BASE_StatusType ReadData(unsigned char *recvBuf, unsigned int recvAddr, unsigned int len)
{
    unsigned char buffer[124] = {0}; /* The array size is 124 bytes */
    BASE_StatusType ret;
    buffer[0] = SPI_W25X_READ_DATA;           /* Read data command */
    buffer[1] = (recvAddr & 0xFF0000) >> 16;  /* Shift right by 16 bits */
    buffer[2] = (recvAddr & 0xFF00) >> 8;     /* Shift right by 8 bits, 2 array subscript */
    buffer[3] = (recvAddr & 0xFF);            /* Lower 8 bits of the sector erase address, 3 array subscript */

    unsigned char readBuf[124] = {0};
    /* len + 4 is total size */
    ret = HAL_SPI_WriteReadBlocking(&g_spiSampleHandle, readBuf, (unsigned char *)buffer, (len + 4), MAX_TIMEOUT_VAL);
    if (ret != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    for (unsigned int i = 0; i < len; i++) {
        recvBuf[i] = readBuf[i + 4]; /* read valid data from array subscript 4 */
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Performs read data and write page on w25q32.
  * @param None.
  * @retval None.
  */
static void SPI_WritePageAndReadData(void)
{
    BASE_StatusType ret;
    DBG_PRINTF("write data is : \r\n");
    unsigned int writeLen = 120; /* transfer 120 bytes of data */
    for (unsigned int i = 0; i < writeLen; i++) {
        g_writeBuf[i] = i;
        DBG_PRINTF("%d ", g_writeBuf[i]);
    }
    DBG_PRINTF("\r\n");
    ret = WritePage(g_writeBuf, g_flashAddr, writeLen); /* w25q32 single page write */
    if (ret != BASE_STATUS_OK) {
        DBG_PRINTF("write page failed!\r\n");
    }

    BASE_FUNC_DELAY_MS(5); /* Delay 5ms */

    unsigned int recvLen = 120; /* 120 bytes of data needs to be received */
    for (unsigned int index = 0; index < recvLen; index++) {
        g_recvBuf[index] = 0;
    }
    ret = ReadData(g_recvBuf, g_flashAddr, recvLen); /* Read data in the g_flashAddr address */
    if (ret != BASE_STATUS_OK) {
        DBG_PRINTF("read page failed!\r\n");
    } else {
        DBG_PRINTF("read data is : \r\n");
        for (unsigned int i = 0; i < recvLen; i++) {
            DBG_PRINTF("%d ", g_recvBuf[i]); /* print received Data */
        }
        DBG_PRINTF("\r\n");
    }
}

/**
  * @brief Performs read and write operations on w25q32.
  * @param None.
  * @retval None.
  */
static void SPI_W25Q32_TEST(void)
{
    BASE_StatusType ret;
    ret = SectorErase(g_flashAddr); /* w25q32 sector erase function */
    if (ret != BASE_STATUS_OK) {
        DBG_PRINTF("sector erase failed!\r\n");
    }
    BASE_FUNC_DELAY_MS(500); /* Delay 500 ms */

    SPI_WritePageAndReadData();

    bool flag = true;
    unsigned int recvLen = 120;
    for (unsigned int i = 0; i < recvLen; i++) {
        if (g_recvBuf[i] != g_writeBuf[i]) { /* compare the transmitted data with the received data */
            DBG_PRINTF("data err! \r\n");
            flag = false;
            break;
        }
    }
    if (flag) {
        DBG_PRINTF("data correct \r\n");
    }
    DBG_PRINTF("\r\n");
}

/**
  * @brief Spi blocking sample w25q32 processing.
  * @param None.
  * @retval None.
  */
void W25Q32BlockingSampleProcessing(void)
{
    SystemInit();
    HAL_SPI_SetChipConfigSelectEx(&g_spiSampleHandle, HAL_SPI_CHIP_CONFIG_SOFR_SET);
    HAL_SPI_RegisterCallback(&g_spiSampleHandle, SPI_CS_CB_ID, SPICsCallback);
    while (1) {
        SPI_W25Q32_TEST();
        BASE_FUNC_DELAY_MS(1000); /* Delay 1000 ms */
    }
}
