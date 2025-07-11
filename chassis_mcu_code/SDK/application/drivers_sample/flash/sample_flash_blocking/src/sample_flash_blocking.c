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
  * @file    sample_flash_blocking.c
  * @author  MCU Driver Team
  * @brief   Sample for Flash Module Blocking.
  * @details This file provides sample code for users to operate the flash memory in blocking mode.
  *          This example code operates on page15 of the flash memory. (Be careful when performing the flash erasing
  *          operation to avoid damaging the running program code).
  *          This sample code demonstrates how to use the module initialization, read, write, and erase interfaces by
  *          operating on page 15 of the flash memory. The operation data is printed and output through the serial port.
  */
#include "main.h"
#include "flash.h"
#include "debug.h"
#include "sample_flash_blocking.h"

#define FLASH_TEMP_DATA_SIZE    4096

#define FLASH_SAMPLE_ERASE_NUM                1               /* Number of flash pages erased at a time */
#define FLASH_SAMPLE_ERASE_START_ADDR         FLASH_PAGE_9   /* The erase address must be 8k aligned.  */
#define FLASH_SAMPLE_READ_START_ADDR          FLASH_PAGE_9   /* The read address must be byte-aligned. */
#define FLASH_SAMPLE_WRITE_START_ADDR         FLASH_PAGE_9   /* The write address must be 16-byte aligned. */

static FLASH_Handle g_flashBlockingHandle;
static unsigned char g_tempData[FLASH_TEMP_DATA_SIZE] = {0};

/**
  * @brief Flash bloccking sample init.
  * @param None
  * @retval None
  */
static void FlashBlockingInit(void)
{
    g_flashBlockingHandle.baseAddress = EFC;
    g_flashBlockingHandle.peMode = FLASH_PE_OP_BLOCK;
    HAL_FLASH_Init(&g_flashBlockingHandle);
}

/**
  * @brief Flash bloccking sample processing.
  * @param None
  * @retval None
  */
void FlashBlockingProcessing(void)
{
    BASE_StatusType ret;
    unsigned char dataBuff[FLASH_TEMP_DATA_SIZE + 1] = {0};

    SystemInit();
    FlashBlockingInit();
    for (unsigned int i = 0 ; i < FLASH_TEMP_DATA_SIZE; i++) {
        g_tempData[i] = 0xA5;
    }
    
    ret = HAL_FLASH_EraseBlocking(&g_flashBlockingHandle, FLASH_ERASE_MODE_PAGE,
                                  FLASH_SAMPLE_ERASE_START_ADDR, FLASH_SAMPLE_ERASE_NUM);
    if (ret != BASE_STATUS_OK) {
        DBG_PRINTF("Erase:0x%x  fail\r\n ", FLASH_SAMPLE_ERASE_START_ADDR);
    }

    ret = HAL_FLASH_WriteBlocking(&g_flashBlockingHandle, (uintptr_t)g_tempData,
                                  FLASH_SAMPLE_WRITE_START_ADDR, FLASH_TEMP_DATA_SIZE);
    if (ret != BASE_STATUS_OK) {
        DBG_PRINTF("Write:0x%x  fail\r\n ", FLASH_SAMPLE_WRITE_START_ADDR);
    }

    ret = HAL_FLASH_Read(&g_flashBlockingHandle, FLASH_SAMPLE_READ_START_ADDR, FLASH_TEMP_DATA_SIZE,
                         dataBuff, FLASH_TEMP_DATA_SIZE);
    if (ret != BASE_STATUS_OK) {
        DBG_PRINTF("Read:0x%x  fail\r\n ", FLASH_TEMP_DATA_SIZE);
    }
    DBG_PRINTF("read addr :0x%x \r\n ", FLASH_SAMPLE_READ_START_ADDR);
    for (unsigned int i = 0; i < FLASH_TEMP_DATA_SIZE; i++) {
        DBG_PRINTF("%x ", dataBuff[i]);
    }
    DBG_PRINTF("\r\n");
    while (1) {
        ;
    }
}