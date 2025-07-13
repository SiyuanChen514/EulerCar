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
  * @file    sample_flash_interrupt.c
  * @author  MCU Driver Team
  * @brief   Sample for Flash Module Interrupt.
  * @details This file provides sample code for users to operate the flash memory in Interrupt mode.
  *          This example code operates on page15 of the flash memory. (Be careful when performing the flash erasing
  *          operation to avoid damaging the running program code).
  *          This sample code demonstrates how to use the module initialization, read, write, and erase interfaces by
  *          operating on page 15 of the flash memory. The operation data is printed and output through the serial port.
  */
#include "main.h"
#include "flash.h"
#include "debug.h"
#include "sample_flash_interrupt.h"

#define FLASH_SAMPLE_FLAG_UNSET   0
#define FLASH_SAMPLE_FLAG_SET     1

#define FLASH_TEMP_DATA_SIZE    4096

#define FLASH_SAMPLE_ERASE_NUM                1               /* Number of flash pages erased at a time */
#define FLASH_SAMPLE_ERASE_START_ADDR         FLASH_PAGE_9   /* The erase address must be 8k aligned.  */
#define FLASH_SAMPLE_READ_START_ADDR          FLASH_PAGE_9   /* The read address must be byte-aligned. */
#define FLASH_SAMPLE_WRITE_START_ADDR         FLASH_PAGE_9   /* The write address must be 16-byte aligned. */

static FLASH_Handle g_flashInterruptHandle;
static unsigned char g_tempData[FLASH_TEMP_DATA_SIZE] = {0};
static volatile unsigned int g_eraseDoneFlag = FLASH_SAMPLE_FLAG_UNSET;
static volatile unsigned int g_writeDoneFlag = FLASH_SAMPLE_FLAG_UNSET;

/**
  * @brief Flash interrupt sample handle.
  * @param handle Flash handle.
  * @param event Flash callback event.
  * @param opAddr Current operation address.
  * @retval None
  */
static void FlashInterruptHandle(void *handle, FLASH_CallBackEvent event, unsigned int opAddr)
{
    BASE_FUNC_UNUSED(handle);
    switch (event) {
        case FLASH_WRITE_EVENT_SUCCESS : /* One-time write success callback. */
            DBG_PRINTF("write success \r\n 0x%x \r\n", opAddr);
            break;
        case FLASH_WRITE_EVENT_DONE : /* All content is written. */
            g_writeDoneFlag = FLASH_SAMPLE_FLAG_SET;
            DBG_PRINTF("write done \r\n");
            break;
        case FLASH_WRITE_EVENT_FAIL : /* Write failed. */
            DBG_PRINTF("wtite failed\r\n 0x%x \r\n", opAddr);
            break;
        case FLASH_ERASE_EVENT_SUCCESS : /* One-time erase success callback. */
            DBG_PRINTF("erase success \r\n 0x%x \r\n", opAddr);
            break;
        case FLASH_ERASE_EVENT_DONE: /* All erase operations are complete. */
            g_eraseDoneFlag = FLASH_SAMPLE_FLAG_SET;
            DBG_PRINTF("erase done \r\n");
            break;
        case FLASH_ERASE_EVENT_FAIL : /* Erase failed. */
            DBG_PRINTF("erase failed\r\n 0x%x \r\n", opAddr);
            break;
        default :
            break;
    }
}

/**
  * @brief Flash interrupt sample init.
  * @param None
  * @retval None
  */
static void FlashInterruptInit(void)
{
    g_flashInterruptHandle.baseAddress = EFC;
    g_flashInterruptHandle.peMode = FLASH_PE_OP_IT;
    HAL_FLASH_Init(&g_flashInterruptHandle);
    HAL_FLASH_RegisterCallback(&g_flashInterruptHandle, FlashInterruptHandle);

    IRQ_Register(IRQ_EFC, HAL_FLASH_IrqHandler, &g_flashInterruptHandle);
    IRQ_Register(IRQ_EFC_ERR, HAL_FLASH_IrqHandlerError, &g_flashInterruptHandle);
    IRQ_SetPriority(IRQ_EFC_ERR, 1); /* set flash interrupt priority to 1 */
    IRQ_SetPriority(IRQ_EFC, 1);     /* set flash interrupt priority to 1 */
    IRQ_EnableN(IRQ_EFC);
    IRQ_EnableN(IRQ_EFC_ERR);
}

/**
  * @brief Flash interrupt sample processing.
  * @param None
  * @retval None
  */
void FlashInterruptProcessing(void)
{
    BASE_StatusType ret;
    unsigned char dataBuff[FLASH_TEMP_DATA_SIZE + 1] = {0};

    SystemInit();
    FlashInterruptInit();
    for (unsigned int i = 0 ; i < FLASH_TEMP_DATA_SIZE; i++) {
        g_tempData[i] = 0x5A;
    }

    ret = HAL_FLASH_EraseIT(&g_flashInterruptHandle, FLASH_ERASE_MODE_PAGE,
                            FLASH_SAMPLE_ERASE_START_ADDR, FLASH_SAMPLE_ERASE_NUM);
    if (ret != BASE_STATUS_OK) {
        DBG_PRINTF("Erase:0x%x  fail\r\n ", FLASH_SAMPLE_ERASE_START_ADDR);
    }
    while (g_eraseDoneFlag == FLASH_SAMPLE_FLAG_UNSET) {
        ;
    }

    ret = HAL_FLASH_WriteIT(&g_flashInterruptHandle, (uintptr_t)g_tempData,
                            FLASH_SAMPLE_WRITE_START_ADDR, FLASH_TEMP_DATA_SIZE);
    if (ret != BASE_STATUS_OK) {
        DBG_PRINTF("Write:0x%x  fail\r\n ", FLASH_SAMPLE_WRITE_START_ADDR);
    }
    while (g_writeDoneFlag == FLASH_SAMPLE_FLAG_UNSET) {
        ;
    }
    ret = HAL_FLASH_Read(&g_flashInterruptHandle, FLASH_SAMPLE_READ_START_ADDR, FLASH_TEMP_DATA_SIZE,
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