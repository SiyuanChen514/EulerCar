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
  * @file    flash.c
  * @author  MCU Driver Team
  * @brief   FLASH module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the FLASH.
  *          + Initialization and de-initialization functions.
  *          + Read, write, and erase functions.
  */

/* Includes ------------------------------------------------------------------ */
#include "flash.h"

#define FLASH_CRC_SAVE_BUFFER_LEN   2
#define FLASH_ALL_INTERRUPT_ENABLE  0x001F0010
#define FLASH_ERR_INTERRUPT_MASK    0x001F0000
#define FLASH_CMD_INTERRUPT_MASK    0x00000010

#define FLASH_KEY_REGISTER_UNLOCK_VALUE   0xFEDCBA98
#define FLASH_KEY_REGISTER_LOCK_VALUE     0x0

#define FLASH_INT_ERR_ECC_CHK_MASK      (1 << 20)
#define FLASH_INT_ERR_ECC_CORR_MASK     (1 << 19)
#define FLASH_INT_ERR_AHB_MASK          (1 << 18)
#define FLASH_INT_ERR_SMWR_MASK         (1 << 17)
#define FLASH_INT_ERR_ILLEGAL_MASK      (1 << 16)
#define FLASH_INT_FINISH_MASK           (1<< 4)

#define FLASH_INT_CLEAR_ALL             0xFFFFFFFF

/**
  * @brief Check whether errors occur.
  * @param handle FLASH handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType CheckErrorStatus(FLASH_Handle *handle)
{
    /* Check whether errors occur. */
    if (handle->baseAddress->INT_RAW_STATUS.BIT.int_raw_err_illegal ||
        handle->baseAddress->INT_RAW_STATUS.BIT.int_raw_err_erase) {
        return BASE_STATUS_ERROR;
    }
    if (handle->baseAddress->FLASH_STATUS.reg != 0) {
        return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Writes to the flash memory in the unit of words.
  * @param handle FLASH handle.
  * @param wordNum Number of size written, unit: word.
  * @retval None.
  */
static void FlashPopulateDefaults(FLASH_Handle *handle, const unsigned int wordNum)
{
    /* Complement missing data values. */
    if (wordNum % FLASH_MIN_PGM_WORDS_SIZE) {
        for (unsigned int i = (wordNum % FLASH_MIN_PGM_WORDS_SIZE); i < FLASH_MIN_PGM_WORDS_SIZE; i++) {
            handle->baseAddress->PGM_WDATA = 0xFFFFFFFF; /* The default value of flash is 0xFFFFFFFF. */
        }
    }
}

/**
  * @brief Writes to the flash memory in the unit of words.
  * @param handle FLASH handle.
  * @param srcAddr Start address of the data buffer to be written.
  * @param destAddr Flash destination address, which must be word-aligned.
  * @param size Number of size written, unit: byte.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType FLASH_WriteWords(FLASH_Handle *handle,
                                        const unsigned int srcAddr,
                                        const unsigned int destAddr,
                                        const unsigned int size)
{
    unsigned int *data = NULL;
    unsigned int i;
    unsigned int writeSize;
    unsigned int wordNum;
    /* Make sure the last operation is complete. */
    if (handle->baseAddress->EFLASH_CMD.BIT.cmd_start) {
        return BASE_STATUS_BUSY;
    }

    /* Get the number of last remaining data. */
    wordNum = size / FLASH_ONE_WORD_BYTES_SIZE;
    wordNum += ((size % FLASH_ONE_WORD_BYTES_SIZE) == 0) ? 0 : 1;

    handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_UNLOCK_VALUE;
    /* The mask of program wdata celar is 0xFF. */
    if ((handle->baseAddress->BUF_CLEAR.reg >> FLASH_PGM_WBUF_CNT_POS) & 0xFF) {
        handle->baseAddress->BUF_CLEAR.BIT.pgm_wdata_clr = BASE_CFG_SET; /* program wdata celar enable. */
    }

    /* Step 1: Calculated the cmd program size, get srcAddress and get destAddress. */
    writeSize = ((wordNum % FLASH_MIN_PGM_WORDS_SIZE) != 0) ? (wordNum / FLASH_MIN_PGM_WORDS_SIZE + 1) :
                wordNum / FLASH_MIN_PGM_WORDS_SIZE;
    data = (unsigned int *)(uintptr_t)srcAddr;
    handle->baseAddress->EFLASH_ADDR.BIT.cmd_addr = destAddr;
    for (i = 0; i < wordNum; i++) {
        handle->baseAddress->PGM_WDATA = *data;
        data++;
    }
    /* Complement missing data values. */
    FlashPopulateDefaults(handle, wordNum);

    /* Step 2: Configure the parameters and start programming. */
    handle->baseAddress->EFLASH_CMD.BIT.cmd_pgm_size = writeSize;
    handle->baseAddress->EFLASH_CMD.BIT.cmd_code = FLASH_OPERATION_PROGRAM;
    handle->baseAddress->EFLASH_CMD.BIT.cmd_start = BASE_CFG_SET;

    /* Step 3: If the blocking mode is used, wait until the program operation is complete. */
    if (handle->peMode == FLASH_PE_OP_BLOCK) {
        while (handle->baseAddress->EFLASH_CMD.BIT.cmd_start) {
            ;
        }
        if (CheckErrorStatus(handle) != BASE_STATUS_OK) {
            /* Clears data in the cache and clears the interrupt flag. */
            handle->baseAddress->CACHE_CTRL.BIT.cache_invalid_req = BASE_CFG_SET;
            handle->baseAddress->INT_CLEAR.reg = FLASH_INT_CLEAR_ALL;
            handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_LOCK_VALUE;
            return BASE_STATUS_ERROR;
        }
        handle->baseAddress->CACHE_CTRL.BIT.cache_invalid_req = BASE_CFG_SET;
        handle->baseAddress->INT_CLEAR.reg = FLASH_INT_CLEAR_ALL;
    }
    handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_LOCK_VALUE;
    return BASE_STATUS_OK;
}

/**
  * @brief Obtains the number of words to be supplemented for write alignment to prevent cross-page write.
  * @param handle FLASH handle.
  * @retval Words.
  */
static unsigned int FLASH_GetWriteAlignmentWords(FLASH_Handle *handle)
{
    unsigned int numWords;
    /* Step 1: Calculate the number of words occupied at the start address. */
    numWords = handle->destAddr % FLASH_MAX_PGM_WORD_SIZE;
    if (numWords > 0) {
        /* Step 2: Calculate the number of words in the remaining space of the ROW. */
        return FLASH_MAX_PGM_WORD_SIZE - numWords;
    }
    return 0;
}

/**
  * @brief Flash erase operation.
  * @param handle FLASH handle.
  * @param startAddr Erasing start address, which must be aligned with the minimum erasing unit.
  * @param mode Erasing operation mode, supporting chip,and page.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType FLASH_EraseWithMode(FLASH_Handle *handle, unsigned int startAddr, unsigned int mode)
{
    /* Make sure the last operation is complete. */
    if (handle->baseAddress->EFLASH_CMD.BIT.cmd_start) {
        return BASE_STATUS_BUSY;
    }

    handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_UNLOCK_VALUE;

    /* Step 1: Configure the erase start address and erase mode, then make cmd_satrt enable. */
    handle->baseAddress->EFLASH_ADDR.BIT.cmd_addr = startAddr;
    handle->baseAddress->EFLASH_CMD.BIT.cmd_code  = mode;
    handle->baseAddress->EFLASH_CMD.BIT.cmd_start = BASE_CFG_SET;

    /* Step 2: If the blocking mode is used, wait until the erase operation is complete. */
    if (handle->peMode == FLASH_PE_OP_BLOCK) {
        while (handle->baseAddress->EFLASH_CMD.BIT.cmd_start) {
            ;
        }
        /* Check whether errors occur. */
        if (handle->baseAddress->INT_RAW_STATUS.BIT.int_raw_err_illegal ||
            handle->baseAddress->INT_RAW_STATUS.BIT.int_raw_err_erase) {
            handle->baseAddress->CACHE_CTRL.BIT.cache_invalid_req = BASE_CFG_SET;
            handle->baseAddress->INT_CLEAR.reg = FLASH_INT_CLEAR_ALL;
            handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_LOCK_VALUE;
            return BASE_STATUS_ERROR;
        }
        if (handle->baseAddress->FLASH_STATUS.reg != 0) {
            handle->baseAddress->CACHE_CTRL.BIT.cache_invalid_req = BASE_CFG_SET;
            handle->baseAddress->INT_CLEAR.reg = FLASH_INT_CLEAR_ALL;
            handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_LOCK_VALUE;
            return BASE_STATUS_ERROR;
        }
    }
    handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_LOCK_VALUE;
    return BASE_STATUS_OK;
}

/**
  * @brief Write interrupt processing function,
  *        which completes the internal processing of the write operation in interrupt mode.
  * @param handle FLASH handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType FLASH_WriteHandler(FLASH_Handle *handle)
{
    unsigned int dataLeft;
    unsigned int tempAddr;
    BASE_StatusType ret;
    /* If the number of bytes to be written is greater than a Row,
       data is written based on the bytes number of a row. */
    if ((handle->writeLen / FLASH_MAX_PGM_BYTE_SIZE) > 0) {
        ret = FLASH_WriteWords(handle, handle->srcAddr, handle->destAddr, FLASH_MAX_PGM_BYTE_SIZE);
        if (ret != BASE_STATUS_OK) {
            return ret;
        }
        /* Update the value of destination address, source address, and length. */
        handle->srcAddr += FLASH_MAX_PGM_WORD_SIZE * FLASH_ONE_WORD_BYTES_SIZE;
        handle->destAddr += FLASH_MAX_PGM_WORD_SIZE;
        handle->writeLen -= FLASH_MAX_PGM_WORD_SIZE * FLASH_ONE_WORD_BYTES_SIZE;
    } else if (handle->writeLen > 0) {
        /* If the number of bytes to be written is less than a Row,
           data is written in the unit of words. In addition, if data is less than one word, complete one word. */
        dataLeft = handle->writeLen;
        ret = FLASH_WriteWords(handle, handle->srcAddr, handle->destAddr, dataLeft);
        if (ret != BASE_STATUS_OK) {
            return ret;
        }
        /* Updata the srcAddress, destAddress and write length. */
        handle->writeLen = 0;
        tempAddr = (dataLeft / FLASH_ONE_WORD_BYTES_SIZE);
        if (tempAddr == 0) {
            tempAddr = FLASH_ONE_WORD_BYTES_SIZE;
        } else { /* Get the address change value, aligned with 4words. */
            tempAddr = ((tempAddr % FLASH_ONE_WORD_BYTES_SIZE) == 0) ? tempAddr :
                       ((tempAddr / FLASH_ONE_WORD_BYTES_SIZE + 1) * FLASH_ONE_WORD_BYTES_SIZE);
        }
        handle->destAddr += tempAddr;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Erase interrupt processing function,
  *        which completes the internal processing of the erase operation in interrupt mode.
  * @param handle FLASH handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType FLASH_EraseHandler(FLASH_Handle *handle)
{
    /* Check whether the erasing mode is valid. */
    FLASH_PARAM_CHECK_WITH_RET((handle->destAddr <= (FLASH_PAGE_MAX / FLASH_ONE_WORD_BYTES_SIZE)), BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET((handle->destAddr % (FLASH_ONE_PAGE_WORD_SIZE) == 0), BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET((((FLASH_MAX_PAGE_NUM - (handle->destAddr / FLASH_ONE_PAGE_WORD_SIZE)) >=\
                               handle->eraseNum) && handle->eraseNum > 0), BASE_STATUS_ERROR);

    BASE_StatusType ret;
    ret = FLASH_EraseWithMode(handle, handle->destAddr, FLASH_ERASE_MODE_PAGE);
    if (ret != BASE_STATUS_OK) {
        return ret;
    }
    /* Updata the erase destAddress and erase number. */
    handle->destAddr += FLASH_ONE_PAGE_SIZE / sizeof(unsigned int);
    handle->eraseNum--;
    return BASE_STATUS_OK;
}

/**
  * @brief Initializing the FLASH Module.
  * @param handle FLASH handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_FLASH_Init(FLASH_Handle *handle)
{
    FLASH_ASSERT_PARAM(handle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(handle->baseAddress));
    FLASH_PARAM_CHECK_WITH_RET(IsFlashOperationMode(handle->peMode), BASE_STATUS_ERROR);

    handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_UNLOCK_VALUE; /* Unlock key registers */
    if (handle->peMode == FLASH_PE_OP_IT) {
        /* Enable the interrupt mode and clear the interrupt flag bit. */
        handle->baseAddress->CMD_CFG_COMMON.BIT.int_mode = BASE_CFG_SET;
        handle->baseAddress->INT_ENABLE.reg = FLASH_ALL_INTERRUPT_ENABLE;
        handle->baseAddress->INT_CLEAR.reg = FLASH_ALL_INTERRUPT_ENABLE;
    } else {
        /* If blocking mode is used, disable int_mode. */
        handle->baseAddress->CMD_CFG_COMMON.BIT.int_mode = BASE_CFG_UNSET;
    }
    handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_LOCK_VALUE; /* Lock key registers */
    handle->state = FLASH_STATE_READY;
    return BASE_STATUS_OK;
}

/**
  * @brief Deinitialize the FLASH Module.
  * @param handle FLASH handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_FLASH_DeInit(FLASH_Handle *handle)
{
    FLASH_ASSERT_PARAM(handle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(handle->baseAddress));
    handle->state = FLASH_STATE_RESET;
    handle->userCallBack.FlashCallBack = NULL; /* Clean interrupt callback functions. */
    handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_UNLOCK_VALUE;
    /* Disable interrupt mode and interrupt enable bit. */
    handle->baseAddress->CMD_CFG_COMMON.BIT.int_mode = BASE_CFG_UNSET;
    handle->baseAddress->INT_ENABLE.reg = 0x00000000;
    handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_LOCK_VALUE; /* Locking Key Registers */
    return BASE_STATUS_OK;
}

/**
  * @brief Registering the Callback Function of the Flash Module.
  * @param handle FLASH handle.
  * @param pcallback Pointer to the callback function.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_FLASH_RegisterCallback(FLASH_Handle *handle, FLASH_CallbackFunType pcallback)
{
    FLASH_ASSERT_PARAM(handle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(handle->baseAddress));
    handle->userCallBack.FlashCallBack = pcallback;
    return BASE_STATUS_OK;
}

/**
 * @brief blocking write error handle.
 * @param handle FLASH handle.
 * @retval None.
 */
static void FLASH_WritteBlockingErrorHandle(FLASH_Handle *handle)
{
    handle->baseAddress->CACHE_CTRL.BIT.cache_invalid_req = BASE_CFG_SET;
    handle->baseAddress->INT_CLEAR.reg = FLASH_INT_CLEAR_ALL;
    handle->state = FLASH_STATE_READY;
}

/**
  * @brief Write the flash memory in blocking mode.
  * @param handle FLASH handle.
  * @param srcAddr Start address of the data buffer to be written.
  * @param destAddr Start address of the flash to be written.The address must be aligned with the minimum writable unit.
  * @param srcLen Length of data to be written,unit:bytes.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_FLASH_WriteBlocking(FLASH_Handle *handle, unsigned int srcAddr,
                                        unsigned int destAddr, const unsigned int srcLen)
{
    BASE_StatusType ret;
    unsigned int i;
    unsigned int currentLen;
    unsigned int currentWords;
    unsigned int dataLeft;
    /* Check related parameters. */
    FLASH_ASSERT_PARAM(handle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(handle->baseAddress));
    FLASH_PARAM_CHECK_WITH_RET((handle->peMode == FLASH_PE_OP_BLOCK), BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET(handle->state == FLASH_STATE_READY, BASE_STATUS_ERROR);
    /* Check whether the write address is valid. */
    FLASH_PARAM_CHECK_WITH_RET(IsFlashWriteSrcAddress(srcAddr), BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET((destAddr < FLASH_MAX_SIZE), BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET((destAddr % FLASH_MIN_PGM_BYTES_SIZE) == 0, BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET(srcLen > 0, BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET(srcLen <= (FLASH_MAX_SIZE - destAddr), BASE_STATUS_ERROR);

    handle->state = FLASH_STATE_PGM;
    handle->destAddr = destAddr / FLASH_ONE_WORD_BYTES_SIZE; /* Convert the destination address unit to word. */
    handle->srcAddr = srcAddr;

    /* Get the number of words in the remaining space of the ROW. */
    currentWords = FLASH_GetWriteAlignmentWords(handle);
    /* Step 1: If there is remaining space and write length greater than remaining space,
       write data in the remaining space. */
    if (srcLen > (currentWords * FLASH_ONE_WORD_BYTES_SIZE) && currentWords > 0) {
        ret = FLASH_WriteWords(handle, handle->srcAddr, handle->destAddr, (currentWords * FLASH_ONE_WORD_BYTES_SIZE));
        if (ret != BASE_STATUS_OK) {
            FLASH_WritteBlockingErrorHandle(handle);
            return ret;
        }
        handle->srcAddr += currentWords * FLASH_ONE_WORD_BYTES_SIZE;
        handle->destAddr += currentWords;
        currentLen = srcLen - currentWords * FLASH_ONE_WORD_BYTES_SIZE;
    } else {
        currentLen = srcLen;
    }
    /* Step 2: If the number of bytes to be written is greater than a Row,
       data is written based on the bytes number of a row. */
    for (i = 0; i < currentLen / FLASH_MAX_PGM_BYTE_SIZE; i++) {
        ret = FLASH_WriteWords(handle, handle->srcAddr, handle->destAddr, FLASH_MAX_PGM_BYTE_SIZE);
        if (ret != BASE_STATUS_OK) {
            FLASH_WritteBlockingErrorHandle(handle);
            return ret;
        }
        handle->srcAddr += FLASH_MAX_PGM_WORD_SIZE * FLASH_ONE_WORD_BYTES_SIZE;
        handle->destAddr += FLASH_MAX_PGM_WORD_SIZE;
    }
    
    /* Get the number of last remaining data. */
    dataLeft = currentLen % FLASH_MAX_PGM_BYTE_SIZE;
    if (dataLeft > 0) { /* This branch is executed only when the remaining data is not completely written. */
        ret = FLASH_WriteWords(handle, handle->srcAddr, handle->destAddr, dataLeft);
    }

    FLASH_WritteBlockingErrorHandle(handle);
    return ret;
}

/**
  * @brief WriteErase the flash memory in blocking mode.
  * @param handle FLASH handle.
  * @param eraseMode Erasing mode. The options are chip erasing and page erasing.
  * @param startAddr Start address of the flash to be erase. The address must be aligned with the minimum erasable unit.
  * @param eraseNum Number of pages to be erased.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_FLASH_EraseBlocking(FLASH_Handle *handle, FLASH_EraseMode eraseMode,
                                        FLASH_SectorAddr startAddr, unsigned int eraseNum)
{
    /* Check related parameters. */
    FLASH_ASSERT_PARAM(handle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(handle->baseAddress));
    FLASH_PARAM_CHECK_WITH_RET((handle->peMode == FLASH_PE_OP_BLOCK), BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET(handle->state == FLASH_STATE_READY, BASE_STATUS_ERROR);
    /* Check whether the erasing mode is valid. */
    FLASH_PARAM_CHECK_WITH_RET(IsFlashEraseMode(eraseMode), BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET((startAddr <= FLASH_PAGE_MAX), BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET((startAddr % FLASH_ONE_PAGE_SIZE == 0) || (eraseMode == FLASH_ERASE_MODE_CHIP),\
                               BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET(eraseNum > 0 && eraseNum <= (FLASH_MAX_PAGE_NUM - startAddr / FLASH_ONE_PAGE_SIZE),\
                               BASE_STATUS_ERROR);

    BASE_StatusType ret = BASE_STATUS_OK;

    handle->eraseNum = eraseNum;
    handle->destAddr = startAddr / sizeof(unsigned int); /* Convert the destination address unit to word. */
    handle->state = FLASH_STATE_ERASE;

    if (eraseMode == FLASH_ERASE_MODE_CHIP) {
        /* If the FLASH_ERASE_MODE_CHIP mode is used, all contents in the flash memory are erased. */
        ret = FLASH_EraseWithMode(handle, handle->destAddr, FLASH_ERASE_MODE_CHIP);
        if (ret != BASE_STATUS_OK) {
            handle->baseAddress->CACHE_CTRL.BIT.cache_invalid_req = BASE_CFG_SET;
            handle->state = FLASH_STATE_READY;
            return ret;
        }
    } else if (eraseMode == FLASH_ERASE_MODE_PAGE) {
        /* If the FLASH_ERASE_MODE_PAGE mode is used, erasing requires page-by-page. */
        while (handle->eraseNum) {
            ret = FLASH_EraseHandler(handle);
            if (ret != BASE_STATUS_OK) {
                handle->baseAddress->CACHE_CTRL.BIT.cache_invalid_req = BASE_CFG_SET;
                handle->state = FLASH_STATE_READY;
                return ret;
            }
        }
    }
    /* Clear the data in the cache to invalidate the data. */
    handle->baseAddress->CACHE_CTRL.BIT.cache_invalid_req = BASE_CFG_SET;
    handle->state = FLASH_STATE_READY;
    return ret;
}

/**
  * @brief Write the flash memory in interrupt mode.
  * @param handle FLASH handle.
  * @param srcAddr Start address of the data buffer to be written.
  * @param destAddr Start address of the flash to be written.The address must be aligned with the minimum writable unit.
  * @param srcLen Length of data to be written,unit:bytes.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_FLASH_WriteIT(FLASH_Handle *handle, unsigned int srcAddr,
                                  unsigned int destAddr, unsigned int srcLen)
{
    unsigned int currentWords;
    BASE_StatusType ret;
    /* Check the validity of the base address and operation mode of the flash memory. */
    FLASH_ASSERT_PARAM(handle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(handle->baseAddress));
    FLASH_PARAM_CHECK_WITH_RET((handle->peMode == FLASH_PE_OP_IT), BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET(handle->state == FLASH_STATE_READY, BASE_STATUS_ERROR);
    /* Check whether the write address is valid. */
    FLASH_PARAM_CHECK_WITH_RET(IsFlashWriteSrcAddress(srcAddr), BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET((destAddr < FLASH_MAX_SIZE), BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET((destAddr % FLASH_MIN_PGM_BYTES_SIZE) == 0, BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET((srcLen > 0), BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET(srcLen <= (FLASH_MAX_SIZE - destAddr), BASE_STATUS_ERROR);

    handle->state = FLASH_STATE_PGM;
    handle->destAddr = destAddr / FLASH_ONE_WORD_BYTES_SIZE; /* Convert the destination address unit to word. */
    handle->srcAddr = srcAddr;
    handle->writeLen = srcLen;

    /* Get the number of words in the remaining space of the ROW. */
    currentWords = FLASH_GetWriteAlignmentWords(handle);
    /* If there is remaining space and write length greater than remaining space,
       write data in the remaining space. */
    if (handle->writeLen > (currentWords * FLASH_ONE_WORD_BYTES_SIZE) && currentWords > 0) {
        ret = FLASH_WriteWords(handle, handle->srcAddr, handle->destAddr, (currentWords * FLASH_ONE_WORD_BYTES_SIZE));
        if (ret != BASE_STATUS_OK) {
            handle->state = FLASH_STATE_READY;
            return ret;
        }
        handle->srcAddr += currentWords * FLASH_ONE_WORD_BYTES_SIZE;
        handle->destAddr += currentWords;
        handle->writeLen -= currentWords * FLASH_ONE_WORD_BYTES_SIZE;
    } else {
        /* Write the last remaining data. */
        ret = FLASH_WriteHandler(handle);
        if (ret != BASE_STATUS_OK) {
            handle->state = FLASH_STATE_READY;
            return ret;
        }
    }
    return BASE_STATUS_OK;
}

/**
  * @brief WriteErase the flash memory in interrupt mode.
  * @param handle FLASH handle.
  * @param eraseMode Erasing mode. The options are chip erasing and page erasing.
  * @param startAddr Start address of the flash to be erase. The address must be aligned with the minimum erasable unit.
  * @param eraseNum Number of pages to be erased.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_FLASH_EraseIT(FLASH_Handle *handle, FLASH_EraseMode eraseMode,
                                  FLASH_SectorAddr startAddr, unsigned int eraseNum)
{
    BASE_StatusType ret = BASE_STATUS_OK;
    FLASH_ASSERT_PARAM(handle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(handle->baseAddress));
    FLASH_PARAM_CHECK_WITH_RET((handle->peMode == FLASH_PE_OP_IT), BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET((handle->state == FLASH_STATE_READY), BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET(IsFlashEraseMode(eraseMode), BASE_STATUS_ERROR);
    /* Check whether the address is valid. */
    FLASH_PARAM_CHECK_WITH_RET((startAddr <= FLASH_PAGE_MAX), BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET((startAddr % FLASH_ONE_PAGE_SIZE == 0) || (eraseMode == FLASH_ERASE_MODE_CHIP),\
                               BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET(eraseNum > 0 && eraseNum <= (FLASH_MAX_PAGE_NUM - startAddr / FLASH_ONE_PAGE_SIZE),\
                               BASE_STATUS_ERROR);
    /* Obtains the value entered by the user. */
    handle->eraseNum = eraseNum;
    handle->state = FLASH_STATE_ERASE;
    handle->destAddr = startAddr / sizeof(unsigned int); /* Convert the destination address unit to word. */

    if (eraseMode == FLASH_ERASE_MODE_CHIP) {
        /* If the FLASH_ERASE_MODE_CHIP mode is used, all contents in the flash memory are erased. */
        ret = FLASH_EraseWithMode(handle, handle->destAddr, FLASH_ERASE_MODE_CHIP);
        if (ret != BASE_STATUS_OK) {
            handle->state = FLASH_STATE_READY;
            return ret;
        }
        handle->eraseNum = 0;
    } else if (eraseMode == FLASH_ERASE_MODE_PAGE) {
        /* If the FLASH_ERASE_MODE_PAGE mode is used, erasing requires page-by-page. */
        ret = FLASH_EraseHandler(handle);
        if (ret != BASE_STATUS_OK) {
            handle->state = FLASH_STATE_READY;
            return ret;
        }
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Interface for reading data from the flash memory.
  * @param handle FLASH handle.
  * @param srcAddr Flash address of the data to be read. The address must be aligned with the minimum readable unit.
  * @param readLen Read Data Length,unit:bytes.
  * @param dataBuff Buffer for storing read data.
  * @param buffLen Buffer size for storing read data,unit:bytes.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_FLASH_Read(FLASH_Handle *handle,
                               unsigned int srcAddr,
                               unsigned int readLen,
                               unsigned char *dataBuff,
                               unsigned int buffLen)
{
    unsigned char *ptemp = NULL;
    unsigned char *dtemp = NULL;
    unsigned int tempLen = readLen;
#ifndef FLASH_PARAM_CHECK
    BASE_FUNC_UNUSED(handle); /* Used to avoid code check alarm prompts. */
#endif
    FLASH_ASSERT_PARAM(handle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(handle->baseAddress));
    FLASH_ASSERT_PARAM(dataBuff != NULL);
    FLASH_PARAM_CHECK_WITH_RET(srcAddr < FLASH_MAX_SIZE, BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET(readLen <= (FLASH_MAX_SIZE - srcAddr), BASE_STATUS_ERROR);

    dtemp = dataBuff;
    /* The basic offset address needs to be added to srcAddress. */
    ptemp = (unsigned char *)(uintptr_t)srcAddr + FLASH_READ_BASE;
    if (readLen > buffLen) {
        return BASE_STATUS_ERROR;
    }
    while (tempLen > 0) { /* Read data cyclically. */
        tempLen--;
        *dtemp++ = *ptemp++;
    }
    return BASE_STATUS_OK;
}

/**
 * @brief Interrupt Processing Write.
 * @param handle FLASH handle.
 * @param status Interrupt status
 * @retval None
 */
static void InterruptWriteHandle(FLASH_Handle *handle, unsigned int status)
{
    /* Check whether the parameter is valid. */
    FLASH_PARAM_CHECK_NO_RET(IsFlashWriteSrcAddress(handle->srcAddr));
    FLASH_PARAM_CHECK_NO_RET((handle->destAddr % FLASH_MIN_PGM_WORDS_SIZE) == 0);
    FLASH_PARAM_CHECK_NO_RET((handle->destAddr <= (FLASH_MAX_SIZE / FLASH_ONE_WORD_BYTES_SIZE)));
    FLASH_PARAM_CHECK_NO_RET(handle->writeLen <= (FLASH_MAX_SIZE - (handle->destAddr * FLASH_ONE_WORD_BYTES_SIZE)));
    /* One operation complete */
    if ((status & FLASH_INT_FINISH_MASK) > 0) {
        if (handle->userCallBack.FlashCallBack != NULL) {
            handle->userCallBack.FlashCallBack(handle, FLASH_WRITE_EVENT_SUCCESS, handle->destAddr);
        }
    }
    /* All operations are complete. */
    if (handle->writeLen == 0) {
        if (handle->userCallBack.FlashCallBack != NULL) {
            handle->userCallBack.FlashCallBack(handle, FLASH_WRITE_EVENT_DONE, handle->destAddr);
        }
        handle->baseAddress->CACHE_CTRL.BIT.cache_invalid_req = BASE_CFG_SET;
        handle->state = FLASH_STATE_READY;
    } else {
        FLASH_WriteHandler(handle);
    }
}

/**
 * @brief Interrupt processing erase.
 * @param handle FLASH handle.
 * @param status Interrupt status
 * @retval None
 */
static void InterruptEraseHandle(FLASH_Handle *handle, unsigned int status)
{
    /* One operation complete */
    if ((status & FLASH_INT_FINISH_MASK) > 0) {
        if (handle->userCallBack.FlashCallBack != NULL) {
            handle->userCallBack.FlashCallBack(handle, FLASH_ERASE_EVENT_SUCCESS, handle->destAddr);
        }
    }
    /* All operations are complete. */
    if (handle->eraseNum == 0) {
        if (handle->userCallBack.FlashCallBack != NULL) {
            handle->userCallBack.FlashCallBack(handle, FLASH_ERASE_EVENT_DONE, handle->destAddr);
        }
        handle->baseAddress->CACHE_CTRL.BIT.cache_invalid_req = BASE_CFG_SET;
        handle->state = FLASH_STATE_READY;
    } else {
        FLASH_EraseHandler(handle);
    }
}

/**
  * @brief Interrupt Handling Function.
  * @param handle Handle pointers
  * @retval None
  */
void HAL_FLASH_IrqHandler(void *handle)
{
    FLASH_Handle *flashHandle = (FLASH_Handle *)handle;
    unsigned int status;
    FLASH_ASSERT_PARAM(flashHandle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(flashHandle->baseAddress));
    FLASH_PARAM_CHECK_NO_RET(flashHandle->peMode == FLASH_PE_OP_IT);

    status = flashHandle->baseAddress->INT_RAW_STATUS.reg;
    flashHandle->baseAddress->INT_CLEAR.reg = status & FLASH_CMD_INTERRUPT_MASK;
    /* Invoke the function for programming or erasing. */
    if (flashHandle->state == FLASH_STATE_PGM) { /* If state is FLASH_STATE_PGM, call write callback function. */
        InterruptWriteHandle(flashHandle, status);
    } else if (flashHandle->state == FLASH_STATE_ERASE) {
        /* If state is FLASH_STATE_ERASE, call erase callback function. */
        InterruptEraseHandle(flashHandle, status);
    }
}

/**
  * @brief Flash Error interrupt Handling Function.
  * @param handle Handle pointers
  * @retval None
  */
void HAL_FLASH_IrqHandlerError(void *handle)
{
    FLASH_Handle *flashHandle = (FLASH_Handle *)handle;
    unsigned int status;
    FLASH_ASSERT_PARAM(flashHandle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(flashHandle->baseAddress));
    status = flashHandle->baseAddress->INT_RAW_STATUS.reg;
    flashHandle->baseAddress->INT_CLEAR.reg = status & FLASH_ERR_INTERRUPT_MASK;
    
    /* If any error occurs, call the programming error or erase error callback function. */
    if ((status & (FLASH_INT_ERR_ECC_CHK_MASK | FLASH_INT_ERR_ECC_CORR_MASK |
                  FLASH_INT_ERR_AHB_MASK | FLASH_INT_ERR_SMWR_MASK | FLASH_INT_ERR_ILLEGAL_MASK)) > 0) {
        if (flashHandle->userCallBack.FlashCallBack != NULL) {
            switch (flashHandle->state) {
                case FLASH_STATE_PGM :  /* If state is FLASH_STATE_PGM, call write error callback function. */
                    flashHandle->userCallBack.FlashCallBack(flashHandle, FLASH_WRITE_EVENT_FAIL, flashHandle->destAddr);
                    break;
                case FLASH_STATE_ERASE : /* If state is FLASH_STATE_ERASE, call erase error callback function. */
                    flashHandle->userCallBack.FlashCallBack(flashHandle, FLASH_ERASE_EVENT_FAIL, flashHandle->destAddr);
                    break;
                default:
                    break;
            }
        }
        flashHandle->state = FLASH_STATE_READY;
    }
}