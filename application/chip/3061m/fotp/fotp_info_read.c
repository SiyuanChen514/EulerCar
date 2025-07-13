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
  * @file      fotp_info_read.c
  * @author    MCU Driver Team
  * @brief     This file provides firmware functions to manage the following
  *            functionalities of the fotp control register.
  *                + FOTP INFO Read API
  */
#include "chipinc.h"
#include "flash.h"
#include "fotp_info_read.h"
#define FOTP_INFO_RNG0_BASEADDR 0x800000
#define FOTP_INFO_RNG1_BASEADDR 0x801000
#define REG_WORDS_NUM           16
#define FLASH_READ_128BIT       1

/**
  * @brief Read Four words of FOTP.
  * @param efc  Flash control register base address
  * @retval BASE_STATUS_ERROR fail.
  * @retval BASE_STATUS_OK success.
  */
static unsigned int FOTP_CheckReadStatus(EFC_RegStruct *efc)
{
    /* Check for errors in the flash reading process. */
    if (efc->INT_RAW_STATUS.BIT.int_raw_err_illegal ||
        efc->INT_RAW_STATUS.BIT.int_raw_err_ecc_corr ||
        efc->INT_RAW_STATUS.BIT.int_raw_err_ecc_chk) {
        efc->INT_CLEAR.BIT.int_clr_err_ecc_corr = BASE_CFG_SET;
        efc->INT_CLEAR.BIT.int_clr_err_illegal = BASE_CFG_SET;
        efc->INT_CLEAR.BIT.int_clr_err_ecc_chk = BASE_CFG_SET;
        efc->MAGIC_LOCK = FLASH_KEY_REGISTER_LOCK_VALUE;
        return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Read Four words of FOTP.
  * @param type  FOTP Range Type
  * @param index FOTP register index
  * @param buf   Buffer of read data
  * @retval BASE_STATUS_ERROR fail.
  * @retval BASE_STATUS_OK success.
  */
unsigned int FOTP_InfoGet(FOTP_InfoRngType type, unsigned int index, FOTP_CommonData *buf)
{
    EFC_RegStruct *p = EFC;
    unsigned int addr;

    if (buf == NULL) {
        return BASE_STATUS_ERROR;
    }
    
    if ((type >= FOTP_INFO_MAXTYPE) || (index > FOTP_INFO_REG_MAX_ID)) {
        return BASE_STATUS_ERROR;
    }

    /* If there is a read command, return */
    if (p->EFLASH_CMD.BIT.cmd_start) {
        return BASE_STATUS_ERROR;
    }

    p->MAGIC_LOCK = FLASH_KEY_REGISTER_UNLOCK_VALUE;

    /* Configure the read command parameters and start the read command */
    addr = (type == FOTP_INFO_RNG0) ? FOTP_INFO_RNG0_BASEADDR : FOTP_INFO_RNG1_BASEADDR;
    addr += index * REG_WORDS_NUM;
    p->EFLASH_ADDR.BIT.cmd_addr = addr >> 2; /* Right shift 2 bit change to word */
    p->EFLASH_CMD.BIT.cmd_code = FLASH_OPERATION_READ;
    p->EFLASH_CMD.BIT.cmd_read_size = FLASH_READ_128BIT;
    p->EFLASH_CMD.BIT.cmd_start = BASE_CFG_SET;

    while (p->EFLASH_CMD.BIT.cmd_start) {
        ;
    }
    while (p->EFLASH_CMD.BIT.exec_state) {
        ;
    }
    /* read error, clear interrupt and return */
    if (FOTP_CheckReadStatus(p) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    /* Read data from FIFO to buffer */
    for (unsigned int i = 0; i < sizeof(buf->data) / sizeof(buf->data[0]); ++i) {
        buf->data[i] = p->FLASH_RDATA;
    }
    p->INT_CLEAR.BIT.int_clr_finish = BASE_CFG_SET;
    p->MAGIC_LOCK = FLASH_KEY_REGISTER_LOCK_VALUE;
    return BASE_STATUS_OK;
}