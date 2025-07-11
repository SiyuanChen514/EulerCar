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
  * @file    flash_ip.h
  * @author  MCU Driver Team
  * @brief   FLASH module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the FLASH.
  *          + Register definition structure
  *          + Basic parameter configuration macro
  */

/* Define to prevent recursive inclusion ----------------------------------------*/
#ifndef McuMagicTag_FLASH_IP_H
#define McuMagicTag_FLASH_IP_H

/* Includes ---------------------------------------------------------------------*/
#include "baseinc.h"

/* Macro definitions -----------------------------------------------------------*/
#ifdef FLASH_PARAM_CHECK
#define FLASH_ASSERT_PARAM  BASE_FUNC_ASSERT_PARAM
#define FLASH_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define FLASH_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define FLASH_ASSERT_PARAM(para)  ((void)0U)
#define FLASH_PARAM_CHECK_NO_RET(para) ((void)0U)
#define FLASH_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif

/**
  * @addtogroup FLASH
  * @{
  */

/**
  * @defgroup FLASH_IP FLASH_IP
  * @brief FLASH_IP: flash_v0
  * @{
  */

#define FLASH_BASE                        0x0U            /* Flash PE operation base address. */
#define FLASH_READ_BASE                   0x3000000U      /* Base address for the flash read operation. */
#define FLASH_ONE_PAGE_SIZE               0x2000U         /* Size of a page,unit: bytes. 8K. */
#define FLASH_MAX_SIZE                    0x28000U        /* Flash space size 160k bytes. */

#define FLASH_KEY_REGISTER_UNLOCK_VALUE   0xFEDCBA98
#define FLASH_KEY_REGISTER_LOCK_VALUE     0x0

#define FLASH_MAX_PGM_WORD_SIZE           0x80
#define FLASH_MIN_PGM_BYTES_SIZE          0x10
#define FLASH_MIN_PGM_WORDS_SIZE          4
#define FLASH_ONE_WORD_BYTES_SIZE         4
#define FLASH_MAX_PAGE_NUM                20

#define FLASH_INFORMATUON_CAPACITY_POS    16
#define FLASH_INFORMATUON_CAPACITY_MASK   (0xFFFF << FLASH_INFORMATUON_CAPACITY_POS)

#define FLASH_PGM_WBUF_CNT_POS            8
#define FLASH_PGM_WBUF_CNT_MASK           (0xFF << FLASH_PGM_WBUF_CNT_POS)

#define FLASH_MAX_CMD_READ_SIZE           0x3
#define FLASH_MAX_CMD_PROGRAM_SIZE        0x20

#define FLASH_SRAM_START_ADDRESS        0x04000000
#define FLASH_SRAM_END_ADDRESS          0x04003FFF
#define FLASH_MAIN_RNG_START_ADDRESS    0x03000000
#define FLASH_MAIN_RNG_END_ADDRESS      0x03027FFF

/**
  * @defgroup FLASH_Param_Def FLASH Parameters Definition
  * @brief Definition of FLASH configuration parameters.
  * @{
  */
/* Typedef definitions --------------------------------------------------------*/
/**
 * @brief PE Operation Mode Enumeration Definition.
 */
typedef enum {
    FLASH_PE_OP_BLOCK = 0x00000000U,
    FLASH_PE_OP_IT    = 0x00000001U
} FLASH_PE_OpMode;

/**
 * @brief Erase operation type enumeration definition.
 */
typedef enum {
    FLASH_ERASE_MODE_PAGE = 0x00000004U,
    FLASH_ERASE_MODE_CHIP = 0x00000006U
} FLASH_EraseMode;

/**
  * @brief Flash page address enumeration.
  */
typedef enum {
    FLASH_PAGE_0 = FLASH_BASE,
    FLASH_PAGE_1 = FLASH_BASE + FLASH_ONE_PAGE_SIZE,
    FLASH_PAGE_2 = FLASH_BASE + 2 * FLASH_ONE_PAGE_SIZE,
    FLASH_PAGE_3 = FLASH_BASE + 3 * FLASH_ONE_PAGE_SIZE,
    FLASH_PAGE_4 = FLASH_BASE + 4 * FLASH_ONE_PAGE_SIZE,
    FLASH_PAGE_5 = FLASH_BASE + 5 * FLASH_ONE_PAGE_SIZE,
    FLASH_PAGE_6 = FLASH_BASE + 6 * FLASH_ONE_PAGE_SIZE,
    FLASH_PAGE_7 = FLASH_BASE + 7 * FLASH_ONE_PAGE_SIZE,
    FLASH_PAGE_8 = FLASH_BASE + 8 * FLASH_ONE_PAGE_SIZE,
    FLASH_PAGE_9 = FLASH_BASE + 9 * FLASH_ONE_PAGE_SIZE,
    FLASH_PAGE_10 = FLASH_BASE + 10 * FLASH_ONE_PAGE_SIZE,
    FLASH_PAGE_11 = FLASH_BASE + 11 * FLASH_ONE_PAGE_SIZE,
    FLASH_PAGE_12 = FLASH_BASE + 12 * FLASH_ONE_PAGE_SIZE,
    FLASH_PAGE_13 = FLASH_BASE + 13 * FLASH_ONE_PAGE_SIZE,
    FLASH_PAGE_14 = FLASH_BASE + 14 * FLASH_ONE_PAGE_SIZE,
    FLASH_PAGE_15 = FLASH_BASE + 15 * FLASH_ONE_PAGE_SIZE,
    FLASH_PAGE_16 = FLASH_BASE + 16 * FLASH_ONE_PAGE_SIZE,
    FLASH_PAGE_17 = FLASH_BASE + 17 * FLASH_ONE_PAGE_SIZE,
    FLASH_PAGE_18 = FLASH_BASE + 18 * FLASH_ONE_PAGE_SIZE,
    FLASH_PAGE_19 = FLASH_BASE + 19 * FLASH_ONE_PAGE_SIZE,
    FLASH_PAGE_MAX = FLASH_PAGE_19
} FLASH_SectorAddr;

/**
  * @brief Flash operation word enumeration definition.
  */
typedef enum {
    FLASH_OPERATION_READ         = 0x00000001U,
    FLASH_OPERATION_PROGRAM      = 0x00000002U,
    FLASH_OPERATION_ERASE        = 0x00000004U,
    FLASH_OPERATION_MASS_ERASE   = 0x00000006U
} FLASH_OperationType;

/**
  * @brief Flash operation status enumeration definition.
  */
typedef enum {
    FLASH_EXECUTION_STATUS_IDLE    = 0x00000000U,
    FLASH_EXECUTION_STATUS_RUNNING = 0x00000001U,
    FLASH_EXECUTION_STATUS_FINISH  = 0x00000002U
} FLASH_ExecutionStatusType;

/**
  * @brief Flash operation cmd code enumeration definition.
  */
typedef enum {
    FLASH_CMD_READ             = 0x00000001U,
    FLASH_CMD_MAIN_RGN_PROGEAM = 0x00000002U,
    FLASH_CMD_INFO_RGN_PROGEAM = 0x00000003U,
    FLASH_CMD_MAIN_RGN_ERASE   = 0x00000004U,
    FLASH_CMD_INFO_RGN_ERASE   = 0x00000005U,
    FLASH_CMD_MASS_ERASE       = 0x00000006U
} FLASH_CmdCodeType;

/**
  * @brief Callback Triggering Event Enumeration Definition
  */
typedef enum {
    FLASH_WRITE_EVENT_SUCCESS,
    FLASH_WRITE_EVENT_DONE,
    FLASH_WRITE_EVENT_FAIL,
    FLASH_ERASE_EVENT_SUCCESS,
    FLASH_ERASE_EVENT_DONE,
    FLASH_ERASE_EVENT_FAIL,
} FLASH_CallBackEvent;

/**
  * @brief FLASH extend handle, configuring some special parameters.
  */
typedef struct {
} FLASH_ExtendHandle;

/**
  * @brief User-defined callback function.
  */
typedef struct {
    /** Event callback function of the flash module */
    void (*FlashCallBack)(void *handle, FLASH_CallBackEvent event, unsigned int opAddr);
} FLASH_UserCallBcak;
/**
  * @}
  */

/**
  * @defgroup FLASH_Reg_Def FLASH Register Definition
  * @brief register mapping structure
  * @{
  */

/**
  * @brief EFLASH command registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cmd_start       : 1;  /**< Write 0:no effect, 1:start cmd operation;
                                                Read 0:cmd operation is complete, 1:cmd operation isn't complete. */
        unsigned int reserved0       : 5;
        unsigned int exec_state      : 2;  /**< Read 0: no operation or operation completed,
                                                     1: an operation is being performed,
                                                     2: the operation is complete. */
        unsigned int cmd_code        : 3;  /**< Values represent 1: read,
                                                2: main_rgn0/main_rgn1 Program,
                                                3: info_rgn0/info_rgn1 Program,
                                                4: main_rgn0/main_rgn1 Erase,
                                                5: info_rgn0/info_rgn1 Erase,
                                                6: mass erase. */
        unsigned int reserved1       : 9;
        unsigned int cmd_pgm_size    : 6;  /**< Program Size, unit:word(32bits). 0x1:4, 0x2:8,..., 0x1F:124, 0x20:128,
                                                other values are invalid. */
        unsigned int reserved3       : 2;
        unsigned int cmd_read_size   : 2;  /**< Read Size, unit:word(32bits). 0x0:1, 0x1:4, 0x2:8, 0x3:12. */
        unsigned int reserved4       : 2;
    } BIT;
} volatile EFLASH_CMD_REG;

/**
  * @brief EFLASH address registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0        : 2;
        unsigned int cmd_addr         : 22; /**< Program, erase, or read start address register. Unit:byte(8bits).
                                                 start address of Main_rgn0/main_rgn1: 0x00_0000,
                                                 start address of info_rgn0/info_rgn1: 0x80_0000,
                                                 note: the lower 2 bits cannot be written. */
        unsigned int reserved1        : 8;
    } BIT;
} volatile EFLASH_ADDR_REG;

/**
  * @brief Command configuration registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0        : 1;
        unsigned int int_mode         : 1;   /**< Command operation mode 0:blocking mode, 1:interrupt mode. */
        unsigned int reserved1        : 30;
    } BIT;
} volatile CMD_CFG_COMMON_REG;

/**
  * @brief The raw interrupt status registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0                : 1;
        unsigned int reserved1                : 1;
        unsigned int reserved2                : 1;
        unsigned int reserved3                : 1;
        unsigned int int_raw_finish           : 1;  /**< Operation completion status,
                                                         0:no operation performed or operation completed,
                                                         1: the operation completed. */
        unsigned int reserved4                : 11;
        unsigned int int_raw_err_illegal      : 1;  /**< Invalid cmd operation errors, 0:no errors,
                                                         1: cmd operation error. */
        unsigned int int_raw_err_smwr         : 1;  /**< PGM/ERASE error, 0:pass, 1:failure. */
        unsigned int int_raw_err_ahb          : 1;  /**< AHB request error, 0:no errors, 1:AHB read address request
                                                         exceeds the range of Main_rgn0/main_rgn1 or
                                                         AHB write request occurs. */
        unsigned int int_raw_err_ecc_corr     : 1;  /**< Main_rgn0/rgn1 Read Data ECC Correction Error, 0:no errors,
                                                         1:Uncorrectable ECC error occurred. */
        unsigned int int_raw_err_ecc_chk      : 1;  /**< Main_rgn0/main_rgn1 read data ECC error, 0:no errors,
                                                         1:an ECC check error occurred. */
        unsigned int reserved5                : 11;
    } BIT;
} volatile INT_RAW_STATUS_REG;

/**
  * @brief The interrupt enable configuration registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0                : 1;
        unsigned int reserved1                : 1;
        unsigned int reserved2                : 1;
        unsigned int reserved3                : 1;
        unsigned int int_en_finish            : 1;  /**< Operation completion interrupt enable, 0:disable, 1:enable. */
        unsigned int reserved4                : 11;
        unsigned int int_en_err_illegal       : 1;  /**< Invalid Cmd operation error interrupt enable,
                                                         0:disable, 1:enable. */
        unsigned int int_en_err_smwr          : 1;  /**< PGM/ERASE error interrupt enable, 0:disable, 1:enable. */
        unsigned int int_en_err_ahb           : 1;  /**< AHB request error interrupt enable, 0:disable, 1:enable. */
        unsigned int int_en_err_ecc_corr      : 1;  /**< Main_rgn0/rgn1 read data ECC correction error interrupt,
                                                         0:disable, 1:enable. */
        unsigned int int_en_err_ecc_chk       : 1;  /**< Main_rgn0/rgn1 read data ECC check error interrupt enable,
                                                         0:disable, 1:enable. */
        unsigned int reserved5                : 11;
    } BIT;
} volatile INT_ENABLE_REG;

/**
  * @brief Interrupt clear registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0                : 1;
        unsigned int reserved1                : 1;
        unsigned int reserved2                : 1;
        unsigned int reserved3                : 1;
        unsigned int int_clr_finish           : 1;  /**< Operation completion interrupt clear, 0:not clear,
                                                         1:clear raw interrupts and interrupt status. */
        unsigned int reserved4                : 11;
        unsigned int int_clr_err_illegal      : 1;  /**< Invalid CMD operation error interrupt clear, 0:not clear,
                                                         1:clear raw interrupts and interrupt status. */
        unsigned int int_clr_err_smwr         : 1;  /**< Pgm/erase error interrupt clear, 0:not clear,
                                                         1:clear raw interrupts and interrupt status. */
        unsigned int int_clr_err_ahb          : 1;  /**< AHB request error interrupt clear, 0:not clear,
                                                         1:clear raw interrupts and interrupt status. */
        unsigned int int_clr_err_ecc_corr     : 1;  /**< Main_rgn0/rgn1 read data ECC correction error interrupt clear,
                                                         0:not clear, 1:clear raw interrupts and interrupt status. */
        unsigned int int_clr_err_ecc_chk      : 1;  /**< Main_rgn0/main_rgn1 read data ECC error interrupt clear,
                                                         0:not clear, 1:clear raw interrupts and interrupt status. */
        unsigned int reserved5                : 11;
    } BIT;
} volatile INT_CLEAR_REG;

/**
  * @brief Prefetch control registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int prefetch_enable        : 1;  /**< Prefetch control enable, 0:disabled, 1:enable. */
        unsigned int reserved0              : 31;
    } BIT;
} volatile PREFETCH_CTRL_REG;

/**
  * @brief Cache control registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cache_enable           : 1;  /**< Prefetch control enable, 0:disabled, 1:enable. */
        unsigned int reserved0              : 3;
        unsigned int cache_replacement_sel  : 1;  /**< Cache replacement policy selection, 0:PLRU policy,
                                                       1:round robin policy. */
        unsigned int reserved1              : 3;
        unsigned int cache_invalid_req      : 1;  /**< Cache data invalid request, 0:invalidation,
                                                       1:request cache invalid. */
        unsigned int reserved2              : 23;
    } BIT;
} volatile CACHE_CTRL_REG;

/**
  * @brief Flash ECC error detection and correction enable control registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int flash_main_ecc_check_enable    : 1;  /**< Main_rgn0/main_rgn1 error detection enable,
                                                               0:no ECC check, 1:ECC check. */
        unsigned int flash_main_ecc_correct_enable  : 1;  /**< Main_rgn0/main_rgn1 error detection enable,
                                                               0:no ECC check, 1:ECC check. */
        unsigned int flash_info_ecc_check_enable    : 1;  /**< Main_rgn0/main_rgn1 ECC error detection enable,
                                                               0:no ECC check, 1:ECC check. */
        unsigned int flash_info_ecc_correct_enable  : 1;  /**< Main_rgn0/main_rgn1 ECC error correction function,
                                                               0:no ECC check, 1:ECC check. */
        unsigned int flash_ecc_blank_filter_enable  : 1;  /**< Flash unprogrammed area ECC mask and filter enable,
                                                               0:disable, 1:enable. */
        unsigned int reserved0                      : 27;
    } BIT;
} volatile FLASH_ECC_CTRL_REG;

/**
  * @brief Flash status registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int opcode_illegal       : 3;  /**< Invalid opcode value. */
        unsigned int reserved0            : 1;
        unsigned int mid_illegal          : 3;  /**< Invalid mid value. */
        unsigned int reserved1            : 1;
        unsigned int info_rgn0_illegal    : 1;  /**< Illegally operation info_rgn0, 0:no error,
                                                     1:illegally access occurs. */
        unsigned int info_rgn1_illegal    : 1;  /**< Illegally operation info_rgn1, 0:no error,
                                                     1:illegally access occurs. */
        unsigned int reserved2            : 2;
        unsigned int main_rgn0_illegal    : 1;  /**< Illegally operation main_rgn0, 0:no error,
                                                     1:illegally access occurs. */
        unsigned int main_rgn1_illegal    : 1;  /**< Illegally operation main_rgn1, 0:no error,
                                                     1:illegally access occurs. */
        unsigned int reserved3            : 2;
        unsigned int parameter_illegal    : 1;  /**< Operation parameter is valid, 0:no error,
                                                     1:Operation parameter error. */
        unsigned int address_unmap        : 1;  /**< Operation address out-of-bounds, 0:no error,
                                                     1:address out-of-bounds error. */
        unsigned int reserved4            : 2;
        unsigned int reserved5            : 12;
    } BIT;
} volatile FLASH_STATUS_REG;

/**
  * @brief Flash read data validity indicator registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int flash_rdata_vld       : 1;  /**< Read data FIFO data validity indicator, 0:valid, 1:invalid. */
        unsigned int reserved0             : 31;
    } BIT;
} volatile FLASH_RDATA_VLD_REG;

/**
  * @brief Flash Module information 1 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int page_size               : 16; /**< Info_rgn0/info_rgn1 capacity, unit:byte. */
        unsigned int information_capacity    : 16; /**< Eflash page capacity, unit:byte. */
    } BIT;
} volatile EFLASH_CAPACITY_1_REG;

/**
  * @brief Flash Module information 2 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int io_read_length                  : 4; /**< Read I/O size. */
        unsigned int io_write_length_information     : 4; /**< Write information I/O size. */
        unsigned int io_write_length_main            : 4; /**< Write main I/O size. */
        unsigned int min_pgm_size_information        : 4; /**< Minimal programming size of information I/O size. */
        unsigned int min_pgm_size_main               : 4; /**< Minimal programming size of main I/O size. */
        unsigned int max_pgm_size                    : 4; /**< Max programming size of I/O size. */
        unsigned int min_erase_size                  : 4; /**< Minimal erase size of I/O size. */
        unsigned int reserved0                       : 4;
    } BIT;
} volatile EFLASH_CAPACITY_2_REG;

/**
  * @brief Flash clears the programming data buffer registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pgm_wdata_clr        : 1;   /**< Clear Control, 0:no effect, 1:clear current buffer. */
        unsigned int reserved0            : 7;
        unsigned int pgm_wbuf_cnt         : 8;   /**< Obtains the size of the data in the buffer, unit:word. */
        unsigned int reserved1            : 16;
    } BIT;
} volatile BUF_CLEAR_REG;

/**
  * @brief Flash clock divider registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    m20ns_div                    : 4;  /**< Clock divider, valid valuebits:0x0~0x0F. */
        unsigned int    nread_div                    : 4;  /**< Eflash normal read, valid valuebits:0x0~0x0F. */
        unsigned int    sread_div                    : 3;  /**< Eflash special read. */
        unsigned int    reserved_0                   : 1;
        unsigned int    ef_timer_option_unit         : 8;  /**< Used to setup timer option for eflash program/erase. */
        unsigned int    busclk_sw_req                : 1;  /**< Check whether the handover is complete. */
        unsigned int    busclk_switch_protect_enable : 1;  /**< Frequency switching process protection,
                                                                0:disable, 1:enable. */
        unsigned int    reserved_1                   : 2;
        unsigned int    data_vld_sel                 : 2;  /**< Data vld sel. */
        unsigned int    reserved_2                   : 6;
    } BIT;
} volatile EFLASH_CLK_CFG_REG;

/**
  * @brief Flash smart write timer control registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    reserved_0             : 8;
        unsigned int    smw_timer_option_value : 5;   /**< Define of state required for PV/EV read cycle. */
        unsigned int    reserved_1             : 19;
    } BIT;
} volatile SMW_TIMER_OPTION_REG;

/**
 * @brief FLASH Register definition structure
 */
typedef struct {
    EFLASH_CMD_REG                  EFLASH_CMD;        /**< Command register. Offset Address: 0x0000. */
    EFLASH_ADDR_REG                 EFLASH_ADDR;       /**< Address register. Offset Address: 0x0004. */
    unsigned char                   space0[120];       /**< 0x8~0x7c */
    CMD_CFG_COMMON_REG              CMD_CFG_COMMON;    /**< CMD configuration register. Offset Address: 0x0080. */
    unsigned char                   space1[124];       /**< 0x84~0xfc */
    INT_RAW_STATUS_REG              INT_RAW_STATUS;    /**< Raw interrupt status register. Offset Address: 0x0100. */
    unsigned char                   space2[4];         /**< 0x104 */
    INT_ENABLE_REG                  INT_ENABLE;        /**< Interrupt enable configuration register.
                                                            Offset Address: 0x0108. */
    INT_CLEAR_REG                   INT_CLEAR;         /**< Interrupt clear register. Offset Address: 0x010C. */
    unsigned char                   space3[16];        /**< 0x110~0x11c */
    PREFETCH_CTRL_REG               PREFETCH_CTRL;     /**< Prefetch control register. Offset Address: 0x0120. */
    CACHE_CTRL_REG                  CACHE_CTRL;        /**< Cache control register. Offset Address: 0x0124. */
    unsigned char                   space4[4];         /**< 0x128~0x12C */
    FLASH_ECC_CTRL_REG              FLASH_ECC_CTRL;    /**< ECC error detection and correction enable control register.
                                                            Offset Address: 0x012C. */
    FLASH_STATUS_REG                FLASH_STATUS;      /**< CMD operation flash status register.
                                                            Offset Address: 0x0130. */
    FLASH_RDATA_VLD_REG             FLASH_RDATA_VLD;   /**< Flash read data validity indicator register.
                                                            Offset Address: 0x0134. */
    unsigned int                    AHB_ERR_ADDR;      /**< AHB error request address record register.
                                                            Offset Address: 0x0138. */
    unsigned char                   space5[196];       /**< 0x13c~0x1fc */
    unsigned int                    MAGIC_LOCK;        /**< CMD magic word protection register.
                                                            Offset Address: 0x0200. */
    unsigned char                   space6[492];       /**< 0x204~0x3ec */
    unsigned int                    EFLASH_CAPACITY_0; /**< Module information register 0. Offset Address: 0x03F0. */
    EFLASH_CAPACITY_1_REG           EFLASH_CAPACITY_1; /**< Module information register 1. Offset Address: 0x03F4. */
    EFLASH_CAPACITY_2_REG           EFLASH_CAPACITY_2; /**< Module information register 2. Offset Address: 0x03F8. */
    unsigned char                   space7[4];         /**< 0x3fc */
    unsigned int                    PGM_WDATA;         /**< Program Data Register. Offset Address: 0x0400. */
    unsigned char                   space8[508];       /**< 0x404~0x5fc */
    unsigned int                    FLASH_RDATA;       /**< Read data register. Offset Address: 0x0600. */
    BUF_CLEAR_REG                   BUF_CLEAR;         /**< Programming data buffer cleanup register.
                                                            Offset Address: 0x0604. */
    unsigned int                    space9[206];       /**< 0x608~0x93c. */
    EFLASH_CLK_CFG_REG              EFLASH_CLK_CFG;    /**< Clock divider register. Offset Address: 0x940. */
    unsigned int                    space10[307];      /**< 0x944~0xe0c. */
    SMW_TIMER_OPTION_REG            SMW_TIMER_OPTION;  /**< Smart write timer control register.
                                                            Offset Address: 0xe10. */
} volatile EFC_RegStruct;
/**
  * @}
  */

/* Parameter check definition-------------------------------------------*/
/**
  * @brief Check flash cmd code.
  * @param cmdCode Flash cmd code.
  * @retval true
  * @retval false
  */
static inline bool IsFlashCmdCode(FLASH_CmdCodeType cmdCode)
{
    return (cmdCode == FLASH_CMD_READ || cmdCode == FLASH_CMD_MAIN_RGN_PROGEAM || \
            cmdCode == FLASH_CMD_INFO_RGN_PROGEAM || cmdCode == FLASH_CMD_MAIN_RGN_ERASE || \
            cmdCode == FLASH_CMD_INFO_RGN_ERASE || cmdCode == FLASH_CMD_MASS_ERASE);
}

/**
  * @brief Check flash cmd program size.
  * @param size cmd program size, unit:Word(32bit).
  * @retval true
  * @retval false
  */
static inline bool IsFlashCmdProgramSize(unsigned int size)
{
    return ((size > 0x00) && (size <= FLASH_MAX_CMD_PROGRAM_SIZE)); /* The max value of cmd program size is 0x20. */
}

/**
  * @brief Check flash cmd read size.
  * @param size cmd read size, unit:Word(32bit).
  * @retval true
  * @retval false
  */
static inline bool IsFlashCmdReadSize(unsigned int size)
{
    return (size <= FLASH_MAX_CMD_READ_SIZE); /* The max value of cmd read size is 0x03. */
}

/**
  * @brief Check flash program address.
  * @param addr program address, unit:Byte(8bit).
  * @retval true
  * @retval false
  */
static inline bool IsFlashProgramAddress(unsigned int addr)
{
    return (((addr % FLASH_MIN_PGM_BYTES_SIZE) == 0) && (addr < FLASH_MAX_SIZE));
}

/**
  * @brief Check flash erase address.
  * @param addr erase address, unit:Byte(8bit).
  * @retval true
  * @retval false
  */
static inline bool IsFlashEraseAddress(unsigned int addr)
{
    return ((addr % FLASH_ONE_PAGE_SIZE) == 0) && (addr <= FLASH_PAGE_MAX);
}

/**
  * @brief Check flash read address.
  * @param addr cmd read size, unit:Byte(8bit).
  * @retval true
  * @retval false
  */
static inline bool IsFlashReadAddress(unsigned int addr)
{
    return addr < FLASH_MAX_SIZE;
}

/**
  * @brief Check Operation mode selection.
  * @param opMode Flash Operation mode.
  * @retval true
  * @retval false
  */
static inline bool IsFlashOperationMode(FLASH_PE_OpMode opMode)
{
    return (opMode == FLASH_PE_OP_BLOCK ||
            opMode == FLASH_PE_OP_IT);
}

/**
  * @brief Check flash write source addresss.
  * @param addr write source addresss.
  * @retval true
  * @retval false
  */
static inline bool IsFlashWriteSrcAddress(unsigned int addr)
{
    return ((addr >= FLASH_SRAM_START_ADDRESS && addr <= FLASH_SRAM_END_ADDRESS) ||
            (addr >= FLASH_MAIN_RNG_START_ADDRESS && addr <= FLASH_MAIN_RNG_END_ADDRESS));
}

/**
  * @brief Enable flash command start.
  * @param efc FLASH register base address.
  * @retval None.
  */
static inline void DCL_FLASH_CmdStartEnable(EFC_RegStruct *efcx)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    efcx->EFLASH_CMD.BIT.cmd_start = BASE_CFG_ENABLE;
}

/**
  * @brief Disable flash command start.
  * @param efcx FLASH register base address.
  * @retval None.
  */
static inline void DCL_FLASH_CmdStartDisable(EFC_RegStruct *efcx)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    efcx->EFLASH_CMD.BIT.cmd_start = BASE_CFG_DISABLE;
}

/**
  * @brief Getting flash command start State.
  * @param efcx FLASH register base address.
  * @retval command start value, 1: Operation complete or no operation, 0: Operation is not complete.
  */
static inline unsigned int DCL_FLASH_GetCmdStartState(EFC_RegStruct *efcx)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    return efcx->EFLASH_CMD.BIT.cmd_start;
}

/**
  * @brief Setting FLASH cmd code.
  * @param efcx FLASH register base address.
  * @param cmdCode flash cmd code.
  * @retval None.
  */
static inline void DCL_FLASH_SetCmdCode(EFC_RegStruct *efcx, FLASH_CmdCodeType cmdCode)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    FLASH_PARAM_CHECK_NO_RET(IsFlashCmdCode(cmdCode));
    efcx->EFLASH_CMD.BIT.cmd_code = cmdCode;
}

/**
  * @brief Getting FLASH cmd code.
  * @param efcx FLASH register base address.
  * @retval cmd code, 1:READ, 2:FLASH_CMD_MAIN_RGN_PROGEAM, 3:FLASH_CMD_INFO_RGN_PROGEAM, 4:FLASH_CMD_MAIN_RGN_ERASE,
                      5:FLASH_CMD_INFO_RGN_ERASE, 6:FLASH_CMD_MASS_ERASE.
  */
static inline unsigned int DCL_FLASH_GetCmdCode(EFC_RegStruct *efcx)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    return efcx->EFLASH_CMD.BIT.cmd_code;
}

/**
  * @brief Setting FLASH cmd program size.
  * @param efcx FLASH register base address.
  * @param size flash cmd program size, unit:Word(32bit).
  * @retval None.
  */
static inline void DCL_FLASH_SetCmdProgramSize(EFC_RegStruct *efcx, unsigned int size)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    FLASH_PARAM_CHECK_NO_RET(IsFlashCmdProgramSize(size));
    efcx->EFLASH_CMD.BIT.cmd_pgm_size = size;
}

/**
  * @brief Getting FLASH cmd program size.
  * @param efcx FLASH register base address.
  * @retval cmd program size, unit:Word(32bit).
  */
static inline unsigned int DCL_FLASH_GetCmdProgramSize(EFC_RegStruct *efcx)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    return efcx->EFLASH_CMD.BIT.cmd_pgm_size;
}

/**
  * @brief Setting FLASH program start address.
  * @param efcx FLASH register base address.
  * @param addr flash cmd program start address, unit:Byte(8bit).
  * @retval None.
  */
static inline void DCL_FLASH_SetProgramAddress(EFC_RegStruct *efcx, unsigned int addr)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    FLASH_PARAM_CHECK_NO_RET(IsFlashProgramAddress(addr));
    efcx->EFLASH_ADDR.BIT.cmd_addr = addr;
}

/**
  * @brief Setting FLASH erase start address.
  * @param efcx FLASH register base address.
  * @param addr flash cmd erase start address, unit:Byte(8bit).
  * @retval None.
  */
static inline void DCL_FLASH_SetEraseAddress(EFC_RegStruct *efcx, unsigned int addr)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    FLASH_PARAM_CHECK_NO_RET(IsFlashEraseAddress(addr));
    efcx->EFLASH_ADDR.BIT.cmd_addr = addr;
}

/**
  * @brief Getting FLASH cmd program, erase, read start address.
  * @param efcx FLASH register base address.
  * @retval cmd program, erase, read start address, unit:Byte(8bit).
  */
static inline unsigned int DCL_FLASH_GetCmdStartAddress(EFC_RegStruct *efcx)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    return efcx->EFLASH_ADDR.BIT.cmd_addr;
}

/**
  * @brief Setting FLASH operation mode.
  * @param efcx FLASH register base address.
  * @param mode flash operation mode.
  * @retval None.
  */
static inline void DCL_FLASH_SetOptMode(EFC_RegStruct *efcx, FLASH_PE_OpMode mode)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    FLASH_PARAM_CHECK_NO_RET(IsFlashOperationMode(mode));
    efcx->CMD_CFG_COMMON.BIT.int_mode = mode;
}

/**
  * @brief Getting FLASH operation mode.
  * @param efcx FLASH register base address.
  * @retval operation mode, 0:FLASH_PE_OP_BLOCK, 1:FLASH_PE_OP_IT.
  */
static inline unsigned int DCL_FLASH_GetOptMode(EFC_RegStruct *efcx)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    return efcx->CMD_CFG_COMMON.BIT.int_mode;
}

/**
  * @brief Obtains the interrupt status.
  * @param efcx FLASH register base address.
  * @retval Interrupt Status.
  */
static inline unsigned int DCL_FLASH_GetInterrupRawtStatus(EFC_RegStruct *efcx)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    return efcx->INT_RAW_STATUS.reg;
}

/**
  * @brief Configuring Interrupt Enable.
  * @param efcx FLASH register base address.
  * @param intrEn Corresponding interrupt enable bit, for example, 110011.
  * @retval None.
  */
static inline void DCL_FLASH_SetInterruptEn(EFC_RegStruct *efcx, unsigned int intrEn)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    efcx->INT_ENABLE.reg = intrEn;
}

/**
  * @brief Obtaining the Interrupt Enable Configuration.
  * @param efcx FLASH register base address.
  * @retval Interrupt enable value.
  */
static inline unsigned int DCL_FLASH_GetInterruptEnState(EFC_RegStruct *efcx)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    return efcx->INT_ENABLE.reg;
}

/**
  * @brief Clear Interrupt.
  * @param efcx FLASH register base address.
  * @param intrRaw Corresponding interrupt bit, for example, 110011.
  * @retval None.
  */
static inline void DCL_FLASH_ClearIrq(EFC_RegStruct *efcx, unsigned int intrRaw)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    efcx->INT_CLEAR.reg = intrRaw;
}

/**
  * @brief FLASH cache invalid request enable.
  * @param efcx FLASH register base address.
  * @retval None.
  */
static inline void DCL_FLASH_CacheInvalidRequestEnable(EFC_RegStruct *efcx)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    efcx->CACHE_CTRL.BIT.cache_invalid_req = BASE_CFG_ENABLE;
}

/**
  * @brief FLASH cache invalid request disable.
  * @param efcx FLASH register base address.
  * @retval None.
  */
static inline void DCL_FLASH_CacheInvalidRequestDisable(EFC_RegStruct *efcx)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    efcx->CACHE_CTRL.BIT.cache_invalid_req = BASE_CFG_DISABLE;
}

/**
  * @brief Getting FLASH cache invalid request state.
  * @param efcx FLASH register base address.
  * @retval state 0:The latest invalid request has been completed,
                  1:The latest invalid request is not completed.
  */
static inline unsigned int DCL_FLASH_GetCacheInvalidRequestState(EFC_RegStruct *efcx)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    return efcx->CACHE_CTRL.BIT.cache_invalid_req;
}

/**
  * @brief Getting FLASH command operation status.
  * @param efcx FLASH register base address.
  * @retval command operation status.
  */
static inline unsigned int DCL_FLASH_GetCommandOptStatus(EFC_RegStruct *efcx)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    return efcx->FLASH_STATUS.reg;
}

/**
  * @brief Setting FLASH magic lock.
  * @param efcx FLASH register base address.
  * @retval None.
  */
static inline void DCL_FLASH_MagicLock(EFC_RegStruct *efcx)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    efcx->MAGIC_LOCK = FLASH_KEY_REGISTER_LOCK_VALUE;
}

/**
  * @brief Setting FLASH magic unlock.
  * @param efcx FLASH register base address.
  * @retval None.
  */
static inline void DCL_FLASH_MagicUnlock(EFC_RegStruct *efcx)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    efcx->MAGIC_LOCK = FLASH_KEY_REGISTER_UNLOCK_VALUE;
}

/**
  * @brief Getting FLASH magic lock.
  * @param efcx FLASH register base address.
  * @retval The value of magic lock, The value 0xFEDC_BA98 indicates magic unlock, others values is magic lock.
  */
static inline unsigned int DCL_FLASH_GetMagicLock(EFC_RegStruct *efcx)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    return efcx->MAGIC_LOCK;
}

/**
  * @brief Setting FLASH program wdata value.
  * @param efcx FLASH register base address.
  * @param value The value of program wdata.
  * @retval None.
  */
static inline void DCL_FLASH_SetProgramWdata(EFC_RegStruct *efcx, unsigned int value)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    efcx->PGM_WDATA = value;
}

/**
  * @brief FLASH program wdata celar enable.
  * @param efcx FLASH register base address.
  * @retval None.
  */
static inline void DCL_FLASH_ProgramWdataClearEnable(EFC_RegStruct *efcx)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    efcx->BUF_CLEAR.BIT.pgm_wdata_clr = BASE_CFG_ENABLE;
}

/**
  * @brief FLASH program wdata celar disable.
  * @param efcx FLASH register base address.
  * @retval None.
  */
static inline void DCL_FLASH_ProgramWdataClearDisable(EFC_RegStruct *efcx)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    efcx->BUF_CLEAR.BIT.pgm_wdata_clr = BASE_CFG_DISABLE;
}

/**
  * @brief Getting FLASH buf clear value.
  * @param efcx FLASH register base address.
  * @retval None.
  */
static inline unsigned int DCL_FLASH_GetBufClearValue(EFC_RegStruct *efcx)
{
    FLASH_ASSERT_PARAM(efcx != NULL);
    return efcx->BUF_CLEAR.reg;
}

/**
  * @}
  */

/**
  * @}
  */
#endif /* #ifndef McuMagicTag_FLASH_IP_H */