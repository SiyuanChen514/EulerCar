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
  * @file    i2c_ip.h
  * @author  MCU Driver Team
  * @brief   I2C module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the I2C.
  *          + Register definition structure
  *          + Timing command enumeration
  *          + Direct configuration layer interface
  *          + Basic parameter configuration macro
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef McuMagicTag_I2C_IP_H
#define McuMagicTag_I2C_IP_H

/* Includes ------------------------------------------------------------------*/
#include "baseinc.h"

/* Macro definitions --------------------------------------------------------- */
#ifdef I2C_PARAM_CHECK
#define I2C_ASSERT_PARAM  BASE_FUNC_ASSERT_PARAM
#define I2C_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define I2C_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define I2C_ASSERT_PARAM(para)  ((void)0U)
#define I2C_PARAM_CHECK_NO_RET(para) ((void)0U)
#define I2C_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif

/**
  * @addtogroup I2C
  * @{
  */

/**
  * @defgroup I2C_IP I2C_IP
  * @brief I2C_IP: i2c_v0
  * @{
  */

#define I2C_IGNORE_NAK_ENABLE     BASE_CFG_ENABLE  /**< Ignore acknowledgment configuration enable. */
#define I2C_IGNORE_NAK_DISABLE    BASE_CFG_DISABLE /**< Ignore acknowledgment configuration disable. */

#define I2C_MSG_STOP_FLAG_ENABLG  BASE_CFG_ENABLE  /**< Last frame data flag enable. */
#define I2C_MSG_STOP_FLAG_DISABLE BASE_CFG_DISABLE /**< Last frame data flag disable. */

#define I2C_STANDARD_FREQ_TH      100000      /**< Standard mode,the frequency band is less than or equal to 100 kHz. */
#define I2C_SDA_HOLD_DURATION     0x0000000AU /**< Sda hold duration.The value is fixed to 0xa. */

#define I2C_INTR_RAW_ALL_ENABLE   0x00001FFFU /**< 1111111111111 */
#define I2C_INTR_RAWALL_DISABLE   0x00000000U /**< 0000000000000 */

#define I2C_INTR_EN_ALL_ENABLE    0x00001FFFU /**< 1111111111111 */
#define I2C_INTR_EN_ALL_DISABLE   0x00000000U /**< 0000000000000 */

#define I2C_MAX_CMD_OFFSET_LEN    32

#define I2C_SDA_HOLD_DURATION_POS   8
#define I2C_SDA_HOLD_DURATION_MASK (0xFFFF << I2C_SDA_HOLD_DURATION_POS)

#define I2C_DEV_ADDR_BYTE1_POS      0
#define I2C_DEV_ADDR_BYTE1_MASK    (0xFF << I2C_DEV_ADDR_BYTE1_POS)
#define I2C_DEV_ADDR_BYTE2_POS      8
#define I2C_DEV_ADDR_BYTE2_MASK    (0xFF << I2C_DEV_ADDR_BYTE2_POS)
#define I2C_DEV_ADDR_BYTE3_POS      16
#define I2C_DEV_ADDR_BYTE3_MASK    (0xFF << I2C_DEV_ADDR_BYTE3_POS)
#define I2C_DEV_ADDR_BYTE4_POS      24
#define I2C_DEV_ADDR_BYTE4_MASK    (0xFF << I2C_DEV_ADDR_BYTE4_POS)

#define I2C_DATA_BUF_BYTE1_POS      0
#define I2C_DATA_BUF_BYTE1_MASK    (0xFF << I2C_DATA_BUF_BYTE1_POS)
#define I2C_DATA_BUF_BYTE2_POS      8
#define I2C_DATA_BUF_BYTE2_MASK    (0xFF << I2C_DATA_BUF_BYTE2_POS)
#define I2C_DATA_BUF_BYTE3_POS      16
#define I2C_DATA_BUF_BYTE3_MASK    (0xFF << I2C_DATA_BUF_BYTE3_POS)
#define I2C_DATA_BUF_BYTE4_POS      24
#define I2C_DATA_BUF_BYTE4_MASK    (0xFF << I2C_DATA_BUF_BYTE4_POS)

#define I2C_PATTERN_DATA_BYTE1_POS   0
#define I2C_PATTERN_DATA_BYTE1_MASK (0xFF << I2C_PATTERN_DATA_BYTE1_POS)
#define I2C_PATTERN_DATA_BYTE2_POS   8
#define I2C_PATTERN_DATA_BYTE2_MASK (0xFF << I2C_PATTERN_DATA_BYTE2_POS)
#define I2C_PATTERN_DATA_BYTE3_POS   16
#define I2C_PATTERN_DATA_BYTE3_MASK (0xFF << I2C_PATTERN_DATA_BYTE3_POS)
#define I2C_PATTERN_DATA_BYTE4_POS   24
#define I2C_PATTERN_DATA_BYTE4_MASK (0xFF << I2C_PATTERN_DATA_BYTE4_POS)

#define I2C_PATTERN_DATA_BYTE5_POS   0
#define I2C_PATTERN_DATA_BYTE5_MASK (0xFF << I2C_PATTERN_DATA_BYTE5_POS)
#define I2C_PATTERN_DATA_BYTE6_POS   8
#define I2C_PATTERN_DATA_BYTE6_MASK (0xFF << I2C_PATTERN_DATA_BYTE6_POS)
#define I2C_PATTERN_DATA_BYTE7_POS   16
#define I2C_PATTERN_DATA_BYTE7_MASK (0xFF << I2C_PATTERN_DATA_BYTE7_POS)
#define I2C_PATTERN_DATA_BYTE8_POS   24
#define I2C_PATTERN_DATA_BYTE8_MASK (0xFF << I2C_PATTERN_DATA_BYTE8_POS)

/**
  * @defgroup I2C_Param_Def I2C Parameters Definition
  * @brief Definition of I2C configuration parameters.
  * @{
  */
/* Typedef definitions -------------------------------------------------------*/
/**
 * @brief Address Mode Selection Enumeration Definition
 */
typedef enum {
    I2C_7_BITS  = 0x00000000U,
    I2C_10_BITS = 0x00000001U
} I2C_AddressMode;

/**
 * @brief I2C DMA operation type enumeration definition
 */
typedef enum {
    I2C_DMA_OP_NONE  = 0x00000000U,
    I2C_DMA_OP_READ  = 0x00000003U,
    I2C_DMA_OP_WRITE = 0x00000002U
} I2C_DmaOperationType;

/**
 * @brief I2C operation timing enumeration definition
 */
typedef enum {
    I2C_CMD_EXIT  = 0x00000000U, /**< End command for logical exit. */
    I2C_CMD_S     = 0x00000001U, /**< Bus START command. */
    I2C_CMD_WDA4  = 0x00000002U, /**< Send the dev_addr_byte4 command. */
    I2C_CMD_WDA3  = 0x00000003U, /**< Send the dev_addr_byte3 command. */
    I2C_CMD_WDA2  = 0x00000004U, /**< Send the dev_addr_byte2 command. */
    I2C_CMD_WDA1  = 0x00000005U, /**< Send the dev_addr_byte1 command. */
    I2C_CMD_WDB4  = 0x00000006U, /**< Send the data_buf_byte4 command. */
    I2C_CMD_WDB3  = 0x00000007U, /**< Send the data_buf_byte3 command. */
    I2C_CMD_WDB2  = 0x00000008U, /**< Send the data_buf_byte2 command. */
    I2C_CMD_WDB1  = 0x00000009U, /**< Send the data_buf_byte1 command. */
    I2C_CMD_WPD8  = 0x0000000AU, /**< Send the pattern_data_byte8 command. */
    I2C_CMD_WPD7  = 0x0000000BU, /**< Send the pattern_data_byte7 command. */
    I2C_CMD_WPD6  = 0x0000000CU, /**< Send the pattern_data_byte6 command. */
    I2C_CMD_WPD5  = 0x0000000DU, /**< Send the pattern_data_byte5 command. */
    I2C_CMD_WPD4  = 0x0000000EU, /**< Send the pattern_data_byte4 command. */
    I2C_CMD_WPD3  = 0x0000000FU, /**< Send the pattern_data_byte3 command. */
    I2C_CMD_WPD2  = 0x00000010U, /**< Send the pattern_data_byte2 command. */
    I2C_CMD_WPD1  = 0x00000011U, /**< Send the pattern_data_byte1 command. */
    I2C_CMD_RD    = 0x00000012U, /**< Command for receiving 1-byte data. */
    I2C_CMD_RACK  = 0x00000013U, /**< Receive low-level acknowledgment command. */
    I2C_CMD_RNACK = 0x00000014U, /**< Receive high level no acknowledgment command.*/
    I2C_CMD_RNC   = 0x00000015U, /**< Receives the response command. The high or low level does not matter.*/
    I2C_CMD_SACK  = 0x00000016U, /**< Send low-level acknowledgment command. */
    I2C_CMD_SNACK = 0x00000017U, /**< Send high-level no-acknowledge command. */
    I2C_CMD_JMPN1 = 0x00000018U, /**< Jump command for a limited number of times.
                                      The purpose is indicated by the DST1 register.
                                      and the number of times is indicated by the LOOP1 register. */
    I2C_CMD_JMPN2 = 0x00000019U, /**< Jump command for a limited number of times.
                                      The purpose is indicated by the DST2 register.
                                      and the number of times is indicated by the LOOP2 register. */
    I2C_CMD_JMPN3 = 0x0000001AU, /**< Jump command for a limited number of times.
                                      The purpose is indicated by the DST3 register.
                                      and the number of times is indicated by the LOOP3 register. */
    I2C_CMD_UDB1  = 0x0000001DU, /**< Update data from the TX FIFO to data_buf_byte1. */
    I2C_CMD_SR    = 0x0000001EU, /**< Bus Repeated START. */
    I2C_CMD_P     = 0x0000001FU  /**< Bus STOP. */
} I2C_CmdType;

/**
  * @brief I2C mode selection enumeration definition
  */
typedef enum {
    I2C_MODE_SELECT_NONE         = 0x00000000U,
    I2C_MODE_SELECT_MASTER_ONLY  = 0x00000001U,
    I2C_MODE_SELECT_SLAVE_ONLY   = 0x00000002U,
    I2C_MODE_SELECT_MASTER_SLAVE = 0x00000003U
} I2C_ModeSelectType;

/**
  * @brief I2C extend handle, configuring some special parameters.
  */
typedef struct {
} I2C_ExtendHandle;

/**
  * @brief User-defined callback function.
  */
typedef struct {
    void (*TxCplCallback)(void* handle); /**< Sending completion callback function. */
    void (*RxCplCallback)(void* handle); /**< Receive completion callback function. */
    void (*ErrorCallback)(void* handle); /**< Error callback function. */
} I2C_UserCallBack;

/**
  * @}
  */

/**
  * @defgroup I2C_Reg_Def I2C Register Definition
  * @brief register mapping structure
  * @{
  */

/**
  * @brief I2C global configuration registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int i2c_enable        : 1;   /**< Enable Control, 0:disable, 1:enble. */
        unsigned int reserved0         : 7;
        unsigned int sda_hold_duration : 16;  /**< SDA hold time. */
        unsigned int reserved1         : 8;
    } BIT;
} volatile I2C_GLB_REG;

/**
  * @brief I2C high level duration registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int i2c_high_duration : 16;   /**< SCL high level duration. */
        unsigned int reserved0         : 16;
    } BIT;
} volatile I2C_HCNT_REG;

/**
  * @brief I2C low level duration registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int i2c_low_duration : 16;   /**< SCL Low Level Duration. */
        unsigned int reserved0        : 16;
    } BIT;
} volatile I2C_LCNT_REG;

/**
  * @brief I2C device address registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int dev_addr_byte1 : 8;   /**< Device Address Byte 1. */
        unsigned int dev_addr_byte2 : 8;   /**< Device Address Byte 2. */
        unsigned int dev_addr_byte3 : 8;   /**< Device Address Byte 3. */
        unsigned int dev_addr_byte4 : 8;   /**< Device Address Byte 4. */
    } BIT;
} volatile I2C_DEV_ADDR_REG;

/**
  * @brief I2C data buffer registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int data_buf_byte1 : 8;   /**< Data buffer byte 1. */
        unsigned int data_buf_byte2 : 8;   /**< Data buffer byte 2. */
        unsigned int data_buf_byte3 : 8;   /**< Data buffer byte 3. */
        unsigned int data_buf_byte4 : 8;   /**< Data buffer byte 4. */
    } BIT;
} volatile I2C_DATA_BUF_REG;

/**
  * @brief I2C indicates the pattern data 1 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pattern_data_byte1 : 8;   /**< PATTERN data byte 1. */
        unsigned int pattern_data_byte2 : 8;   /**< PATTERN data byte 2. */
        unsigned int pattern_data_byte3 : 8;   /**< PATTERN data byte 3. */
        unsigned int pattern_data_byte4 : 8;   /**< PATTERN data byte 4. */
    } BIT;
} volatile I2C_PATTERN_DATA1_REG;

/**
  * @brief I2C indicates the pattern data 1 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pattern_data_byte5 : 8;   /**< PATTERN data byte 5. */
        unsigned int pattern_data_byte6 : 8;   /**< PATTERN data byte 6. */
        unsigned int pattern_data_byte7 : 8;   /**< PATTERN data byte 7. */
        unsigned int pattern_data_byte8 : 8;   /**< PATTERN data byte 8. */
    } BIT;
} volatile I2C_PATTERN_DATA2_REG;

/**
  * @brief I2C TX FIFO data registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int tx_fifo   : 8;    /**< TX FIFO entry. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile I2C_TX_FIFO_REG;

/**
  * @brief I2C RX FIFO data registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int rx_fifo   : 8;    /**< RX FIFO entry. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile I2C_RX_FIFO_REG;

/**
  * @brief I2C timing command registers union structure definition.
  */
typedef union {
    unsigned int reg[32];
    struct {
        unsigned int timing_cmd : 5;    /**< Timing Commands. */
        unsigned int reserved0  : 27;
    } BIT[32];
} volatile I2C_TIMING_CMD_REG;

/**
  * @brief I2C cycle count 1 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int loop_num1 : 32;   /**< Specifies the number of cycles. */
    } BIT;
} volatile I2C_LOOP1_REG;

/**
  * @brief I2C jump destination 1 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int dst_timing_cmd1 : 5;    /**< Specifies which timing command to jump. */
        unsigned int reserved0       : 27;
    } BIT;
} volatile I2C_DST1_REG;

/**
  * @brief I2C cycle count 2 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int loop_num2 : 32;   /**< Specifies the number of cycles. */
    } BIT;
} volatile I2C_LOOP2_REG;

/**
  * @brief I2C jump destination 2 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int dst_timing_cmd2 : 5;    /**< Specifies which timing command to jump. */
        unsigned int reserved0       : 27;
    } BIT;
} volatile I2C_DST2_REG;

/**
  * @brief I2C cycle count 3 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int loop_num3 : 32;    /**< Specifies which timing command to jump. */
    } BIT;
} volatile I2C_LOOP3_REG;

/**
  * @brief I2C jump destination 3 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int dst_timing_cmd3 : 5;    /**< Specifies which timing command to jump. */
        unsigned int reserved0       : 27;
    } BIT;
} volatile I2C_DST3_REG;

/**
  * @brief I2C TX FIFO threshold registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int tx_watermark : 6;    /**< TX FIFO Threshold. */
        unsigned int reserved0    : 26;
    } BIT;
} volatile I2C_TX_WATERMARK_REG;

/**
  * @brief I2C RX FIFO threshold registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int rx_watermark : 6;    /**< RX FIFO Threshold. */
        unsigned int reserved0    : 26;
    } BIT;
} volatile I2C_RX_WATERMARK_REG;

/**
  * @brief I2C control 1 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int start         : 1;    /**< Start-up control. */
        unsigned int reserved1     : 7;
        unsigned int dma_operation : 2;    /**< DMA operation control. */
        unsigned int reserved0     : 22;
    } BIT;
} volatile I2C_CTRL1_REG;

/**
  * @brief I2C control 2 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int force_sda_oen : 1;   /**< Controls the SDA pin level, 0:low level, 1:high level. */
        unsigned int  reserved0    : 3;
        unsigned int force_scl_oen : 1;   /**< Controls the SCL pin level, 0:low level, 1:high level. */
        unsigned int reserved1     : 3;
        unsigned int gpio_mode     : 1;   /**< Use gpio mode, 0:disable, 1:enable. */
        unsigned int reserved2     : 7;
        unsigned int i2c_sda_in    : 1;   /**< Monitors external the SDA level, 0:low level, 1:high level. */
        unsigned int  reserved3    : 3;
        unsigned int i2c_scl_in    : 1;   /**< Monitors external the SCL level, 0:low level, 1:high level. */
        unsigned int reserved4     : 3;
        unsigned int i2c_sda_oen   : 1;   /**< Monitors internal the SDA level, 0:low level, 1:high level. */
        unsigned int reserved5     : 3;
        unsigned int i2c_scl_oen   : 1;   /**< Monitors internal the SCL level, 0:low level, 1:high level. */
        unsigned int reserved6     : 3;
    } BIT;
} volatile I2C_CTRL2_REG;

/**
  * @brief I2C FIFO status registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int rx_fifo_vld_num   : 7;   /**< Number of valid values in the RX FIFO. */
        unsigned int reserved0         : 1;
        unsigned int tx_fifo_vld_num   : 7;   /**< Number of valid values in the TX FIFO. */
        unsigned int reserved1         : 1;
        unsigned int rx_fifo_not_empty : 1;   /**< RX FIFO non-empty indicator, 0:empty, 1:non-empty. */
        unsigned int rx_fifo_not_full  : 1;   /**< RX FIFO non-full indicator, 0:full, 1:non-full. */
        unsigned int tx_fifo_not_empty : 1;   /**< TX FIFO non-empty indicator, 0:empty, 1:non-empty. */
        unsigned int tx_fifo_not_full  : 1;   /**< TX FIFO non-full indicator, 0:full, 1:non-full. */
        unsigned int reserved2         : 12;
    } BIT;
} volatile I2C_FIFO_STAT_REG;

/**
  * @brief I2C raw interrupt registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ack_bit_unmatch_raw : 1;   /**< Ack unmatch interrupt, 0:no interrupt, 1:interrupt. */
        unsigned int reserved0           : 1;
        unsigned int rx_gt_watermark_raw : 1;   /**< Above the rx watermark, 0:no interrupt, 1:interrupt. */
        unsigned int reserved1           : 1;
        unsigned int tx_lt_watermark_raw : 1;   /**< Below the tx watermark, 0:no interrupt, 1:interrupt. */
        unsigned int reserved2           : 4;
        unsigned int stop_det_raw        : 1;   /**< Stop detected. */
        unsigned int start_det_raw       : 1;   /**< Start detected. */
        unsigned int arb_lost_raw        : 1;   /**< Arbitration loss. */
        unsigned int all_cmd_done_raw    : 1;   /**< Timing command sequence execution completed. */
        unsigned int reserved3           : 19;
    } BIT;
} volatile I2C_INTR_RAW_REG;

/**
  * @brief I2C interrupt enable registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ack_bit_unmatch_en : 1;   /**< Interrupt enable, 0:disable, 1:enable. */
        unsigned int reserved0          : 1;
        unsigned int rx_gt_watermark_en : 1;   /**< Interrupt enable, 0:disable, 1:enable. */
        unsigned int reserved1          : 1;
        unsigned int tx_lt_watermark_en : 1;   /**< Interrupt enable, 0:disable, 1:enable. */
        unsigned int reserved2          : 4;
        unsigned int stop_det_en        : 1;   /**< Interrupt enable, 0:disable, 1:enable. */
        unsigned int start_det_en       : 1;   /**< Interrupt enable, 0:disable, 1:enable. */
        unsigned int arb_lost_en        : 1;   /**< Interrupt enable, 0:disable, 1:enable. */
        unsigned int all_cmd_done_en    : 1;   /**< Interrupt enable, 0:disable, 1:enable. */
        unsigned int reserved3          : 19;
    } BIT;
} volatile I2C_INTR_EN_REG;

/**
  * @brief I2C interrupt status registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ack_bit_unmatch : 1;   /**< Interrupt status flag, 0:no interrupt, 1:interrupt. */
        unsigned int reserved0       : 1;
        unsigned int rx_gt_watermark : 1;   /**< Interrupt status flag, 0:no interrupt, 1:interrupt. */
        unsigned int reserved1       : 1;
        unsigned int tx_lt_watermark : 1;   /**< Interrupt status flag, 0:no interrupt, 1:interrupt. */
        unsigned int reserved2       : 4;
        unsigned int stop_det        : 1;   /**< Interrupt status flag, 0:no interrupt, 1:interrupt. */
        unsigned int start_det       : 1;   /**< Interrupt status flag, 0:no interrupt, 1:interrupt. */
        unsigned int arb_lost        : 1;   /**< Interrupt status flag, 0:no interrupt, 1:interrupt. */
        unsigned int all_cmd_done    : 1;   /**< Interrupt status flag, 0:no interrupt, 1:interrupt. */
        unsigned int reserved3       : 19;
    } BIT;
} volatile I2C_INTR_STAT_REG;

/**
  * @brief I2C Register definition structure
  */
typedef struct {
    I2C_GLB_REG           I2C_GLB;           /**< Global configuration register. Offset Address: 0x0000. */
    I2C_HCNT_REG          I2C_HCNT;          /**< High level duration register. Offset Address: 0x0004. */
    I2C_LCNT_REG          I2C_LCNT;          /**< Low level duration register. Offset Address: 0x0008. */
    unsigned char         space0[4];
    I2C_DEV_ADDR_REG      I2C_DEV_ADDR;      /**< Device address register. Offset Address: 0x0010. */
    I2C_DATA_BUF_REG      I2C_DATA_BUF;      /**< Data buffer register. Offset Address: 0x0014. */
    I2C_PATTERN_DATA1_REG I2C_PATTERN_DATA1; /**< PATTERN data 1 register. Offset Address: 0x0018. */
    I2C_PATTERN_DATA2_REG I2C_PATTERN_DATA2; /**< PATTERN data 2 register. Offset Address: 0x001C. */
    I2C_TX_FIFO_REG       I2C_TX_FIFO;       /**< TX FIFO data register. Offset Address: 0x0020. */
    I2C_RX_FIFO_REG       I2C_RX_FIFO;       /**< RX FIFO data register. Offset Address: 0x0024. */
    unsigned char         space1[8];
    I2C_TIMING_CMD_REG    I2C_TIMING_CMD;    /**< Timing command register. Offset Address: 0x0030. */
    I2C_LOOP1_REG         I2C_LOOP1;         /**< Cycle count 1 register. Offset Address: 0x00b0. */
    I2C_DST1_REG          I2C_DST1;          /**< Jump destination 1 register. Offset Address: 0x00b4. */
    I2C_LOOP2_REG         I2C_LOOP2;         /**< Cycle count 2 register. Offset Address: 0x00b8. */
    I2C_DST2_REG          I2C_DST2;          /**< Jump destination 2 register. Offset Address: 0x00bc. */
    I2C_LOOP3_REG         I2C_LOOP3;         /**< Cycle count 3 register. Offset Address: 0x00c0. */
    I2C_DST3_REG          I2C_DST3;          /**< Jump destination 3 register. Offset Address: 0x00c4. */
    I2C_TX_WATERMARK_REG  I2C_TX_WATERMARK;  /**< TX FIFO threshold register. Offset Address: 0x00c8. */
    I2C_RX_WATERMARK_REG  I2C_RX_WATERMARK;  /**< RX FIFO threshold register. Offset Address: 0x00cc. */
    I2C_CTRL1_REG         I2C_CTRL1;         /**< Control 1 register. Offset Address: 0x00d0. */
    I2C_CTRL2_REG         I2C_CTRL2;         /**< Control 2 register. Offset Address: 0x00d4. */
    I2C_FIFO_STAT_REG     I2C_FIFO_STAT;     /**< FIFO status register.Offset Address: 0x00d8. */
    unsigned char         space2[4];
    I2C_INTR_RAW_REG      I2C_INTR_RAW;      /**< Raw interrupt register. Offset Address: 0x00e0. */
    I2C_INTR_EN_REG       I2C_INTR_EN;       /**< Interrupt enable register. Offset Address: 0x00e4. */
    I2C_INTR_STAT_REG     I2C_INTR_STAT;     /**< Interrupt status register. Offset Address: 0x00e8 .*/
} volatile I2C_RegStruct;
/**
  * @}
  */

/* Parameter check definition-------------------------------------------*/
/**
  * @brief Check address mode selection.
  * @param addrMode I2C instance
  * @retval true
  * @retval false
  */
static inline bool IsI2cAddressMode(I2C_AddressMode addrMode)
{
    return (addrMode == I2C_7_BITS ||
            addrMode == I2C_10_BITS);
}

/**
  * @brief Check i2c sda hold time.
  * @param sdaHoldTime I2C instance
  * @retval true
  * @retval false
  */
static inline bool IsI2cSdaHoldTime(unsigned int sdaHoldTime)
{
    return (sdaHoldTime <= 0xFFFF); /* SdaHoldTime value is 0 to 0xFFFF */
}

/**
  * @brief Check i2c freq.
  * @param freq I2C freq
  * @retval true
  * @retval false
  */
static inline bool IsI2cFreq(unsigned int freq)
{
    return (freq > 0);
}

/**
  * @brief Check i2c ignore ack flag.
  * @param ignoreAckFlag I2C ignore ack flag.
  * @retval true
  * @retval false
  */
static inline bool IsI2cIgnoreAckFlag(unsigned int ignoreAckFlag)
{
    return (ignoreAckFlag == I2C_IGNORE_NAK_ENABLE ||
            ignoreAckFlag == I2C_IGNORE_NAK_DISABLE);
}

/**
  * @brief Check i2c tx water mark.
  * @param txWaterMark I2C tx water mark.
  * @retval true
  * @retval false
  */
static inline bool IsI2cTxWaterMark(unsigned int txWaterMark)
{
    return (txWaterMark <= 0x3F); /* The txWaterMark value is 0 to 0x3F */
}

/**
  * @brief Check i2c rx water mark.
  * @param rxWaterMark I2C rx water mark.
  * @retval true
  * @retval false
  */
static inline bool IsI2cRxWaterMark(unsigned int rxWaterMark)
{
    return (rxWaterMark <= 0x3F); /* The rxWaterMark value is 0 to 0x3F */
}

/**
  * @brief DCL I2C enable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_Enable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_GLB.BIT.i2c_enable = BASE_CFG_SET;
}

/**
  * @brief DCL I2C disable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_Disable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_GLB.BIT.i2c_enable = BASE_CFG_UNSET;
}

/**
  * @brief DCL I2C get enable state.
  * @param i2cx I2C register base address.
  * @retval enable state 0 or 1.
  */
static inline int DCL_I2C_GetEnableState(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_GLB.BIT.i2c_enable;
}

/**
  * @brief DCL Configuring i2c SDA Hold Time.
  * @param i2cx I2C register base address.
  * @param sdaHoldTime Sda hold time.
  * @retval None.
  */
static inline void DCL_I2C_SetSdaHoldDuration(I2C_RegStruct *i2cx, unsigned short sdaHoldTime)
{
    unsigned int glbReg;
    unsigned int temp;
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    /* Read the entire register and write it back. */
    temp = ((unsigned int)sdaHoldTime) << I2C_SDA_HOLD_DURATION_POS;
    glbReg = (i2cx->I2C_GLB.reg & (~I2C_SDA_HOLD_DURATION_MASK)) | temp;
    i2cx->I2C_GLB.reg = glbReg;
}

/**
  * @brief Get DCL Configuring i2c SDA Hold Time.
  * @param i2cx I2C register base address.
  * @retval Sda hold time,0-65535.
  */
static inline int DCL_I2C_GetSdaHoldDuration(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return ((i2cx->I2C_GLB.reg >> I2C_SDA_HOLD_DURATION_POS) & 0xFFFF); /* 0xFFFF is mask of sda hold time. */
}

/**
  * @brief DCL Configuring i2c SCL High Hold Time.
  * @param i2cx I2C register base address.
  * @param sclHighTime Scl high hold time.
  * @retval None.
  */
static inline void DCL_I2C_SetHighDuration(I2C_RegStruct *i2cx, unsigned short sclHighTime)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_HCNT.BIT.i2c_high_duration = sclHighTime;
}

/**
  * @brief DCL get i2c SCL High Hold Time.
  * @param i2cx I2C register base address.
  * @retval Scl high hold time,0-65535.
  */
static inline int DCL_I2C_GetHighDuration(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_HCNT.BIT.i2c_high_duration;
}

/**
  * @brief DCL Configuring i2c SCL low Hold Time.
  * @param i2cx I2C register base address.
  * @param sclLowTime scl low hold time.
  * @retval None.
  */
static inline void DCL_I2C_SetLowDuration(I2C_RegStruct *i2cx, unsigned short sclLowTime)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_LCNT.BIT.i2c_low_duration = sclLowTime;
}

/**
  * @brief DCL Get i2c SCL low Hold Time.
  * @param i2cx I2C register base address.
  * @retval Scl low hold time,0-65535.
  */
static inline int DCL_I2C_GetLowDuration(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_LCNT.BIT.i2c_low_duration;
}

/**
  * @brief DCL Set I2C Slave Address.
  * @param i2cx I2C register base address.
  * @param devAddr Slave address
  * @retval None.
  */
static inline void DCL_I2C_SetDevAddr(I2C_RegStruct *i2cx, unsigned int devAddr)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_DEV_ADDR.reg = devAddr;
}

/**
  * @brief DCL Get I2C Slave Address.
  * @param i2cx I2C register base address.
  * @retval Slave address.
  */
static inline unsigned int DCL_I2C_GetDevAddr(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_DEV_ADDR.reg;
}

/**
  * @brief DCL Set I2C data buffer.
  * @param i2cx I2C register base address.
  * @param dataBuff Buffer data.
  * @retval None.
  */
static inline void DCL_I2C_SetDataBuff(I2C_RegStruct *i2cx, unsigned int dataBuff)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_DATA_BUF.reg = dataBuff;
}

/**
  * @brief DCL Get I2C data buffer.
  * @param i2cx I2C register base address.
  * @retval Buffer data.
  */
static inline unsigned int DCL_I2C_GetDataBuff(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_DATA_BUF.reg;
}

/**
  * @brief DCL Set I2C pattern data1.
  * @param i2cx I2C register base address.
  * @param patternData Pattern data1.
  * @retval None.
  */
static inline void DCL_I2C_SetPatternData1(I2C_RegStruct *i2cx, unsigned int patternData)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_PATTERN_DATA1.reg = patternData;
}

/**
  * @brief DCL Get I2C pattern data1.
  * @param i2cx I2C register base address.
  * @retval Pattern data1.
  */
static inline unsigned int DCL_I2C_GetPatternData1(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_PATTERN_DATA1.reg;
}

/**
  * @brief DCL Set I2C pattern data2.
  * @param i2cx I2C register base address.
  * @param patternData Pattern data2.
  * @retval None.
  */
static inline void DCL_I2C_SetPatternData2(I2C_RegStruct *i2cx, unsigned int patternData)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_PATTERN_DATA2.reg = patternData;
}

/**
  * @brief DCL Get I2C pattern data2.
  * @param i2cx I2C register base address.
  * @retval Pattern data2.
  */
static inline unsigned int DCL_I2C_GetPatternData2(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_PATTERN_DATA2.reg;
}

/**
  * @brief DCL Set I2C Tx fifo.
  * @param i2cx I2C register base address.
  * @param fifoData Tx fifo data.
  * @retval None.
  */
static inline void DCL_I2C_SetTxFifo(I2C_RegStruct *i2cx, unsigned char fifoData)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_TX_FIFO.BIT.tx_fifo = fifoData;
}

/**
  * @brief DCL Get I2C Rx fifo.
  * @param i2cx I2C register base address.
  * @retval Rx fifo data.
  */
static inline int DCL_I2C_GetRxFifo(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_RX_FIFO.BIT.rx_fifo;
}

/**
  * @brief DCL Set I2C timing cmd.
  * @param i2cx I2C register base address.
  * @param cmd Timing cmd
  * @param offset Instruction storage offset position
  * @retval None.
  */
static inline void DCL_I2C_SetTimingCmd(I2C_RegStruct *i2cx, I2C_CmdType cmd, unsigned char offset)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    I2C_ASSERT_PARAM(offset < I2C_MAX_CMD_OFFSET_LEN);
    i2cx->I2C_TIMING_CMD.BIT[offset].timing_cmd = cmd;
}

/**
  * @brief DCL Get I2C timing cmd.
  * @param i2cx I2C register base address.
  * @param offset Instruction storage offset position
  * @retval Timing cmd.
  */
static inline unsigned int DCL_I2C_GetTimingCmd(I2C_RegStruct *i2cx, unsigned char offset)
{
    I2C_ASSERT_PARAM(offset < I2C_MAX_CMD_OFFSET_LEN);
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_TIMING_CMD.BIT[offset].timing_cmd;
}

/**
  * @brief DCL Set I2C timing loop1 number.
  * @param i2cx I2C register base address.
  * @param loopValue Number of loop
  * @retval None.
  */
static inline void DCL_I2C_SetLoop1(I2C_RegStruct *i2cx, unsigned int loopValue)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_LOOP1.BIT.loop_num1 = loopValue;
}

/**
  * @brief DCL Get I2C timing loop1 number.
  * @param i2cx I2C register base address.
  * @retval Number of loop.
  */
static inline unsigned int DCL_I2C_GetLoop1(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_LOOP1.BIT.loop_num1;
}

/**
  * @brief DCL Set I2C timing loop2 number.
  * @param i2cx I2C register base address.
  * @param loopValue Number of loop
  * @retval None.
  */
static inline void DCL_I2C_SetLoop2(I2C_RegStruct *i2cx, unsigned int loopValue)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_LOOP2.BIT.loop_num2 = loopValue;
}

/**
  * @brief DCL Get I2C timing loop2 number.
  * @param i2cx I2C register base address.
  * @retval Number of loop.
  */
static inline unsigned int DCL_I2C_GetLoop2(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_LOOP2.BIT.loop_num2;
}

/**
  * @brief DCL Set I2C timing loop3 number.
  * @param i2cx I2C register base address.
  * @param loopValue Number of loop
  * @retval None.
  */
static inline void DCL_I2C_SetLoop3(I2C_RegStruct *i2cx, unsigned int loopValue)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_LOOP3.BIT.loop_num3 = loopValue;
}

/**
  * @brief DCL Get I2C timing loop3 number.
  * @param i2cx I2C register base address.
  * @retval Number of loop.
  */
static inline unsigned int DCL_I2C_GetLoop3(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_LOOP3.BIT.loop_num3;
}

/**
  * @brief Configuring the Command for Jumping to a Specified Timing.
  * @param i2cx I2C register base address.
  * @param dstTimingCmd Jump timing command position
  * @retval None.
  */
static inline void DCL_I2C_SetDst1(I2C_RegStruct *i2cx, unsigned char dstTimingCmd)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_DST1.BIT.dst_timing_cmd1 = dstTimingCmd;
}

/**
  * @brief Get the Command for Jumping to a Specified Timing.
  * @param i2cx I2C register base address.
  * @retval Jump timing command position.
  */
static inline unsigned int DCL_I2C_GetDst1(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_DST1.BIT.dst_timing_cmd1;
}

/**
  * @brief Configuring the Command for Jumping to a Specified Timing.
  * @param i2cx I2C register base address.
  * @param dstTimingCmd Jump timing command position
  * @retval None.
  */
static inline void DCL_I2C_SetDst2(I2C_RegStruct *i2cx, unsigned char dstTimingCmd)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_DST2.BIT.dst_timing_cmd2 = dstTimingCmd;
}

/**
  * @brief Get the Command for Jumping to a Specified Timing.
  * @param i2cx I2C register base address.
  * @retval Jump timing command position.
  */
static inline unsigned int DCL_I2C_GetDst2(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_DST2.BIT.dst_timing_cmd2;
}

/**
  * @brief Configuring the Command for Jumping to a Specified Timing.
  * @param i2cx I2C register base address.
  * @param dstTimingCmd Jump timing command position
  * @retval None.
  */
static inline void DCL_I2C_SetDst3(I2C_RegStruct *i2cx, unsigned char dstTimingCmd)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_DST3.BIT.dst_timing_cmd3 = dstTimingCmd;
}

/**
  * @brief Get the Command for Jumping to a Specified Timing.
  * @param i2cx I2C register base address.
  * @retval Jump timing command position.
  */
static inline unsigned int DCL_I2C_GetDst3(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_DST3.BIT.dst_timing_cmd3;
}

/**
  * @brief Set the I2C TX threshold.
  * @param i2cx I2C register base address.
  * @param waterMark I2C Tx threshold, 0-63.
  * @retval None.
  */
static inline void DCL_I2C_SetTxWaterMark(I2C_RegStruct *i2cx, unsigned char waterMark)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_TX_WATERMARK.BIT.tx_watermark = waterMark;
}

/**
  * @brief Get the I2C TX threshold.
  * @param i2cx I2C register base address.
  * @retval I2C tx threshold.
  */
static inline unsigned int DCL_I2C_GetTxWaterMark(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_TX_WATERMARK.BIT.tx_watermark;
}

/**
  * @brief Set the I2C RX threshold.
  * @param i2cx I2C register base address.
  * @param waterMark I2C Rx threshold, 0-63.
  * @retval None.
  */
static inline void DCL_I2C_SetRxWaterMark(I2C_RegStruct *i2cx, unsigned char waterMark)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_RX_WATERMARK.BIT.rx_watermark = waterMark;
}

/**
  * @brief Get the I2C RX threshold.
  * @param i2cx I2C register base address.
  * @param waterMark I2C Rx threshold, 0-63.
  * @retval I2C rx threshold.
  */
static inline int DCL_I2C_GetRxWaterMark(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_RX_WATERMARK.BIT.rx_watermark;
}

/**
  * @brief Set the I2C DMA mode.
  * @param i2cx I2C register base address.
  * @param mode I2C DMA operation mode.
  * @retval None.
  */
static inline void DCL_I2C_SetDmaMode(I2C_RegStruct *i2cx, unsigned char mode)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_CTRL1.BIT.dma_operation = mode;
}

/**
  * @brief Start and stop I2C timing execution.
  * @param i2cx I2C register base address.
  * @param startStop start : 1, stop :0.
  * @retval None.
  */
static inline void DCL_I2C_SetStart(I2C_RegStruct *i2cx, unsigned char startStop)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_CTRL1.BIT.start = startStop;
}

/**
  * @brief Get start and stop I2C timing status.
  * @param i2cx I2C register base address.
  * @retval 0 or 1.
  */
static inline unsigned int DCL_I2C_GetStart(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_CTRL1.BIT.start;
}

/**
  * @brief Set the low level of the SDA pin.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_SetFroceSdaOenLowLevel(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_CTRL2.BIT.force_sda_oen = BASE_CFG_UNSET;
}

/**
  * @brief Set the high level of the SDA pin.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_SetFroceSdaOenHighLevel(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_CTRL2.BIT.force_sda_oen = BASE_CFG_SET;
}

/**
  * @brief Get the level of the SDA pin.
  * @param i2cx I2C register base address.
  * @retval 0 or 1
  */
static inline unsigned int DCL_I2C_GetFroceSdaOen(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_CTRL2.BIT.force_sda_oen;
}

/**
  * @brief Set the SCL and SDA pins of the I2C to GPIO mode.
  * @param i2cx I2C register base address.
  * @param mode 0 disable,1 enable.
  * @retval None.
  */
static inline void DCL_I2C_SetGpioMode(I2C_RegStruct *i2cx, unsigned char mode)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_CTRL2.BIT.gpio_mode = mode;
}

/**
  * @brief Get the SCL and SDA pins of the I2C to GPIO mode.
  * @param i2cx I2C register base address.
  * @retval 0 or 1
  */
static inline unsigned int DCL_I2C_GetGpioMode(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_CTRL2.BIT.gpio_mode;
}

/**
  * @brief Get the level of the external I2C bus SDA.
  * @param i2cx I2C register base address.
  * @retval 0 or 1
  */
static inline unsigned int DCL_I2C_GetSdaIn(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_CTRL2.BIT.i2c_sda_in;
}

/**
  * @brief Get the SDA output level.
  * @param i2cx I2C register base address.
  * @retval 0 or 1.
  */
static inline unsigned int DCL_I2C_GetSdaOen(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_CTRL2.BIT.i2c_sda_oen;
}

/**
  * @brief Get the SCL output level.
  * @param i2cx I2C register base address.
  * @retval 0 or 1.
  */
static inline unsigned int DCL_I2C_GetSclOen(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_CTRL2.BIT.i2c_scl_oen;
}

/**
  * @brief Enable SDA output level of the internal I2C.
  * @param i2cx I2C register base address.
  * @retval null.
  */
static inline void DCL_I2C_SclOenEnable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_CTRL2.BIT.i2c_scl_oen = BASE_CFG_ENABLE;
}

/**
  * @brief Disable SDA output level of the internal I2C.
  * @param i2cx I2C register base address.
  * @retval null.
  */
static inline void DCL_I2C_SclOenDisable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_CTRL2.BIT.i2c_scl_oen = BASE_CFG_DISABLE;
}

/**
  * @brief Check whether the TX FIFO is not full.
  * @param i2cx I2C register base address.
  * @retval true or false.
  */
static inline bool DCL_I2C_CheckTxFifoNotFull(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_FIFO_STAT.BIT.rx_fifo_not_full;
}

/**
  * @brief Check whether the TX FIFO is not empty.
  * @param i2cx I2C register base address.
  * @retval true or false.
  */
static inline bool DCL_I2C_CheckTxFifoNotEmpty(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_FIFO_STAT.BIT.rx_fifo_not_empty;
}

/**
  * @brief Check whether the RX FIFO is not full.
  * @param i2cx I2C register base address.
  * @retval true or false.
  */
static inline bool DCL_I2C_CheckRxFifoNotFull(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_FIFO_STAT.BIT.tx_fifo_not_full;
}

/**
  * @brief Check whether the RX FIFO is not empty.
  * @param i2cx I2C register base address.
  * @retval true or false.
  */
static inline bool DCL_I2C_CheckRxFifoNotEmpty(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_FIFO_STAT.BIT.tx_fifo_not_empty;
}

/**
  * @brief Clear Interrupt.
  * @param i2cx I2C register base address.
  * @param intrRaw Corresponding interrupt bit, for example, 110011.
  * @retval None.
  */
static inline void DCL_I2C_ClearIrq(I2C_RegStruct *i2cx, unsigned int intrRaw)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_INTR_RAW.reg = intrRaw;
}

/**
  * @brief Obtaining the Interrupt Raw Configuration.
  * @param i2cx I2C register base address.
  * @retval Interrupt raw status value.
  */
static inline unsigned int DCL_I2C_GetInterruptRaw(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_INTR_RAW.reg;
}

/**
  * @brief Configuring Interrupt Enable.
  * @param i2cx I2C register base address.
  * @param intrEn Corresponding interrupt enable bit, for example, 110011.
  * @retval None.
  */
static inline void DCL_I2C_SetInterruptEn(I2C_RegStruct *i2cx, unsigned int intrEn)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_INTR_EN.reg = intrEn;
}

/**
  * @brief Obtaining the Interrupt Enable Configuration.
  * @param i2cx I2C register base address.
  * @retval Interrupt enable value.
  */
static inline unsigned int DCL_I2C_GetInterruptEn(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_INTR_EN.reg;
}

/**
  * @brief Obtains the interrupt status.
  * @param i2cx I2C register base address.
  * @retval Interrupt Status.
  */
static inline unsigned int DCL_I2C_GetInterruptStatus(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_INTR_STAT.reg;
}

/**
  * @}
  */

/**
  * @}
  */
#endif /* #ifndef McuMagicTag_I2C_IP_H */