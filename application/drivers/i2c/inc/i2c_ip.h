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
  *          + Extended Configuration Parameter Struct Definition.
  *          + Register definition structure.
  *          + Timing command enumeration.
  *          + Direct configuration layer interface.
  *          + Basic parameter configuration macro.
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
  * @brief I2C_IP: i2c_v1
  * @{
  */

#define I2C_IGNORE_NAK_ENABLE        BASE_CFG_ENABLE  /**< Ignore acknowledgment configuration enable. */
#define I2C_IGNORE_NAK_DISABLE       BASE_CFG_DISABLE /**< Ignore acknowledgment configuration disable. */

#define I2C_STANDARD_FREQ_TH     100000      /**< Standard mode,the frequency band is less than or equal to 100 kHz. */

#define I2C_INTR_RAW_ALL_ENABLE  0x00FFFFFFU /**< 1111111111111 */
#define I2C_INTR_RAW_ALL_DISABLE  0x00000000U /**< 0000000000000 */

#define I2C_INTR_EN_ALL_ENABLE   0x00FFFFFFU /**< 1111111111111 */
#define I2C_INTR_EN_ALL_DISABLE  0x00000000U /**< 0000000000000 */

#define I2C_ONCE_TRANS_MAX_NUM   0x400

#define I2C_SCL_HIGH_TIME_POS        16
#define I2C_SCL_HIGHT_TIME_MASK     (0xFFFF << I2C_SCL_HIGH_TIME_POS)
#define I2C_SCL_LOW_TIME_POS         0
#define I2C_SCL_LOW_TIME_MASK       (0xFFFF << I2C_SCL_LOW_TIME_POS)

#define I2C_SDA_HOLD_DURATION_POS    16
#define I2C_SDA_HOLD_DURATION_MASK  (0xFFFF << I2C_SDA_HOLD_DURATION_POS)

#define I2C_TXFIFO_WDATA_POS         0
#define I2C_TXFIFO_WDATA_MASK       (0xFF << I2C_TXFIFO_WDATA_POS)
#define I2C_TXFIFO_CMD_POS           8
#define I2C_TXFIFO_CMD_MASK         (0xF << I2C_TXFIFO_CMD_POS)

#define XMBUS_OWN_ADDRESS_MASK       0x3FF

/**
  * @defgroup I2C_Param_Def I2C Parameters Definition.
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
    I2C_DMA_OP_NONE        = 0x00000000U,
    I2C_DMA_OP_READ        = 0x00000001U,
    I2C_DMA_OP_WRITE       = 0x00000002U,
    I2C_DMA_OP_WRITE_READ  = 0x00000003U
} I2C_DmaOperationType;

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
  * @brief Callback Function ID Enumeration Definition
  */
typedef enum {
    I2C_MASTER_TX_COMPLETE_CB_ID = 0x00000000U,
    I2C_MASTER_RX_COMPLETE_CB_ID = 0x00000001U,
    I2C_SLAVE_TX_COMPLETE_CB_ID  = 0x00000002U,
    I2C_SLAVE_RX_COMPLETE_CB_ID  = 0x00000003U,
    I2C_ERROR_CB_ID              = 0x00000004U,
} I2C_CallbackId;
/**
  * @brief I2C operation timing enumeration definition
  */
typedef enum {
    I2C_CMD_INT1                       = 0x00000000U,
    I2C_CMD_S                          = 0x00000001U,
    I2C_CMD_M_TD_RACK_S_RD_TACK        = 0x00000002U,
    I2C_CMD_M_TD_RNACK_S_RD_TNACK      = 0x00000003U,
    I2C_CMD_M_TD_S_RD                  = 0x00000004U,
    I2C_CMD_M_RD_TACK_S_TD_RACK        = 0x00000005U,
    I2C_CMD_M_RD_TNACK_S_TD_RNACK      = 0x00000006U,
    I2C_CMD_M_RD_S_TD                  = 0x00000007U,
    I2C_CMD_M_TPEC_RACK_S_RPEC_TACK    = 0x0000000AU,
    I2C_CMD_M_TPEC_RNACK_S_RPEC_TNACK  = 0x0000000BU,
    I2C_CMD_M_RPEC_TACK_S_TPEC_RACK    = 0x0000000DU,
    I2C_CMD_M_RPEC_TNACK_S_TPEC_RNACK  = 0x0000000EU,
    I2C_CMD_P                          = 0x0000000FU
} I2C_CmdType;

/**
  * @brief I2C data transfer sequence enumeration definition.
  */
typedef enum {
    I2C_BIG_BIT_FIRST         = 0x00000000U,
    I2C_LITTLE_BIT_FIRST      = 0x00000001U,
} I2C_DataTransferSequenceType;

/**
  * @brief I2C clock stretching enumeration definition.
  */
typedef enum {
    I2C_CLOCK_STRETCH_ENABLE   = 0x00000000U,
    I2C_CLOCK_STRETCH_DISABLE  = 0x00000001U,
} I2C_ClockStretchType;

/**
  * @brief I2C extend handle, configuring some special parameters.
  */
typedef struct {
    unsigned int    spikeFilterTime;          /**< The SDA and SCL Glitch Filtering Configuration. */
    unsigned int    sdaDelayTime;             /**< The SDA delay sampling configuration. */
    unsigned int    slaveOwnXmbAddressEnable; /**< Enable the I2C second own address function. */
    unsigned int    slaveOwnXmbAddress;       /**< The second own address as slave. */
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
  * @brief Description I2C register mapping structure
  * @{
  */

/**
  * @brief I2C mode configuration registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int mst_slv_function    : 2;  /**< Master and Slave Function Selection. */
        unsigned int lit_end             : 1;  /**< Data Transfer Sequence. 0:MSbit-First mode, 1:LSbit-First mode. */
        unsigned int xmb_pec_en          : 1;  /**< PEC calculation enable for SMBus and PMBus,
                                                    0:disable, 1:enable. */
        unsigned int rack_mode           : 1;  /**< ACK/NACK receiving mode, 0:ack mode, 1:ignore ack. */
        unsigned int scl_stretch_disable : 1;  /**< Clock stretching enable. 0:enable, 1:disable. */
        unsigned int reserved0           : 26;
    } BIT;
} volatile I2C_MODE_REG;

/**
  * @brief I2C SCL high-level and low-level duration registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int scl_low_time  : 16;  /**< SCL Low Level Duration. */
        unsigned int scl_high_time : 16;  /**< SCL High Level Duration. */
    } BIT;
} volatile I2C_SCL_CFG_REG;

/**
  * @brief I2C SDA timing configuration registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int sda_delay_time : 4;  /**< SDA delay sampling configuration. */
        unsigned int reserved0      : 12;
        unsigned int sda_hold_time  : 16; /**< SDA hold time configuration. */
    } BIT;
} volatile I2C_SDA_CFG_REG;

/**
  * @brief I2C Slave Address Configuration registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int own_address         : 10; /**< Own address as slave. */
        unsigned int reserved0           : 2;
        unsigned int own_address_mask    : 10; /**< Slave's own address mask. */
        unsigned int reserved1           : 2;
        unsigned int i2c_general_call_en : 1;  /**< Enable General Call Address Receiving, 0:disable, 1:enable. */
        unsigned int i2c_device_id_en    : 1;  /**< Enable the function of receiving device ID addresses,
                                                    0:disable, 1:enable. */
        unsigned int i2c_start_byte_en   : 1;  /**< Enable START Byte Address Receiving, 0:disable, 1:enable. */
        unsigned int i2c_10bit_slave_en  : 1;  /**< Enable 10bit Slave Addressing Receiving, 0:disable, 1:enable. */
        unsigned int reserved2           : 4;
    } BIT;
} volatile I2C_OWN_ADDR_REG;

/**
  * @brief I2C SMBus PMBus Device Dedicated Address Configuration registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int xmb_address           : 10;  /**< I2C SMBus PMBus Device Dedicated Address. */
        unsigned int reserved0             : 2;
        unsigned int xmb_address_mask      : 10;  /**< I2C SMBus PMBus Device Private Address Mask. */
        unsigned int xmb_address_en        : 1;   /**< I2C SMBus PMBus Device Dedicated Address Enable,
                                                       0:disable, 1:enable. */
        unsigned int reserved1             : 1;
        unsigned int smb_host_notify_en    : 1;   /**< Enable receiving SMBus Host Address, 0:disable, 1:enable. */
        unsigned int smb_alert_response_en : 1;   /**< Enable receiving SMBus Alert Response Address,
                                                       0:disable, 1:enable. */
        unsigned int smb_dev_default_en    : 1;   /**< Enable receiving SMBus Device Default Address,
                                                       0:disable, 1:enable. */
        unsigned int reserved2             : 1;
        unsigned int pmb_zone_read_en      : 1;   /**< Enable RX PMBus Zone Read Address, 0:disable, 1:enable. */
        unsigned int pmb_zone_write_en     : 1;   /**< Enable PMBus Zone Write Address Receiving,
                                                       0:disable, 1:enable. */
        unsigned int reserved3             : 2;
    } BIT;
} volatile XMB_DEV_ADDR_REG;

/**
  * @brief Address received by the I2C slave, R/W bit registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int rx_rw     : 1;   /**< The address received by the slave. */
        unsigned int rx_addr   : 10;  /**< R/W bit received by the slave. */
        unsigned int reserved0 : 21;
    } BIT;
} volatile I2C_RX_ADDR_REG;

/**
  * @brief I2C TX FIFO registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int tx_fifo_wdata : 8;  /**< The software writes the data to be sent, write only. */
        unsigned int mst_slv_cmd   : 4;  /**< Master Timing Command. */
        unsigned int reserved0     : 20;
    } BIT;
} volatile I2C_TX_FIFO_REG;

/**
  * @brief I2C RX FIFO registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int rx_fifo_rdata : 8;  /**< The software writes the data to be receive, read only. */
        unsigned int reserved0     : 24;
    } BIT;
} volatile I2C_RX_FIFO_REG;

/**
  * @brief I2C TX threshold registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int tx_watermark : 4;  /**< TX FIFO Threshold. */
        unsigned int reserved0    : 28;
    } BIT;
} volatile I2C_TX_WATERMARK_REG;

/**
  * @brief I2C RX threshold registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int rx_watermark : 4;  /**< RX FIFO Threshold. */
        unsigned int reserved0    : 28;
    } BIT;
} volatile I2C_RX_WATERMARK_REG;

/**
  * @brief I2C control 1 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int mst_start       : 1;  /**< Master startup control, 0:disable, 1:enable. */
        unsigned int reserved0       : 1;
        unsigned int rst_rx_fifo     : 1;  /**< Resetting the RX FIFO, 0:clearing completed, 1:clearing in progress. */
        unsigned int rst_tx_fifo     : 1;  /**< Resetting the TX FIFO, 0:clearing completed, 1:clearing in progress. */
        unsigned int reserved1       : 4;
        unsigned int dma_operation   : 2;  /**< DMA operation control. */
        unsigned int dma_rx_lsreq_en : 1;  /**< Flow control enable for the RX peripheral of the DMA I2C module,
                                                0:disable, 1:enable. */
        unsigned int reserved2       : 21;
    } BIT;
} volatile I2C_CTRL1_REG;

/**
  * @brief I2C control 2 registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int force_sda          : 1;  /**< Forcibly sets the value of an I2C pin, 0:set 0, 1:set 1. */
        unsigned int reserved0          : 3;
        unsigned int force_scl          : 1;  /**< Forcibly sets the value of an I2C pin, 0:set 0, 1:set 1. */
        unsigned int reserved1          : 3;
        unsigned int gpio_mode          : 1;  /**< The I2C pin is used as a GPIO pin. */
        unsigned int reserved2          : 3;
        unsigned int smb_alert_n_oe_n   : 1;  /**< SMBus SMBALERT# Output Value,
                                                   0:Low-level Open-Drain output,
                                                   1:Open-Drain output high impedance. */
        unsigned int smb_alert_n_in     : 1;  /**< SMBus SMBALERT# Input value,
                                                   0:Low level input, 1:High level input . */
        unsigned int reserved3          : 2;
        unsigned int smb_suspend_n_oe_n : 1;  /**< SMBus SMBSUS# is output, 0:push-pull output,
                                                   1:no output, high impedance. */
        unsigned int smb_suspend_n_out  : 1;  /**< SMBus SMBSUS# Output Value,
                                                   0:low level output, 1:high level output. */
        unsigned int smb_suspend_n_in   : 1;  /**< SMBus SMBSUS# Input Value,
                                                   0:low level output, 1:high level output. */
        unsigned int reserved4          : 13;
    } BIT;
} volatile I2C_CTRL2_REG;

/**
  * @brief I2C FIFO status registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int rx_fifo_vld_num : 5;  /**< Number of valid values in the RX FIFO. */
        unsigned int reserved0       : 3;
        unsigned int tx_fifo_vld_num : 5;  /**< Number of valid values in the TX FIFO. */
        unsigned int reserved1       : 3;
        unsigned int reserved2       : 16;
    } BIT;
} volatile I2C_FIFO_STAT_REG;

/**
  * @brief I2C state machine status registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0      : 10;
        unsigned int mst_cmd_exe    : 4;  /**< Sequence command being executed by the master. */
        unsigned int mst_busy       : 1;  /**< Master Busy, 0:master idle, 1:master busy. */
        unsigned int reserved1      : 6;
        unsigned int slv_cmd_exe    : 4;  /**< Sequence command being executed by the slave node. */
        unsigned int slv_busy       : 1;  /**< Slave Busy, 0:slave idle, 1:slave busy. */
        unsigned int reserved3      : 4;
        unsigned int i2c_bus_free   : 1;  /**< I2C bus idele, 0:i2c bus busy, 1:i2c bus idle. */
        unsigned int reserved2      : 1;
    } BIT;
} volatile I2C_FSM_STAT_REG;

/**
  * @brief I2C raw interrupt registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int mst_rx_ack_unmatch_raw : 1;  /**< The RX acknowledgment bit meets the expectation,
                                                       0:match, 1:unmatch. */
        unsigned int rx_fifo_not_empty_raw  : 1;  /**< The RX FIFO is not empty, 0:no interrupt, 1:have interrupt. */
        unsigned int rx_ge_watermark_raw    : 1;  /**< The number of data records in the RX FIFO is greater than or
                                                       equal to the threshold, 0:no interrupt, 1:have interrupt. */
        unsigned int rx_fifo_full_raw       : 1;  /**< RX FIFO Full, 0:no interrupt, 1:have interrupt. */
        unsigned int tx_le_watermark_raw    : 1;  /**< The number of data records in the TX FIFO is less than or
                                                       equal to the threshold, 0:no interrupt, 1:have interrupt. */
        unsigned int tx_fifo_empty_raw      : 1;  /**< The TX FIFO is empty, 0:no interrupt, 1:have interrupt. */
        unsigned int tx_fifo_not_full_raw   : 1;  /**< The TX FIFO is not full, 0:no interrupt, 1:have interrupt. */
        unsigned int rx_data_ready_raw      : 1;  /**< The RX FIFO receives new data,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int tx_data_request_raw    : 1;  /**< The TX FIFO requests new commands and data,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int stop_det_raw           : 1;  /**< STOP detected, 0:no interrupt, 1:have interrupt. */
        unsigned int start_det_raw          : 1;  /**< Checked to START, 0:no interrupt, 1:have interrupt. */
        unsigned int arb_lost_raw           : 1;  /**< Arbitration loss, 0:no interrupt, 1:have interrupt. */
        unsigned int mst_cmd_done_raw       : 1;  /**< The master timing command is completed normally,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int scl_low_timeout_raw    : 1;  /**< SCL Low Timeout Detected, 0:no interrupt, 1:have interrupt. */
        unsigned int smb_alert_raw          : 1;  /**< Falling edge of SMBus SMBALERT# input signal detected,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int smb_suspend_raw        : 1;  /**< Falling edge of SMBus SMBALERT# input signal detected,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int mst_cmd_int1_raw       : 1;  /**< Master timing command interrupt 1,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int slv_cmd_int1_raw       : 1;  /**< Slave timing command interrupt 1,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int mst_pec_check_fail_raw : 1;  /**< The master checks the received PEC error,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int slv_pec_check_fail_raw : 1;  /**< The slave node checks the received PEC error,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int slv_rx_ack_unmatch_raw : 1;  /**< Check whether the Slave RX acknowledgment bit meets the
                                                       expectation of the timing command,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int slv_addr_match_raw     : 1;  /**< The slave detects the received address matches,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int reserved0              : 10;
    } BIT;
} volatile I2C_INTR_RAW_REG;

/**
  * @brief I2C interrupt enable registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int mst_rx_ack_unmatch_en : 1;  /**< The RX acknowledgment bit meets the expectation,
                                                       0:disable, 1:enable. */
        unsigned int rx_fifo_not_empty_en  : 1;  /**< The RX FIFO is not empty, 0:disable, 1:enable. */
        unsigned int rx_ge_watermark_en    : 1;  /**< The rx_ge_watermark enable, 0:disable, 1:enable. */
        unsigned int rx_fifo_full_en       : 1;  /**< RX FIFO Full enable, 0:disable, 1:enable. */
        unsigned int tx_le_watermark_en    : 1;  /**< The tx_le_watermark, 0:disable, 1:enable. */
        unsigned int tx_fifo_empty_en      : 1;  /**< The TX FIFO is empty, 0:disable, 1:enable. */
        unsigned int tx_fifo_not_full_en   : 1;  /**< The TX FIFO is not full, 0:disable, 1:enable. */
        unsigned int rx_data_ready_en      : 1;  /**< The RX FIFO receives new data, 0:disable, 1:enable. */
        unsigned int tx_data_request_en    : 1;  /**< The TX FIFO requests new commands and data,
                                                      0:disable, 1:enable. */
        unsigned int stop_det_en           : 1;  /**< STOP detected enable, 0:disable, 1:enable. */
        unsigned int start_det_en          : 1;  /**< Checked to START enable, 0:disable, 1:enable. */
        unsigned int arb_lost_en           : 1;  /**< Arbitration loss enable, 0:disable, 1:enable. */
        unsigned int mst_cmd_done_en       : 1;  /**< The master timing command is completed normally,
                                                      0:disable, 1:enable. */
        unsigned int scl_low_timeout_en    : 1;  /**< SCL Low Timeout Detected enable, 0:disable, 1:enable. */
        unsigned int smb_alert_en          : 1;  /**< Falling edge of SMBus SMBALERT# input signal detected,
                                                      0:disable, 1:enable. */
        unsigned int smb_suspend_en        : 1;  /**< Falling edge of SMBus SMBALERT# input signal detected,
                                                      0:disable, 1:enable. */
        unsigned int mst_cmd_int1_en       : 1;  /**< Master timing command interrupt 1 enable, 0:disable, 1:enable. */
        unsigned int slv_cmd_int1_en       : 1;  /**< Slave timing command interrupt 1 enable, 0:disable, 1:enable. */
        unsigned int mst_pec_check_fail_en : 1;  /**< The master checks the received PEC error, 0:disable, 1:enable. */
        unsigned int slv_pec_check_fail_en : 1;  /**< The slave node checks the received PEC error,
                                                      0:disable, 1:enable. */
        unsigned int slv_rx_ack_unmatch_en : 1;  /**< Check whether the Slave RX acknowledgment bit meets the
                                                      expectation of the timing command, 0:disable, 1:enable. */
        unsigned int slv_addr_match_en     : 1;  /**< The slave detects the received address matches,
                                                      0:disable, 1:enable. */
        unsigned int reserved0             : 10;
    } BIT;
} volatile I2C_INTR_EN_REG;

/**
  * @brief I2C interrupt status registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int mst_rx_ack_unmatch_int : 1;  /**< The RX acknowledgment bit meets the expectation,
                                                       0:match, 1:unmatch. */
        unsigned int rx_fifo_not_empty_int  : 1;  /**< The RX FIFO is not empty, 0:no interrupt, 1:have interrupt. */
        unsigned int rx_ge_watermark_int    : 1;  /**< The number of data records in the RX FIFO is greater than or
                                                       equal to the threshold, 0:no interrupt, 1:have interrupt. */
        unsigned int rx_fifo_full_int       : 1;  /**< RX FIFO Full, 0:no interrupt, 1:have interrupt. */
        unsigned int tx_le_watermark_int    : 1;  /**< The number of data records in the TX FIFO is less than or
                                                       equal to the threshold, 0:no interrupt, 1:have interrupt. */
        unsigned int tx_fifo_empty_int      : 1;  /**< The TX FIFO is empty, 0:no interrupt, 1:have interrupt. */
        unsigned int tx_fifo_not_full_int   : 1;  /**< The TX FIFO is not full, 0:no interrupt, 1:have interrupt. */
        unsigned int rx_data_ready_int      : 1;  /**< The RX FIFO receives new data,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int tx_data_request_int    : 1;  /**< The TX FIFO requests new commands and data,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int stop_det_int           : 1;  /**< STOP detected, 0:no interrupt, 1:have interrupt. */
        unsigned int start_det_int          : 1;  /**< Checked to START, 0:no interrupt, 1:have interrupt. */
        unsigned int arb_lost_int           : 1;  /**< Arbitration loss, 0:no interrupt, 1:have interrupt. */
        unsigned int mst_cmd_done_int       : 1;  /**< The master timing command is completed normally,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int scl_low_timeout_int    : 1;  /**< SCL Low Timeout Detected, 0:no interrupt, 1:have interrupt. */
        unsigned int smb_alert_int          : 1;  /**< Falling edge of SMBus SMBALERT# input signal detected,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int smb_suspend_int        : 1;  /**< Falling edge of SMBus SMBALERT# input signal detected,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int mst_cmd_int1           : 1;  /**< Master timing command interrupt 1,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int slv_cmd_int1           : 1;  /**< Slave timing command interrupt 1,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int mst_pec_check_fail_int : 1;  /**< The master checks the received PEC error,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int slv_pec_check_fail_int : 1;  /**< The slave node checks the received PEC error,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int slv_rx_ack_unmatch_int : 1;  /**< Check whether the Slave RX acknowledgment bit meets the
                                                       expectation of the timing command,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int slv_addr_match_int     : 1;  /**< The slave detects the received address matches,
                                                       0:no interrupt, 1:have interrupt. */
        unsigned int reserved0              : 10;
    } BIT;
} volatile I2C_INTR_STAT_REG;

/**
  * @brief I2C version number registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int version : 32;  /**< I2C controller version. */
    } BIT;
} volatile I2C_VERSION_REG;

/**
  * @brief I2C SCL timeout threshold registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int scl_low_timeout : 23;  /**< SCL low-level timeout configuration. */
        unsigned int reserved0 : 9;
    } BIT;
} volatile I2C_SCL_TIMEOUT_REG;

/**
  * @brief I2C bus idle threshold registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int bus_free_time : 16;  /**< Bus Idle Threshold. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile I2C_BUS_FREE_REG;

/**
  * @brief I2C SDA and SCL filtering configuration registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int spike_filter_time : 4;  /**< SDA and SCL Glitch Filtering Configuration. */
        unsigned int reserved0 : 28;
    } BIT;
} volatile I2C_FILTER_REG;

/**
  * @brief Define the I2C register structure
  */
typedef struct {
    I2C_MODE_REG           I2C_MODE;           /**< I2C mode configuration register, Offset address: 0x0000U. */
    I2C_SCL_CFG_REG        I2C_SCL_CFG;        /**< I2C SCL high/low level time register, Offset address: 0x0004U. */
    I2C_SDA_CFG_REG        I2C_SDA_CFG;        /**< I2C SDA timing configuration register, Offset address: 0x0008U. */
    I2C_OWN_ADDR_REG       I2C_OWN_ADDR;       /**< I2C slave address configuration register,
                                                    Offset address: 0x000CU. */
    XMB_DEV_ADDR_REG       XMB_DEV_ADDR;       /**< I2C SMBus PMBus Device Dedicated Address Configuration register,
                                                    Offset address: 0x0010U. */
    unsigned char          space0[4];
    I2C_RX_ADDR_REG        I2C_RX_ADDR;        /**< Address received by the I2C slave, R/W bit register,
                                                    Offset address: 0x0018U. */
    unsigned char          space1[4];
    I2C_TX_FIFO_REG        I2C_TX_FIFO;        /**< I2C TX FIFO register, Offset address: 0x0020U. */
    I2C_RX_FIFO_REG        I2C_RX_FIFO;        /**< I2C_RX_FIFO register, Offset address: 0x0024U. */
    unsigned char          space2[160];
    I2C_TX_WATERMARK_REG   I2C_TX_WATERMARK;   /**< I2C TX threshold register, Offset address: 0x00C8U. */
    I2C_RX_WATERMARK_REG   I2C_RX_WATERMARK;   /**< I2C RX threshold register, Offset address: 0x00CCU. */
    I2C_CTRL1_REG          I2C_CTRL1;          /**< I2C control register 1, Offset address: 0x00D0U. */
    I2C_CTRL2_REG          I2C_CTRL2;          /**< I2C control register 2, Offset address: 0x00D4U. */
    I2C_FIFO_STAT_REG      I2C_FIFO_STAT;      /**< I2C FIFO status register, Offset address: 0x00D8U. */
    I2C_FSM_STAT_REG       I2C_FSM_STAT;       /**< I2C state machine status register, Offset address: 0x00DCU. */
    I2C_INTR_RAW_REG       I2C_INTR_RAW;       /**< I2C raw interrupt register, Offset address: 0x00E0U. */
    I2C_INTR_EN_REG        I2C_INTR_EN;        /**< I2C interrupt enable register, Offset address: 0x00E4U. */
    I2C_INTR_STAT_REG      I2C_INTR_STAT;      /**< I2C interrupt status register, Offset address: 0x00E8U. */
    unsigned char          space3[20];
    I2C_VERSION_REG        I2C_VERSION;        /**< I2C version number register, Offset address: 0x0100U. */
    I2C_SCL_TIMEOUT_REG    I2C_SCL_TIMEOUT;    /**< I2C SCL timeout threshold register, Offset address: 0x0104U. */
    I2C_BUS_FREE_REG       I2C_BUS_FREE;       /**< I2C bus idle threshold register, Offset address: 0x0108U. */
    I2C_FILTER_REG         I2C_FILTER;         /**< I2C SDA and SCL filtering configuration register,
                                                    Offset address: 0x010CU. */
} volatile I2C_RegStruct;
/**
  * @}
  */

/* Parameter check definition-------------------------------------------*/
/**
  * @brief Check I2C function mode selection.
  * @param functionMode I2C function mode type.
  * @retval true
  * @retval false
  */
static inline bool IsI2cFunctionMode(I2C_ModeSelectType functionMode)
{
    return (functionMode == I2C_MODE_SELECT_NONE ||
            functionMode == I2C_MODE_SELECT_MASTER_ONLY ||
            functionMode == I2C_MODE_SELECT_SLAVE_ONLY ||
            functionMode == I2C_MODE_SELECT_MASTER_SLAVE);
}

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
  * @brief Check I2C general call mode.
  * @param generalCallMode I2C general call mode.
  * @retval true
  * @retval false
  */
static inline bool IsI2cGeneralCallMode(unsigned int generalCallMode)
{
    return (generalCallMode == BASE_CFG_ENABLE ||
            generalCallMode == BASE_CFG_DISABLE);
}

/**
  * @brief Check I2C lit end mode.
  * @param litEnd I2C lit end mode.
  * @retval true
  * @retval false
  */
static inline bool IsI2cLitEndMode(unsigned int litEnd)
{
    return (litEnd == BASE_CFG_ENABLE ||
            litEnd == BASE_CFG_DISABLE);
}

/**
  * @brief Check I2C scl stretch Disable mode.
  * @param sclStretchDisable I2C scl stretch Disable mode.
  * @retval true
  * @retval false
  */
static inline bool IsI2cSclStretchDisableMode(unsigned int sclStretchDisable)
{
    return (sclStretchDisable == BASE_CFG_ENABLE ||
            sclStretchDisable == BASE_CFG_DISABLE);
}

/**
  * @brief Check i2c own address.
  * @param ownAddress I2C own address.
  * @retval true
  * @retval false
  */
static inline bool IsI2cOwnAddressOrMask(unsigned int ownAddress)
{
    return (ownAddress <= XMBUS_OWN_ADDRESS_MASK); /* Own address value is 0 to 0x3FF */
}

/**
  * @brief Check XMBus address.
  * @param xmbusAddress XMBus address.
  * @retval true
  * @retval false
  */
static inline bool IsXMBusAddressOrMask(unsigned int xmbusAddress)
{
    return (xmbusAddress <= XMBUS_OWN_ADDRESS_MASK); /* XMBus address value is 0 to 0x3FF */
}

/**
  * @brief Check XMBus address Enable.
  * @param xmbusAddress XMBus address.
  * @retval true
  * @retval false
  */
static inline bool IsXMBusAddressEnable(unsigned int slaveOwnXmbAddressEnable)
{
    return (slaveOwnXmbAddressEnable == BASE_CFG_ENABLE || slaveOwnXmbAddressEnable == BASE_CFG_DISABLE);
}

/**
  * @brief Check i2c SDA and SCL Glitch Filtering Time Configuration.
  * @param spikeFilterTime I2C SDA and SCL Glitch Filtering Time.
  * @retval true
  * @retval false
  */
static inline bool IsI2cSpikeFilterTime(unsigned int spikeFilterTime)
{
    return (spikeFilterTime <= 0xF); /* The spikeFilterTime value is 0 to 0xF */
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
    return (txWaterMark <= 0xF); /* The txWaterMark value is 0 to 0xF */
}

/**
  * @brief Check i2c rx water mark.
  * @param rxWaterMark I2C rx water mark.
  * @retval true
  * @retval false
  */
static inline bool IsI2cRxWaterMark(unsigned int rxWaterMark)
{
    return (rxWaterMark <= 0xF); /* The rxWaterMark value is 0 to 0xF */
}

/**
  * @brief Check i2c DMA Operation type.
  * @param mode I2C DMA Operation type.
  * @retval true
  * @retval false
  */
static inline bool IsI2CDmaOperationType(I2C_DmaOperationType mode)
{
    return (mode == I2C_DMA_OP_NONE ||
            mode == I2C_DMA_OP_WRITE ||
            mode == I2C_DMA_OP_READ ||
            mode == I2C_DMA_OP_WRITE_READ);
}

/**
  * @brief Check i2c set one bit value.
  * @param value value is set.
  * @retval true
  * @retval false
  */
static inline bool IsI2cSetOneBitValue(unsigned int value)
{
    return (value == BASE_CFG_UNSET || value == BASE_CFG_SET);
}

/**
  * @brief Check i2c scl timeout value.
  * @param time the value of scl time out.
  * @retval true
  * @retval false
  */
static inline bool IsI2cSclTimoutValue(unsigned int time)
{
    return (time <= 0x7FFFFF); /* The i2c scl timeout max value is 0x7FFFFF. */
}

/**
  * @brief Check i2c bus free time value.
  * @param time the value of bus free time.
  * @retval true
  * @retval false
  */
static inline bool IsI2cBusFreeTimeValue(unsigned int time)
{
    return (time <= 0xFFFF); /* The i2c bus free time max value is 0xFFFF. */
}

/**
  * @brief Check i2c data transfer sequence value.
  * @param arg data transfer sequence value.
  * @retval true
  * @retval false
  */
static inline bool IsI2cDataTransferSequence(I2C_DataTransferSequenceType arg)
{
    return (arg == I2C_BIG_BIT_FIRST || arg == I2C_LITTLE_BIT_FIRST);
}

/**
  * @brief Check i2c clock stretching enumeration value.
  * @param arg clock stretching enumeration value.
  * @retval true
  * @retval false
  */
static inline bool IsI2cClockStretchValue(I2C_ClockStretchType arg)
{
    return (arg == I2C_CLOCK_STRETCH_ENABLE || arg == I2C_CLOCK_STRETCH_DISABLE);
}

/**
  * @brief Check i2c SCL low-level timeout value.
  * @param sclLowTimeout SCL low-level timeout value.
  * @retval true
  * @retval false
  */
static inline bool IsI2cSclLowTimeout(unsigned int sclLowTimeout)
{
    return (sclLowTimeout <= 0x7FFFFF); /* The SCL low-level timeout upper limit is 0x7FFFFF. */
}

/**
  * @brief Check i2c SDA delay time.
  * @param sdaDelayTime The value of SDA delay time.
  * @retval true
  * @retval false
  */
static inline bool IsI2cSdaDelayTime(unsigned int sdaDelayTime)
{
    return (sdaDelayTime <= 0x0F); /* The SDA delay time upper limit is 0x0F. */
}

/**
  * @brief Check i2c bus idle threshold value.
  * @param busFreeTime bus idle threshold value.
  * @retval true
  * @retval false
  */
static inline bool IsI2cBusFreeTime(unsigned int busFreeTime)
{
    return (busFreeTime <= 0xFFFF); /* The SCL bus idle threshold is 0xFFFF. */
}

/**
  * @brief DCL I2C mode function set.
  * @param i2cx I2C register base address.
  * @param function I2C mode function
  * @retval None.
  */
static inline void DCL_I2C_SetFunction(I2C_RegStruct *i2cx, unsigned int function)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    I2C_ASSERT_PARAM(IsI2cFunctionMode(function));
    i2cx->I2C_MODE.BIT.mst_slv_function = function;
}

/**
  * @brief DCL I2C mode function get.
  * @param i2cx I2C register base address.
  * @retval I2C mode function.
  */
static inline unsigned int DCL_I2C_GetFunction(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_MODE.BIT.mst_slv_function;
}

/**
  * @brief DCL I2C endian set.
  * @param i2cx I2C register base address.
  * @param endian data transfer sequence enumeration value.
  * @retval None.
  */
static inline void DCL_I2C_SetEndian(I2C_RegStruct *i2cx, I2C_DataTransferSequenceType endian)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    I2C_PARAM_CHECK_NO_RET(IsI2cDataTransferSequence(endian));
    i2cx->I2C_MODE.BIT.lit_end = endian;
}

/**
  * @brief DCL I2C endian get.
  * @param i2cx I2C register base address.
  * @retval I2C endian.
  */
static inline unsigned int DCL_I2C_GetEndian(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_MODE.BIT.lit_end;
}

/**
  * @brief DCL I2C enable PEC.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_PECEnable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_ENABLE;
}

/**
  * @brief DCL I2C disable PEC.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_PECDisable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_DISABLE;
}

/**
  * @brief DCL I2C PEC status get.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline unsigned int DCL_I2C_GetPECStatus(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_MODE.BIT.xmb_pec_en;
}

/**
  * @brief DCL I2C enable ignore rack mode.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_IgnoreRackModeEnable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_MODE.BIT.rack_mode = I2C_IGNORE_NAK_ENABLE;
}

/**
  * @brief DCL I2C disable ignore rack mode.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_IgnoreRackModeDisable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_MODE.BIT.rack_mode = I2C_IGNORE_NAK_DISABLE;
}

/**
  * @brief DCL I2C enable scl stretch function.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_SclStrechEnable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_MODE.BIT.scl_stretch_disable = BASE_CFG_DISABLE;
}

/**
  * @brief DCL I2C disable scl stretch function.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_SclStrechDisable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_MODE.BIT.scl_stretch_disable = BASE_CFG_ENABLE;
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
    glbReg = (i2cx->I2C_SDA_CFG.reg & 0xF) | temp;
    i2cx->I2C_SDA_CFG.reg = glbReg;
}

/**
  * @brief Get DCL Configuring i2c SDA Hold Time.
  * @param i2cx I2C register base address.
  * @retval Sda hold time, 0-65535.
  */
static inline int DCL_I2C_GetSdaHoldDuration(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return ((i2cx->I2C_SDA_CFG.reg >> I2C_SDA_HOLD_DURATION_POS) & 0xFFFF); /* The mask of sda hold time is 0xFFFF. */
}

/**
  * @brief DCL Configuring i2c SDA delay Time.
  * @param i2cx I2C register base address.
  * @param delay Sda delay time.
  * @retval None.
  */
static inline void DCL_I2C_SetSdaDelayTime(I2C_RegStruct *i2cx, unsigned delay)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    I2C_PARAM_CHECK_NO_RET(IsI2cSdaDelayTime(delay));
    i2cx->I2C_SDA_CFG.BIT.sda_delay_time = delay;
}

/**
  * @brief DCL Get i2c SDA delay Time.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline unsigned int DCL_I2C_GetSdaDelayTime(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_SDA_CFG.BIT.sda_delay_time;
}

/**
  * @brief DCL Configuring i2c SCL High Hold Time.
  * @param i2cx I2C register base address.
  * @param sclHighTime Scl high hold time.
  * @retval None.
  */
static inline void DCL_I2C_SetHighDuration(I2C_RegStruct *i2cx, unsigned short sclHighTime)
{
    unsigned int tempReg;
    unsigned int temp;

    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    tempReg = i2cx->I2C_SCL_CFG.reg;
    /* Read the entire register and write it back. */
    temp = ((unsigned int)sclHighTime) << I2C_SCL_HIGH_TIME_POS;
    tempReg = (i2cx->I2C_SCL_CFG.reg & I2C_SCL_LOW_TIME_MASK) | temp;
    i2cx->I2C_SCL_CFG.reg = tempReg;
}

/**
  * @brief DCL get i2c SCL High Hold Time.
  * @param i2cx I2C register base address.
  * @retval Scl high hold time,0-65535.
  */
static inline int DCL_I2C_GetHighDuration(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return ((i2cx->I2C_SCL_CFG.reg >> I2C_SCL_HIGH_TIME_POS) & 0xFFFF); /* The mask of scl high hold time is 0xFFFF. */
}

/**
  * @brief DCL Configuring i2c SCL low Hold Time.
  * @param i2cx I2C register base address.
  * @param sclLowTime The value of I2C SCL low time.
  * @retval None.
  */
static inline void DCL_I2C_SetLowDuration(I2C_RegStruct *i2cx, unsigned short sclLowTime)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_SCL_CFG.BIT.scl_low_time = sclLowTime;
}

/**
  * @brief DCL Get i2c SCL low Hold Time.
  * @param i2cx I2C register base address.
  * @retval Scl low hold time,0-65535.
  */
static inline int DCL_I2C_GetLowDuration(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_SCL_CFG.BIT.scl_low_time;
}

/**
  * @brief DCL Set I2C owner Address.
  * @param i2cx I2C register base address.
  * @param ownAddr Slave address
  * @retval None.
  */
static inline void DCL_I2C_SetOwnAddr(I2C_RegStruct *i2cx, unsigned int ownAddr)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    I2C_ASSERT_PARAM(IsI2cOwnAddressOrMask(ownAddr));
    i2cx->I2C_OWN_ADDR.BIT.own_address = ownAddr;
}

/**
  * @brief DCL Get I2C owner Address.
  * @param i2cx I2C register base address.
  * @retval I2C owner address.
  */
static inline unsigned int DCL_I2C_GetOwnAddr(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_OWN_ADDR.BIT.own_address;
}

/**
  * @brief DCL Set I2C owner address mask.
  * @param i2cx I2C register base address.
  * @param ownAddrMask The maske of I2C slave address.
  * @retval None.
  */
static inline void DCL_I2C_SetOwnMaskAddr(I2C_RegStruct *i2cx, unsigned int ownAddrMask)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    I2C_ASSERT_PARAM(IsI2cOwnAddressOrMask(ownAddrMask));
    i2cx->I2C_OWN_ADDR.BIT.own_address_mask = ownAddrMask;
}

/**
  * @brief DCL Set I2C owner address mask.
  * @param i2cx I2C register base address.
  * @retval I2C owner address mask.
  */
static inline unsigned int DCL_I2C_GetOwnMaskAddr(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_OWN_ADDR.BIT.own_address_mask;
}

/**
  * @brief DCL Set I2C 10bit slave enable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_10BitSlaveEnable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_OWN_ADDR.BIT.i2c_10bit_slave_en = BASE_CFG_ENABLE;
}

/**
  * @brief DCL Set I2C 10bit slave disable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_10BitSlaveDisable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_OWN_ADDR.BIT.i2c_10bit_slave_en = BASE_CFG_DISABLE;
}

/**
  * @brief DCL Set XMBus Address.
  * @param i2cx I2C register base address.
  * @param xmbusAddr The address is used for I2C, SMBus and PMBus Device.
  * @retval None.
  */
static inline void DCL_I2C_SetXMBusAddr(I2C_RegStruct *i2cx, unsigned int xmbusAddr)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    I2C_ASSERT_PARAM(IsXMBusAddressOrMask(xmbusAddr));
    i2cx->XMB_DEV_ADDR.BIT.xmb_address = xmbusAddr;
}

/**
  * @brief DCL Get XMBus Address.
  * @param i2cx I2C register base address.
  * @retval The address of I2C, SMBus and PMBus Device.
  */
static inline unsigned int DCL_I2C_GetXMBusAddr(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->XMB_DEV_ADDR.BIT.xmb_address;
}

/**
  * @brief DCL Set xmbus address mask.
  * @param i2cx I2C register base address.
  * @param xmbusAddrMask The maske of xmbus device address.
  * @retval None.
  */
static inline void DCL_I2C_SetXMBusMaskAddr(I2C_RegStruct *i2cx, unsigned int xmbusAddrMask)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    I2C_ASSERT_PARAM(IsXMBusAddressOrMask(xmbusAddrMask));
    i2cx->XMB_DEV_ADDR.BIT.xmb_address_mask = xmbusAddrMask;
}

/**
  * @brief DCL Get XMBus device address mask.
  * @param i2cx I2C register base address.
  * @retval XMBus device address mask.
  */
static inline unsigned int DCL_I2C_GetXMBusMaskAddr(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->XMB_DEV_ADDR.BIT.xmb_address_mask;
}

/**
  * @brief DCL Set XMBus address enable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_XMBusAddressEnable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->XMB_DEV_ADDR.BIT.xmb_address_en = BASE_CFG_ENABLE;
}

/**
  * @brief DCL Set XMBus address disable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_XMBusAddressDisable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->XMB_DEV_ADDR.BIT.xmb_address_en = BASE_CFG_DISABLE;
}

/**
  * @brief DCL Set SMBus host notify enable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_SMBusHostNotifyEnable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->XMB_DEV_ADDR.BIT.smb_host_notify_en = BASE_CFG_ENABLE;
}

/**
  * @brief DCL Set SMBus host notify disable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_SMBusHostNotifyDisable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->XMB_DEV_ADDR.BIT.smb_host_notify_en = BASE_CFG_DISABLE;
}

/**
  * @brief DCL Set SMBus alert response enable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_SMBusAlertResponseEnable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->XMB_DEV_ADDR.BIT.smb_alert_response_en = BASE_CFG_ENABLE;
}

/**
  * @brief DCL Set SMBus SMBus alert response disable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_SMBusAlertResponseDisable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->XMB_DEV_ADDR.BIT.smb_alert_response_en = BASE_CFG_DISABLE;
}

/**
  * @brief DCL Set Receive SMBus device default address enable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_SMBusDevDefaultEnable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->XMB_DEV_ADDR.BIT.smb_dev_default_en = BASE_CFG_ENABLE;
}

/**
  * @brief DCL Set Receive SMBus device default address disable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_SMBusDevDefaultDisable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->XMB_DEV_ADDR.BIT.smb_dev_default_en = BASE_CFG_DISABLE;
}

/**
  * @brief DCL Set Receive PMBus Zone Read Address Enable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_PMBusZoneReadEnable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->XMB_DEV_ADDR.BIT.pmb_zone_read_en = BASE_CFG_ENABLE;
}

/**
  * @brief DCL Set Receive PMBus Zone Read Address Disable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_PMBusZoneReadDisable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->XMB_DEV_ADDR.BIT.pmb_zone_read_en = BASE_CFG_DISABLE;
}

/**
  * @brief DCL Set Receive PMBus Zone Write Address Enable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_PMBusZoneWriteEnable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->XMB_DEV_ADDR.BIT.pmb_zone_write_en = BASE_CFG_ENABLE;
}

/**
  * @brief DCL Set Receive PMBus Zone Write Address Disable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_PMBusZoneWriteDisable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->XMB_DEV_ADDR.BIT.pmb_zone_write_en = BASE_CFG_DISABLE;
}

/**
  * @brief DCL Set I2C start byte enable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_StartByteEnable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_OWN_ADDR.BIT.i2c_start_byte_en = BASE_CFG_ENABLE;
}

/**
  * @brief DCL Set I2C start byte disable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_StartByteDisable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_OWN_ADDR.BIT.i2c_start_byte_en = BASE_CFG_DISABLE;
}

/**
  * @brief DCL Set I2C device id enable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_DeviceIDEnable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_OWN_ADDR.BIT.i2c_device_id_en = BASE_CFG_ENABLE;
}

/**
  * @brief DCL Set I2C device id disable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_DeviceIDDisable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_OWN_ADDR.BIT.i2c_device_id_en = BASE_CFG_DISABLE;
}

/**
  * @brief DCL Set I2C general call enable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_GeneralCallEnable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_OWN_ADDR.BIT.i2c_general_call_en = BASE_CFG_ENABLE;
}

/**
  * @brief DCL Set I2C general call disable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_GeneralCallDisable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_OWN_ADDR.BIT.i2c_general_call_en = BASE_CFG_DISABLE;
}

/**
  * @brief Get the I2C RX received R/W Bits.
  * @param i2cx I2C register base address.
  * @retval The value of I2C RX received R/W Bits, 0: write, 1: read.
  */
static inline unsigned int DCL_I2C_GetRxReadOrWrite(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_RX_ADDR.BIT.rx_rw;
}

/**
  * @brief Get the I2C RX address.
  * @param i2cx I2C register base address.
  * @retval RX address.
  */
static inline unsigned int DCL_I2C_GetRxAddr(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_RX_ADDR.reg & 0x3FF; /* The mask of RX address is 0x3FF. */
}

/**
  * @brief Set the I2C TX FIFO.
  * @param i2cx I2C register base address.
  * @param cmd I2C operation commands.
  * @param data I2C operation data
  * @retval None.
  */
static inline void DCL_I2C_SetTxFIFO(I2C_RegStruct *i2cx, I2C_CmdType cmd, unsigned char data)
{
    unsigned int temp;

    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    temp = (((unsigned int)cmd << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
    temp |= (((unsigned int)data << I2C_TXFIFO_WDATA_POS) & I2C_TXFIFO_WDATA_MASK);
    i2cx->I2C_TX_FIFO.reg = temp; /* Set xommand and data */
}

/**
  * @brief Get the I2C RX FIFO.
  * @param i2cx I2C register base address.
  * @retval RX FIFO data.
  */
static inline unsigned int DCL_I2C_GetRxFIFO(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_RX_FIFO.reg;
}

/**
  * @brief Set the I2C TX threshold.
  * @param i2cx I2C register base address.
  * @param waterMark I2C Tx threshold, 0-15.
  * @retval None.
  */
static inline void DCL_I2C_SetTxWaterMark(I2C_RegStruct *i2cx, unsigned char waterMark)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    I2C_PARAM_CHECK_NO_RET(IsI2cTxWaterMark(waterMark));
    i2cx->I2C_TX_WATERMARK.BIT.tx_watermark = waterMark;
}

/**
  * @brief Get the I2C TX threshold.
  * @param i2cx I2C register base address.
  * @retval I2C tx threshold.
  */
static inline unsigned int DCL_I2C_GetTxWaterMark(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_TX_WATERMARK.BIT.tx_watermark;
}

/**
  * @brief Set the I2C RX threshold.
  * @param i2cx I2C register base address.
  * @param waterMark I2C Rx threshold, 0-15.
  * @retval None.
  */
static inline void DCL_I2C_SetRxWaterMark(I2C_RegStruct *i2cx, unsigned char waterMark)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    I2C_PARAM_CHECK_NO_RET(IsI2cRxWaterMark(waterMark));
    i2cx->I2C_RX_WATERMARK.BIT.rx_watermark = waterMark;
}

/**
  * @brief Get the I2C RX threshold.
  * @param i2cx I2C register base address.
  * @retval I2C rx threshold.
  */
static inline int DCL_I2C_GetRxWaterMark(const I2C_RegStruct *i2cx)
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
static inline void DCL_I2C_SetDmaMode(I2C_RegStruct *i2cx, I2C_DmaOperationType mode)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    I2C_ASSERT_PARAM(IsI2CDmaOperationType(mode));
    i2cx->I2C_CTRL1.BIT.dma_operation = mode;
}

/**
  * @brief DCL Set Start I2C timing execution Enable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_SetMasterStartEnable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_CTRL1.BIT.mst_start = BASE_CFG_SET;
}

/**
  * @brief DCL Set Start I2C timing execution Disable.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_SetMasterStartDisable(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_CTRL1.BIT.mst_start = BASE_CFG_UNSET;
}

/**
  * @brief Get start and stop I2C timing status.
  * @param i2cx I2C register base address.
  * @retval start : 1, stop :0.
  */
static inline unsigned int DCL_I2C_GetStart(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_CTRL1.BIT.mst_start;
}

/**
  * @brief Rest Tx FIFO.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_ResetTxFIFO(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_CTRL1.BIT.rst_tx_fifo = BASE_CFG_SET;
}

/**
  * @brief Rest Rx FIFO.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_ResetRxFIFO(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_CTRL1.BIT.rst_rx_fifo = BASE_CFG_SET;
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
    I2C_ASSERT_PARAM(IsI2cSetOneBitValue(mode));
    i2cx->I2C_CTRL2.BIT.gpio_mode = mode;
}

/**
  * @brief Get the SCL and SDA pins of the I2C to GPIO mode.
  * @param i2cx I2C register base address.
  * @retval 0 or 1
  */
static inline unsigned int DCL_I2C_GetGpioMode(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_CTRL2.BIT.gpio_mode;
}

/**
  * @brief Set the SDA output level.
  * @param i2cx I2C register base address.
  * @param level The sda output level.
  * @retval 0 or 1.
  */
static inline void DCL_I2C_SetSdaLevel(I2C_RegStruct *i2cx, unsigned int level)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    I2C_ASSERT_PARAM(IsI2cSetOneBitValue(level));
    i2cx->I2C_CTRL2.BIT.force_sda = level;
}

/**
  * @brief Get the SDA output level.
  * @param i2cx I2C register base address.
  * @retval 0 or 1.
  */
static inline unsigned int DCL_I2C_GetSdaLevel(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_CTRL2.BIT.force_sda;
}

/**
  * @brief Set the SCL output level.
  * @param i2cx I2C register base address.
  * @param level The scl output level.
  * @retval None.
  */
static inline void DCL_I2C_SetSclLevel(I2C_RegStruct *i2cx, unsigned int level)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    I2C_ASSERT_PARAM(IsI2cSetOneBitValue(level));
    i2cx->I2C_CTRL2.BIT.force_scl = level;
}

/**
  * @brief Get the SCL output level.
  * @param i2cx I2C register base address.
  * @retval 0 or 1.
  */
static inline unsigned int DCL_I2C_GetSclLevel(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_CTRL2.BIT.force_scl;
}

/**
  * @brief Get SMBus suspend_n_in level.
  * @param i2cx I2C register base address.
  * @retval 0 or 1.
  */
static inline unsigned int DCL_I2C_GetSMBusSuspendIn(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_CTRL2.BIT.smb_suspend_n_in;
}

/**
  * @brief Set SMBus suspend_n_out low level.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_SetSMBusSuspendOutLowLevel(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_CTRL2.BIT.smb_suspend_n_out = BASE_CFG_DISABLE;
}

/**
  * @brief Set SMBus suspend_n_out high level.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_SetSMBusSuspendOutHighLevel(I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_CTRL2.BIT.smb_suspend_n_out = BASE_CFG_ENABLE;
}

/**
  * @brief Get SMBus suspend_n_out level.
  * @param i2cx I2C register base address.
  * @retval 0 or 1.
  */
static inline unsigned int DCL_I2C_GetSMBusSuspendOut(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_CTRL2.BIT.smb_suspend_n_out;
}

/**
  * @brief Set SMBus suspend_n_oe_n.
  * @param i2cx I2C register base address.
  * @param selcet The output value of SMBSUS#.
  * @retval None.
  */
static inline void DCL_I2C_SetSMBusSuspendOe(I2C_RegStruct *i2cx, unsigned int selcet)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    I2C_ASSERT_PARAM(IsI2cSetOneBitValue(selcet));
    i2cx->I2C_CTRL2.BIT.smb_suspend_n_oe_n = selcet;
}

/**
  * @brief Get SMBus suspend_n_oe_n.
  * @param i2cx I2C register base address.
  * @retval 0 or 1.
  */
static inline unsigned int DCL_I2C_GetSMBusSuspendOe(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_CTRL2.BIT.smb_suspend_n_oe_n;
}

/**
  * @brief Get SMBus alert_n_in.
  * @param i2cx I2C register base address.
  * @retval 0 or 1.
  */
static inline unsigned int DCL_I2C_GetSMBusAlertIn(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_CTRL2.BIT.smb_alert_n_in;
}

/**
  * @brief Set SMBus alert_n_oe_n.
  * @param i2cx I2C register base address.
  * @param selcet The smbus alert set or unset.
  * @retval None.
  */
static inline void DCL_I2C_SetSMBusAlertOe(I2C_RegStruct *i2cx, unsigned int selcet)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    I2C_ASSERT_PARAM(IsI2cSetOneBitValue(selcet));
    i2cx->I2C_CTRL2.BIT.smb_alert_n_oe_n = selcet;
}

/**
  * @brief Get SMBus alert_n_oe_n.
  * @param i2cx I2C register base address.
  * @retval The smbus alert value.
  */
static inline unsigned int DCL_I2C_GetSMBusAlertOe(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_CTRL2.BIT.smb_alert_n_oe_n;
}

/**
  * @brief Get the number of valid Tx FIFOs.
  * @param i2cx I2C register base address.
  * @retval Tx FIFO valid value.
  */
static inline unsigned int DCL_I2C_GetTxFIFOValidNum(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_FIFO_STAT.BIT.tx_fifo_vld_num;
}

/**
  * @brief Get the number of valid Rx FIFOs.
  * @param i2cx I2C register base address.
  * @retval Tx FIFO valid value.
  */
static inline unsigned int DCL_I2C_GetRxFIFOValidNum(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_FIFO_STAT.BIT.rx_fifo_vld_num;
}

/**
  * @brief Get the status of I2C bus.
  * @param i2cx I2C register base address.
  * @retval 0: bus busy, 1: bus free.
  */
static inline unsigned int DCL_I2C_GetI2cBusSatus(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_FSM_STAT.BIT.i2c_bus_free;
}

/**
  * @brief Clearing I2C raw interrupts.
  * @param i2cx I2C register base address.
  * @param cleanStatus Need to clean Raw status
  * @retval None.
  */
static inline void DCL_I2C_CleanRawINT(I2C_RegStruct *i2cx, unsigned int cleanStatus)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_INTR_RAW.reg = cleanStatus;
}

/**
  * @brief Get I2C raw interrupts status.
  * @param i2cx I2C register base address.
  * @retval The value of I2C raw interrupts status.
  */
static inline unsigned int DCL_I2C_GetRawINTStatus(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_INTR_RAW.reg;
}

/**
  * @brief Set enable I2C interrupts.
  * @param i2cx I2C register base address.
  * @param enableSelect Need to set interrupt enable.
  * @retval None.
  */
static inline void DCL_I2C_SetInterruptsEnable(I2C_RegStruct *i2cx, unsigned int enableSelect)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    i2cx->I2C_INTR_EN.reg = enableSelect;
}

/**
  * @brief Get enable I2C interrupts.
  * @param i2cx I2C register base address.
  * @retval I2C interrupt enable value.
  */
static inline unsigned int DCL_I2C_GetInterruptsEnable(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_INTR_EN.reg;
}

/**
  * @brief Get I2C Interrupts Status.
  * @param i2cx I2C register base address.
  * @retval I2C interrupt status.
  */
static inline unsigned int DCL_I2C_GetInterruptsStatus(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_INTR_STAT.reg;
}

/**
  * @brief Get I2C version ID.
  * @param i2cx I2C register base address.
  * @retval I2C version ID.
  */
static inline unsigned int DCL_I2C_GetVersionID(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_VERSION.reg;
}

/**
  * @brief Set I2C SCL timeout.
  * @param i2cx I2C register base address.
  * @param timeout SCL low-level timeout configuration.
  * @retval None.
  */
static inline void DCL_I2C_SetSclTimeout(I2C_RegStruct *i2cx, unsigned int timeout)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    I2C_ASSERT_PARAM(IsI2cSclTimoutValue(timeout));
    i2cx->I2C_SCL_TIMEOUT.reg = timeout;
}

/**
  * @brief Get I2C SCL timeout.
  * @param i2cx I2C register base address.
  * @retval I2C Scl timeout value.
  */
static inline unsigned int DCL_I2C_GetSclTimeout(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_SCL_TIMEOUT.BIT.scl_low_timeout;
}

/**
  * @brief I2C bus idle threshold configuration.
  * @param i2cx I2C register base address.
  * @param time The I2C bus idle threshold.
  * @retval None.
  */
static inline void DCL_I2C_SetIdleThreshold(I2C_RegStruct *i2cx, unsigned int time)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    I2C_ASSERT_PARAM(IsI2cBusFreeTimeValue(time));
    i2cx->I2C_BUS_FREE.BIT.bus_free_time = time;
}

/**
  * @brief Get I2C bus idle threshold.
  * @param i2cx I2C register base address.
  * @retval I2C bus idle threshold.
  */
static inline unsigned int DCL_I2C_GetIdleThreshold(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_BUS_FREE.BIT.bus_free_time;
}

/**
  * @brief I2C filtering configuration.
  * @param i2cx I2C register base address.
  * @retval None.
  */
static inline void DCL_I2C_SetFilter(I2C_RegStruct *i2cx, unsigned int time)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    I2C_PARAM_CHECK_NO_RET(time <= 0xF); /* The maximum spike filter time is 0xF;  */
    i2cx->I2C_FILTER.BIT.spike_filter_time = time;
}

/**
  * @brief Get I2C filtering configuration.
  * @param i2cx I2C register base address.
  * @retval I2C filtering configuration.
  */
static inline unsigned int DCL_I2C_GetFilter(const I2C_RegStruct *i2cx)
{
    I2C_ASSERT_PARAM(IsI2CInstance(i2cx));
    return i2cx->I2C_FILTER.BIT.spike_filter_time;
}

/**
  * @}
  */

/**
  * @}
  */
#endif /* #ifndef McuMagicTag_I2C_IP_H */