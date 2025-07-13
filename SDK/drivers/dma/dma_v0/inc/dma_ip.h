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
  * @file    dma_ip.h
  * @author  MCU Driver Team
  * @brief   DMA module driver
  * @details This file provides DCL functions to manage DMA and Definition of
  *          specific parameters.
  *           + Definition of DMA configuration parameters.
  *           + DMA register mapping structure.
  *           + Parameters check functions.
  *           + Direct configuration layer interface.
  */

#ifndef McuMagicTag_DMA_IP_H
#define McuMagicTag_DMA_IP_H

#include "baseinc.h"

#define CHANNEL_MAX_NUM 4
#define TRANSIZE_MAX 4095
#define TRANS_BLOCK 4092

#ifdef DMA_PARAM_CHECK
#define DMA_ASSERT_PARAM  BASE_FUNC_ASSERT_PARAM
#define DMA_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define DMA_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define DMA_ASSERT_PARAM(para) ((void)0U)
#define DMA_PARAM_CHECK_NO_RET(para) ((void)0U)
#define DMA_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif

/**
  * @addtogroup DMA
  * @{
  */

/**
  * @defgroup DMA_IP DMA_IP
  * @brief DMA_IP: dma_v0.
  * @{
  */

/**
  * @defgroup DMA_Param_Def DMA Parameters Definition
  * @brief Description of DMA configuration parameters.
  * @{
  */

/**
  * @brief Indicates the burst length of the destination device and the source device.
  */
typedef enum {
    DMA_BURST_LENGTH_1 = 0x00000000U,
    DMA_BURST_LENGTH_4 = 0x00000001U,
    DMA_BURST_LENGTH_8 = 0x00000002U,
    DMA_BURST_LENGTH_16 = 0x00000003U,
    DMA_BURST_LENGTH_32 = 0x00000004U,
    DMA_BURST_LENGTH_64 = 0x00000005U,
    DMA_BURST_LENGTH_128 = 0x00000006U,
    DMA_BURST_LENGTH_256 = 0x00000007U
} DMA_BurstLength;

/**
  * @brief DMA transfer width definition.
  */
typedef enum {
    DMA_TRANSWIDTH_BYTE = 0x00000000U,
    DMA_TRANSWIDTH_HALFWORD = 0x00000001U,
    DMA_TRANSWIDTH_WORD = 0x00000002U
} DMA_TransmisWidth;

/**
  * @brief DMA channel ID, a smaller channel ID indicates a higher priority.
  */
typedef enum {
    DMA_CHANNEL_ZERO = 0x00000000U,
    DMA_CHANNEL_ONE = 0x00000001U,
    DMA_CHANNEL_TWO = 0x00000002U,
    DMA_CHANNEL_THREE = 0x00000003U
} DMA_ChannelNum;

/**
  * @brief DMA channel ID, a smaller channel ID indicates a higher priority.
  */
typedef enum {
    DMA_CHANNEL_FINISH = 0x00000000U,
    DMA_CHANNEL_ERROR = 0x00000001U
} DMA_CallbackFun_Type;

/**
  * @brief DMA Master type.
  */
typedef enum {
    DMA_MASTER1 = 0x00000000U,
    DMA_MASTER2 = 0x00000001U
} DMA_Master;

/**
  * @brief DMA request peripheral. The multiplexed transmitter requires additional
  * configuration of the system register.
  * @details DMA request line type:
  *          + DMA_REQUEST_I2C_RX -- I2C_RX and TIMER2 reuse the request line numbered 0
  *          + DMA_REQUEST_I2C_TX -- I2C_TX and TIMER3 reuse the request line numbered 1
  *          + DMA_REQUEST_UART0_RX -- UART0_RX use the request line numbered 2
  *          + DMA_REQUEST_UART0_TX -- UART0_TX use the request line numbered 3
  *          + DMA_REQUEST_UART1_RX -- UART1_RX use the request line numbered 4
  *          + DMA_REQUEST_UART1_TX -- UART1_RX and APT8 reuse the request line numbered 5
  *          + DMA_REQUEST_UART2_RX -- SPI_RX and UART2_RX reuse the request line numbered 6
  *          + DMA_REQUEST_UART2_TX -- SPI_TX and UART2_TX reuse the request line numbered 7
  *          + DMA_REQUEST_SPI_RX -- SPI_RX and UART2_RX reuse the request line numbered 6
  *          + DMA_REQUEST_SPI_TX -- UART1_RX and APT8 reuse the request line numbered 5
  *          + DMA_REQUEST_CAPM0 -- CAPM0 and APT0 reuse the request line numbered 8
  *          + DMA_REQUEST_CAPM1 -- CAPM1 and APT1 reuse the request line numbered 9
  *          + DMA_REQUEST_CAPM2 -- CAPM2 and APT2 reuse the request line numbered 10
  *          + DMA_REQUEST_ADC0 -- ADC0 and APT3 reuse the request line numbered 11
  *          + DMA_REQUEST_ADC1 -- ADC1 and APT4 reuse the request line numbered 12
  *          + DMA_REQUEST_ADC2 -- ADC2 and APT5 reuse the request line numbered 13
  *          + DMA_REQUEST_TIMER0 -- TIMER0 and APT6 reuse the request line numbered 14
  *          + DMA_REQUEST_TIMER1 -- TIMER1 and APT7 reuse the request line numbered 15
  *          + DMA_REQUEST_TIMER2 -- TIMER2 and I2C_RX reuse the request line numbered 0
  *          + DMA_REQUEST_TIMER3 -- TIMER3 and I2C_TX reuse the request line numbered 1
  *          + DMA_REQUEST_APT0 -- CAPM0 and APT0 reuse the request line numbered 8
  *          + DMA_REQUEST_APT1 -- CAMP1 and APT1 reuse the request line numbered 9
  *          + DMA_REQUEST_APT2 -- CAPM2 and APT2 reuse the request line numbered 10
  *          + DMA_REQUEST_APT3 -- ADC0 and APT3 reuse the request line numbered 11
  *          + DMA_REQUEST_APT4 -- ADC1 and APT4 reuse the request line numbered 12
  *          + DMA_REQUEST_APT5 -- ADC2 and APT5 reuse the request line numbered 13
  *          + DMA_REQUEST_APT6 -- TIMER0 and APT6 reuse the request line numbered 14
  *          + DMA_REQUEST_APT7 -- TIMER1 and APT7 reuse the request line numbered 15
  *          + DMA_REQUEST_APT8 -- APT8 and UART1_RX reuse the request line numbered 5
  *          + DMA_REQUEST_MEM -- The source and destination devices are memory
  */
typedef enum {
    DMA_REQUEST_I2C_RX = 0x00000000U,
    DMA_REQUEST_I2C_TX = 0x00000001U,
    DMA_REQUEST_UART0_RX = 0x00000002U,
    DMA_REQUEST_UART0_TX = 0x00000003U,
    DMA_REQUEST_UART1_RX = 0x00000004U,
    DMA_REQUEST_UART1_TX = 0x00000005U,
    DMA_REQUEST_SPI_RX = 0x00000006U,
    DMA_REQUEST_SPI_TX = 0x00000007U,
    DMA_REQUEST_CAPM0 = 0x00000008U,
    DMA_REQUEST_CAPM1 = 0x00000009U,
    DMA_REQUEST_CAPM2 = 0x0000000AU,
    DMA_REQUEST_ADC0 = 0x0000000BU,
    DMA_REQUEST_ADC1 = 0x0000000CU,
    DMA_REQUEST_ADC2 = 0x0000000DU,
    DMA_REQUEST_TIMER0 = 0x0000000EU,
    DMA_REQUEST_TIMER1 = 0x0000000FU,
    DMA_REQUEST_UART2_RX = 0x00000010U,
    DMA_REQUEST_UART2_TX = 0x00000011U,
    DMA_REQUEST_APT8 = 0x00000012U,
    DMA_REQUEST_APT0 = 0x00000013U,
    DMA_REQUEST_APT1 = 0x00000014U,
    DMA_REQUEST_APT2 = 0x00000015U,
    DMA_REQUEST_APT3 = 0x00000016U,
    DMA_REQUEST_APT4 = 0x00000017U,
    DMA_REQUEST_APT5 = 0x00000018U,
    DMA_REQUEST_APT6 = 0x00000019U,
    DMA_REQUEST_APT7 = 0x0000001AU,
    DMA_REQUEST_TIMER2 = 0x0000001BU,
    DMA_REQUEST_TIMER3 = 0x0000001CU,
    DMA_REQUEST_MEM = 0x0000001DU
} DMA_RequestLineNum;

/**
  * @brief DMA peripheral request line. The multiplexed transmitter requires additional
  * configuration of the system register.
  */
typedef enum {
    DMA_REQLINEVAL_0 = 0x00000000U,
    DMA_REQLINEVAL_1 = 0x00000001U,
    DMA_REQLINEVAL_2 = 0x00000002U,
    DMA_REQLINEVAL_3 = 0x00000003U,
    DMA_REQLINEVAL_4 = 0x00000004U,
    DMA_REQLINEVAL_5 = 0x00000005U,
    DMA_REQLINEVAL_6 = 0x00000006U,
    DMA_REQLINEVAL_7 = 0x00000007U,
    DMA_REQLINEVAL_8 = 0x00000008U,
    DMA_REQLINEVAL_9 = 0x00000009U,
    DMA_REQLINEVAL_10 = 0x0000000AU,
    DMA_REQLINEVAL_11 = 0x0000000BU,
    DMA_REQLINEVAL_12 = 0x0000000CU,
    DMA_REQLINEVAL_13 = 0x0000000DU,
    DMA_REQLINEVAL_14 = 0x0000000EU,
    DMA_REQLINEVAL_15 = 0x0000000FU
} DMA_ReqLineVal;

/**
  * @brief Configuration value definition of the peripheral multiplexing DMA request line.
  */
typedef enum {
    DMA_SYSCTRLSET_0 = 0x00000000U,
    DMA_SYSCTRLSET_1 = 0x00000001U,
    DMA_SYSCTRLSET_2 = 0x00000002U
} DMA_SysctrlSet;

/**
  * @brief DMA Transfer Byte Order.
  */
typedef enum {
    DMA_BYTEORDER_SMALLENDIAN = 0x00000000U,
    DMA_BYTEORDER_BIGENDIAN = 0x00000001U
} DMA_ByteOrder;

/**
  * @brief Define the transmission direction type and data flow controller.
  * @details Transmission direction type:
  *          + DMA_MEMORY_TO_MEMORY_BY_DMAC -- Direc: memory to memory, control: DMAC
  *          + DMA_MEMORY_TO_PERIPH_BY_DMAC -- Direc: memory to peripheral, control: DMAC
  *          + DMA_PERIPH_TO_MEMORY_BY_DMAC -- Direc: peripheral to memory, control: DMAC
  *          + DMA_PERIPH_TO_PERIPH_BY_DMAC -- Direc: peripheral to peripheral, control: DMAC
  *          + DMA_PERIPH_TO_PERIPH_BY_DES  -- Direc: peripheral to peripheral, control: destination peripheral
  *          + DMA_MEMORY_TO_PERIPH_BY_DES  -- Direc: memory to peripheral, control: destination peripheral
  *          + DMA_PERIPH_TO_MEMORY_BY_SRC  -- Direc: peripheral to memory, control: source peripheral
  *          + DMA_PERIPH_TO_PERIPH_BY_SRC  -- Direc: peripheral to peripheral, control: source peripheral
  *
  */
typedef enum {
    DMA_MEMORY_TO_MEMORY_BY_DMAC = 0x00000000U,
    DMA_MEMORY_TO_PERIPH_BY_DMAC = 0x00000001U,
    DMA_PERIPH_TO_MEMORY_BY_DMAC = 0x00000002U,
    DMA_PERIPH_TO_PERIPH_BY_DMAC = 0x00000003U,
    DMA_PERIPH_TO_PERIPH_BY_DES = 0x00000004U,
    DMA_MEMORY_TO_PERIPH_BY_DES = 0x00000005U,
    DMA_PERIPH_TO_MEMORY_BY_SRC = 0x00000006U,
    DMA_PERIPH_TO_PERIPH_BY_SRC = 0x00000007U
} DMA_TransDirection;

/**
  * @brief Address increase configuration. Peripherals can only be set to unaltered, memory can be set to two mode.
  */
typedef enum {
    DMA_ADDR_UNALTERED = 0x00000000U,
    DMA_ADDR_INCREASE = 0x00000001U
} DMA_AddrIncMode;

/**
  * @brief DMA extend handle.
  */
typedef struct _DMA_ExtendHandle {
    DMA_ByteOrder         srcByteOrder;  /**< Master1 is defined for source device, set byteOrder */
    DMA_ByteOrder         destByteOrder; /**< Master2 is defined for destination device, set byteOrder */
} DMA_ExtendHandle;

/**
  * @brief DMA user callback.
  */
typedef struct {
    struct {
        void (* ChannelFinishCallBack)(void *handle);
        void (* ChannelErrorCallBack)(void *handle);
    } DMA_CallbackFuns[CHANNEL_MAX_NUM];
} DMA_UserCallBack;
/**
  * @}
  */

/**
  * @defgroup DMA_Reg_Def DMA Register Definition
  * @brief Description DMA register mapping structure.
  * @{
  */

/**
  * @brief DMAC interrupt status register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ch0_int_stat : 1;    /**< Masked interrupt status of channel 0. */
        unsigned int ch1_int_stat : 1;    /**< Masked interrupt status of channel 1. */
        unsigned int ch2_int_stat : 1;    /**< Masked interrupt status of channel 2. */
        unsigned int ch3_int_stat : 1;    /**< Masked interrupt status of channel 3. */
        unsigned int reserved0 : 28;
    } BIT;
} volatile DMAC_INT_STAT_REG;

/**
  * @brief DMAC transfer completion interrupt status register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ch0_int_tc_stat : 1;    /**< Masked transfer completion interrupt status of channel 0. */
        unsigned int ch1_int_tc_stat : 1;    /**< Masked transfer completion interrupt status of channel 1. */
        unsigned int ch2_int_tc_stat : 1;    /**< Masked transfer completion interrupt status of channel 2. */
        unsigned int ch3_int_tc_stat : 1;    /**< Masked transfer completion interrupt status of channel 3. */
        unsigned int reserved0 : 28;
    } BIT;
} volatile DMAC_INT_TC_STAT_REG;

/**
  * @brief DMAC transfer completion interrupt clear register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ch0_int_tc_clr : 1;    /**< Clear the channel 0 transfer completion interrupt. */
        unsigned int ch1_int_tc_clr : 1;    /**< Clear the channel 1 transfer completion interrupt. */
        unsigned int ch2_int_tc_clr : 1;    /**< Clear the channel 2 transfer completion interrupt. */
        unsigned int ch3_int_tc_clr : 1;    /**< Clear the channel 3 transfer completion interrupt. */
        unsigned int reserved1 : 28;
    } BIT;
} volatile DMAC_INT_TC_CLR_REG;

/**
  * @brief DMAC error interrupt status register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ch0_int_err_stat : 1;    /**< Masked error interrupt status of channel 0. */
        unsigned int ch1_int_err_stat : 1;    /**< Masked error interrupt status of channel 1. */
        unsigned int ch2_int_err_stat : 1;    /**< Masked error interrupt status of channel 2. */
        unsigned int ch3_int_err_stat : 1;    /**< Masked error interrupt status of channel 3. */
        unsigned int reserved0 : 28;
    } BIT;
} volatile DMAC_INT_ERR_STAT_REG;

/**
  * @brief DMAC error interrupt clear register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ch0_int_err_clr : 1;    /**< Clear channel 0 error interrupt. */
        unsigned int ch1_int_err_clr : 1;    /**< Clear channel 1 error interrupt. */
        unsigned int ch2_int_err_clr : 1;    /**< Clear channel 2 error interrupt. */
        unsigned int ch3_int_err_clr : 1;    /**< Clear channel 3 error interrupt. */
        unsigned int reserved0 : 28;
    } BIT;
} volatile DMAC_INT_ERR_CLR_REG;

/**
  * @brief DMAC raw transfer completion interrupt register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ch0_raw_int_tc : 1;    /**< Raw transfer completion interrupt status of channel 0. */
        unsigned int ch1_raw_int_tc : 1;    /**< Raw transfer completion interrupt status of channel 1. */
        unsigned int ch2_raw_int_tc : 1;    /**< Raw transfer completion interrupt status of channel 2. */
        unsigned int ch3_raw_int_tc : 1;    /**< Raw transfer completion interrupt status of channel 3. */
        unsigned int reserved0 : 28;
    } BIT;
} volatile DMAC_RAW_INT_TC_STAT_REG;

/**
  * @brief DMAC raw error interrupt register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ch0_raw_int_err : 1;    /**< Raw error interrupt status of channel 0. */
        unsigned int ch1_raw_int_err : 1;    /**< Raw error interrupt status of channel 1. */
        unsigned int ch2_raw_int_err : 1;    /**< Raw error interrupt status of channel 2. */
        unsigned int ch3_raw_int_err : 1;    /**< Raw error interrupt status of channel 3. */
        unsigned int reserved0 : 28;
    } BIT;
} volatile DMAC_RAW_INT_ERR_STAT_REG;

/**
  * @brief DMAC channel enable status register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ch0_enabled : 1;    /**< Channel 0 enable status. */
        unsigned int ch1_enabled : 1;    /**< Channel 1 enable status. */
        unsigned int ch2_enabled : 1;    /**< Channel 2 enable status. */
        unsigned int ch3_enabled : 1;    /**< Channel 3 enable status. */
        unsigned int reserved0 : 28;
    } BIT;
} volatile DMAC_ENABLED_CHNS_REG;

/**
  * @brief DMAC software burst transfer request register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soft_breq : 16;    /**< Software control the generation of DMA burst transfer request. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile DMAC_SOFT_BREQ_REG;

/**
  * @brief DMAC software single transfer request register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soft_sreq : 16;    /**< Software control the generation of DMA single transfer request. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile DMAC_SOFT_SREQ_REG;

/**
  * @brief DMAC software last burst request register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soft_lbreq : 16;    /**< Software initiate a last burst request. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile DMAC_SOFT_LBREQ_REG;

/**
  * @brief DMAC software last single request register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soft_lsreq : 16;    /**< Software initiate a last single request. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile DMAC_SOFT_LSREQ_REG;

/**
  * @brief DMAC parameter configuration register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int dmac_enable : 1;    /**< DMA controller enable. */
        unsigned int m1_endianness : 1;  /**< Master 1 byte sequence configuration. */
        unsigned int m2_endianness : 1;  /**< Master 2 byte sequence configuration. */
        unsigned int reserved0 : 29;
    } BIT;
} volatile DMAC_CONFIG_REG;

/**
  * @brief DMAC request line synchronization enable.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int dmac_sync : 16;    /**< Control whether the request line needs to be synchronized.. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile DMAC_SYNC_REG;

/**
  * @brief Source address register of DMA channel n (n = 0, 1, 2, 3).
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int src_addr : 32;    /**< DMA source address. */
    } BIT;
} volatile DMAC_Cn_SRC_ADDR_REG;

/**
  * @brief Destination address register of DMA channel n (n = 0, 1, 2, 3).
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int dest_addr : 32;    /**< DMA destination address. */
    } BIT;
} volatile DMAC_Cn_DEST_ADDR_REG;

/**
  * @brief Linked list information register for DMA channel n (n = 0, 1, 2, 3).
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ll_master : 1;    /**< Master of the next linked list node : Master1 or Master2. */
        unsigned int reserved0 : 1;
        unsigned int ll_item : 30;     /**< Address of the next linked list node. */
    } BIT;
} volatile DMAC_Cn_LLI_REG;

/**
  * @brief DMA channel n (n = 0, 1, 2, 3) control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int trans_size : 12;    /**< Length of the DMA transfer, provided that the DMAC flow controller. */
        unsigned int sbsize : 3;         /**< Burst length of the source device. */
        unsigned int dbsize : 3;         /**< Burst length of the destination device. */
        unsigned int swidth : 3;         /**< Transfer bit width of the source device,
                                              which cannot be greater than Master bit width. */
        unsigned int dwidth : 3;         /**< Transfer bit width of the destination device,
                                              which cannot be greater than Master bit width. */
        unsigned int src_select : 1;     /**< Set Master for accessing the source device : Master 1 or Master 2. */
        unsigned int dest_select : 1;    /**< Set Master for accessing the destination device : Master 1 or Master 2. */
        unsigned int src_incr : 1;       /**< Set the incremental mode of the source address. */
        unsigned int dest_incr : 1;      /**< Set the incremental mode of the destination address. */
        unsigned int reserved0 : 3;
        unsigned int int_tc_enable : 1;  /**< Transfer completion interrupt enable. */
    } BIT;
} volatile DMAC_Cn_CONTROL_REG;

/**
  * @brief DMA channel n (n = 0, 1, 2, 3) configuration register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ch_en : 1;          /**< Channel enable. */
        unsigned int src_periph : 4;     /**< Source device, ignore this field if memory device. */
        unsigned int reserved0 : 1;
        unsigned int dest_periph : 4;    /**< Destination device, ignore this field if memory device. */
        unsigned int reserved1 : 1;
        unsigned int flow_ctrl : 3;      /**< Flow control and transmission Type. */
        unsigned int err_int_msk : 1;    /**< Error interrupt mask flag. */
        unsigned int tc_int_msk : 1;     /**< Transfer completion interrupt mask flag. */
        unsigned int ch_lock : 1;        /**< Lock transmission enable on the bus. */
        unsigned int ch_active : 1;      /**< Whether the data in the channel FIFO. */
        unsigned int ch_halt : 1;        /**< Whether ignore DMA requests. */
        unsigned int reserved2 : 13;
    } BIT;
} volatile DMAC_Cn_CONFIG_REG;

/**
  * @brief DMA register mapping structure.
  */
typedef struct {
    DMAC_INT_STAT_REG           DMAC_INT_STAT;          /**< DMAC interrupt status register,
                                                             Offset address: 0x00000000U. */
    DMAC_INT_TC_STAT_REG        DMAC_INT_TC_STAT;       /**< DMAC transfer completion interrupt status register,
                                                             Offset address: 0x00000004U. */
    DMAC_INT_TC_CLR_REG         DMAC_INT_TC_CLR;        /**< DMAC transfer completion interrupt clear register,
                                                             Offset address: 0x00000008U. */
    DMAC_INT_ERR_STAT_REG       DMAC_INT_ERR_STAT;      /**< DMAC error interrupt status register,
                                                             Offset address: 0x0000000CU. */
    DMAC_INT_ERR_CLR_REG        DMAC_INT_ERR_CLR;       /**< DMAC error interrupt clear register,
                                                             Offset address: 0x00000010U. */
    DMAC_RAW_INT_TC_STAT_REG    DMAC_RAW_INT_TC_STAT;   /**< DMAC raw transfer completion interrupt register,
                                                             Offset address: 0x00000014U. */
    DMAC_RAW_INT_ERR_STAT_REG   DMAC_RAW_INT_ERR_STAT;  /**< DMAC raw error interrupt register,
                                                             Offset address: 0x00000018U. */
    DMAC_ENABLED_CHNS_REG       DMAC_ENABLED_CHNS;      /**< DMAC channel enable status register,
                                                             Offset address: 0x0000001CU. */
    DMAC_SOFT_BREQ_REG          DMAC_SOFT_BREQ;         /**< DMAC software burst transfer request register,
                                                             Offset address: 0x00000020U. */
    DMAC_SOFT_SREQ_REG          DMAC_SOFT_SREQ;         /**< DMAC software single transfer request register,
                                                             Offset address: 0x00000024U. */
    DMAC_SOFT_LBREQ_REG         DMAC_SOFT_LBREQ;        /**< DMAC software last burst request register,
                                                             Offset address: 0x00000028U. */
    DMAC_SOFT_LSREQ_REG         DMAC_SOFT_LSREQ;        /**< DMAC software last single request register,
                                                             Offset address: 0x0000002CU. */
    DMAC_CONFIG_REG             DMAC_CONFIG;            /**< DMAC parameter configuration register,
                                                             Offset address: 0x00000030U. */
    DMAC_SYNC_REG               DMAC_SYNC;              /**< DMAC request line synchronization enable,
                                                             Offset address: 0x00000034U. */
    char space0[200];
    DMAC_Cn_SRC_ADDR_REG        DMAC_C0_SRC_ADDR;       /**< Source address register of DMA channel 0,
                                                             Offset address: 0x00000100U. */
    DMAC_Cn_DEST_ADDR_REG       DMAC_C0_DEST_ADDR;      /**< Destination address register of DMA channel 0,
                                                             Offset address: 0x00000104U. */
    DMAC_Cn_LLI_REG             DMAC_C0_LLI;            /**< Linked list information register for DMA channel 0,
                                                             Offset address: 0x00000108U. */
    DMAC_Cn_CONTROL_REG         DMAC_C0_CONTROL;        /**< DMA channel 0 control register,
                                                             Offset address: 0x0000010CU. */
    DMAC_Cn_CONFIG_REG          DMAC_C0_CONFIG;         /**< DMA channel 0 configuration register,
                                                             Offset address: 0x00000110U. */
    char space1[12];
    DMAC_Cn_SRC_ADDR_REG        DMAC_C1_SRC_ADDR;       /**< Source address register of DMA channel 1,
                                                             Offset address: 0x00000120U. */
    DMAC_Cn_DEST_ADDR_REG       DMAC_C1_DEST_ADDR;      /**< Destination address register of DMA channel 1,
                                                             Offset address: 0x00000124U. */
    DMAC_Cn_LLI_REG             DMAC_C1_LLI;            /**< Linked list information register for DMA channel 1,
                                                             Offset address: 0x00000128U. */
    DMAC_Cn_CONTROL_REG         DMAC_C1_CONTROL;        /**< DMA channel 1 control register,
                                                             Offset address: 0x0000012CU. */
    DMAC_Cn_CONFIG_REG          DMAC_C1_CONFIG;         /**< DMA channel 1 configuration register,
                                                             Offset address: 0x00000130U. */
    char space2[12];
    DMAC_Cn_SRC_ADDR_REG        DMAC_C2_SRC_ADDR;       /**< Source address register of DMA channel 2,
                                                             Offset address: 0x00000140U. */
    DMAC_Cn_DEST_ADDR_REG       DMAC_C2_DEST_ADDR;      /**< Destination address register of DMA channel 2,
                                                             Offset address: 0x00000144U. */
    DMAC_Cn_LLI_REG             DMAC_C2_LLI;            /**< Linked list information register for DMA channel 2,
                                                             Offset address: 0x00000148U. */
    DMAC_Cn_CONTROL_REG         DMAC_C2_CONTROL;        /**< DMA channel 2 control register,
                                                             Offset address: 0x0000014CU. */
    DMAC_Cn_CONFIG_REG          DMAC_C2_CONFIG;         /**< DMA channel 2 configuration register,
                                                             Offset address: 0x00000150U. */
    char space3[12];
    DMAC_Cn_SRC_ADDR_REG        DMAC_C3_SRC_ADDR;       /**< Source address register of DMA channel 3,
                                                             Offset address: 0x00000160U. */
    DMAC_Cn_DEST_ADDR_REG       DMAC_C3_DEST_ADDR;      /**< Destination address register of DMA channel 3,
                                                             Offset address: 0x00000164U. */
    DMAC_Cn_LLI_REG             DMAC_C3_LLI;            /**< Linked list information register for DMA channel 3,
                                                             Offset address: 0x00000168U. */
    DMAC_Cn_CONTROL_REG         DMAC_C3_CONTROL;        /**< DMA channel 3 control register,
                                                             Offset address: 0x0000016CU. */
    DMAC_Cn_CONFIG_REG          DMAC_C3_CONFIG;         /**< DMA channel 3 configuration register,
                                                             Offset address: 0x00000170U. */
} volatile DMA_RegStruct;

/**
  * @brief Channel register mapping structure.
  */
typedef struct {
    DMAC_Cn_SRC_ADDR_REG        DMAC_Cn_SRC_ADDR;     /**< Source address register of DMA channel. */
    DMAC_Cn_DEST_ADDR_REG       DMAC_Cn_DEST_ADDR;    /**< Destination address register of DMA channel. */
    DMAC_Cn_LLI_REG             DMAC_Cn_LLI;          /**< Linked list information register for DMA channel. */
    DMAC_Cn_CONTROL_REG         DMAC_Cn_CONTROL;      /**< DMA channel control register. */
    DMAC_Cn_CONFIG_REG          DMAC_Cn_CONFIG;       /**< DMA channel configuration register. */
} volatile DMA_ChannelRegStruct;

/**
  * @brief DMA linked list structure.
  */
typedef struct _DMA_LinkList {
    unsigned int                srcAddr;     /**< Source device start address. */
    unsigned int                destAddr;    /**< Destination device start address. */
    struct _DMA_LinkList       *lliNext;     /**< Pointer to the next node. */
    DMAC_Cn_CONTROL_REG         control;     /**< Channel parameters configured for the node. */
} DMA_LinkList;

/**
  * @brief A large amount of block data needs to be splited. Split functions need to transfer the following structure.
  */
typedef struct {
    unsigned int    srcAddr;    /**< Source device start address. */
    unsigned int    destAddr;   /**< Destination device start address. */
    unsigned int    srcIn;      /**< Source address single increment size. */
    unsigned int    destIn;     /**< destnation address single increment size. */
    unsigned int    chnParam;   /**< Channel parameters configured for the splited node. */
    unsigned int    totalSize;  /**< Total amount of block data. */
} DMA_SplitParam;

/**
  * @brief Struct of DMA peripheral request line and system register multiplexing.
  */
typedef struct {
    DMA_ReqLineVal reqLineVal;    /**< DMA peripheral request line. */
    DMA_SysctrlSet sysctrVal;     /**< Configuration value definition of the peripheral multiplexing. */
    unsigned int shiftLeft;       /**< Left shift of peripheral multiplexing configuration value. */
} DMA_PeriphReq;
/**
  * @}
  */


/**
  * @brief Check DMA channel num parameter.
  * @param channel The number of channel.
  * @retval bool
  */
static inline bool IsDmaChannelNum(DMA_ChannelNum channel)
{
    if ((channel == DMA_CHANNEL_ZERO) || (channel == DMA_CHANNEL_ONE) ||
        (channel == DMA_CHANNEL_TWO) || (channel == DMA_CHANNEL_THREE)) {
        return true;
    }
    return false;
}

/**
  * @brief Check DMA channel transfer width.
  * @param width DMA transfer width.
  * @retval bool
  */
static inline bool IsDmaWidth(DMA_TransmisWidth width)
{
    if ((width == DMA_TRANSWIDTH_BYTE) ||
        (width == DMA_TRANSWIDTH_HALFWORD) ||
        (width == DMA_TRANSWIDTH_WORD)) {
        return true;
    }
    return false;
}

/**
  * @brief Check DMA channel burst length.
  * @param burstLength DMA transfer burst length.
  * @retval bool
  */
static inline bool IsDmaBurstLength(DMA_BurstLength burstLength)
{
    if ((burstLength == DMA_BURST_LENGTH_1) || (burstLength == DMA_BURST_LENGTH_4) ||
        (burstLength == DMA_BURST_LENGTH_8) || (burstLength == DMA_BURST_LENGTH_16) ||
        (burstLength == DMA_BURST_LENGTH_32) || (burstLength == DMA_BURST_LENGTH_64) ||
        (burstLength == DMA_BURST_LENGTH_128) || (burstLength == DMA_BURST_LENGTH_256)) {
        return true;
    }
    return false;
}

/**
  * @brief Check DMA type of byte order.
  * @param byteOrder DMA master1/master2 byte order.
  * @retval bool
  */
static inline bool IsDmaByteOrder(DMA_ByteOrder byteOrder)
{
    return (byteOrder == DMA_BYTEORDER_SMALLENDIAN) || (byteOrder == DMA_BYTEORDER_BIGENDIAN);
}

/**
  * @brief Check DMA type of address change.
  * @param byteOrder DMA source/destination address change type.
  * @retval bool
  */
static inline bool IsDmaAddrMode(DMA_AddrIncMode addrMode)
{
    return (addrMode == DMA_ADDR_UNALTERED) || (addrMode == DMA_ADDR_INCREASE);
}

/**
  * @brief Check DMA type of direction.
  * @param direction DMA transmfer direction.
  * @retval bool
  */
static inline bool IsDmaDirection(DMA_TransDirection direction)
{
    if ((direction == DMA_MEMORY_TO_MEMORY_BY_DMAC) || (direction == DMA_MEMORY_TO_PERIPH_BY_DMAC) ||
        (direction == DMA_PERIPH_TO_MEMORY_BY_DMAC) || (direction == DMA_PERIPH_TO_PERIPH_BY_DMAC) ||
        (direction == DMA_PERIPH_TO_PERIPH_BY_DES) || (direction == DMA_MEMORY_TO_PERIPH_BY_DES) ||
        (direction == DMA_PERIPH_TO_MEMORY_BY_SRC) || (direction == DMA_PERIPH_TO_PERIPH_BY_SRC)) {
        return true;
    }
    return false;
}
/**
  * @brief Check DMA num of request peripheral.
  * @param reqPeriph peripherals supported by the DMA.
  * @retval bool
  */
static inline bool IsDmaReqPeriph(DMA_RequestLineNum reqPeriph)
{
    return (reqPeriph >= DMA_REQUEST_I2C_RX) && (reqPeriph <= DMA_REQUEST_MEM);
}

/**
  * @brief Check whether the address is valid.
  * @param address Address for the DMA to transfer data.
  * @retval bool
  */
static inline bool IsDmaValidAddress(unsigned long long address)
{
    return (address >= SRAM_START && address <= SRAM_END) || (address >= REGISTER_START && address <= REGISTER_END);
}

/**
  * @brief Check whether the master is valid.
  * @param master Master of DMA.
  * @retval bool
  */
static inline bool IsDmaMaster(DMA_Master master)
{
    return (master == DMA_MASTER1) || (master == DMA_MASTER2);
}

/**
  * @brief DMA configurate the byte order of master1.
  * @param dmax DMA register base address.
  * @param byteOrder DMA byte order.
  * @retval None.
  */
static inline void DCL_DMA_SetMast1ByteOrder(DMA_RegStruct * const dmax, DMA_ByteOrder byteOrder)
{
    DMA_ASSERT_PARAM(IsDMAInstance(dmax));
    DMA_PARAM_CHECK_NO_RET(IsDmaByteOrder(byteOrder));
    dmax->DMAC_CONFIG.BIT.m1_endianness = byteOrder;
}

/**
  * @brief DMA configurate the byte order of master2.
  * @param dmax DMA register base address.
  * @param byteOrder DMA byte order.
  * @retval None.
  */
static inline void DCL_DMA_SetMast2ByteOrder(DMA_RegStruct * const dmax, DMA_ByteOrder byteOrder)
{
    DMA_ASSERT_PARAM(IsDMAInstance(dmax));
    DMA_PARAM_CHECK_NO_RET(IsDmaByteOrder(byteOrder));
    dmax->DMAC_CONFIG.BIT.m2_endianness = byteOrder;
}

/**
  * @brief DMA configurate the direction.
  * @param dmaChannelx DMA channel register base address.
  * @param direction Direction of channel.
  * @retval None.
  */
static inline void DCL_DMA_SetDirection(DMA_ChannelRegStruct * const dmaChannelx, DMA_TransDirection direction)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(IsDmaDirection(direction));
    dmaChannelx->DMAC_Cn_CONFIG.BIT.flow_ctrl = direction;
}

/**
  * @brief DMA configurate the address of source.
  * @param dmaChannelx DMA channel register base address.
  * @param srcAddr Address of source.
  * @retval None.
  */
static inline void DCL_DMA_SetSrcAddr(DMA_ChannelRegStruct * const dmaChannelx, unsigned int srcAddr)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(IsDmaValidAddress(srcAddr));
    dmaChannelx->DMAC_Cn_SRC_ADDR.BIT.src_addr = srcAddr;
}

/**
  * @brief DMA configurate the address of destnation.
  * @param dmaChannelx DMA channel register base address.
  * @param destAddr Address of destnation.
  * @retval None.
  */
static inline void DCL_DMA_SetDestAddr(DMA_ChannelRegStruct * const dmaChannelx, unsigned int destAddr)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(IsDmaValidAddress(destAddr));
    dmaChannelx->DMAC_Cn_DEST_ADDR.BIT.dest_addr = destAddr;
}

/**
  * @brief DMA configurate the address mode of source.
  * @param dmaChannelx DMA channel register base address.
  * @param srcAddrInc The address mode of source.
  * @retval None.
  */
static inline void DCL_DMA_SetSrcAddrMode(DMA_ChannelRegStruct * const dmaChannelx, DMA_AddrIncMode srcAddrInc)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(IsDmaAddrMode(srcAddrInc));
    dmaChannelx->DMAC_Cn_CONTROL.BIT.src_incr = srcAddrInc;
}

/**
  * @brief DMA configurate the address mode of destnation.
  * @param dmaChannelx DMA channel register base address.
  * @param destAddrInc The address mode of destnation.
  * @retval None.
  */
static inline void DCL_DMA_SetDestAddrMode(DMA_ChannelRegStruct * const dmaChannelx, DMA_AddrIncMode destAddrInc)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(IsDmaAddrMode(destAddrInc));
    dmaChannelx->DMAC_Cn_CONTROL.BIT.dest_incr = destAddrInc;
}

/**
  * @brief DMA configurate the bit width of source.
  * @param dmaChannelx DMA channel register base address.
  * @param srcWidth The bit width of source.
  * @retval None.
  */
static inline void DCL_DMA_SetSrcWidth(DMA_ChannelRegStruct * const dmaChannelx, DMA_TransmisWidth srcWidth)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(IsDmaWidth(srcWidth));
    dmaChannelx->DMAC_Cn_CONTROL.BIT.swidth = srcWidth;
}

/**
  * @brief DMA configurate the bit width of destnation.
  * @param dmaChannelx DMA channel register base address.
  * @param destWidth The bit width of destnation.
  * @retval None.
  */
static inline void DCL_DMA_SetDestWidth(DMA_ChannelRegStruct * const dmaChannelx, DMA_TransmisWidth destWidth)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(IsDmaWidth(destWidth));
    dmaChannelx->DMAC_Cn_CONTROL.BIT.dwidth = destWidth;
}

/**
  * @brief DMA configurate the burst size of source.
  * @param dmaChannelx DMA channel register base address.
  * @param srcBurst The burst size of source.
  * @retval None.
  */
static inline void DCL_DMA_SetSrcBurst(DMA_ChannelRegStruct * const dmaChannelx, DMA_BurstLength srcBurst)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(IsDmaBurstLength(srcBurst));
    dmaChannelx->DMAC_Cn_CONTROL.BIT.sbsize = srcBurst;
}

/**
  * @brief DMA configurate the burst size of destnation.
  * @param dmaChannelx DMA channel register base address.
  * @param destBurst The burst size of destnation.
  * @retval None.
  */
static inline void DCL_DMA_SetDestBurst(DMA_ChannelRegStruct * const dmaChannelx, DMA_BurstLength destBurst)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(IsDmaBurstLength(destBurst));
    dmaChannelx->DMAC_Cn_CONTROL.BIT.dbsize = destBurst;
}

/**
  * @brief DMA configurate the transfer size.
  * @param dmaChannelx DMA channel register base address.
  * @param dataLength The transfer size.
  * @retval None.
  */
static inline void DCL_DMA_SetTransferSize(DMA_ChannelRegStruct * const dmaChannelx, unsigned int dataLength)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(dataLength <= 0xFFF);
    dmaChannelx->DMAC_Cn_CONTROL.BIT.trans_size = dataLength;
}

/**
  * @brief Enable channel completion interrupt.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_EnableIT(DMA_ChannelRegStruct * const dmaChannelx)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    dmaChannelx->DMAC_Cn_CONFIG.BIT.err_int_msk = BASE_CFG_ENABLE;
    dmaChannelx->DMAC_Cn_CONFIG.BIT.tc_int_msk = BASE_CFG_ENABLE;
}

/**
  * @brief Disable channel completion interrupt.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_DisableIT(DMA_ChannelRegStruct * const dmaChannelx)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    dmaChannelx->DMAC_Cn_CONFIG.BIT.err_int_msk = BASE_CFG_DISABLE;
    dmaChannelx->DMAC_Cn_CONFIG.BIT.tc_int_msk = BASE_CFG_DISABLE;
}

/**
  * @brief Enables the channel to start transmission.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_EnableChannel(DMA_ChannelRegStruct * const dmaChannelx)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    dmaChannelx->DMAC_Cn_CONFIG.BIT.ch_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable the channel to start transmission.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_DisableChannel(DMA_ChannelRegStruct * const dmaChannelx)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    dmaChannelx->DMAC_Cn_CONFIG.BIT.ch_en = BASE_CFG_DISABLE;
}

/**
  * @brief Enable the DMA controller.
  * @param dmax DMA register base address.
  * @retval None.
  */
static inline void DCL_DMA_EnableDMAController(DMA_RegStruct * const dmax)
{
    DMA_ASSERT_PARAM(IsDMAInstance(dmax));
    dmax->DMAC_CONFIG.BIT.dmac_enable = BASE_CFG_ENABLE;
}

/**
  * @brief Disable the DMA controller.
  * @param dmax DMA register base address.
  * @retval None.
  */
static inline void DCL_DMA_DisableDMAController(DMA_RegStruct * const dmax)
{
    DMA_ASSERT_PARAM(IsDMAInstance(dmax));
    dmax->DMAC_CONFIG.BIT.dmac_enable = BASE_CFG_DISABLE;
}

/**
  * @brief Clear the transfer completion interrupt.
  * @param dmax DMA register base address.
  * @param channel channel of DMA.
  * @retval None.
  */
static inline void DCL_DMA_ClearTransferCompleteInt(DMA_RegStruct * const dmax, DMA_ChannelNum channel)
{
    DMA_ASSERT_PARAM(IsDMAInstance(dmax));
    DMA_PARAM_CHECK_NO_RET(IsDmaChannelNum(channel));
    dmax->DMAC_INT_TC_CLR.reg |= (1U << (unsigned int)channel);
}

/**
  * @brief Clear the transfer error interrupt.
  * @param dmax DMA register base address.
  * @param channel channel of DMA.
  * @retval None.
  */
static inline void DCL_DMA_ClearTransferErrorInt(DMA_RegStruct * const dmax, DMA_ChannelNum channel)
{
    DMA_ASSERT_PARAM(IsDMAInstance(dmax));
    DMA_PARAM_CHECK_NO_RET(IsDmaChannelNum(channel));
    dmax->DMAC_INT_ERR_CLR.reg |= (1U << (unsigned int)channel);
}

/**
  * @brief Enable request line synchronization.
  * @param dmax DMA register base address.
  * @retval None.
  */
static inline void DCL_DMA_EnableRequestSync(DMA_RegStruct * const dmax)
{
    DMA_ASSERT_PARAM(IsDMAInstance(dmax));
    dmax->DMAC_SYNC.reg = 0x00;
}

/**
  * @brief Disable request line synchronization.
  * @param dmax DMA register base address.
  * @retval None.
  */
static inline void DCL_DMA_DisableRequestSync(DMA_RegStruct * const dmax)
{
    DMA_ASSERT_PARAM(IsDMAInstance(dmax));
    dmax->DMAC_SYNC.reg = 0xFF;
}

/**
  * @brief Clearing Channel Configuration Parameters.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_ClearChannalParam(DMA_ChannelRegStruct * const dmaChannelx)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    dmaChannelx->DMAC_Cn_CONFIG.reg = 0x00;
}

/**
  * @brief Configure the source master interface of the channel.
  * @param dmaChannelx DMA channel register base address.
  * @param master Master interface type of DMA.
  * @retval None.
  */
static inline void DCL_DMA_SetSrcMasterChannal(DMA_ChannelRegStruct * const dmaChannelx, DMA_Master master)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(IsDmaMaster(master));
    dmaChannelx->DMAC_Cn_CONTROL.BIT.src_select = master;
}

/**
  * @brief Configure the destination master interface of the channel.
  * @param dmaChannelx DMA channel register base address.
  * @param master Master interface type of DMA.
  * @retval None.
  */
static inline void DCL_DMA_SetDestMasterChannal(DMA_ChannelRegStruct * const dmaChannelx, DMA_Master master)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(IsDmaMaster(master));
    dmaChannelx->DMAC_Cn_CONTROL.BIT.dest_select = master;
}

/**
  * @brief Interrupt generated when channel transfer.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_ChannalEnableInt(DMA_ChannelRegStruct * const dmaChannelx)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    dmaChannelx->DMAC_Cn_CONTROL.BIT.int_tc_enable = BASE_CFG_ENABLE;
}

/**
  * @brief No interrupt is generated after the channel transfer.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_ChannalDisableInt(DMA_ChannelRegStruct * const dmaChannelx)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    dmaChannelx->DMAC_Cn_CONTROL.BIT.int_tc_enable = BASE_CFG_DISABLE;
}

/**
  * @brief Obtaining the DMA channel state.
  * @param dmax DMA register base address.
  * @param channel channel of DMA.
  * @retval unsigned int.
  */
static inline unsigned int DCL_DMA_GetChannelState(DMA_RegStruct * const dmax, DMA_ChannelNum channel)
{
    DMA_ASSERT_PARAM(IsDMAInstance(dmax));
    DMA_PARAM_CHECK_WITH_RET(IsDmaChannelNum(channel), BASE_STATUS_ERROR);
    unsigned int val = dmax->DMAC_ENABLED_CHNS.reg;
    unsigned int ret = (val & (1U << channel));  /* Select the state of the specified channel */
    return ret;
}

/**
  * @brief Obtaining the DMA interrupt state.
  * @param dmax DMA register base address.
  * @param channel channel of DMA.
  * @retval unsigned int.
  */
static inline unsigned int DCL_DMA_GetIntState(DMA_RegStruct * const dmax, DMA_ChannelNum channel)
{
    DMA_ASSERT_PARAM(IsDMAInstance(dmax));
    DMA_PARAM_CHECK_WITH_RET(IsDmaChannelNum(channel), BASE_STATUS_ERROR);
    unsigned int val = dmax->DMAC_INT_STAT.reg;
    unsigned int ret = (val & (1U << channel));  /* Select the state of the specified interrupt */
    return ret;
}

/**
  * @brief Obtaining the DMA interrupt transfer complete state.
  * @param dmax DMA register base address.
  * @retval unsigned int.
  */
static inline unsigned int DCL_DMA_GetIntFinsihState(DMA_RegStruct * const dmax)
{
    DMA_ASSERT_PARAM(IsDMAInstance(dmax));
    return dmax->DMAC_INT_TC_STAT.reg;
}

/**
  * @brief Obtaining the DMA interrupt error state.
  * @param dmax DMA register base address.
  * @retval unsigned int.
  */
static inline unsigned int DCL_DMA_GetIntErrorState(DMA_RegStruct * const dmax)
{
    DMA_ASSERT_PARAM(IsDMAInstance(dmax));
    return dmax->DMAC_INT_ERR_STAT.reg;
}

/**
  * @brief Set the next node information.
  * @param dmaChannelx DMA channel register base address.
  * @param value Linked list field.
  * @retval None.
  */
static inline void DCL_DMA_SetListNextNode(DMA_ChannelRegStruct * const dmaChannelx, unsigned int value)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(value < 0xFFFFFFF0);
    dmaChannelx->DMAC_Cn_LLI.reg = value;
}

/**
  * @brief Halt DMA channel request.
  * @param dmaChannelx DMA channel register base address.
  * @retval void.
  */
static inline void DCL_DMA_HaltChannelRequest(DMA_ChannelRegStruct * const dmaChannelx)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    dmaChannelx->DMAC_Cn_CONFIG.BIT.ch_halt = BASE_CFG_ENABLE;
}

/**
  * @brief Allow DMA channel request.
  * @param dmaChannelx DMA channel register base address.
  * @retval void.
  */
static inline void DCL_DMA_AllowChannelRequest(DMA_ChannelRegStruct * const dmaChannelx)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    dmaChannelx->DMAC_Cn_CONFIG.BIT.ch_halt = BASE_CFG_DISABLE;
}

/**
  * @brief Allow DMA channel request.
  * @param dmaChannelx DMA channel register base address.
  * @retval unsigned int, 1: FIFO has data, 0: FIFO has not data.
  */
static inline unsigned int DCL_DMA_GetFifoState(DMA_ChannelRegStruct * const dmaChannelx)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    return dmaChannelx->DMAC_Cn_CONFIG.BIT.ch_active;
}
/**
  * @}
  */

/**
  * @}
  */
#endif  /* McuMagicTag_DMA_IP_H */