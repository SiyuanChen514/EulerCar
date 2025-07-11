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
#define CHANNEL_MAX_NUM 6

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
  * @brief DMA_IP: dma_v1.
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
    DMA_CHANNEL_THREE = 0x00000003U,
    DMA_CHANNEL_FOUR = 0x00000004U,
    DMA_CHANNEL_FIVE = 0x00000005U,
} DMA_ChannelNum;

/**
  * @brief DMA callback type.
  */
typedef enum {
    DMA_CHANNEL_FINISH = 0x00000000U,
    DMA_CHANNEL_ERROR = 0x00000001U
} DMA_CallbackFun_Type;

/**
  * @brief DMA channel priority.
  */
typedef enum {
    DMA_PRIORITY_LOW = 0x00000000U,
    DMA_PRIORITY_MEDIUM = 0x00000001U,
    DMA_PRIORITY_HIGH = 0x00000002U,
    DMA_PRIORITY_HIGHEST = 0x00000003U,
} DMA_ChannelPriority;

/**
  * @brief DMA request peripheral. The multiplexed transmitter requires additional
  * configuration of the system register.
  * @details DMA request line type:
  *          + DMA_REQUEST_I2C0_RX -- I2C0_RX use the request line numbered 0
  *          + DMA_REQUEST_I2C0_TX -- I2C0_TX use the request line numbered 1
  *          + DMA_REQUEST_I2C1_RX -- I2C1_RX use the request line numbered 2
  *          + DMA_REQUEST_I2C1_TX -- I2C1_RX use the request line numbered 3
  *          + DMA_REQUEST_UART0_RX -- UART0_TX use the request line numbered 4
  *          + DMA_REQUEST_UART0_TX -- UART0_TX use the request line numbered 5
  *          + DMA_REQUEST_UART1_RX -- UART1_RX use the request line numbered 6
  *          + DMA_REQUEST_UART1_TX -- UART1_TX use the request line numbered 7
  *          + DMA_REQUEST_UART2_RX -- UART2_RX use the request line numbered 8
  *          + DMA_REQUEST_UART2_TX -- UART2_TX use the request line numbered 9
  *          + DMA_REQUEST_UART3_RX -- UART3_RX use the request line numbered 30
  *          + DMA_REQUEST_UART3_TX -- UART3_TX use the request line numbered 31
  *          + DMA_REQUEST_CAPM0 -- CAPM0 use the request line numbered 10
  *          + DMA_REQUEST_CAPM1 -- CAPM1 use the request line numbered 11
  *          + DMA_REQUEST_CAPM2 -- CAPM2 use the request line numbered 12
  *          + DMA_REQUEST_ADC0 -- ADC0 use the request line numbered 13
  *          + DMA_REQUEST_TIMER0 -- TIMER0 use the request line numbered 14
  *          + DMA_REQUEST_TIMER1 -- TIMER1 use the request line numbered 15
  *          + DMA_REQUEST_TIMER2 -- TIMER2 use the request line numbered 16
  *          + DMA_REQUEST_TIMER3 -- TIMER3 use the request line numbered 17
  *          + DMA_REQUEST_SPI0_RX -- SPI0_RX ause the request line numbered 18
  *          + DMA_REQUEST_SPI0_TX -- SPI0_TX use the request line numbered 19
  *          + DMA_REQUEST_SPI1_RX -- SPI1_RX use the request line numbered 20
  *          + DMA_REQUEST_SPI1_TX -- SPI1_TX use the request line numbered 21
  *          + DMA_REQUEST_APT0 -- APT0 use the request line numbered 22
  *          + DMA_REQUEST_APT1 -- APT1 use the request line numbered 23
  *          + DMA_REQUEST_APT2 -- APT2 use the request line numbered 24
  *          + DMA_REQUEST_APT3 -- APT3 use the request line numbered 25
  *          + DMA_REQUEST_GPT0 -- GPT0 use the request line numbered 26
  *          + DMA_REQUEST_GPT1 -- GPT1 use the request line numbered 27
  *          + DMA_REQUEST_GPT2 -- GPT2 use the request line numbered 28
  *          + DMA_REQUEST_GPT3 -- GPT3 use the request line numbered 29
  *          + DMA_REQUEST_MEM -- The source and destination devices are memory
  */
typedef enum {
    DMA_REQUEST_I2C0_RX = 0x00000000U,
    DMA_REQUEST_I2C0_TX = 0x00000001U,
    DMA_REQUEST_I2C1_RX = 0x00000002U,
    DMA_REQUEST_I2C1_TX = 0x00000003U,
    DMA_REQUEST_UART0_RX = 0x00000004U,
    DMA_REQUEST_UART0_TX = 0x00000005U,
    DMA_REQUEST_UART1_RX = 0x00000006U,
    DMA_REQUEST_UART1_TX = 0x00000007U,
    DMA_REQUEST_UART2_RX = 0x00000008U,
    DMA_REQUEST_UART2_TX = 0x00000009U,
    DMA_REQUEST_UART3_RX = 0x0000001EU,
    DMA_REQUEST_UART3_TX = 0x0000001FU,
    DMA_REQUEST_CAPM0 = 0x0000000AU,
    DMA_REQUEST_CAPM1 = 0x0000000BU,
    DMA_REQUEST_CAPM2 = 0x0000000CU,
    DMA_REQUEST_ADC0 = 0x0000000DU,
    DMA_REQUEST_TIMER0 = 0x0000000EU,
    DMA_REQUEST_TIMER1 = 0x0000000FU,
    DMA_REQUEST_TIMER2 = 0x00000010U,
    DMA_REQUEST_TIMER3 = 0x00000011U,
    DMA_REQUEST_SPI0_RX = 0x00000012U,
    DMA_REQUEST_SPI0_TX = 0x00000013U,
    DMA_REQUEST_SPI1_RX = 0x00000014U,
    DMA_REQUEST_SPI1_TX = 0x00000015U,
    DMA_REQUEST_APT0 = 0x00000016U,
    DMA_REQUEST_APT1 = 0x00000017U,
    DMA_REQUEST_APT2 = 0x00000018U,
    DMA_REQUEST_APT3 = 0x00000019U,
    DMA_REQUEST_GPT0 = 0x0000001AU,
    DMA_REQUEST_GPT1 = 0x0000001BU,
    DMA_REQUEST_GPT2 = 0x0000001CU,
    DMA_REQUEST_GPT3 = 0x0000001DU,
    DMA_REQUEST_MEM = 0x00000020U,
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
  *          + DMA_MEMORY_TO_MEMORY_BY_DMAC -- Direc: memory to memory, control: DMA
  *          + DMA_MEMORY_TO_PERIPH_BY_DMAC -- Direc: memory to peripheral, control: DMA
  *          + DMA_PERIPH_TO_MEMORY_BY_DMAC -- Direc: peripheral to memory, control: DMA
  *          + DMA_PERIPH_TO_PERIPH_BY_DMAC -- irec: peripheral to peripheral, control: DMA
  *          + DMA_PERIPH_TO_PERIPH_BY_DES  -- Direc: peripheral to peripheral, control: destination peripheral
  *          + DMA_MEMORY_TO_PERIPH_BY_DES  -- Direc: memory to peripheral, control: destination peripheral
  *          + DMA_PERIPH_TO_MEMORY_BY_SRC  -- Direc: peripheral to memory, control: source peripheral
  *          + DMA_PERIPH_TO_PERIPH_BY_SRC  -- Direc: peripheral to peripheral, control: source peripheral
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
  * @brief DMA interrupt status register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ch0_int_stat : 1;    /**< Masked interrupt status of channel 0. */
        unsigned int ch1_int_stat : 1;    /**< Masked interrupt status of channel 1. */
        unsigned int ch2_int_stat : 1;    /**< Masked interrupt status of channel 2. */
        unsigned int ch3_int_stat : 1;    /**< Masked interrupt status of channel 3. */
        unsigned int ch4_int_stat : 1;    /**< Masked interrupt status of channel 4. */
        unsigned int ch5_int_stat : 1;    /**< Masked interrupt status of channel 5. */
        unsigned int reserved0 : 26;
    } BIT;
} volatile DMA_INT_STAT_REG;

/**
  * @brief DMA transfer completion interrupt status register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ch0_int_tc_stat : 1;    /**< Masked transfer completion interrupt status of channel 0. */
        unsigned int ch1_int_tc_stat : 1;    /**< Masked transfer completion interrupt status of channel 1. */
        unsigned int ch2_int_tc_stat : 1;    /**< Masked transfer completion interrupt status of channel 2. */
        unsigned int ch3_int_tc_stat : 1;    /**< Masked transfer completion interrupt status of channel 3. */
        unsigned int ch4_int_tc_stat : 1;    /**< Masked transfer completion interrupt status of channel 4. */
        unsigned int ch5_int_tc_stat : 1;    /**< Masked transfer completion interrupt status of channel 5. */
        unsigned int reserved0 : 26;
    } BIT;
} volatile DMA_INT_TC_STAT_REG;

/**
  * @brief DMA transfer completion interrupt clear register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ch0_int_tc_clr : 1;    /**< Clear the channel 0 transfer completion interrupt. */
        unsigned int ch1_int_tc_clr : 1;    /**< Clear the channel 1 transfer completion interrupt. */
        unsigned int ch2_int_tc_clr : 1;    /**< Clear the channel 2 transfer completion interrupt. */
        unsigned int ch3_int_tc_clr : 1;    /**< Clear the channel 3 transfer completion interrupt. */
        unsigned int ch4_int_tc_clr : 1;    /**< Clear the channel 4 transfer completion interrupt. */
        unsigned int ch5_int_tc_clr : 1;    /**< Clear the channel 5 transfer completion interrupt. */
        unsigned int reserved0 : 26;
    } BIT;
} volatile DMA_INT_TC_CLR_REG;

/**
  * @brief DMA error interrupt status register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ch0_int_err_stat : 1;    /**< Masked error interrupt status of channel 0. */
        unsigned int ch1_int_err_stat : 1;    /**< Masked error interrupt status of channel 1. */
        unsigned int ch2_int_err_stat : 1;    /**< Masked error interrupt status of channel 2. */
        unsigned int ch3_int_err_stat : 1;    /**< Masked error interrupt status of channel 3. */
        unsigned int ch4_int_err_stat : 1;    /**< Masked error interrupt status of channel 4. */
        unsigned int ch5_int_err_stat : 1;    /**< Masked error interrupt status of channel 5. */
        unsigned int reserved0 : 26;
    } BIT;
} volatile DMA_INT_ERR_STAT_REG;

/**
  * @brief DMA error interrupt clear register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ch0_int_err_clr : 1;    /**< Clear channel 0 error interrupt. */
        unsigned int ch1_int_err_clr : 1;    /**< Clear channel 1 error interrupt. */
        unsigned int ch2_int_err_clr : 1;    /**< Clear channel 2 error interrupt. */
        unsigned int ch3_int_err_clr : 1;    /**< Clear channel 3 error interrupt. */
        unsigned int ch4_int_err_clr : 1;    /**< Clear channel 4 error interrupt. */
        unsigned int ch5_int_err_clr : 1;    /**< Clear channel 5 error interrupt. */
        unsigned int reserved0 : 26;
    } BIT;
} volatile DMA_INT_ERR_CLR_REG;

/**
  * @brief DMA raw transfer completion interrupt register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ch0_raw_int_tc : 1;    /**< Raw transfer completion interrupt status of channel 0. */
        unsigned int ch1_raw_int_tc : 1;    /**< Raw transfer completion interrupt status of channel 1. */
        unsigned int ch2_raw_int_tc : 1;    /**< Raw transfer completion interrupt status of channel 2. */
        unsigned int ch3_raw_int_tc : 1;    /**< Raw transfer completion interrupt status of channel 3. */
        unsigned int ch4_raw_int_tc : 1;    /**< Raw transfer completion interrupt status of channel 4. */
        unsigned int ch5_raw_int_tc : 1;    /**< Raw transfer completion interrupt status of channel 5. */
        unsigned int reserved0 : 26;
    } BIT;
} volatile DMA_RAW_INT_TC_STAT_REG;

/**
  * @brief DMA raw error interrupt register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ch0_raw_int_err : 1;    /**< Raw error interrupt status of channel 0. */
        unsigned int ch1_raw_int_err : 1;    /**< Raw error interrupt status of channel 1. */
        unsigned int ch2_raw_int_err : 1;    /**< Raw error interrupt status of channel 2. */
        unsigned int ch3_raw_int_err : 1;    /**< Raw error interrupt status of channel 3. */
        unsigned int ch4_raw_int_err : 1;    /**< Raw error interrupt status of channel 4. */
        unsigned int ch5_raw_int_err : 1;    /**< Raw error interrupt status of channel 5. */
        unsigned int reserved0 : 26;
    } BIT;
} volatile DMA_RAW_INT_ERR_STAT_REG;

/**
  * @brief DMA channel enable status register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ch0_enabled : 1;    /**< Channel 0 enable status. */
        unsigned int ch1_enabled : 1;    /**< Channel 1 enable status. */
        unsigned int ch2_enabled : 1;    /**< Channel 2 enable status. */
        unsigned int ch3_enabled : 1;    /**< Channel 3 enable status. */
        unsigned int ch4_enabled : 1;    /**< Channel 4 enable status. */
        unsigned int ch5_enabled : 1;    /**< Channel 5 enable status. */
        unsigned int reserved0 : 26;
    } BIT;
} volatile DMA_ENABLED_CHNS_REG;

/**
  * @brief DMA parameter configuration register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int dma_enable : 1;    /**< DMA controller enable. */
        unsigned int reserved0 : 31;
    } BIT;
} volatile DMA_CONFIG_REG;


/**
  * @brief DMA request line synchronization enable.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int dma_sync_disable : 32;    /**< Control whether the request line needs to be synchronized.. */
    } BIT;
} volatile DMA_SYNC_REG;

/**
  * @brief Source address register of DMA channel n (n = 0, 1, 2, 3).
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int src_addr : 32;    /**< DMA source address. */
    } BIT;
} volatile DMA_Cn_SRC_ADDR_REG;

/**
  * @brief Destination address register of DMA channel n (n = 0, 1, 2, 3).
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int dest_addr : 32;    /**< DMA destination address. */
    } BIT;
} volatile DMA_Cn_DEST_ADDR_REG;

/**
  * @brief Linked list information register for DMA channel n (n = 0, 1, 2, 3).
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0 : 2;
        unsigned int ll_item : 30;     /**< Address of the next linked list node. */
    } BIT;
} volatile DMA_Cn_LLI_REG;

/**
  * @brief DMA channel n (n = 0, 1, 2, 3) control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int trans_size : 12;    /**< Length of the DMA transfer, provided that the DMA flow controller. */
        unsigned int sbsize : 3;         /**< Burst length of the source device. */
        unsigned int dbsize : 3;         /**< Burst length of the destination device. */
        unsigned int swidth : 3;         /**< Transfer bit width of the source device,
                                              which cannot be greater than Master bit width. */
        unsigned int dwidth : 3;         /**< Transfer bit width of the destination device,
                                              which cannot be greater than Master bit width. */
        unsigned int reserved0 : 2;
        unsigned int src_incr : 1;       /**< Set the incremental mode of the source address. */
        unsigned int dest_incr : 1;      /**< Set the incremental mode of the destination address. */
        unsigned int reserved1 : 3;
        unsigned int int_tc_enable : 1;  /**< Transfer completion interrupt enable. */
    } BIT;
} volatile DMA_Cn_CONTROL_REG;

/**
  * @brief DMA channel n (n = 0, 1, 2, 3) configuration register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ch_en : 1;          /**< Channel enable. */
        unsigned int src_periph : 5;     /**< Source device, ignore this field if memory device. */
        unsigned int dest_periph : 5;    /**< Destination device, ignore this field if memory device. */
        unsigned int flow_ctrl : 3;      /**< Flow control and transmission Type. */
        unsigned int err_int_msk : 1;    /**< Error interrupt mask flag. */
        unsigned int tc_int_msk : 1;     /**< Transfer completion interrupt mask flag. */
        unsigned int ch_lock : 1;        /**< Lock transmission enable on the bus. */
        unsigned int ch_active : 1;      /**< Whether the data in the channel FIFO. */
        unsigned int ch_halt : 1;        /**< Whether ignore DMA requests. */
        unsigned int reserved0 : 5;
        unsigned int ch_priority : 2;    /**< Channel Priority, larger value indicates a higher priority. */
        unsigned int reserved1 : 6;
    } BIT;
} volatile DMA_Cn_CONFIG_REG;

/**
  * @brief DMA register mapping structure.
  */
typedef struct {
    DMA_INT_STAT_REG           DMA_INT_STAT;          /**< DMA interrupt status register.
                                                             Offset address: 0x00000000U. */
    DMA_INT_TC_STAT_REG        DMA_INT_TC_STAT;       /**< DMA transfer completion interrupt status register.
                                                             Offset address: 0x00000004U. */
    DMA_INT_TC_CLR_REG         DMA_INT_TC_CLR;        /**< DMA transfer completion interrupt clear register.
                                                             Offset address: 0x00000008U. */
    DMA_INT_ERR_STAT_REG       DMA_INT_ERR_STAT;      /**< DMA error interrupt status register.
                                                             Offset address: 0x0000000CU. */
    DMA_INT_ERR_CLR_REG        DMA_INT_ERR_CLR;       /**< DMA error interrupt clear register.
                                                             Offset address: 0x00000010U. */
    DMA_RAW_INT_TC_STAT_REG    DMA_RAW_INT_TC_STAT;   /**< DMA raw transfer completion interrupt register.
                                                             Offset address: 0x00000014U. */
    DMA_RAW_INT_ERR_STAT_REG   DMA_RAW_INT_ERR_STAT;  /**< DMA raw error interrupt register.
                                                             Offset address: 0x00000018U. */
    DMA_ENABLED_CHNS_REG       DMA_ENABLED_CHNS;      /**< DMA channel enable status register.
                                                             Offset address: 0x0000001CU. */
    unsigned char space0[16];
    DMA_CONFIG_REG             DMA_CONFIG;            /**< DMA parameter configuration register.
                                                             Offset address: 0x00000030U. */
    DMA_SYNC_REG               DMA_SYNC;              /**< DMA request line synchronization enable.
                                                             Offset address: 0x00000034U. */
    unsigned char space1[200];
    DMA_Cn_SRC_ADDR_REG        DMA_C0_SRC_ADDR;       /**< Source address register of DMA channel 0.
                                                             Offset address: 0x00000100U. */
    DMA_Cn_DEST_ADDR_REG       DMA_C0_DEST_ADDR;      /**< Destination address register of DMA channel 0.
                                                             Offset address: 0x00000104U. */
    DMA_Cn_LLI_REG             DMA_C0_LLI;            /**< Linked list information register for DMA channel 0.
                                                             Offset address: 0x00000108U. */
    DMA_Cn_CONTROL_REG         DMA_C0_CONTROL;        /**< DMA channel 0 control register.
                                                             Offset address: 0x0000010CU. */
    DMA_Cn_CONFIG_REG          DMA_C0_CONFIG;         /**< DMA channel 0 configuration register.
                                                             Offset address: 0x00000110U. */
    unsigned char space2[12];
    DMA_Cn_SRC_ADDR_REG        DMA_C1_SRC_ADDR;       /**< Source address register of DMA channel 1.
                                                             Offset address: 0x00000120U. */
    DMA_Cn_DEST_ADDR_REG       DMA_C1_DEST_ADDR;      /**< Destination address register of DMA channel 1.
                                                             Offset address: 0x00000124U. */
    DMA_Cn_LLI_REG             DMA_C1_LLI;            /**< Linked list information register for DMA channel 1.
                                                             Offset address: 0x00000128U. */
    DMA_Cn_CONTROL_REG         DMA_C1_CONTROL;        /**< DMA channel 1 control register.
                                                             Offset address: 0x0000012CU. */
    DMA_Cn_CONFIG_REG          DMA_C1_CONFIG;         /**< DMA channel 1 configuration register.
                                                             Offset address: 0x00000130U. */
    unsigned char space3[12];
    DMA_Cn_SRC_ADDR_REG        DMA_C2_SRC_ADDR;       /**< Source address register of DMA channel 2.
                                                             Offset address: 0x00000140U. */
    DMA_Cn_DEST_ADDR_REG       DMA_C2_DEST_ADDR;      /**< Destination address register of DMA channel 2.
                                                             Offset address: 0x00000144U. */
    DMA_Cn_LLI_REG             DMA_C2_LLI;            /**< Linked list information register for DMA channel 2.
                                                             Offset address: 0x00000148U. */
    DMA_Cn_CONTROL_REG         DMA_C2_CONTROL;        /**< DMA channel 2 control register.
                                                             Offset address: 0x0000014CU. */
    DMA_Cn_CONFIG_REG          DMA_C2_CONFIG;         /**< DMA channel 2 configuration register.
                                                             Offset address: 0x00000150U. */
    unsigned char space4[12];
    DMA_Cn_SRC_ADDR_REG        DMA_C3_SRC_ADDR;       /**< Source address register of DMA channel 3.
                                                             Offset address: 0x00000160U. */
    DMA_Cn_DEST_ADDR_REG       DMA_C3_DEST_ADDR;      /**< Destination address register of DMA channel 3.
                                                             Offset address: 0x00000164U. */
    DMA_Cn_LLI_REG             DMA_C3_LLI;            /**< Linked list information register for DMA channel 3.
                                                             Offset address: 0x00000168U. */
    DMA_Cn_CONTROL_REG         DMA_C3_CONTROL;        /**< DMA channel 3 control register.
                                                             Offset address: 0x0000016CU. */
    DMA_Cn_CONFIG_REG          DMA_C3_CONFIG;         /**< DMA channel 3 configuration register.
                                                             Offset address: 0x00000170U. */
    unsigned char space5[12];
    DMA_Cn_SRC_ADDR_REG        DMA_C4_SRC_ADDR;       /**< Source address register of DMA channel 4.
                                                             Offset address: 0x00000180U. */
    DMA_Cn_DEST_ADDR_REG       DMA_C4_DEST_ADDR;      /**< Destination address register of DMA channel 4.
                                                             Offset address: 0x00000184U. */
    DMA_Cn_LLI_REG             DMA_C4_LLI;            /**< Linked list information register for DMA channel 4.
                                                             Offset address: 0x00000188U. */
    DMA_Cn_CONTROL_REG         DMA_C4_CONTROL;        /**< DMA channel 4 control register.
                                                             Offset address: 0x0000018CU. */
    DMA_Cn_CONFIG_REG          DMA_C4_CONFIG;         /**< DMA channel 4 configuration register.
                                                             Offset address: 0x00000190U. */
    unsigned char space6[12];
    DMA_Cn_SRC_ADDR_REG        DMA_C5_SRC_ADDR;       /**< Source address register of DMA channel 5.
                                                             Offset address: 0x00000200U. */
    DMA_Cn_DEST_ADDR_REG       DMA_C5_DEST_ADDR;      /**< Destination address register of DMA channel 5.
                                                             Offset address: 0x00000204U. */
    DMA_Cn_LLI_REG             DMA_C5_LLI;            /**< Linked list information register for DMA channel 5.
                                                             Offset address: 0x00000208U. */
    DMA_Cn_CONTROL_REG         DMA_C5_CONTROL;        /**< DMA channel 5 control register.
                                                             Offset address: 0x0000020CU. */
    DMA_Cn_CONFIG_REG          DMA_C5_CONFIG;         /**< DMA channel 5 configuration register.
                                                             Offset address: 0x00000210U. */
} volatile DMA_RegStruct;

/**
  * @brief Channel register mapping structure.
  */
typedef struct {
    DMA_Cn_SRC_ADDR_REG        DMA_Cn_SRC_ADDR;     /**< Source address register of DMA channel. */
    DMA_Cn_DEST_ADDR_REG       DMA_Cn_DEST_ADDR;    /**< Destination address register of DMA channel. */
    DMA_Cn_LLI_REG             DMA_Cn_LLI;          /**< Linked list information register for DMA channel. */
    DMA_Cn_CONTROL_REG         DMA_Cn_CONTROL;      /**< DMA channel control register. */
    DMA_Cn_CONFIG_REG          DMA_Cn_CONFIG;       /**< DMA channel configuration register. */
} volatile DMA_ChannelRegStruct;

/**
  * @brief DMA linked list structure.
  */
typedef struct _DMA_LinkList {
    unsigned int                srcAddr;     /**< Source device start address. */
    unsigned int                destAddr;    /**< Destination device start address. */
    struct _DMA_LinkList       *lliNext;     /**< Pointer to the next node. */
    DMA_Cn_CONTROL_REG         control;     /**< Channel parameters configured for the node. */
} DMA_LinkList;

/**
  * @brief A large amount of block data needs to be Splitd. Split functions need to transfer the following structure.
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
  * @}
  */


/**
  * @brief Check DMA channel num parameter.
  * @param channel The number of channel.
  * @retval bool
  */
static inline bool IsDmaChannelNum(DMA_ChannelNum channel)
{
    /* channel 0-5 */
    if ((channel == DMA_CHANNEL_ZERO) || (channel == DMA_CHANNEL_ONE) ||
        (channel == DMA_CHANNEL_TWO) || (channel == DMA_CHANNEL_THREE) ||
        (channel == DMA_CHANNEL_FOUR) || (channel == DMA_CHANNEL_FIVE)) {
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
  * @brief Check DMA channel priority.
  * @param priority DMA channel priority.
  * @retval bool
  */
static inline bool IsDmaPriority(DMA_ChannelPriority priority)
{
    if ((priority == DMA_PRIORITY_LOW) || (priority == DMA_PRIORITY_MEDIUM) ||
        (priority == DMA_PRIORITY_HIGH) || (priority == DMA_PRIORITY_HIGHEST)) {
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
    return (reqPeriph >= DMA_REQUEST_I2C0_RX) && (reqPeriph <= DMA_REQUEST_MEM);
}

/**
  * @brief Check whether the address is valid.
  * @param address Address for the DMA to transfer data.
  * @retval bool
  */
static inline bool IsDmaValidAddress(unsigned int address)
{
    return (address >= SRAM_START && address <= SRAM_END) || (address >= REGISTER_START && address <= REGISTER_END);
}

/**
  * @brief DMA configurate the direction.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_SetDirection(DMA_ChannelRegStruct * const dmaChannelx, DMA_TransDirection direction)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(IsDmaDirection(direction));
    dmaChannelx->DMA_Cn_CONFIG.BIT.flow_ctrl = direction;
}

/**
  * @brief DMA configurate the address of source.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_SetSrcAddr(DMA_ChannelRegStruct * const dmaChannelx, unsigned int srcAddr)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(srcAddr > 0);
    dmaChannelx->DMA_Cn_SRC_ADDR.BIT.src_addr = srcAddr;
}

/**
  * @brief DMA configurate the address of destnation.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_SetDestAddr(DMA_ChannelRegStruct * const dmaChannelx, unsigned int destAddr)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(destAddr > 0);
    dmaChannelx->DMA_Cn_DEST_ADDR.BIT.dest_addr = destAddr;
}

/**
  * @brief DMA configurate the address mode of source.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_SetSrcAddrMode(DMA_ChannelRegStruct * const dmaChannelx, DMA_AddrIncMode srcAddrInc)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(IsDmaAddrMode(srcAddrInc));
    dmaChannelx->DMA_Cn_CONTROL.BIT.src_incr = srcAddrInc;
}

/**
  * @brief DMA configurate the address mode of destnation.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_SetDestAddrMode(DMA_ChannelRegStruct * const dmaChannelx, DMA_AddrIncMode destAddrInc)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(IsDmaAddrMode(destAddrInc));
    dmaChannelx->DMA_Cn_CONTROL.BIT.dest_incr = destAddrInc;
}

/**
  * @brief DMA configurate the bit width of source.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_SetSrcWidth(DMA_ChannelRegStruct * const dmaChannelx, DMA_TransmisWidth srcWidth)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(IsDmaWidth(srcWidth));
    dmaChannelx->DMA_Cn_CONTROL.BIT.swidth = srcWidth;
}

/**
  * @brief DMA configurate the bit width of destnation.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_SetDestWidth(DMA_ChannelRegStruct * const dmaChannelx, DMA_TransmisWidth destWidth)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(IsDmaWidth(destWidth));
    dmaChannelx->DMA_Cn_CONTROL.BIT.dwidth = destWidth;
}

/**
  * @brief DMA configurate the burst size of source.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_SetSrcBurst(DMA_ChannelRegStruct * const dmaChannelx, DMA_BurstLength srcBurst)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(IsDmaBurstLength(srcBurst));
    dmaChannelx->DMA_Cn_CONTROL.BIT.sbsize = srcBurst;
}

/**
  * @brief DMA configurate the burst size of source.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_SetDestBurst(DMA_ChannelRegStruct * const dmaChannelx, DMA_BurstLength destBurst)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(IsDmaBurstLength(destBurst));
    dmaChannelx->DMA_Cn_CONTROL.BIT.dbsize = destBurst;
}

/**
  * @brief DMA configurate the transfer size.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_SetTransferSize(DMA_ChannelRegStruct * const dmaChannelx, unsigned int dataLength)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    DMA_PARAM_CHECK_NO_RET(dataLength <= 0xFFF);
    dmaChannelx->DMA_Cn_CONTROL.BIT.trans_size = dataLength;
}

/**
  * @brief Enable channel completion interrupt.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_EnableIT(DMA_ChannelRegStruct * const dmaChannelx)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    dmaChannelx->DMA_Cn_CONTROL.BIT.int_tc_enable = BASE_CFG_ENABLE;
    dmaChannelx->DMA_Cn_CONFIG.BIT.tc_int_msk = BASE_CFG_ENABLE;
}

/**
  * @brief Disable channel completion interrupt.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_DisableIT(DMA_ChannelRegStruct * const dmaChannelx)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    dmaChannelx->DMA_Cn_CONFIG.BIT.tc_int_msk = BASE_CFG_DISABLE;
}

/**
  * @brief Enables the channel to start transmission.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_EnableChannel(DMA_ChannelRegStruct * const dmaChannelx)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    dmaChannelx->DMA_Cn_CONFIG.BIT.ch_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable the channel to start transmission.
  * @param dmaChannelx DMA channel register base address.
  * @retval None.
  */
static inline void DCL_DMA_DisableChannel(DMA_ChannelRegStruct * const dmaChannelx)
{
    DMA_ASSERT_PARAM(IsDMACHXInstance(dmaChannelx));
    dmaChannelx->DMA_Cn_CONFIG.BIT.ch_en = BASE_CFG_DISABLE;
}
/**
  * @}
  */

/**
  * @}
  */
#endif  /* McuMagicTag_DMA_IP_H */