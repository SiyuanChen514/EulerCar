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
  * @file    uart_ip.h
  * @author  MCU Driver Team
  * @brief   UART module driver
  * @details This file provides DCL functions to manage UART and Definition of
  *          specific parameters.
  *          + Definition of UART configuration parameters.
  *          + UART register mapping structure.
  *          + Parameters check functions.
  *          + Direct configuration layer interface.
  */

/* Macro definitions */
#ifndef McuMagicTag_UART_IP_H
#define McuMagicTag_UART_IP_H

#include "baseinc.h"

#ifdef UART_PARAM_CHECK
#define UART_ASSERT_PARAM BASE_FUNC_ASSERT_PARAM
#define UART_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define UART_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define UART_ASSERT_PARAM(para) ((void)0U)
#define UART_PARAM_CHECK_NO_RET(para) ((void)0U)
#define UART_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif

#define UART_FIFOFULL_ONE_TWO 0x0000000FU
#define UART_FIFOFULL_ONE_EIGHT 0x00000008U

/**
  * @addtogroup UART
  * @{
  */

/**
  * @defgroup UART_IP UART_IP
  * @brief UART_IP: uart_v1
  * @{
  */

/**
  * @defgroup UART_Param_Def UART Parameters Definition
  * @brief Definition of UART configuration parameters.
  * @{
  */

/**
 * @brief UART def of oversampling
 */
typedef enum {
    UART_OVERSAMPLING_16X = 0x00000000U,
    UART_OVERSAMPLING_15X = 0x00000001U,
    UART_OVERSAMPLING_14X = 0x00000002U,
    UART_OVERSAMPLING_13X = 0x00000003U,
    UART_OVERSAMPLING_12X = 0x00000004U,
} UART_OversampleMultiple;

/**
 * @brief Extent handle definition of UART
 */
typedef struct {
    UART_OversampleMultiple     overSampleMultiple;   /**< Oversampling multiplier  */
    bool                        msbFirst;             /**< Configures data transmission sequence */
} UART_ExtendHandle;

/**
  * @brief Type ID of callback function registered by the user.
  */
typedef enum {
    UART_WRITE_IT_FINISH    =   0x00000000U,
    UART_READ_IT_FINISH     =   0x00000001U,
    UART_WRITE_DMA_FINISH   =   0x00000002U,
    UART_READ_DMA_FINISH    =   0x00000003U,
    UART_TRNS_IT_ERROR      =   0x00000004U,
    UART_TRNS_DMA_ERROR     =   0x00000005U,
    UART_BAUD_DETECT_FINISH =   0x00000006U,
    UART_BAUD_DETECT_ERROR  =   0x00000007U,
    UART_CHARACTER_MATCH    =   0x00000008U
} UART_CallbackFun_Type;

/**
 * @brief User Callback Function Definition
 */
typedef struct {
    void (* WriteItFinishCallBack)(void *handle);             /**< UART tx interrupt complete callback
                                                                                 function for users */
    void (* ReadItFinishCallBack)(void *handle);              /**< UART rx interrupt complete callback
                                                                                 function for users */
    void (* WriteDmaFinishCallBack)(void *handle);            /**< UART tx  DMA complete callback function
                                                                                 for users */
    void (* ReadDmaFinishCallBack)(void *handle);             /**< UART rx DMA complete callback function
                                                                                 for users */
    void (* TransmitItErrorCallBack)(void *handle);           /**< UART interrupt mode error callback
                                                                                 function for users */
    void (* TransmitDmaErrorCallBack)(void *handle);          /**< UART interrupt mode error callback
                                                                                 function for users */
    void (* BaudDetectSuccessCallBack)(void *handle);         /**< Callback function for successful
                                                                            UART baud rate detection */
    void (* BaudDetectErrorCallBack)(void *handle);           /**< UART baud rate detection failure
                                                                                   callback function */
    void (* CharacterMatchCallBack)(void *handle);            /**< UART character matching callback
                                                                                           function. */
}UART_UserCallBack;

/**
  * @brief Type of error callback functuions.
  */
typedef enum {
    UART_ERROR_FRAME = 0x00000080U,
    UART_ERROR_PARITY = 0x00000100U,
    UART_ERROR_BREAK = 0x00000200U,
    UART_ERROR_OVERFLOW = 0x00000400U
} UART_Error_Type;

/**
  * @brief The number of data bits transmitted or received in a frame.
  */
typedef enum {
    UART_DATALENGTH_5BIT = 0x00000000U,
    UART_DATALENGTH_6BIT = 0x00000001U,
    UART_DATALENGTH_7BIT = 0x00000002U,
    UART_DATALENGTH_8BIT = 0x00000003U
} UART_DataLength;

/**
  * @brief UART parity mode.
  * @details parity mode:
  *          + UART_PARITY_ODD   -- odd check
  *          + UART_PARITY_EVEN  -- even check
  *          + UART_PARITY_NONE  -- none odd or even check
  *          + UART_PARITY_MARK  -- mark check
  *          + UART_PARITY_SPACE -- space check
  */
typedef enum {
    UART_PARITY_ODD = 0x00000000U,
    UART_PARITY_EVEN = 0x00000001U,
    UART_PARITY_MARK = 0x00000002U,
    UART_PARITY_SPACE = 0x00000003U,
    UART_PARITY_NONE = 0x00000004U
} UART_Parity_Mode;

/**
  * @brief Stop bit setting.
  * @details Stop bit type:
  *          + UART_STOPBITS_ONE -- frame with one stop bit
  *          + UART_STOPBITS_TWO -- frame with two stop bits
  */
typedef enum {
    UART_STOPBITS_ONE = 0x00000000U,
    UART_STOPBITS_TWO = 0x00000001U
} UART_StopBits;

/**
  * @brief Three transmit mode: blocking, DMA, interrupt.
  */
typedef enum {
    UART_MODE_BLOCKING = 0x00000000U,
    UART_MODE_INTERRUPT = 0x00000001U,
    UART_MODE_DMA = 0x00000002U,
    UART_MODE_DISABLE = 0x00000003U
} UART_Transmit_Mode;

/**
  * @brief Hardware flow control mode disable/enable.
  */
typedef enum {
    UART_HW_FLOWCTR_DISABLE = 0x00000000U,
    UART_HW_FLOWCTR_ENABLE = 0x00000001U
} UART_HW_FlowCtr;

/**
  * @brief UART running status: deinit, ready, busy, busy(TX), busy(RX).
  */
typedef enum {
    UART_STATE_NONE_INIT = 0x00000000U,
    UART_STATE_READY = 0x00000001U,
    UART_STATE_BUSY = 0x00000002U,
    UART_STATE_BUSY_TX = 0x00000003U,
    UART_STATE_BUSY_RX = 0x00000004U,
} UART_State_Type;

/**
  * @brief UART RX/TX FIFO line interrupt threshold. An interrupt is triggered when the received or discovered data
  * crosses the FIFO threshold.
  * @details Description:
  *          + UART_FIFODEPTH_SIZE0     --   rxFIFO >= 0 Bytes,   txFIFO <= 0 Bytes
  *          + UART_FIFODEPTH_SIZE1     --   rxFIFO >= 1 Bytes,   txFIFO <= 1 Bytes
  *          + UART_FIFODEPTH_SIZE2     --   rxFIFO >= 2 Bytes,   txFIFO <= 2 Bytes
  *          + UART_FIFODEPTH_SIZE3     --   rxFIFO >= 3 Bytes,   txFIFO <= 3 Bytes
  *          + UART_FIFODEPTH_SIZE4     --   rxFIFO >= 4 Bytes,   txFIFO <= 4 Bytes
  *          + UART_FIFODEPTH_SIZE5     --   rxFIFO >= 5 Bytes,   txFIFO <= 5 Bytes
  *          + UART_FIFODEPTH_SIZE6     --   txFIFO <= 6 Bytes,   txFIFO <= 6 Bytes
  *          + UART_FIFODEPTH_SIZE7     --   txFIFO <= 7 Bytes,   txFIFO <= 7 Bytes
  *          + UART_FIFODEPTH_SIZE8     --   txFIFO <= 8 Bytes,   txFIFO <= 8 Bytes
  *          + UART_FIFODEPTH_SIZE9     --   txFIFO <= 9 Bytes,   txFIFO <= 9 Bytes
  *          + UART_FIFODEPTH_SIZE10    --   txFIFO <= 10 Bytes,  txFIFO <= 10 Bytes
  *          + UART_FIFODEPTH_SIZE11    --   txFIFO <= 11 Bytes,  txFIFO <= 11 Bytes
  *          + UART_FIFODEPTH_SIZE12    --   txFIFO <= 12 Bytes,  txFIFO <= 12 Bytes
  *          + UART_FIFODEPTH_SIZE13    --   txFIFO <= 13 Bytes,  txFIFO <= 13 Bytes
  *          + UART_FIFODEPTH_SIZE14    --   txFIFO <= 14 Bytes,  txFIFO <= 14 Bytes
  *          + UART_FIFODEPTH_SIZE15    --   txFIFO <= 15 Bytes,  txFIFO <= 15 Bytes
  */
typedef enum {
    UART_FIFODEPTH_SIZE0  = 0x00000000U,
    UART_FIFODEPTH_SIZE1  = 0x00000001U,
    UART_FIFODEPTH_SIZE2  = 0x00000002U,
    UART_FIFODEPTH_SIZE3  = 0x00000003U,
    UART_FIFODEPTH_SIZE4  = 0x00000004U,
    UART_FIFODEPTH_SIZE5  = 0x00000005U,
    UART_FIFODEPTH_SIZE6  = 0x00000006U,
    UART_FIFODEPTH_SIZE7  = 0x00000007U,
    UART_FIFODEPTH_SIZE8  = 0x00000008U,
    UART_FIFODEPTH_SIZE9  = 0x00000009U,
    UART_FIFODEPTH_SIZE10 = 0x0000000AU,
    UART_FIFODEPTH_SIZE11 = 0x0000000BU,
    UART_FIFODEPTH_SIZE12 = 0x0000000CU,
    UART_FIFODEPTH_SIZE13 = 0x0000000DU,
    UART_FIFODEPTH_SIZE14 = 0x0000000EU,
    UART_FIFODEPTH_SIZE15 = 0x0000000FU
} UART_FIFO_Threshold;

/**
  * @brief UART data transfer sequence.
  */
typedef enum {
    UART_SEQUENCE_START_LSB = 0x00000000U,
    UART_SEQUENCE_START_MSB = 0x00000001U,
} UART_SequenceMode;

/**
  * @}
  */

/**
  * @defgroup UART_Reg_Def UART Register Definition
  * @brief register mapping structure
  * @{
  */

/**
  * @brief UART data register, which stores the RX data and TX data and reads the RX status from this register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int data      :   8;   /**< Receives data and transmits data. */
        unsigned int fe        :   1;   /**< Frame error. */
        unsigned int pe        :   1;   /**< Verification error. */
        unsigned int be        :   1;   /**< Break error. */
        unsigned int oe        :   1;   /**< Overflow error. */
        unsigned int reserved0 :   20;
    } BIT;
} volatile UART_DR_REG;

/**
  * @brief Receive status register/error clear register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int fe        : 1;    /**< Frame error. */
        unsigned int pe        : 1;    /**< Verification error. */
        unsigned int be        : 1;    /**< Break error. */
        unsigned int oe        : 1;    /**< Overflow error. */
        unsigned int reserved0 : 28;
    } BIT;
} volatile UART_RSR_REG;

/**
  * @brief UART flag register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cts       : 1;   /**< Hardware flow control status. */
        unsigned int reserved0 : 2;
        unsigned int busy      : 1;   /**< UART busy/idle status bit. */
        unsigned int rxfe      : 1;   /**< RX FIFO empty flag. */
        unsigned int txff      : 1;   /**< TX FIFO full flag. */
        unsigned int rxff      : 1;   /**< RX FIFO full flag. */
        unsigned int txfe      : 1;   /**< TX FIFO empty flag. */
        unsigned int reserved1 : 24;
    } BIT;
} volatile UART_FR_REG;

/**
  * @brief Integer baud rate register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int bauddivint : 16;  /**< Integer baud rate divider value. */
        unsigned int reserved0  : 16;
    } BIT;
} volatile UART_IBRD_REG;

/**
  * @brief Fractional baud rate register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int bauddivfrac : 6; /**< Fractional baud rate divider. */
        unsigned int reserved0   : 26;
    } BIT;
} volatile UART_FBRD_REG;

/**
  * @brief Line control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int brk       : 1;  /**< Send a break. */
        unsigned int pen       : 1;  /**< Check select bit. */
        unsigned int eps       : 1;  /**< Parity check selection during transmission and reception. */
        unsigned int stp2      : 1;  /**< TX frame tail stop bit select. */
        unsigned int fen       : 1;  /**< TX and RX FIFO enable control. */
        unsigned int wlen      : 2;  /**< Indicates the number of transmitted and received data bits in a frame. */
        unsigned int sps       : 1;  /**< Select stick parity. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile UART_LCR_H_REG;

/**
  * @brief UART_CR is a UART control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int uarten    : 1;  /**< UART enable. */
        unsigned int reserved0 : 6;
        unsigned int lbe       : 1;  /**< Indicates whether to enable loopback. */
        unsigned int txe       : 1;  /**< UART TX enable. */
        unsigned int rxe       : 1;  /**< UART RX enable. */
        unsigned int dtr       : 1;  /**< UART time run. */
        unsigned int rts       : 1;  /**< Request to send. */
        unsigned int reserved1 : 2;
        unsigned int rtsen     : 1;  /**< RTS hardware flow control enable. */
        unsigned int ctsen     : 1;  /**< CTS hardware flow control enable. */
        unsigned int reserved2 : 16;
    } BIT;
} volatile UART_CR_REG;

/**
  * @brief Interrupt FIFO threshold select register.
  *        It is used to set the FIFO interrupt trigger threshold (UART_TXinTR or UART_RXinTR).
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int txiflsel  : 4;  /**< Configure the threshold of the TX FIFO. */
        unsigned int reserved0 : 4;
        unsigned int rxiflsel  : 4;  /**< RX FIFO threshold. */
        unsigned int reserved1 : 20;
    } BIT;
} volatile UART_IFLS_REG;

/**
  * @brief Interrupt mask register, which is used to mask interrupts.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0 : 1;
        unsigned int ctsmim    : 1;  /**< Mask status of the CTS interrupt. */
        unsigned int reserved1 : 2;
        unsigned int rxim      : 1;  /**< Mask status of the RX interrupt. */
        unsigned int txim      : 1;  /**< Mask status of the TX interrupt. */
        unsigned int rtim      : 1;  /**< Mask status of the RX timeout interrupt. */
        unsigned int feim      : 1;  /**< Mask status of the frame error interrupt. */
        unsigned int peim      : 1;  /**< Mask status of the parity interrupt. */
        unsigned int beim      : 1;  /**< Mask status of the break error interrupt. */
        unsigned int oeim      : 1;  /**< Mask status of the overflow error interrupt. */
        unsigned int reserved2 : 1;
        unsigned int txfeim    : 1;  /**< Mask status of the TX FIFO empty interrupt. */
        unsigned int txfneim   : 1;  /**< Mask status of the TX FIFO non-empt interrupt. */
        unsigned int txtcim    : 1;  /**< Mask status of the TX completion interrupt. */
        unsigned int reserved3 : 1;
        unsigned int rxfeim    : 1;  /**< Mask status of the RX FIFO empty interrupt. */
        unsigned int rxfneim   : 1;  /**< Mask status of the RX FIFO non-empt interrupt. */
        unsigned int rxffim    : 1;  /**< Mask status of the RX FIFO full interrupt. */
        unsigned int abdcim    : 1;  /**< Mask status of the auto-baud check completion interrupt. */
        unsigned int abdeim    : 1;  /**< Mask status of auto-baud detection error interrupts. */
        unsigned int cmim      : 1;  /**< Mask status of the character match success interrupt. */
        unsigned int reserved4 : 10;
    } BIT;
} volatile UART_IMSC_REG;

/**
  * @brief Raw interrupt status register. The content of this register is not affected by interrupt mask register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0  : 1;
        unsigned int ctsmis     : 1;  /**< Raw CTS interrupt status. */
        unsigned int reserved1  : 2;
        unsigned int rxris      : 1;  /**< Raw RX threshold interrupt status. */
        unsigned int txris      : 1;  /**< Original TX threshold interrupt status. */
        unsigned int rtris      : 1;  /**< Raw RX timeout interrupt status. */
        unsigned int feris      : 1;  /**< Raw frame error interrupt status. */
        unsigned int peris      : 1;  /**< Raw parity interrupt status. */
        unsigned int beris      : 1;  /**< Raw break error interrupt status. */
        unsigned int oeris      : 1;  /**< Raw overflow error interrupt status. */
        unsigned int reserved2  : 1;
        unsigned int txferis    : 1;  /**< Original TX FIFO empty interrupt status. */
        unsigned int txfneris   : 1;  /**< Raw TX FIFO non-empty interrupt status. */
        unsigned int txtcris    : 1;  /**< Raw TX completion interrupt status. */
        unsigned int reserved3  : 1;
        unsigned int rxferis    : 1;  /**< Raw RX FIFO empty interrupt status. */
        unsigned int rxfneris   : 1;  /**< Raw RX FIFO non-empty interrupt status. */
        unsigned int rxffris    : 1;  /**< Status of the raw RX FIFO full interrupt. */
        unsigned int abdcris    : 1;  /**< Raw auto-baud detection completion interrupt status. */
        unsigned int abderis    : 1;  /**< Raw auto-baud detection error interrupt status. */
        unsigned int cmris      : 1;  /**< Status of the original character matching success interrupt. */
        unsigned int reserved4  : 10;
    } BIT;
} volatile UART_RIS_REG;

/**
  * @brief Masked interrupt status register.
  *        It is result of AND operation between raw interrupt status and interrupt mask.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0 : 1;
        unsigned int ctsmmis   : 1;   /**< Masked CTS interrupt status. */
        unsigned int reserved1 : 2;
        unsigned int rxmis     : 1;   /**< Masked RX interrupt status. */
        unsigned int txmis     : 1;   /**< Masked TX interrupt status. */
        unsigned int rtmis     : 1;   /**< Masked RX timeout interrupt status. */
        unsigned int femis     : 1;   /**< Status of masked frame error interrupts. */
        unsigned int pemis     : 1;   /**< Masked parity interrupt status. */
        unsigned int bemis     : 1;   /**< Status of masked break error interrupts. */
        unsigned int oemis     : 1;   /**< Masked overflow error interrupt status. */
        unsigned int reserved2 : 1;
        unsigned int txfeis    : 1;   /**< Masked TX FIFO empty interrupt status. */
        unsigned int txfneis   : 1;   /**< Status of the masked TX FIFO non-empty interrupt. */
        unsigned int txtcis    : 1;   /**< Masked TX completion interrupt status. */
        unsigned int reserved3 : 1;
        unsigned int rxfeis    : 1;   /**< Masked RX FIFO empty interrupt status. */
        unsigned int rxfneis   : 1;   /**< Status of the masked RX FIFO non-empt interrupt. */
        unsigned int rxffis    : 1;   /**< Status of the masked RX FIFO full interrupt. */
        unsigned int abdcis    : 1;   /**< Status of the masked auto-baud check completion interrupt. */
        unsigned int abdeis    : 1;   /**< Status of masked auto-baud detection error interrupts. */
        unsigned int cmis      : 1;   /**< Masked character matching success interrupt status. */
        unsigned int reserved4 : 10;
    } BIT;
} volatile UART_MIS_REG;

/**
  * @brief Interrupt clear register
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0 : 1;
        unsigned int ctsmic    : 1;   /**< Clears the CTS interrupt. */
        unsigned int reserved1 : 2;
        unsigned int rxic      : 1;   /**< Clears the RX interrupt. */
        unsigned int txic      : 1;   /**< Clears the TX interrupt. */
        unsigned int rtic      : 1;   /**< Receive timeout interrupt clear. */
        unsigned int feic      : 1;   /**< Frame error interrupt clear. */
        unsigned int peic      : 1;   /**< Clears the parity interrupt. */
        unsigned int beic      : 1;   /**< Clears the break error interrupt. */
        unsigned int oeic      : 1;   /**< Clears the overflow error interrupt. */
        unsigned int reserved2 : 1;
        unsigned int txfeic    : 1;   /**< Clears the TX FIFO empty interrupt status. */
        unsigned int txfneic   : 1;   /**< TX FIFO non-empty interrupt clear status. */
        unsigned int txtcic    : 1;   /**< Transmit completion interrupt clear status. */
        unsigned int reserved3 : 1;
        unsigned int rxfeic    : 1;   /**< RX FIFO empty interrupt clear status. */
        unsigned int rxfneic   : 1;   /**< RX FIFO non-empty interrupt clear status. */
        unsigned int rxffic    : 1;   /**< RX FIFO full interrupt clear status. */
        unsigned int abdcic    : 1;   /**< Auto-baud detection completion interrupt clear status. */
        unsigned int abdeic    : 1;   /**< Auto-baud detection error interrupt clear status. */
        unsigned int cmic      : 1;   /**< Clears the character matching success interrupt. */
        unsigned int reserved4 : 10;
    } BIT;
} volatile UART_ICR_REG;

/**
  * @brief DMA control register, which is used to enable DMA of TX FIFO and RX FIFO.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int rxdmae        : 1; /** DMA enable control for the RX FIFO. */
        unsigned int txdmae        : 1; /** DMA enable control for the TX FIFO. */
        unsigned int dmaonerr      : 1; /** DMA enable control for RX channel when UART error interrupt occurs. */
        unsigned int rxlastsreq_en : 1; /** REQ enable for the last data stream supported by the UART RX DMA. */
        unsigned int reserved0     : 28;
    } BIT;
} volatile UART_DMACR_REG;

/**
  * @brief Data transfer sequence configuration register. It is used to configure data transfer sequence.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int msbfirst   : 1;   /**< Most significant bit before enable. */
        unsigned int reserved0  : 31;
    } BIT;
} volatile UART_DS_REG;

/**
  * @brief RX timeout duration configuration register, which is used to configure conditions for determining RX timeout.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int rtcfg     : 24;     /**< Indicates the receive timeout interval, in bits. */
        unsigned int reserved0 : 8;
    } BIT;
} volatile UART_RTCFG_REG;

/**
  * @brief Oversampling configuration register. It is used to configure the oversampling multiple.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int spcfg     : 4;    /**< Configure the oversampling multiplier. */
        unsigned int reserved0 : 28;
    } BIT;
} volatile UART_SPCFG_REG;

/**
  * @brief Auto-baud detection enable register. It is used to enable auto-baud detection function.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int abden     : 1;  /**< Auto-baud detection enable. */
        unsigned int reserved0 : 3;
        unsigned int abdbusy   : 1;  /**< Auto-baud detection busy flag. */
        unsigned int abdenvld  : 1;  /**< The abden sign is already valid. */
        unsigned int reserved1 : 26;
    } BIT;
} volatile UART_ABDEN_REG;

/**
  * @brief Character match configuration register, which is used to configure characters to be matched.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int chamat    : 8;   /**< Binary character to be matched. */
        unsigned int reserved0 : 23;
        unsigned int cmen      : 1;   /**< Character match detection enable. */
    } BIT;
} volatile UART_CHARMATCH_REG;

/**
  * @brief Register mapping structure.
  */
typedef struct {
    UART_DR_REG         UART_DR;        /**< Data register, offset address: 0x00000000U */
    UART_RSR_REG        UART_RSR;       /**< Receiving status/error clearing register, offset address: 0x00000004U */
    unsigned char       space0[16];
    UART_FR_REG         UART_FR;        /**< Flag register, offset address: 0x00000018U */
    unsigned char       space1[8];
    UART_IBRD_REG       UART_IBRD;      /**< Integer baud rate register, offset address: 0x00000024U */
    UART_FBRD_REG       UART_FBRD;      /**< Fractional baud rate register, offset address: 0x00000028U */
    UART_LCR_H_REG      UART_LCR_H;     /**< Wire control register, offset address: 0x0000002CU */
    UART_CR_REG         UART_CR;        /**< Control register, offset address: 0x00000030U */
    UART_IFLS_REG       UART_IFLS;      /**< Interrupt FIFO threshold register, offset address: 0x00000034U */
    UART_IMSC_REG       UART_IMSC;      /**< Interrupt mask status register, offset address: 0x00000038U */
    UART_RIS_REG        UART_RIS;       /**< Raw interrupt status register, offset address: 0x0000003CU */
    UART_MIS_REG        UART_MIS;       /**< Masked interrupt status register, offset address: 0x00000040U */
    UART_ICR_REG        UART_ICR;       /**< Interrupt clear register, offset address: 0x00000044U */
    UART_DMACR_REG      UART_DMACR;     /**< DMA control register register, offset address: 0x00000048U */
    unsigned char       space2[4];
    UART_DS_REG         UART_DS;        /**< Data transfer sequence set register, offset address: 0x00000050U */
    UART_RTCFG_REG      UART_RTCFG;     /**< RX timeout duration configuration register, offset address: 0x00000054U */
    UART_SPCFG_REG      UART_SPCFG;     /**< Oversampling configuration register, offset address: 0x00000058U */
    UART_ABDEN_REG      UART_ABDEN;     /**< Auto-baud detection enable register, offset address: 0x0000005CU */
    UART_CHARMATCH_REG  UART_CHARMATCH; /**< Character match configuration register, offset address: 0x00000060U */
} volatile UART_RegStruct;
/**
  * @}
  */

/**
  * @brief Check UART datalength parameter.
  * @param datalength The number of data bits in a frame, @ref UART_DataLength
  * @retval bool
  */
static inline bool IsUartDatalength(UART_DataLength datalength)
{
    return (datalength >= UART_DATALENGTH_5BIT) && (datalength <= UART_DATALENGTH_8BIT);
}

/**
  * @brief Check UART stopbits parameter.
  * @param stopbits The number of stop bits in a frame, @ref UART_StopBits
  * @retval bool
  */
static inline bool IsUartStopbits(UART_StopBits stopbits)
{
    return (stopbits == UART_STOPBITS_ONE) || (stopbits == UART_STOPBITS_TWO);
}

/**
  * @brief Check UART paritymode parameter.
  * @param paritymode UART parity check mode, @ref UART_Parity_Mode
  * @retval bool
  */
static inline bool IsUartParitymode(UART_Parity_Mode paritymode)
{
    if ((paritymode >= UART_PARITY_ODD) && (paritymode <= UART_PARITY_NONE)) {
        return true;
    }
    return false;
}

/**
  * @brief Check UART transmode parameter.
  * @param transmode Transmit mode, @ref UART_Transmit_Mode
  * @retval bool
  */
static inline bool IsUartTransmode(UART_Transmit_Mode transmode)
{
    if ((transmode == UART_MODE_BLOCKING) ||
        (transmode == UART_MODE_INTERRUPT) ||
        (transmode == UART_MODE_DMA) ||
        (transmode == UART_MODE_DISABLE)) {
        return true;
    }
    return false;
}

/**
  * @brief Check UART fifoThreshold parameter.
  * @param fifoThreshold UART TX/RX FIFO line interrupt threshold, @ref UART_FIFO_Threshold
  * @retval bool
  */
static inline bool IsUartFIFOThreshold(UART_FIFO_Threshold fifoThreshold)
{
    return (fifoThreshold >= UART_FIFODEPTH_SIZE0) && (fifoThreshold <= UART_FIFODEPTH_SIZE15);
}


/**
  * @brief Check UART Oversampling multiple.
  * @param multiple Oversampling multiple, @ref UART_OversampleMultiple
  * @retval bool
  */
static inline bool IsUartOversampleMultiple(UART_OversampleMultiple multiple)
{
    return (multiple >= UART_OVERSAMPLING_16X) && (multiple <= UART_OVERSAMPLING_12X);
}

/**
  * @brief Check UART data transfer sequential mode.
  * @param mode UART TX/RX sequential mode, @ref UART_SequenceMode
  * @retval bool
  */
static inline bool IsUartSequenceMode(UART_SequenceMode mode)
{
    return (mode == UART_SEQUENCE_START_LSB) || (mode == UART_SEQUENCE_START_MSB);
}

/* Direct configuration layer */
/**
  * @brief Send a character by UART
  * @param uartx UART register base address.
  * @param data Character to be sent.
  * @retval None.
  */
static inline void DCL_UART_WriteData(UART_RegStruct * const uartx, unsigned char data)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_DR.BIT.data = data;
}

/**
  * @brief Receive a character from UART.
  * @param uartx UART register base address.
  * @retval Data, read the received data from the UART data register.
  */
static inline unsigned char DCL_UART_ReadData(const UART_RegStruct *uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_DR.BIT.data;
}

/**
  * @brief UART TX enable.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_WriteEnable(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_CR.BIT.txe = BASE_CFG_ENABLE;
}

/**
  * @brief UART TX disable.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_WriteDisable(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_CR.BIT.txe = BASE_CFG_DISABLE;
}

/**
  * @brief UART RX enable.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_ReadEnable(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_CR.BIT.rxe = BASE_CFG_ENABLE;
}

/**
  * @brief UART RX disable.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_ReadDisable(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_CR.BIT.rxe = BASE_CFG_DISABLE;
}

/**
  * @brief UART TX use DMA .
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_DMA_WriteEnable(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_DMACR.BIT.txdmae = BASE_CFG_ENABLE;
}

/**
  * @brief UART TX cannot use DMA .
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_DMA_WriteDisable(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_DMACR.BIT.txdmae = BASE_CFG_DISABLE;
}

/**
  * @brief UART RX use DMA .
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_DMA_ReadEnable(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_DMACR.BIT.rxdmae = BASE_CFG_ENABLE;
}

/**
  * @brief UART RX cannot use DMA .
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_DMA_ReadDisable(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_DMACR.BIT.rxdmae = BASE_CFG_DISABLE;
}

/**
  * @brief UART word length setting.
  * @param uartx UART register base address.
  * @param dataLength Word length of sending and receiving, @ref UART_DataLength
  * @retval None.
  */
static inline void DCL_UART_SetDataLength(UART_RegStruct * const uartx, UART_DataLength dataLength)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    UART_PARAM_CHECK_NO_RET(IsUartDatalength(dataLength));
    uartx->UART_LCR_H.BIT.wlen = dataLength;
}

/**
  * @brief Gettintg UART word length.
  * @param uartx UART register base address.
  * @retval Word length.
  */
static inline unsigned int DCL_UART_GetDataLength(const UART_RegStruct * uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_LCR_H.BIT.wlen;
}

/**
  * @brief Setting UART odd parity check.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_SetParityOdd(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_LCR_H.BIT.eps = BASE_CFG_DISABLE;
    uartx->UART_LCR_H.BIT.pen = BASE_CFG_ENABLE;
}

/**
  * @brief Setting UART even parity check.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_SetParityEven(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_LCR_H.BIT.eps = BASE_CFG_ENABLE;
    uartx->UART_LCR_H.BIT.pen = BASE_CFG_ENABLE;
}

/**
  * @brief UART does not use parity check.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_SetParityNone(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_LCR_H.BIT.pen = BASE_CFG_DISABLE;
}

/**
  * @brief Getting UART odd/even parity check.
  * @param uartx UART register base address.
  * @retval Odd/even parity check, 0: odd, 1: even, 2: None.
  */
static inline unsigned int DCL_UART_GetParityCheck(const UART_RegStruct * uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    unsigned int eps =  uartx->UART_LCR_H.BIT.eps;
    unsigned int pen = uartx->UART_LCR_H.BIT.pen;
    if (eps == 0) {
        return UART_PARITY_NONE;
    } else if (pen == 0) {
        return UART_PARITY_ODD;
    } else {
        return UART_PARITY_EVEN;
    }
}

/**
  * @brief Setting the stop bit.
  * @param uartx UART register base address.
  * @param bit One or two stop bit, @ref UART_StopBits
  * @retval None.
  */
static inline void DCL_UART_SetStopBits(UART_RegStruct * const uartx, UART_StopBits bit)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    UART_PARAM_CHECK_NO_RET(IsUartStopbits(bit));
    uartx->UART_LCR_H.BIT.stp2 = bit;
}

/**
  * @brief Getting the stop bit.
  * @param uartx UART register base address.
  * @retval Stop bit of UART.
  */
static inline unsigned int DCL_UART_GetStopBits(const UART_RegStruct *uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_LCR_H.BIT.stp2;
}

/**
  * @brief UART uses hardware flow control.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_Enable_HwFlowCtr(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_CR.BIT.ctsen = BASE_CFG_ENABLE;
    uartx->UART_CR.BIT.rtsen = BASE_CFG_ENABLE;
}

/**
  * @brief UART uses hardware flow control.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_Disable_HwFlowCtr(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_CR.BIT.ctsen = BASE_CFG_DISABLE;
    uartx->UART_CR.BIT.rtsen = BASE_CFG_DISABLE;
}

/**
  * @brief UART Disable function of stick parity.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_DisableStickParity(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_LCR_H.BIT.sps = BASE_CFG_DISABLE;
}

/**
  * @brief UART enable function of stick parity 0-bit check.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_EnableStickParity_Zero(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_LCR_H.BIT.pen = BASE_CFG_ENABLE;
    uartx->UART_LCR_H.BIT.eps = BASE_CFG_ENABLE;
    uartx->UART_LCR_H.BIT.sps = BASE_CFG_ENABLE;
}

/**
  * @brief UART enable function of stick parity 1-bit check.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_EnableStickParity_One(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_LCR_H.BIT.pen = BASE_CFG_ENABLE;
    uartx->UART_LCR_H.BIT.eps = BASE_CFG_DISABLE;
    uartx->UART_LCR_H.BIT.sps = BASE_CFG_ENABLE;
}

/**
  * @brief UART enable interrupt of CTS.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_EnableCTSInt(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_LCR_H.BIT.pen = BASE_CFG_ENABLE;
}

/**
  * @brief UART clear interrupt of CTS.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_ClearCTSInt(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_ICR.BIT.ctsmic = BASE_CFG_ENABLE;
    uartx->UART_IMSC.BIT.ctsmim = BASE_CFG_DISABLE;
}

/**
  * @brief UART get interrupt status of CTS.
  * @param uartx UART register base address.
  * @retval status, 1: Interrupt generation, 0:  interrupt is not generated.
  */
static inline unsigned int DCL_UART_GetCTSIntStatus(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_MIS.BIT.ctsmmis;
}

/**
  * @brief Set the data bits. The first bit to be transmitted and received is the LSB.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_SetDataLSB(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_DS.BIT.msbfirst = 0;
}

/**
  * @brief Set the data bits. The first bit to be transmitted and received is the MSB.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_SetDataMSB(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_DS.BIT.msbfirst = 1;
}

/**
  * @brief Setting data sequences of UART.
  * @param uartx UART register base address.
  * @param bool  1: enable MSB 0: enable LSB.
  * @retval None.
  */
static inline void DCL_UART_SetDataSequences(UART_RegStruct * const uartx, bool dataSequence)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_DS.BIT.msbfirst = dataSequence;
}


/**
  * @brief Configuring the upper limit of receiving timeout.
  * @param uartx UART register base address.
  * @param timeOfBits timeout, time required to transmit a certain bit.
  * @retval None.
  */
static inline void DCL_UART_SetRxTimeOut(UART_RegStruct * const uartx, unsigned int timeOfBits)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    UART_PARAM_CHECK_NO_RET(timeOfBits <= 0xFFFFFF);
    uartx->UART_RTCFG.reg = timeOfBits;
}

/**
  * @brief Enable automatic baud rate detection.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_EnableBaudRateDetection(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_ABDEN.BIT.abden = BASE_CFG_ENABLE;
}

/**
  * @brief Disable automatic baud rate detection.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_DisableBaudRateDetection(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_ABDEN.BIT.abden = BASE_CFG_DISABLE;
}

/**
  * @brief Enable character adaptation.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_EnableMatchCharater(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_CHARMATCH.BIT.cmen = BASE_CFG_ENABLE;
}

/**
  * @brief Disable character adaptation.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_DisableMatchCharater(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_CHARMATCH.BIT.cmen = BASE_CFG_DISABLE;
}

/**
  * @brief Sets the character to be matched.
  * @param uartx UART register base address.
  * @param ascii ascii of character.
  * @retval None.
  */
static inline void DCL_UART_SetMatchCharater(UART_RegStruct * const uartx, unsigned int ascii)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    UART_PARAM_CHECK_NO_RET(ascii <= 0xFF);
    uartx->UART_CHARMATCH.BIT.chamat = ascii;
}

/**
  * @brief Sets UART oversampling multiple.
  * @param uartx UART register base address.
  * @param multiple Oversampling multiple, @ref UART_OversampleMultiple
  * @retval None.
  */
static inline void DCL_UART_OversampleMultiple(UART_RegStruct * const uartx, UART_OversampleMultiple multiple)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    UART_PARAM_CHECK_NO_RET(IsUartOversampleMultiple(multiple));
    uartx->UART_SPCFG.BIT.spcfg = multiple;
}
/**
  * @}
  */

/**
  * @}
  */
#endif  /* McuMagicTag_UART_IP_H */