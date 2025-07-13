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

/**
  * @addtogroup UART
  * @{
  */

/**
  * @defgroup UART_IP UART_IP
  * @brief UART_IP: uart_v0
  * @{
  */

/**
  * @defgroup UART_Param_Def UART Parameters Definition
  * @brief Definition of UART configuration parameters.
  * @{
  */

/**
 * @brief Extent handle definition of UART
 */
typedef struct {
} UART_ExtendHandle;

/**
  * @brief Type ID of the callback function registered by the user.
  */
typedef enum {
    UART_WRITE_IT_FINISH = 0x00000000U,
    UART_READ_IT_FINISH = 0x00000001U,
    UART_WRITE_DMA_FINISH = 0x00000002U,
    UART_READ_DMA_FINISH = 0x00000003U,
    UART_TRNS_IT_ERROR = 0x00000004U,
    UART_TRNS_DMA_ERROR = 0x00000005U
} UART_CallbackFun_Type;

/**
 * @brief User Callback Function Definition
 */
typedef struct {
    void (* WriteItFinishCallBack)(void *handle);     /**< UART tx interrupt complete callback
                                                                          function for users */
    void (* ReadItFinishCallBack)(void *handle);      /**< UART rx interrupt complete callback
                                                                          function for users */
    void (* WriteDmaFinishCallBack)(void *handle);    /**< UART tx  DMA complete callback function
                                                                          for users */
    void (* ReadDmaFinishCallBack)(void *handle);     /**< UART rx DMA complete callback function
                                                                          for users */
    void (* TransmitItErrorCallBack)(void *handle);   /**< UART interrupt mode error callback
                                                                          function for users */
    void (* TransmitDmaErrorCallBack)(void *handle);  /**< UART interrupt mode error callback
                                                                          function for users */
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
  *          + UART_PARITY_ODD  -- odd check
  *          + UART_PARITY_EVEN -- even check
  *          + UART_PARITY_NONE -- none odd or even check
  */
typedef enum {
    UART_PARITY_ODD = 0x00000000U,
    UART_PARITY_EVEN = 0x00000001U,
    UART_PARITY_NONE = 0x00000002U
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
  *          + UART_FIFOFULL_ONE_EIGHT           -- rxFIFO >= 1/8 FULL, txFIFO <= 1/8 FULL
  *          + UART_FIFOFULL_ONE_FOUR            -- rxFIFO >= 1/4 FULL, txFIFO <= 1/4 FULL
  *          + UART_FIFOFULL_THREE_FOUR          -- rxFIFO >= 3/4 FULL, txFIFO <= 3/4 FULL
  *          + UART_FIFOFULL_SEVEN_EIGHT         -- rxFIFO >= 7/8 FULL, txFIFO <= 7/8 FULL
  *          + UART_FIFOFULL_ONE_SIXTEEN         -- rxFIFO >= 1/16 FULL
  *          + UART_FIFOFULL_ONE_THIRTYTWO       -- rxFIFO >= 1/32 FULL
  *          + UART_FIFOFULL_FIVETEEN_SIXTEEN    -- txFIFO <= 15/16 FULL
  *          + UART_FIFOFULL_THIRTYONE_THIRTYTWO -- txFIFO <= 31/32 FULL
  */
typedef enum {
    UART_FIFOFULL_ONE_EIGHT = 0x00000000U,
    UART_FIFOFULL_ONE_FOUR = 0x00000001U,
    UART_FIFOFULL_ONE_TWO = 0x00000002U,
    UART_FIFOFULL_THREE_FOUR = 0x00000003U,
    UART_FIFOFULL_SEVEN_EIGHT = 0x00000004U,
    UART_FIFOFULL_ONE_SIXTEEN = 0x00000005U,
    UART_FIFOFULL_ONE_THIRTYTWO = 0x00000006U,
    UART_FIFOFULL_FIVETEEN_SIXTEEN = 0x00000005U,
    UART_FIFOFULL_THIRTYONE_THIRTYTWO = 0x00000006U
} UART_FIFO_Threshold;

/**
  * @}
  */

/**
  * @defgroup UART_Reg_Def UART Register Definition
  * @brief register mapping structure
  * @{
  */

  /**
  * @brief UART data register, which stores RX data and TX data. and reads RX status from this register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int data : 8;        /**< Receives data and transmits data. */
        unsigned int fe : 1;          /**< Frame error. */
        unsigned int pe : 1;          /**< Verification error. */
        unsigned int be : 1;          /**< Break error. */
        unsigned int oe : 1;          /**< Overflow error. */
        unsigned int reserved0 : 20;
    } BIT;
} volatile UART_DR_REG;

 /**
  * @brief Receive status register/error clear register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int fe : 1;       /**< Frame error. */
        unsigned int pe : 1;       /**< parity check error. */
        unsigned int be : 1;       /**< Break error. */
        unsigned int oe : 1;       /**< Overflow error. */
        unsigned int reserved0 : 28;
    } BIT;
} volatile UART_RSR_REG;

 /**
  * @brief UART flag register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cts : 1;       /**< Hardware flow control status. */
        unsigned int reserved0 : 2;
        unsigned int busy : 1;      /**< UART busy/idle status bit. */
        unsigned int rxfe : 1;      /**< RX hold register/RX FIFO status. The value is 1 when it is empty. */
        unsigned int txff : 1;      /**< TX hold register/Tx FIFO status. The value is 1 when it is full. */
        unsigned int rxff : 1;      /**< RX hold register/RX FIFO status. The value is 1 when it is full. */
        unsigned int txfe : 1;      /**< TX hold register/Tx FIFO status. The value is 1 when it is empty. */
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
        unsigned int reserved0 : 16;
    } BIT;
} volatile UART_IBRD_REG;

 /**
  * @brief Fractional baud rate register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int bauddivfrac : 6;  /**< Fractional baud rate divider. */
        unsigned int reserved0 : 26;
    } BIT;
} volatile UART_FBRD_REG;

 /**
  * @brief Line control register. UART_LCR_H, UART_IBRD, and UART_FBRD constitute a 30-bit register UART_LCR.
  *        UART_LCR is not flushed until UART_LCR_H is written.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int brk : 1;       /**< Send a break. */
        unsigned int pen : 1;       /**< Parity check select bit. */
        unsigned int eps : 1;       /**< Parity check selection during transmission and reception. */
        unsigned int stp2 : 1;      /**< TX frame tail stop bit select. */
        unsigned int fen : 1;       /**< TX and RX FIFO enable control. */
        unsigned int wlen : 2;      /**< Indicates number of transmitted and received data bits in a frame. */
        unsigned int sps : 1;       /**< Select stick parity. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile UART_LCR_H_REG;

 /**
  * @brief UART control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int uarten : 1;     /**< UART enable. */
        unsigned int reserved0 : 6;
        unsigned int lbe : 1;        /**< Indicates whether to enable loopback. */
        unsigned int txe : 1;        /**< UART TX enable. */
        unsigned int rxe : 1;        /**< UART RX enable. */
        unsigned int reserved1 : 1;
        unsigned int rts : 1;        /**< Request to send. */
        unsigned int reserved2 : 2;
        unsigned int rtsen : 1;      /**< RTS hardware flow control enable. */
        unsigned int ctsen : 1;      /**< CTS hardware flow control enable. */
        unsigned int reserved3 : 16;
    } BIT;
} volatile UART_CR_REG;

 /**
  * @brief UART Interrupt FIFO threshold select register.
  *        It is used to set threshold for triggering FIFO interrupt (UART_TXinTR or UART_RXinTR).
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int txiflsel : 3;   /**< Threshold of the TX interrupt FIFO. */
        unsigned int rxiflsel : 3;   /**< Threshold of the RX interrupt FIFO. */
        unsigned int reserved0 : 26;
    } BIT;
} volatile UART_IFLS_REG;

 /**
  * @brief UART interrupt mask register, which is used to mask interrupts.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0 : 1;
        unsigned int ctsmim : 1;    /**< Mask status of the CTS interrupt. */
        unsigned int reserved1 : 2;
        unsigned int rxim : 1;      /**< Mask status of the RX interrupt. */
        unsigned int txim : 1;      /**< Mask status of the TX interrupt. */
        unsigned int rtim : 1;      /**< Mask status of the RX timeout interrupt. */
        unsigned int feim : 1;      /**< Mask status of the frame error interrupt. */
        unsigned int peim : 1;      /**< Mask status of the parity interrupt. */
        unsigned int beim : 1;      /**< Mask status of break error interrupts. */
        unsigned int oeim : 1;      /**< Mask status of the overflow error interrupt. */
        unsigned int reserved2 : 21;
    } BIT;
} volatile UART_IMSC_REG;

 /**
  * @brief UART raw interrupt status register.
  *        The content of this register is not affected by interrupt mask register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0 : 1;
        unsigned int ctsmis : 1;     /**< Raw CTS interrupt status. */
        unsigned int reserved1 : 2;
        unsigned int rxris : 1;      /**< Raw RX interrupt status. */
        unsigned int txris : 1;      /**< Raw TX interrupt status. */
        unsigned int rtris : 1;      /**< Raw RX timeout interrupt status. */
        unsigned int feris : 1;      /**< Raw frame error interrupt status. */
        unsigned int peris : 1;      /**< Raw parity interrupt status. */
        unsigned int beris : 1;      /**< Raw break error interrupt status. */
        unsigned int oeris : 1;      /**< Raw overflow error interrupt status. */
        unsigned int reserved2 : 21;
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
        unsigned int ctsmmis : 1;   /**< Masked CTS interrupt status. */
        unsigned int reserved1 : 2;
        unsigned int rxmis : 1;     /**< Masked RX interrupt status. */
        unsigned int txmis : 1;     /**< Masked TX interrupt status. */
        unsigned int rtmis : 1;     /**< Masked RX timeout interrupt status. */
        unsigned int femis : 1;     /**< Status of masked frame error interrupts. */
        unsigned int pemis : 1;     /**< Masked parity interrupt status. */
        unsigned int bemis : 1;     /**< Status of masked break error interrupts. */
        unsigned int oemis : 1;     /**< Masked overflow error interrupt status. */
        unsigned int reserved2 : 21;
    } BIT;
} volatile UART_MIS_REG;

 /**
  * @brief Interrupt clear register.
  *        Writing 1 clears the corresponding interrupt, and writing 0 does not take effect.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0 : 1;
        unsigned int ctsmic : 1;    /**< Clears the CTS interrupt. */
        unsigned int reserved1 : 2;
        unsigned int rxic : 1;      /**< Clears the RX interrupt. */
        unsigned int txic : 1;      /**< Clear the TX interrupt. */
        unsigned int rtic : 1;      /**< Receive timeout interrupt clear. */
        unsigned int feic : 1;      /**< Frame error interrupt clear. */
        unsigned int peic : 1;      /**< Clears the parity interrupt. */
        unsigned int beic : 1;      /**< Clears the break error interrupt. */
        unsigned int oeic : 1;      /**< Clears the overflow error interrupt. */
        unsigned int reserved2 : 21;
    } BIT;
} volatile UART_ICR_REG;

 /**
  * @brief DMA control register.
  *        which is used to enable the DMA of the TX FIFO and RX FIFO.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int rxdmae : 1;        /**< DMA enable control for RX FIFO. */
        unsigned int txdmae : 1;        /**< DMA enable control for TX FIFO. */
        unsigned int dmaonerr : 1;      /**< DMA enable control for RX channel when UART error interrupt occurs. */
        unsigned int rxlastsreq_en : 1; /**< REQ enable for last data stream supported by UART RX DMA. */
        unsigned int reserved0 : 28;
    } BIT;
} volatile UART_DMACR_REG;

/**
  * @brief Register mapping structure.
  */
typedef struct {
    UART_DR_REG     UART_DR;    /**< Data register, offset address: 0x00000000U */
    UART_RSR_REG    UART_RSR;   /**< Receiving status/error clearing register, offset address: 0x00000004U */
    unsigned char   space0[16];
    UART_FR_REG     UART_FR;    /**< Flag register, offset address: 0x00000018U */
    unsigned char   space1[8];
    UART_IBRD_REG   UART_IBRD;  /**< Integer baud rate register, offset address: 0x00000024U */
    UART_FBRD_REG   UART_FBRD;  /**< Fractional baud rate register, offset address: 0x00000028U */
    UART_LCR_H_REG  UART_LCR_H; /**< Wire control register, offset address: 0x0000002CU */
    UART_CR_REG     UART_CR;    /**< Control register, offset address: 0x00000030U */
    UART_IFLS_REG   UART_IFLS;  /**< Interrupt FIFO threshold register, offset address: 0x00000034U */
    UART_IMSC_REG   UART_IMSC;  /**< Interrupt mask status register, offset address: 0x00000038U */
    UART_RIS_REG    UART_RIS;   /**< Raw interrupt status register, offset address: 0x0000003CU */
    UART_MIS_REG    UART_MIS;   /**< Masked interrupt status register, offset address: 0x00000040U */
    UART_ICR_REG    UART_ICR;   /**< Interrupt clear register, offset address: 0x00000044U */
    UART_DMACR_REG  UART_DMACR; /**< DMA control register register, offset address: 0x00000048U */
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
    if ((paritymode == UART_PARITY_ODD) ||
        (paritymode == UART_PARITY_EVEN) ||
        (paritymode == UART_PARITY_NONE)) {
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
    return (fifoThreshold >= UART_FIFOFULL_ONE_EIGHT) && (fifoThreshold <= UART_FIFOFULL_ONE_THIRTYTWO);
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
    uartx->UART_DR.BIT.data = data; /* Data to be sent. */
}

/**
  * @brief Receive a character from UART.
  * @param uartx UART register base address.
  * @retval Data, read the received data from the UART data register.
  */
static inline unsigned char DCL_UART_ReadData(const UART_RegStruct *uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_DR.BIT.data; /* Data to be read. */
}

/**
  * @brief Get receiving status.
  * @param uartx UART register base address.
  * @retval overall receive status
  */
static inline unsigned int DCL_UART_ReceiveStatus(const UART_RegStruct *uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_DR.reg; /* unsigned overall receive status */
}

/**
  * @brief UART TX enable.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_WriteEnable(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_CR.BIT.txe = BASE_CFG_ENABLE; /* Tx send enable */
}

/**
  * @brief UART TX disable.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_WriteDisable(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_CR.BIT.txe = BASE_CFG_DISABLE; /* Tx send disable */
}

/**
  * @brief UART RX enable.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_ReadEnable(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_CR.BIT.rxe = BASE_CFG_ENABLE; /* Rx read enable */
}

/**
  * @brief UART RX disable.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_ReadDisable(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_CR.BIT.rxe = BASE_CFG_DISABLE; /* Rx read disable */
}

/**
  * @brief Request Tx send, output signal is 0.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_EnableRequestTxSend(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_CR.BIT.rts = BASE_CFG_ENABLE; /* Rx read enable */
}

/**
  * @brief Request Tx send, output signal does not change.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_DisableRequestTxSend(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_CR.BIT.rts = BASE_CFG_DISABLE;
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
  * @brief Enable UART.
  * @param uartx UART register base address.
  * @retval None
  */
static inline void DCL_UART_EnableUart(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_CR.BIT.uarten = BASE_CFG_ENABLE;
}

/**
  * @brief Disable UART. If the UART is disabled during Tx and Rx,
  *        transfer of the current data ends before it stops normally.
  * @param uartx UART register base address.
  * @retval None
  */
static inline void DCL_UART_DisableUart(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_CR.BIT.uarten = BASE_CFG_DISABLE;
}

/**
  * @brief Enable Loopback Tx-Rx.
  * @param uartx UART register base address.
  * @retval None
  */
static inline void DCL_UART_EnableLoopBack(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_CR.BIT.lbe = BASE_CFG_ENABLE;
}

/**
  * @brief Disable Loopback Tx-Rx.
  * @param uartx UART register base address.
  * @retval None
  */
static inline void DCL_UART_DisableLoopBack(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_CR.BIT.lbe = BASE_CFG_DISABLE;
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
  * @brief UART TX not use DMA .
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_DMA_WriteDisable(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_DMACR.BIT.txdmae = BASE_CFG_DISABLE;
}

/**
  * @brief UART RX use DMA.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_DMA_ReadEnable(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_DMACR.BIT.rxdmae = BASE_CFG_ENABLE;
}

/**
  * @brief UART RX not use DMA .
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_DMA_ReadDisable(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_DMACR.BIT.rxdmae = BASE_CFG_DISABLE;
}

/**
  * @brief Set UART word length.
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
  * @brief Get UART word length.
  * @param uartx UART register base address.
  * @retval unsigned int: Word length of sending and receiving.
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
  * @brief Set stop bit.
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
  * @brief Get stop bit.
  * @param uartx UART register base address.
  * @retval bool: 0: 1-bit stop bit is attached to the transmitted frame tail.
  *               1: 2-bit stop bit at the end of the transmitted frame.
  */
static inline bool DCL_UART_GetStopBits(const UART_RegStruct *uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_LCR_H.BIT.stp2;
}

/**
  * @brief UART disable stick parity.
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
    uartx->UART_IMSC.BIT.ctsmim = BASE_CFG_ENABLE;
}

/**
  * @brief UART disable interrupt of CTS.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_DisableCTSInt(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_IMSC.BIT.ctsmim = BASE_CFG_DISABLE;
}

/**
  * @brief Set line control.
  * @param uartx UART register base address.
  * @param  controlValue Configuration value of line controller.
  * @retval None.
  */
static inline void DCL_UART_SetLineControl(UART_RegStruct * const uartx, unsigned int controlValue)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_LCR_H.reg = controlValue;
}

/**
  * @brief Enable TX and RX FIFO.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_EnableFIFO(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_LCR_H.BIT.fen = BASE_CFG_ENABLE;
}

/**
  * @brief Disable TX and RX FIFO.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_DisableFIFO(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_LCR_H.BIT.fen = BASE_CFG_DISABLE;
}

/**
  * @brief Enable Tx break. The current data is transmitted.
  *        Tx continuously outputs a low level.
  * @param uartx UART register base address.
  * @retval None
  */
static inline void DCL_UART_EnableTxBreak(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_LCR_H.BIT.brk = BASE_CFG_ENABLE;
}

/**
  * @brief Disable Tx break. no effect.
  * @param uartx UART register base address.
  * @retval None
  */
static inline void DCL_UART_DisableTxBreak(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_LCR_H.BIT.brk = BASE_CFG_DISABLE;
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
  * @brief Clear Interrupts.
  * @param uartx UART register base address.
  * @param clearInterruptBits bit 1: clear interrupt, bit 0: no effect.
  * @retval None
  */
static inline void DCL_UART_ClearInterrupts(UART_RegStruct * const uartx, unsigned int clearInterruptBits)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_ICR.reg = clearInterruptBits;
}

/**
  * @brief Clear Tx interrupt.
  * @param uartx UART register base address.
  * @retval void
  */
static inline void DCL_UART_ClearTxInt(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_ICR.BIT.txic = BASE_CFG_ENABLE;
}

/**
  * @brief Clear overflow interrupt.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_ClearOverflowINT(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_ICR.BIT.oeic = BASE_CFG_ENABLE;
}

/**
  * @brief Clear break error interrupt.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_ClearBreakErrorINT(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_ICR.BIT.beic = BASE_CFG_ENABLE;
}

/**
  * @brief Clear parity interrupt.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_ClearParityINT(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_ICR.BIT.peic = BASE_CFG_ENABLE;
}

/**
  * @brief Clear frame error interrupt.
  * @param uartx UART register base address.
  * @retval None.
  */
static inline void DCL_UART_ClearFrameErrorINT(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_ICR.BIT.feic = BASE_CFG_ENABLE;
}

/**
  * @brief UART get MIS interrupt status of CTS.
  * @param uartx UART register base address.
  * @retval status, 1: Interrupt generation, 0:  interrupt is not generated.
  */
static inline unsigned int DCL_UART_GetMISCTSIntStatus(const UART_RegStruct *uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_MIS.BIT.ctsmmis;
}

/**
  * @brief MIS error interrupt status.
  * @param uartx UART register base address.
  * @retval bool: 1 Error interrupt generated, 0 No Error interrupt generated.
  */
static inline bool DCL_UART_GetMISErrorINTStatus(const UART_RegStruct * uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_MIS.BIT.oemis;
}

/**
  * @brief MIS break interrupt status.
  * @param uartx UART register base address.
  * @retval bool: 1 Break interrupt generated, 0 No break interrupt generated.
  */
static inline bool DCL_UART_GetMISBreakINTStatus(const UART_RegStruct * uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_MIS.BIT.bemis;
}

/**
  * @brief MIS Parity interrupt status.
  * @param uartx UART register base address.
  * @retval bool: 1 verification interrupt generated, 0 No verification interrupt generated.
  */
static inline bool DCL_UART_GetMISParityINTStatus(const UART_RegStruct * uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_MIS.BIT.pemis;
}

/**
  * @brief MIS frame error interrupt status.
  * @param uartx UART register base address.
  * @retval bool: 1 frame error interrupt generated, 0 No frame error interrupt generated.
  */
static inline bool DCL_UART_GetMISFrameErrorINTStatus(const UART_RegStruct * uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_MIS.BIT.femis;
}

/**
  * @brief MIS send interrupt status.
  * @param uartx UART register base address.
  * @retval bool: 1 send interrupt generated 0 No send interrupt generated.
  */
static inline bool DCL_UART_GetMISSendINTStatus(const UART_RegStruct * uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_MIS.BIT.txmis;
}

/**
  * @brief MIS receive interrupt status.
  * @param uartx UART register base address.
  * @retval bool: 1 receive interrupt generated 0 No receive interrupt generated.
  */
static inline bool DCL_UART_GetMISReceiveINTStatus(const UART_RegStruct * uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_MIS.BIT.rxmis;
}


/**
  * @brief MIS receive timeout interrupt status.
  * @param uartx UART register base address.
  * @retval bool: 1 receive timeout interrupt generated 0 No receive timeout interrupt generated.
  */
static inline bool DCL_UART_GetMISReceiveTimeOutINTStatus(const UART_RegStruct * uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_MIS.BIT.rtmis;
}

/**
  * @brief Get Busy/idle status of UART.
  * @param uartx UART register base address.
  * @retval status, 1: busy, 0: idle.
  */
static inline bool DCL_UART_GetBusyIdleStatus(const UART_RegStruct * uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_FR.BIT.busy;
}

/**
  * @brief Get Tx FIFO Full status.
  * @param uartx UART register base address.
  * @retval bool: 1 TxFIFO is Full
  */
static inline bool DCL_UART_GetTxFIFOFullStatus(const UART_RegStruct *uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_FR.BIT.txff;
}

/**
  * @brief Whether Tx FIFO is empty.
  * @param uartx UART register base address.
  * @retval bool: 1 TxFIFO is empty
  */
static inline bool DCL_UART_GetTxFIFOEmptyStatus(const UART_RegStruct *uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_FR.BIT.txfe;
}

/**
  * @brief Get Rx FIFO status.
  * @param uartx UART register base address.
  * @retval bool: 1 TxFIFO is Full.
  */
static inline bool DCL_UART_GetRxFIFOFullStatus(const UART_RegStruct *uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_FR.BIT.rxff;
}

/**
  * @brief Whether Rx FIFO is empty.
  * @param uartx UART register base address.
  * @retval bool: 1 RxFIFO is empty.
  */
static inline bool DCL_UART_GetRxFIFOEmptyStatus(const UART_RegStruct *uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    return uartx->UART_FR.BIT.rxfe;
}

/**
  * @brief Set fractional baud rate.
  * @param uartx UART register base address.
  * @param fractionBaud fractional baud rate.
  * @retval None.
  */
static inline void DCL_UART_SetfractiondBaud(UART_RegStruct * const uartx, unsigned int fractionBaud)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_FBRD.reg = fractionBaud;
}

/**
  * @brief Set integer baud rate.
  * @param uartx UART register base address.
  * @param integerBaud integer baud rate.
  * @retval None
  */
static inline void DCL_UART_SetIntegerBaud(UART_RegStruct * const uartx, unsigned int IntegerBaud)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_IBRD.reg = IntegerBaud;
}

/**
  * @brief Set Rx FIFO threshold select.
  * @param uartx UART register base address.
  * @param thresholdValue value of Rx FIFO threshold, @ref UART_FIFO_Threshold
  * @retval None.
  */
static inline void DCL_UART_SetRxFIFOThreshold(UART_RegStruct * const uartx, UART_FIFO_Threshold thresholdValue)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    UART_ASSERT_PARAM(IsUartFIFOThreshold(thresholdValue));
    uartx->UART_IFLS.BIT.rxiflsel = thresholdValue;
}

/**
  * @brief Set Tx FIFO threshold select.
  * @param uartx UART register base address.
  * @param thresholdValue value of tx FIFO threshold, @ref UART_FIFO_Threshold
  * @retval None.
  */
static inline void DCL_UART_SetTxFIFOThreshold(UART_RegStruct * const uartx, UART_FIFO_Threshold thresholdValue)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    UART_ASSERT_PARAM(IsUartFIFOThreshold(thresholdValue));
    uartx->UART_IFLS.BIT.txiflsel = thresholdValue;
}

/**
  * @brief Masking interrupts.
  * @param uartx UART register base address.
  * @param clearInterruptBits bit 1: Not mask interrupt, bit 0: Mask interrupt.
  * @retval None
  */
static inline void DCL_UART_MaskingInterrupts(UART_RegStruct * const uartx, unsigned int maskingInterruptBits)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_IMSC.reg = maskingInterruptBits;
}

/**
  * @brief Enable MSC Tx Interrupt.
  * @param uartx UART register base address.
  * @retval None
  */
static inline void DCL_UART_EnableMSCTxInterrupt(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_IMSC.BIT.txim = BASE_CFG_ENABLE;
}

/**
  * @brief Disable MSC Tx Interrupt.
  * @param uartx UART register base address.
  * @retval None
  */
static inline void DCL_UART_DisableMSCTxInterrupt(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_IMSC.BIT.txim = BASE_CFG_DISABLE;
}

/**
  * @brief Disable Rx interrupt.
  * @param uartx UART register base address.
  * @retval void
  */
static inline void DCL_UART_DisableRxInt(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_IMSC.BIT.rxim = BASE_CFG_DISABLE;
}

/**
  * @brief Enable Rx interrupt.
  * @param uartx UART register base address.
  * @retval void
  */
static inline void DCL_UART_EnableRxInt(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_IMSC.BIT.rxim = BASE_CFG_ENABLE;
}

/**
  * @brief UART DMA enable Rx last request.
  * @param uartx UART register base address.
  * @retval None
  */
static inline void DCL_UART_EnableDMARxLastReq(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_DMACR.BIT.rxlastsreq_en = BASE_CFG_ENABLE;
}

/**
  * @brief UART DMA disable Rx last request.
  * @param uartx UART register base address.
  * @retval None
  */
static inline void DCL_UART_DisableDMARxLastReq(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_DMACR.BIT.rxlastsreq_en = BASE_CFG_DISABLE;
}

/**
  * @brief UART DMA enable DMA on error interrupt.
  * @param uartx UART register base address.
  * @retval None
  */
static inline void DCL_UART_EnableDMANoErrorINT(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_DMACR.BIT.dmaonerr = BASE_CFG_ENABLE;
}

/**
  * @brief UART DMA disable DMA on error interrupt.
  * @param uartx UART register base address.
  * @retval None
  */
static inline void DCL_UART_DisableDMANoErrorINT(UART_RegStruct * const uartx)
{
    UART_ASSERT_PARAM(IsUARTInstance(uartx));
    uartx->UART_DMACR.BIT.dmaonerr = BASE_CFG_DISABLE;
}

/**
  * @}
  */

/**
  * @}
  */
#endif  /* McuMagicTag_UART_IP_H */