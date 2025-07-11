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
  * @file    spi_ip.h
  * @author  MCU Driver Team.
  * @brief   SPI module driver.
  * @details This file provides firmware functions to manage the following.
  *          functionalities of the SPI.
  *          + Register definition structure.
  *          + Direct configuration layer interface.
  *          + Parameter check inline function.
  */
#ifndef McuMagicTag_SPI_IP_H
#define McuMagicTag_SPI_IP_H

/* Includes ------------------------------------------------------------------*/
#include "baseinc.h"
#ifdef SPI_PARAM_CHECK
#define SPI_ASSERT_PARAM BASE_FUNC_ASSERT_PARAM
#define SPI_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define SPI_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define SPI_ASSERT_PARAM(para) ((void)0U)
#define SPI_PARAM_CHECK_NO_RET(para) ((void)0U)
#define SPI_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif

/**
  * @addtogroup SPI
  * @{
  */

/**
  * @defgroup SPI_IP SPI_IP
  * @brief SPI_IP: spi_v1.
  * @{
  */

#define SPI_CR0_SCR_POS    8
#define SPI_CR0_SCR_MASK    (0xFF << SPI_CR0_SCR_POS)

/**
  * @defgroup SPI_Param_Def SPI Parameters Definition
  * @brief Definition of SPI configuration parameters.
  * @{
  */

/* Typedef definitions -------------------------------------------------------*/
/**
 * @brief Master and Slave Device Enumeration Definition.
 */
typedef enum {
    HAL_SPI_MASTER = 0x00000000U,
    HAL_SPI_SLAVE  = 0x00000001U
} HAL_SPI_Mode;

/**
 * @brief Clock Polarity Enumeration Definition.
 */
typedef enum {
    HAL_SPI_CLKPOL_0 = 0x00000000U,
    HAL_SPI_CLKPOL_1 = 0x00000001U
} HAL_SPI_ClkPol;

/**
 * @brief Clock Phase Enumeration Definition.
 */
typedef enum {
    HAL_SPI_CLKPHA_0 = 0x00000000U,
    HAL_SPI_CLKPHA_1 = 0x00000001U
} HAL_SPI_ClkPha;

/**
 * @brief Enumeration definition of data endian.
 */
typedef enum {
    HAL_SPI_BIG_ENDIAN     = 0x00000000U,
    HAL_SPI_LITTILE_ENDIAN = 0x00000001U
} HAL_SPI_Endian;

/**
 * @brief Enumerated definition of data frame mode selection.
 */
typedef enum {
    HAL_SPI_MODE_MOTOROLA  = 0x00000000U,
    HAL_SPI_MODE_TI        = 0x00000001U,
    HAL_SPI_MODE_MICROWIRE = 0x00000002U
} HAL_SPI_FrameMode;

/**
 * @brief Transmission Mode Selection Enumeration Definition.
 */
typedef enum {
    HAL_XFER_MODE_BLOCKING    = 0x00000000U,
    HAL_XFER_MODE_INTERRUPTS  = 0x00000001U,
    HAL_XFER_MODE_DMA         = 0x00000002U
} HAL_SPI_XferMode;

/**
 * @brief SPI Chip Select Config Definition.
 */
typedef enum {
    HAL_SPI_CHIP_CONFIG_AUTO       = 0x00000000U,
    HAL_SPI_CHIP_CONFIG_ALTASENS   = 0x00000001U,
    HAL_SPI_CHIP_CONFIG_SOFR_UNSET = 0x00000002U,
    HAL_SPI_CHIP_CONFIG_SOFR_SET   = 0x00000003U
} HAL_SPI_CHIP_CONFIG;

/**
 * @brief Data Bit Width Enumeration Definition.
 */
typedef enum {
    SPI_DATA_WIDTH_4BIT  = 0x00000003U,
    SPI_DATA_WIDTH_5BIT  = 0x00000004U,
    SPI_DATA_WIDTH_6BIT  = 0x00000005U,
    SPI_DATA_WIDTH_7BIT  = 0x00000006U,
    SPI_DATA_WIDTH_8BIT  = 0x00000007U,
    SPI_DATA_WIDTH_9BIT  = 0x00000008U,
    SPI_DATA_WIDTH_10BIT = 0x00000009U,
    SPI_DATA_WIDTH_11BIT = 0x0000000aU,
    SPI_DATA_WIDTH_12BIT = 0x0000000bU,
    SPI_DATA_WIDTH_13BIT = 0x0000000cU,
    SPI_DATA_WIDTH_14BIT = 0x0000000dU,
    SPI_DATA_WIDTH_15BIT = 0x0000000eU,
    SPI_DATA_WIDTH_16BIT = 0x0000000fU
} HAL_SPI_DataWidth;

/**
 * @brief Definitions of available parameters for interrupt Tx thresholds.
 */
typedef enum {
    SPI_TX_INTERRUPT_SIZE_0     = 0x00000000U,
    SPI_TX_INTERRUPT_SIZE_1     = 0x00000001U,
    SPI_TX_INTERRUPT_SIZE_2     = 0x00000002U,
    SPI_TX_INTERRUPT_SIZE_3     = 0x00000003U,
    SPI_TX_INTERRUPT_SIZE_4     = 0x00000004U,
    SPI_TX_INTERRUPT_SIZE_5     = 0x00000005U,
    SPI_TX_INTERRUPT_SIZE_6     = 0x00000006U,
    SPI_TX_INTERRUPT_SIZE_7     = 0x00000007U,
    SPI_TX_INTERRUPT_SIZE_8     = 0x00000008U,
    SPI_TX_INTERRUPT_SIZE_9     = 0x00000009U,
    SPI_TX_INTERRUPT_SIZE_10    = 0x0000000AU,
    SPI_TX_INTERRUPT_SIZE_11    = 0x0000000BU,
    SPI_TX_INTERRUPT_SIZE_12    = 0x0000000CU,
    SPI_TX_INTERRUPT_SIZE_13    = 0x0000000DU,
    SPI_TX_INTERRUPT_SIZE_14    = 0x0000000EU,
    SPI_TX_INTERRUPT_SIZE_15    = 0x0000000FU
} HAL_SPI_TxInterruptSize;

/**
 * @brief Definitions of available parameters for interrupt Rx thresholds.
 */
typedef enum {
    SPI_RX_INTERRUPT_SIZE_0     = 0x00000000U,
    SPI_RX_INTERRUPT_SIZE_1     = 0x00000001U,
    SPI_RX_INTERRUPT_SIZE_2     = 0x00000002U,
    SPI_RX_INTERRUPT_SIZE_3     = 0x00000003U,
    SPI_RX_INTERRUPT_SIZE_4     = 0x00000004U,
    SPI_RX_INTERRUPT_SIZE_5     = 0x00000005U,
    SPI_RX_INTERRUPT_SIZE_6     = 0x00000006U,
    SPI_RX_INTERRUPT_SIZE_7     = 0x00000007U,
    SPI_RX_INTERRUPT_SIZE_8     = 0x00000008U,
    SPI_RX_INTERRUPT_SIZE_9     = 0x00000009U,
    SPI_RX_INTERRUPT_SIZE_10    = 0x0000000AU,
    SPI_RX_INTERRUPT_SIZE_11    = 0x0000000BU,
    SPI_RX_INTERRUPT_SIZE_12    = 0x0000000CU,
    SPI_RX_INTERRUPT_SIZE_13    = 0x0000000DU,
    SPI_RX_INTERRUPT_SIZE_14    = 0x0000000EU,
    SPI_RX_INTERRUPT_SIZE_15    = 0x0000000FU,
} HAL_SPI_RxInterruptSize;

/**
 * @brief Definitions of available parameters for DMA Tx thresholds.
 */
typedef enum {
    SPI_TX_DMA_BURST_SIZE_0     = 0x00000000U,
    SPI_TX_DMA_BURST_SIZE_1     = 0x00000001U,
    SPI_TX_DMA_BURST_SIZE_2     = 0x00000002U,
    SPI_TX_DMA_BURST_SIZE_3     = 0x00000003U,
    SPI_TX_DMA_BURST_SIZE_4     = 0x00000004U,
    SPI_TX_DMA_BURST_SIZE_5     = 0x00000005U,
    SPI_TX_DMA_BURST_SIZE_6     = 0x00000006U,
    SPI_TX_DMA_BURST_SIZE_7     = 0x00000007U,
    SPI_TX_DMA_BURST_SIZE_8     = 0x00000008U,
    SPI_TX_DMA_BURST_SIZE_9     = 0x00000009U,
    SPI_TX_DMA_BURST_SIZE_10    = 0x0000000AU,
    SPI_TX_DMA_BURST_SIZE_11    = 0x0000000BU,
    SPI_TX_DMA_BURST_SIZE_12    = 0x0000000CU,
    SPI_TX_DMA_BURST_SIZE_13    = 0x0000000DU,
    SPI_TX_DMA_BURST_SIZE_14    = 0x0000000EU,
    SPI_TX_DMA_BURST_SIZE_15    = 0x0000000FU,
} HAL_SPI_TxDmaBurstSize;

/**
 * @brief Definitions of available parameters for DMA Rx thresholds.
 */
typedef enum {
    SPI_RX_DMA_BURST_SIZE_0      = 0x00000000U,
    SPI_RX_DMA_BURST_SIZE_1      = 0x00000001U,
    SPI_RX_DMA_BURST_SIZE_2      = 0x00000002U,
    SPI_RX_DMA_BURST_SIZE_3      = 0x00000003U,
    SPI_RX_DMA_BURST_SIZE_4      = 0x00000004U,
    SPI_RX_DMA_BURST_SIZE_5      = 0x00000005U,
    SPI_RX_DMA_BURST_SIZE_6      = 0x00000006U,
    SPI_RX_DMA_BURST_SIZE_7      = 0x00000007U,
    SPI_RX_DMA_BURST_SIZE_8      = 0x00000008U,
    SPI_RX_DMA_BURST_SIZE_9      = 0x00000009U,
    SPI_RX_DMA_BURST_SIZE_10     = 0x0000000AU,
    SPI_RX_DMA_BURST_SIZE_11     = 0x0000000BU,
    SPI_RX_DMA_BURST_SIZE_12     = 0x0000000CU,
    SPI_RX_DMA_BURST_SIZE_13     = 0x0000000DU,
    SPI_RX_DMA_BURST_SIZE_14     = 0x0000000EU,
    SPI_RX_DMA_BURST_SIZE_15     = 0x0000000FU,
} HAL_SPI_RxDmaBurstSize;

/**
 * @brief Defines the SPI chip select channel.
 */
typedef enum {
    SPI_CHIP_SELECT_CHANNEL_0   = 0x00000000U,
    SPI_CHIP_SELECT_CHANNEL_1   = 0x00000001U,
    SPI_CHIP_SELECT_CHANNEL_2   = 0x00000002U,
    SPI_CHIP_SELECT_CHANNEL_3   = 0x00000003U,
    SPI_CHIP_SELECT_CHANNEL_MAX = 0x00000004U
} SPI_ChipSelectChannel;

/**
  * @brief SPI extend handle.
  */
typedef struct _SPI_ExtendHandle {
} SPI_ExtendHandle;

/**
  * @brief SPI user callback.
  */
typedef struct {
    /* Sending completion callback function */
    void (* TxCpltCallback)(void *handle);
    /* Receive completion callback function */
    void (* RxCpltCallback)(void *handle);
    /* Receive and Sending completion callback function */
    void (* TxRxCpltCallback)(void *handle);
    /* Error callback function */
    void (* ErrorCallback)(void *handle);
    /* CS callback function */
    void (* CsCtrlCallback)(void *handle);
} SPI_UserCallBack;
/**
  * @}
  */

/**
  * @defgroup SPI_Reg_Def SPI Register Definition
  * @brief register mapping structure
  * @{
  */
/* Register Description Definition----------------------------------- */

/**
  * @brief SPI clock, polarity, phase, frame format, data bit control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int dss       : 4;    /**< data width. */
        unsigned int frf       : 2;    /**< frame format: Motorola TI Mircowire. */
        unsigned int spo       : 1;    /**< motorola polarity. */
        unsigned int sph       : 1;    /**< motorola phase. */
        unsigned int scr       : 8;    /**< serial clock rate. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile SPICR0_REG;

/**
  * @brief SPI parameter control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int lbm           : 1;    /**< loopback mode enable. */
        unsigned int sse           : 1;    /**< SPI enable. */
        unsigned int ms            : 1;    /**< Master or Salve mode. */
        unsigned int reserved0     : 1;
        unsigned int bitend        : 1;    /**< set the endian mode. */
        unsigned int reserved1     : 3;
        unsigned int waitval       : 7;    /**< Microwire wait time. */
        unsigned int waiten        : 1;    /**< Microwire wait enable. */
        unsigned int reserved2     : 16;
    } BIT;
} volatile SPICR1_REG;

/**
  * @brief SPI data FIFO register.
  */
typedef struct {
    unsigned int data      : 16;    /**< send and receive FIFO. */
    unsigned int reserved0 : 16;
} volatile SPIDR_REG;

/**
  * @brief SPI status register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int tfe       : 1;    /**< tx FIFO empty flag. */
        unsigned int tnf       : 1;    /**< tx FIFO not full flag. */
        unsigned int rne       : 1;    /**< rx FIFO not empty flag. */
        unsigned int rff       : 1;    /**< rx FIFO full flag. */
        unsigned int bsy       : 1;    /**< SPI busy flag. */
        unsigned int reserved0 : 27;
    } BIT;
} volatile SPISR_REG;

/**
  * @brief SPI clock divider register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cpsdvsr   : 8;    /**< clock divider value, value is even number between 2 and 254. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile SPICPSR_REG;

/**
  * @brief SPI interrupt mask control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int rorim     : 1;    /**< rx overflow interrupt mask. */
        unsigned int rtim      : 1;    /**< rx timeout interrupt mask. */
        unsigned int rxim      : 1;    /**< rx FIFO interrupt mask. */
        unsigned int txim      : 1;    /**< tx FIFO interrupt mask. */
        unsigned int reserved0 : 28;
    } BIT;
} volatile SPIIMSC_REG;

/**
  * @brief SPI raw interrupt status register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int rorris    : 1;    /**< raw status of the rx overflow interrupt. */
        unsigned int rtris     : 1;    /**< raw status of the rx timeout interrupt. */
        unsigned int rxris     : 1;    /**< raw status of the rx FIFO interrupt. */
        unsigned int txris     : 1;    /**< raw status of the tx FIFO interrupt. */
        unsigned int reserved0 : 28;
    } BIT;
} volatile SPIRIS_REG;

/**
  * @brief SPI masked interrupt status register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int rormis    : 1;    /**< masked status of the rx overflow interrupt. */
        unsigned int rtmis     : 1;    /**< masked status of the rx timeout interrupt. */
        unsigned int rxmis     : 1;    /**< masked status of the rx FIFO interrupt. */
        unsigned int txmis     : 1;    /**< masked status of the tx FIFO interrupt. */
        unsigned int reserved0 : 28;
    } BIT;
} volatile SPIMIS_REG;

/**
  * @brief SPI interrupt clear register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int roric     : 1;    /**< clear the rx overflow interrupt. */
        unsigned int rtic      : 1;    /**< clear the rx timeout interrupt. */
        unsigned int reserved0 : 30;
    } BIT;
} volatile SPIICR_REG;

/**
  * @brief SPI DMA control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int rxdmae      : 1;    /**< DMA rx FIFO enable. */
        unsigned int txdmae      : 1;    /**< DMA tx FIFO enable. */
        unsigned int rxdmalsreqe : 1;    /**< DMA rx FIFO SPI flow control. */
        unsigned int reserved0   : 29;
    } BIT;
} volatile SPIDMACR_REG;

/**
  * @brief SPI tx FIFO control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int txintsize   : 4;    /**< set the threshold of the tx FIFO request interrupt. */
        unsigned int reserved0   : 28;
    } BIT;
} volatile SPITXFIFOCR_REG;

/**
  * @brief SPI rx FIFO control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int rxintsize   : 4;    /**< set the threshold of the rx FIFO request interrupt. */
        unsigned int reserved0   : 28;
    } BIT;
} volatile SPIRXFIFOCR_REG;


/**
  * @brief SPI cs mode control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int spi_csn_sel  : 2;    /**< chip select. */
        unsigned int reserved0    : 2;
        unsigned int spi_csn_mode : 2;    /**< chip select mode. */
        unsigned int reserved1    : 26;
    } BIT;
} volatile SPICSNCR_REG;

/**
 * @brief SPI Register definition structure
 */
typedef struct {
    SPICR0_REG      SPICR0;      /**< SPI parameter control register 0. */
    SPICR1_REG      SPICR1;      /**< SPI parameter control register 1. */
    SPIDR_REG       SPIDR;       /**< SPI data FIFO register. */
    SPISR_REG       SPISR;       /**< SPI status register. */
    SPICPSR_REG     SPICPSR;     /**< SPI clock divider register. */
    SPIIMSC_REG     SPIIMSC;     /**< SPI interrupt mask control register. */
    SPIRIS_REG      SPIRIS;      /**< SPI raw interrupt status register. */
    SPIMIS_REG      SPIMIS;      /**< SPI masked interrupt status register. */
    SPIICR_REG      SPIICR;      /**< SPI interrupt clear register. */
    SPIDMACR_REG    SPIDMACR;    /**< SPI DMA control register. */
    SPITXFIFOCR_REG SPITXFIFOCR; /**< SPI tx FIFO control register. */
    SPIRXFIFOCR_REG SPIRXFIFOCR; /**< SPI rx FIFO control register. */
    SPICSNCR_REG    SPICSNCR;    /**< SPI cs mode control register. */
} volatile SPI_RegStruct;
/**
  * @}
  */

/**
  * @brief Check whether the SPI mode is used.
  * @param mode Spi mode
  * @retval true
  * @retval false
  */
static inline bool IsSpiMode(unsigned int mode)
{
    if (mode == HAL_SPI_MASTER || mode == HAL_SPI_SLAVE) {
        return true;
    }
    return false;
}

/**
  * @brief Check if the transfer mode specified for the SPI.
  * @param xFermode Transfer mode.
  * @retval true
  * @retval false
  */
static inline bool IsSpiXferMode(unsigned int xFermode)
{
    if (xFermode == HAL_XFER_MODE_BLOCKING ||
        xFermode == HAL_XFER_MODE_INTERRUPTS ||
        xFermode == HAL_XFER_MODE_DMA) {
        return true;
    }
    return false;
}

/**
  * @brief Checking SPI Polarity Parameters.
  * @param clkPolarity Polarity Parameters.
  * @retval true
  * @retval false
  */
static inline bool IsSpiClkPolarity(unsigned int clkPolarity)
{
    if (clkPolarity == HAL_SPI_CLKPOL_0 ||
        clkPolarity == HAL_SPI_CLKPOL_1) {
        return true;
    }
    return false;
}

/**
  * @brief Checking SPI Phase Parameters.
  * @param clkPhase Phase Parameters.
  * @retval true
  * @retval false
  */
static inline bool IsSpiClkPhase(unsigned int clkPhase)
{
    if (clkPhase == HAL_SPI_CLKPHA_0 ||
        clkPhase == HAL_SPI_CLKPHA_1) {
        return true;
    }
    return false;
}

/**
  * @brief Check the SPI big-endian configuration parameters.
  * @param endian Big-endian configuration parameters.
  * @retval true
  * @retval false
  */
static inline bool IsSpiEndian(unsigned int endian)
{
    if (endian == HAL_SPI_BIG_ENDIAN ||
        endian == HAL_SPI_LITTILE_ENDIAN) {
        return true;
    }
    return false;
}

/**
  * @brief Check the SPI frame format configuration.
  * @param framFormat Frame format.
  * @retval true
  * @retval false
  */
static inline bool IsSpiFrameFormat(unsigned int framFormat)
{
    if (framFormat == HAL_SPI_MODE_MOTOROLA ||
        framFormat == HAL_SPI_MODE_TI ||
        framFormat == HAL_SPI_MODE_MICROWIRE) {
        return true;
    }
    return false;
}

/**
  * @brief Checking the SPI Data Bit Width Configuration.
  * @param dataWidth Data Bit Width.
  * @retval true
  * @retval false
  */
static inline bool IsSpiDataWidth(unsigned int dataWidth)
{
    if (dataWidth >= SPI_DATA_WIDTH_4BIT && dataWidth <= SPI_DATA_WIDTH_16BIT) {
        return true;
    }
    return false;
}

/**
  * @brief Check the configuration of the waiting time between the TX and RX in the SPI microwire frame format.
  * @param waitVal Waiting time.
  * @retval true
  * @retval false
  */
static inline bool IsSpiWaitVal(unsigned char waitVal)
{
    /* waitval value is 0 to 0x7f */
    if (waitVal <= 0x7f) {
        return true;
    }
    return false;
}

/**
  * @brief Check the SPI interrupt TX threshold configuration.
  * @param txIntSize TX threshold configuration.
  * @retval true
  * @retval false
  */
static inline bool IsSpiTxIntSize(unsigned int txIntSize)
{
    if (txIntSize <= SPI_TX_INTERRUPT_SIZE_15) {
        return true;
    }
    return false;
}

/**
  * @brief Check the SPI interrupt RX threshold configuration.
  * @param rxIntSize RX threshold configuration.
  * @retval true
  * @retval false
  */
static inline bool IsSpiRxIntSize(unsigned int rxIntSize)
{
    if (rxIntSize <= SPI_RX_INTERRUPT_SIZE_15) {
        return true;
    }
    return false;
}

/**
  * @brief Check the SPI DMA TX threshold configuration.
  * @param txDMABurstSize TX threshold.
  * @retval true
  * @retval false
  */
static inline bool IsSpiTxDmaBurstSize(unsigned int txDMABurstSize)
{
    if (txDMABurstSize <= SPI_TX_DMA_BURST_SIZE_15) {
        return true;
    }
    return false;
}

/**
  * @brief Check the SPI DMA RX threshold configuration.
  * @param rxDMABurstSize RX threshold.
  * @retval true
  * @retval false
  */
static inline bool IsSpiRxDmaBurstSize(unsigned int rxDMABurstSize)
{
    if (rxDMABurstSize <= SPI_RX_DMA_BURST_SIZE_15) {
        return true;
    }
    return false;
}

/**
  * @brief Checking SPI frequency divider parameters.
  * @param freqCpsdvsr Frequency division parameters to be checked.
  * @retval true
  * @retval false
  */
static inline bool IsSpiFreqCpsdvsr(unsigned char freqCpsdvsr)
{
    /* FreqCpsdvsr value is 0 to 255 */
    if (freqCpsdvsr >= 2) {
        return true;
    }
    return false;
}

/**
  * @brief Setting SPI Chip Config Mode
  * @param mode Spi Chip Config Mode
  * @retval true
  * @retval false
  */
static inline bool IsSpiChipConfigMode(unsigned int mode)
{
    if (mode <= HAL_SPI_CHIP_CONFIG_SOFR_SET) {
        return true;
    }
    return false;
}

/**
  * @brief Setting SPI Chip Select Channel
  * @param csn Spi Chip Select
  * @retval true
  * @retval false
  */
static inline bool IsSpiChipSelectChannel(unsigned int csn)
{
    if (csn < SPI_CHIP_SELECT_CHANNEL_MAX) {
        return true;
    }
    return false;
}

/* Direct configuration layer interface----------------------------------*/
/**
  * @brief SPI module enable.
  * @param spix SPI register base address.
  * @param spiEnable SPI enable or disable.
  * @retval None.
  */
static inline void DCL_SPI_SetSpiEnable(SPI_RegStruct *spix, bool spiEnable)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    spix->SPICR1.BIT.sse = spiEnable;
}

/**
  * @brief Get SPI enable status.
  * @param spix SPI register base address.
  * @retval bool SPI enable or disable.
  */
static inline bool DCL_SPI_GetSpiEnable(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPICR1.BIT.sse;
}

/**
  * @brief Set SPI loopback.
  * @param spix SPI register base address.
  * @param loop enable or disable
  * @retval None.
  */
static inline void DCL_SPI_SetLoopBack(SPI_RegStruct *spix, bool loop)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    spix->SPICR1.BIT.lbm = loop;
}

/**
  * @brief Get SPI loopback.
  * @param spix SPI register base address.
  * @retval bool loopback is enable or disable.
  */
static inline bool DCL_SPI_GetLoopBack(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPICR1.BIT.lbm;
}

/**
  * @brief Configuring SPI polarity
  * @param spix SPI register base address.
  * @param clkPolarity SPI Polarity,the value is 0 or 1.
  * @retval None.
  */
static inline void DCL_SPI_SetClkPolarity(SPI_RegStruct *spix, unsigned int clkPolarity)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    SPI_PARAM_CHECK_NO_RET(IsSpiClkPolarity(clkPolarity));
    spix->SPICR0.BIT.spo = clkPolarity;
}

/**
  * @brief Get SPI polarity.
  * @param spix SPI register base address.
  * @retval SPI Polarity,the value is 0 or 1.
  */
static inline unsigned char DCL_SPI_GetClkPolarity(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPICR0.BIT.spo;
}

/**
  * @brief Configuring SPI phase.
  * @param spix SPI register base address.
  * @param clkPhase SPI phase,the value is 0 or 1.
  * @retval None.
  */
static inline void DCL_SPI_SetClkPhase(SPI_RegStruct *spix, unsigned int clkPhase)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    SPI_PARAM_CHECK_NO_RET(IsSpiClkPhase(clkPhase));
    spix->SPICR0.BIT.sph =  clkPhase;
}

/**
  * @brief Get SPI phase.
  * @param spix SPI register base address.
  * @retval SPI phase,the value is 0 or 1.
  */
static inline unsigned char DCL_SPI_GetClkPhase(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPICR0.BIT.sph;
}

/**
  * @brief SPI data big endian configuration.
  * @param spix SPI register base address.
  * @param bitEnd Big-endian configuration parameter. The value can be 0 or 1.
  * @retval None.
  */
static inline void DCL_SPI_SetBitEnd(SPI_RegStruct *spix, unsigned int bitEnd)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    SPI_PARAM_CHECK_NO_RET(IsSpiEndian(bitEnd));
    spix->SPICR1.BIT.bitend = bitEnd;
}

/**
  * @brief Get SPI data big endian configuration.
  * @param spix SPI register base address.
  * @retval Big-endian configuration parameter. The value is 0 or 1.
  */
static inline unsigned char DCL_SPI_GetBitEnd(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPICR1.BIT.bitend;
}

/**
  * @brief SPI frame format configuration.
  * @param spix SPI register base address.
  * @param frameFormat Value: Motorola: 00, TI synchronous serial: 01, National Microwire: 10.
  * @retval None.
  */
static inline void DCL_SPI_SetFrameFormat(SPI_RegStruct *spix, unsigned int frameFormat)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    SPI_PARAM_CHECK_NO_RET(IsSpiFrameFormat(frameFormat));
    spix->SPICR0.BIT.frf = frameFormat;
}

/**
  * @brief Get SPI frame format configuration.
  * @param spix SPI register base address.
  * @retval Motorola: 00, TI synchronous serial: 01, National Microwire: 10.
  */
static inline unsigned char DCL_SPI_GetFrameFormat(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPICR0.BIT.frf;
}

/**
  * @brief Configuring the SPI data bit width.
  * @param spix SPI register base address.
  * @param dataWidth The data bit width can be set to 4 to 16 bits.
  * @retval None.
  */
static inline void DCL_SPI_SetDataWidth(SPI_RegStruct *spix, unsigned int dataWidth)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    SPI_PARAM_CHECK_NO_RET(IsSpiDataWidth(dataWidth));
    spix->SPICR0.BIT.dss = dataWidth;
}

/**
  * @brief Get the SPI data bit width configuring.
  * @param spix SPI register base address.
  * @retval SPI Data Bit Width configuring.
  */
static inline unsigned char DCL_SPI_GetDataWidth(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPICR0.BIT.dss;
}

/**
  * @brief SPI serial clock rate configuration.
  * @param spix SPI register base address.
  * @param freqScr Value range: 0 to 255.
  * @retval None.
  */
static inline void DCL_SPI_SetFreqScr(SPI_RegStruct *spix, unsigned char freqScr)
{
    unsigned int cr0Reg;
    unsigned int temp;
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    /* Read the entire register and write it back. */
    temp = ((unsigned int)freqScr) << SPI_CR0_SCR_POS;
    cr0Reg = (spix->SPICR0.reg & (~SPI_CR0_SCR_MASK)) | temp;
    spix->SPICR0.reg = cr0Reg;
}

/**
  * @brief Get SPI serial clock rate configuration.
  * @param spix SPI register base address.
  * @retval Value range: 0 to 255.
  */
static inline unsigned char DCL_SPI_GetFreqScr(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return ((spix->SPICR0.reg >> SPI_CR0_SCR_POS) & 0xFF); /* Minimum 8-bit mask 0xFF */
}

/**
  * @brief SPI clock divider setting.
  * @param spix SPI register base address.
  * @param freqCpsdvsr The value must be an even number between 2 and 255.
  * @retval None.
  */
static inline void DCL_SPI_SetFreqCpsdvsr(SPI_RegStruct *spix, unsigned char freqCpsdvsr)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    SPI_PARAM_CHECK_NO_RET(IsSpiFreqCpsdvsr(freqCpsdvsr));
    spix->SPICPSR.BIT.cpsdvsr = freqCpsdvsr;
}

/**
  * @brief Get SPI clock divider setting.
  * @param spix SPI register base address.
  * @retval The value is an even number between 2 and 255.
  */
static inline unsigned char DCL_SPI_GetFreqCpsdvsr(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPICPSR.BIT.cpsdvsr;
}

/**
  * @brief Configuring the SPI TX threshold.
  * @param spix SPI register base address.
  * @param txIntSize The value can be 0-15. For details, see the register manual.
  * @retval None.
  */
static inline void DCL_SPI_SetTxIntSize(SPI_RegStruct *spix, unsigned int txIntSize)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    SPI_PARAM_CHECK_NO_RET(IsSpiTxIntSize(txIntSize));
    spix->SPITXFIFOCR.BIT.txintsize = txIntSize;
}

/**
  * @brief Get the SPI TX threshold configuring.
  * @param spix SPI register base address.
  * @retval The value is 0-15. For details, see the register manual.
  */
static inline unsigned char DCL_SPI_GetTxIntSize(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPITXFIFOCR.BIT.txintsize;
}

/**
  * @brief Configuring the SPI RX threshold.
  * @param spix SPI register base address.
  * @param rxIntSize The value can be 0-15. For details, see the register manual.
  * @retval None.
  */
static inline void DCL_SPI_SetRxIntSize(SPI_RegStruct *spix, unsigned int rxIntSize)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    SPI_PARAM_CHECK_NO_RET(IsSpiRxIntSize(rxIntSize));
    spix->SPIRXFIFOCR.BIT.rxintsize = rxIntSize;
}

/**
  * @brief Get the SPI RX threshold configuring.
  * @param spix SPI register base address.
  * @retval The value is 0-15. For details, see the register manual.
  */
static inline unsigned char DCL_SPI_GetRxIntSize(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPIRXFIFOCR.BIT.rxintsize;
}

/**
  * @brief Configuring the CS Channel.
  * @param spix SPI register base address.
  * @param channel SPI chip select channel.
  * @retval None.
  */
static inline void DCL_SPI_SetChipSelect(SPI_RegStruct *spix, unsigned int channel)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    SPI_PARAM_CHECK_NO_RET(IsSpiChipSelectChannel(channel));
    spix->SPICSNCR.BIT.spi_csn_sel = channel;
}

/**
  * @brief Obtains the channel of the current CS.
  * @param spix SPI register base address.
  * @retval SPI_ChipSelectChannel.
  */
static inline unsigned char DCL_SPI_GetChipSelect(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPICSNCR.BIT.spi_csn_sel;
}

/**
  * @brief SPI CHIP CONFIG SET.
  * @param spix SPI register base address.
  * @param mode The value can be 0-4. For details, see the register manual.
  * @retval None.
  */
static inline void DCL_SPI_SetChipConfigSelect(SPI_RegStruct *spix, HAL_SPI_CHIP_CONFIG mode)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    SPI_PARAM_CHECK_NO_RET(IsSpiChipConfigMode(mode));
    spix->SPICSNCR.BIT.spi_csn_mode = mode;
}

/**
  * @brief SPI CHIP CONFIG GET.
  * @param spix SPI register base address.
  * @retval HAL_SPI_CHIP_CONFIG.
  */
static inline HAL_SPI_CHIP_CONFIG DCL_SPI_GetChipConfigSelect(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPICSNCR.BIT.spi_csn_mode;
}

/**
  * @brief Setting the Master/Slave Mode.
  * @param spix SPI register base address.
  * @param mode @ref HAL_SPI_Mode.
  * @retval None.
  */
static inline void DCL_SPI_SetMasterSlaveMode(SPI_RegStruct *spix, HAL_SPI_Mode mode)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    SPI_PARAM_CHECK_NO_RET(IsSpiMode(mode));
    spix->SPICR1.BIT.ms = mode;
}

/**
  * @brief Getting the Master/Slave Mode.
  * @param spix SPI register base address.
  * @retval HAL_SPI_Mode master or slave.
  */
static inline HAL_SPI_Mode DCL_SPI_GetMasterSlaveMode(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPICR1.BIT.ms;
}

/**
  * @brief Set microwire waitval.
  * @param spix SPI register base address.
  * @param value is microwire wait beats.
  * @retval None.
  */
static inline void DCL_SPI_SetMircoWaitVal(SPI_RegStruct *spix, unsigned char value)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    SPI_PARAM_CHECK_NO_RET(IsSpiWaitVal(value));
    spix->SPICR1.BIT.waitval = value;
}

/**
  * @brief Get microwire waitval.
  * @param spix SPI register base address.
  * @retval unsigned char, For details, see the register manual
  */
static inline unsigned char DCL_SPI_GetMircoWaitVal(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPICR1.BIT.waitval;
}

/**
  * @brief Set microwire wait enable or disable.
  * @param spix SPI register base address.
  * @param waitEn is microwire wait enable or disable.
  * @retval None.
  */
static inline void DCL_SPI_SetMircoWaitEn(SPI_RegStruct *spix, bool waitEn)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    spix->SPICR1.BIT.waiten = waitEn;
}

/**
  * @brief Get microwire wait enable or disable.
  * @param spix SPI register base address.
  * @retval bool is microwire wait enable or disable
  */
static inline bool DCL_SPI_GetMircoWaitEn(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPICR1.BIT.waiten;
}

/**
  * @brief Put the data into the TX FIFO.
  * @param spix SPI register base address.
  * @param data is input data.
  * @retval None.
  */
static inline void DCL_SPI_SetData(SPI_RegStruct *spix, unsigned short data)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    spix->SPIDR.data = data;
}

/**
  * @brief Get data from the RX FIFO.
  * @param spix SPI register base address.
  * @retval unsigned short data from the RX FIFO.
  */
static inline unsigned short DCL_SPI_GetData(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPIDR.data;
}

/**
  * @brief Get whether the TX FIFO is empty.
  * @param spix SPI register base address.
  * @retval bool TX FIFO is not empty or is empty.
  */
static inline bool DCL_SPI_GetTxFifoEmpty(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPISR.BIT.tfe;
}

/**
  * @brief Get whether the TX FIFO is full.
  * @param spix SPI register base address.
  * @retval bool TX FIFO is not full or is full.
  */
static inline bool DCL_SPI_GetTxFifoFull(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPISR.BIT.tnf;
}

/**
  * @brief Get whether the RX FIFO is empty.
  * @param spix SPI register base address.
  * @retval bool RX FIFO is not empty or is empty.
  */
static inline bool DCL_SPI_GetRxFifoEmpty(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPISR.BIT.rne;
}

/**
  * @brief Get whether the RX FIFO is full.
  * @param spix SPI register base address.
  * @retval bool RX FIFO is not full or is full.
  */
static inline bool DCL_SPI_GetRxFifoFull(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPISR.BIT.rff;
}

/**
  * @brief Get Whether the SPI is busy.
  * @param spix SPI register base address.
  * @retval bool SPI is busy or not busy.
  */
static inline bool DCL_SPI_GetBusyFlag(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPISR.BIT.bsy;
}

/**
  * @brief Set the interrupt mask.
  * @param spix SPI register base address.
  * @param intMask For details, see the register manual.
  * @retval None.
  */
static inline void DCL_SPI_SetIntMask(SPI_RegStruct *spix, unsigned int intMask)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    spix->SPIIMSC.reg = intMask;
}

/**
  * @brief Get the interrupt mask.
  * @param spix SPI register base address.
  * @retval unsigned int interrupt mask.
  */
static inline unsigned int DCL_SPI_GetIntMask(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPIIMSC.reg;
}

/**
  * @brief Get SPIMIS register all mask interrupt status.
  * @param spix SPI register base address.
  * @retval unsigned short SPIMIS register interrupt mask.
  */
static inline unsigned int DCL_SPI_GetMisInt(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPIMIS.reg;
}

/**
  * @brief Clear RX timeout interrupt
  * @param spix SPI register base address.
  * @retval None.
  */
static inline void DCL_SPI_ClearRxTimeInt(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    spix->SPIICR.BIT.roric = BASE_CFG_SET;
}

/**
  * @brief Clear RX overflow interrupt
  * @param spix SPI register base address.
  * @retval None.
  */
static inline void DCL_SPI_ClearRxOverInt(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    spix->SPIICR.BIT.rtic = BASE_CFG_SET;
}

/**
  * @brief Set DMA FIFO enable register.
  * @param spix SPI register base address.
  * @param dmaCtl control DMA FIFO enable.
  * @retval None.
  */
static inline void DCL_SPI_SetDmaTxFifo(SPI_RegStruct *spix, unsigned int dmaCtl)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    spix->SPIDMACR.reg = dmaCtl;
}

/**
  * @brief Get DMA FIFO enable register status.
  * @param spix SPI register base address.
  * @retval unsigned int DMA Control Register Status.
  */
static inline unsigned int DCL_SPI_GetDmaTxFifo(SPI_RegStruct *spix)
{
    SPI_ASSERT_PARAM(IsSPIInstance(spix));
    return spix->SPIDMACR.reg;
}

/**
  * @}
  */

/**
  * @}
  */
#endif /* #ifndef McuMagicTag_SPI_IP_H */