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
  * @file      spi.h
  * @author    MCU Driver Team
  * @brief     SPI module driver.
  * @details   This file provides firmware functions to manage the following.
  *            functionalities of the SPI.
  *            + Initialization and de-initialization functions.
  *            + Peripheral transmit and receiving functions.
  *            + Enumerated definition of SPI basic parameter configuration.
  */
#ifndef McuMagicTag_SPI_H
#define McuMagicTag_SPI_H

/* Includes ------------------------------------------------------------------*/
#include "dma.h"
#include "spi_ip.h"

/**
  * @defgroup SPI SPI
  * @brief SPI module.
  * @{
  */

 /**
  * @defgroup SPI_Common SPI Common
  * @brief SPI common external module.
  * @{
  */

/* Macro definitions ---------------------------------------------------------*/

/* Definition of the chip selection configuration macro */
#define SPI_CHIP_DESELECT                0
#define SPI_CHIP_SELECT                  1

/* Definition of the chip selection mode selection macro */
#define SPI_CHIP_SELECT_MODE_INTERNAL    0
#define SPI_CHIP_SELECT_MODE_CALLBACK    1

/**
  * @defgroup SPI_Handle_Definition SPI Handle Definition
  * @{
  */

/* Typedef definitions -------------------------------------------------------*/
/**
 * @brief Callback Function ID Enumeration Definition.
 */
typedef enum {
    SPI_TX_COMPLETE_CB_ID    = 0x00000000U,
    SPI_RX_COMPLETE_CB_ID    = 0x00000001U,
    SPI_TX_RX_COMPLETE_CB_ID = 0x00000002U,
    SPI_ERROR_CB_ID          = 0x00000003U,
    SPI_CS_CB_ID             = 0x00000004U
} HAL_SPI_CallbackID;

/**
 * @brief Module Status Enumeration Definition.
 */
typedef enum {
    HAL_SPI_STATE_RESET      = 0x00000000U,    /**< Peripheral not Initialized                         */
    HAL_SPI_STATE_READY      = 0x00000001U,    /**< Peripheral Initialized and ready for use           */
    HAL_SPI_STATE_BUSY       = 0x00000002U,    /**< An internal process is ongoing                     */
    HAL_SPI_STATE_BUSY_TX    = 0x00000003U,    /**< Data Transmission process is ongoing               */
    HAL_SPI_STATE_BUSY_RX    = 0x00000004U,    /**< Data Reception process is ongoing                  */
    HAL_SPI_STATE_BUSY_TX_RX = 0x00000005U,    /**< Data Transmission and Reception process is ongoing */
    HAL_SPI_STATE_ERROR      = 0x00000006U,    /**< SPI error state                                    */
    HAL_SPI_STATE_ABORT      = 0x00000007U     /**< SPI abort is ongoing                               */
} HAL_SPI_State;

/**
 * @brief Module handle structure definition.
 */
typedef struct _SPI_Handle {
    SPI_RegStruct    *baseAddress;        /**< Register base address. */

    unsigned int      mode;               /**< See HAL_SPI_Mode. */
    unsigned int      csMode;             /**< SPI_CHIP_SELECT_MODE_INTERNAL or SPI_CHIP_SELECT_MODE_CALLBACK. */
    unsigned int      xFerMode;           /**< See HAL_SPI_XferMode. */
    unsigned int      clkPolarity;        /**< See HAL_SPI_ClkPol. */
    unsigned int      clkPhase;           /**< See HAL_SPI_ClkPha. */
    unsigned int      endian;             /**< See HAL_SPI_Endian. */
    unsigned int      frameFormat;        /**< See HAL_SPI_FrameMode. */
    unsigned int      dataWidth;          /**< See HAL_SPI_DataWidth. */
    unsigned char     freqScr;            /**< Frequency scr, value range: 0 to 255. */
    unsigned char     freqCpsdvsr;        /**< Frequency Cpsdvsr, an even number ranging from 0 to 254. */
    unsigned char     waitVal;            /**< Number of beats waiting between write and read in National
                                               Microwire frame format. */
    bool              waitEn;             /**< SPI Microwire waiting enable. */
    unsigned int      txIntSize;          /**< TX interrupt transmission threshold. */
    unsigned int      rxIntSize;          /**< RX interrupt transmission threshold. */

    unsigned int      txDMABurstSize;     /**< TX DMA transmission threshold. */
    unsigned int      rxDMABurstSize;     /**< RX DMA transmission threshold. */
    DMA_Handle       *dmaHandle;          /**< SPI_DMA control handle*/
    unsigned int      txDmaCh;            /**< SPI DMA tx channel */
    unsigned int      rxDmaCh;            /**< SPI DMA rx channel */

    unsigned int      csCtrl;             /**< Chip select status. */
    unsigned char    *rxBuff;             /**< Rx buffer pointer address. */
    unsigned char    *txBuff;             /**< Tx buffer pointer address. */
    unsigned int      transferSize;       /**< Total length of transmitted data. */
    unsigned int      txCount;            /**< Tx Length of data transferred. */
    unsigned int      rxCount;            /**< Rx Length of data transferred. */

    HAL_SPI_State     state;              /**< Running Status. */
    BASE_StatusType   errorCode;          /**< Error Code. */
    SPI_UserCallBack  userCallBack;       /**< User callback. */
    SPI_ExtendHandle  handleEx;           /**< SPI extend parameter. */
} SPI_Handle;
/**
  * @}
  */

/**
 * @brief Callback Function Type Definition.
 */
typedef void (* SPI_CallbackFuncType)(void *handle);

/**
  * @defgroup SPI_API_Declaration SPI HAL API
  * @{
  */

BASE_StatusType HAL_SPI_Init(SPI_Handle *handle);
BASE_StatusType HAL_SPI_Deinit(SPI_Handle *handle);
BASE_StatusType HAL_SPI_ConfigParameter(SPI_Handle *handle);
BASE_StatusType HAL_SPI_RegisterCallback(SPI_Handle *handle,
                                         HAL_SPI_CallbackID callbackID,
                                         SPI_CallbackFuncType pcallback);
BASE_StatusType HAL_SPI_ReadBlocking(SPI_Handle *handle,
                                     unsigned char *rData,
                                     unsigned int dataSize,
                                     unsigned int timeout);
BASE_StatusType HAL_SPI_WriteBlocking(SPI_Handle *handle,
                                      unsigned char *wData,
                                      unsigned int dataSize,
                                      unsigned int timeout);
BASE_StatusType HAL_SPI_WriteReadBlocking(SPI_Handle *handle,
                                          unsigned char *rData,
                                          unsigned char *wData,
                                          unsigned int dataSize,
                                          unsigned int timeout);
BASE_StatusType HAL_SPI_ReadIT(SPI_Handle *handle, unsigned char *rData, unsigned int dataSize);
BASE_StatusType HAL_SPI_WriteIT(SPI_Handle *handle, unsigned char *wData, unsigned int dataSize);
BASE_StatusType HAL_SPI_WriteReadIT(SPI_Handle *handle,
                                    unsigned char *rData,
                                    unsigned char *wData,
                                    unsigned int dataSizeout);
BASE_StatusType HAL_SPI_ReadDMA(SPI_Handle *handle, unsigned char *rData, unsigned int dataSize);
BASE_StatusType HAL_SPI_WriteDMA(SPI_Handle *handle, unsigned char *wData, unsigned int dataSize);
BASE_StatusType HAL_SPI_WriteReadDMA(SPI_Handle *handle,
                                     unsigned char *rData,
                                     unsigned char *wData,
                                     unsigned int dataSizeout);
BASE_StatusType HAL_SPI_DMAStop(SPI_Handle *handle);
BASE_StatusType HAL_SPI_ChipSelectChannelSet(SPI_Handle *handle, SPI_ChipSelectChannel channel);
BASE_StatusType HAL_SPI_ChipSelectChannelGet(SPI_Handle *handle, SPI_ChipSelectChannel *channel);
void HAL_SPI_IrqHandler(void *handle);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* #ifndef McuMagicTag_SPI_H */