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
  * @file    spi.c
  * @author  MCU Driver Team
  * @brief   SPI module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the SPI.
  *          + Initialization and de-initialization functions
  *          + Peripheral Control functions
  */

/* Includes ------------------------------------------------------------------*/
#include "interrupt.h"
#include "systick.h"
#include "spi.h"
/* Macro definitions ---------------------------------------------------------*/
#define SPI_WAIT_TIMEOUT   0x400

#define SPI_DATA_WIDTH_SHIFT_8BIT    1
#define SPI_DATA_WIDTH_SHIFT_16BIT   2
#define SPI_WRITE_FIFO_SIZE 2

#define SPI_INTERRUPT_SET_ALL 0xF
#define SPI_DMA_FIFO_ENABLE   0x3

#define SPI_TICK_MS_DIV       1000
/**
  * @brief Check all initial configuration parameters.
  * @param handle SPI handle.
  * @retval BASE status type: OK, ERROR.
  */
static BASE_StatusType CheckAllInitParameters(SPI_Handle *handle)
{
    SPI_PARAM_CHECK_WITH_RET(IsSpiMode(handle->mode), BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(IsSpiXferMode(handle->xFerMode), BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(IsSpiEndian(handle->endian), BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(IsSpiFrameFormat(handle->frameFormat), BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(IsSpiDataWidth(handle->dataWidth), BASE_STATUS_ERROR);
    /* Check spi freqCpsdvsr */
    if (handle->mode == HAL_SPI_MASTER) {
        SPI_PARAM_CHECK_WITH_RET(IsSpiFreqCpsdvsr(handle->freqCpsdvsr), BASE_STATUS_ERROR);
    }
    /* Check motorola clkPolarity and clkPhase */
    if (handle->frameFormat == HAL_SPI_MODE_MOTOROLA) {
        SPI_PARAM_CHECK_WITH_RET(IsSpiClkPolarity(handle->clkPolarity), BASE_STATUS_ERROR);
        SPI_PARAM_CHECK_WITH_RET(IsSpiClkPhase(handle->clkPhase), BASE_STATUS_ERROR);
    }
    /* Check microwire waitVal */
    if (handle->frameFormat == HAL_SPI_MODE_MICROWIRE) {
        SPI_PARAM_CHECK_WITH_RET(IsSpiWaitVal(handle->waitVal), BASE_STATUS_ERROR);
    }
    /* Check tx rx interrupt size */
    if (handle->xFerMode == HAL_XFER_MODE_INTERRUPTS) {
        SPI_PARAM_CHECK_WITH_RET(IsSpiTxIntSize(handle->txIntSize), BASE_STATUS_ERROR);
        SPI_PARAM_CHECK_WITH_RET(IsSpiRxIntSize(handle->rxIntSize), BASE_STATUS_ERROR);
    }
    /* Check tx rx dma burst size */
    if (handle->xFerMode == HAL_XFER_MODE_DMA) {
        SPI_PARAM_CHECK_WITH_RET(IsSpiTxDmaBurstSize(handle->txDMABurstSize), BASE_STATUS_ERROR);
        SPI_PARAM_CHECK_WITH_RET(IsSpiRxDmaBurstSize(handle->rxDMABurstSize), BASE_STATUS_ERROR);
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Configuring the Register Parameters of the Three Transfer Modes.
  * @param handle SPI handle.
  * @retval BASE status type: OK, ERROR.
  */
static BASE_StatusType ConfigThreeTransferParam(SPI_Handle *handle)
{
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    /* Configurations related to the three transmission modes */
    if (handle->xFerMode == HAL_XFER_MODE_BLOCKING) {
        handle->baseAddress->SPIIMSC.reg = 0x0;
    } else if (handle->xFerMode == HAL_XFER_MODE_INTERRUPTS) {
        handle->baseAddress->SPIIMSC.reg = SPI_INTERRUPT_SET_ALL;
        /* Setting the rx and tx interrupt transfer size */
        handle->baseAddress->SPITXFIFOCR.BIT.txintsize = handle->txIntSize;
        handle->baseAddress->SPIRXFIFOCR.BIT.rxintsize = handle->rxIntSize;
    } else if (handle->xFerMode == HAL_XFER_MODE_DMA) {
        handle->baseAddress->SPIIMSC.reg = 0x0;
        /* Setting the DMA rx and tx burst transfer size */
        handle->baseAddress->SPITXFIFOCR.BIT.txintsize = handle->txDMABurstSize;
        handle->baseAddress->SPIRXFIFOCR.BIT.rxintsize = handle->rxDMABurstSize;
    } else {
        /* xFerMode set error */
        handle->errorCode = BASE_STATUS_ERROR;
        handle->state = HAL_SPI_STATE_RESET;
        return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Internal chip select control.
  * @param handle SPI handle.
  * @param control SPI_CHIP_DESELECT or SPI_CHIP_SELECT
  * @retval None.
  */
static void InternalCsControl(SPI_Handle *handle, unsigned int control)
{
    BASE_FUNC_UNUSED(handle);
    BASE_FUNC_UNUSED(control);
}

/**
  * @brief Chip select control.
  * @param handle SPI handle.
  * @param control SPI_CHIP_DESELECT or SPI_CHIP_SELECT
  * @retval None.
  */
static void SpiCsControl(SPI_Handle *handle, unsigned int control)
{
    /* The chip select signal is determined by the chip logic. */
    if (handle->csMode == SPI_CHIP_SELECT_MODE_INTERNAL) {
        InternalCsControl(handle, control);
    } else {
        /* The chip select signal is determined by callback */
        if (handle->userCallBack.CsCtrlCallback != NULL) {
            handle->csCtrl = control;
            handle->userCallBack.CsCtrlCallback(handle);
        }
    }
}

/**
  * @brief Invoke rx tx callback function.
  * @param handle SPI handle.
  * @retval None.
  */
static void SpiRxTxCallack(void *handle)
{
    SPI_Handle *spiHandle = (SPI_Handle *) handle;
    SPI_ASSERT_PARAM(spiHandle != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(spiHandle->baseAddress));
    if (spiHandle->txCount == spiHandle->transferSize) {
        /* Invoke tx callback function. */
        if (spiHandle->userCallBack.TxCpltCallback != NULL) {
            spiHandle->userCallBack.TxCpltCallback(spiHandle);
        }
        spiHandle->baseAddress->SPIIMSC.BIT.txim = 0x0;
    }

    if (spiHandle->rxCount >= spiHandle->transferSize) {
        /* Disable all interrupt */
        spiHandle->baseAddress->SPIIMSC.reg = 0x0;
        /* Clear all interrupt */
        spiHandle->baseAddress->SPIICR.BIT.roric = BASE_CFG_SET;
        spiHandle->baseAddress->SPIICR.BIT.rtic = BASE_CFG_SET;

        SpiCsControl(spiHandle, SPI_CHIP_DESELECT);

        /* Invoke rx callback function. */
        if (spiHandle->userCallBack.RxCpltCallback != NULL) {
            spiHandle->userCallBack.RxCpltCallback(spiHandle);
        }
        /* Invoke tx rx callback function. */
        if (spiHandle->userCallBack.TxRxCpltCallback != NULL) {
            spiHandle->userCallBack.TxRxCpltCallback(spiHandle);
        }
        spiHandle->state = HAL_SPI_STATE_READY;
    }
}

/**
  * @brief Writes data from the buffer to the FIFO.
  * @param handle SPI handle.
  * @retval None.
  */
static void WriteData(SPI_Handle *handle)
{
    unsigned int size = 0;
    while ((size < SPI_WRITE_FIFO_SIZE) && (handle->baseAddress->SPISR.BIT.tnf) &&
           (handle->transferSize > handle->txCount)) {
        if (handle->dataWidth > SPI_DATA_WIDTH_8BIT) {
            /* Only data needs to be read. Due to SPI characteristics,
               data must be transmitted before data can be read. Therefore, 0x0 is transmitted. */
            if (handle->txBuff == NULL) {
                handle->baseAddress->SPIDR.data = 0x0;
                handle->txCount += SPI_DATA_WIDTH_SHIFT_16BIT;
            } else {
                handle->baseAddress->SPIDR.data = *(unsigned short *)handle->txBuff;
                handle->txCount += SPI_DATA_WIDTH_SHIFT_16BIT; /* txCount is number of bytes transferred */
                handle->txBuff += SPI_DATA_WIDTH_SHIFT_16BIT;
            }
        } else { /* datawidth is 8bit */
            if (handle->txBuff == NULL) {
                handle->baseAddress->SPIDR.data = 0x0;
                handle->txCount += SPI_DATA_WIDTH_SHIFT_8BIT;
            } else {
                handle->baseAddress->SPIDR.data = *(unsigned char *)handle->txBuff;
                handle->txCount += SPI_DATA_WIDTH_SHIFT_8BIT; /* txCount is number of bytes transferred */
                handle->txBuff += SPI_DATA_WIDTH_SHIFT_8BIT;
            }
        }
        size++;
    }
}

/**
  * @brief Reads data from the FIFO to the buffer.
  * @param handle SPI handle.
  * @retval None.
  */
static void ReadData(SPI_Handle *handle)
{
    unsigned short val;

    while ((handle->baseAddress->SPISR.BIT.rne) && (handle->transferSize > handle->rxCount)) {
        if (handle->dataWidth > SPI_DATA_WIDTH_8BIT) {
            /* When only data is transmitted, the data in the RX FIFO needs to be read. */
            if (handle->rxBuff == NULL) {
                val = handle->baseAddress->SPIDR.data;
                BASE_FUNC_UNUSED(val);
                handle->rxCount += SPI_DATA_WIDTH_SHIFT_16BIT;
            } else {
                *(unsigned short *)handle->rxBuff = handle->baseAddress->SPIDR.data;
                handle->rxCount += SPI_DATA_WIDTH_SHIFT_16BIT;
                handle->rxBuff += SPI_DATA_WIDTH_SHIFT_16BIT;
            }
        } else { /* datawidth is 8bit */
            if (handle->rxBuff == NULL) {
                val = handle->baseAddress->SPIDR.data;
                BASE_FUNC_UNUSED(val);
                handle->rxCount += SPI_DATA_WIDTH_SHIFT_8BIT;
            } else {
                *(unsigned char *)handle->rxBuff = handle->baseAddress->SPIDR.data & 0xff;
                handle->rxCount += SPI_DATA_WIDTH_SHIFT_8BIT;
                handle->rxBuff += SPI_DATA_WIDTH_SHIFT_8BIT;
            }
        }
    }
}

/**
  * @brief Read/write based on input parameters.
  *        The Motorola SPI/TI synchronous serial interface is full-duplex.
  *        Each data is received. Even if only data needs to be transmitted,
  *        the RX FIFO needs to be cleared.
  * @param handle SPI handle.
  * @retval None.
  */
static void ReadWriteData(SPI_Handle *handle)
{
    unsigned int preTick = DCL_SYSTICK_GetTick();
    unsigned int curTick = preTick;
    unsigned long long delta = 0;
    /* Calculate the timeout tick. */
    unsigned long long targetDelta = HAL_CRG_GetIpFreq(SYSTICK_BASE) / SPI_TICK_MS_DIV * SPI_WAIT_TIMEOUT;
    WriteData(handle);
    while (true) {
        /* Wait for the write operation to complete */
        if (handle->baseAddress->SPISR.BIT.bsy == BASE_CFG_UNSET &&
            handle->baseAddress->SPISR.BIT.tfe == BASE_CFG_SET &&
            handle->baseAddress->SPISR.BIT.rne == BASE_CFG_SET) {
                break;
        }
        curTick = DCL_SYSTICK_GetTick();
        delta += curTick > preTick ? curTick - preTick : SYSTICK_MAX_VALUE - preTick + curTick;
        /* Exit upon timeout */
        if (delta >= targetDelta) {
            handle->errorCode = BASE_STATUS_TIMEOUT;
            break;
        }
        preTick = curTick;
    }
    ReadData(handle);
}

/**
  * @brief Reads and writes data based on the interrupt flag.
  * @param handle SPI handle.
  * @retval None.
  */
static void ReadWriteInt(SPI_Handle *handle)
{
    /* 0x02 Receive timeout interrupt, 0x04 receive FIFO interrupt */
    if ((handle->mode == HAL_SPI_SLAVE) &&
        ((handle->baseAddress->SPIMIS.reg == 0x04) ||
        (handle->baseAddress->SPIMIS.reg == 0x02))) {
        ReadData(handle);
    } else {   /* master mode */
        if (handle->baseAddress->SPIMIS.BIT.rxmis || handle->baseAddress->SPIMIS.BIT.rtmis) {
            ReadData(handle);
        }
        if (handle->baseAddress->SPIMIS.BIT.txmis) {
            WriteData(handle);
        }
    }
}

/**
  * @brief Blocking read data processing.
  * @param handle SPI handle.
  * @param timeout Timeout period,unit: ms.
  * @retval None.
  */
static void ReadBlocking(SPI_Handle *handle, unsigned int timeout)
{
    unsigned int preTick = DCL_SYSTICK_GetTick();
    unsigned int curTick = preTick;
    unsigned long long delta = 0;
    unsigned long long targetDelta = HAL_CRG_GetIpFreq(SYSTICK_BASE) / SPI_TICK_MS_DIV * timeout;

    /* Pull down the CS before transmitting data. */
    SpiCsControl(handle, SPI_CHIP_SELECT);
    if (!handle->baseAddress->SPICR1.BIT.sse) {
        handle->baseAddress->SPICR1.BIT.sse = BASE_CFG_SET; /* spi enable */
    }
    while (handle->transferSize > handle->rxCount) {
        curTick = DCL_SYSTICK_GetTick();
        delta += curTick > preTick ? curTick - preTick : SYSTICK_MAX_VALUE - preTick + curTick;
        if (delta >= targetDelta) { /* The configured timeout period is exceeded. */
            handle->errorCode = BASE_STATUS_TIMEOUT;
            break;
        }
        ReadData(handle);
        preTick = curTick;
    }
    /* Pull up the CS after transmitting data. */
    SpiCsControl(handle, SPI_CHIP_DESELECT);
    handle->state = HAL_SPI_STATE_READY;
}

/**
  * @brief Blocking read/write data processing.
  * @param handle SPI handle.
  * @param timeout Timeout period,unit: ms.
  * @retval None.
  */
static void ReadWriteBlocking(SPI_Handle *handle, unsigned int timeout)
{
    unsigned int preTick = DCL_SYSTICK_GetTick();
    unsigned int curTick = preTick;
    unsigned long long delta = 0;
    unsigned long long targetDelta = HAL_CRG_GetIpFreq(SYSTICK_BASE) / SPI_TICK_MS_DIV * timeout;
    /* Pull down the CS before transmitting data. */
    SpiCsControl(handle, SPI_CHIP_SELECT);
    if (!handle->baseAddress->SPICR1.BIT.sse) {
        handle->baseAddress->SPICR1.BIT.sse = BASE_CFG_SET; /* spi enable */
    }

    while (handle->transferSize > handle->txCount || handle->transferSize > handle->rxCount) {
        curTick = DCL_SYSTICK_GetTick();
        delta += curTick > preTick ? curTick - preTick : SYSTICK_MAX_VALUE - preTick + curTick;
        if (delta >= targetDelta) { /* The configured timeout period is exceeded. */
            handle->errorCode = BASE_STATUS_TIMEOUT;
            break;
        }
        ReadWriteData(handle);
        preTick = curTick;
    }
    /* Pull up the CS after transmitting data. */
    SpiCsControl(handle, SPI_CHIP_DESELECT);
    handle->state = HAL_SPI_STATE_READY;
}

/**
 * @brief SPI read/write parameter configuration.
 * @param handle SPI handle.
 * @param rData Address of the data buff to be Receiving.
 * @param wData Address of the data buff to be sent.
 * @param dataSiz Number of the data to be Receivingd and sent.
 * @retval None.
 */
static void ConfigTransmissionParameter(SPI_Handle *handle,
                                        unsigned char *rData,
                                        unsigned char *wData,
                                        unsigned int dataSize)
{
    handle->errorCode = BASE_STATUS_OK;
    handle->rxBuff = rData;
    handle->txBuff = wData;
    if (handle->dataWidth > SPI_DATA_WIDTH_8BIT &&
        handle->xFerMode == HAL_XFER_MODE_DMA) {
        handle->transferSize = dataSize / 2; /* Processes 2 bytes at a time */
    } else {
        handle->transferSize = dataSize;
    }
    handle->txCount = 0;
    handle->rxCount = 0;
}

/**
 * @brief SPI Clear Rx Fifo.
 * @param handle SPI handle.
 * @retval None.
 */
static void ClearSpiRxFifo(SPI_Handle *handle)
{
    /* Invalid data in the RX FIFO, Clearing the RX FIFO. */
    unsigned short val;
    while (handle->baseAddress->SPISR.BIT.rne) {
        val = handle->baseAddress->SPIDR.data;
        BASE_FUNC_UNUSED(val);
    }
}

/**
  * @brief Initializing the SPI Module.
  * @param handle SPI handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_Init(SPI_Handle *handle)
{
    unsigned int cr0Reg;
    unsigned int temp;
    unsigned char frCps;
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    /* Check whether initialization parameters are correctly set */
    if (CheckAllInitParameters(handle) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }

    handle->state = HAL_SPI_STATE_BUSY;

    handle->baseAddress->SPICR1.BIT.lbm = BASE_CFG_UNSET;
    handle->baseAddress->SPICR1.BIT.sse = BASE_CFG_UNSET;
    handle->baseAddress->SPICR1.BIT.bitend = handle->endian; /* Setting the endian mode */
    handle->baseAddress->SPICR1.BIT.ms = handle->mode;

    temp = ((unsigned int)handle->freqScr) << SPI_CR0_SCR_POS;
    cr0Reg = (handle->baseAddress->SPICR0.reg & (~SPI_CR0_SCR_MASK)) | temp;
    handle->baseAddress->SPICR0.reg = cr0Reg;
    
    if (handle->mode == HAL_SPI_MASTER) {
        frCps = handle->freqCpsdvsr;
        /* Modulo 2 to get an even number */
        handle->baseAddress->SPICPSR.BIT.cpsdvsr = ((frCps % 2) == 0 ? frCps : frCps - 1);
    }
    if (handle->frameFormat == HAL_SPI_MODE_MOTOROLA) {
        handle->baseAddress->SPICR0.BIT.sph = handle->clkPhase;
        handle->baseAddress->SPICR0.BIT.spo = handle->clkPolarity;
    }
    
    handle->baseAddress->SPICR0.BIT.frf = handle->frameFormat;
    handle->baseAddress->SPICR0.BIT.dss = handle->dataWidth;

    /* Indicates whether to enable the Microwire wait period. */
    if ((handle->frameFormat == HAL_SPI_MODE_MICROWIRE) && (handle->waitEn == BASE_CFG_ENABLE)) {
        handle->baseAddress->SPICR1.BIT.waitval = handle->waitVal;
        handle->baseAddress->SPICR1.BIT.waiten = BASE_CFG_SET;
    } else {
        handle->baseAddress->SPICR1.BIT.waiten = BASE_CFG_UNSET;
    }

    if (ConfigThreeTransferParam(handle) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    handle->state = HAL_SPI_STATE_READY;
    return BASE_STATUS_OK;
}

/**
  * @brief Deinitialize the SPI module.
  * @param handle SPI handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_Deinit(SPI_Handle *handle)
{
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));

    handle->state = HAL_SPI_STATE_BUSY;

    /* Disable rx and tx DMA, SPI disable */
    handle->baseAddress->SPIIMSC.reg = 0x0;
    handle->baseAddress->SPIDMACR.BIT.rxdmae = BASE_CFG_UNSET;
    handle->baseAddress->SPIDMACR.BIT.txdmae = BASE_CFG_UNSET;
    handle->baseAddress->SPICR1.BIT.sse = BASE_CFG_UNSET;
    handle->state = HAL_SPI_STATE_RESET;
    /* Clean callback */
    handle->userCallBack.TxCpltCallback = NULL;
    handle->userCallBack.RxCpltCallback = NULL;
    handle->userCallBack.TxRxCpltCallback = NULL;
    handle->userCallBack.ErrorCallback = NULL;
    handle->userCallBack.CsCtrlCallback = NULL;
    return BASE_STATUS_OK;
}

/**
  * @brief SPI Parameter Configuration.
  * @param handle SPI handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_ConfigParameter(SPI_Handle *handle)
{
    unsigned int cr0Reg;
    unsigned int temp;
    unsigned char frCps;
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    if (CheckAllInitParameters(handle) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }

    handle->baseAddress->SPICR1.BIT.sse = BASE_CFG_UNSET;
    handle->baseAddress->SPICR1.BIT.ms = handle->mode;
    handle->baseAddress->SPICR0.BIT.frf = handle->frameFormat;
    handle->baseAddress->SPICR0.BIT.dss = handle->dataWidth;
    handle->baseAddress->SPICR1.BIT.bitend = handle->endian;
    /* Set freqScr */
    temp = ((unsigned int)handle->freqScr) << SPI_CR0_SCR_POS;
    cr0Reg = (handle->baseAddress->SPICR0.reg & (~SPI_CR0_SCR_MASK)) | temp;
    handle->baseAddress->SPICR0.reg = cr0Reg;

    if (handle->mode == HAL_SPI_MASTER) {
        frCps = handle->freqCpsdvsr;
        /* Modulo 2 to get an even number */
        handle->baseAddress->SPICPSR.BIT.cpsdvsr = ((frCps % 2) == 0 ? frCps : frCps - 1);
    }
    if (handle->frameFormat == HAL_SPI_MODE_MOTOROLA) {
        handle->baseAddress->SPICR0.BIT.sph = handle->clkPhase;
        handle->baseAddress->SPICR0.BIT.spo = handle->clkPolarity;
    }

    if (handle->frameFormat == HAL_SPI_MODE_MICROWIRE) {
        handle->baseAddress->SPICR1.BIT.waitval = handle->waitVal;
    }

    /* Setting the Interrupt Thresholds */
    if (handle->xFerMode == HAL_XFER_MODE_INTERRUPTS) {
        handle->baseAddress->SPITXFIFOCR.BIT.txintsize = handle->txIntSize;
        handle->baseAddress->SPIRXFIFOCR.BIT.rxintsize = handle->rxIntSize;
    } else if (handle->xFerMode == HAL_XFER_MODE_DMA) {
        handle->baseAddress->SPITXFIFOCR.BIT.txintsize = handle->txDMABurstSize;
        handle->baseAddress->SPIRXFIFOCR.BIT.rxintsize = handle->rxDMABurstSize;
    } else {
        ;
    }

    return BASE_STATUS_OK;
}

/**
  * @brief Callback Function Registration.
  * @param handle SPI handle.
  * @param callbackID Callback function ID..
  * @param pcallback Pointer to the address of the registered callback function.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_RegisterCallback(SPI_Handle *handle,
                                         HAL_SPI_CallbackID callbackID,
                                         SPI_CallbackFuncType pcallback)
{
    BASE_StatusType ret = BASE_STATUS_OK;
    SPI_ASSERT_PARAM(handle != NULL && pcallback != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));

    if (handle->state == HAL_SPI_STATE_READY) {
        switch (callbackID) {
            case SPI_TX_COMPLETE_CB_ID :
                handle->userCallBack.TxCpltCallback = pcallback;
                break;
            case SPI_RX_COMPLETE_CB_ID :
                handle->userCallBack.RxCpltCallback = pcallback;
                break;
            case SPI_TX_RX_COMPLETE_CB_ID :
                handle->userCallBack.TxRxCpltCallback = pcallback;
                break;
            case SPI_ERROR_CB_ID :
                handle->userCallBack.ErrorCallback = pcallback;
                break;
            case SPI_CS_CB_ID:
                handle->userCallBack.CsCtrlCallback = pcallback;
                break;
            default :
                handle->errorCode = BASE_STATUS_ERROR;
                ret = BASE_STATUS_ERROR;
                break;
        }
    } else {
        handle->errorCode = BASE_STATUS_ERROR;
        ret = BASE_STATUS_ERROR;
    }
    return ret;
}

/**
  * @brief Receiving data in blocking mode.
  * @param handle SPI handle.
  * @param rData Address of the data buff to be Receiving.
  * @param dataSize Number of the data to be Receiving.
  * @param timeout Timeout period,unit: ms.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_ReadBlocking(SPI_Handle *handle,
                                     unsigned char *rData,
                                     unsigned int dataSize,
                                     unsigned int timeout)
{
    SPI_ASSERT_PARAM(handle != NULL && rData != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    SPI_PARAM_CHECK_WITH_RET(handle->state == HAL_SPI_STATE_READY, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);

    handle->state = HAL_SPI_STATE_BUSY_RX;
    /* Configuring SPI transmission parameters */
    ConfigTransmissionParameter(handle, rData, NULL, dataSize);
    ClearSpiRxFifo(handle); /* If there is residual data in the read FIFO, clear the data. */
    if (handle->mode == HAL_SPI_MASTER) {
        ReadWriteBlocking(handle, timeout);
    } else {
        ReadBlocking(handle, timeout);
    }

    if (handle->errorCode != BASE_STATUS_OK) {
        if (handle->userCallBack.ErrorCallback != NULL) {
            handle->userCallBack.ErrorCallback(handle);
        }
        return handle->errorCode;
    }
    if (handle->userCallBack.RxCpltCallback != NULL) {
        handle->userCallBack.RxCpltCallback(handle);
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Send data in blocking mode.
  * @param handle SPI handle.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be sent.
  * @param timeout Timeout period,unit: ms.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_WriteBlocking(SPI_Handle *handle,
                                      unsigned char *wData,
                                      unsigned int dataSize,
                                      unsigned int timeout)
{
    SPI_ASSERT_PARAM(handle != NULL && wData != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));

    SPI_PARAM_CHECK_WITH_RET(handle->state == HAL_SPI_STATE_READY, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);

    handle->state = HAL_SPI_STATE_BUSY_TX;
    /* Configuring SPI transmission parameters */
    ConfigTransmissionParameter(handle, NULL, wData, dataSize);
    ClearSpiRxFifo(handle); /* If there is residual data in the read FIFO, clear the data. */
    ReadWriteBlocking(handle, timeout);
    if (handle->errorCode != BASE_STATUS_OK) {
        if (handle->userCallBack.ErrorCallback != NULL) {
            handle->userCallBack.ErrorCallback(handle);
        }
        return handle->errorCode;
    }
    if (handle->userCallBack.TxCpltCallback != NULL) {
        handle->userCallBack.TxCpltCallback(handle);
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Receiving and send data in blocking mode.
  * @param handle SPI handle.
  * @param rData Address of the data buff to be Receiving.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be Receivingd and sent.
  * @param timeout Timeout period,unit: ms.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_WriteReadBlocking(SPI_Handle *handle,
                                          unsigned char *rData,
                                          unsigned char *wData,
                                          unsigned int dataSize,
                                          unsigned int timeout)
{
    SPI_ASSERT_PARAM(handle != NULL && rData != NULL && wData != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));

    SPI_PARAM_CHECK_WITH_RET(handle->state == HAL_SPI_STATE_READY, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);

    handle->state = HAL_SPI_STATE_BUSY_TX_RX;
    /* Configuring SPI transmission parameters */
    ConfigTransmissionParameter(handle, rData, wData, dataSize);
    ClearSpiRxFifo(handle); /* If there is residual data in the read FIFO, clear the data. */
    ReadWriteBlocking(handle, timeout);
    if (handle->errorCode != BASE_STATUS_OK) {
        if (handle->userCallBack.ErrorCallback != NULL) {
            handle->userCallBack.ErrorCallback(handle);
        }
        return handle->errorCode;
    }
    if (handle->userCallBack.TxRxCpltCallback != NULL) {
        handle->userCallBack.TxRxCpltCallback(handle);
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Receiving data in interrupts mode.
  * @param handle SPI handle.
  * @param rData Address of the data buff to be Receiving.
  * @param dataSize Number of the data to be Receiving.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_ReadIT(SPI_Handle *handle, unsigned char *rData, unsigned int dataSize)
{
    SPI_ASSERT_PARAM(handle != NULL && rData != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));

    SPI_PARAM_CHECK_WITH_RET(handle->state == HAL_SPI_STATE_READY, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);

    handle->state = HAL_SPI_STATE_BUSY_RX;
    ConfigTransmissionParameter(handle, rData, NULL, dataSize);
    ClearSpiRxFifo(handle); /* If there is residual data in the read FIFO, clear the data. */
    SpiCsControl(handle, SPI_CHIP_SELECT);
    if (!handle->baseAddress->SPICR1.BIT.sse) {
        handle->baseAddress->SPICR1.BIT.sse = BASE_CFG_SET;
    }
    /* Enable related interrupts. */
    if (handle->mode == HAL_SPI_MASTER) {
        /* 0x0F indicate enables all interrupt. */
        handle->baseAddress->SPIIMSC.reg = 0x0F;
    } else {
        /* 0x07 indicate enables the RX FIFO, RX timeout, and RX overflow interrupt. */
        handle->baseAddress->SPIIMSC.reg = 0x07;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Send data in interrupts mode.
  * @param handle SPI handle.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be sent.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_WriteIT(SPI_Handle *handle, unsigned char *wData, unsigned int dataSize)
{
    SPI_ASSERT_PARAM(handle != NULL && wData != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));

    SPI_PARAM_CHECK_WITH_RET(handle->state == HAL_SPI_STATE_READY, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);

    handle->state = HAL_SPI_STATE_BUSY_TX;
    ConfigTransmissionParameter(handle, NULL, wData, dataSize);
    ClearSpiRxFifo(handle); /* If there is residual data in the read FIFO, clear the data. */
    /* interrupt enable */
    SpiCsControl(handle, SPI_CHIP_SELECT);
    if (!handle->baseAddress->SPICR1.BIT.sse) {
        handle->baseAddress->SPICR1.BIT.sse = BASE_CFG_SET;
    }
    /* 0x0F indicate enables all interrupt. */
    handle->baseAddress->SPIIMSC.reg = 0x0F;

    return BASE_STATUS_OK;
}

/**
  * @brief Receiving and send data in interrupts mode.
  * @param handle SPI handle.
  * @param rData Address of the data buff to be Receiving.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be Receiving and sent.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_WriteReadIT(SPI_Handle *handle,
                                    unsigned char *rData,
                                    unsigned char *wData,
                                    unsigned int dataSize)
{
    SPI_ASSERT_PARAM(handle != NULL && rData != NULL && wData != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));

    SPI_PARAM_CHECK_WITH_RET(handle->state == HAL_SPI_STATE_READY, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);

    handle->state = HAL_SPI_STATE_BUSY_TX_RX;
    ConfigTransmissionParameter(handle, rData, wData, dataSize);
    ClearSpiRxFifo(handle); /* If there is residual data in the read FIFO, clear the data. */
    SpiCsControl(handle, SPI_CHIP_SELECT);
    if (!handle->baseAddress->SPICR1.BIT.sse) {
        handle->baseAddress->SPICR1.BIT.sse = BASE_CFG_SET;
    }
    /* 0x0F indicate enables all interrupt. */
    handle->baseAddress->SPIIMSC.reg = 0x0F;

    return BASE_STATUS_OK;
}

/**
  * @brief Wait until the SPI data transmission is complete.
  * @param handle SPI handle.
  * @retval None.
  */
static void WaitComplete(void *handle)
{
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_Handle *spiHandle = (SPI_Handle *)(handle);
    SPI_ASSERT_PARAM(IsSPIInstance(spiHandle->baseAddress));
    while (true) {
        /* Wait for the write operation to complete */
        if (spiHandle->baseAddress->SPISR.BIT.bsy == BASE_CFG_UNSET &&
            spiHandle->baseAddress->SPISR.BIT.tfe == BASE_CFG_SET &&
            spiHandle->baseAddress->SPISR.BIT.rne == BASE_CFG_UNSET) {
            break;
        }
    }
}

/**
  * @brief SPI DMA read completion callback function.
  * @param handle SPI handle.
  * @retval None
  */
static void ReadDmaFinishFun(void *handle)
{
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_Handle *spiHandle = (SPI_Handle *)(handle);
    SPI_ASSERT_PARAM(IsSPIInstance(spiHandle->baseAddress));
    /* Waiting for SPI data transfer to complete */
    WaitComplete(spiHandle);
    SpiCsControl(spiHandle, SPI_CHIP_DESELECT);

    if (spiHandle->state == HAL_SPI_STATE_BUSY_RX) {
        if (spiHandle->userCallBack.RxCpltCallback != NULL) {
            spiHandle->userCallBack.RxCpltCallback(spiHandle);
        }
    }

    if (spiHandle->state == HAL_SPI_STATE_BUSY_TX_RX) {
        if (spiHandle->userCallBack.TxRxCpltCallback != NULL) {
            spiHandle->userCallBack.TxRxCpltCallback(spiHandle);
        }
    }

    if (spiHandle->state == HAL_SPI_STATE_BUSY_TX) {
        if (spiHandle->userCallBack.TxCpltCallback != NULL) {
            spiHandle->userCallBack.TxCpltCallback(spiHandle);
        }
    }

    spiHandle->state = HAL_SPI_STATE_READY;
    /* Disable rx fifo DMA */
    spiHandle->baseAddress->SPIDMACR.BIT.rxdmae = BASE_CFG_UNSET;
}

/**
  * @brief SPI DMA write completion callback function.
  * @param handle SPI handle.
  * @retval None
  */
static void WriteDmaFinishFun(void *handle)
{
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_Handle *spiHandle = (SPI_Handle *)(handle);
    SPI_ASSERT_PARAM(IsSPIInstance(spiHandle->baseAddress));
    /* Waiting for SPI data transfer to complete */
    WaitComplete(spiHandle);
    SpiCsControl(spiHandle, SPI_CHIP_DESELECT);
    /* Disable tx fifo DMA */
    spiHandle->baseAddress->SPIDMACR.BIT.txdmae = BASE_CFG_UNSET;
    if (spiHandle->userCallBack.TxCpltCallback != NULL && spiHandle->state == HAL_SPI_STATE_READY) {
        spiHandle->userCallBack.TxCpltCallback(spiHandle);
    }
    if (spiHandle->frameFormat == HAL_SPI_MODE_MICROWIRE) {
        spiHandle->state = HAL_SPI_STATE_READY;
    }
}

/**
  * @brief SPI DMA error callback function.
  * @param handle SPI handle.
  * @retval None
  */
static void DmaErrorFun(void *handle)
{
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_Handle *spiHandle = (SPI_Handle *)(handle);
    SPI_ASSERT_PARAM(IsSPIInstance(spiHandle->baseAddress));
    SpiCsControl(spiHandle, SPI_CHIP_DESELECT);
    /* Disable rx and tx fifo DMA */
    spiHandle->baseAddress->SPIDMACR.reg = 0;

    if (spiHandle->userCallBack.ErrorCallback != NULL) {
        spiHandle->userCallBack.ErrorCallback(spiHandle);
    }
    spiHandle->state = HAL_SPI_STATE_READY;
}

/**
  * @brief DMA enable Configuration.
  * @param handle SPI handle.
  * @retval None
  */
static void EnableDma(SPI_Handle *handle)
{
    handle->baseAddress->SPIIMSC.reg = 0x0;
    SpiCsControl(handle, SPI_CHIP_SELECT);
    if (!handle->baseAddress->SPICR1.BIT.sse) {
        handle->baseAddress->SPICR1.BIT.sse = BASE_CFG_SET;
    }
    handle->baseAddress->SPIDMACR.reg = SPI_DMA_FIFO_ENABLE;
}

/**
  * @brief SPI read and write configures the DMA for channel callback functions.
  * @param handle SPI handle.
  * @retval None
  */
static void SetDmaCallBack(SPI_Handle *handle)
{
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->rxDmaCh].ChannelFinishCallBack = ReadDmaFinishFun;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->rxDmaCh].ChannelErrorCallBack = DmaErrorFun;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelFinishCallBack = WriteDmaFinishFun;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelErrorCallBack = DmaErrorFun;
}

/**
  * @brief Receiving  data in DMA mode.
  * @param handle SPI handle.
  * @param rData Address of the data buff to be Receiving.
  * @param dataSize Number of the data to be Receiving.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_ReadDMA(SPI_Handle *handle, unsigned char *rData, unsigned int dataSize)
{
    static unsigned short writeVal = 0;
    BASE_StatusType ret;

    SPI_ASSERT_PARAM(handle != NULL && rData != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    SPI_ASSERT_PARAM(handle->dmaHandle != NULL); /* Check the DMA transfer handle and channel. */
    SPI_PARAM_CHECK_WITH_RET(handle->txDmaCh < CHANNEL_MAX_NUM, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(handle->rxDmaCh < CHANNEL_MAX_NUM, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET((handle->rxDmaCh != handle->txDmaCh), BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(handle->state == HAL_SPI_STATE_READY, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);

    handle->state = HAL_SPI_STATE_BUSY_RX;
    /* Configuring SPI transmission parameters */
    ConfigTransmissionParameter(handle, rData, NULL, dataSize);
    ClearSpiRxFifo(handle); /* If there is residual data in the read FIFO, clear the data. */
    SetDmaCallBack(handle);
    /* To set the auto-increment mode of the source and destination addresses */
    handle->dmaHandle->DMA_Channels[handle->rxDmaCh].srcAddrInc = DMA_ADDR_UNALTERED;
    handle->dmaHandle->DMA_Channels[handle->rxDmaCh].destAddrInc = DMA_ADDR_INCREASE;
    handle->dmaHandle->DMA_Channels[handle->txDmaCh].srcAddrInc = DMA_ADDR_UNALTERED;
    handle->dmaHandle->DMA_Channels[handle->txDmaCh].destAddrInc = DMA_ADDR_UNALTERED;

    /* DMA rx channel Interrupt Transfer */
    ret = HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t) & (handle->baseAddress->SPIDR),
                          (uintptr_t)handle->rxBuff, handle->transferSize, handle->rxDmaCh);
    if (ret != BASE_STATUS_OK) {
        handle->state = HAL_SPI_STATE_READY;
        return ret;
    }
    /* DMA tx channel Interrupt Transfer */
    if (handle->mode == HAL_SPI_MASTER) {
        ret = HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t) & writeVal, (uintptr_t) & (handle->baseAddress->SPIDR),
            handle->transferSize, handle->txDmaCh);
        if (ret != BASE_STATUS_OK) {
            handle->state = HAL_SPI_STATE_READY;
            return ret;
        }
    }
    EnableDma(handle);
    return ret;
}

/**
  * @brief Send data in DMA mode.
  * @param handle SPI handle.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be sent.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_WriteDMA(SPI_Handle *handle, unsigned char *wData, unsigned int dataSize)
{
    static unsigned short readVal;
    BASE_StatusType ret;

    SPI_ASSERT_PARAM(handle != NULL && wData != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    SPI_ASSERT_PARAM(handle->dmaHandle != NULL); /* Check the DMA transfer handle and channel. */
    SPI_PARAM_CHECK_WITH_RET(handle->txDmaCh < CHANNEL_MAX_NUM, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(handle->rxDmaCh < CHANNEL_MAX_NUM, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET((handle->rxDmaCh != handle->txDmaCh), BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(handle->state == HAL_SPI_STATE_READY, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);

    handle->state = HAL_SPI_STATE_BUSY_TX;
    /* Configuring SPI transmission parameters */
    ConfigTransmissionParameter(handle, NULL, wData, dataSize);
    ClearSpiRxFifo(handle); /* If there is residual data in the read FIFO, clear the data. */
    SetDmaCallBack(handle);
    /* To set the auto-increment mode of the source and destination addresses */
    handle->dmaHandle->DMA_Channels[handle->rxDmaCh].srcAddrInc = DMA_ADDR_UNALTERED;
    handle->dmaHandle->DMA_Channels[handle->rxDmaCh].destAddrInc = DMA_ADDR_UNALTERED;
    handle->dmaHandle->DMA_Channels[handle->txDmaCh].srcAddrInc = DMA_ADDR_INCREASE;
    handle->dmaHandle->DMA_Channels[handle->txDmaCh].destAddrInc = DMA_ADDR_UNALTERED;
    /* DMA tx channel Interrupt Transfer */
    ret = HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)handle->txBuff,
                          (uintptr_t) & (handle->baseAddress->SPIDR), handle->transferSize, handle->txDmaCh);
    if (ret != BASE_STATUS_OK) {
        handle->state = HAL_SPI_STATE_READY;
        return ret;
    }
    /* DMA rx channel Interrupt Transfer */
    ret = HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t) & (handle->baseAddress->SPIDR),
                          (uintptr_t) & readVal, handle->transferSize, handle->rxDmaCh);
    if (ret != BASE_STATUS_OK) {
        handle->state = HAL_SPI_STATE_READY;
        return ret;
    }
    EnableDma(handle);
    return ret;
}

/**
  * @brief Receiving and send data in DMA mode.
  * @param handle SPI handle.
  * @param rData Address of the data buff to be Receiving.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be Receiving and sent.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_WriteReadDMA(SPI_Handle *handle,
                                     unsigned char *rData,
                                     unsigned char *wData,
                                     unsigned int dataSize)
{
    BASE_StatusType ret;

    SPI_ASSERT_PARAM(handle != NULL && rData != NULL && wData != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    SPI_ASSERT_PARAM(handle->dmaHandle != NULL); /* Check the DMA transfer handle and channel. */
    SPI_PARAM_CHECK_WITH_RET(handle->txDmaCh < CHANNEL_MAX_NUM, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(handle->rxDmaCh < CHANNEL_MAX_NUM, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET((handle->rxDmaCh != handle->txDmaCh), BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(handle->state == HAL_SPI_STATE_READY, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);

    handle->state = HAL_SPI_STATE_BUSY_TX_RX;
    /* Configuring SPI transmission parameters */
    ConfigTransmissionParameter(handle, rData, wData, dataSize);
    ClearSpiRxFifo(handle); /* If there is residual data in the read FIFO, clear the data. */
    SetDmaCallBack(handle);
    /* To set the auto-increment mode of the source and destination addresses */
    handle->dmaHandle->DMA_Channels[handle->rxDmaCh].srcAddrInc = DMA_ADDR_UNALTERED;
    handle->dmaHandle->DMA_Channels[handle->rxDmaCh].destAddrInc = DMA_ADDR_INCREASE;
    handle->dmaHandle->DMA_Channels[handle->txDmaCh].srcAddrInc = DMA_ADDR_INCREASE;
    handle->dmaHandle->DMA_Channels[handle->txDmaCh].destAddrInc = DMA_ADDR_UNALTERED;
    /* DMA rx channel Interrupt Transfer */
    ret = HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t) & (handle->baseAddress->SPIDR),
                          (uintptr_t)handle->rxBuff, handle->transferSize, handle->rxDmaCh);
    if (ret != BASE_STATUS_OK) {
        handle->state = HAL_SPI_STATE_READY;
        return ret;
    }
    /* DMA tx channel Interrupt Transfer */
    ret = HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)handle->txBuff,
                          (uintptr_t) & (handle->baseAddress->SPIDR), handle->transferSize, handle->txDmaCh);
    if (ret != BASE_STATUS_OK) {
        handle->state = HAL_SPI_STATE_READY;
        return ret;
    }
    EnableDma(handle);
    return ret;
}

/**
  * @brief Stop DMA transfer.
  * @param handle SPI handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_DMAStop(SPI_Handle *handle)
{
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    BASE_StatusType ret;

    ret = HAL_DMA_StopChannel(handle->dmaHandle, handle->txDmaCh);
    if (ret != BASE_STATUS_OK) {
        return ret;
    }
    ret = HAL_DMA_StopChannel(handle->dmaHandle, handle->rxDmaCh);
    return ret;
}

/**
  * @brief CS Channel Configuration.
  * @param handle SPI handle.
  * @param channel SPI CS channel.For details, see the enumeration definition of SPI_ChipSelectChannel.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_SPI_ChipSelectChannelSet(SPI_Handle *handle, SPI_ChipSelectChannel channel)
{
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    /* Check the validity of the CS parameters. */
    SPI_PARAM_CHECK_WITH_RET(channel >= SPI_CHIP_SELECT_CHANNEL_0 && channel < SPI_CHIP_SELECT_CHANNEL_MAX,
                             BASE_STATUS_ERROR);
    handle->baseAddress->SPICSNCR.BIT.spi_csn_sel = channel;
    return BASE_STATUS_OK;
}

/**
  * @brief Obtains the currently configured CS channel.
  * @param handle SPI handle.
  * @param channel Pointer to the address for storing the obtained CS channel value.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_ChipSelectChannelGet(SPI_Handle *handle, SPI_ChipSelectChannel *channel)
{
    SPI_ASSERT_PARAM(handle != NULL && channel != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    *channel = handle->baseAddress->SPICSNCR.BIT.spi_csn_sel;
    return BASE_STATUS_OK;
}


/**
  * @brief Interrupt Handling Function.
  * @param handle SPI_Handle.
  * @retval None.
  */
void HAL_SPI_IrqHandler(void *handle)
{
    SPI_Handle *spiHandle = (SPI_Handle *)handle;
    SPI_ASSERT_PARAM(spiHandle != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(spiHandle->baseAddress));
    /* Indicates that there is no interruption. */
    if (spiHandle->baseAddress->SPIMIS.reg == 0) {
        return;
    }

    /* Generating RX overflow interrupt. */
    if (spiHandle->baseAddress->SPIMIS.BIT.rormis) {
        spiHandle->baseAddress->SPIIMSC.reg = 0x0;
        /* Clear rx interrupt. */
        spiHandle->baseAddress->SPIICR.BIT.roric = BASE_CFG_SET;
        spiHandle->baseAddress->SPIICR.BIT.rtic = BASE_CFG_SET;

        spiHandle->errorCode = BASE_STATUS_ERROR;
        spiHandle->state = HAL_SPI_STATE_ERROR;
        /* Invoke the error callback function. */
        if (spiHandle->userCallBack.ErrorCallback != NULL) {
            spiHandle->userCallBack.ErrorCallback(spiHandle);
        }
        SpiCsControl(spiHandle, SPI_CHIP_DESELECT);
        return;
    }
    /* Reads and writes data based on the interrupt flag. */
    ReadWriteInt(spiHandle);
    SpiRxTxCallack(spiHandle);
}