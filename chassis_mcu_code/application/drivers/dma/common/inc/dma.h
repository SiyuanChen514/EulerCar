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
  * @file    dma.h
  * @author  MCU Driver Team
  * @brief   DMA module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the DMA.
  *           + The definition of the DMA handle structure.
  *           + Initialization and de-initialization functions
  *           + Peripheral querying the transmission functions.
  *           + Peripheral interrupt handler and callback registration functions.
  */

/* Includes ------------------------------------------------------------------*/
#ifndef McuMagicTag_DMA_H
#define McuMagicTag_DMA_H
#include "dma_ip.h"

/**
  * @defgroup DMA DMA
  * @brief DMA module.
  * @{
  */

/**
  * @defgroup DMA_Common DMA Common
  * @brief DMA common external module.
  * @{
  */

/**
  * @defgroup DMA_Handle_Definition DMA Handle Definition
  * @{
  */

/**
  * @brief The definition of the DMA handle structure.
  */
typedef struct _DMA_Handle {
    DMA_RegStruct               *baseAddress;   /**< DMA common registers base address */
    struct {
        DMA_ChannelRegStruct    *channelAddr;   /**< DMA channel registers base address */
        DMA_TransDirection       direction;     /**< The transmission direction type */
        DMA_RequestLineNum       srcPeriph;     /**< Source device request line, memory ignore configuration */
        DMA_RequestLineNum       destPeriph;    /**< Destination device request line, memory ignore configuration */
        DMA_AddrIncMode          srcAddrInc;    /**< Address increase configuration of source device */
        DMA_AddrIncMode          destAddrInc;   /**< Address increase configuration of destination device */
        DMA_BurstLength          srcBurst;      /**< Burst length of source device */
        DMA_BurstLength          destBurst;     /**< Burst length of destination device */
        DMA_TransmisWidth        srcWidth;      /**< Transfer width of source device */
        DMA_TransmisWidth        destWidth;     /**< Transfer width of destination device */
        void                    *pHandle;       /**< Handle of the modules that use the DMA */
        unsigned int             srcAddr;       /**< Readback value from the source address to the register */
        unsigned int             destAddr;      /**< Readback value from the destnation address to the register */
        unsigned int             controlVal;    /**< Readback value of the DMA control register */
        unsigned int             configVal;     /**< Readback value of the DMA configuration register */
    } DMA_Channels[CHANNEL_MAX_NUM];
    DMA_UserCallBack             userCallBack;  /**< User callback */
    DMA_ExtendHandle             handleEx;      /**< DMA extend parameter */
} DMA_Handle;

/**
  * @brief The definition of the DMA channel param structure.
  */
typedef struct {
    DMA_RequestLineNum       srcPeriph;         /**< Source device request line, memory ignore configuration */
    DMA_RequestLineNum       destPeriph;        /**< Destination device request line, memory ignore configuration */
    DMA_TransDirection       direction;         /**< The transmission direction type */
    DMA_AddrIncMode          srcAddrInc;        /**< Address increase configuration of source device */
    DMA_AddrIncMode          destAddrInc;       /**< Address increase configuration of destination device */
    DMA_BurstLength          srcBurst;          /**< Burst length of source device */
    DMA_BurstLength          destBurst;         /**< Burst length of destination device */
    DMA_TransmisWidth        srcWidth;          /**< Transfer width of source device */
    DMA_TransmisWidth        destWidth;         /**< Transfer width of destination device */
    void                    *pHandle;           /**< Parameter handle of the users callback function */
} DMA_ChannelParam;

typedef void (* DMA_CallbackType)(void *handle);
/**
  * @}
  */

/**
  * @defgroup DMA_API_Declaration DMA HAL API
  * @{
  */
/* Hardware abstraction layer */
BASE_StatusType HAL_DMA_Init(DMA_Handle *dmaHandle);
BASE_StatusType HAL_DMA_Deinit(DMA_Handle *dmaHandle);
BASE_StatusType HAL_DMA_Start(DMA_Handle *dmaHandle, unsigned int srcAddr,
                              unsigned int destAddr, unsigned int dataLength, unsigned int channel);
BASE_StatusType HAL_DMA_StartIT(DMA_Handle *dmaHandle, unsigned int srcAddr,
                                unsigned int destAddr, unsigned int dataLength, unsigned int channel);
BASE_StatusType HAL_DMA_StopChannel(DMA_Handle *dmaHandle, unsigned int channel);
BASE_StatusType HAL_DMA_GetChannelState(DMA_Handle *dmaHandle, unsigned int channel);
BASE_StatusType HAL_DMA_InitChannel(DMA_Handle *dmaHandle, DMA_ChannelParam *channelParam, unsigned int channel);
void HAL_DMA_IrqHandlerTc(void *handle);
void HAL_DMA_IrqHandlerError(void *handle);
void HAL_DMA_RegisterCallback(DMA_Handle *dmaHandle, DMA_CallbackFun_Type typeID,
                              DMA_ChannelNum channel, DMA_CallbackType pCallback);
BASE_StatusType HAL_DMA_ListAddNode(DMA_LinkList *head, DMA_LinkList *newNode);
BASE_StatusType HAL_DMA_InitNewNode(DMA_LinkList *node, const DMA_ChannelParam *param,
                                    unsigned int srcAddr, unsigned int destAddr, unsigned int tranSize);
BASE_StatusType HAL_DMA_StartListTransfer(DMA_Handle *dmaHandle, DMA_LinkList *head, unsigned int channel);
#ifdef BASE_DEFINE_DMA_QUICKSTART
void HAL_DMA_QuickStart(DMA_Handle *dmaHandle, unsigned int channel);
#endif
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif  /* McuMagicTag_DMA_H */