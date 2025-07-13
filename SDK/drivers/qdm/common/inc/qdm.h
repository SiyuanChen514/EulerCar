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
  * @file    qdm.h
  * @author  MCU Driver Team
  * @brief   QDM HAL level module driver head file.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the QDM.
  *           + Initialization and de-initialization functions.
  *           + Capm Module Control functions.
  *           + Speed measure use M function.
  *           + Stall condition detection.
  */
#ifndef McuMagicTag_QDM_H
#define McuMagicTag_QDM_H

#include "typedefs.h"
#include "qdm_ip.h"

#define SECONDS_PER_MINUTES     60

/**
  * @defgroup QDM QDM
  * @brief QDM module.
  * @{
  */

/**
  * @defgroup QDM_Common QDM Common
  * @brief QDM common external module.
  * @{
  */


/**
  * @defgroup QDM_Common_Param QDM Common Parameters
  * @{
  */

/**
  * @brief  QDM callback function type
  */
typedef enum {
    QDM_TSU_CYCLE               = 0x00000000U,
    QDM_SPEED_LOSE              = 0x00000001U,
    QDM_INDEX_LOCKED            = 0x00000002U,
    QDM_DIR_CHANGE              = 0x00000003U,
    QDM_PHASE_ERROR             = 0x00000004U,
    QDM_POS_MATCH               = 0x00000005U,
    QDM_POS_READY               = 0x00000006U,
    QDM_POS_CNT_ERROR           = 0x00000007U,
    QDM_POS_CNT_OVERFLOW        = 0x00000008U,
    QDM_POS_CNT_UNDERFLOW       = 0x00000009U
} QDM_CallbackFuncType;

/**
  * @}
  */

/**
  * @defgroup QDM_Handle_Definition QDM Handle Definition
  * @{
  */

/**
  * @brief configurations of QDU register
  */
typedef struct {
    QDM_DecoderMode   decoderMode;
    QDM_Resolution    resolution;
    QDM_QtrgLockMode  trgLockMode;
    QDM_PtuMode       ptuMode;
    QDM_SwapSelect    swap;
    unsigned int      polarity;
} QDMCtrlConfigure;

/**
  * @brief configurations of input filter level
  */
typedef struct {
    unsigned int qdmAFilterLevel;
    unsigned int qdmBFilterLevel;
    unsigned int qdmZFilterLevel;
} QDMFilter;

/**
  * @brief configurations of input filter level
  */
typedef struct _QDM_handle {
    QDM_RegStruct       *baseAddress;       /**< base address */
    QDM_EmulationMode   emuMode;            /**< emulation mode select */
    QDMFilter           inputFilter;        /**< filter settings */
    QDMCtrlConfigure    ctrlConfig;         /**< QDM control configurations */
    QDM_PcntMode        pcntMode;           /**< position count mode */
    QDM_PcntRstMode     pcntRstMode;        /**< position count reset mode */
    QDM_PcntIdxInitMode pcntIdxInitMode;    /**< position count index initial mode */
    bool                subModeEn;          /**< sub-module enable */
    QDM_TSUPrescaler    tsuPrescaler;       /**< tsu prescaler */
    QDM_CEVTPrescaler   cevtPrescaler;      /**< cevt prescaler */
    unsigned int        posInit;            /**< init position */
    unsigned int        posMax;             /**< max position */
    unsigned int        qcMax;              /**< TSU maximum counter number, default zero */
    unsigned int        period;             /**< PTU period*/
    unsigned int        interruptEn;        /**< interrupt settings by bits */
    int                 motorLineNum;       /**< encoder line number */
    int                 speedRpm;           /**< motor speed */
    QDM_IndexLockMode lock_mode;          /**< QDM Z index lock mode */
    QDM_UserCallBack  userCallBack;         /**< QDM Interrupt callback functions */
    QDM_ExtendHandle  handleEx;             /**< QDM extend parameter */
} QDM_Handle;

typedef void (* QDM_CallbackType)(void *handle);
/**
  * @}
  */

/**
  * @defgroup QDM_API_Declaration QDM HAL API
  * @{
  */

/* Hardware abstraction layer */
BASE_StatusType HAL_QDM_Init(QDM_Handle *qdmHandle);
BASE_StatusType HAL_QDM_DeInit(QDM_Handle *qdmHandle);
void HAL_QDM_GetPhaseErrorStatus(const QDM_Handle *qdmHandle, unsigned int *errStatus);
void HAL_QDM_ReadPosCountAndDir(const QDM_Handle *qdmHandle, unsigned int *count, unsigned int *dir);
int HAL_QDM_GetSpeedRpmM(QDM_Handle *qdmHandle);
int HAL_QDM_GetSpeedRpmMT(QDM_Handle *qdmHandle);
void HAL_QDM_IrqHandler(void *handle);
void HAL_QDM_RegisterCallback(QDM_Handle *qdmHandle, QDM_CallbackFuncType typeID, QDM_CallbackType pCallback);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif