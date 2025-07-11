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
  * @file    capm.h
  * @author  MCU Driver Team
  * @brief   CAPM module driver
  * @details This file provides firmware CAPM Handle Structure and functions
  *          prototypes to manage the following functionalities of the CAPM.
  *           + CAPM handle structure definition.
  *           + Initialization and de-initialization functions.
  *           + CAPM Service Functions.
  */
#ifndef McuMagicTag_CAPM_H
#define McuMagicTag_CAPM_H

#include "typedefs.h"
#include "dma.h"
#include "capm_ip.h"

/**
  * @defgroup CAPM CAPM
  * @brief CAPM module.
  * @{
  */

/**
  * @defgroup CAPM_Common CAPM Common
  * @brief CAPM common external module.
  * @{
  */


/**
  * @defgroup CAPM_Common_Param CAPM Common Parameters
  * @{
  */
#define CAPM_NUM_0 0
#define CAPM_NUM_1 1
#define CAPM_NUM_2 2

/**
  * @brief Capture edge mode
  */
typedef enum {
    CAPM_FALLING,
    CAPM_RISING,
} CAPM_CapEvent;

/**
  * @brief Reset mode
  */
typedef enum {
    CAPM_NOTRESET,
    CAPM_RESET,
} CAPM_RegRestMode;

/**
  * @brief Signal level
  */
typedef enum {
    CAPM_LOW_LEVEL,
    CAPM_UP_EDGE,
    CAPM_DOWN_EDGE,
    CAPM_HIGHT_LEVEL,
} CAPM_CaptureLevel;

/**
  * @brief Numbers of ECR
  */
typedef enum {
    CAPM_ECR_NUM1,
    CAPM_ECR_NUM2,
    CAPM_ECR_NUM3,
    CAPM_ECR_NUM4
} CAPM_ECRNum;

/**
  * @brief Used ECR of next load
  */
typedef enum {
    CAPM_NEXT_LOAD_ECR1,
    CAPM_NEXT_LOAD_ECR2,
    CAPM_NEXT_LOAD_ECR3,
    CAPM_NEXT_LOAD_ECR4,
} CAPM_NextLoadECR;

/**
  * @brief  CAPM callback function type
  */
typedef enum {
    CAPM_EVT_FINISH = 0x00000000U,
    CAPM_DMA_ERROR = 0x00000001U,
    CAPM_DMA_FINISH = 0x00000002U
} CAPM_CallbackFuncType;

/**
  * @}
  */

/**
  * @defgroup CAPM_Handle_Definition CAPM Handle Definition
  * @{
  */

/**
  * @brief Configurations of each capture register
  */
typedef struct CapmCapRegConfig {
    CAPM_CapEvent capEvent;
    CAPM_RegRestMode regReset;
} CAPM_CapRegConfig;

/**
  * @brief The definition of the CAPM handle structure
  */
typedef struct _CAPM_Handle {
    CAPM_RegStruct *baseAddress;                          /**< base address */
    unsigned int tscntDiv;                                /**< TSR count division, value range: 0~65535 */
    DMA_Handle *dmaHandle;                                /**< DMA handle */
    unsigned int dmaChannel;                              /**< Used DMA channel */
    
    unsigned int preScale;                                /**< preScale factor. value range: 0~127 */
    unsigned int deburrNum;                               /**< deburr level. value range:0~8192. 0: Disable deburr */
    unsigned int useCapNum;                               /**< number of cap to be use.
                                                               value range: 1~CAPM_MAX_CAP_REG_NUM */
    unsigned int triggleDmaReg;                           /**< which ECR to triggle DMA interrupt.
                                                               value range:1 ~ useCapNum */
    unsigned int syncPhs;                                 /**< TSRֵ sync phase value */
    bool enableSync;                                      /**< enable sync */
    CAPM_SyncSrc syncSrc;                                 /**< CAPM synchronized input source */
    unsigned int enableIntFlags;                          /**< enable interrupt */
    CAPM_CapMode capMode;                                 /**< capture mode. continue or one-shot */
    CAPM_InputSrc inputSrc;                               /**< capture input source */
    CAPM_CapRegConfig capRegConfig[CAPM_MAX_CAP_REG_NUM]; /**< each capture register configuration */
    CAPM_UserCallBack userCallBack;                       /**< CAPM Interrupt callback functions.*/
    CAPM_ExtendHandle handleEx;                           /**< CAPM extend parameter */
} CAPM_Handle;

typedef void (* CAPM_CallbackType)(void *handle);
/**
  * @}
  */

/**
  * @defgroup CAPM_API_Declaration CAPM HAL API
  * @{
  */
BASE_StatusType HAL_CAPM_Init(CAPM_Handle *handle);
BASE_StatusType HAL_CAPM_DeInit(CAPM_Handle *handle);

unsigned int HAL_CAPM_GetECRValue(CAPM_Handle *handle, CAPM_ECRNum ecrNum);
unsigned char HAL_CAPM_GetCrtEdge(CAPM_Handle *handle);
unsigned char HAL_CAPM_GetNextLoadECRNum(CAPM_Handle *handle);
BASE_StatusType HAL_CAPM_GetECRValueDMA(CAPM_Handle *handle, unsigned int *saveData, unsigned int dataLength);

void HAL_CAPM_SetSyncPhs(CAPM_Handle *handle, unsigned int phase);
unsigned int HAL_CAPM_GetSyncPhs(CAPM_Handle *handle);

void HAL_CAPM_IrqHandler(void *handle);
void HAL_CAPM_RegisterCallback(CAPM_Handle *capmHandle, CAPM_CallbackFuncType typeID, CAPM_CallbackType pCallback);
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