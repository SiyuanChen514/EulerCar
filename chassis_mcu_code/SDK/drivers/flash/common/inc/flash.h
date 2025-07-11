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
  * @file    flash.h
  * @author  MCU Driver Team
  * @brief   FLASH module driver.
  * @details This file provides firmware functions to manage the following functionalities of the FLASH.
  *          + Basic parameter configuration macro.
  *          + FLASH parameter handle definition.
  *          + Initialization and de-initialization functions.
  *          + Definition of flash read/write erase functions.
  */
#ifndef McuMagicTag_FLASH_H
#define McuMagicTag_FLASH_H

/* Includes ---------------------------------------------------------------------*/
#include "flash_ip.h"

/**
  * @defgroup FLASH FLASH
  * @brief FLASH module.
  * @{
  */

/**
  * @defgroup FLASH_Common FLASH Common
  * @brief FLASH common external module.
  * @{
  */

/* Macro definitions -----------------------------------------------------------*/

/**
  * @defgroup FLASH_Handle_Definition FLASH Handle Definition
  * @{
  */

/**
  * @brief Module Status Enumeration Definition
  */
typedef enum {
    FLASH_STATE_RESET   = 0x00000000U,
    FLASH_STATE_READY   = 0x00000001U,
    FLASH_STATE_PGM     = 0x00000002U,
    FLASH_STATE_ERASE   = 0x00000003U,
    FLASH_STATE_ERROR   = 0x00000004U
} FLASH_StateType;

/**
 * @brief Module handle structure definition
 */
typedef struct _FLASH_Handle {
    EFC_RegStruct         *baseAddress;  /**< Register base address. */
    FLASH_PE_OpMode        peMode;       /**< PE operation type. For details, see FLASH_PE_OpMode. */
    volatile unsigned int  destAddr;     /**< Destination address for storing interrupt operations. */
    volatile unsigned int  srcAddr;      /**< Used to store the source address in interrupt mode. */
    volatile unsigned int  writeLen;     /**< Indicates the length of the data to be written in interrupt mode. */
    volatile unsigned int  eraseNum;     /**< Used to store the number of erase blocks in interrupt mode. */
    FLASH_StateType        state;        /**< Running status of the flash module. For details, see FLASH_StateType. */
    FLASH_UserCallBcak     userCallBack; /**< User-defined callback function. */
    FLASH_ExtendHandle     handleEx;     /**< Extend handle,  configuring some special parameters. */
} FLASH_Handle;

/**
 * @brief Callback Function Type Definition.
 */
typedef void (*FLASH_CallbackFunType)(void *handle, FLASH_CallBackEvent event, unsigned int opAddr);
/**
  * @}
  */

/**
  * @defgroup FLASH_API_Declaration FLASH HAL API
  * @{
  */
BASE_StatusType HAL_FLASH_Init(FLASH_Handle *handle);
BASE_StatusType HAL_FLASH_DeInit(FLASH_Handle *handle);
BASE_StatusType HAL_FLASH_RegisterCallback(FLASH_Handle *handle, FLASH_CallbackFunType pcallback);
BASE_StatusType HAL_FLASH_WriteBlocking(FLASH_Handle *handle, unsigned int srcAddr,
                                        unsigned int destAddr, unsigned int srcLen);
BASE_StatusType HAL_FLASH_EraseBlocking(FLASH_Handle *handle,
                                        FLASH_EraseMode eraseMode,
                                        FLASH_SectorAddr startAddr,
                                        unsigned int eraseNum);
BASE_StatusType HAL_FLASH_WriteIT(FLASH_Handle *handle, unsigned int srcAddr,
                                  unsigned int destAddr, unsigned int srcLen);
BASE_StatusType HAL_FLASH_EraseIT(FLASH_Handle *handle,
                                  FLASH_EraseMode eraseMode,
                                  FLASH_SectorAddr startAddr,
                                  unsigned int eraseNum);
BASE_StatusType HAL_FLASH_Read(FLASH_Handle *handle,
                               unsigned int srcAddr,
                               unsigned int readLen,
                               unsigned char *dataBuff,
                               unsigned int buffLen);
void HAL_FLASH_IrqHandler(void *handle);
void HAL_FLASH_IrqHandlerError(void *handle);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* #ifndef McuMagicTag_FLASH_H */