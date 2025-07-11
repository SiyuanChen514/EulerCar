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
 * @file    pga.h
 * @author  MCU Driver Team
 * @brief   Programmable Gain Apmlifier HAL level module driver head file.
 *          This file provides firmware functions to manage the following
 *          functionalities of the Amplifier.
 *           + Initialization and de-initialization functions
 *           + Programmable Gain Amplifier set gain value functions
 */
#ifndef McuMagicTag_PGA_H
#define McuMagicTag_PGA_H

#include "pga_ip.h"
#include "baseinc.h"

/**
  * @defgroup PGA PGA
  * @brief PGA module.
  * @{
  */

/**
  * @defgroup PGA_Common PGA Common
  * @brief PGA common external module.
  * @{
  */

/**
  * @defgroup PGA_Handle_Definition PGA Handle Definition
  * @{
  */
/**
  * @brief The define of the PGA handle structure
  */
typedef struct _PGA_Handle {
    PGA_RegStruct         *baseAddress;                 /**< PGA registers base address. */
    PGA_GainValue          gain;                        /**< PGA gain selection. */
    bool                   externalResistorMode;        /**< PGA resistance mode. */

    PGA_ExtendHandle       handleEx;                    /**< PGA handle extend. */
} PGA_Handle;

/**
  * @}
  */

/**
  * @defgroup PGA_API_Declaration PGA HAL API
  * @{
  */
BASE_StatusType HAL_PGA_Init(PGA_Handle *pgaHandle);             /* Initializet function */
BASE_StatusType HAL_PGA_DeInit(PGA_Handle *pgaHandle);           /* Deinitialize function */
void HAL_PGA_SetGain(PGA_Handle *pgaHandle, PGA_GainValue gain); /* Set amplifier's gain function */
void HAL_PGA_Start(PGA_Handle *pgaHandle);                       /* Start PGA */
void HAL_PGA_Stop(PGA_Handle *pgaHandle);                        /* Stop PGA */
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