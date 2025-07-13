/**
  * @ Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2022-2023. All rights reserved.
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
  * @file      mcs_startup.h
  * @author    MCU Algorithm Team
  * @brief     Motor transition process from one speed and angle to another speed and angle.
  */


#ifndef McuMagicTag_MCS_STARTUP_H
#define McuMagicTag_MCS_STARTUP_H

/**
  * @brief Startup process enum.
  * @details Speed transition stages:
  *          + STARTUP_STAGE_CURR -- Stage of current AMP is changing
  *          + STARTUP_STAGE_SPD -- Stage of speed is changing
  *          + STARTUP_STAGE_SWITCH -- Stage of switch
  *          + STARTUP_STAGE_DETECT -- Stage of detect switch open loop
  */
typedef enum {
    STARTUP_STAGE_CURR = 1,
    STARTUP_STAGE_SPD,
    STARTUP_STAGE_SWITCH,
    STARTUP_STAGE_DETECT,
} STARTUP_Stage;

/**
  * @brief Startup handover method struct members and parameters.
  */
typedef struct {
    STARTUP_Stage stage; /**< Startup switching status. */
    float spdBegin;      /**< Startup switching start speed (Hz). */
    float spdEnd;        /**< Startup switching end speed (Hz). */
    float regionInv;     /**< Inverse of the speed region. */
    float initCurr;      /**< The initial current (A). */
} STARTUP_Handle;


/**
  * @defgroup STARTUP_API  STARTUP API
  * @brief The startup management API declaration.
  * @{
  */
void STARTUP_Init(STARTUP_Handle *startHandle, float spdBegin, float spdEnd);
void STARTUP_Clear(STARTUP_Handle *startHandle);
float STARTUP_CurrCal(const STARTUP_Handle *startHandle, float refHz);

#endif
