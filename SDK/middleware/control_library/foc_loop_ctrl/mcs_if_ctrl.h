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
  * @file      mcs_if_ctrl.h
  * @author    MCU Algorithm Team
  * @brief     Current controller for motor I/F control.
  *            This file provides functions declaration of I/F control.
  */

/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_IF_CTRL_H
#define McuMagicTag_MCS_IF_CTRL_H

/**
  * @defgroup IF_MODULE  I/F MODULE
  * @brief The I/F motor control method module.
  * @{
  */

/**
  * @defgroup IF_STRUCT  I/F STRUCT
  * @brief The I/F motor control method data struct definition.
  * @{
  */
/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief General IF controller struct members and parameters.
  */
typedef struct {
    float anglePeriod;  /**< Calculation period of the I/F angle (s). */
    float curAmpPeriod; /**< Calculation period of the I/F current amplitude (s). */

    float targetAmp;    /**< Target value of the I/F current (A). */
    float curAmp;       /**< Current value of the I/F current (A). */
    float stepAmp;      /**< Increment of the I/F current (A). */
    float angle;        /**< I/F output angle. */
} IF_Handle;

/**
  * @defgroup IF_API  I/F API
  * @brief The I/F motor control method API declaration.
  * @{
  */
void IF_Init(IF_Handle *ifHandle, float targetAmp, float currSlope, float stepAmpPeriod, float anglePeriod);

void IF_Clear(IF_Handle *ifHandle);

float IF_CurrAmpCalc(IF_Handle *ifHandle);

float IF_CurrAngleCalc(IF_Handle *ifHandle, float spdRef);

void IF_SetAngleTs(IF_Handle *ifHandle, float ts);

/**
  * @}
  */

/**
  * @}
  */

#endif
