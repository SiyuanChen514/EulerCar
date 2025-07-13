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
  * @file      mcs_curr_ctrl.h
  * @author    MCU Algorithm Team
  * @brief     Current controller for motor control.
  *            This file provides functions declaration of the current controller module.
  */

/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_CURR_CTRL_H
#define McuMagicTag_MCS_CURR_CTRL_H

/* Includes ------------------------------------------------------------------------------------ */
#include "mcs_typedef.h"
#include "mcs_pid_ctrl.h"
#include "mcs_mtr_param.h"

/**
  * @defgroup CURRENT_CONTROLLER CURRENT CONTROLLER MODULE
  * @brief The current controller function.
  * @{
  */

/**
  * @defgroup CURRENT_CONTROLLER_STRUCT CURRENT CONTROLLER STRUCT
  * @brief The current controller's data structure definition.
  * @{
  */

/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief Current controller struct members and parameters.
  */
typedef struct {
    DqAxis *idqRef;            /**< Current reference in the d-q coordinate (A). */
    DqAxis *idqFbk;            /**< Current feedback in the d-q coordinate (A). */
    DqAxis idqFf;             /**< Current feedforward value (V). */
    PID_Handle dAxisPi;        /**< d-axis current PI controller. */
    PID_Handle qAxisPi;        /**< q-axis current PI controller. */
    MOTOR_Param *mtrParam; /**< Motor parameters. */
    float outLimit;            /**< Current controller output voltage limitation (V). */
    float ts;                  /**< Current controller control period (s). */
} CURRCTRL_Handle;
/**
  * @}
  */

/**
  * @defgroup CURRENT_CONTROLLER_API CURRENT CONTROLLER API
  * @brief The current controller's API declaration.
  * @{
  */
void CURRCTRL_Init(CURRCTRL_Handle *currHandle, MOTOR_Param *mtrParam, DqAxis *idqRef, DqAxis *idqFbk,
                   const PI_Param dAxisPi, const PI_Param qAxisPi, float ts);

void CURRCTRL_Reset(CURRCTRL_Handle *currHandle);

void CURRCTRL_Clear(CURRCTRL_Handle *currHandle);

void CURRCTRL_Exec(CURRCTRL_Handle *currHandle, DqAxis *vdqRef, float spd, int ffEnable);

void CURRCTRL_SetTs(CURRCTRL_Handle *currHandle, float ts);

/**
  * @}
  */

/**
  * @}
  */

#endif
