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
  * @file      pfc_curr_ctrl.h
  * @author    MCU Algorithm Team
  * @brief     Current loop control. This file provides function of power factor correction(PFC) current control
  */
/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_PFC_CURR_CTRL_H
#define McuMagicTag_PFC_CURR_CTRL_H

/* Includes ------------------------------------------------------------------------------------ */
#include "mcs_pid_ctrl.h"

/**
  * @defgroup PFC_CURRENT_CONTROLLER PFC_CURRENT CONTROLLER MODULE
  * @brief The current controller function.
  * @{
  */

/**
  * @defgroup PFC_CURRENT_CONTROLLER_STRUCT PFC_CURRENT CONTROLLER STRUCT
  * @brief The current controller's data structure definition.
  * @{
  */

/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief current Controller Struct members and parameters.
  */
typedef struct {
    float currRef;            /* < current loop control reference current(A) */
    float currFdbk;           /* < current loop control feedback current(A) */
    float unitCurrFdbk;       /* < current loop control feedback unitary current */
    float maxCurrFdbk;        /* < current loop control max feedback current(A) */
    float startCurrFdbk;      /* < current loop control start feedback current(A) */
    float stopCurrFdbk;       /* < current loop control stop feedback current(A) */
    float pwmDuty;            /* < current loop control pulse width modulation(PWM) duty */
    float pwmOut;             /* < current loop control PWM final output (output = cmpst + duty) */
    float rectVoltFdbk;       /* < current loop control rectified feedback voltage(V) */
    float unitRectVoltFdbk;   /* < current loop control rectified feedback unitary voltage */
    float compensation;
    PID_Handle currPiCtrl;     /* < current loop controller define */
} PFC_CURRCTRL_Handle;
/**
  * @}
  */

/**
  * @defgroup PFC_CURRENT_CONTROLLER_API PFC_CURRENT CONTROLLER API
  * @brief The current controller's API declaration.
  * @{
  */

void PFC_CurrCtrlClear(PFC_CURRCTRL_Handle *currCtrl);

void PFC_CurrCtrlExec(PFC_CURRCTRL_Handle *currCtrl);
/**
  * @}
  */

/**
  * @}
  */
#endif  /* McuMagicTag_PFC_CURR_CTRL_H */
