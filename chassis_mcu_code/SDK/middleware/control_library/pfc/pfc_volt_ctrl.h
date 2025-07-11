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
  * @file      pfc_volt_ctrl.h
  * @author    MCU Algorithm Team
  * @brief     Voltage loop control. This file provides function of power factor correction(PFC) voltage control
  */
/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_PFC_VOLT_CTRL_H
#define McuMagicTag_PFC_VOLT_CTRL_H

/* Includes ------------------------------------------------------------------------------------ */
#include "mcs_pid_ctrl.h"

/**
  * @defgroup VOLTAGE_CONTROLLER VOLTAGE CONTROLLER MODULE
  * @brief The voltage controller function.
  * @{
  */

/**
  * @defgroup VOLTAGE_CONTROLLER_STRUCT VOLTAGE CONTROLLER STRUCT
  * @brief The voltage controller's data structure definition.
  * @{
  */

/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief Voltage controller struct.
  */
typedef struct {
    float uniVoltRef;           /* < voltage loop control unitary reference voltage(V) */
    float voltFdbk;          /* < voltage loop control feedback voltage(V) */
    float unitVoltFdbk;      /* < voltage loop control feedback unitary voltage */
    float startVolt;         /* < voltage loop control start voltage(V) */
    float voltOut;           /* < voltage loop control output */
    PID_Handle voltPiCtrl;    /* < voltage loop controller define */
} PFC_VOLTCTRL_Handle;
/**
  * @}
  */

/**
  * @defgroup VOLTAGE_CONTROLLER_API VOLTAGE CONTROLLER API
  * @brief The voltage controller's API declaration.
  * @{
  */
void PFC_VoltCtrlClear(PFC_VOLTCTRL_Handle *voltCtrl);

void PFC_VoltCtrlExec(PFC_VOLTCTRL_Handle *voltCtrl);
/**
  * @}
  */

/**
  * @}
  */
#endif  /* McuMagicTag_PFC_VOLT_CTRL_H */
