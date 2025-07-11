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
  * @file      mcs_ctlmode_config.h
  * @author    MCU Algorithm Team
  * @brief     This file provides config macros for ECMCU105H app.
  */

 /* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_CTLMODECONFIG_H
#define McuMagicTag_MCS_CTLMODECONFIG_H

#include "debug.h"
#include "typedefs.h"

typedef enum {
    FOC_STARTUP_IF = 0,
    FOC_STARTUP_VF,
    FOC_STARTUP_HFI
} MOTOR_STARTUPMODE_CONFIG;

typedef enum {
    FOC_OBSERVERTYPE_SMO1TH = 0,
    FOC_OBSERVERTYPE_SMO1TH_PLL,
    FOC_OBSERVERTYPE_SMO4TH,
    FOC_OBSERVERTYPE_SMO4TH_PLL,
    FOC_OBSERVERTYPE_LUNBORG,
    FOC_OBSERVERTYPE_FLUX
} MOTOR_OBSERVERTYPE_CONFIG;

typedef enum {
    FOC_CONTROLMODE_SPEED = 0,
    FOC_CONTROLMODE_TORQUE
} MOTOR_CONTROLMODE_CONFIG;

typedef enum {
    FOC_CURQAXISPID_PARAMS = 0,
    FOC_CURDAXISPID_PARAMS,
    FOC_SPDPID_PARAMS
} MOTOR_PID_SET;

typedef enum {
    MOTOR_PARAMS_BASE = 0,
    MOTOR_PARAMS_SPECIAL,
    MOTOR_PARAMS_BOARD
} MOTOR_PARAMS_SET;

typedef enum {
    CUST_SPEED_ADJUST = 0,
    HOST_SPEED_ADJUST
} MODE_ADSPEED_CONFIG;

typedef enum {
    CONNECTING = 0,
    CONNECTED,
    DISCONNECT
} UART_STATUS;

#endif