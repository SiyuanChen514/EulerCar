/**
  * Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file   mcs_pos_ctrl.h
  * @author MCU Algorithm Team
  * @brief  This file provides functions declaration of position control .
  */
/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_MCS_POS_CTRL_H
#define McuMagicTag_MCS_MCS_POS_CTRL_H

/* Includes ------------------------------------------------------------------------------------ */
#include "mcs_typedef.h"
#include "mcs_pid_ctrl.h"
#include "mcs_ramp_mgmt.h"
#include "mcs_mtr_param.h"


/* Macro definitions --------------------------------------------------------------------------- */

/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief Position control mode.
  */
typedef enum {
    POSCTRL_MODE_CONTINUOUS = 0,
    POSCTRL_MODE_TRAJ
} POSCTRL_Mode;

/**
  * @brief Position controller struct members and parameters.
  */
typedef struct {
    PID_Handle posPi;           /**< PI controller struct in the position controller. */
    float posTarget;           /**< position controller input target value (rad) */
    float posTargetBk;
    RMG_Handle posRmg;          /**< position reference ramp management . */
    float ts;                  /**< position controller control period (s). */
    int angFbkLoop;            /**< feedback position loop count. */
    float angFbkPrev;          /**< feedback position in last cycle (rad). */
    float posFbk;              /**< feedback position absolutely (rad). */
    float posIncRef;           /**< position controller reference (rad) */
    float posIncRefPrev;       /**< position controller reference in last cycle (rad) */
    float spdRef;              /**< position controller outpur speed reference (Hz) */
    float posRef;              /**< position controller reference (rad) */
    float posFbkPrev;
    float posErr;

    /* trajectory planning */
    /* position controller work mode. 0: continuous mode; 1: trajectory control mode. */
    /* trajectory mode can only be enabled when input mode is set absolute position.  */
    POSCTRL_Mode mode;
    float posTargetShadow;
    float runTime;             /**< single trajectory control last time time (s). */
    float timeTick;            /**< trajectory control inner timer (s) */
    float deltaTime;
    float accMax;
    float jerk;
    int targetUpdateBlockFlag;  /**< whether the position target can be update or not. 0: can be updated; 1: block */
    float deltaTimeSq;
    float deltaTimeCu;
    float timeStg[7];
} POSCTRL_Handle;

/**
  * @defgroup POSITION_CONTROLLER_API  POSITION CONTROLLER API
  * @brief The position controller API declaration.
  * @retval Speed Reference.
  */
void POSCTRL_Clear(POSCTRL_Handle *posHandle);
void POSCTRL_Init(POSCTRL_Handle *posHandle, const PID_Param *piCtrlTable, float ts);
float POSCTRL_PidExec(POSCTRL_Handle *posHandle, float posErr);
void POSCTRL_ModeSelect(POSCTRL_Handle *posHandle, POSCTRL_Mode mode);
void POSCTRL_SetSlope(POSCTRL_Handle *posHandle, float slope);
void POSCTRL_SetTarget(POSCTRL_Handle *posHandle, float posTarget);
float POSCTRL_Exec(POSCTRL_Handle *posHandle, float spdTarget, float posFbk);
float POSCTRL_AngleExpand(POSCTRL_Handle *posHandle, float angFbk);
void POSCTRL_SetKp(POSCTRL_Handle *posHandle, float kp);
void POSCTRL_SetKi(POSCTRL_Handle *posHandle, float ki);
void POSCTRL_SetKd(POSCTRL_Handle *posHandle, float kd);
void POSCTRL_SetNs(POSCTRL_Handle *posHandle, float ns);

#endif /* McuMagicTag_MCS_POS_CTRL_H */