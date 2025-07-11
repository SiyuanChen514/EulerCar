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
  * @file      mcs_pid_ctrl.h
  * @author    MCU Algorithm Team
  * @brief     General PI controller.
  *            This file provides functions declaration of the PI controller module.
  */

/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_PID_CTRL_H
#define McuMagicTag_MCS_PID_CTRL_H

/**
  * @defgroup PID  PID
  * @brief The PID module.
  * @{
  */

/**
  * @defgroup PID_STRUCT  PID STRUCT
  * @brief The PID control structure definition.
  * @{
  */
/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief General PID Controller struct members and parameters.
  */
typedef struct {
    float error;       /**< Error feedback. */
    float errorLast;   /**< Error feedback history values. */
    float feedforward; /**< Feedforward item. */
    float integral;    /**< Integral item. */
    float saturation;  /**< Saturation value of the integral item. */
    float differ;      /**< Differential item. */
    float kp;          /**< Gained of the proportional item. */
    float ki;          /**< Gained of the integral item, not multiplied by control period. */
    float kd;          /**< Gained of the differential item. */
    float ns;          /**< Filter parameter of the differential item. */
    float ka;          /**< Gained of the saturation item. */
    float ts;          /**< Control period (s) */
    float upperLimit;  /**< The upper limit value of the pid comp output. */
    float lowerLimit;  /**< The lower limit value of the pid output. */
} PID_Handle;

typedef struct {
    float kp;
    float ki;
    float upperLim;
    float lowerLim;
} PI_Param;

typedef struct {
    float kp;
    float ki;
    float kd;
    float ns;          /**< Filter parameter of the differential item. */
    float ka;          /**< Gained of the saturation item. */
    float saturation;
    float upperLim;
    float lowerLim;
} PID_Param;
/**
  * @}
  */

/**
  * @defgroup PID_API  PID API
  * @brief The PID control API definitions.
  * @{
  */
void PID_Reset(PID_Handle *pidHandle);
void PID_Clear(PID_Handle *pidHandle);
float PI_Exec(PID_Handle *pidHandle);
float PID_Exec(PID_Handle *pidHandle);

void PID_SetKp(PID_Handle *pidHandle, float kp);
void PID_SetKi(PID_Handle *pidHandle, float ki);
void PID_SetKd(PID_Handle *pidHandle, float kd);
void PID_SetNs(PID_Handle *pidHandle, float ns);
void PID_SetTs(PID_Handle *pidHandle, float ts);
void PID_SetLimit(PID_Handle *pidHandle, float limit);
/**
  * @}
  */

/**
  * @}
  */

#endif
