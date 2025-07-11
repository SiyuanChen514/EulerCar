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
  * @file      mcs_ramp_mgmt.h
  * @author    MCU Algorithm Team
  * @brief     Ramp generation and management for motor control.
  *            This file provides functions declaration of ramp generation and management module.
  */

#ifndef McuMagicTag_MCS_RAMP_MGMT_H
#define McuMagicTag_MCS_RAMP_MGMT_H


/**
  * @brief Ramp mgmt Struct.
  */
typedef struct {
    float delta;      /**< Step value per calculate period. */
    float yLast;      /**< History value of output value. */
    float ts;         /**< Control period of the RMG module. */
    float slope;      /**< Slope, target value divide time of variation. */
} RMG_Handle;


/**
  * @defgroup RAMP_API  RAMP API
  * @brief The RAMP API definitions.
  * @{
  */
void RMG_Init(RMG_Handle *rmg, float ts, float slope);
void RMG_Clear(RMG_Handle *rmg);
float RMG_Exec(RMG_Handle *rmg, float targetVal);
void RMG_SetTs(RMG_Handle *rmg, float ts);
void RMG_SetSlope(RMG_Handle *rmg, float slope);
#endif
