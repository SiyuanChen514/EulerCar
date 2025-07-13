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
  * @file      mcs_typedef.h
  * @author    MCU Algorithm Team
  * @brief     This file provides the definition of the motor basic data structure.
  */

#ifndef McuMagicTag_MCS_TYPEDEF_H
#define McuMagicTag_MCS_TYPEDEF_H


/**
  * @defgroup MCS COORDINATE
  * @brief Motor Basic coordinate data structures.
  * @{
  */

/**
  * @brief Rotor synchronous rotation coordinate frame Variables.
  */
typedef struct {
    float d; /**< Component d of the rotor synchronous rotation coordinate variable. */
    float q; /**< Component q of the rotor synchronous rotation coordinate variable. */
} DqAxis;

/**
  * @brief  Two-phase stationary coordinate frame variable.
  */
typedef struct {
    float alpha; /**< Component alpha of the two-phase stationary coordinate variable. */
    float beta;  /**< Component beta of the two-phase stationary coordinate variable. */
} AlbeAxis;

/**
  * @brief Three-phase static coordinate frame variable.
  */
typedef struct {
    float u; /**< Component u of the three-phase static coordinate frame variable. */
    float v; /**< Component v of the three-phase static coordinate frame variable. */
    float w; /**< Component w of the three-phase static coordinate frame variable. */
} UvwAxis;


#endif  /* McuMagicTag_MCS_TYPEDEF_H */
