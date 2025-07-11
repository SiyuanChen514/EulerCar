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
  * @file      base_math.h
  * @author    MCU Driver Team
  * @brief     BASE module driver
  * @details   This file provides functions declaration of math
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_BASE_MATH_H
#define McuMagicTag_BASE_MATH_H

/* Includes ------------------------------------------------------------------ */
#include "chipinc.h"
#include "typedefs.h"
#include "assert.h"

/**
  * @defgroup BASE BASE
  * @brief BASE module.
  * @{
  */

/**
  * @defgroup MATH Math Definition
  * @brief Definition of MATH Definition.
  * @{
  */

/**
  * @defgroup MATH_STRUCTURE_DEFINITION math structure Definition
  * @brief Definition of math structure Definition.
  * @{
  */
/* Typedef definitions ------------------------------------------------------- */
/**
  * @brief sin, cos status definition.
  */
typedef struct {
    int sin : 16;
    int cos : 16;
} BASE_MathTypeSinCos;

/**
  * @brief q-axis d-axis status definition.
  */
typedef struct {
    int q : 16;
    int d : 16;
} BASE_MathTypeQD;

/**
  * @brief current component a,b status definition.
  */
typedef struct {
    int a : 16;
    int b : 16;
} BASE_MathTypeAB;

/**
  * @brief alpha-axis beta-axis status definition.
  */
typedef struct {
    int alpha : 16;
    int beta : 16;
} BASE_MathTypeAlphaBeta;
/**
  * @}
  */
/**
  * @defgroup MATH_API_DEFINITION Math API
  * @brief Definition of math API Definition.
  * @{
  */
/* Macro definitions --------------------------------------------------------- */
#define BASE_MATH_ABS(x)  ((x) < 0 ? -(x) : (x))

/* Radian to angle. */
#define BASE_MATH_RADIAN_TO_ANGLE(radian) ((radian) * 57.295779524)

/* Exported global functions ------------------------------------------------- */
BASE_MathTypeSinCos BASE_MATH_GetSinCos(short angle);
float BASE_MATH_GetSin(float angle);
float BASE_MATH_GetCos(float angle);
float BASE_MATH_Sqrt(const float x);
float BASE_MATH_Pow(float x, int n);
BASE_MathTypeAlphaBeta BASE_MATH_Clarke(BASE_MathTypeAB input);
BASE_MathTypeQD BASE_MATH_Park(BASE_MathTypeAlphaBeta input, short theta);
BASE_MathTypeAlphaBeta BASE_MATH_RevPark(BASE_MathTypeQD input, short theta);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* McuMagicTag_BASE_MATH_H */