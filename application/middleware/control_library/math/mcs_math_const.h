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
  * @file      mcs_math_const.h
  * @author    MCU Algorithm Team
  * @brief     This file provides math constant macro definition functionality for
  *            managing math calculation number definitions.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_MCS_MATH_CONST_H
#define McuMagicTag_MCS_MATH_CONST_H

/**
 * @addtogroup MATH
 * @brief Math const definition.
 * @{
 */

/**
  * @defgroup MATH_CONST MATH CONST
  * @brief The common math const definition for motor control.
  * @{
  */
/* Macro definitions ---------------------------------------------------------*/
#define ONE_DIV_THREE       (0.3333333f)      /**< 1/3 */
#define TWO_DIV_THREE       (0.6666667f)      /**< 2/3 */
#define ONE_PI_DIV_SIX      (0.5235988f)      /**< PI/6 */
#define ONE_PI_DIV_THREE    (1.047197f)       /**< PI/3 */
#define ONE_PI              (3.141593f)       /**< PI */
#define DOUBLE_PI_DIV_THREE (2.094395f)       /**< 2PI/3 */
#define DOUBLE_PI           (6.283185f)       /**< 2*PI */
#define SQRT3_DIV_TWO       (0.8660254f)      /**< Sqrt(3)/2 */
#define ONE_DIV_SQRT3       (0.5773503f)      /**< 1/sqrt(3) */
#define ONE_DIV_DOUBLE_PI   (0.1591549f)      /**< 1/(2*PI) */
#define RAD_TO_DEG          (57.29578f)       /**< 1/pi*180 */
#define RAD_TO_DIGITAL      (10430.06f)       /**< 1/pi*32767 */
#define DIGITAL_TO_RAD      (0.00009587673f)  /**< pi/32767 */
#define HALF_PI             (1.5707963f)      /**< 0.5*pi */
#define THREE_PI_DIV_TWO    (4.7123890f)      /**< 1.5*pi */
#define ONE_DIV_SIX         (0.16666667f)     /**< 1/6 */
#define SEVEN_DIV_SIX       (1.16666667f)     /**< 7/6 */
#define SIXTY_FIVE_DIV_SIX  (10.8333333f)     /**< 65/6 */
#define SEVENTY_ONE_DIV_SIX (11.8333333f)     /**< 71/6 */
#define ONE_DIV_NINE        (0.11111111f)     /**< 1/9 */
#define ONE_DIV_TWELVE      (0.08333333f)     /**< 1/12 */
#define SQRT2               (1.41421356f)     /**< sqrt(2) */
#define SMALL_FLOAT         (0.00000001f)
#define LARGE_FLOAT         (10000.0f)
/**
  * @}
  */

 /**
  * @}
  */

#endif /* McuMagicTag_MCS_MATH_CONST_H */
