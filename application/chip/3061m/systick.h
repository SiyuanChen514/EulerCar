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
  * @file      systick.h
  * @author    MCU Driver Team
  * @brief     SYSTICK module driver.
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the SYSTICK.
  *                + SYSTICK register mapping structure
  *                + Get SysTick counter
  */


#ifndef McuMagicTag_SYSTICK_H
#define McuMagicTag_SYSTICK_H

/* Includes ------------------------------------------------------------------*/
#include "baseaddr.h"
#include "systickinit.h"
/**
  * @addtogroup SYSTICK
  * @{
  */

/**
  * @defgroup SYSTICK_IP SYSTICK_IP
  * @brief SYSTICK_IP: systick
  * @{
  */

/**
  * @defgroup SYSTICK_Param_Def SYSTICK Parameters Definition
  * @brief Definition of SYSTICK configuration parameters.
  * @{
  */

#define SYSTICK_MAX_VALUE 0xFFFFFFFFUL

/**
  * @}
  */

/**
 * @brief SYSTICK control register structure.
 */
typedef union {
    unsigned int   reg;
    struct {
        unsigned int   enable      : 1;  /**< Mtimer enable. */
        unsigned int   clksrc      : 1;  /**< Mtimer clock source select. */
        unsigned int   stop_tmr_en : 2;  /**< Parameter change test. */
        unsigned int   reserved    : 28;
    } BIT;
} TIMER_CTRL_REG;

/**
  * @brief SYSTICK DIV control register.
  */
typedef union {
    unsigned int   reg;
    struct {
        unsigned int   div      : 10;  /**< Timer frequency division control. */
        unsigned int   reserved : 22;
    } BIT;
} TIMER_DIV_REG;

/**
 * @brief SYSTICK register structure
 */
typedef struct {
    TIMER_CTRL_REG       TIMER_CTRL; /**< Mtimer control register. Offset address: 0x00000000U. */
    TIMER_DIV_REG        TIMER_DIV;  /**< Mtimer frequency divider register. Offset address: 0x00000004U. */
    unsigned int         MTIME;      /**< Mtimer count value lower 32-bit register. Offset address: 0x00000008U. */
    unsigned int         MTIME_H;    /**< Upper 32-bit register for Mtimer count value. Offset address: 0x0000000CU. */
    unsigned int         MTIMECMP;   /**< Mtimer comparison value lower 32-bit register. Offset address: 0x00000010U. */
    unsigned int         MTIMECMP_H; /**< Upper 32-bit Mtimer comparison value register. Offset address: 0x00000014U. */
} volatile SYSTICK_RegStruct;

/**
  * @}
  */

/**
  * @brief   Get the systick
  * @param   None
  * @retval  The SysTick Value
  */
static inline unsigned int DCL_SYSTICK_GetTick(void)
{
    return SYSTICK->MTIME;   /* Systick value(Lower 32bit register) */
}

/**
 * @}
 */
#endif /* McuMagicTag_SYSTICK_H */