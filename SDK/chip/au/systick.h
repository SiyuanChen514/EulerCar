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

#ifdef NOS_TASK_SUPPORT
#ifndef CFG_SYSTICK_TICKINTERVAL_US
#define CFG_SYSTICK_TICKINTERVAL_US 100
#endif
unsigned int SYSTICK_GetTickInterval(void);
#endif

#define SYSTICK_MAX_VALUE 0xFFFFFFFFUL

unsigned int SYSTICK_GetCRGHZ(void);
unsigned int DCL_SYSTICK_GetTick(void);
unsigned int SYSTICK_GetTimeStampUs(void);

/**
 * @}
 */
#endif /* McuMagicTag_SYSTICK_H */
