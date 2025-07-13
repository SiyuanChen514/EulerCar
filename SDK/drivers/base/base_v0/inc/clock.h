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
  * @file      clock.h
  * @author    MCU Driver Team
  * @brief     BASE module driver
  * @brief     Include the header file of the clock.c file.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_CLOCK_H
#define McuMagicTag_CLOCK_H

/* Includes ------------------------------------------------------------------ */
#include "chipinc.h"
#include "typedefs.h"

/**
  * @defgroup BASE BASE
  * @brief BASE module.
  * @{
  */

/**
  * @defgroup CLOCK Clock Definition
  * @brief Definition of Clock Definition.
  * @{
  */

/**
  * @defgroup CLOCK_ENUM_DEFINITION Delay Enum Definition
  * @brief Definition of BASE_DelayUnit enum
  * @{
  */

/* Typedef definitions ------------------------------------------------------- */
/**
  * @brief Multiples of the parameters of the delay function based on microseconds in different time units.
  * @details BASE_DelayUnit:
  *           + BASE_DEFINE_DELAY_SECS -- Needed delay amount is in seconds
  *           + BASE_DEFINE_DELAY_MILLISECS -- Needed delay amount is in milliseconds
  *           + BASE_DEFINE_DELAY_MICROSECS -- Needed delay amount is in microseconds
  */
typedef enum {
    BASE_DEFINE_DELAY_SECS = 1,
    BASE_DEFINE_DELAY_MILLISECS = 1000,
    BASE_DEFINE_DELAY_MICROSECS = 1000000
} BASE_DelayUnit;
/**
  * @}
  */

/**
  * @defgroup CLOCK_MACRO_DEFINITION Delay Macro Function Definition
  * @brief Definition of BASE_DelayUnit macro.
  * @{
  */
/* Macro definitions --------------------------------------------------------- */
#define BASE_DEFINE_DELAY_MS_IN_SEC 1000
#define BASE_DEFINE_DELAY_US_IN_MS  1000

#define BASE_FUNC_DELAY_S(n)  BASE_FUNC_Delay(n, BASE_DEFINE_DELAY_SECS)
#define BASE_FUNC_DELAY_MS(n) BASE_FUNC_Delay(n, BASE_DEFINE_DELAY_MILLISECS)
#define BASE_FUNC_DELAY_US(n) BASE_FUNC_Delay(n, BASE_DEFINE_DELAY_MICROSECS)
/**
  * @}
  */

/**
  * @defgroup CLOCK_API_DEFINITION Clock Delay API
  * @brief Definition of clcok API.
  * @{
  */
/* Exported global functions ------------------------------------------------------------------ */
unsigned int BASE_FUNC_GetCpuFreqHz(void);
void BASE_FUNC_Delay(unsigned int delay, BASE_DelayUnit units);
void BASE_FUNC_DelayUs(unsigned int us);
void BASE_FUNC_DelayMs(unsigned int ms);
void BASE_FUNC_DelaySeconds(unsigned int seconds);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* McuMagicTag_CLOCK_H */