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
  * @file      typedefs.h
  * @author    MCU Driver Team
  * @brief     BASE module driver
  * @brief     This file contains generic definitions
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_TYPEDEFS_H
#define McuMagicTag_TYPEDEFS_H
/**
  * @defgroup BASE BASE
  * @brief BASE module.
  * @{
  */

/**
  * @defgroup TYPRDEF Typedef Definition
  * @brief Definition of RESET Definition.
  * @{
  */

/**
  * @defgroup TYPEDEF_MACRO_DEFINITION TYPEDEF MACRO Definition
  * @brief Definition of TYPEDEF MACRO Definition.
  * @{
  */
/* Macro definitions --------------------------------------------------------- */
#ifndef bool
#define bool _Bool
#endif /* bool */

#ifndef false
#define false 0
#endif /* false */

#ifndef true
#define true 1
#endif /* true */

#ifndef NULL
#define NULL ((void *)0)
#endif /* NULL */

#ifndef FLT_EPSILON
#define FLT_EPSILON 0.000001
#endif /* float min error definition */

#ifndef INT16_MAX
#define INT16_MAX 0x7FFF
#endif /* INT16_MAX */

#ifndef INT16_MIN
#define INT16_MIN (-0x8000)
#endif /* INT16_MIN */

#ifndef INT_MAX
#define INT_MAX 0x7FFFFFFF
#endif /* INT_MAX */

#ifndef UINT_MAX
#define UINT_MAX 0xFFFFFFFFU
#endif /* UINT_MAX */

#define BASE_FUNC_UNUSED(X) (void)(X)

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

#define BASE_CFG_UNSET 0x00
#define BASE_CFG_SET 0x01

#define BASE_CFG_DISABLE 0x00
#define BASE_CFG_ENABLE 0x01

#define RAM_CODE __attribute__((section(".text.sram")))
#define RESERVED_DATA __attribute__((section(".reserved.data")))

typedef int intptr_t;
typedef unsigned int uintptr_t;
/**
  * @}
  */

/**
  * @defgroup TYPEDEF_ENUM_DEFINITION TYPEDEF ENUM Definition
  * @brief Definition of TYPEDEF ENUM Definition.
  * @{
  */
/**
  * @brief  BASE Status structures definition
  */
typedef enum {
    BASE_STATUS_OK          = 0x00000000U,
    BASE_STATUS_ERROR       = 0x00000001U,
    BASE_STATUS_BUSY        = 0x00000002U,
    BASE_STATUS_TIMEOUT     = 0x00000003U,
    BASE_STATUS_NOT_SUPPORT = 0x00000004U,
} BASE_StatusType;

/**
  * @brief Indicates the status of the general state machine. The user should add the service status to this enum.
  */
typedef enum {
    BASE_FSM_START,
    BASE_DEFINE_FSM_END
} BASE_FSM_Status;

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* McuMagicTag_TYPEDEFS_H */