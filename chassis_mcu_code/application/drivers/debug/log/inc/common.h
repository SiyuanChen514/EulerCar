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
  * @file    common.h
  * @author  MCU Driver Team
  * @brief   Header file containing functions prototypes of cmd module.
  *          + Initialization and de-initialization functions
  *          + Format common function
  */
#ifndef COMMON_H
#define COMMON_H

#include "type.h"
#include "ext_log.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

/**
  * @addtogroup DEBUG_Log
  * @brief DEBUG external module.
  * @{
  */

 /**
  * @defgroup CONFIG_Def CONFIG_Def
  * @brief Processing Special Characters.
  * @{
  */

#define EXT_ARRAY_COUNT(x) (sizeof(x) / sizeof(x[0]))
#define EXT_ALIGN_4(x)     ((unsigned int)(x + 0x3) & (~0x3))
#define CHAR_CR        '\r' /* 0x0D */
#define CHAR_LF        '\n' /* 0x0A */

#define EXT_REG_WRITE(addr, val)            (*(volatile unsigned int *)(addr) = (val)) /* Write by register address */
#define EXT_REG_READ(addr, val)             ((val) = *(volatile unsigned int *)(addr)) /* Read by register address */
#define EXT_REG_READ32(addr)                (*(volatile unsigned int *)(addr))
#define EXT_REG_WRITE32(addr, val)          (*(volatile unsigned int *)(addr) = (val))
#define EXT_REG_WRITE_MASK(addr, val, mask) (*(volatile unsigned int *)((addr) & 0xFFFFFFFC) = \
    ((*(volatile unsigned int *)((addr)& 0xFFFFFFFC)) & (~(mask))) | ((val) & (mask)))
#define EXT_REG_TOOLWRITE(addr, val)        (*(volatile unsigned int *)((addr) & 0xFFFFFFFC) = (val))
#define EXT_REG_TOOLREAD(addr, val)         ((val) = *(volatile unsigned int *)((addr) & 0xFFFFFFFC))

#define ABS(x) (((x) >= 0) ? (x) : -(x))
#define MAX(a, b) (((a) >= (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define CLIP(a) (((a) >= 0) ? (a) : 0)
#define CLIP2(m, n, a) (((a) > (m)) ? (m) : ((a) < (n) ? (n) : (a)))
#define CLIP3(low, high, x) (MAX(MIN((x), high), low))
#define RSHFT(x, n) ((x) >= 1 ? \
    (((x) + (1 << ((n)-1))) >> (n)) : (-(((-(x)) + (1 << ((n)-1))) >> (n))))

#define ROUND_UP(x, align) (((x) + (align)-1) & ~((align)-1))
#define ROUND_DOWN(x, align)  ((x) & (~((align) - 1)))

#define EXT_FENCE(void) do {    \
    __asm__("fence\n\r"); \
} while (0)
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */
/**
  * @}
  */

/**
  * @}
  */
#endif /* __EXT_COMMON_H__ */

