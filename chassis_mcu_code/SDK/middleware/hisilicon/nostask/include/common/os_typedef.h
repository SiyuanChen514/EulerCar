/**
  * @copyright Copyright (c) 2023, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      os_typedef.h
  */
#ifndef OS_TYPEDEF_H
#define OS_TYPEDEF_H

#include <stdint.h>
#include <stdbool.h>

#ifndef INLINE
#define INLINE static __inline __attribute__((always_inline))
#endif

#ifndef OS_EMBED_ASM
#define OS_EMBED_ASM __asm__ __volatile__
#endif

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

typedef void (*OsVoidFunc)(void);

#ifndef FALSE
#define FALSE ((bool)0)
#endif

#ifndef TRUE
#define TRUE ((bool)1)
#endif

#ifndef NULL
#define NULL ((void *)0)
#endif

#define OS_FAIL    1
#define OS_ERROR   (unsigned int)(-1)
#define OS_INVALID (-1)

#ifndef OS_OK
#define OS_OK 0
#endif

#ifndef U8_INVALID
#define U8_INVALID 0xffU
#endif

#ifndef U12_INVALID
#define U12_INVALID 0xfffU
#endif

#ifndef U16_INVALID
#define U16_INVALID 0xffffU
#endif

#ifndef U32_INVALID
#define U32_INVALID 0xffffffffU
#endif

#ifndef U64_INVALID
#define U64_INVALID 0xffffffffffffffffUL
#endif

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* OS_TYPEDEF_H */
