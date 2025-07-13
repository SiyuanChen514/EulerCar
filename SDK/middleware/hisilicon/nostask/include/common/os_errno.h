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
  * @file      os_errno.h
  */
#ifndef OS_ERRNO_H
#define OS_ERRNO_H

#include "os_typedef.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/*
 * @ingroup OS_err
 * OS错误码标记位。(0x00表示Guest OS,0xFF表示DRV,0x01表示Host OS
 *
 */
#define ERRNO_OS_ID    (0x00U << 16)

/*
 * @ingroup OS_err
 * 定义错误的等级:提示级别
 *
 */
#define ERRTYPE_NORMAL (0x00U << 24)

/*
 * @ingroup OS_err
 * 定义错误的等级:告警级别
 *
 */
#define ERRTYPE_WARN   (0x01U << 24)

/*
 * @ingroup OS_err
 * 定义错误的等级:严重级别
 *
 */
#define ERRTYPE_ERROR  (0x02U << 24)

/*
 * @ingroup OS_err
 * 定义错误的等级:致命级别
 *
 */
#define ERRTYPE_FATAL  (0x03U << 24)

/*
 * @ingroup  OS_err
 * Description: 定义OS致命错误。
 *
 * @par 描述
 * 宏定义，定义OS致命错误。
 *
 * @attention 无
 *
 * @param  mid   [IN] 模块ID编号。
 * @param  errno [IN] 错误码编号。
 *
 * @retval 无
 * @par 依赖
 * os_errno.h: 该宏定义所在的头文件。
 * @see OS_ERRNO_BUILD_ERROR | OS_ERRNO_BUILD_WARN | OS_ERRNO_BUILD_NORMAL
 */
#define OS_ERRNO_BUILD_FATAL(mid, errno) (ERRTYPE_FATAL | ERRNO_OS_ID | ((unsigned int)(mid) << 8) | (errno))

/*
 * @ingroup  OS_err
 * Description: 定义OS严重错误
 *
 * @par 描述
 * 宏定义，定义OS严重错误
 *
 * @attention 无
 * @param  mid   [IN] 模块ID编号。
 * @param  errno [IN] 错误码编号。
 *
 * @retval 无
 * @par 依赖
 * os_errno.h: 该宏定义所在的头文件。
 * @see OS_ERRNO_BUILD_FATAL | OS_ERRNO_BUILD_WARN | OS_ERRNO_BUILD_NORMAL
 */
#define OS_ERRNO_BUILD_ERROR(mid, errno) (ERRTYPE_ERROR | ERRNO_OS_ID | ((unsigned int)(mid) << 8) | (errno))

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* OS_ERRNO_H */
