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
  * @file      os_sched.h
  */
#ifndef OS_SCHED_H
#define OS_SCHED_H

#include "os_errno.h"
#include "os_module.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/*
 * 调度模块的错误码定义。
 */
/*
 * @ingroup OS_sched
 * 调度错误码：调度域配置参数非法。
 *
 * 值: 0x02004c01
 *
 * 解决方案: 检查调度域配置参数的数组指针非空时配置的个数是否为0或者超过上限核数。
 */
#define OS_ERRNO_SCHED_DOMAIN_CONFIG_INVALID                   OS_ERRNO_BUILD_ERROR(OS_MID_SCHED, 0x01)

/*
 * @ingroup OS_sched
 * 调度错误码：调度域申请内存失败。
 *
 * 值: 0x02004c02
 *
 * 解决方案: 检查是否配置的调度组数目过大，或者增加内存分区的大小。
 */
#define OS_ERRNO_SCHED_NO_MEMORY                               OS_ERRNO_BUILD_ERROR(OS_MID_SCHED, 0x02)

/*
 * @ingroup OS_sched
 * 调度错误码：调度域参数里的核掩码为0。
 *
 * 值: 0x02004c03
 *
 * 解决方案: 检查配置的调度核掩码是否正确，不能全为0.
 */
#define OS_ERRNO_SCHED_DOMAIN_INFO_COREMASK_INVALID            OS_ERRNO_BUILD_ERROR(OS_MID_SCHED, 0x03)

/*
 * @ingroup OS_sched
 * 调度错误码：调度域参数里的调度类型不正确。
 *
 * 值: 0x02004c04
 *
 * 解决方案: 检查配置的调度类型请参照OsScheduleType.
 */
#define OS_ERRNO_SCHED_DOMAIN_SCHED_TYPE_INVALID               OS_ERRNO_BUILD_ERROR(OS_MID_SCHED, 0x04)

/*
 * @ingroup OS_sched
 * 调度错误码：调度核掩码不连续。
 *
 * 值: 0x02004c05
 *
 * 解决方案: 检查配置的调度核掩码是否正确，核号必须连续，充分利用核在硬件上的共享资源。
 */
#define OS_ERRNO_SCHED_DOMAIN_CORE_MASK_NOT_CONTINUED          OS_ERRNO_BUILD_ERROR(OS_MID_SCHED, 0x05)

/*
 * @ingroup OS_sched
 * 调度错误码：调度核掩码间有交叉。
 *
 * 值: 0x02004c06
 *
 * 解决方案: 检查配置的调度域间的核掩码是否有交叉。
 */
#define OS_ERRNO_SCHED_DOMAIN_CORE_MASK_CROSSED                OS_ERRNO_BUILD_ERROR(OS_MID_SCHED, 0x06)

/*
 * @ingroup OS_sched
 * 调度错误码：指针参数为空。
 *
 * 值: 0x02004c07
 *
 * 解决方案: 检查参数指针是否为NUL。
 */
#define OS_ERRNO_SCHED_DOMAIN_INPUT_PTR_NULL                    OS_ERRNO_BUILD_ERROR(OS_MID_SCHED, 0x07)

/*
 * @ingroup OS_sched
 * 调度错误码：获取调度域信息时个数为0
 *
 * 值: 0x02004c08
 *
 * 解决方案: 检查获取调度域信息时个数是否为0
 */
#define OS_ERRNO_SCHED_DOMAIN_INFO_NUM_INVLID                   OS_ERRNO_BUILD_ERROR(OS_MID_SCHED, 0x08)

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* OS_SCHED_H */
