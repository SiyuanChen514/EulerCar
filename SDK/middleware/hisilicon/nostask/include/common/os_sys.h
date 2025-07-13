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
  * @file      os_sys.h
  */
#ifndef OS_SYS_H
#define OS_SYS_H

#include "os_errno.h"
#include "os_module.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：指针参数为空。
 *
 * 值: 0x02000001
 *
 * 解决方案: 请检查入参是否为空
 */
#define OS_ERRNO_SYS_PTR_NULL OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x01)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：系统主频配置非法。
 *
 * 值: 0x02000002
 *
 * 解决方案: 在nos_config.h中配置合理的系统主频。
 */
#define OS_ERRNO_SYS_CLOCK_INVALID OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x02)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：用户的配置选项OS_SYS_CORE_MAX_NUM有误，
 * 应该和芯片匹配，且OS_SYS_CORE_MAX_NUM值不能超过该芯片支持的最大核数。
 *
 * 值: 0x02000003
 *
 * 解决方案:修改配置选项OS_SYS_CORE_MAX_NUM，和芯片匹配。
 *
 */
#define OS_ERRNO_SYS_MAXNUMOFCORES_IS_INVALID OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x03)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：配置的运行核数非法。
 *
 * 值: 0x02000004
 *
 * 解决方案:配置的运行核数必须小于配置的芯片最大核数，且不能为0.
 *
 */
#define OS_ERRNO_SYS_CORE_RUNNUM_INVALID OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x04)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：获取系统时间函数Cortex-AX平台配置为NULL。
 *
 * 值: 0x02000006
 *
 * 解决方案:配置获取系统时间函数时，Cortex-AX平台必须配置为非NULL。
 *
 */
#define OS_ERRNO_SYS_TIME_HOOK_NULL OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x06)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码:镜像的个数配置错误。
 *
 * 值: 0x02000007
 *
 * 解决方案: 查看每个镜像配置的镜像个数是否不相等或者镜像个数为0或者大于最大镜像个数。
 *
 */
#define OS_ERRNO_SYS_IMAGE_NUM_INVALID OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x07)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码:镜像的镜像ID配置不可用。
 *
 * 值: 0x02000008
 *
 * 解决方案: 查看每个镜像配置的镜像ID是否有重复或者配置值错误。镜像ID值不能大于等于镜像个数。
 *
 */
#define OS_ERRNO_SYS_IMAGE_ID_INVALID OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x08)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：用户的配置选项OS_SYS_CORE_MAX_NUM有误，
 * 应该和芯片匹配，且OS_SYS_CORE_MAX_NUM值不能超过该芯片支持的最大核数。
 *
 * 值: 0x02000009
 *
 * 解决方案:修改配置选项OS_SYS_CORE_MAX_NUM，和芯片匹配。
 *
 */
#define OS_ERRNO_SYS_CORERUNNUM_IS_INVALID OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x09)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：依赖的模块功能宏没有打开。
 *
 * 值: 0x0200000a
 *
 * 解决方案:请打开相应的模块的功能宏。
 *
 */
#define OS_ERRNO_SYS_MODE_CLOSE OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x0a)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：配置的系统栈过小。
 *
 * 值: 0x0200000b
 *
 * 解决方案:请增大配置的系统栈大小。
 *
 */
#define OS_ERRNO_SYS_STACK_SIZE_NOT_ENOUGH OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x0b)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：tskmon模块功能宏依赖的任务模块功能未打开。
 *
 * 值: 0x0200000c
 *
 * 解决方案:检查依赖的任务模块裁剪开关是否打开。
 *
 */
#define OS_ERRNO_SYS_TASK_CLOSE OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x0c)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：任务饿死撑死功能控依赖的功能tskmon模块未打开。
 *
 * 值: 0x0200000d
 *
 * 解决方案:检查依赖的tskmon模块裁剪开关是否打开。
 *
 */
#define OS_ERRNO_SYS_TASKMON_CLOSE OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x0d)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：任务监功能控依赖的功能TICK未打开。
 *
 * 值: 0x0200000e
 *
 * 解决方案:检查依赖的TICK裁剪开关是否打开。
 *
 */
#define OS_ERRNO_SYS_TICK_CLOSE OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x0e)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：配置的系统模式不正确。
 *
 * 值: 0x0200000f
 *
 * 解决方案:系统模式必须为super模式或者user模式。
 *
 */
#define OS_ERRNO_SYS_MODE_INVALID OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x0f)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：CPUP告警被裁减
 *
 * 值: 0x02000011
 *
 * 解决方案:排除config项是否正确，该平台不支持CPUP告警
 *
 */
#define OS_ERRNO_SYS_NO_CPUP_WARN OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x11)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：获取或清除对应核初始化阶段状态时，核号错误。
 *
 * 值: 0x02000013
 *
 * 解决方案:确保入参核号不能大于等于OS_MAX_CORE_NUM。
 *
 */
#define OS_ERRNO_SYS_COREID_INVALID OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x13)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：软中断模块已经被裁减。
 *
 * 值: 0x0200001c
 *
 * 解决方案:排除config项是否正确，该平台不支持软中断，必须将配置项OS_TASK_INCLUDE配置为YES。
 *
 */
#define OS_ERRNO_SYS_NO_SWI OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x1c)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：init trace模块名字空间对象申请失败。
 *
 * 值: 0x0200001d
 *
 * 解决方案:扩大名字空间大小。
 *
 */
#define OS_ERRNO_SYS_INIT_TRACE_NAME_ALLOC_FAIL OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x1d)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：获取系统栈信息时指定的出参结构体为空。
 *
 * 值: 0x02000023
 *
 * 解决方案: 检查获取系统栈时传入的结构体是否为空。
 */
#define OS_ERRNO_SYS_STACKINFO_PTR_NULL OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x23)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：获取当前系统实例ID时的保存参数指针为空。
 *
 * 值: 0x02000024
 *
 * 解决方案: 请确保传入的用户存放实例ID的地址不为空。
 */
#define OS_ERRNO_SYS_INST_ID_ADDR_NULL OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x24)

/*
 * @ingroup OS_sys
 * init trace的地址为NULL
 *
 * 值: 0x02000025
 *
 * 解决方案:可能是使用过早，建议在触发后使用。
 *
 */
#define OS_ERRNO_SYS_TRACE_ADDR_NULL OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x25)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：设置RND值的时候入参非法。
 *
 * 值: 0x02000026
 *
 * 解决方案: 请确保设置RND值时入参合法。
 */
#define OS_ERRNO_SYS_RND_PARA_INVALID  OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x26)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：注册获取系统时间函数重复注册了
 *
 * 值: 0x02000027
 *
 * 解决方案:获取系统时间函数不允许重复注册
 *
 */
#define OS_ERRNO_SYS_TIME_HOOK_REGISTER_REPEATED OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x27)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：内存分区过小。
 *
 * 值: 0x02000028
 *
 * 解决方案: 请确保分区待截取的size小于分区大小。
 */
#define OS_ERRNO_SYS_RND_OVERFLOW_PTSIZE  OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x28)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：自检小版本Idle激活失败。
 *
 * 值: 0x02000029
 *
 * 解决方案: 请确保Idle钩子注册。
 */
#define OS_ERRNO_SYS_IDLE_ACTIVE_FAILED  OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x29)

/*
 * @ingroup OS_sys
 * 系统基本功能错误码：配置的中断个数不正确
 *
 * 值: 0x0200002a
 *
 * 解决方案: 检查配置的中断个数是否为0或者超过了硬件范围
 */
#define OS_ERRNO_SYS_HWI_MAX_NUM_CONFIG_INVALID OS_ERRNO_BUILD_ERROR(OS_MID_SYS, 0x2a)

/*
 * 系统初始化阶段状态
 */
/*
 * @ingroup OS_sys
 * 表示初始态。
 *
 */
#define OS_DEFAULT_PHASE 0x00

/*
 * @ingroup OS_sys
 * 表示主核处于实模式启动阶段状态（该阶段主要完成MMU使能初始化），仅SD6183平台有效。
 *
 */
#define OS_MCORE_PHY_BOOT_PHASE 0x01

/*
 * @ingroup OS_sys
 * 表示主核处于虚模式启动阶段状态，仅SD6183平台有效。
 *
 */
#define OS_MCORE_VIR_BOOT_PHASE 0x02

/*
 * @ingroup OS_sys
 * 表示进入NOS_HardBootInit。
 *
 */
#define OS_BOOT_PHASE 0x03

/*
 * @ingroup OS_sys
 * 表示开始多实例启动参数配置。
 *
 */
#define OS_STARTPARAMCFG_PHASE 0x04

/*
 * @ingroup OS_sys
 * 表示退出NOS_HardBootInit。
 *
 */
#define OS_AFTER_BOOT_PHASE 0x05

/*
 * @ingroup OS_sys
 * 表示进入BSS段初始化。
 *
 */
#define OS_BSSINIT_PHASE 0x06

/*
 * @ingroup OS_sys
 * 表示Dcache初始化完毕，SD6183平台由于在reset中已经完成，故不支持该阶段状态。
 *
 */
#define OS_DCACHEINIT_PHASE 0x07

/*
 * @ingroup OS_sys
 * 表示进入C lib库初始化。
 *
 */
#define OS_LIBCINIT_PHASE 0x08

/*
 * @ingroup OS_sys
 * 表示系统在进行OS模块注册阶段，匹配MOUDLE_ID之后，标记进入MODULE_ID的注册。
 *
 */
#define OS_REGISTER_PHASE 0x09

/*
 * @ingroup OS_sys
 * 表示系统在进行OS模块初始化阶段，匹配MOUDLE_ID之后，标记进入MODULE_ID的初始化。
 *
 */
#define OS_INITIALIZE_PHASE 0x0a

/*
 * @ingroup OS_sys
 * 表示系统在进行产品驱动初始化阶段，匹配MOUDLE_ID之后，标记进入MODULE_ID的初始化。
 *
 */
#define OS_DEVDRVINIT_PHASE 0x0b

/*
 * @ingroup OS_sys
 * 表示系统在进行OS启动阶段，匹配MOUDLE_ID之后，标记进入MODULE_ID的启动。
 *
 */
#define OS_START_PHASE 0x0c

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif /* OS_SYS_H */
