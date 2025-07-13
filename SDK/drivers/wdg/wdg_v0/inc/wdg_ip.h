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
  * @file      wdg_ip.h
  * @author    MCU Driver Team
  * @brief     WDG module driver
  * @details   The header file contains the following declaration:
  *             + WDG configuration enums.
  *             + WDG register structures.
  *             + WDG DCL Functions.
  *             + Parameters check functions.
  */

#ifndef McuMagicTag_WDG_IP_H
#define McuMagicTag_WDG_IP_H

/* Includes ------------------------------------------------------------------*/
#include "baseinc.h"
/* Macro definition */
#ifdef WDG_PARAM_CHECK
    #define WDG_ASSERT_PARAM         BASE_FUNC_ASSERT_PARAM
    #define WDG_PARAM_CHECK_NO_RET   BASE_FUNC_PARAMCHECK_NO_RET
    #define WDG_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
    #define WDG_ASSERT_PARAM(para)                ((void)0U)
    #define WDG_PARAM_CHECK_NO_RET(para)          ((void)0U)
    #define WDG_PARAM_CHECK_WITH_RET(param, ret)  ((void)0U)
#endif
/**
  * @addtogroup WDG
  * @{
  */

/**
  * @defgroup WDG_IP WDG_IP
  * @brief WDG_IP: wdg_v0.
  * @{
  */

/**
  * @defgroup WDG_Param_Def WDG Parameters Definition
  * @brief Description of WDG configuration parameters.
  * @{
  */
/* MACRO definitions -------------------------------------------------------*/
#define FREQ_CONVERT_MS_UNIT  1000
#define FREQ_CONVERT_US_UNIT  1000000
/* Typedef definitions -------------------------------------------------------*/
typedef enum {
    WDG_TIME_UNIT_TICK = 0x00000000U,
    WDG_TIME_UNIT_S = 0x00000001U,
    WDG_TIME_UNIT_MS = 0x00000002U,
    WDG_TIME_UNIT_US = 0x00000003U
} WDG_TimeType;

/**
  * @brief WDG extend handle.
  */
typedef struct _WDG_ExtendHandle {
} WDG_ExtendHandle;

/**
  * @brief WDG user callback.
  */
typedef struct {
    void (* CallbackFunc)(void *handle);  /**< WDG callback Function */
} WDG_UserCallBack;
/**
  * @}
  */

/**
  * @defgroup WDG_Reg_Def WDG Register Definition
  * @brief Description WDG register mapping structure.
  * @{
  */

/**
  * @brief WDG enable interrupt and reset.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int wdgen : 1;    /**< enable interrupt. */
        unsigned int resen : 1;    /**< enable reset. */
        unsigned int reserved0 : 30;
    } BIT;
} volatile WDG_CONTROL_REG;

/**
  * @brief WDG original interrupt signal.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int wdogris : 1;    /**< original interrupt status. */
        unsigned int reserved : 31;
    } BIT;
} volatile WDG_RIS_REG;

/**
  * @brief mask interrupt signal.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int wdogmis : 1;    /**< maske interrupt status. */
        unsigned int reserved : 31;
    } BIT;
} volatile WDG_MIS_REG;

/**
  * @brief WDG Register Structure definition.
  */
typedef struct {
    unsigned int    wdg_load;       /**< WDG load value register. */
    unsigned int    wdgvalue;       /**< WDG current value register. */
    WDG_CONTROL_REG WDG_CONTROL;    /**< WDG interrupt and reset enable register. */
    unsigned int    wdg_intclr;     /**< WDG interrupt clear register. */
    WDG_RIS_REG     WDG_RIS;        /**< WDG original interrupt register. */
    WDG_MIS_REG     WDG_MIS;        /**< WDG mask interrupt register. */
    unsigned int    reserved0[762];
    unsigned int    wdg_lock;       /**< WDG lock register. */
} volatile WDG_RegStruct;

/**
  * @}
  */

/**
  * @brief Setting the load value of the WDG counter.
  * @param wdgx Value of @ref WDG_RegStruct.
  * @param  loadValue Load value of the WDG counter.
  * @retval None.
  */
static inline void DCL_WDG_SetLoadValue(WDG_RegStruct *wdgx, unsigned int loadValue)
{
    WDG_ASSERT_PARAM(IsWDGInstance(wdgx));
    wdgx->wdg_load = loadValue;
}

/**
  * @brief Getting the load value of the WDG load register.
  * @param wdgx Value of @ref WDG_RegStruct.
  * @retval unsigned int WDG load value.
  */
static inline unsigned int DCL_WDG_GetLoadValue(const WDG_RegStruct *wdgx)
{
    WDG_ASSERT_PARAM(IsWDGInstance(wdgx));
    return wdgx->wdg_load;
}

/**
  * @brief Getting the value of the WDG counter register.
  * @param wdgx Value of @ref WDG_RegStruct.
  * @retval unsigned int WDG counter value.
  */
static inline unsigned int DCL_WDG_GetCounterValue(const WDG_RegStruct *wdgx)
{
    WDG_ASSERT_PARAM(IsWDGInstance(wdgx));
    return wdgx->wdgvalue;
}

/**
  * @brief Clear interrupt and reload watchdog counter value.
  * @param wdgx Value of @ref WDG_RegStruct.
  * @retval None.
  */
static inline void DCL_WDG_Refresh(WDG_RegStruct *wdgx)
{
    WDG_ASSERT_PARAM(IsWDGInstance(wdgx));
    wdgx->wdg_intclr = BASE_CFG_SET;
}

/**
  * @brief Getting value of WDG RIS register.
  * @param wdgx Value of @ref WDG_RegStruct.
  * @retval unsigned int Value of WDG RIS register.
  */
static inline unsigned int DCL_WDG_GetRIS(WDG_RegStruct *wdgx)
{
    WDG_ASSERT_PARAM(IsWDGInstance(wdgx));
    return wdgx->WDG_RIS.BIT.wdogris;
}

/**
  * @brief Getting value of WDG MIS register.
  * @param wdgx Value of @ref WDG_RegStruct.
  * @retval unsigned int Value of WDG MIS register.
  */
static inline unsigned int DCL_WDG_GetMIS(WDG_RegStruct *wdgx)
{
    WDG_ASSERT_PARAM(IsWDGInstance(wdgx));
    return wdgx->WDG_MIS.BIT.wdogmis;
}

/**
  * @brief Disable write and read WDG registers except WDG_LOCK.
  * @param wdgx Value of @ref WDG_RegStruct.
  * @retval None.
  */
static inline void DCL_WDG_LockReg(WDG_RegStruct *wdgx)
{
    WDG_ASSERT_PARAM(IsWDGInstance(wdgx));
    wdgx->wdg_lock = BASE_CFG_SET;
}

/**
  * @brief Enable write and read WDG registers.
  * @param wdgx Value of @ref WDG_RegStruct.
  * @retval None.
  */
static inline void DCL_WDG_UnlockReg(WDG_RegStruct *wdgx)
{
    WDG_ASSERT_PARAM(IsWDGInstance(wdgx));
    wdgx->wdg_lock = 0x1ACCE551U; /* Unlock register value */
}

/**
  * @brief Enable reset signal.
  * @param wdgx Value of @ref WDG_RegStruct.
  * @retval None.
  */
static inline void DCL_WDG_EnableReset(WDG_RegStruct *wdgx)
{
    WDG_ASSERT_PARAM(IsWDGInstance(wdgx));
    wdgx->WDG_CONTROL.BIT.resen = BASE_CFG_SET;
}

/**
  * @brief Disable reset signal.
  * @param wdgx Value of @ref WDG_RegStruct.
  * @retval None.
  */
static inline void DCL_WDG_DisableReset(WDG_RegStruct *wdgx)
{
    WDG_ASSERT_PARAM(IsWDGInstance(wdgx));
    wdgx->WDG_CONTROL.BIT.resen = BASE_CFG_UNSET;
}

/**
  * @brief Start watchdog and enable interrupt signal.
  * @param wdgx Value of @ref WDG_RegStruct.
  * @retval None.
  */
static inline void DCL_WDG_EnableInterrupt(WDG_RegStruct *wdgx)
{
    WDG_ASSERT_PARAM(IsWDGInstance(wdgx));
    wdgx->WDG_CONTROL.BIT.wdgen = BASE_CFG_SET;
}

/**
  * @brief Disable interrupt signal.
  * @param wdgx Value of @ref WDG_RegStruct.
  * @retval None.
  */
static inline void DCL_WDG_DisableInterrupt(WDG_RegStruct *wdgx)
{
    WDG_ASSERT_PARAM(IsWDGInstance(wdgx));
    wdgx->WDG_CONTROL.BIT.wdgen = BASE_CFG_UNSET;
}

/**
  * @brief check wdg time type parameter.
  * @param timeType Value of @ref WDG_TimeType.
  * @retval Bool.
  */
static inline bool IsWdgTimeType(WDG_TimeType timeType)
{
    return (timeType == WDG_TIME_UNIT_TICK ||
            timeType == WDG_TIME_UNIT_S ||
            timeType == WDG_TIME_UNIT_MS ||
            timeType == WDG_TIME_UNIT_US);
}

/**
  * @brief check wdg time value parameter.
  * @param baseAddress Value of @ref WDG_RegStruct
  * @param timeValue time value
  * @param timeType Value of @ref WDG_TimeType.
  * @retval Bool.
  */
static inline bool IsWdgTimeValue(WDG_RegStruct *baseAddress, float timeValue, WDG_TimeType timeType)
{
    float clockFreq = (float)HAL_CRG_GetIpFreq((void *)baseAddress);
    float maxSecond = (float)(0xFFFFFFFF / clockFreq); /* 0xFFFFFFFF max WDG register value */
    return ((timeType == WDG_TIME_UNIT_TICK && timeValue <= 0xFFFFFFFF) ||
            (timeType == WDG_TIME_UNIT_S && maxSecond >= timeValue) ||
            (timeType == WDG_TIME_UNIT_MS && maxSecond >= timeValue / FREQ_CONVERT_MS_UNIT) ||
            (timeType == WDG_TIME_UNIT_US && maxSecond >= timeValue / FREQ_CONVERT_US_UNIT));
}
/**
  * @}
  */

/**
  * @}
  */

#endif /* McuMagicTag_WDG_IP_H */