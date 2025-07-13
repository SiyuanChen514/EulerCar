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
  * @file      iwdg_ip.h
  * @author    MCU Driver Team
  * @brief     IWDG module driver
  * @details   The header file contains the following declaration:
  *             + IWDG configuration enums.
  *             + IWDG register structures.
  *             + IWDG DCL Functions.
  *             + Parameters check functions.
  */

#ifndef McuMagicTag_IWDG_IP_H
#define McuMagicTag_IWDG_IP_H

/* Includes ------------------------------------------------------------------*/
#include "baseinc.h"
/* Macro definition */
#ifdef IWDG_PARAM_CHECK
    #define IWDG_ASSERT_PARAM         BASE_FUNC_ASSERT_PARAM
    #define IWDG_PARAM_CHECK_NO_RET   BASE_FUNC_PARAMCHECK_NO_RET
    #define IWDG_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
    #define IWDG_ASSERT_PARAM(para)                ((void)0U)
    #define IWDG_PARAM_CHECK_NO_RET(para)          ((void)0U)
    #define IWDG_PARAM_CHECK_WITH_RET(param, ret)  ((void)0U)
#endif
/**
  * @addtogroup IWDG
  * @{
  */

/**
  * @defgroup IWDG_IP IWDG_IP
  * @brief IWDG_IP: iwdg_v0.
  * @{
  */

/**
  * @defgroup IWDG_Param_Def IWDG Parameters Definition
  * @brief Description of IWDG configuration parameters.
  * @{
  */
/* MACRO definitions -------------------------------------------------------*/
#define FREQ_CONVERT_MS_UNIT  1000
#define FREQ_CONVERT_US_UNIT  1000000
/* Typedef definitions -------------------------------------------------------*/
typedef enum {
    IWDG_TIME_UNIT_TICK = 0x00000000U,
    IWDG_TIME_UNIT_S = 0x00000001U,
    IWDG_TIME_UNIT_MS = 0x00000002U,
    IWDG_TIME_UNIT_US = 0x00000003U
} IWDG_TimeType;

/**
  * @brief IWDG extend handle.
  */
typedef struct _IWDG_ExtendHandle {
} IWDG_ExtendHandle;

/**
  * @brief IWDG user callback.
  */
typedef struct {
    void (* CallbackFunc)(void *handle);  /**< IWDG callback Function */
} IWDG_UserCallBack;
/**
  * @}
  */

/**
  * @defgroup IWDG_Reg_Def IWDG Register Definition
  * @brief Description IWDG register mapping structure.
  * @{
  */

/**
  * @brief enable interrupt and reset.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int wdgen : 1;    /**< enable interrupt. */
        unsigned int resen : 1;    /**< enable reset. */
        unsigned int reserved0 : 30;
    } BIT;
} volatile IWDG_CONTROL_REG;

/**
  * @brief original interrupt signal.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int wdogris : 1;    /**< original interrupt status. */
        unsigned int reserved : 31;
    } BIT;
} volatile IWDG_RIS_REG;

/**
  * @brief mask interrupt signal.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int wdogmis : 1;    /**< maske interrupt status. */
        unsigned int reserved : 31;
    } BIT;
} volatile IWDG_MIS_REG;

/**
  * @brief IWDG Register Structure definition.
  */
typedef struct {
    unsigned int     wdg_load;       /**< WDG load value register. */
    unsigned int     wdgvalue;       /**< WDG current value register. */
    IWDG_CONTROL_REG WDG_CONTROL;    /**< WDG interrupt and reset enable register. */
    unsigned int     wdg_intclr;     /**< WDG interrupt clear register. */
    IWDG_RIS_REG     WDG_RIS;        /**< WDG original interrupt register. */
    IWDG_MIS_REG     WDG_MIS;        /**< WDG mask interrupt register. */
    unsigned int     reserved0[762];
    unsigned int     wdg_lock;       /**< WDG lock register. */
} volatile IWDG_RegStruct;

/**
  * @}
  */

/**
  * @brief Setting the load value of the IWDG counter.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @param loadValue Load value of the IWDG counter.
  * @retval None.
  */
static inline void DCL_IWDG_SetLoadValue(IWDG_RegStruct *iwdgx, unsigned int loadValue)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->wdg_load = loadValue;
}

/**
  * @brief Getting the load value of the IWDG load register.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval unsigned int IWDG load value.
  */
static inline unsigned int DCL_IWDG_GetLoadValue(const IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    return iwdgx->wdg_load;
}

/**
  * @brief Getting the value of the IWDG counter register.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval unsigned int IWDG counter value.
  */
static inline unsigned int DCL_IWDG_GetCounterValue(const IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    return iwdgx->wdgvalue;
}

/**
  * @brief Clear interrupt and reload watchdog counter value.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_IWDG_Refresh(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->wdg_intclr = BASE_CFG_SET;
}

/**
  * @brief Getting value of IWDG RIS register.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval unsigned int Value of IWDG RIS register.
  */
static inline unsigned int DCL_IWDG_GetRIS(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    return iwdgx->WDG_RIS.BIT.wdogris;
}

/**
  * @brief Getting value of IWDG MIS register.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval unsigned int Value of IWDG MIS register.
  */
static inline unsigned int DCL_IWDG_GetMIS(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    return iwdgx->WDG_MIS.BIT.wdogmis;
}

/**
  * @brief Disable write and read IWDG registers except IWDG_LOCK.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_IWDG_LockReg(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->wdg_lock = BASE_CFG_SET;
}

/**
  * @brief Enable write and read IWDG registers.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_IWDG_UnlockReg(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->wdg_lock = 0x1ACCE551U; /* Unlock register value */
}

/**
  * @brief Enable reset signal.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_IWDG_EnableReset(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->WDG_CONTROL.BIT.resen = BASE_CFG_SET;
}

/**
  * @brief Disable reset signal.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_IWDG_DisableReset(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->WDG_CONTROL.BIT.resen = BASE_CFG_UNSET;
}

/**
  * @brief Start watchdog and enable interrupt signal.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_IWDG_EnableInterrupt(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->WDG_CONTROL.BIT.wdgen = BASE_CFG_SET;
}

/**
  * @brief Disable interrupt signal.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_IWDG_DisableInterrupt(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->WDG_CONTROL.BIT.wdgen = BASE_CFG_UNSET;
}

/**
  * @brief check iwdg time type parameter.
  * @param timeType Value of @ref IWDG_TimeType.
  * @retval Bool.
  */
static inline bool IsIwdgTimeType(IWDG_TimeType timeType)
{
    return (timeType == IWDG_TIME_UNIT_TICK ||
            timeType == IWDG_TIME_UNIT_S ||
            timeType == IWDG_TIME_UNIT_MS ||
            timeType == IWDG_TIME_UNIT_US);
}

/**
  * @brief check iwdg time value parameter.
  * @param baseAddress Value of @ref IWDG_RegStruct
  * @param timeValue time value
  * @param timeType Value of @ref IWDG_TimeType.
  * @retval Bool.
  */
static inline bool IsIwdgTimeValue(IWDG_RegStruct *baseAddress, float timeValue, IWDG_TimeType timeType)
{
    float clockFreq = (float)HAL_CRG_GetIpFreq((void *)baseAddress);
    float maxSecond = (float)(0xFFFFFFFF / clockFreq); /* 0xFFFFFFFF max IWDG register value */
    return ((timeType == IWDG_TIME_UNIT_TICK && timeValue <= 0xFFFFFFFF) ||
            (timeType == IWDG_TIME_UNIT_S && maxSecond >= timeValue) ||
            (timeType == IWDG_TIME_UNIT_MS && maxSecond >= timeValue / FREQ_CONVERT_MS_UNIT) ||
            (timeType == IWDG_TIME_UNIT_US && maxSecond >= timeValue / FREQ_CONVERT_US_UNIT));
}
/**
  * @}
  */

/**
  * @}
  */

#endif /* McuMagicTag_IWDG_IP_H */