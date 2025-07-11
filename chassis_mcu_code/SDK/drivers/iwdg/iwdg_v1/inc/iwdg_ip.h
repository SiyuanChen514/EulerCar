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
  * @brief IWDG_IP: iwdg_v1.
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
#define IWDG_UNLOCK_REG_CMD    0x55 /* 0x55 CMD: key equal 0x55 will unlock all reg write function */
#define IWDG_LOCK_REG_CMD      0xFF /* 0xFF CMD: key not equal 0x55 will lock reg write function except key reg */

/* Typedef definitions -------------------------------------------------------*/
typedef enum {
    IWDG_TIME_UNIT_TICK = 0x00000000U,
    IWDG_TIME_UNIT_S = 0x00000001U,
    IWDG_TIME_UNIT_MS = 0x00000002U,
    IWDG_TIME_UNIT_US = 0x00000003U
} IWDG_TimeType;

typedef enum {
    IWDG_FREQ_DIV_NONE = 0x00000000U,
    IWDG_FREQ_DIV_2 = 0x00000001U,
    IWDG_FREQ_DIV_4 = 0x00000002U,
    IWDG_FREQ_DIV_8 = 0x00000003U,
    IWDG_FREQ_DIV_16 = 0x00000004U,
    IWDG_FREQ_DIV_32 = 0x00000005U,
    IWDG_FREQ_DIV_64 = 0x00000006U,
    IWDG_FREQ_DIV_128 = 0x00000007U,
    IWDG_FREQ_DIV_256 = 0x00000008U,
    IWDG_FREQ_DIV_MAX
} IWDG_FreqDivType;

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
  * @brief IWDG load init value.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int iwdg_load : 8;    /**< init value. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile IWDG_LOAD_REG;

/**
  * @brief IWDG get current value.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int iwdg_value : 8;    /**< current value. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile IWDG_VALUE_REG;

/**
  * @brief IWDG set window value.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int iwdg_window : 8;    /**< window value. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile IWDG_WINDOW_REG;

/**
  * @brief IWDG cmd function value.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int iwdg_key : 8;    /**< cmd function value. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile IWDG_KEY_REG;

/**
  * @brief IWDG clk pre div value.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int iwdg_pre_div : 4;    /**< clk pre div value. */
        unsigned int reserved0 : 28;
    } BIT;
} volatile IWDG_PRE_DIV_REG;

/**
  * @brief IWDG enable interrupt and reset.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int inten : 1;    /**< enable interrupt. */
        unsigned int resen : 1;    /**< enable reset. */
        unsigned int window_mode_en : 1;    /**< enable window mode. */
        unsigned int reserved0 : 29;
    } BIT;
} volatile IWDG_CONTROL_REG;

/**
  * @brief IWDG orignal interrupt signal.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int iwdogris : 1;    /**< original interrupt status. */
        unsigned int reserve : 31;
    } BIT;
} volatile IWDG_RIS_REG;

/**
  * @brief mask interrupt signal.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int iwdogmis : 1;    /**< maske interrupt status. */
        unsigned int reserve : 31;
    } BIT;
} volatile IWDG_MIS_REG;

/**
  * @brief IWDG Register Structure definition.
  */
typedef struct {
    IWDG_LOAD_REG     IWDOG_LOAD;       /**< IWDG load value register. */
    IWDG_VALUE_REG    IWDOG_VALUE;      /**< IWDG current value register. */
    IWDG_WINDOW_REG   IWDOG_WINDOW;     /**< IWDG Window value register. */
    IWDG_KEY_REG      IWDOG_KEY;        /**< IWDG instruction word register. */
    IWDG_PRE_DIV_REG  IWDOG_PRE_DIV;    /**< IWDG prescale register. */
    IWDG_CONTROL_REG  IWDOG_CONTROL;    /**< IWDG interrupt, reset and window enable register. */
    IWDG_RIS_REG      IWDOG_RIS;        /**< IWDG orignal interrupt register. */
    IWDG_MIS_REG      IWDOG_MIS;        /**< IWDG mask interrupt register. */
} volatile IWDG_RegStruct;

/**
  * @}
  */

/**
  * @brief Setting the load value of the IWDG counter.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @param  loadValue Load value of the IWDG counter.
  * @retval None.
  */
static inline void DCL_IWDG_SetLoadValue(IWDG_RegStruct *iwdgx, unsigned char loadValue)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->IWDOG_KEY.BIT.iwdg_key = IWDG_UNLOCK_REG_CMD;
    iwdgx->IWDOG_LOAD.BIT.iwdg_load = loadValue;
    iwdgx->IWDOG_KEY.BIT.iwdg_key = IWDG_LOCK_REG_CMD;
}

/**
  * @brief Getting the load value of the IWDG load register.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval unsigned char IWDG load value.
  */
static inline unsigned char DCL_IWDG_GetLoadValue(const IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    return iwdgx->IWDOG_LOAD.BIT.iwdg_load;
}

/**
  * @brief Getting the value of the IWDG counter register.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval unsigned char IWDG counter value.
  */
static inline unsigned char DCL_IWDG_GetCounterValue(const IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    return iwdgx->IWDOG_VALUE.BIT.iwdg_value;
}

/**
  * @brief Setting window value, windowValue need bigger than 4.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @param windowValue window value of the IWDG counter.
  * @retval None.
  */
static inline void DCL_IWDG_SetWindowValue(IWDG_RegStruct *iwdgx, unsigned char windowValue)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    IWDG_PARAM_CHECK_NO_RET(windowValue > 4); /* litter than 4 could be error */
    iwdgx->IWDOG_KEY.BIT.iwdg_key = IWDG_UNLOCK_REG_CMD;
    iwdgx->IWDOG_WINDOW.BIT.iwdg_window = windowValue;
    iwdgx->IWDOG_KEY.BIT.iwdg_key = IWDG_LOCK_REG_CMD;
}

/**
  * @brief Getting window value, windowValue need bigger than 4.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @param windowValue window value of the IWDG counter.
  * @retval unsigned char iwdg window value.
  */
static inline unsigned char DCL_IWDG_GetWindowValue(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    return iwdgx->IWDOG_WINDOW.BIT.iwdg_window;
}

/**
  * @brief Start iwdg function.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_IWDG_Start(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->IWDOG_KEY.BIT.iwdg_key = 0xCC; /* 0xCC CMD: start iwdg function */
}

/**
  * @brief Stop iwdg function.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_IWDG_Stop(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->IWDOG_KEY.BIT.iwdg_key = 0xDD; /* 0xDD CMD: stop iwdg function */
}

/**
  * @brief Clear interrupt and reload watchdog counter value.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_IWDG_Refresh(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->IWDOG_KEY.BIT.iwdg_key = 0xAA; /* 0xAA CMD: clear interrupt and reload value */
}

/**
  * @brief Disable write and read IWDG registers except IWDG_LOCK.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_IWDG_LockReg(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->IWDOG_KEY.BIT.iwdg_key = 0xFF; /* 0xFF CMD: key not equal 0x55 will lock reg write function except key reg */
}

/**
  * @brief Enable write and read IWDG registers.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_IWDG_UnlockReg(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->IWDOG_KEY.BIT.iwdg_key = 0x55; /* 0x55 CMD: key equal 0x55 will unlock all reg write function */
}

/**
  * @brief Setting freq div value.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @param  freqDiv freqDiv value of the IWDG counter.
  * @retval None.
  */
static inline void DCL_IWDG_SetFreqDivValue(IWDG_RegStruct *iwdgx, IWDG_FreqDivType freqDiv)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    IWDG_PARAM_CHECK_NO_RET(freqDiv < IWDG_FREQ_DIV_MAX);
    iwdgx->IWDOG_KEY.BIT.iwdg_key = IWDG_UNLOCK_REG_CMD;
    iwdgx->IWDOG_PRE_DIV.BIT.iwdg_pre_div = freqDiv; /* freqDiv parameters set */
    iwdgx->IWDOG_KEY.BIT.iwdg_key = IWDG_UNLOCK_REG_CMD;
}

/**
  * @brief Getting freq div value.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @param  freqDiv freqDiv value of the IWDG counter.
  * @retval None.
  */
static inline unsigned char DCL_IWDG_GetFreqDivValue(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    return iwdgx->IWDOG_PRE_DIV.BIT.iwdg_pre_div;
}

/**
  * @brief Enable reset signal.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_IWDG_EnableReset(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->IWDOG_KEY.BIT.iwdg_key = IWDG_UNLOCK_REG_CMD;
    iwdgx->IWDOG_CONTROL.BIT.resen = BASE_CFG_SET;
    iwdgx->IWDOG_KEY.BIT.iwdg_key = IWDG_LOCK_REG_CMD;
}

/**
  * @brief Disable reset signal.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_IWDG_DisableReset(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->IWDOG_KEY.BIT.iwdg_key = IWDG_UNLOCK_REG_CMD;
    iwdgx->IWDOG_CONTROL.BIT.resen = BASE_CFG_UNSET;
    iwdgx->IWDOG_KEY.BIT.iwdg_key = IWDG_LOCK_REG_CMD;
}

/**
  * @brief Start watchdog and enable interrupt signal.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_IWDG_EnableInterrupt(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->IWDOG_KEY.BIT.iwdg_key = IWDG_UNLOCK_REG_CMD;
    iwdgx->IWDOG_CONTROL.BIT.inten = BASE_CFG_SET;
    iwdgx->IWDOG_KEY.BIT.iwdg_key = IWDG_LOCK_REG_CMD;
}

/**
  * @brief Disable interrupt signal.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_IWDG_DisableInterrupt(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->IWDOG_KEY.BIT.iwdg_key = IWDG_UNLOCK_REG_CMD;
    iwdgx->IWDOG_CONTROL.BIT.inten = BASE_CFG_UNSET;
    iwdgx->IWDOG_KEY.BIT.iwdg_key = IWDG_LOCK_REG_CMD;
}

/**
  * @brief Ensable Windows mode.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_IWDG_EnableWindowsMode(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->IWDOG_KEY.BIT.iwdg_key = IWDG_UNLOCK_REG_CMD;
    iwdgx->IWDOG_CONTROL.BIT.window_mode_en = BASE_CFG_SET;
    iwdgx->IWDOG_KEY.BIT.iwdg_key = IWDG_LOCK_REG_CMD;
}

/**
  * @brief Disable Windows mode.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_IWDG_DisableWindowsMode(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    iwdgx->IWDOG_KEY.BIT.iwdg_key = IWDG_UNLOCK_REG_CMD;
    iwdgx->IWDOG_CONTROL.BIT.window_mode_en = BASE_CFG_UNSET;
    iwdgx->IWDOG_KEY.BIT.iwdg_key = IWDG_LOCK_REG_CMD;
}

/**
  * @brief Get Windows mode.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval bool is enable or disable.
  */
static inline bool DCL_IWDG_GetWindowsMode(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    return iwdgx->IWDOG_CONTROL.BIT.window_mode_en;
}

/**
  * @brief Getting value of IWDG RIS register.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval unsigned int Value of IWDG RIS register.
  */
static inline unsigned int DCL_IWDG_GetRIS(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    return iwdgx->IWDOG_RIS.BIT.iwdogris;
}

/**
  * @brief Getting value of IWDG MIS register.
  * @param iwdgx Value of @ref IWDG_RegStruct.
  * @retval unsigned int Value of IWDG MIS register.
  */
static inline unsigned int DCL_IWDG_GetMIS(IWDG_RegStruct *iwdgx)
{
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgx));
    return iwdgx->IWDOG_MIS.BIT.iwdogmis;
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
    float maxSecond = (float)(0xFFFFFFFF / clockFreq); /* 0xFFFFFFFF max input value */
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