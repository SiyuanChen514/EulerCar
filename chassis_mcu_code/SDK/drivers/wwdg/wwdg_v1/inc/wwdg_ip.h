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
  * @file      wwdg_ip.h
  * @author    MCU Driver Team
  * @brief     WWDG module driver
  * @details   The header file contains the following declaration:
  *             + WWDG configuration enums.
  *             + WWDG register structures.
  *             + WWDG DCL Functions.
  *             + Parameters check functions.
  */

#ifndef McuMagicTag_WWDG_IP_H
#define McuMagicTag_WWDG_IP_H

/* Includes ------------------------------------------------------------------*/
#include "baseinc.h"

/* Macro definition */

#ifdef WWDG_PARAM_CHECK
    #define WWDG_ASSERT_PARAM         BASE_FUNC_ASSERT_PARAM
    #define WWDG_PARAM_CHECK_NO_RET   BASE_FUNC_PARAMCHECK_NO_RET
    #define WWDG_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
    #define WWDG_ASSERT_PARAM(para)                ((void)0U)
    #define WWDG_PARAM_CHECK_NO_RET(para)          ((void)0U)
    #define WWDG_PARAM_CHECK_WITH_RET(para, ret)  ((void)0U)
#endif

/**
  * @addtogroup WWDG
  * @{
  */

/**
  * @defgroup WWDG_IP WWDG_IP
  * @brief WWDG_IP: wwdg_v1.
  * @{
  */

/**
  * @defgroup WWDG_Param_Def WWDG Parameters Definition
  * @brief Description of WWDG configuration parameters.
  * @{
  */
/* MACRO definitions -------------------------------------------------------*/
#define FREQ_CONVERT_MS_UNIT  1000
#define FREQ_CONVERT_US_UNIT  1000000
#define WWDG_UNLOCK_REG_CMD    0x55 /* 0x55 CMD: key equal 0x55 will unlock all reg write function */
#define WWDG_LOCK_REG_CMD      0xFF /* 0xFF CMD: key not equal 0x55 will lock reg write function except key reg */

/* Typedef definitions -------------------------------------------------------*/
typedef enum {
    WWDG_TIME_UNIT_TICK = 0x00000000U,
    WWDG_TIME_UNIT_S = 0x00000001U,
    WWDG_TIME_UNIT_MS = 0x00000002U,
    WWDG_TIME_UNIT_US = 0x00000003U
} WWDG_TimeType;

typedef enum {
    WWDG_FREQ_DIV_NONE = 0x00000000U,
    WWDG_FREQ_DIV_2 = 0x00000001U,
    WWDG_FREQ_DIV_4 = 0x00000002U,
    WWDG_FREQ_DIV_8 = 0x00000003U,
    WWDG_FREQ_DIV_16 = 0x00000004U,
    WWDG_FREQ_DIV_32 = 0x00000005U,
    WWDG_FREQ_DIV_64 = 0x00000006U,
    WWDG_FREQ_DIV_128 = 0x00000007U,
    WWDG_FREQ_DIV_256 = 0x00000008U,
    WWDG_FREQ_DIV_512 = 0x00000009U,
    WWDG_FREQ_DIV_1024 = 0x0000000AU,
    WWDG_FREQ_DIV_2048 = 0x0000000BU,
    WWDG_FREQ_DIV_4096 = 0x0000000CU,
    WWDG_FREQ_DIV_8192 = 0x0000000DU,
    WWDG_FREQ_DIV_MAX
} WWDG_FreqDivType;

/**
  * @brief WWDG extend handle.
  */
typedef struct _WWDG_ExtendHandle {
} WWDG_ExtendHandle;

/**
  * @brief WWDG user callback.
  */
typedef struct {
    void (* CallbackFunc)(void *handle);  /**< WWDG callback Function */
} WWDG_UserCallBack;
/**
  * @}
  */

/**
  * @defgroup WWDG_Reg_Def WWDG Register Definition
  * @brief Description WWDG register mapping structure.
  * @{
  */
/**
  * @brief WWDG load init value.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int wwdg_load : 16;    /**< init value. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile WWDG_LOAD_REG;

/**
  * @brief WWDG get current value.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int wwdg_value : 16;    /**< current value. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile WWDG_VALUE_REG;

/**
  * @brief WWDG set window value.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int wwdg_window : 16;    /**< window value. */
        unsigned int reserved0 : 16;
    } BIT;
} volatile WWDG_WINDOW_REG;

/**
  * @brief WWDG cmd function value.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int wwdg_key : 8;    /**< cmd function value. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile WWDG_KEY_REG;

/**
  * @brief WWDG clk pre div value.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int wwdg_pre_div : 4;    /**< clk pre div value. */
        unsigned int reserved0 : 28;
    } BIT;
} volatile WWDG_PRE_DIV_REG;

/**
  * @brief WWDG enable interrupt and reset.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int inten : 1;    /**< enable interrupt. */
        unsigned int resen : 1;    /**< enable reset. */
        unsigned int window_mode_en : 1;    /**< enable window mode. */
        unsigned int reserved0 : 29;
    } BIT;
} volatile WWDG_CONTROL_REG;

/**
  * @brief WWDG orignal interrupt signal.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int wwdogris : 1;    /**< original interrupt status. */
        unsigned int reserve : 31;
    } BIT;
} volatile WWDG_RIS_REG;

/**
  * @brief mask interrupt signal.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int wwdogmis : 1;    /**< maske interrupt status. */
        unsigned int reserve : 31;
    } BIT;
} volatile WWDG_MIS_REG;

/**
  * @brief WWDG Register Structure definition.
  */
typedef struct {
    WWDG_LOAD_REG     WWDOG_LOAD;       /**< WWDG load value register. */
    WWDG_VALUE_REG    WWDOG_VALUE;      /**< WWDG current value register. */
    WWDG_WINDOW_REG   WWDOG_WINDOW;     /**< WWDG Window value register. */
    WWDG_KEY_REG      WWDOG_KEY;        /**< WWDG instruction word register. */
    WWDG_PRE_DIV_REG  WWDOG_PRE_DIV;    /**< WWDG prescale register. */
    WWDG_CONTROL_REG  WWDOG_CONTROL;    /**< WWDG interrupt, reset and window enable register. */
    WWDG_RIS_REG      WWDOG_RIS;        /**< WWDG orignal interrupt register. */
    WWDG_MIS_REG      WWDOG_MIS;        /**< WWDG mask interrupt register. */
} volatile WWDG_RegStruct;

/**
  * @}
  */

/**
  * @brief Setting the load value of the WWDG counter.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @param  loadValue Load value of the WWDG counter.
  * @retval None.
  */
static inline void DCL_WWDG_SetLoadValue(WWDG_RegStruct *wwdgx, unsigned short loadValue)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    wwdgx->WWDOG_KEY.BIT.wwdg_key = WWDG_UNLOCK_REG_CMD;
    wwdgx->WWDOG_LOAD.BIT.wwdg_load = loadValue;
    wwdgx->WWDOG_KEY.BIT.wwdg_key = WWDG_LOCK_REG_CMD;
}

/**
  * @brief Getting the load value of the WWDG load register.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @retval unsigned short WWDG load value.
  */
static inline unsigned short DCL_WWDG_GetLoadValue(const WWDG_RegStruct *wwdgx)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    return wwdgx->WWDOG_LOAD.BIT.wwdg_load;
}

/**
  * @brief Getting the value of the WWDG counter register.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @retval unsigned short WWDG counter value.
  */
static inline unsigned short DCL_WWDG_GetCounterValue(const WWDG_RegStruct *wwdgx)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    return wwdgx->WWDOG_VALUE.BIT.wwdg_value;
}

/**
  * @brief Setting window value.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @param windowValue window value of the WWDG counter.
  * @retval None.
  */
static inline void DCL_WWDG_SetWindowValue(WWDG_RegStruct *wwdgx, unsigned short windowValue)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    wwdgx->WWDOG_KEY.BIT.wwdg_key = WWDG_UNLOCK_REG_CMD;
    wwdgx->WWDOG_WINDOW.BIT.wwdg_window = windowValue;
    wwdgx->WWDOG_KEY.BIT.wwdg_key = WWDG_LOCK_REG_CMD;
}

/**
  * @brief Getting window value, windowValue need bigger than 4.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @param windowValue window value of the WWDG counter.
  * @retval unsigned short wwdg window value.
  */
static inline unsigned short DCL_WWDG_GetWindowValue(WWDG_RegStruct *wwdgx)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    return wwdgx->WWDOG_WINDOW.BIT.wwdg_window;
}

/**
  * @brief Start wwdg function.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_WWDG_Start(WWDG_RegStruct *wwdgx)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    wwdgx->WWDOG_KEY.BIT.wwdg_key = 0xCC; /* 0xCC CMD: start wwdg function */
}

/**
  * @brief Stop wwdg function.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_WWDG_Stop(WWDG_RegStruct *wwdgx)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    wwdgx->WWDOG_KEY.BIT.wwdg_key = 0xDD; /* 0xDD CMD: stop wwdg function */
}

/**
  * @brief Clear interrupt and reload watchdog counter value.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_WWDG_Refresh(WWDG_RegStruct *wwdgx)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    wwdgx->WWDOG_KEY.BIT.wwdg_key = 0xAA; /* 0xAA CMD: clear interrupt and reload value */
}

/**
  * @brief Disable write and read WWDG registers except WWDG_LOCK.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_WWDG_LockReg(WWDG_RegStruct *wwdgx)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    wwdgx->WWDOG_KEY.BIT.wwdg_key = 0xFF; /* 0xFF CMD: key not equal 0x55 will lock reg write function except key reg */
}

/**
  * @brief Enable write and read WWDG registers.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_WWDG_UnlockReg(WWDG_RegStruct *wwdgx)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    wwdgx->WWDOG_KEY.BIT.wwdg_key = 0x55; /* 0x55 CMD: key equal 0x55 will unlock all reg write function */
}

/**
  * @brief Setting freq div value, value need litter than 13.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @param  freqDiv freqDiv value of the WWDG counter.
  * @retval None.
  */
static inline void DCL_WWDG_SetFreqDivValue(WWDG_RegStruct *wwdgx, WWDG_FreqDivType freqDiv)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    WWDG_PARAM_CHECK_NO_RET(freqDiv < WWDG_FREQ_DIV_MAX);
    wwdgx->WWDOG_KEY.BIT.wwdg_key = WWDG_UNLOCK_REG_CMD;
    wwdgx->WWDOG_PRE_DIV.BIT.wwdg_pre_div = freqDiv; /* freqDiv parameters set */
    wwdgx->WWDOG_KEY.BIT.wwdg_key = WWDG_UNLOCK_REG_CMD;
}

/**
  * @brief Getting freq div value, value need litter than 13.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @param  freqDiv freqDiv value of the WWDG counter.
  * @retval None.
  */
static inline unsigned char DCL_WWDG_GetFreqDivValue(WWDG_RegStruct *wwdgx)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    return wwdgx->WWDOG_PRE_DIV.BIT.wwdg_pre_div;
}

/**
  * @brief Enable reset signal.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_WWDG_EnableReset(WWDG_RegStruct *wwdgx)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    wwdgx->WWDOG_KEY.BIT.wwdg_key = WWDG_UNLOCK_REG_CMD;
    wwdgx->WWDOG_CONTROL.BIT.resen = BASE_CFG_SET;
    wwdgx->WWDOG_KEY.BIT.wwdg_key = WWDG_LOCK_REG_CMD;
}

/**
  * @brief Disable reset signal.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_WWDG_DisableReset(WWDG_RegStruct *wwdgx)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    wwdgx->WWDOG_KEY.BIT.wwdg_key = WWDG_UNLOCK_REG_CMD;
    wwdgx->WWDOG_CONTROL.BIT.resen = BASE_CFG_UNSET;
    wwdgx->WWDOG_KEY.BIT.wwdg_key = WWDG_LOCK_REG_CMD;
}

/**
  * @brief Start watchdog and enable interrupt signal.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_WWDG_EnableInterrupt(WWDG_RegStruct *wwdgx)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    wwdgx->WWDOG_KEY.BIT.wwdg_key = WWDG_UNLOCK_REG_CMD;
    wwdgx->WWDOG_CONTROL.BIT.inten = BASE_CFG_SET;
    wwdgx->WWDOG_KEY.BIT.wwdg_key = WWDG_LOCK_REG_CMD;
}

/**
  * @brief Disable interrupt signal.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_WWDG_DisableInterrupt(WWDG_RegStruct *wwdgx)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    wwdgx->WWDOG_KEY.BIT.wwdg_key = WWDG_UNLOCK_REG_CMD;
    wwdgx->WWDOG_CONTROL.BIT.inten = BASE_CFG_UNSET;
    wwdgx->WWDOG_KEY.BIT.wwdg_key = WWDG_LOCK_REG_CMD;
}

/**
  * @brief Ensable Windows mode.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_WWDG_EnableWindowsMode(WWDG_RegStruct *wwdgx)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    wwdgx->WWDOG_KEY.BIT.wwdg_key = WWDG_UNLOCK_REG_CMD;
    wwdgx->WWDOG_CONTROL.BIT.window_mode_en = BASE_CFG_SET;
    wwdgx->WWDOG_KEY.BIT.wwdg_key = WWDG_LOCK_REG_CMD;
}

/**
  * @brief Disable Windows mode.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @retval None.
  */
static inline void DCL_WWDG_DisableWindowsMode(WWDG_RegStruct *wwdgx)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    wwdgx->WWDOG_KEY.BIT.wwdg_key = WWDG_UNLOCK_REG_CMD;
    wwdgx->WWDOG_CONTROL.BIT.window_mode_en = BASE_CFG_UNSET;
    wwdgx->WWDOG_KEY.BIT.wwdg_key = WWDG_LOCK_REG_CMD;
}

/**
  * @brief Get Windows mode.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @retval bool is enable or disable.
  */
static inline bool DCL_WWDG_GetWindowsMode(WWDG_RegStruct *wwdgx)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    return wwdgx->WWDOG_CONTROL.BIT.window_mode_en;
}

/**
  * @brief Getting value of WWDG RIS register.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @retval unsigned int Value of WWDG RIS register.
  */
static inline unsigned int DCL_WWDG_GetRIS(WWDG_RegStruct *wwdgx)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    return wwdgx->WWDOG_RIS.BIT.wwdogris;
}

/**
  * @brief Getting value of WWDG MIS register.
  * @param wwdgx Value of @ref WWDG_RegStruct.
  * @retval unsigned int Value of WWDG MIS register.
  */
static inline unsigned int DCL_WWDG_GetMIS(WWDG_RegStruct *wwdgx)
{
    WWDG_ASSERT_PARAM(IsWWDGInstance(wwdgx));
    return wwdgx->WWDOG_MIS.BIT.wwdogmis;
}

/**
  * @brief check wwdg time type parameter.
  * @param timeType Value of @ref WWDG_TimeType.
  * @retval Bool.
  */
static inline bool IsWwdgTimeType(WWDG_TimeType timeType)
{
    return (timeType == WWDG_TIME_UNIT_TICK ||
            timeType == WWDG_TIME_UNIT_S ||
            timeType == WWDG_TIME_UNIT_MS ||
            timeType == WWDG_TIME_UNIT_US);
}

/**
  * @brief check wdg time value parameter.
  * @param baseAddress Value of @ref WDG_RegStruct
  * @param timeValue time value
  * @param timeType Value of @ref WDG_TimeType.
  * @retval Bool.
  */
static inline bool IsWwdgTimeValue(WWDG_RegStruct *baseAddress, float timeValue, WWDG_TimeType timeType)
{
    float clockFreq = (float)HAL_CRG_GetIpFreq((void *)baseAddress);
    float maxSecond = (float)(0xFFFFFFFF / clockFreq); /* 0xFFFFFFFF  max input value */
    return ((timeType == WWDG_TIME_UNIT_TICK && timeValue <= 0xFFFFFFFF) ||
            (timeType == WWDG_TIME_UNIT_S && maxSecond >= timeValue) ||
            (timeType == WWDG_TIME_UNIT_MS && maxSecond >= timeValue / FREQ_CONVERT_MS_UNIT) ||
            (timeType == WWDG_TIME_UNIT_US && maxSecond >= timeValue / FREQ_CONVERT_US_UNIT));
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* McuMagicTag_WWDG_IP_H */