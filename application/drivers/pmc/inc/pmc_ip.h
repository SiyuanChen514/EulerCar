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
  * @file    pmc_ip.h
  * @author  MCU Driver Team
  * @brief   Header file containing PMC module DCL driver functions.
  *          This file provides functions to manage the following functionalities of PMC module.
  *          + Definition of PMC configuration parameters.
  *          + PMC registers mapping structures.
  *          + Direct Configutration Layer driver functions.
  */
#ifndef McuMagicTag_PMC_IP_H
#define McuMagicTag_PMC_IP_H

#include "baseinc.h"

#ifdef PMC_PARAM_CHECK
#define PMC_ASSERT_PARAM BASE_FUNC_ASSERT_PARAM
#define PMC_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define PMC_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define PMC_ASSERT_PARAM(para) ((void)0U)
#define PMC_PARAM_CHECK_NO_RET(para) ((void)0U)
#define PMC_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif

#define PMC_WAKEUP_SRC_MARSK      0x3F
#define PMC_WAKEUP_ACT_MODE_MARSK 0x3

/**
  * @addtogroup PMC
  * @{
  */

/**
  * @defgroup PMC_IP PMC_IP
  * @brief PMC_IP: pmc_v1.
  * @{
  */

/**
 * @defgroup PMC_Param_Def PMC Parameters Definition
 * @brief Definition of PMC configuration parameters
 * @{
 */


/**
  * @brief wakeup pin level mode of PMC module.
  * @details status flag:
  *         + PMC_WAKEUP_ACT_UP_EDGE -- Wakeup valid in up edge
  *         + PMC_WAKEUP_ACT_DOWN_EDGE -- Wakeup valid in down edge
  *         + PMC_WAKEUP_ACT_HIGH_LEVEL -- Wakeup valid in high edge
  *         + PMC_WAKEUP_ACT_LOW_LEVEL -- Wakeup valid in low edge
  */
typedef enum {
    PMC_WAKEUP_ACT_UP_EDGE      = 0x00000000U,
    PMC_WAKEUP_ACT_DOWN_EDGE    = 0x00000001U,
    PMC_WAKEUP_ACT_HIGH_LEVEL   = 0x00000002U,
    PMC_WAKEUP_ACT_LOW_LEVEL    = 0x00000003U,
} PMC_ActMode;

/**
  * @brief Wakeup source of deep sleep.
  * @details status flag:
  *         + PMC_WAKEUP_0   -- Wakeup from DS_WAKEUP0.
  *         + PMC_WAKEUP_1   -- Wakeup from DS_WAKEUP1.
  *         + PMC_WAKEUP_2   -- Wakeup from DS_WAKEUP2.
  *         + PMC_WAKEUP_3   -- Wakeup from DS_WAKEUP3.
  *         + PMC_WAKEUP_CNT -- Wakeup from timer.
  *         + PMC_WAKEUP_NONE --No Wakeup source.
  */
typedef enum {
    PMC_WAKEUP_0    = 0x00000000U,
    PMC_WAKEUP_1    = 0x00000001U,
    PMC_WAKEUP_2    = 0x00000002U,
    PMC_WAKEUP_3    = 0x00000003U,
    PMC_WAKEUP_CNT  = 0x00000004U,
    PMC_WAKEUP_NONE  = 0x00000005U,
} PMC_LowpowerWakeupSrc;

/**
  * @brief Callback Triggering Event Enumeration Definition
  */
typedef enum {
    PMC_PVD_INT_ID         = 0x00,
} PMC_CallBackID;

/**
  * @brief Lowpower type.
  * @details status flag:
  *         + PMC_LP_NONE      -- Non-lowpower mode.
  *         + PMC_LP_DEEPSLEEP -- Deepsleep mode.
  */
typedef enum {
    PMC_LP_NONE       = 0x00000000U,
    PMC_LP_DEEPSLEEP  = 0x00000001U,
} PMC_LowpowerType;

static unsigned int g_internalPvdValueTable[8][2] = {
    {0x00, 0x00}, /* rising edge 2.18V, falling edge 2.08V. */
    {0x01, 0x01}, /* rising edge 2.28V, falling edge 2.18V. */
    {0x02, 0x02}, /* rising edge 2.38V, falling edge 2.28V. */
    {0x03, 0x03}, /* rising edge 2.48V, falling edge 2.38V. */
    {0x04, 0x04}, /* rising edge 2.58V, falling edge 2.48V. */
    {0x05, 0x05}, /* rising edge 2.68V, falling edge 2.58V. */
    {0x06, 0x06}, /* rising edge 2.78V, falling edge 2.68V. */
    {0x07, 0x07}, /* rising edge 2.88V, falling edge 2.78V. */
};

/**
  * @brief PMC PVD threshold voltage level.
  * @details PMC_PVD_THRED_LEVEL, For details, see g_pvdValueTable.
  */
typedef enum {
    PMC_PVD_THRED_LEVEL0    = 0x00000000U,
    PMC_PVD_THRED_LEVEL1    = 0x00000001U,
    PMC_PVD_THRED_LEVEL2    = 0x00000002U,
    PMC_PVD_THRED_LEVEL3    = 0x00000003U,
    PMC_PVD_THRED_LEVEL4    = 0x00000004U,
    PMC_PVD_THRED_LEVEL5    = 0x00000005U,
    PMC_PVD_THRED_LEVEL6    = 0x00000006U,
    PMC_PVD_THRED_LEVEL7    = 0x00000007U,
} PMC_PvdThreshold;

/**
  * @brief PMC extend handle, configuring some special parameters.
  */
typedef struct {
} PMC_ExtendHandle;

/**
  * @brief User-defined callback function.
  */
typedef struct {
    /** Event callback function of the flash module */
    void (*PmcCallBack)(void *handle);
} PMC_UserCallBack;

/**
  * @}
  */

/**
  * @defgroup PMC_REG_Definition PMC Register Structure.
  * @brief PMC Register Structure Definition.
  * @{
  */

/**
  * @brief Low-power mode control registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved_0     : 4;
        unsigned int deepsleep_req  : 1;  /**< The system enters the deepsleep mode. */
        unsigned int reserved_1     : 27;
    } BIT;
} volatile PMC_LOWPOWER_MODE;

/**
  * @brief Wakeup control in deepsleep mode registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int wakeup0_act_mode : 2;  /**< Valid mode select of WakeUP0. */
        unsigned int wakeup1_act_mode : 2;  /**< Valid mode select of wakeup1. */
        unsigned int wakeup2_act_mode : 2;  /**< Valid mode select of wakeup2. */
        unsigned int wakeup3_act_mode : 2;  /**< Valid mode select of wakeup3. */
        unsigned int wakeup0_en       : 1;  /**< Wakeup0 enable. */
        unsigned int wakeup1_en       : 1;  /**< Wakeup1 enable. */
        unsigned int wakeup2_en       : 1;  /**< Wakeup2 enable. */
        unsigned int wakeup3_en       : 1;  /**< Wakeup3 enable. */
        unsigned int reserved_0       : 4;
        unsigned int cnt32k_wakeup_en : 1;  /**< Scheduled wakeup enable. */
        unsigned int reserved_1       : 15;
    } BIT;
} volatile PMC_WAKEUP_CTRL;

/**
  * @brief Low-power status query registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int wakeup_src_lock       : 6; /**< Starts the wakeup source query. */
        unsigned int reserved_0            : 3;
        unsigned int starup_from_deepsleep : 1; /**< Start from the deepsleep state. */
        unsigned int reserved_1            : 2;
        unsigned int wakeup0_status        : 1; /**< Wakeup0 wakeup source status. */
        unsigned int wakeup1_status        : 1; /**< Wakeup1 wakeup source status. */
        unsigned int wakeup2_status        : 1; /**< Wakeup2 wakeup source status. */
        unsigned int wakeup3_status        : 1; /**< Wakeup3 wakeup source status. */
        unsigned int reserved_2            : 16;
    } BIT;
} volatile PMC_LOWPOWER_STATUS;

/**
  * @brief PMC registers definition structure.
  */
typedef struct {
    unsigned int        reserved_0[128];
    PMC_LOWPOWER_MODE   LOWPOWER_MODE;   /**< Low-power mode control register. Offset address: 0x200. */
    unsigned int        CNT32K_WAKE_CYC; /**< Timed wakeup period config reg. Offset address: 0x204. */
    PMC_WAKEUP_CTRL     WAKEUP_CTRL;     /**< Wakeup control register in deepsleep mode.
                                              Offset address: 0x208. */
    PMC_LOWPOWER_STATUS LOWPOWER_STATUS; /**< Low-power status query register. Offset address: 0x20C. */
    unsigned int        reserved_1[828];
    unsigned int        AON_USER_REG0;   /**< AON domain user register 0. Offset address: 0xF00. */
    unsigned int        AON_USER_REG1;   /**< AON domain user register 1. Offset address: 0xF04. */
    unsigned int        AON_USER_REG2;   /**< AON domain user register 2. Offset address: 0xF08. */
    unsigned int        AON_USER_REG3;   /**< AON domain user register 3. Offset address: 0xF0C. */
} volatile PMC_RegStruct;

/**
  * @brief Check PVD threshold voltage level.
  * @param value value of losc rtrim value.
  * @retval true
  * @retval false
  */
static inline bool IsPvdThreshold(PMC_PvdThreshold value)
{
    return (value == PMC_PVD_THRED_LEVEL0 || value == PMC_PVD_THRED_LEVEL1 || \
            value == PMC_PVD_THRED_LEVEL2 || value == PMC_PVD_THRED_LEVEL3 || \
            value == PMC_PVD_THRED_LEVEL4 || value == PMC_PVD_THRED_LEVEL5 || \
            value == PMC_PVD_THRED_LEVEL6 || value == PMC_PVD_THRED_LEVEL7);
}

/**
  * @brief Check PMC Wakeup source.
  * @param wakeSrc value of Wakeup source.
  * @retval true
  * @retval false
  */
static inline bool IsWakeupSrc(PMC_LowpowerWakeupSrc wakeSrc)
{
    return (wakeSrc == PMC_WAKEUP_0 || wakeSrc == PMC_WAKEUP_1 || \
            wakeSrc == PMC_WAKEUP_2 || wakeSrc == PMC_WAKEUP_3 || \
            wakeSrc == PMC_WAKEUP_CNT);
}

/**
  * @brief Check PMC active mode.
  * @param mode value of active mode.
  * @retval true
  * @retval false
  */
static inline bool IsActiveMode(PMC_ActMode mode)
{
    return (mode == PMC_WAKEUP_ACT_UP_EDGE || mode == PMC_WAKEUP_ACT_DOWN_EDGE || \
            mode == PMC_WAKEUP_ACT_HIGH_LEVEL || mode == PMC_WAKEUP_ACT_LOW_LEVEL);
}

/**
  * @brief Enter sleep mode interface.
  * @param None.
  * @retval None.
  */
static inline void DCL_PMC_EnterSleep(void)
{
#if defined(USER_MODE_ENABLE) && (USER_MODE_ENABLE == 1)
    /* If user mode is supported, make sure to execute WFI
       commands in machine mode */
    static unsigned int priv = RISCV_U_MODE;
    RISCV_PRIV_MODE_SWITCH(priv);
    __asm("wfi");
    RISCV_PRIV_MODE_SWITCH(priv);
#else
   /* Only machine mode, no need for mode switching */
    __asm("wfi");
#endif
}

/**
  * @brief Enter deepsleep mode interface.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_EnterDeepSleep(PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->LOWPOWER_MODE.BIT.deepsleep_req = BASE_CFG_ENABLE;
}

/**
  * @brief Quit deepsleep mode interface.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_QuitDeepSleep(PMC_RegStruct * const pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->LOWPOWER_MODE.BIT.deepsleep_req = BASE_CFG_DISABLE;
}
/**
  * @brief Setting wakeup timer cycle.
  * @param pmcx PMC register base address.
  * @param cycle Timer cycle value.
  * @retval None.
  */
static inline void DCL_PMC_SetFixTimeWakeupTimer(PMC_RegStruct *pmcx, unsigned int cycle)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->CNT32K_WAKE_CYC = cycle;
}

/**
  * @brief Enable wakeup from timer.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_FixTimeWakeupEnable(PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.cnt32k_wakeup_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable wakeup from timer.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_FixTimeWakeupDisable(PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.cnt32k_wakeup_en = BASE_CFG_DISABLE;
}

/**
  * @brief Enable wakeup from WAKEUP0.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_Wakeup0Enable(PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.wakeup0_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable wakeup from WAKEUP0.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_Wakeup0Disable(PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.wakeup0_en = BASE_CFG_DISABLE;
}

/**
  * @brief Enable wakeup from WAKEUP1.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_Wakeup1Enable(PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.wakeup1_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable wakeup from WAKEUP1.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_Wakeup1Disable(PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.wakeup1_en = BASE_CFG_DISABLE;
}

/**
  * @brief Enable wakeup from WAKEUP2.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_Wakeup2Enable(PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.wakeup2_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable wakeup from WAKEUP2.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_Wakeup2Disable(PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.wakeup2_en = BASE_CFG_DISABLE;
}

/**
  * @brief Enable wakeup from WAKEUP3.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_Wakeup3Enable(PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.wakeup3_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable wakeup from WAKEUP3.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_Wakeup3Disable(PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.wakeup3_en = BASE_CFG_DISABLE;
}

/**
  * @brief Setting WAKEUP0 active level mode.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_SetWakeup0ActiveMode(PMC_RegStruct *pmcx, PMC_ActMode mode)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    PMC_PARAM_CHECK_NO_RET(IsActiveMode(mode));
    pmcx->WAKEUP_CTRL.BIT.wakeup0_act_mode = ((unsigned int)mode & PMC_WAKEUP_ACT_MODE_MARSK);
}

/**
  * @brief Setting WAKEUP1 active level mode.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_SetWakeup1ActiveMode(PMC_RegStruct *pmcx, PMC_ActMode mode)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    PMC_PARAM_CHECK_NO_RET(IsActiveMode(mode));
    pmcx->WAKEUP_CTRL.BIT.wakeup1_act_mode = ((unsigned int)mode & PMC_WAKEUP_ACT_MODE_MARSK);
}

/**
  * @brief Setting WAKEUP2 active level mode.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_SetWakeup2ActiveMode(PMC_RegStruct *pmcx, PMC_ActMode mode)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    PMC_PARAM_CHECK_NO_RET(IsActiveMode(mode));
    pmcx->WAKEUP_CTRL.BIT.wakeup2_act_mode = ((unsigned int)mode & PMC_WAKEUP_ACT_MODE_MARSK);
}

/**
  * @brief Setting WAKEUP3 active level mode.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_SetWakeup3ActiveMode(PMC_RegStruct *pmcx, PMC_ActMode mode)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    PMC_PARAM_CHECK_NO_RET(IsActiveMode(mode));
    pmcx->WAKEUP_CTRL.BIT.wakeup3_act_mode = ((unsigned int)mode & PMC_WAKEUP_ACT_MODE_MARSK);
}

/**
  * @brief Getting WAKEUP0 status.
  * @param pmcx PMC register base address.
  * @retval Wakeup status.
  */
static inline bool DCL_PMC_GetWakeup0Status(const PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    return (pmcx->LOWPOWER_STATUS.BIT.wakeup0_status);
}

/**
  * @brief Getting WAKEUP1 status.
  * @param pmcx PMC register base address.
  * @retval Wakeup status.
  */
static inline bool DCL_PMC_GetWakeup1Status(const PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    return (pmcx->LOWPOWER_STATUS.BIT.wakeup1_status);
}

/**
  * @brief Getting WAKEUP2 status.
  * @param pmcx PMC register base address.
  * @retval Wakeup status.
  */
static inline bool DCL_PMC_GetWakeup2Status(const PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    return (pmcx->LOWPOWER_STATUS.BIT.wakeup2_status);
}

/**
  * @brief Getting WAKEUP3 status.
  * @param pmcx PMC register base address.
  * @retval Wakeup status.
  */
static inline bool DCL_PMC_GetWakeup3Status(const PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    return (pmcx->LOWPOWER_STATUS.BIT.wakeup3_status);
}

/**
  * @brief Getting flag of wakeup from deepsleep mode.
  * @param pmcx PMC register base address.
  * @retval flag of wakeup from deepsleep mode.
  */
static inline bool DCL_PMC_GetStartupFromDeepSleepFlag(const PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    return (pmcx->LOWPOWER_STATUS.BIT.starup_from_deepsleep);
}

/**
  * @brief Getting wakeup source.
  * @param pmcx PMC register base address.
  * @retval source of wakeup.
  */
static inline unsigned int DCL_PMC_GetWakeupSrc(const PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    return (pmcx->LOWPOWER_STATUS.BIT.wakeup_src_lock & PMC_WAKEUP_SRC_MARSK);
}

/**
  * @brief Setting always on user's regsiter 0.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_SetAlwaysOnUserReg0(PMC_RegStruct *pmcx, unsigned int value)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->AON_USER_REG0 = value;
}

/**
  * @brief Getting always on user's regsiter 0.
  * @param pmcx PMC register base address.
  * @retval Register0's value.
  */
static inline unsigned int DCL_PMC_GetAlwaysOnUserReg0(const PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    return (pmcx->AON_USER_REG0);
}

/**
  * @brief Setting always on user's regsiter 1.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_SetAlwaysOnUserReg1(PMC_RegStruct *pmcx, unsigned int value)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->AON_USER_REG1 = value;
}

/**
  * @brief Getting always on user's regsiter 1.
  * @param pmcx PMC register base address.
  * @retval Register1's value.
  */
static inline unsigned int DCL_PMC_GetAlwaysOnUserReg1(const PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    return (pmcx->AON_USER_REG1);
}

/**
  * @brief Setting always on user's regsiter 2.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_SetAlwaysOnUserReg2(PMC_RegStruct *pmcx, unsigned int value)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->AON_USER_REG2 = value;
}

/**
  * @brief Getting always on user's regsiter 2.
  * @param pmcx PMC register base address.
  * @retval Register2's value.
  */
static inline unsigned int DCL_PMC_GetAlwaysOnUserReg2(const PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    return (pmcx->AON_USER_REG2);
}

/**
  * @brief Setting always on user's regsiter 3.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_SetAlwaysOnUserReg3(PMC_RegStruct *pmcx, unsigned int value)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->AON_USER_REG3 = value;
}

/**
  * @brief Getting always on user's regsiter 3.
  * @param pmcx PMC register base address.
  * @retval Register3's value.
  */
static inline unsigned int DCL_PMC_GetAlwaysOnUserReg3(const PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    return (pmcx->AON_USER_REG3);
}

/**
  * @brief Enable PVD function.
  * @retval None.
  */
static inline void DCL_PMC_EnablePvd(void)
{
    SYSCTRL1_RegStruct *sysCtrl1x = SYSCTRL1_BASE;
    sysCtrl1x->PVD_CFG.BIT.pvd_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable PVD function.
  * @retval None.
  */
static inline void DCL_PMC_DisablePvd(void)
{
    SYSCTRL1_RegStruct *sysCtrl1x = SYSCTRL1_BASE;
    sysCtrl1x->PVD_CFG.BIT.pvd_en = BASE_CFG_DISABLE;
}

/**
  * @brief Set PVD threshold.
  * @param threshold PMC PVD threshold voltage level.
  * @retval None.
  */
static inline void DCL_PMC_SetPvdThreshold(PMC_PvdThreshold threshold)
{
    PMC_ASSERT_PARAM(IsPvdThreshold(threshold));
    SYSCTRL1_RegStruct *sysCtrl1x = SYSCTRL1_BASE;
    sysCtrl1x->PVD_CFG.BIT.pvd_fall_thd = g_internalPvdValueTable[threshold][0];
    sysCtrl1x->PVD_CFG.BIT.pvd_rise_thd = g_internalPvdValueTable[threshold][1];
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* McuMagicTag_PMC_IP_H */