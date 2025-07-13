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
  *          + Direct Configuration Layer driver functions.
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

#define PMC_WAKEUP_SRC_MARSK 0x3F

/**
  * @addtogroup PMC
  * @{
  */

/**
  * @defgroup PMC_IP PMC_IP
  * @brief PMC_IP: pmc_v0.
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
  */
typedef enum {
    PMC_WAKEUP_0    = 0x00000000U,
    PMC_WAKEUP_1    = 0x00000001U,
    PMC_WAKEUP_2    = 0x00000002U,
    PMC_WAKEUP_3    = 0x00000003U,
    PMC_WAKEUP_CNT  = 0x00000004U,
} PMC_DeepSleepWakeupSrc;

/**
  * @brief Lowpower type.
  * @details status flag:
  *         + PMC_LP_NONE      -- Non-lowpower mode.
  *         + PMC_LP_DEEPSLEEP -- Deepsleep mode.
  *         + PMC_LP_SHUTDOWN  -- Shutdown mode.
  */
typedef enum {
    PMC_LP_NONE       = 0x00000000U,
    PMC_LP_DEEPSLEEP  = 0x00000001U,
    PMC_LP_SHUTDOWN   = 0x00000002U,
} PMC_LowpowerType;

/**
  * @brief PMC PVD threshold voltage level.
  * @details status flag:
  *         + PMC_PVD_THRED_LEVEL2  -- rising edge 2.38V, falling edge 2.28V.
  *         + PMC_PVD_THRED_LEVEL3  -- rising edge 2.48V, falling edge 2.38V.
  *         + PMC_PVD_THRED_LEVEL4  -- rising edge 2.58V, falling edge 2.48V.
  *         + PMC_PVD_THRED_LEVEL5  -- rising edge 2.68V, falling edge 2.58V.
  *         + PMC_PVD_THRED_LEVEL6  -- rising edge 2.78V, falling edge 2.68V.
  *         + PMC_PVD_THRED_LEVEL7  -- rising edge 2.88V, falling edge 2.78V.
  */
typedef enum {
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
  * @brief Low-power mode control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int shutdown_req   : 1;  /**< Indicates the request for the system to enter the shutdown mode. */
        unsigned int reserved_0     : 3;
        unsigned int deepsleep_req  : 1;  /**< The system enters the deepsleep mode. */
        unsigned int reserved_1     : 27;
    } BIT;
} volatile PMC_LOWPOWER_MODE;

/**
  * @brief Wakeup control register in deepsleep mode.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int wakeup0_act_mode : 2;  /**< Valid mode select of WakeUP0. */
        unsigned int wakeup1_act_mode : 2;  /**< Validity mode select of wakeup1. */
        unsigned int wakeup2_act_mode : 2;  /**< Valid mode select of WakeUP2. */
        unsigned int wakeup3_act_mode : 2;  /**< Validity mode select of wakeup3. */
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
  * @brief Low-power status query register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int wakeup_src_lock       : 6; /**< Starts the wakeup source query. */
        unsigned int reserved_0            : 2;
        unsigned int starup_from_shutdown  : 1; /**< Indicates whether to start from the shutdown state. */
        unsigned int starup_from_deepsleep : 1; /**< Start from the deepsleep state. */
        unsigned int reserved_1            : 2;
        unsigned int wakeup0_status        : 1; /**< Wakeup0 wakeup source status. */
        unsigned int wakeup1_status        : 1; /**< Wakeup1 wakeup source status. */
        unsigned int wakeup2_status        : 1; /**< Wakeup2 wakeup source status. */
        unsigned int wakeup3_status        : 1; /**< wakeup3: wakeup source status. */
        unsigned int reserved_2            : 8;
        unsigned int cldo_po_cnt           : 8; /**< Indicates the cldo power-on time statistics. */
    } BIT;
} volatile PMC_LOWPOWER_STATUS;

/**
  * @brief Core domain POR reset duration configuration register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int core_por_dly : 3;  /**< Power-on reset duration of the core domain(ms). */
        unsigned int reserved_0 : 29;
    } BIT;
} volatile PMC_CORE_POR_CTRL;

/**
  * @brief PVD control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pvd_en       : 1;  /**< PVD enable. */
        unsigned int reserved_0   : 3;
        unsigned int pvd_lv       : 3;  /**< PVD voltage threshold. */
        unsigned int reserved_1   : 24;
        unsigned int pvd_int      : 1;  /**< PVD interrupt status. */
    } BIT;
} volatile PMC_PVD_CTRL;

/**
  * @brief PMU CLDO voltage control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved_0      : 16;
        unsigned int pmu_cldo_trim   : 4;    /**< PMU CLDO calibration value. */
        unsigned int reserved_1      : 12;
    } BIT;
} volatile PMC_PMU_CLDO;

/**
  * @brief PMC registers definition structure.
  */
typedef struct {
    unsigned int        reserved_0[128];
    PMC_LOWPOWER_MODE   LOWPOWER_MODE;   /**< Low-power mode control register. Offset address: 0x00000200U. */
    unsigned int        CNT32K_WAKE_CYC; /**< Timed wakeup period config reg. Offset address: 0x00000204U. */
    PMC_WAKEUP_CTRL     WAKEUP_CTRL;     /**< Wakeup control register in deepsleep mode. Offset address: 0x00000208U. */
    PMC_LOWPOWER_STATUS LOWPOWER_STATUS; /**< Low-power status query register. Offset address: 0x0000020CU. */
    PMC_CORE_POR_CTRL   CORE_POR_CTRL;   /**< Core domain POR reset duration config reg. Offset address: 0x00000210U. */
    unsigned int        reserved_1[507];
    PMC_PVD_CTRL        PVD_CTRL;        /**< PVD control register. Offset address: 0x00000A00U. */
    unsigned int        reserved_2[2];
    PMC_PMU_CLDO        PMU_CLDO;        /**< PMU CLDO voltage control register. Offset address: 0x00000A0CU. */
    unsigned int        reserved_3[316];
    unsigned int        AON_USER_REG0;   /**< AON domain user register 0. Offset address: 0x00000F00U. */
    unsigned int        AON_USER_REG1;   /**< AON domain user register 1. Offset address: 0x00000F04U. */
    unsigned int        AON_USER_REG2;   /**< AON domain user register 2. Offset address: 0x00000F08U. */
    unsigned int        AON_USER_REG3;   /**< AON domain user register 3. Offset address: 0x00000F0CU. */
} volatile PMC_RegStruct;

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
static inline void DCL_PMC_EnterDeepSleep(PMC_RegStruct * const pmcx)
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
  * @brief Enter shutdown mode interface.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_EnterShutDown(PMC_RegStruct * const pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->LOWPOWER_MODE.BIT.shutdown_req = BASE_CFG_ENABLE;
}

/**
  * @brief Quit shutdown mode interface.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_QuitShutDown(PMC_RegStruct * const pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->LOWPOWER_MODE.BIT.shutdown_req = BASE_CFG_DISABLE;
}

/**
  * @brief Setting wakeup timer cycle.
  * @param pmcx PMC register base address.
  * @param cycle Timer cycle value.
  * @retval None.
  */
static inline void DCL_PMC_SetFixTimeWakeupTimer(PMC_RegStruct * const pmcx, unsigned int cycle)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->CNT32K_WAKE_CYC = cycle;
}

/**
  * @brief Enable wakeup from timer.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_FixTimeWakeupEnable(PMC_RegStruct * const pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.cnt32k_wakeup_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable wakeup from timer.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_FixTimeWakeupDisable(PMC_RegStruct * const pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.cnt32k_wakeup_en = BASE_CFG_DISABLE;
}

/**
  * @brief Enable wakeup from WAKEUP0.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_Wakeup0Enable(PMC_RegStruct * const pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.wakeup0_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable wakeup from WAKEUP0.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_Wakeup0Disable(PMC_RegStruct * const pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.wakeup0_en = BASE_CFG_DISABLE;
}

/**
  * @brief Enable wakeup from WAKEUP1.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_Wakeup1Enable(PMC_RegStruct * const pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.wakeup1_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable wakeup from WAKEUP1.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_Wakeup1Disable(PMC_RegStruct * const pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.wakeup1_en = BASE_CFG_DISABLE;
}

/**
  * @brief Enable wakeup from WAKEUP2.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_Wakeup2Enable(PMC_RegStruct * const pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.wakeup2_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable wakeup from WAKEUP2.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_Wakeup2Disable(PMC_RegStruct * const pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.wakeup2_en = BASE_CFG_DISABLE;
}

/**
  * @brief Enable wakeup from WAKEUP3.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_Wakeup3Enable(PMC_RegStruct * const pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.wakeup3_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable wakeup from WAKEUP3.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_Wakeup3Disable(PMC_RegStruct * const pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->WAKEUP_CTRL.BIT.wakeup3_en = BASE_CFG_DISABLE;
}

/**
  * @brief Setting WAKEUP0 active level mode.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_SetWakeup0ActiveMode(PMC_RegStruct * const pmcx, PMC_ActMode mode)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    PMC_PARAM_CHECK_NO_RET(mode <= PMC_WAKEUP_ACT_LOW_LEVEL);
    pmcx->WAKEUP_CTRL.BIT.wakeup0_act_mode = ((unsigned int)mode & 0x3);
}

/**
  * @brief Setting WAKEUP1 active level mode.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_SetWakeup1ActiveMode(PMC_RegStruct * const pmcx, PMC_ActMode mode)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    PMC_PARAM_CHECK_NO_RET(mode <= PMC_WAKEUP_ACT_LOW_LEVEL);
    pmcx->WAKEUP_CTRL.BIT.wakeup1_act_mode = ((unsigned int)mode & 0x3);
}

/**
  * @brief Setting WAKEUP2 active level mode.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_SetWakeup2ActiveMode(PMC_RegStruct * const pmcx, PMC_ActMode mode)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    PMC_PARAM_CHECK_NO_RET(mode <= PMC_WAKEUP_ACT_LOW_LEVEL);
    pmcx->WAKEUP_CTRL.BIT.wakeup2_act_mode = ((unsigned int)mode & 0x3);
}

/**
  * @brief Setting WAKEUP3 active level mode.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_SetWakeup3ActiveMode(PMC_RegStruct * const pmcx, PMC_ActMode mode)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    PMC_PARAM_CHECK_NO_RET(mode <= PMC_WAKEUP_ACT_LOW_LEVEL);
    pmcx->WAKEUP_CTRL.BIT.wakeup3_act_mode = ((unsigned int)mode & 0x3);
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
  * @brief Getting flag of wakeup from shutdown mode.
  * @param pmcx PMC register base address.
  * @retval flag of wakeup from shutdown mode.
  */
static inline bool DCL_PMC_GetStartupFromShutDownFlag(const PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    return (pmcx->LOWPOWER_STATUS.BIT.starup_from_shutdown);
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
  * @brief Getting CLDO power-on time.
  * @param pmcx PMC register base address.
  * @retval time of CLDO power-on.
  */
static inline unsigned int DCL_PMC_GetCldoPowerOnTime(const PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    return (pmcx->LOWPOWER_STATUS.BIT.cldo_po_cnt);
}

/**
  * @brief Setting always on user's register 0.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_SetAlwaysOnUserReg0(PMC_RegStruct * const pmcx, unsigned int value)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->AON_USER_REG0 = value;
}

/**
  * @brief Getting always on user's register 0.
  * @param pmcx PMC register base address.
  * @retval Register0's value.
  */
static inline unsigned int DCL_PMC_GetAlwaysOnUserReg0(const PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    return (pmcx->AON_USER_REG0);
}

/**
  * @brief Setting always on user's register 1.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_SetAlwaysOnUserReg1(PMC_RegStruct * const pmcx, unsigned int value)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->AON_USER_REG1 = value;
}

/**
  * @brief Getting always on user's register 1.
  * @param pmcx PMC register base address.
  * @retval Register1's value.
  */
static inline unsigned int DCL_PMC_GetAlwaysOnUserReg1(const PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    return (pmcx->AON_USER_REG1);
}

/**
  * @brief Setting always on user's register 2.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_SetAlwaysOnUserReg2(PMC_RegStruct * const pmcx, unsigned int value)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->AON_USER_REG2 = value;
}

/**
  * @brief Getting always on user's register 2.
  * @param pmcx PMC register base address.
  * @retval Register2's value.
  */
static inline unsigned int DCL_PMC_GetAlwaysOnUserReg2(const PMC_RegStruct *pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    return (pmcx->AON_USER_REG2);
}

/**
  * @brief Setting always on user's register 3.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_SetAlwaysOnUserReg3(PMC_RegStruct * const pmcx, unsigned int value)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->AON_USER_REG3 = value;
}

/**
  * @brief Getting always on user's register 3.
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
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_EnablePvd(PMC_RegStruct * const pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->PVD_CTRL.BIT.pvd_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable PVD function.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_DisablePvd(PMC_RegStruct * const pmcx)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->PVD_CTRL.BIT.pvd_en = BASE_CFG_DISABLE;
}

/**
  * @brief Set PVD threshold function.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_SetPvdThreshold(PMC_RegStruct * const pmcx, PMC_PvdThreshold threshold)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->PVD_CTRL.BIT.pvd_lv = threshold;
}

/**
  * @brief Set Core poweron reset delay value, delay by ms.
  * @param pmcx PMC register base address.
  * @retval None.
  */
static inline void DCL_PMC_SetCorePorDelay(PMC_RegStruct * const pmcx, unsigned char delayValue)
{
    PMC_ASSERT_PARAM(IsPMCInstance(pmcx));
    pmcx->CORE_POR_CTRL.BIT.core_por_dly = (delayValue & 0x7); /* 0x7: delay value valid mask */
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