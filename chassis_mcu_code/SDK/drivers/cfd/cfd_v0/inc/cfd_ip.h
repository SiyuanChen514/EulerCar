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
  * @file      cfd_ip.h
  * @author    MCU Driver Team
  * @brief     CFD module driver.
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the CFD.
  *             + Register Struct of CFD
  *             + CFD Register Map struct
  *             + Direct Configuration Layer functions of CFD
  */

#ifndef McuMagicTag_CFD_IP_H
#define McuMagicTag_CFD_IP_H

/* Includes ------------------------------------------------------------------ */
#include "baseinc.h"
/* Macro definitions ---------------------------------------------------------*/
#ifdef CFD_PARAM_CHECK
    #define CFD_ASSERT_PARAM         BASE_FUNC_ASSERT_PARAM
    #define CFD_PARAM_CHECK_NO_RET   BASE_FUNC_PARAMCHECK_NO_RET
    #define CFD_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
    #define CFD_ASSERT_PARAM(para)                ((void)0U)
    #define CFD_PARAM_CHECK_NO_RET(para)          ((void)0U)
    #define CFD_PARAM_CHECK_WITH_RET(param, ret)  ((void)0U)
#endif
/**
  * @addtogroup CFD
  * @{
  */

/**
  * @defgroup CFD_IP
  * @{
  */

/**
  * @defgroup CFD_Param_Def CFD Parameters Definition
  * @brief Description of CFD configuration parameters.
  * @{
  */

/* Typedef definitions ------------------------------------------------------- */
/**
  * @brief The CFD module interrupt type mask.
  */
typedef enum {
    CFD_INT_CHECK_END_MASK = 0x00000001U,
    CFD_INT_PLL_REF_CLOCK_STOP_MASK = 0x00000002U,
    CFD_INT_MAX_MASK
} CFD_Interrupt_Type;

/**
  * @}
  */

/**
  * @defgroup CFD_Reg_Def CFD Register Definition
  * @brief Description CFD register mapping structure.
  * @{
  */

/**
  * @brief CFD version registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int month_day : 16;            /**< Month and day. */
        unsigned int year : 8;                  /**< Year. */
        unsigned int release_substep : 1;       /**< Version information. */
        unsigned int release_step : 1;          /**< Version information. */
        unsigned int release_ver : 1;           /**< Version information. */
        unsigned int reserved0 : 5;
    } BIT;
} volatile CFDVER_Reg;

/**
  * @brief CFD control registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfden : 1;                   /**< CFD enable or disable. */
        unsigned int reserved0 : 31;
    } BIT;
} volatile CFDCTRL_Reg;

/**
  * @brief CFD check window upper bound registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfdwdoh : 8;                /**< CFD check window upper bound value. */
        unsigned int reserved0 : 24;
    } BIT;
} volatile CFDWDOH_Reg;

/**
  * @brief CFD count locked value registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfdcnt_lock : 8;             /**< CFD count locked value */
        unsigned int reserved0 : 24;
    } BIT;
} volatile CFDCNTLOCK_Reg;

/**
  * @brief CFD interrupt enable registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int chk_end_en : 1;              /**< CFD check end interrupt enable. */
        unsigned int clk_fail_en : 1;             /**< CFD clock failure interrupt enable. */
        unsigned int reserved0 : 30;
    } BIT;
} volatile CFDINTENA_Reg;

/**
  * @brief CFD interrupt status registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int chk_end_int : 1;             /**< CFD check end interrupt status. */
        unsigned int clk_fail_int : 1;            /**< CFD clock failure interrupt status. */
        unsigned int reserved0 : 30;
    } BIT;
} volatile CFDINTSTS_Reg;

/**
  * @brief CFD initial interrupt registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int chk_end_raw : 1;             /**< CFD check end initial interrupt. */
        unsigned int clk_fail_raw : 1;            /**< CFD clock failure initial interrupt. */
        unsigned int reserved0 : 30;
    } BIT;
} volatile CFDINTRAW_Reg;

/**
  * @brief CFD interrupt injection registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int chk_end_inj : 1;             /**< CFD check end interrupt injection. */
        unsigned int clk_fail_inj : 1;            /**< CFD clock failure interrupt injection. */
        unsigned int reserved0 : 30;
    } BIT;
} volatile CFDINTINJ_Reg;

/**
  * @brief CFD interrupt callback functions.
  *
  */
typedef struct {
    void (*PllClockStopCallback)(void *handle); /**< Pll clock stop callback function. */
    void (*CheckEndCallback)(void *handle); /**< End of each check callback function. */
} CFD_UserCallBack;

/**
  * @brief CFD extend handle.
  */
typedef struct _CFD_ExtendeHandle {
} CFD_ExtendHandle;


/**
  * @brief CFD register mapping structure.
  */
typedef struct {
    CFDVER_Reg CFDVER;         /**< CFD version register, offset address: 0x0000. */
    CFDCTRL_Reg CFDCTRL;       /**< CFD control register, offset address: 0x0004. */
    CFDWDOH_Reg CFDWDOH;       /**< CFD check window upper bound register, offset address: 0x0008. */
    CFDCNTLOCK_Reg CFDCNTLOCK; /**< CFD count locked value register, offset address: 0x000C. */
    CFDINTENA_Reg CFDINTENA;   /**< CFD interrupt enable register, offset address: 0x0010. */
    CFDINTSTS_Reg CFDINTSTS;   /**< CFD interrupt status register, offset address: 0x0014. */
    CFDINTRAW_Reg CFDINTRAW;   /**< CFD initial interrupt register, offset address: 0x0018. */
    CFDINTINJ_Reg CFDINTINJ;   /**< CFD interrupt injection register, offset address: 0x001C. */
} volatile CFD_RegStruct;

/**
  * @}
  */

/**
  * @brief Enable CFD module.
  * @param cfdx CFD register base address.
  * @retval None.
  */
static inline void DCL_CFD_Enable(CFD_RegStruct *cfdx)
{
    CFD_ASSERT_PARAM(IsCFDInstance(cfdx));
    cfdx->CFDCTRL.BIT.cfden = BASE_CFG_ENABLE;
}

/**
  * @brief Disable CFD module.
  * @param cfdx CFD register base address.
  * @retval None.
  */
static inline void DCL_CFD_Disable(CFD_RegStruct *cfdx)
{
    CFD_ASSERT_PARAM(IsCFDInstance(cfdx));
    cfdx->CFDCTRL.BIT.cfden = BASE_CFG_DISABLE;
}

/**
  * @brief Sets the upper boundary of the detection window.
  * @param cfdx CFD register base address.
  * @param value The value of the upper bound.
  * @retval None.
  */
static inline void DCL_CFD_SetWindowUpperBound(CFD_RegStruct *cfdx, unsigned char value)
{
    CFD_ASSERT_PARAM(IsCFDInstance(cfdx));
    cfdx->CFDWDOH.BIT.cfdwdoh = value;
}

/**
  * @brief Gets the upper boundary of the detection window.
  * @param cfdx CFD register base address.
  * @retval The value of the upper bound.
  */
static inline unsigned char DCL_CFD_GetWindowUpperBound(CFD_RegStruct *cfdx)
{
    CFD_ASSERT_PARAM(IsCFDInstance(cfdx));
    return cfdx->CFDWDOH.BIT.cfdwdoh;
}

/**
  * @brief Internal counter count latch value.
  * @param cfdx CFD register base address.
  * @retval unsigned char. latch value.
  */
static inline unsigned char DCL_CFD_GetCntValue(CFD_RegStruct *cfdx)
{
    CFD_ASSERT_PARAM(IsCFDInstance(cfdx));
    return cfdx->CFDCNTLOCK.BIT.cfdcnt_lock;
}

/**
  * @brief Enables the specified type of interrupt.
  * @param cfdx CFD register base address.
  * @param type Mask of the interrupt type.
  * @retval None.
  */
static inline void DCL_CFD_EnableInterrupt(CFD_RegStruct *cfdx, CFD_Interrupt_Type type)
{
    CFD_ASSERT_PARAM(IsCFDInstance(cfdx));
    CFD_PARAM_CHECK_NO_RET(type == CFD_INT_CHECK_END_MASK || \
                           type == CFD_INT_PLL_REF_CLOCK_STOP_MASK);
    cfdx->CFDINTENA.reg |= type;
}

/**
  * @brief Disables the specified type of interrupt.
  * @param cfdx CFD register base address.
  * @param type Mask of the interrupt type.
  * @retval None.
  */
static inline void DCL_CFD_DisableInterrupt(CFD_RegStruct *cfdx, CFD_Interrupt_Type type)
{
    CFD_ASSERT_PARAM(IsCFDInstance(cfdx));
    CFD_PARAM_CHECK_NO_RET(type == CFD_INT_CHECK_END_MASK || \
                           type == CFD_INT_PLL_REF_CLOCK_STOP_MASK);
    cfdx->CFDINTENA.reg &= (~type);
}

/**
  * @brief Check whether the specified interrupt is triggered.
  * @param cfdx CFD register base address.
  * @param type Mask of the interrupt type.
  * @retval bool.
  */
static inline bool DCL_CFD_GetInterruptStatus(CFD_RegStruct *cfdx, CFD_Interrupt_Type type)
{
    CFD_ASSERT_PARAM(IsCFDInstance(cfdx));
    CFD_PARAM_CHECK_WITH_RET(type == CFD_INT_CHECK_END_MASK || \
                             type == CFD_INT_PLL_REF_CLOCK_STOP_MASK, false);
    return (cfdx->CFDINTSTS.reg & type) == 0 ? false : true;
}

/**
  * @brief Clears interrupts of the specified type.
  * @param cfdx CFD register base address.
  * @param type Mask of the interrupt type.
  * @retval None.
  */
static inline void DCL_CFD_ClearInterrupt(CFD_RegStruct *cfdx, CFD_Interrupt_Type type)
{
    CFD_ASSERT_PARAM(IsCFDInstance(cfdx));
    CFD_PARAM_CHECK_NO_RET(type == CFD_INT_CHECK_END_MASK || \
                           type == CFD_INT_PLL_REF_CLOCK_STOP_MASK);
    cfdx->CFDINTRAW.reg |= type;
}

/**
  * @brief Injects interrupts of the specified type.
  * @param cfdx CFD register base address.
  * @param type Mask of the interrupt type.
  * @retval None.
  */
static inline void DCL_CFD_EnableInterruptInject(CFD_RegStruct *cfdx, CFD_Interrupt_Type type)
{
    CFD_ASSERT_PARAM(IsCFDInstance(cfdx));
    CFD_PARAM_CHECK_NO_RET(type == CFD_INT_CHECK_END_MASK || \
                           type == CFD_INT_PLL_REF_CLOCK_STOP_MASK);
    cfdx->CFDINTINJ.reg |= type;
}

/**
  * @brief Stop injecting interrupts of a specified type.
  * @param cfdx CFD register base address.
  * @param type Mask of the interrupt type.
  * @retval None.
  */
static inline void DCL_CFD_DisableInterruptInject(CFD_RegStruct *cfdx, CFD_Interrupt_Type type)
{
    CFD_ASSERT_PARAM(IsCFDInstance(cfdx));
    CFD_PARAM_CHECK_NO_RET(type == CFD_INT_CHECK_END_MASK || \
                           type == CFD_INT_PLL_REF_CLOCK_STOP_MASK);
    cfdx->CFDINTINJ.reg &= (~type);
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* McuMagicTag_CFD_IP_H */