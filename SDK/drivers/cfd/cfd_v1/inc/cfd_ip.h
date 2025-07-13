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
#include "cmm_ip.h"
#include "crg_ip.h"
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
    CFD_INT_CHECK_END_MASK = 0x00000002U,
    CFD_INT_PLL_REF_CLOCK_STOP_MASK = 0x00000008U,
    CFD_INT_MAX_MASK
} CFD_Interrupt_Type;

/**
  * @}
  */

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
typedef CMM_RegStruct CFD_RegStruct;

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
    cfdx->CMCTRL.BIT.cfen = BASE_CFG_ENABLE;
    cfdx->CMCTRL.BIT.cmen = BASE_CFG_ENABLE;
}

/**
  * @brief Disable CFD module.
  * @param cfdx CFD register base address.
  * @retval None.
  */
static inline void DCL_CFD_Disable(CFD_RegStruct *cfdx)
{
    CFD_ASSERT_PARAM(IsCFDInstance(cfdx));
    cfdx->CMCTRL.BIT.cfen = BASE_CFG_DISABLE;
    cfdx->CMCTRL.BIT.cmen = BASE_CFG_DISABLE;
}

/**
  * @brief Sets the target and reference clock source of the CFD.
  * @param cfdx CFD register base address.
  * @retval None.
  */
static inline void DCL_CFD_SetCfdClock(CFD_RegStruct *cfdx)
{
    CFD_ASSERT_PARAM(IsCFDInstance(cfdx));
    if (DCL_CRG_GetPllRefClkSel(CRG) == CRG_PLL_REF_CLK_SELECT_HOSC) {
        cfdx->CMTGTCTRL.BIT.tgtsel = CMM_TARGET_CLK_HOSC;
    } else {
        cfdx->CMTGTCTRL.BIT.tgtsel = CMM_TARGET_CLK_TCXO;
    }
    cfdx->CMTGTCTRL.BIT.tgtscale = CMM_TARGET_FREQ_DIV_8192;  /* target clock frequence 8192 division. */
    cfdx->CMREFCTRL.BIT.refsel = CMM_REF_CLK_LOSC;
    cfdx->CMREFCTRL.BIT.refdiv = CMM_REF_FREQ_DIV_0;
}

/**
  * @brief Sets the upper boundary of the detection window.
  * @param cfdx CFD register base address.
  * @param value The value of the upper bound.
  * @retval None.
  */
static inline void DCL_CFD_SetWindowUpperBound(CFD_RegStruct *cfdx, unsigned int value)
{
    CFD_ASSERT_PARAM(IsCFDInstance(cfdx));
    CMWDOH_Reg cfdwdoh;
    cfdwdoh.reg = cfdx->CMWDOH.reg;  /* Retain the cmwdoh original value. */
    cfdwdoh.BIT.cfdwdoh = value;
    cfdx->CMWDOH.reg = cfdwdoh.reg;
}

/**
  * @brief Gets the upper boundary of the detection window.
  * @param cfdx CFD register base address.
  * @retval The value of the upper bound.
  */
static inline unsigned int DCL_CFD_GetWindowUpperBound(CFD_RegStruct *cfdx)
{
    CFD_ASSERT_PARAM(IsCFDInstance(cfdx));
    CMWDOH_Reg cfdwdoh;
    cfdwdoh.reg = cfdx->CMWDOH.reg;
    return cfdwdoh.BIT.cfdwdoh;
}

/**
  * @brief Internal counter count latch value.
  * @param cfdx CFD register base address.
  * @retval unsigned int. latch value.
  */
static inline unsigned int DCL_CFD_GetCntValue(CFD_RegStruct *cfdx)
{
    CFD_ASSERT_PARAM(IsCFDInstance(cfdx));
    return cfdx->CMCNTLOCK.BIT.cmcnt_lock;
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
    cfdx->CMINTENA.reg |= type;
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
    cfdx->CMINTENA.reg &= (~type);
}

/**
  * @brief Get CFD interrupt type.
  * @param cfdx CFD register base address.
  * @retval unsigned int.
  */
static inline unsigned int DCL_CFD_GetInterruptType(CFD_RegStruct *cfdx)
{
    CFD_ASSERT_PARAM(IsCFDInstance(cfdx));
    return cfdx->CMINTENA.reg;
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
    return (cfdx->CMINTSTS.reg & type) == 0 ? false : true;
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
    cfdx->CMINTRAW.reg |= type;
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
    cfdx->CMINTINJ.reg |= type;
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
    cfdx->CMINTINJ.reg &= (~type);
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* McuMagicTag_CFD_IP_H */