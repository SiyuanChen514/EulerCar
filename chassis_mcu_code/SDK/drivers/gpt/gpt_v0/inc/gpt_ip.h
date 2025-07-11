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
  * @file      gpt_ip.h
  * @author    MCU Driver Team
  * @brief     GPT module driver.
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the GPT.
  *                + Register Struct of GPT
  *                + GPT Register Map struct
  *                + Direct Configuration Layer functions of GPT
  */

#ifndef McuMagicTag_GPT_IP_H
#define McuMagicTag_GPT_IP_H

/* Includes-------------------------------------------------------------------*/
#include "baseinc.h"

/**
  * @addtogroup GPT
  * @{
  */

/**
  * @defgroup GPT_IP GPT_IP
  * @brief GPT_IP: gpt_v0
  * @{
  */

/**
 * @defgroup GPT_Param_Def GPT Parameters Definition
 * @brief Definition of GPT configuration parameters.
 * @{
 */

/* Macro definitions ---------------------------------------------------------*/
#ifdef  GPT_PARAM_CHECK
#define GPT_ASSERT_PARAM          BASE_FUNC_ASSERT_PARAM
#define GPT_PARAM_CHECK_NO_RET    BASE_FUNC_PARAMCHECK_NO_RET
#define GPT_PARAM_CHECK_WITH_RET  BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define GPT_ASSERT_PARAM(para)               ((void)0U)
#define GPT_PARAM_CHECK_NO_RET(para)         ((void)0U)
#define GPT_PARAM_CHECK_WITH_RET(para, ret)  ((void)0U)
#endif

#define GPT_PWM_MAX_NUM            0x3FFU         /**< The max num of pwmo_num */
#define GPT_PWM_PERIOD_MIN_VALUE   2U             /**< The min value of pwm_period */
#define GPT_PWM_DUTY_MIN_VALUE     1U             /**< The min value of pwm_duty */
#define GPT_PWM_PERIOD_MAX_VALUE   0x3FFFFFFUL    /**< The max value of pwm_period */
#define GPT_PWM_DUTY_MAX_VALUE     0x3FFFFFFUL    /**< The max value of pwm_duty */
#define GPT_PWM_PERIOD_INVALID_VALUE 0xFFFFFFFF
#define GPT_PWM_DUTY_INVALID_VALUE   0xFFFFFFFF

/**
  * @}
  */

/**
  * @defgroup GPT_Reg_Def GPT Register Definition
  * @brief register mapping structure
  * @{
  */
/**
 * @brief PWM CFG2 register structure
 */
typedef struct {
    unsigned int pwm_num   : 10;  /**< Number of output square waves of the GPTx. */
    unsigned int reserved0 : 22;
} volatile PWM_CFG2_Reg;

/**
 * @brief PWM CTRL register union
 */
typedef union {
    unsigned int  value;
    struct {
        unsigned int  pwm_enable : 1;  /**< GPT enable. */
        unsigned int  pwm_inv    : 1;  /**< Positive and negative phase control of the GPT output. */
        unsigned int  pwm_keep   : 1;  /**< GPT output mode. */
        unsigned int  reserved0  : 29;
    } BIT;
} volatile PWM_CTRL_Reg;

/**
 * @brief PWM STATE2 register union.
 */
typedef union {
    unsigned int  value;
    struct {
        unsigned int  pwm_num_st    : 10; /**< Number of output square waves used by internal modules. */
        unsigned int  pwm_busy      : 1;  /**< Working status of the GPTx module. */
        unsigned int  pwm_keep_st   : 1;  /**< Output square wave mode used by the internal modules. */
        unsigned int  pwm_cnt_st    : 10; /**< Indicates number of remaining output square waves. */
        unsigned int  reserved0     : 10;
    } BIT;
} volatile PWM_STATE2_Reg;

/**
 * @brief GPT register structure.
 */
typedef struct {
    unsigned int     pwm_period;    /**< Number of cycles of PWM. Offset address: 0x00000000U. */
    unsigned int     pwm_duty;      /**< The number of high-level beats of PWM. Offset address: 0x00000004U. */
    PWM_CFG2_Reg     PWM_CFG2;      /**< PWM Config Register 2. Offset address: 0x00000008U.*/
    PWM_CTRL_Reg     PWM_CTRL;      /**< PWM Control Register. Offset address: 0x0000000CU. */
    unsigned int     pwm_period_st; /**< Number of counting cycles in internal module. Offset address: 0x00000010U. */
    unsigned int     pwm_duty_st;   /**< High-level beats used by the internal module. Offset address: 0x00000014U. */
    PWM_STATE2_Reg   PWM_STATE2;    /**< PWM State Register. Offset address: 0x00000018U. */
} volatile GPT_RegStruct;
/**
  * @}
  */

/* Parameter Check -----------------------------------------------------------*/

/**
  * @brief Verify GPT max pwm num
  * @param num    Pwm number, only valid if keep equ 0
  * @retval true
  * @retval false
  */
static inline bool IsGptPwmNum(unsigned int num)
{
    return ((num) <= GPT_PWM_MAX_NUM);
}

/**
  * @brief  Verify GPT period value
  * @param period  Period of GPT
  * @retval true
  * @retval false
  */
static inline bool IsGptPeriod(unsigned int period)
{
    return ((period >= GPT_PWM_PERIOD_MIN_VALUE) && (period <= GPT_PWM_PERIOD_MAX_VALUE));
}

/**
  * @brief Verify GPT duty value
  * @param duty    Duty of GPT
  * @retval true
  * @retval false
  */
static inline bool IsGptDuty(unsigned int duty)
{
    return ((duty >= GPT_PWM_DUTY_MIN_VALUE) && (duty <= GPT_PWM_DUTY_MAX_VALUE));
}

/* Direct Configuration Layer Functions --------------------------------------*/
/**
 * @brief   Set PWM Period.
 * @param   gptx    GPTx register baseAddr.
 * @param   period  Number of cycles of PWM.
 * @retval  None
 */
static inline void DCL_GPT_SetPeriod(GPT_RegStruct * const gptx, unsigned int period)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_PARAM_CHECK_NO_RET(IsGptPeriod(period));
    gptx->pwm_period = period;
}

/**
 * @brief   Get PWM Period.
 * @param   gptx    GPTx register baseAddr.
 * @retval  period  Number of cycles of PWM.
 */
static inline unsigned int DCL_GPT_GetPeriod(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    return gptx->pwm_period;
}

/**
 * @brief   Set PWM duty.
 * @param   gptx   GPTx register baseAddr.
 * @param   duty   The number of high-level beats of PWM.
 * @retval  None
 */
static inline void DCL_GPT_SetDuty(GPT_RegStruct * const gptx, unsigned int duty)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_PARAM_CHECK_NO_RET(IsGptDuty(duty));
    gptx->pwm_duty = duty;
}

/**
 * @brief   Get PWM duty.
 * @param   gptx   gptx register baseAddr.
 * @retval  duty   The number of high-level beats of PWM.
 */
static inline unsigned int DCL_GPT_GetDuty(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    return gptx->pwm_duty;
}

/**
 * @brief   Set PWM number, only valid if pwm_keep is set.
 * @param   gptx   GPTx register baseAddr.
 * @param   pwmNum The number of PWM.
 * @retval  None
 */
static inline void DCL_GPT_SetPwmNum(GPT_RegStruct * const gptx, unsigned int pwmNum)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_PARAM_CHECK_NO_RET(IsGptPwmNum(pwmNum));
    gptx->PWM_CFG2.pwm_num = pwmNum;
}

/**
 * @brief   Get PWM number.
 * @param   gptx   GPTx register baseAddr.
 * @retval  None
 */
static inline unsigned int DCL_GPT_GetPwmNum(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    return gptx->PWM_CFG2.pwm_num;
}

/**
 * @brief   Enable GPT.
 * @param   gptx   GPTx register baseAddr.
 * @retval  None
 */
static inline void DCL_GPT_Enable(GPT_RegStruct * const gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    gptx->PWM_CTRL.BIT.pwm_enable = BASE_CFG_SET;
}

/**
 * @brief   Disable GPT.
 * @param   gptx   GPTx register baseAddr.
 * @retval  None
 */
static inline void DCL_GPT_Disable(GPT_RegStruct * const gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    gptx->PWM_CTRL.BIT.pwm_enable = BASE_CFG_UNSET;
}

/**
 * @brief   Get GPT Enable/Disable status.
 * @param   gptx   GPTx register baseAddr.
 * @retval  None
 */
static inline bool DCL_GPT_GetPwmEnableStatus(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    return gptx->PWM_CTRL.BIT.pwm_enable;
}

/**
 * @brief   Set PWM output polarity.
 * @param   gptx     GPTx register baseAddr.
 * @param   polarity PWM output positive and negative control.
 * @retval  None
 */
static inline void DCL_GPT_SetPolarity(GPT_RegStruct * const gptx, bool polarity)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_PARAM_CHECK_NO_RET((polarity == BASE_CFG_SET) || (polarity == BASE_CFG_UNSET));
    gptx->PWM_CTRL.BIT.pwm_inv = polarity;
}

/**
 * @brief   Get PWM output polarity.
 * @param   gptx     GPTx register baseAddr.
 * @retval  pwm_inv  0(PWM output positive) or 1(PWM output negative).
 */
static inline bool DCL_GPT_GetPolarity(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    return gptx->PWM_CTRL.BIT.pwm_inv;
}

/**
 * @brief   Set PWM Keep.
 * @param   gptx     GPTx register baseAddr.
 * @param   keep     Output square wave mode.
 * @retval  None
 */
static inline void DCL_GPT_Setkeep(GPT_RegStruct * const gptx,  bool keep)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_PARAM_CHECK_NO_RET((keep == BASE_CFG_SET) || (keep == BASE_CFG_UNSET));
    gptx->PWM_CTRL.BIT.pwm_keep = keep;
}

/**
 * @brief   Get PWM Keep.
 * @param   gptx     GPTx register baseAddr.
 * @retval  0(Single output) or 1(Continuous output).
 */
static inline bool DCL_GPT_Getkeep(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    return gptx->PWM_CTRL.BIT.pwm_keep;
}

/**
 * @brief   Get PWM State0 Period.
 * @param   gptx     GPTx register baseAddr.
 * @retval  unsigned int period.
 */
static inline unsigned int DCL_GPT_GetState0Period(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    return gptx->pwm_period_st;
}

/**
  * @brief Get PWM duty in State1.
  * @param gptx   GPTx register baseAddr.
  * @retval unsigned int duty.
  */
static inline unsigned int DCL_GPT_GetState1Duty(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    return gptx->pwm_duty_st;
}

/**
  * @brief Get the number of square waves that the module also needs to output.
  * @param gptx GPTx register baseAddr.
  * @retval unsigned int the number of pwm needs to output.
  */
static inline unsigned int DCL_GPT_GetState2PwmCnt(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    return gptx->PWM_STATE2.BIT.pwm_cnt_st;
}

/**
  * @brief Get the output square wave mode adopted by the internal module of PWM.
  * @param gptx GPTx register baseAddr.
  * @retval mode 0(Single output) or 1(Continuous output).
  */
static inline unsigned int DCL_GPT_GetState2Keep(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    return gptx->PWM_STATE2.BIT.pwm_keep_st;
}

/**
  * @brief Get Working status of PWM module.
  * @param gptx GPTx register baseAddr.
  * @retval status 0(idle) or 1(busy).
  */
static inline unsigned int DCL_GPT_GetState2Busy(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    return gptx->PWM_STATE2.BIT.pwm_busy;
}

/**
  * @brief Get the number of output square waves used by the internal module.
  * @param gptx GPTx register baseAddr.
  * @retval unsigned int the number of period.
  */
static inline unsigned int DCL_GPT_GetState2Period(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    return gptx->PWM_STATE2.BIT.pwm_num_st;
}
/**
  * @}
  */

/**
 * @}
 */
#endif /* McuMagicTag_GPT_IP_H */