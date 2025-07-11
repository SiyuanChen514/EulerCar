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

#define GPT_PWM_MAX_NUM              0x000003FFU
#define GPT_PWM_PERIOD_MIN_VALUE     0x00000002U
#define GPT_PWM_PERIOD_MAX_VALUE     0x0000FFFFUL
#define GPT_PWM_DIV_FACTOR_MAX_VALUE 0x00000FFFUL
#define GPT_TC_PRD_MAX_VALUE         0x0000FFFFUL
#define GPT_DIV_FACTOR_MAX_VALUE     0x00000FFFUL

/**
  * @addtogroup GPT
  * @{
  */
/**
  * @defgroup GPT_IP GPT_IP
  * @brief GPT_IP: gpt_v1.
  * @{
  */

/**
  * @defgroup GPT_Param_Def GPT Parameters Definition
  * @brief Description of GPT configuration parameters.
  * @{
  */

/**
 * @brief GPT common enable setting.
 */
typedef enum {
    GPT_SET_DISABLE      = 0x00000000U,
    GPT_SET_ENABLE       = 0x00000001U,
} GPT_SetOption;

/**
 * @brief Trigger DMA request option.
 * @details  DMA request type:
 *          + GPT_PWM0_TRIGGER_DMA        -- pwm output finish triggle
 *          + GPT_PERIOD_TRIGGER_DMA      -- gpt period triggle
 *          + GPT_PWM0_PERIOD_TRIGGER_DMA -- pwm output finish and gpt period triggle
 */
typedef enum {
    GPT_PWM0_TRIGGER_DMA             = 0x00000001U,
    GPT_PERIOD_TRIGGER_DMA           = 0x00000002U,
    GPT_PWM0_PERIOD_TRIGGER_DMA      = 0x00000003U,
} GPT_TriggerDMAType;

/**
 * @brief Trigger ADC request option.
 * @details  ADC request type:
 *          + GPT_PWM0_TRIGGER_ADC        -- pwm output finish triggle
 *          + GPT_PERIOD_TRIGGER_ADC      -- gpt period triggle
 *          + GPT_PWM0_PERIOD_TRIGGER_ADC -- pwm output finish and gpt period triggle
 */
typedef enum  {
    GPT_PWM0_TRIGGER_ADC             = 0x00000001U,
    GPT_PERIOD_TRIGGER_ADC           = 0x00000002U,
    GPT_PWM0_PERIOD_TRIGGER_ADC      = 0x00000003U,
} GPT_TriggerADCType;

/**
 * @brief GPT cache loading status.
 * @details  Loading status:
 *          + GPT_PERIOD_LOAD_STATUS     -- Status of the count period register buffer.
 *          + GPT_REFERA0_LOAD_STATUS    -- Status of the counter reference value A0 register buffer
 *          + GPT_REFERB0_LOAD_STATUS    -- Status of the counter reference value B0 register buffer
 *          + GPT_ACT0_LOAD_STATUS       -- Status of the channel action configuration register buffer
 *          + GPT_PWM0_CFG_LOAD_STATUS   -- Status of the configuration register buffer for channel.
 */
typedef enum {
    GPT_PERIOD_LOAD_STATUS           = 0x00000001U,
    GPT_REFERA0_LOAD_STATUS          = 0x00000002U,
    GPT_REFERB0_LOAD_STATUS          = 0x00000004U,
    GPT_ACT0_LOAD_STATUS             = 0x00000100U,
    GPT_PWM0_CFG_LOAD_STATUS         = 0x00001000U,
} GPT_LoadStatus;

/**
 * @brief GPT count mode.
 */
typedef enum {
    GPT_COUNT_UP            = 0x00000000U,
    GPT_COUNT_DOWN          = 0x00000001U
} GPT_CountMode;

/**
 * @brief PWM output action for referent dot.
 * @details Output action:
 *          + GPT_ACTION_NO_ACTION     -- Prohibit action.
 *          + GPT_ACTION_OUTPUT_LOW    -- Low level.
 *          + GPT_ACTION_OUTPUT_HIGH   -- High level.
 *          + GPT_ACTION_OUTPUT_FLIP   -- Flip the level.
 */
typedef enum {
    GPT_ACTION_NO_ACTION    =  0x00000000U,
    GPT_ACTION_OUTPUT_LOW   =  0x00000001U,
    GPT_ACTION_OUTPUT_HIGH  =  0x00000002U,
    GPT_ACTION_OUTPUT_FLIP  =  0x00000003U
} GPT_ActionType;

/**
 * @brief GPT PWM output reference dot and action.
 */
typedef struct {
    unsigned int       refdot;
    GPT_ActionType    refAction;
}GPT_RefValueAction;

/**
  * @brief GPT reference dot and action config.
  */
typedef struct {
    GPT_RefValueAction  refA0;
    GPT_RefValueAction  refB0;
} GPT_ReferCfg;

/**
 * @brief GPT user interrupt callback function type.
 * @details Function type:
 *          + GPT_INT_PWM_OUTPUT_FIN  --  PWM output finish.
 *          + GPT_INT_PERIOD          --  PWM period output finish.
 */
typedef enum {
    GPT_INT_PWM_OUTPUT_FIN           = 0x00000001,
    GPT_INT_PERIOD                   = 0x00000002
}GPT_CallBackFunType;

/**
 * @brief GPT user interrupt callback function.
 */
typedef struct {
    void (* PWMOutPutFin)(void *handle);    /**< GPT PWM channel out finish callback function for users */
    void (* PWMPeriod)(void *handle);       /**< GPT PWM period output finish callback function for users */
} GPT_UserCallBack;

/**
 * @brief GPT extend configure
 */
typedef struct {
    bool periodIntEnable;           /**< PWM period output finish interrupt. */
    bool outputFinIntEnable;        /**< PWM channel output finish interrupt. */
} GPT_ExtendHandle;

/**
  * @}
  */

/**
  * @defgroup GPT_Reg_Def GPT Register Definition
  * @brief register mapping structure
  * @{
  */

/**
 * @brief GPT Version structure
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    sub_version  : 4;    /**< Subversion number.  */
        unsigned int    main_version : 4;    /**< Major version number.  */
        unsigned int    reserved     : 24;
    } BIT;
} volatile GPT_VER_INFO_REG;

/**
 * @brief Frequency division coefficient register.
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    rg_div_fac : 12;  /**< Frequency division coefficient.
                                               Frequency divider = Frequency division coefficient + 1. */
        unsigned int    reserved   : 20;
    } BIT;
} volatile GPT_TC_DIV_REG;

/**
 * @brief Count period register.
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    rg_cnt_prd : 16;    /**< Counting period of the counter. */
        unsigned int    reserved   : 16;
    } BIT;
} volatile GPT_TC_PRD_REG;

/**
 * @brief Count reference value A0 register.
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    rg_cnt_refa0 : 16;  /**< Count reference value A0, Less than or equal to count period. */
        unsigned int    reserved     : 16;
    } BIT;
} volatile GPT_TC_REFA0_REG;

/**
 * @brief Count reference value B0 register.
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    rg_cnt_refb0 : 16;  /**< Count reference value B0, Less than or equal to count period. */
        unsigned int    reserved     : 16;
    } BIT;
} volatile GPT_TC_REFB0_REG;

/**
 * @brief Count status register.
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    ro_cnt_val : 16;    /**< Current count value of the counterr. */
        unsigned int    ro_div_cnt : 12;    /**< Current count value of the divide. */
        unsigned int    reserved   : 4;
    } BIT;
} volatile GPT_TC_STS_REG;

/**
 * @brief Channel action configuration register.
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    rg_pg_act0_refa0 : 2;   /**< When counter is equal to the reference value A0,
                                                    PWM output of Channel acts. */
        unsigned int    reserved         : 2;
        unsigned int    rg_pg_act0_refb0 : 2;   /**< When counter is equal to the reference value A0,
                                                     PWM output of Channel acts. */
        unsigned int    reserved1        : 26;
    } BIT;
} volatile GPT_PG_ACT0_REG;

/**
 * @brief Interrupt enable register.
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    rg_prd_int_en  : 1;   /**< PWM period output finish interrupt enable. */
        unsigned int    reserved       : 3;
        unsigned int    rg_pwm0_int_en : 1;   /**< PWM output finish interrupt enable. */
        unsigned int    reserved1      : 27;
    } BIT;
} volatile GPT_INT_EN_REG;

/**
 * @brief Interrupt flag register.
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    ro_prd_int_flag  : 1;  /**< Interrupt flag of periodic interrupt. */
        unsigned int    reserved         : 3;
        unsigned int    ro_pwm0_int_flag : 1;  /**< Interrupt flag of output completion interrupt of channel. */
        unsigned int    reserved1        : 11;
        unsigned int    rg_prd_int_clr   : 1;  /**< Periodic interrupt clear. Writing 1 clears the bit. */
        unsigned int    reserved2        : 3;
        unsigned int    rg_pwm0_int_clr  : 1;  /**< Channel output finish interrupt clear. Writing 1 clears the bit. */
        unsigned int    reserved3        : 11;
    } BIT;
} volatile GPT_INT_FLAG_REG;

/**
 * @brief Interrupt injection register.
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    rg_prd_int_inj  : 1; /**< Software injection period finish interrupt. Writing 1 clears */
        unsigned int    reserved        : 3;
        unsigned int    rg_pwm0_int_inj : 1; /**< Software injection output finish interrupt. Writing 1 clears */
        unsigned int    reserved1       : 27;
    } BIT;
} volatile GPT_INT_INJ_REG;

/**
 * @brief SOC/DMA request enable register.
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    rg_soc_prd_en  : 1; /**< Enable for generating the ADC signal at end of counting period. */
        unsigned int    rg_soc_pwm0_en : 1; /**< Enable for channel output completion to generate ADC signal. */
        unsigned int    reserved       : 2;
        unsigned int    rg_dsr_prd_en  : 1; /**< DMA single request signal at end of the counting period. */
        unsigned int    rg_dsr_pwm0_en : 1; /**< DMA single request signal after the output of channel is complete. */
        unsigned int    reserved1      : 2;
        unsigned int    rg_dbr_prd_en  : 1; /**< DMA burst request signal at end of the counting period. */
        unsigned int    rg_dbr_pwm0_en : 1; /**< DMA burst request signal after output of channel is complete. */
        unsigned int    reserved2      : 22;
    } BIT;
} volatile GPT_SOCDR_EN_REG;

/**
 * @brief Channel configuration register.
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    rg_pwm0_num  : 10; /**< Number of PWM output by channel. */
        unsigned int    reserved     : 21;
        unsigned int    rg_pwm0_keep : 1;  /**< PWM output mode of channel. */
    } BIT;
} volatile GPT_PWM0_CFG_REG;

/**
 * @brief GPT enable register.
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    rg_gpt_en : 1;   /**< GPT enable control. 0: The GPT channel is disabled.
                                                                  1: The GPT channel is enabled. */
        unsigned int    reserved  : 31;
    } BIT;
} volatile GPT_EN_REG;

/**
 * @brief Channel status register.
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    ro_pwm0_num_sta : 10; /**< Number of output PWMs of channel. */
        unsigned int    reserved        : 21;
        unsigned int    ro_pwm0_run_sta : 1;  /**< Output status of channel. */
    } BIT;
} volatile GPT_PWM0_STA_REG;

/**
 * @brief Buffer loading enable register.
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    rg_buf_load_en : 1; /**< Buffer loading enable for registers that support buffer function. */
        unsigned int    reserved       : 31;
    } BIT;
} volatile GPT_BUF_LOAD_EN_REG;

/**
 * @brief Buffer loading status register.
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    tc_prd_ld_sts   : 1;  /**< Status of the count cycle register buffer. */
        unsigned int    tc_refa0_ld_sts : 1;  /**< Status of the counter reference value A0 register buffer. */
        unsigned int    tc_refb0_ld_sts : 1;  /**< Status of the count reference value B0 register buffer. */
        unsigned int    reserved        : 5;
        unsigned int    pg_act0_ld_sts  : 1;  /**< Status of the channel action configuration register buffer. */
        unsigned int    reserved1       : 3;
        unsigned int    pwm0_cfg_ld_sts : 1;  /**< Status of the channel configuration register buffer. */
        unsigned int    reserved2       : 19;
    } BIT;
} volatile GPT_LOAD_STS_REG;

/**
  * @brief Register mapping structure of GPT.
  */
typedef struct {
    GPT_VER_INFO_REG       GPT_VER_INFO;    /**< Version information register, offset address: 0x00000000U */
    unsigned int           resereved_0[3];
    GPT_TC_DIV_REG         GPT_TC_DIV;      /**< Frequency division coefficient register, offset address: 0x00000010U */
    GPT_TC_PRD_REG         GPT_TC_PRD;      /**< Count cycle register, offset address: 0x00000014U */
    GPT_TC_REFA0_REG       GPT_TC_REFA0;    /**< Count reference value A0 register, offset address: 0x00000018U */
    GPT_TC_REFB0_REG       GPT_TC_REFB0;    /**< Count reference value B0 register, offset address: 0x0000001CU */
    unsigned int           reserved_1[4];
    GPT_TC_STS_REG         GPT_TC_STS;      /**< Count status register, offset address: 0x00000030U */
    unsigned int           reserved_2[51];
    GPT_PG_ACT0_REG        GPT_PG_ACT0;     /**< Channel action configuration register, offset address: 0x00000100U */
    unsigned int           reserved_3[63];
    GPT_INT_EN_REG         GPT_INT_EN;      /**< Interrupt enable register, offset address: 0x00000200U */
    GPT_INT_FLAG_REG       GPT_INT_FLAG;    /**< Interrupt flag register, offset address: 0x00000204U */
    GPT_INT_INJ_REG        GPT_INT_INJ;     /**< Interrupt injection register, offset address: 0x00000208U */
    unsigned int           reserved_4[61];
    GPT_SOCDR_EN_REG       GPT_SOCDR_EN;    /**< ADC/DMA request enable register, offset address: 0x00000300U */
    unsigned int           reserved_5[63];
    GPT_PWM0_CFG_REG       GPT_PWM0_CFG;    /**< Channel configuration register, offset address: 0x00000400U */
    unsigned int           reserved_6[2];
    GPT_EN_REG             GPT_EN;          /**< GPT enable register., offset address: 0x0000040CU */
    GPT_PWM0_STA_REG       GPT_PWM0_STA;    /**< Channel status register, offset address: 0x00000410U */
    unsigned int           reserved_7[59];
    GPT_BUF_LOAD_EN_REG    GPT_BUF_LOAD_EN; /**< Cache loading enable register, offset address: 0x00000500U */
    GPT_LOAD_STS_REG       GPT_LOAD_STS;    /**< Buffer loading status register, offset address: 0x00000504U */
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
  * @brief  Verify GPT div value
  * @param div  division factor of GPT
  * @retval true
  * @retval false
  */
static inline bool IsGptDiv(unsigned int div)
{
    return (div <= GPT_PWM_DIV_FACTOR_MAX_VALUE);
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
  * @brief  Verify GPT ref dot value
  * @param  value  value of GPT ref dot
  * @retval true
  * @retval false
  */
static inline bool IsGptRefDot(unsigned int value)
{
    return (value <= GPT_TC_PRD_MAX_VALUE);
}

/**
  * @brief  Verify GPT period value
  * @param period  Period of GPT
  * @retval true
  * @retval false
  */
static inline bool IsGptAction(unsigned int action)
{
    return (action <= GPT_ACTION_OUTPUT_FLIP);
}

/**
  * @brief  Verify GPT period value
  * @param period  Period of GPT
  * @retval true
  * @retval false
  */
static inline bool IsGptSetOption(unsigned int option)
{
    return ((option == BASE_CFG_SET) || (option == BASE_CFG_UNSET));
}

/**
  * @brief  Verify GPT triggle DMA type
  * @param period  Period of GPT
  * @retval true
  * @retval false
  */
static inline bool IsGptTriggleDMAType(unsigned int triggleType)
{
    return ((triggleType <= GPT_PWM0_PERIOD_TRIGGER_DMA) && (triggleType >= GPT_PWM0_TRIGGER_DMA));
}


/* Direct Configuration Layer Functions --------------------------------------*/
/**
 * @brief   Set PWM Period
 * @param   gptx    GPTx register baseAddr
 * @param   period  Number of cycles of PWM
 * @retval  None
 */
static inline void DCL_GPT_SetPeriod(GPT_RegStruct *gptx, unsigned int period)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_PARAM_CHECK_NO_RET(IsGptPeriod(period));
    /* Setting the GPT Period */
    GPT_TC_PRD_REG prd;
    prd.reg = gptx->GPT_TC_PRD.reg;
    prd.BIT.rg_cnt_prd = period;
    gptx->GPT_TC_PRD.reg = prd.reg;
}

/**
 * @brief   Get PWM Period
 * @param   gptx    GPTx register baseAdd
 * @retval  period  Number of cycles of PWM
 */
static inline unsigned int DCL_GPT_GetPeriod(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_TC_PRD_REG prd;
    prd.reg = gptx->GPT_TC_PRD.reg;
    return prd.BIT.rg_cnt_prd;
}

/**
 * @brief   Set GPT buffer load
 * @param   gptx    GPTx register baseAddr
 * @param   buffLoad  Buffer loading
 * @retval  None
 */
static inline void DCL_GPT_SetBufLoad(GPT_RegStruct *gptx, bool buffLoad)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    /* 0x1: enable the buffer load, 0: disable the buffer load */
    GPT_BUF_LOAD_EN_REG load;
    load.reg = gptx->GPT_BUF_LOAD_EN.reg;
    load.BIT.rg_buf_load_en = buffLoad;
    gptx->GPT_BUF_LOAD_EN.reg = load.reg;
}

/**
 * @brief   Get GPT buffer load status
 * @param   gptx    GPTx register baseAddr
 * @retval  bool    1: buffer load enable, 0: buffer load disable.
 */
static inline bool DCL_GPT_GetBufLoad(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_TC_PRD_REG prd;
    prd.reg = gptx->GPT_TC_PRD.reg;
    return prd.BIT.rg_cnt_prd;
}

/**
 * @brief   Set GPT PWM output mode
 * @param   gptx        GPTx register baseAddr
 * @param   keepEnable  KeepEnable 1: Outputs PWM waves all the time, 0: Fixed number of PWM waves are output.
 * @retval  None
 */
static inline void DCL_GPT_SetOutputMode(GPT_RegStruct *gptx, bool keepEnable)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_PWM0_CFG_REG outMode;
    /* PWM output mode. 1: continuous output of PWM waveforms; 0: output of a fixed number of PWMs. */
    outMode.reg = gptx->GPT_PWM0_CFG.reg;
    outMode.BIT.rg_pwm0_keep = keepEnable;
    gptx->GPT_PWM0_CFG.reg = outMode.reg;
}

/**
 * @brief   Get GPT PWM output mode
 * @param   gptx   GPTx register baseAddr
 * @retval  bool   1: Outputs PWM waves all the time, 0: Fixed number of PWM waves are output.
 */
static inline bool DCL_GPT_GetOutputMode(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_PWM0_CFG_REG outMode;
    outMode.reg = gptx->GPT_PWM0_CFG.reg;
    return outMode.BIT.rg_pwm0_keep;
}

/**
 * @brief   Set GPT PWM output numbers, only valid when 'rg_pwm0_keep' is set to false.
 * @param   gptx        GPTx register baseAddr
 * @param   pwmNumber   The number of output PWMs.
 * @retval  None
 */
static inline void DCL_GPT_SetPWMNumber(GPT_RegStruct *gptx, bool pwmNumber)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    /* Specifies the number of output PWM wavelengths. This parameter is valid only when PWM output mode is fixed. */
    GPT_PWM0_CFG_REG pwmset;
    pwmset.reg = gptx->GPT_PWM0_CFG.reg;
    pwmset.BIT.rg_pwm0_num = pwmNumber;
    gptx->GPT_PWM0_CFG.reg = pwmset.reg;
}

/**
 * @brief   Obtains the sequence number of the PWM wave that is being output in a channel.
 * @param   gptx        GPTx register baseAddr
 * @retval  When PWM0_CFG.rg_pwm0_keep is 1, the value is always 0.
 *          When PWM0_CFG.rg_pwm0_keep is 0, PWM0_STA.ro_pwm0_num_sta indicates
 *          sequence number of the PWM wave being output by channel.
 */
static inline unsigned int DCL_GPT_GetChannelPWMNumber(GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_PWM0_STA_REG pwmStatus;
    pwmStatus.reg = gptx->GPT_PWM0_STA.reg;
    return pwmStatus.BIT.ro_pwm0_num_sta;
}

/**
 * @brief   PWM output status
 * @param   gptx        GPTx register baseAddr
 * @retval  bool : 0: Channel does not output PWM waves.
 *                 1: Channel is outputting PWM waves.
 */
static inline bool DCL_GPT_GetPWMOutPutStatus(GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_PWM0_STA_REG pwmStatus;
    pwmStatus.reg = gptx->GPT_PWM0_STA.reg;
    return pwmStatus.BIT.ro_pwm0_run_sta;
}

/**
 * @brief   Enable output period finish interrupt of software injection channel, only valid when GPT outputs PWM
 * @param   gptx   GPTx register baseAddr
 * @retval  None
 */
static inline void DCL_GPT_InjPeriodIntrruptEn(GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    /* 0x1: enables the software injection period finish interrupt,
       0x0: disables the software injection period finish interrupt. */
    GPT_INT_INJ_REG injEn;
    injEn.reg = gptx->GPT_INT_INJ.reg;
    injEn.BIT.rg_prd_int_inj = BASE_CFG_ENABLE;
    gptx->GPT_INT_INJ.reg = injEn.reg;
}

/**
 * @brief   Enable output finish interrupt of software injection channel, only valid when GPT outputs PWM
 * @param   gptx   GPTx register baseAddr
 * @retval  None
 */
static inline void DCL_GPT_InjOutFinishIntrruptEn(GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    /* 0x1: enables the software injection output finish interrupt,
       0x0: disables the software injection output finish interrupt. */
    GPT_INT_INJ_REG injEn;
    injEn.reg = gptx->GPT_INT_INJ.reg;
    injEn.BIT.rg_pwm0_int_inj = BASE_CFG_ENABLE;
    gptx->GPT_INT_INJ.reg = injEn.reg;
}

/**
 * @brief   Enable for generating the DMA burst request signal after the output of channel is complete.
 * @param   gptx   GPTx register baseAddr
 * @param   enable bool:
 *                 0: The DMA burst request is disabled when the output of channel is complete.
 *                 1: The DMA burst request signal is generated when the output of channel is complete.
 * @retval  None
 */
static inline void DCL_GPT_SetBurstDMAReqOutFin(GPT_RegStruct *gptx, bool enable)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    /* Sets the DMA burst request. 0x1:enable, 0x0:disable */
    GPT_SOCDR_EN_REG socDMA;
    socDMA.reg = gptx->GPT_SOCDR_EN.reg;
    socDMA.BIT.rg_dbr_pwm0_en = enable;
    gptx->GPT_SOCDR_EN.reg = socDMA.reg;
}

/**
 * @brief   Enable for generating the DMA single request signal after the output of channel is complete.
 * @param   gptx   GPTx register baseAddr
 * @param   enable bool:
 *                 0: The DMA single request is disabled when the output of channel is complete.
 *                 1: The DMA single request signal is generated when the output of channel is complete.
 * @retval  None
 */
static inline void DCL_GPT_SetSingleDMAReqOutFin(GPT_RegStruct *gptx, bool enable)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_SOCDR_EN_REG socDMA;
    /* Single DMA request when pwm out finish, 0x1: enable, 0:disable */
    socDMA.reg = gptx->GPT_SOCDR_EN.reg;
    socDMA.BIT.rg_dsr_pwm0_en = enable;
    gptx->GPT_SOCDR_EN.reg = socDMA.reg;
}

/**
 * @brief   Enable for generating the DMA single request signal at the end of the counting period.
 * @param   gptx   GPTx register baseAddr
 * @param   enable bool:
 *                 0: The DMA single request is disabled at the end of the counting period.
 *                 1: The DMA single request signal is generated at the end of the counting period.
 * @retval  None
 */
static inline void DCL_GPT_SetSingleDMAReqPeriod(GPT_RegStruct *gptx, bool enable)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    /* Single DMA request when pwm period output finish, 0x1: enable, 0:disable */
    GPT_SOCDR_EN_REG socDMA;
    socDMA.reg = gptx->GPT_SOCDR_EN.reg;
    socDMA.BIT.rg_dsr_prd_en = enable;
    gptx->GPT_SOCDR_EN.reg = socDMA.reg;
}

/**
 * @brief   Enable for generating the DMA burst request signal at the end of the counting period.
 * @param   gptx   GPTx register baseAddr
 * @param   enable bool:
 *                0: The DMA burst request is disabled at the end of the counting period.
 *                1: The DMA burst request signal is generated at the end of the counting period.
 * @retval  None
 */
static inline void DCL_GPT_SetBurstDMAReqPeriod(GPT_RegStruct *gptx, bool enable)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_SOCDR_EN_REG socDMA;
    /* Sets whether to initiate a DMA burst request after the PWM period output finish. */
    socDMA.reg = gptx->GPT_SOCDR_EN.reg;
    socDMA.BIT.rg_dbr_prd_en = enable;
    gptx->GPT_SOCDR_EN.reg = socDMA.reg;
}

/**
 * @brief   Enable for generating the ADC signal at the end of the counting period.
 * @param   gptx   GPTx register baseAddr
 * @param   enable bool:
 *                 0: The ADC signal is disabled when the counting period ends.
 *                 1: The ADC signal is generated at the end of the counting period.
 * @retval  None
 */
static inline void DCL_GPT_SetADCReqPeriod(GPT_RegStruct *gptx, bool enable)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    /* Sets whether to initiate an ADC request after the PWM period output finish. */
    GPT_SOCDR_EN_REG socDMA;
    socDMA.reg = gptx->GPT_SOCDR_EN.reg;
    socDMA.BIT.rg_soc_prd_en = enable;
    gptx->GPT_SOCDR_EN.reg = socDMA.reg;
}

/**
 * @brief   Enables the SOC signal generated when the output of channel is complete.
 * @param   gptx   GPTx register baseAddr
 * @param   enable bool:
                   0: The SOC signal is disabled when the output of channel is complete.
                   1: The SoC signal is generated when the output of channel is complete.
 * @retval  None
 */
static inline void DCL_GPT_SetADCReqOutFin(GPT_RegStruct *gptx, bool enable)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    /* Sets whether to initiate an ADC request after the PWM output finish. */
    GPT_SOCDR_EN_REG socDMA;
    socDMA.reg = gptx->GPT_SOCDR_EN.reg;
    socDMA.BIT.rg_soc_pwm0_en = enable;
    gptx->GPT_SOCDR_EN.reg = socDMA.reg;
}


/**
 * @brief   Set PWM Divider factor
 * @param   gptx    GPTx register baseAddr
 * @param   div     divison factor
 * @retval  None
 */
static inline void DCL_GPT_SetDiv(GPT_RegStruct *gptx, unsigned int div)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_PARAM_CHECK_NO_RET(IsGptDiv(div));
    GPT_TC_DIV_REG gptDiv;
    /* Sets the frequency division of GPT. The value ranges from 1 to 4095. */
    gptDiv.reg = gptx->GPT_TC_DIV.reg;
    gptDiv.BIT.rg_div_fac = div;
    gptx->GPT_TC_DIV.reg = gptDiv.reg;
}

/**
 * @brief   Get PWM Divider factor
 * @param   gptx    GPTx register baseAddr
 * @retval  div     divison factor
 */
static inline unsigned int DCL_GPT_GetDiv(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_TC_DIV_REG div;
    div.reg = gptx->GPT_TC_DIV.reg;
    return div.BIT.rg_div_fac;
}

/**
 * @brief   Get PWM current count value of the divider
 * @param   gptx    GPTx register baseAddr
 * @retval  divcnt  Counter of current div counter value
 */
static inline unsigned int DCL_GPT_GetDivCnt(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_TC_STS_REG value;
    value.reg = gptx->GPT_TC_STS.reg;
    return value.BIT.ro_div_cnt;
}

/**
 * @brief   Set PWM Current Counter value
 * @param   gptx    GPTx register baseAddr
 * @retval  counter The current count value of the counter.
 */
static inline unsigned int DCL_GPT_GetCounterValue(GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_TC_STS_REG value;
    value.reg = gptx->GPT_TC_STS.reg;
    return value.BIT.ro_cnt_val;
}

/**
 * @brief   Set Reference A Action
 * @param   gptx    GPTx register baseAddr
 * @param   action   When the counter is equal to the reference value A, the PWM output action, @ref GPT_ActionType
 * @retval  None
 */
static inline void DCL_GPT_SetRefAAction(GPT_RegStruct *gptx, GPT_ActionType action)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_PARAM_CHECK_NO_RET(IsGptAction(action));
    gptx->GPT_PG_ACT0.BIT.rg_pg_act0_refa0 = action;
}

/**
 * @brief   Get Reference A Action
 * @param   gptx    GPTx register baseAddr
 * @retval  action  When the counter is equal to the reference value A, the PWM output action
 */
static inline unsigned int DCL_GPT_GetRefAAction(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    return gptx->GPT_PG_ACT0.BIT.rg_pg_act0_refa0;
}

/**
 * @brief   Set Reference B Action
 * @param   gptx    GPTx register baseAddr
 * @param   action  When the counter is equal to the reference value B, the PWM output action, @ref GPT_ActionType
 * @retval  None
 */
static inline void DCL_GPT_SetRefBAction(GPT_RegStruct *gptx, GPT_ActionType action)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    GPT_PARAM_CHECK_NO_RET(IsGptAction(action));
    gptx->GPT_PG_ACT0.BIT.rg_pg_act0_refb0 = action;
}

/**
 * @brief   Get Reference B Action
 * @param   gptx    GPTx register baseAddr
 * @retval  action  When the counter is equal to the reference value B, the PWM output action
 */
static inline unsigned int DCL_GPT_GetRefBAction(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    return gptx->GPT_PG_ACT0.BIT.rg_pg_act0_refb0;
}

/**
 * @brief   Set Interrupt Enable/Disable
 * @param   gptx    GPTx register baseAddr
 * @param   enable  interrupt enable or disable
 * @retval  None
 */
static inline void DCL_GPT_SetInterruptEn(GPT_RegStruct *gptx, bool enable)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    gptx->GPT_INT_EN.BIT.rg_pwm0_int_en = enable;
}

/**
 * @brief   Set Interrupt Enable/Disable
 * @param   gptx    GPTx register baseAddr
 * @retval  enable  interrupt enable or disable
 */
static inline unsigned int DCL_GPT_GetInterruptEn(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    return gptx->GPT_INT_EN.BIT.rg_pwm0_int_en;
}

/**
 * @brief   Set Period Interrupt Enable/Disable
 * @param   gptx    GPTx register baseAddr
 * @param   enable  interrupt enable or disable
 * @retval  None
 */
static inline void DCL_GPT_SetPeriodInterruptEn(GPT_RegStruct *gptx, bool enable)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    gptx->GPT_INT_EN.BIT.rg_prd_int_en = enable;
}

/**
 * @brief   Get Period Interrupt Enable/Disable
 * @param   gptx    GPTx register baseAddr
 * @retval  enable  interrupt enable or disable
 */
static inline unsigned int DCL_GPT_GetPeriodInterruptEn(const GPT_RegStruct *gptx)
{
    GPT_ASSERT_PARAM(IsGPTInstance(gptx));
    return gptx->GPT_INT_EN.BIT.rg_prd_int_en;
}

/**
  * @}
  */

/**
 * @}
 */
#endif /* McuMagicTag_GPT_IP_H */