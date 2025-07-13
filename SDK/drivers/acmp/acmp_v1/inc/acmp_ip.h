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
  * @file    acmp_ip.h
  * @author  MCU Driver Team
  * @brief   ACMP module driver.
  *          This file provides DCL functions to manage ACMP and Definitions of specific parameters.
  *           + Definition of ACMP configuration parameters.
  *           + ACMP register mapping structure.
  *           + Parameters check functions.
  *           + Direct configuration layer interface.
  */

#ifndef McuMagicTag_ACMP_IP_H
#define McuMagicTag_ACMP_IP_H

#include "baseinc.h"

#ifdef ACMP_PARAM_CHECK
#define ACMP_ASSERT_PARAM BASE_FUNC_ASSERT_PARAM
#define ACMP_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define ACMP_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define ACMP_ASSERT_PARAM(para) ((void)0U)
#define ACMP_PARAM_CHECK_NO_RET(para) ((void)0U)
#define ACMP_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif

#define ACMP_FILTER_STEP_MAX_VALUE    0x00000FFFEU

/**
  * @addtogroup ACMP
  * @{
  */

/**
  * @defgroup ACMP_IP ACMP_IP
  * @brief ACMP_IP: acmp_v1.
  * @{
  */

/**
 * @defgroup ACMP_Param_Def ACMP Parameters Definition
 * @brief Definition of ACMP configuration parameters
 * @{
 */

/**
  * @brief Comparator blking source type
  * @details Description:
  *          + ACMP_BLKING_SRC_SOFT    ----    The software configuration masks the window.
  *          + ACMP_BLKING_SRC_APT0    ----    APT0 output mask window.
  *          + ACMP_BLKING_SRC_APT1    ----    APT1 output mask window.
  *          + ACMP_BLKING_SRC_APT2    ----    APT2 output mask window.
  *          + ACMP_BLKING_SRC_APT3    ----    APT3 output mask window.
  */
typedef enum {
    ACMP_BLKING_SRC_SOFT = 0x00000000U,
    ACMP_BLKING_SRC_APT0 = 0x00000001U,
    ACMP_BLKING_SRC_APT1 = 0x00000002U,
    ACMP_BLKING_SRC_APT2 = 0x00000003U,
    ACMP_BLKING_SRC_APT3 = 0x00000004U,
} ACMP_BlkingSrcType;

/**
  * @brief Comparator hysteresis voltage
  * @details Description:
  *          + ACMP_HYS_VOL_ZERO    ----    Hysteresis voltage 0 mv.
  *          + ACMP_HYS_VOL_10MV    ----    Hysteresis voltage 10 mv.
  *          + ACMP_HYS_VOL_20MV    ----    Hysteresis voltage 20 mv.
  *          + ACMP_HYS_VOL_30MV    ----    Hysteresis voltage 30 mv.
  */
typedef enum {
    ACMP_HYS_VOL_ZERO = 0x00000000U,
    ACMP_HYS_VOL_10MV = 0x00000001U,
    ACMP_HYS_VOL_20MV = 0x00000002U,
    ACMP_HYS_VOL_30MV = 0x00000003U,
} ACMP_HystVol;

/**
  * @brief ACMP P port input select.
  * @details Description:
  *          + ACMP_INPUT_P_SELECT0    ----    Signal source PGA0_OUT.
  *          + ACMP_INPUT_P_SELECT1    ----    Signal source PGA1_OUT.
  *          + ACMP_INPUT_P_SELECT2    ----    From pin (GPIO0_5).
  *          + ACMP_INPUT_P_SELECT3    ----    From pin (GPIO2_5).
  *          + ACMP_INPUT_P_SELECT4    ----    From pin (GPIO3_5).
  *          + ACMP_INPUT_P_SELECT5    ----    Signal source DAC_OUT.
  */
typedef enum {
    ACMP_INPUT_P_SELECT0  = 0x00000000U,
    ACMP_INPUT_P_SELECT1  = 0x00000001U,
    ACMP_INPUT_P_SELECT2  = 0x00000002U,
    ACMP_INPUT_P_SELECT3  = 0x00000003U,
    ACMP_INPUT_P_SELECT4  = 0x00000004U,
    ACMP_INPUT_P_SELECT5  = 0x00000005U,
} ACMP_InputPSel;

/**
  * @brief ACMP N port input select.
  * @details Description:
  *          + ACMP_INPUT_N_SELECT0    ----     Signal source DAC_OUT.
  *          + ACMP_INPUT_N_SELECT1    ----     None.
  *          + ACMP_INPUT_N_SELECT2    ----     From pin (GPIO0_6).
  *          + ACMP_INPUT_N_SELECT3    ----     From pin (GPIO2_6).
  *          + ACMP_INPUT_N_SELECT4    ----     From pin (GPIO3_6).
  *          + ACMP_INPUT_N_SELECT5    ----     Signal source DAC_OUT.
  */
typedef enum {
    ACMP_INPUT_N_SELECT0  = 0x00000000U,
    ACMP_INPUT_N_SELECT1  = 0x00000001U,
    ACMP_INPUT_N_SELECT2  = 0x00000002U,
    ACMP_INPUT_N_SELECT3  = 0x00000003U,
    ACMP_INPUT_N_SELECT4  = 0x00000004U,
    ACMP_INPUT_N_SELECT5  = 0x00000005U,
} ACMP_InputNSel;

/**
  * @brief Comparator output polarity
  */
typedef enum {
    ACMP_OUT_NOT_INVERT = 0x00000000U,
    ACMP_OUT_INVERT     = 0x00000001U,
} ACMP_OutputPolarity;

/**
  * @brief ACMP output selection
  * @details Description:
  *          + ACMP_RESULT_SIMULATION       ----     Simulate the original comparison result.
  *          + ACMP_RESULT_FILTER           ----     Digital Filtering Comparison Results.
  *          + ACMP_RESULT_FILTER_BLOCK     ----     Digital Filtering and Blocking Comparison Results.
  */
typedef enum {
    ACMP_RESULT_SIMULATION       = 0x00000000U,
    ACMP_RESULT_FILTER           = 0x00000001U,
    ACMP_RESULT_FILTER_BLOCK     = 0x00000002U,
} ACMP_ResultSelect;

/**
  * @brief Comparator filter mode
  * @details Description:
  *          + ACMP_FILTER_NONE       ----     Raw analog comparison.
  *          + ACMP_FILTER_BLOCK      ----     Blocking function.
  *          + ACMP_FILTER_FILTER     ----     Filtering funciton.
  *          + ACMP_FILTER_BOTH       ----     Filtering and Blocking function.
  */
typedef enum {
    ACMP_FILTER_NONE     =   0x00000000U,
    ACMP_FILTER_BLOCK    =   0x00000001U,
    ACMP_FILTER_FILTER   =   0x00000002U,
    ACMP_FILTER_BOTH     =   0x00000003U,
} ACMP_FilterMode;

/**
  * @brief Comparator filter control structure
  */
typedef struct {
    ACMP_FilterMode      filterMode;            /**< ACMP filter mode. */
    ACMP_BlkingSrcType   blkingSrcSelect;       /**< Blocking source select.*/
    unsigned short       filterStep;            /**< Filter Step. */
    bool                 blkingPorty;           /**< Polarity select of the mask window. */
} ACMP_FilterCtrl;

/**
  * @brief Comparator input and output configuration structure
  */
typedef struct {
    ACMP_OutputPolarity    polarity;           /**< output polarity settings */
    ACMP_InputPSel         inputPNum;          /**< ACMP input positive number */
    ACMP_InputNSel         inputNNum;          /**< ACMP input negative number */
} ACMP_InOutConfig;

/**
  * @brief ACMP user callback function type.
  */
typedef enum {
    ACMP_POS_INT          = 0x00000000U,
    ACMP_NEG_INT          = 0x00000001U,
    ACMP_EDGE_INT         = 0x00000002U,
} ACMP_CallBackFun_Type;

/**
  * @brief ACMP user interrupt callback function.
  */
typedef struct {
    void (* AcmpPositiveCallBack)(void *handle);   /**< Rising edge interrupt callback function. */
    void (* AcmpNegativeCallBack)(void *handle);   /**< Falling edge interrupt callback function. */
    void (* AcmpEdgedCallBack)(void *handle);      /**< Flip edge interrupt callback function. */
} ACMP_UserCallBack;

/**
  * @brief ACMP extend configure.
  */
typedef struct {
} ACMP_ExtendHandle;

/**
  * @}
  */

/**
  * @defgroup ACMP_REG_Definition ACMP Register Structure.
  * @brief ACMP Register Structure Definition.
  * @{
  */

/**
  * @brief ACMP control reg 0.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int  da_acmp_enh        : 1;    /**< Comparator enable signal. */
        unsigned int  reserved_0         : 31;
    } BIT;
} volatile ACMP_CTRL_REG0;

/**
  * @brief ACMP control reg 1.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int  da_acmp_input_psel  : 3;   /**< Input P vin selection */
        unsigned int  da_acmp_input_nsel  : 3;   /**< Input N vin selection */
        unsigned int  reserved_0          : 26;
    } BIT;
} volatile ACMP_CTRL_REG1;

/**
  * @brief ACMP control reg 2.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int  cfg_acmp_out_sel    : 2;     /**< Comparator output result selection:
                                                        0: original comparison result;
                                                        1: result after filtering;
                                                        2: masked result;
                                                        3: 0. */
        unsigned int  cfg_acmp_out_inv    : 1;     /**< Comparator result polarity selection:
                                                        0: The result is not reversed.
                                                        1: The result is reversed. */
        unsigned int  reserved_0          : 29;
    } BIT;
} volatile ACMP_CTRL_REG2;

/**
  * @brief ACMP filtering control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int  cfg_acmp_filter_en      : 1;      /**< Comparator filtering enable:
                                                             0: disabled;
                                                             1: enabled. */
        unsigned int  cfg_acmp_filter_step    : 16;     /**< Filter step size of the comparator. */
        unsigned int  reserved_0              : 15;
    } BIT;
} volatile ACMP_CTRL_REG3;

/**
  * @brief ACMP mask control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int  cfg_acmp_blk_en         : 1;      /**< Comparator mask enable:
                                                             0: disabled;
                                                             1: enabled. */
        unsigned int  reserved_0              : 7;
        unsigned int  cfg_acmp_blk_sel        : 3;     /**< Comparator Mask Window Selection:
                                                            0: The window is masked by software.
                                                            1: APT0 output mask window;
                                                            2: APT1 output mask window;
                                                            3: APT2 output mask window;
                                                            4: APT3 output mask window;
                                                            else: 0. */
        unsigned int  reserved_1              : 5;
        unsigned int  cfg_acmp_blk_win        : 1;    /**< The software configuration mask window is displayed. */
        unsigned int  reserved_2              : 7;
        unsigned int  cfg_acmp_blk_pol_sel    : 1;   /**< Select the polarity of the mask window.
                                                          0: The high-level mask window is valid.
                                                          1: The low-level shielding window is valid. */
        unsigned int  cfg_acmp_blk_rslt_pol   : 1;   /**< Polarity selection of the masking result:
                                                          0: The masking result is low level.
                                                          1: The masking result is high level. */
        unsigned int  reserved_3              : 6;
    } BIT;
} volatile ACMP_CTRL_REG4;


/**
  * @brief ACMP interrupt raw status register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int  intr_acmp_edge         : 1;    /**< Comparison result reversal edge interrupt status. */
        unsigned int  intr_acmp_neg          : 1;    /**< Comparison result falling edge interrupt status. */
        unsigned int  intr_acmp_pos          : 1;    /**< Interrupt status on the rising edge of comparison result. */
        unsigned int  reserved               : 29;
    } BIT;
} volatile ACMP_INTR_REG;


/**
  * @brief Masked ACMP interrupt status register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int  intr_acmp_edge_msk   : 1;  /**< Status of comparison result reversal edge masked interrupt. */
        unsigned int  intr_acmp_neg_msk    : 1;  /**< Int status after falling edge of comparison result is masked. */
        unsigned int  intr_acmp_pos_msk    : 1;  /**< Int status after rising edge of comparison result is masked. */
        unsigned int  reserved             : 29;
    } BIT;
} volatile ACMP_INTR_MSK_REG;


/**
  * @brief ACMP interrupt mask.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int  intr_acmp_edge_mask         : 1;  /**< Comparison result reversal edge interrupt mask register:
                                                              0: mask interrupts.
                                                              1: not masked. */
        unsigned int  intr_acmp_neg_mask          : 1;  /**< Comparison result falling edge interrupt mask register:
                                                              0: mask interrupts.
                                                              1: not masked. */
        unsigned int  intr_acmp_pos_mask          : 1;  /**<  Comparison result rising edge interrupt mask register:
                                                              0: mask interrupts.
                                                              1: not masked. */
        unsigned int  reserved                    : 29;
    } BIT;
} volatile ACMP_INTR_MASK_REG;


/**
  * @brief ACMP result register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int  cmp_ana_rslt             : 1;      /**< Original comparison result. */
        unsigned int  cmp_filter_rslt          : 1;      /**< Filtered result of the comparator. */
        unsigned int  cmp_blk_rslt             : 1;      /**< Result after the comparator is masked. */
        unsigned int  reserved                 : 29;
    } BIT;
} volatile ACMP_RSLT_REG;

/**
  * @brief ACMP enable delay register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int  cfg_acmp_en_dly          : 8;      /**< Indicates the delay for enabling ACMP (us). */
        unsigned int  reserved                 : 24;
    } BIT;
} volatile ACMP_EN_DLY_REG;

/**
  * @brief ACMP test register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int  da_acmp_test_enh          : 1;      /**< Test enable signal:
                                                               0: disabled;
                                                               1: enabled. */
        unsigned int  da_acmp_test_sel          : 8;      /**< Test signal strobe. */
        unsigned int  reserved                  : 23;
    } BIT;
} volatile ACMP_TEST_REG;


/**
  * @brief ACMP TRIM register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int  da_acmp_trim              : 8;      /**< ACMP TIRM register. */
        unsigned int  reserved                  : 24;
    } BIT;
} volatile ACMP_TRIM_REG;


/**
  * @brief ACMP reserved register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int  da_acmp_rsv               : 2;      /**< Reserved comparator register:
                                                               <1: 0>: The hysteresis voltage is selected. */
        unsigned int  reserved                  : 30;
    } BIT;
} volatile ACMP_RSV_REG;

/**
  * @brief ACMP registers definition structure.
  */
typedef struct _ACMP_RegStruct {
    ACMP_CTRL_REG0        ACMP_CTRL0;       /**< ACMP control register 0. Offset address: 0x00000000U. */
    ACMP_CTRL_REG1        ACMP_CTRL1;       /**< ACMP control register 1. Offset address: 0x00000004U. */
    ACMP_CTRL_REG2        ACMP_CTRL2;       /**< ACMP control register 2. Offset address: 0x00000008U. */
    char                  space0[52];
    ACMP_CTRL_REG3        ACMP_CTRL3;       /**< ACMP filtering control register. Offset address: 0x00000040U. */
    ACMP_CTRL_REG4        ACMP_CTRL4;       /**< ACMP mask control register. Offset address: 0x00000044U. */
    char                  space1[8];
    ACMP_INTR_REG         ACMP_INTR;        /**< ACMP interrupt raw status register. Offset address: 0x00000050U. */
    ACMP_INTR_MSK_REG     ACMP_INTR_MSK;    /**< Masked ACMP interrupt status register. Offset address: 0x00000054U. */
    ACMP_INTR_MASK_REG    ACMP_INTR_MASK;   /**< ACMP interrupt mask register. Offset address: 0x00000058U. */
    char                  space2[20];
    ACMP_RSLT_REG         ACMP_RSLT;        /**< ACMP result register. Offset address: 0x00000070U. */
    char                  space3[12];
    ACMP_EN_DLY_REG       ACMP_EN_DLY;      /**< ACMP enable delay register. Offset address: 0x00000080U. */
    char                  space4[4];
    ACMP_TRIM_REG         ACMP_TRIM;        /**< ACMP TRIM register. Offset address: 0x00000088U. */
    ACMP_RSV_REG          ACMP_RSV;         /**< ACMP reserved register. Offset address: 0x0000008CU. */
} volatile ACMP_RegStruct;

/* Parameter Check------------------------------------------------------------------ */
/**
  * @brief Verify ACMP output polarity configuration.
  * @param polarity: ACMP output polarity
  * @retval true
  * @retval false
  */
static inline bool IsACMPOutputPolarity(ACMP_OutputPolarity polarity)
{
    return ((polarity == ACMP_OUT_NOT_INVERT) || (polarity == ACMP_OUT_INVERT));
}

/**
  * @brief Verify ACMP input P number.
  * @param pNumber: ACMP output source select
  * @retval true
  * @retval false
  */
static inline bool IsACMPInputPNumber(ACMP_InputPSel pNumber)
{
    return (pNumber <= ACMP_INPUT_P_SELECT5);
}

/**
  * @brief Verify ACMP input N number.
  * @param NNumber: ACMP output source select
  * @retval true
  * @retval false
  */
static inline bool IsACMPInputNNumber(ACMP_InputNSel NNumber)
{
    return (NNumber <= ACMP_INPUT_N_SELECT5);
}

/**
  * @brief Verify ACMP blocking source type.
  * @param BlkingSrcType: ACMP output source select
  * @retval true
  * @retval false
  */
static inline bool IsACMPBlkingSrcType(ACMP_BlkingSrcType BlkingSrcType)
{
    return (BlkingSrcType <= ACMP_BLKING_SRC_APT3);
}

/**
  * @brief Verify ACMP output result selection
  * @param resultSelection: ACMP output source selection.
  * @retval true
  * @retval false
  */
static inline bool IsACMPResultSeletion(ACMP_ResultSelect resultSelection)
{
    return (resultSelection <= ACMP_RESULT_FILTER_BLOCK);
}

/* Direct configuration layer ------------------------------------------------*/
/**
  * @brief Set input switch
  * @param acmpx: ACMP register base address.
  * @param inputP: ACMP inputP selection. @ref ACMP_VinSel
  * @param inputN: ACMP inputN selection. @ref ACMP_VinSel
  * @retval None.
  */
static inline void DCL_ACMP_SetInputSwith(ACMP_RegStruct *acmpx, ACMP_InputPSel inputP, ACMP_InputNSel inputN)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    ACMP_PARAM_CHECK_NO_RET(inputP >= ACMP_INPUT_P_SELECT0);
    ACMP_PARAM_CHECK_NO_RET(inputP <= ACMP_INPUT_P_SELECT5);
    ACMP_PARAM_CHECK_NO_RET(inputN >= ACMP_INPUT_N_SELECT0);
    ACMP_PARAM_CHECK_NO_RET(inputN <= ACMP_INPUT_N_SELECT5);
    acmpx->ACMP_CTRL1.BIT.da_acmp_input_nsel = inputN;  /* Input port on the P side. */
    acmpx->ACMP_CTRL1.BIT.da_acmp_input_psel = inputP;  /* Input port on the N side. */
}

/**
  * @brief ACMP output(deshark and synchronize) source.
  * @param acmp: ACMP register base address.
  * @param resultSelection: config value. @ref ACMP_ResultSelect
  * @retval None.
  */
static inline void DCL_ACMP_SetCmpOutputSrc(ACMP_RegStruct *acmpx, ACMP_ResultSelect resultSelection)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    ACMP_PARAM_CHECK_NO_RET(resultSelection >= ACMP_RESULT_SIMULATION);
    ACMP_PARAM_CHECK_NO_RET(resultSelection <= ACMP_RESULT_FILTER_BLOCK);
    acmpx->ACMP_CTRL2.BIT.cfg_acmp_out_sel = resultSelection;   /* ACMP output result select. */
}

/**
  * @brief  Comparator enable blking function
  * @param  acmpx: ACMP register base address.
  * @retval None.
  */
static inline void DCL_ACMP_EnableCmpBlking(ACMP_RegStruct *acmpx)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    acmpx->ACMP_CTRL4.BIT.cfg_acmp_blk_en = BASE_CFG_ENABLE;
}

/**
  * @brief  Comparator disable blking function
  * @param acmpx: ACMP register base address.
  * @retval None.
  */
static inline void DCL_ACMP_DisableCmpBlking(ACMP_RegStruct *acmpx)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    acmpx->ACMP_CTRL4.BIT.cfg_acmp_blk_en  = BASE_CFG_DISABLE;
}

/**
  * @brief Enable the software masking window.
  * @param  acmpx: ACMP register base address.
  * @retval None.
  */
static inline void DCL_ACMP_EnableSoftBlking(ACMP_RegStruct *acmpx)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    acmpx->ACMP_CTRL4.BIT.cfg_acmp_blk_win = BASE_CFG_ENABLE;
}

/**
  * @brief Disable the software masking window.
  * @param acmpx: ACMP register base address.
  * @retval None.
  */
static inline void DCL_ACMP_DisableSoftBlking(ACMP_RegStruct *acmpx)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    acmpx->ACMP_CTRL4.BIT.cfg_acmp_blk_win = BASE_CFG_DISABLE;
}

/**
  * @brief Set blking source.
  * @param acmpx: ACMP register base address.
  * @param source: Source of blking. @ref ACMP_BlkingSrcType
  * @retval None.
  */
static inline void DCL_ACMP_SetCmpBlkingSource(ACMP_RegStruct *acmpx, ACMP_BlkingSrcType source)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    ACMP_PARAM_CHECK_NO_RET(source >= ACMP_BLKING_SRC_SOFT);
    ACMP_PARAM_CHECK_NO_RET(source <= ACMP_BLKING_SRC_APT3);
    acmpx->ACMP_CTRL4.BIT.cfg_acmp_blk_sel = source;
}

/**
  * @brief Set comparator hysteresis voltage.
  * @param acmpx: ACMP register base address.
  * @param volSelect: Hysteresis voltage selection. @ref ACMP_HystVol
  * @retval None.
  */
static inline void DCL_ACMP_SetCmpHysteresisVoltage(ACMP_RegStruct *acmpx, ACMP_HystVol volSelect)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    ACMP_PARAM_CHECK_NO_RET(volSelect >= ACMP_HYS_VOL_ZERO);
    ACMP_PARAM_CHECK_NO_RET(volSelect <= ACMP_HYS_VOL_30MV);
    acmpx->ACMP_RSV.BIT.da_acmp_rsv = volSelect;
}

/**
  * @brief Set comparator's output polarity
  * @param acmp: ACMP register base address.
  * @param polarity: output polarity. @ref ACMP_OutputPolarity
  * @retval None.
  */
static inline void DCL_ACMP_SetCmpOutputPolarity(ACMP_RegStruct *acmpx, ACMP_OutputPolarity polarity)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    ACMP_PARAM_CHECK_NO_RET(polarity >= ACMP_OUT_NOT_INVERT);
    ACMP_PARAM_CHECK_NO_RET(polarity <= ACMP_OUT_INVERT);
    acmpx->ACMP_CTRL2.BIT.cfg_acmp_out_inv = polarity;
}

/**
  * @brief Reading compare result after blocking.
  * @param acmp: ACMP register base address.
  * @retval Blocked result.
  */
static inline unsigned int DCL_ACMP_GetCmpOutValueAfterBlking(ACMP_RegStruct *acmpx)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    return acmpx->ACMP_RSLT.BIT.cmp_blk_rslt;
}

/**
  * @brief Reading compare result after filtering.
  * @param acmp: ACMP register base address.
  * @retval filtered result.
  */
static inline unsigned int DCL_ACMP_GetCmpOutValueAfterFilter(ACMP_RegStruct *acmpx)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    return acmpx->ACMP_RSLT.BIT.cmp_filter_rslt;
}

/**
  * @brief Reading original compare result
  * @param acmp: ACMP register base address.
  * @retval original result.
  */
static inline unsigned int DCL_ACMP_GetCmpOutValueOriginal(ACMP_RegStruct *acmpx)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    return acmpx->ACMP_RSLT.BIT.cmp_ana_rslt;
}

/**
  * @brief Set deshark step by clock.
  * @param acmp: ACMP register base address.
  * @param step: ACMP filter step.
  * @retval None.
  */
static inline void DCL_ACMP_SetFilterStep(ACMP_RegStruct *acmpx, unsigned short step)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    ACMP_PARAM_CHECK_NO_RET(step <= ACMP_FILTER_STEP_MAX_VALUE);
    acmpx->ACMP_CTRL3.BIT.cfg_acmp_filter_step = step;
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
#endif
