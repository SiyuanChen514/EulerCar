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

#define ACMP_MAX_APT_NUM 8
#define ACMP_BLKING_APT_MAX 8
#define ACMP_DESHARK_BY_CLK_MAX   0x0000001fU
#define ACMP_DESHARK_BY_CMP_MAX   0x0000001fU
/**
  * @addtogroup ACMP
  * @{
  */

/**
  * @defgroup ACMP_IP ACMP_IP
  * @brief ACMP_IP: acmp_v0.
  * @{
  */

/**
 * @defgroup ACMP_Param_Def ACMP Parameters Definition
 * @brief Definition of ACMP configuration parameters
 * @{
 */

/**
  * @brief Polarity of the ACMP mask window.
  */
typedef enum {
    ACMP_BLKING_LOW_ACTIVE    = 0x00000000U,
    ACMP_BLKING_HIGH_ACTIVE   = 0x00000001U,
} ACMP_BlkingActLevel;

/**
  * @brief Comparator blking source type
  * @details Description:
  *          + ACMP_BLKING_SRC_APT    ----  Blocking source from APT window.
  *          + ACMP_BLKING_SRC_SOFT   ----  Blocking source from software.
  */
typedef enum {
    ACMP_BLKING_SRC_APT = 0x00000000U,
    ACMP_BLKING_SRC_SOFT = 0x00000001U,
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
  * @brief Output source selection.
  * @details Description:
  *          + ACMP_ORIGINAL_TO_PORT   ----    Original comparison result is output to the port.
  *          + ACMP_DESHARK_TO_PORT    ----    Dejittered comparison result is output to the port.
  *          + ACMP_SYNC_TO_PORT       ----    Comparison result after synchronization is output to the port.
  */
typedef enum {
    ACMP_ORIGINAL_TO_PORT   = 0x00000000U,
    ACMP_DESHARK_TO_PORT    = 0x00000001U,
    ACMP_SYNC_TO_PORT       = 0x00000002U,
} ACMP_OutputSrcSel;

/**
  * @brief Comparator vin select
  */
typedef enum {
    ACMP_VIN_MUX0 = 0x00000001U,
    ACMP_VIN_MUX1 = 0x00000002U,
    ACMP_VIN_MUX2 = 0x00000004U,
    ACMP_VIN_MUX3 = 0x00000008U,
} ACMP_VinMux;

/**
  * @brief VIN switch selection
  */
typedef enum {
    ACMP_SW_VIN0    = 0x00000001U,
    ACMP_SW_VIN1    = 0x00000002U,
    ACMP_SW_VIN2    = 0x00000004U,
    ACMP_SW_VIN3    = 0x00000008U,
} ACMP_VinSel;

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
  *          + ACMP_RESULT_SIMULATION   ----   Original comparison result.
  *          + ACMP_RESULT_DESHAKE      ----   Comparison results after deshake.
  *          + ACMP_RESULT_DELAY        ----   Outputting the comparison result is delayed by one cycle.
  */
typedef enum {
    ACMP_RESULT_SIMULATION = 0x00000000U,
    ACMP_RESULT_DESHAKE    = 0x00000001U,
    ACMP_RESULT_DELAY      = 0x00000002U,
} ACMP_ResultSelect;

/**
  * @brief Comparator filter mode
  * @details Description:
  *          + ACMP_FILTER_NONE    ----   Digital filtering is not used.
  *          + ACMP_FILTER_BLKING  ----   Use blocking processing.
  *          + ACMP_FILTER_DESHARK ----   Use deshark filter processing.
  *          + ACMP_FILTER_BOTH    ----   Use deshark and blocking.
  */
typedef enum {
    ACMP_FILTER_NONE        =    0x00000000U,
    ACMP_FILTER_BLKING      =    0x00000001U,
    ACMP_FILTER_DESHARK     =    0x00000002U,
    ACMP_FILTER_BOTH        =    0x00000003U,
} ACMP_FilterMode;

/**
  * @brief Comparator filter control structure
  */
typedef struct {
    ACMP_FilterMode       filterMode;          /**< filter mode */
    ACMP_BlkingSrcType    blkingSrcSelect;     /**< blking source select. from apt or softeware*/
    unsigned short        desharkByStep;       /**< desharl set by steps*/
    unsigned short        desharkByTimes;      /**< deshark set by times*/
    unsigned short        blkingFromAptNum;    /**< Blocking source from APT number */
    bool                  blkingPorty;         /**< Software blanking polarity. */
} ACMP_FilterCtrl;

/**
  * @brief Comparator input and output configuration structure
  */
typedef struct {
    ACMP_OutputPolarity   polarity;          /**< output polarity settings */
    ACMP_VinMux           vinPNum;           /**< vin positive number */
    ACMP_VinMux           vinNNum;           /**< vin negative number */
    ACMP_VinSel           swVinPNum;         /**< switch vin positive number */
    ACMP_VinSel           swVinNNum;         /**< switch vin negative number */
} ACMP_InOutConfig;

/**
  * @brief ACMP user callback function type.
  */
typedef struct {
} ACMP_CallBackFun_Type;

/**
  * @brief ACMP user interrupt callback function.
  */
typedef struct {
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
  * @brief CMP control 0.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int  cmp_ana_en         : 1;  /**< CMP power enable. */
        unsigned int  cmp_rslt_inv       : 1;  /**< Output polarity select of the comparison result. */
        unsigned int  cmp_hy_vol_sel     : 2;  /**< Select the comparator hysteresis voltage. */
        unsigned int  cmp_sync_sel       : 1;  /**< CMP comparison result selection, which together with
                                                    cmp_qual_en determines the source of the comparison result. */
        unsigned int  cmp_dig_en         : 1;  /**< CMP digital part enable. */
        unsigned int  cmp_blking_cfg     : 1;  /**< Software blanking signal. The blanking polarity is controlled
                                                    by cmp_blking_inv_sel. */
        unsigned int  cmp_blking_sel     : 1;  /**< Source of the blanking signal. */
        unsigned int  cmp_apt_sel        : 4;  /**< Select the APT label from the APT Mask window. */
        unsigned int  cmp_blking_en      : 1;  /**< Blanking signal enable. */
        unsigned int  cmp_qual_en        : 1;  /**< CMP comparison result selection. This parameter and cmp_sync_sel
                                                    determine the source of the comparison result. */
        unsigned int  cmp_blking_inv_sel : 1;  /**< Level selection for which CMP blanking takes effect. */
        unsigned int  reserved_0         : 17;
    } BIT;
} volatile CMP_CTRL_REG;

/**
  * @brief CMP control 1.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int  cmp_mux_n  : 4;          /**< VINN select. Only one bit can be set to 1. */
        unsigned int  cmp_mux_p  : 4;          /**< VINP select. Only one bit can be set to 1. */
        unsigned int  reserved_0 : 24;
    } BIT;
} volatile CMP_CTRL1_REG;

/**
  * @brief CMP control 2.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int  cmp_ana_rslt    : 1;     /**< Simulate the original comparison result. */
        unsigned int  cmp_qual_rslt   : 1;     /**< Comparison results after deshake. */
        unsigned int  cmp_blking_rslt : 1;     /**< Comparison result after blanking. */
        unsigned int  reserved_0      : 29;
    } BIT;
} volatile CMP_CTRL2_REG;

/**
  * @brief CMP debounce.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cmp_qual_sel  : 5;       /**< Deshake gear. */
        unsigned int reserved_0    : 3;
        unsigned int cmp_qual_step : 5;       /**< Deshake step. */
        unsigned int reserved_1    : 19;
    } BIT;
} volatile CMP_QUALI2_REG;

/**
  * @brief CMP input SW control.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cmp_sw_enlv_n  : 4;  /**< SW control of ACMP N end, which is configured together with channel
                                               selection and needs to be configured only when the input is I/O. */
        unsigned int cmp_sw_enlv_p  : 4;  /**< SW control of ACMP P end, which is configured together with channel
                                               selection and needs to be configured only when the input is I/O. */
        unsigned int reserved_0     : 24;
    } BIT;
} volatile CMP_SW_REG;

/**
  * @brief ACMP registers definition structure.
  */
typedef struct _ACMP_RegStruct {
    CMP_CTRL_REG             CMP_CTRL;    /**< CMP control. Offset address 0x00000008U. */
    CMP_CTRL1_REG            CMP_CTRL1;   /**< CMP control. Offset address 0x0000000CU. */
    CMP_CTRL2_REG            CMP_CTRL2;   /**< CMP control. Offset address 0x00000010U. */
    CMP_QUALI2_REG           CMP_QUALI2;  /**< CMP debounce. Offset address 0x00000014U. */
    CMP_SW_REG               CMP_SW;      /**< CMP Input SW Control. Offset address 0x00000018U. */
} volatile ACMP_RegStruct;

/* Parameter Check------------------------------------------------------------------ */
/**
  * @brief Verify ACMP output polarity configuration.
  * @param polarity: ACMP output polarity @ref ACMP_OutputPolarity
  * @retval true
  * @retval false
  */
static inline bool IsACMPOutputPolarity(ACMP_OutputPolarity polarity)
{
    return ((polarity == ACMP_OUT_NOT_INVERT) || (polarity == ACMP_OUT_INVERT));
}

/**
  * @brief Verify ACMP vin number
  * @param vinNum: ACMP vin positive/negative number @ref ACMP_VinMux
  * @retval true
  * @retval false
  */
static inline bool IsACMPVinNumber(ACMP_VinMux vinNum)
{
    return ((vinNum == ACMP_VIN_MUX0) || (vinNum == ACMP_VIN_MUX1) ||
            (vinNum == ACMP_VIN_MUX2) || (vinNum == ACMP_VIN_MUX3));
}

/**
  * @brief Verify ACMP switch vin number
  * @param swithVinNum: ACMP switch vin positive/negative number @ref ACMP_VinSel
  * @retval true
  * @retval false
  */
static inline bool IsACMPSwitchVinNumber(ACMP_VinSel swithVinNum)
{
    return ((swithVinNum == ACMP_SW_VIN0) || (swithVinNum == ACMP_SW_VIN1) ||
            (swithVinNum == ACMP_SW_VIN2) || (swithVinNum == ACMP_SW_VIN3));
}

/**
  * @brief Verify ACMP Blking
  * @param blkingSrcType: ACMP blking source type @ref ACMP_BlkingSrcType
  * @retval true
  * @retval false
  */
static inline bool IsACMPBlkingSrcType(ACMP_BlkingSrcType blkingSrcType)
{
    return ((blkingSrcType == ACMP_BLKING_SRC_APT) || (blkingSrcType == ACMP_BLKING_SRC_SOFT));
}

/**
  * @brief Verify ACMP apt mask window
  * @param aptSelection: ACMP output source select
  * @retval true
  * @retval false
  */
static inline bool IsACMPAptMaskWindow(unsigned short aptSelection)
{
    return (aptSelection <= ACMP_BLKING_APT_MAX);
}

/**
  * @brief Verify ACMP output result selection
  * @param resultSelection: ACMP output result selection
  * @retval true
  * @retval false
  */
static inline bool IsACMPResultSeletion(ACMP_ResultSelect resultSelection)
{
    return (resultSelection <= ACMP_RESULT_DELAY);
}
/* Direct configuration layer ------------------------------------------------*/
/**
  * @brief Set input switch
  * @param acmpx: ACMP register base address.
  * @param inputP: ACMP inputP selection. @ref ACMP_VinSel
  * @param inputN: ACMP inputN selection. @ref ACMP_VinSel
  * @retval None.
  */
static inline void DCL_ACMP_SetInputSwith(ACMP_RegStruct *acmpx, ACMP_VinSel inputP, ACMP_VinSel inputN)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    ACMP_PARAM_CHECK_NO_RET(inputP >= ACMP_SW_VIN0);
    ACMP_PARAM_CHECK_NO_RET(inputP <= ACMP_SW_VIN3);
    ACMP_PARAM_CHECK_NO_RET(inputN >= ACMP_SW_VIN0);
    ACMP_PARAM_CHECK_NO_RET(inputN <= ACMP_SW_VIN3);
    acmpx->CMP_SW.BIT.cmp_sw_enlv_p = inputP; /* switch P input */
    acmpx->CMP_SW.BIT.cmp_sw_enlv_n = inputN; /* switch N input */
}

/**
  * @brief Set comparator blking active level
  * @param acmpx: ACMP register base address.
  * @param actLevel: Active level. @ref ACMP_BlkingActLevel
  * @retval None.
  */
static inline void DCL_ACMP_SetCmpBlkingActiveLevel(ACMP_RegStruct *acmpx, ACMP_BlkingActLevel actLevel)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    ACMP_PARAM_CHECK_NO_RET(actLevel >= ACMP_BLKING_LOW_ACTIVE);
    ACMP_PARAM_CHECK_NO_RET(actLevel <= ACMP_BLKING_HIGH_ACTIVE);
    acmpx->CMP_CTRL.BIT.cmp_blking_inv_sel = actLevel;
}

/**
  * @brief ACMP output(deshark and synchronize) source.
  * @param acmp: ACMP register base address.
  * @param value: config value. @ref ACMP_OutputSrcSel
  * @retval None.
  */
static inline void DCL_ACMP_SetCmpOutputSrc(ACMP_RegStruct *acmpx, ACMP_OutputSrcSel value)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    ACMP_PARAM_CHECK_NO_RET(value >= ACMP_ORIGINAL_TO_PORT);
    ACMP_PARAM_CHECK_NO_RET(value <= ACMP_SYNC_TO_PORT);
    acmpx->CMP_CTRL.BIT.cmp_qual_en = value & 0x01; /* result select bit0 */
    acmpx->CMP_CTRL.BIT.cmp_sync_sel = (value >> 1) & 0x01; /* result select bit1 */
}

/**
  * @brief  Comparator enable blking function
  * @param acmpx: ACMP register base address.
  * @retval None.
  */
static inline void DCL_ACMP_EnableCmpBlking(ACMP_RegStruct *acmpx)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    acmpx->CMP_CTRL.BIT.cmp_blking_en = BASE_CFG_ENABLE;
}

/**
  * @brief  Comparator disable blking function
  * @param acmpx: ACMP register base address.
  * @retval None.
  */
static inline void DCL_ACMP_DisableCmpBlking(ACMP_RegStruct *acmpx)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    acmpx->CMP_CTRL.BIT.cmp_blking_en = BASE_CFG_DISABLE;
}

/**
  * @brief Set blking from which apt number
  * @param acmpx: ACMP register base address.
  * @param aptNum: APT number.
  * @retval None.
  */
static inline void DCL_ACMP_SetCmpBlkingAptSelect(ACMP_RegStruct *acmpx, unsigned char aptNum)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    ACMP_PARAM_CHECK_NO_RET(aptNum <= ACMP_MAX_APT_NUM);
    acmpx->CMP_CTRL.BIT.cmp_apt_sel = aptNum;
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
    ACMP_PARAM_CHECK_NO_RET(source >= ACMP_BLKING_SRC_APT);
    ACMP_PARAM_CHECK_NO_RET(source <= ACMP_BLKING_SRC_SOFT);
    acmpx->CMP_CTRL.BIT.cmp_blking_sel = source;
}

/**
  * @brief Set software blking valid.
  * @param acmpx: ACMP register base address.
  * @retval None.
  */
static inline void DCL_ACMP_CmpBlkingSoftValid(ACMP_RegStruct *acmpx)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    acmpx->CMP_CTRL.BIT.cmp_blking_cfg = BASE_CFG_DISABLE; /* 0, valid; 1, invalid */
}

/**
  * @brief Set software blking invalid.
  * @param acmpx: ACMP register base address.
  * @retval None.
  */
static inline void DCL_ACMP_CmpBlkingSoftInValid(ACMP_RegStruct *acmpx)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    acmpx->CMP_CTRL.BIT.cmp_blking_cfg = BASE_CFG_ENABLE; /* 0, valid; 1, invalid */
}

/**
  * @brief Enable comparator's digital part.
  * @param acmpx: ACMP register base address.
  * @retval None.
  */
static inline void DCL_ACMP_EnableCmpDigital(ACMP_RegStruct *acmpx)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    acmpx->CMP_CTRL.BIT.cmp_dig_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable comparator's digital part.
  * @param acmpx: ACMP register base address.
  * @retval None.
  */
static inline void DCL_ACMP_DisableCmpDigital(ACMP_RegStruct *acmpx)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    acmpx->CMP_CTRL.BIT.cmp_dig_en = BASE_CFG_DISABLE;
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
    acmpx->CMP_CTRL.BIT.cmp_hy_vol_sel = volSelect;
}

/**
  * @brief Set comparator's output polarity
  * @param acmpx: ACMP register base address.
  * @param polarity: output polarity. @ref ACMP_OutputPolarity
  * @retval None.
  */
static inline void DCL_ACMP_SetCmpOutputPolarity(ACMP_RegStruct *acmpx, ACMP_OutputPolarity polarity)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    ACMP_PARAM_CHECK_NO_RET(polarity >= ACMP_OUT_NOT_INVERT);
    ACMP_PARAM_CHECK_NO_RET(polarity <= ACMP_OUT_INVERT);
    acmpx->CMP_CTRL.BIT.cmp_rslt_inv = polarity;
}

/**
  * @brief enable comparator's analog part.
  * @param acmpx: ACMP register base address.
  * @retval None.
  */
static inline void DCL_ACMP_EnableCmpAnalog(ACMP_RegStruct *acmpx)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    acmpx->CMP_CTRL.BIT.cmp_ana_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable comparator's analog part.
  * @param acmpx: ACMP register base address.
  * @retval None.
  */
static inline void DCL_ACMP_DisableCmpAnalog(ACMP_RegStruct *acmpx)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    acmpx->CMP_CTRL.BIT.cmp_ana_en = BASE_CFG_DISABLE;
}

/**
  * @brief Set comparator input muxtex P
  * @param acmpx: ACMP register base address.
  * @param muxValue: input P select value. @ref ACMP_VinMux
  * @retval None.
  */
static inline void DCL_ACMP_SetMuxP(ACMP_RegStruct *acmpx, ACMP_VinMux muxValue)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    ACMP_PARAM_CHECK_NO_RET(muxValue >= ACMP_VIN_MUX0);
    ACMP_PARAM_CHECK_NO_RET(muxValue <= ACMP_VIN_MUX3);
    acmpx->CMP_CTRL1.BIT.cmp_mux_p = muxValue;
}

/**
  * @brief Set comparator input muxtex N
  * @param acmpx: ACMP register base address.
  * @param muxValue: input N value. @ref ACMP_VinMux
  * @retval None.
  */
static inline void DCL_ACMP_SetMuxN(ACMP_RegStruct *acmpx, ACMP_VinMux muxValue)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    ACMP_PARAM_CHECK_NO_RET(muxValue >= ACMP_VIN_MUX0);
    ACMP_PARAM_CHECK_NO_RET(muxValue <= ACMP_VIN_MUX3);
    acmpx->CMP_CTRL1.BIT.cmp_mux_n = muxValue;
}

/**
  * @brief Reading compare result after blking.
  * @param acmpx: ACMP register base address.
  * @retval Blking result.
  */
static inline unsigned int DCL_ACMP_GetCmpOutValueAfterBlking(ACMP_RegStruct *acmpx)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    return acmpx->CMP_CTRL2.BIT.cmp_blking_rslt;
}

/**
  * @brief Reading compare result after deshark.
  * @param acmpx: ACMP register base address.
  * @retval deshark result.
  */
static inline unsigned int DCL_ACMP_GetCmpOutValueAfterDeshark(ACMP_RegStruct *acmpx)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    return acmpx->CMP_CTRL2.BIT.cmp_qual_rslt;
}

/**
  * @brief Reading original compare result
  * @param acmpx: ACMP register base address.
  * @retval original result.
  */
static inline unsigned int DCL_ACMP_GetCmpOutValueOriginal(ACMP_RegStruct *acmpx)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    return acmpx->CMP_CTRL2.BIT.cmp_ana_rslt;
}

/**
  * @brief Set deshark step by clock.
  * @param acmpx: ACMP register base address.
  * @param clkTimes: Times of clock.
  * @retval None.
  */
static inline void DCL_ACMP_SetDesharkStepByClock(ACMP_RegStruct *acmpx, unsigned short clkTimes)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    ACMP_PARAM_CHECK_NO_RET(clkTimes <= ACMP_DESHARK_BY_CLK_MAX);
    acmpx->CMP_QUALI2.BIT.cmp_qual_step = clkTimes;
}

/**
  * @brief Set deshark step by compare times
  * @param acmpx: ACMP register base address.
  * @param cmpTimes: Times of compare.
  * @retval None.
  */
static inline void DCL_ACMP_SetDesharkStepByCmpTimes(ACMP_RegStruct *acmpx, unsigned short cmpTimes)
{
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpx));
    ACMP_PARAM_CHECK_NO_RET(cmpTimes <= ACMP_DESHARK_BY_CMP_MAX);
    acmpx->CMP_QUALI2.BIT.cmp_qual_sel = cmpTimes;
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