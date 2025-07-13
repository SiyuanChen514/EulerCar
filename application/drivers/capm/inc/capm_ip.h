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
  * @file      capm_ip.h
  * @author    MCU Driver Team
  * @brief     CAPM DCL level module driver.
  * @details   This file provides DCL functions to manage CAPM and Definition of
  *            specific parameters.
  *             + Definition of CAPM configuration parameters.
  *             + CAPM register mapping structure.
  *             + Direct configuration layer interface.
  */
#ifndef McuMagicTag_CAPM_IP_H
#define McuMagicTag_CAPM_IP_H

#include "baseinc.h"
#include "baseaddr.h"

#ifdef CAPM_PARAM_CHECK
#define CAPM_ASSERT_PARAM         BASE_FUNC_ASSERT_PARAM
#define CAPM_PARAM_CHECK_NO_RET   BASE_FUNC_PARAMCHECK_NO_RET
#define CAPM_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define CAPM_ASSERT_PARAM(para)               ((void)0U)
#define CAPM_PARAM_CHECK_NO_RET(para)         ((void)0U)
#define CAPM_PARAM_CHECK_WITH_RET(param, ret) ((void)0U)
#endif

/**
  * @addtogroup CAPM
  * @{
  */

/**
  * @defgroup CAPM_IP CAPM_IP
  * @brief CAPM_IP: capm_v1.
  * @{
  */

/**
 * @defgroup CAPM_Param_Def CAPM Parameters Definition
 * @brief Definition of CAPM configuration parameters
 * @{
 */

#define CAPM_IP_VER_MASK        0x0F000000U
#define CAPM_NEXT_LOAD_REG_MASK 0x00000003U
#define CAPM_INTERRUPT_MASK     0x0000001FU
#define CAPM_MAX_FILTER_VALUE   16
#define CAPM_MAX_PRESCALE       127
#define CAPM_MAX_CAP_REG_NUM    4
#define CAPM_MAX_FILTER_LEVEL   0x00001FFFU
#define CAPM_BIT_SHIFT_TWO      2
#define CAPM_MAX_INTERRUPT_NUMBER 8

#define CAPM0_BASEADDR CAPM0
#define CAPM1_BASEADDR CAPM1
#define CAPM2_BASEADDR CAPM2

/**
 * @brief EAR count types.
 * @details Count type:
 *          + CAPM_COUNT_NONE -- EAR do not count
 *          + CAPM_COUNT_RISING_EDGE -- EAR counting at rising edge
 *          + CAPM_COUNT_FALLING_EDGE -- EAR counting at falling edge
 *          + CAPM_COUNT_DOUBLE_EDGE -- EAR counting at rising edge or edge
 */
typedef enum {
    CAPM_COUNT_NONE = 0x00000000U,
    CAPM_COUNT_RISING_EDGE = 0x00000001U,
    CAPM_COUNT_FALLING_EDGE = 0x00000002U,
    CAPM_COUNT_DOUBLE_EDGE = 0x00000003U,
} CAPM_CountType;

/**
 * @brief Interrupt types.
 * @details Type:
 *          +  CAPM_REG1CAP -- ECR0 interrupt
 *          +  CAPM_REG2CAP -- ECR1 interrupt
 *          +  CAPM_REG3CAP -- ECR2 interrupt
 *          +  CAPM_REG4CAP -- ECR3 interrupt
 *          +  CAPM_TSROVF  -- TSR register overflow interrupt
 *          +  CAPM_ECROVF  -- ECR register overflow interrupt
 *          +  CAPM_EARCMPMATCH -- EAR compare match interrupt
 *          +  CAPM_EAROVF -- EAR register overflow interrupt
 *          +  CAPM_DMAREQOVF -- DMA require overflow interrupt
 */
typedef enum {
    CAPM_REG1CAP = 0x00000001U,
    CAPM_REG2CAP = 0x00000002U,
    CAPM_REG3CAP = 0x00000004U,
    CAPM_REG4CAP = 0x00000008U,
    CAPM_TSROVF  = 0x00000010U,
    CAPM_ECROVF  = 0x00000020U,
    CAPM_EARCMPMATCH = 0x00000040U,
    CAPM_EAROVF = 0x00000080U,
    CAPM_DMAREQOVF = 0x00000100U,
} CAPM_Interrupt;

/**
 * @brief ECR number to be used.
 */
typedef enum {
    CAPM_EVT0 = 0x00000000U,
    CAPM_EVT1 = 0x00000001U,
    CAPM_EVT2 = 0x00000002U,
    CAPM_EVT3 = 0x00000003U,
} CampConfigCapRegNum;

/**
 * @brief CAPM capture mode.
 * @details Capture mode:
 *          +  CAPM_CONTINUECAP -- continue cap
 *          +  CAPM_ONESHOTCAP -- one-shot cap
 */
typedef enum {
    CAPM_CONTINUECAP = 0x00000000U, /**< continue cap */
    CAPM_ONESHOTCAP = 0x00000001U,  /**< one-shot cap */
} CAPM_CapMode;

/**
 * @brief CAPM capture edge.
 * @details Capture edge:
 *          +  CAPM_FALLING_EDGE -- capture falling edge
 *          +  CAPM_RISING_EDGE -- capture rising edge
 */
typedef enum {
    CAPM_FALLING_EDGE = 0x00000000U,
    CAPM_RISING_EDGE = 0x00000001U,
} CAPM_POLAR;

/**
 * @brief CAPM input source selection.
 * @details Capture edge:
 *          +  CAPM_INPUT -- CAPMx_IN
 *          +  CAPM_NONE -- No input source
 */
typedef enum {
    CAPM_INPUT = 0x00000000U,
    CAPM_NONE = 0x00000001U,
} CAPM_InputSrc;

/**
 * @brief CAPM sync input source selection.
 * @details Capture edge:
 *          +  CAPM_SYNC_SRC_NONE  -- source none
 *          +  CAPM_SYNC_SRC_APT0  -- source apt0
 *          +  CAPM_SYNC_SRC_APT1  -- source apt1
 *          +  CAPM_SYNC_SRC_APT2  -- source apt2
 *          +  CAPM_SYNC_SRC_APT3  -- source apt3
 *          +  CAPM_SYNC_SRC_APT4  -- source apt4
 *          +  CAPM_SYNC_SRC_APT5  -- source apt5
 *          +  CAPM_SYNC_SRC_APT6  -- source apt6
 *          +  CAPM_SYNC_SRC_APT7  -- source apt7
 *          +  CAPM_SYNC_SRC_APT8  -- source apt8
 */
typedef enum {
    CAPM_SYNC_SRC_NONE = 0x00000000U,
    CAPM_SYNC_SRC_APT0 = 0x00000001U,
    CAPM_SYNC_SRC_APT1 = 0x00000002U,
    CAPM_SYNC_SRC_APT2 = 0x00000003U,
    CAPM_SYNC_SRC_APT3 = 0x00000004U,
    CAPM_SYNC_SRC_APT4 = 0x00000005U,
    CAPM_SYNC_SRC_APT5 = 0x00000006U,
    CAPM_SYNC_SRC_APT6 = 0x00000007U,
    CAPM_SYNC_SRC_APT7 = 0x00000008U,
    CAPM_SYNC_SRC_APT8 = 0x00000009U,
} CAPM_SyncSrc;

/**
  * @}
  */

/**
  * @defgroup CAPM_REG_Definition CAPM Register Structure.
  * @brief CAPM Register Structure Definition.
  * @{
  */

/**
  * @brief CAPM revision information registers structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved1 : 24;
        unsigned int revision : 4;            /**< IP version number. */
        unsigned int reserved0 : 4;
    } BIT;
} volatile REV_INFO_REG;

/**
  * @brief CAPM time-stamp divider registers structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int tscnt_div : 16;           /**< Counter division. */
        unsigned int reserved : 16;
    } BIT;
} volatile TSR_DIV_REG;

/**
  * @brief CAPM edge amount registers structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ear : 16;                  /**< Edge count value. */
        unsigned int reserved : 16;
    } BIT;
} volatile EAR_REG;

/**
  * @brief EAR compare value registers structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int earcmp : 16;               /**< Edge count compare value. */
        unsigned int reserved : 16;
    } BIT;
} volatile EAR_CMP_REG;

/**
  * @brief Event capture sequence registers structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int nxtldecr : 2;              /**< Read next loaded ECR. */
        unsigned int crt_edge : 2;              /**< Current input signal level. */
        unsigned int reserved : 28;
    } BIT;
} volatile ECSEQR_REG;

/**
  * @brief Filter control registers structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ft_en : 1;               /**< Filter function enable. */
        unsigned int ft_lev : 13;             /**< Filter level. */
        unsigned int reserved : 18;
    } BIT;
} volatile FTCR_REG;

/**
  * @brief CCR0 registers structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int evt0pol : 1;       /**< Event0 capture edge selection. */
        unsigned int evt0rst : 1;       /**< Event0 reset TSR. */
        unsigned int evt1pol : 1;       /**< Event0 capture edge selection. */
        unsigned int evt1rst : 1;       /**< Event0 reset TSR. */
        unsigned int evt2pol : 1;       /**< Event1 capture edge selection. */
        unsigned int evt2rst : 1;       /**< Event1 reset TSR. */
        unsigned int evt3pol : 1;       /**< Event2 capture edge selection. */
        unsigned int evt3rst : 1;       /**< Event2 reset TSR. */
        unsigned int ecrlden : 1;       /**< Capture enable. */
        unsigned int dmaevt_sel : 2;    /**< DMA request event selection. */
        unsigned int psc : 8;           /**< Pre-division coefficient of the input signal. */
        unsigned int cnt_edge_sel : 2;  /**< Edge type selection of edge count.*/
        unsigned int reserved : 11;
    } BIT;
} volatile CCR0_REG;

/**
  * @brief CCR1 registers structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0 : 3;
        unsigned int emu_stop_en : 1;     /**< Emulation stop TSR enable. */
        unsigned int reserved1 : 28;
    } BIT;
} volatile CCR1_REG;

/**
  * @brief CCR2 registers structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cap_mode : 1;        /**< Capture mode selection. */
        unsigned int seq_stop : 2;        /**< End of capture sequence/Boundary of circulation. */
        unsigned int reserved : 29;
    } BIT;
} volatile CCR2_REG;

/**
  * @brief CAPM interrupt enable registers structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int evt0_en : 1;             /**< Event0 interrupt enable. */
        unsigned int evt1_en : 1;             /**< Event1 interrupt enable. */
        unsigned int evt2_en : 1;             /**< Event2 interrupt enable. */
        unsigned int evt3_en : 1;             /**< Event3 interrupt enable. */
        unsigned int tsr_ovf_en : 1;          /**< TSR overflow interrupt enable. */
        unsigned int ecr_ovf_en : 1;          /**< Capture overflow interrupt enable. */
        unsigned int earcmp_match_en : 1;     /**< Edge count compare match interrupt enable. */
        unsigned int ear_ovf_en : 1;          /**< Edge count overflow interrupt enable. */
        unsigned int dmareq_ovf_en : 1;       /**< DMA request overflow interrupt enable. */
        unsigned int reserved : 23;
    } BIT;
} volatile INTENR_REG;

/**
  * @brief CAPM initial interrupt registers structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int evt0_raw : 1;            /**< Event0 initial interrupt. */
        unsigned int evt1_raw : 1;            /**< Event1 initial interrupt. */
        unsigned int evt2_raw : 1;            /**< Event2 initial interrupt. */
        unsigned int evt3_raw : 1;            /**< Event3 initial interrupt. */
        unsigned int tsr_ovf_raw : 1;         /**< TSR overflow initial interrupt. */
        unsigned int ecr_ovf_raw : 1;         /**< Capture overflow initial interrupt. */
        unsigned int earcmp_match_raw : 1;    /**< Edge count compare match initial interrupt. */
        unsigned int ear_ovf_raw : 1;         /**< Edge count overflow initial interrupt. */
        unsigned int dmareq_ovf_raw : 1;      /**< DMA request overflow initial interrupt. */
        unsigned int reserved : 23;
    } BIT;
} volatile INTRAWR_REG;

/**
  * @brief CAPM interrupt injection registers structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int evt0_inj : 1;          /**< Event0 interrupt injection. */
        unsigned int evt1_inj : 1;          /**< Event1 interrupt injection. */
        unsigned int evt2_inj : 1;          /**< Event2 interrupt injection. */
        unsigned int evt3_inj : 1;          /**< Event3 interrupt injection. */
        unsigned int tsr_ovf_inj : 1;       /**< TSR overflow interrupt injection. */
        unsigned int ecr_ovf_inj : 1;       /**< Capture overflow interrupt injection. */
        unsigned int earcmp_match_inj : 1;  /**< Edge count compare match interrupt injection. */
        unsigned int ear_ovf_inj : 1;       /**< Edge count overflow interrupt injection. */
        unsigned int dmareq_ovf_inj : 1;    /**< DMA request overflow interrupt injection. */
        unsigned int reserved : 23;
    } BIT;
} volatile INTINJR_REG;

/**
  * @brief CAPM interrupt status registers structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int evt0_int : 1;            /**< Event0 interrupt status. */
        unsigned int evt1_int : 1;            /**< Event1 interrupt status. */
        unsigned int evt2_int : 1;            /**< Event2 interrupt status. */
        unsigned int evt3_int : 1;            /**< Event3 interrupt status. */
        unsigned int tsr_ovf_int : 1;         /**< TSR overflow interrupt status. */
        unsigned int ecr_ovf_int : 1;         /**< Capture overflow interrupt status. */
        unsigned int earcmp_match_int : 1;    /**< Edge count compare match interrupt status. */
        unsigned int ear_ovf_int : 1;         /**< Edge count overflow interrupt status. */
        unsigned int dmareq_ovf_int : 1;      /**< DMA request overflow interrupt status. */
        unsigned int reserved : 23;
    } BIT;
} volatile INTFLGR_REG;

/**
  * @brief Event interrupt
  */
typedef enum {
    CAPM_INTREG1CAP = 0x00000000U,
    CAPM_INTREG2CAP = 0x00000001U,
    CAPM_INTREG3CAP = 0x00000002U,
    CAPM_INTREG4CAP = 0x00000003U,
    CAPM_INTTSROVF = 0x00000004U,
    CAPM_INTECROVF = 0x00000005U,
    CAPM_INTEARCMPMATCH = 0x00000006U,
    CAPM_INTEAROVF = 0x00000007U,
    CAPM_INTDMAREQOVF = 0x00000008U,
} CAPM_IntEvent;

/**
  * @brief CAPM Interrupt callback functions.
  *
  */
typedef void (*EvtCallbackType)(void *handle, CAPM_IntEvent intValue);
typedef struct {
    EvtCallbackType EvtFinishCallback;   /**< event finish callback function. */
    void (*DmaFinishCallback)(void *handle);    /**< DMA finish callback function. */
    void (*DmaErrorCallback)(void *handle);     /**< DMA error callback function. */
} CAPM_UserCallBack;

/**
  * @brief CAPM extend handle.
  */
typedef struct _CAPM_ExtendeHandle {
} CAPM_ExtendHandle;

/**
  * @brief CAPM registers definition structure.
  */
typedef struct {
    REV_INFO_REG  REV_INFO;       /**< CAPM revision information register, offset address: 0x0000. */
    unsigned int tsr;             /**< CAPM time-stamp register, offset address: 0x0004. */
    TSR_DIV_REG TSR_DIV;          /**< CAPM time-stamp divider register, offset address: 0x0008. */
    EAR_REG EAR;                  /**< CAPM edge amount register, offset address: 0x000C. */
    EAR_CMP_REG EAR_CMP;          /**< EAR compare value register, offset address: 0x0010. */
    unsigned int  SYNC_PHS;       /**< Sync phase, offset address: 0x0014. */
    unsigned int  ECR0;           /**< Event0 capture register, offset address: 0x0018. */
    unsigned int  ECR1;           /**< Event1 capture register, offset address: 0x001C. */
    unsigned int  ECR2;           /**< Event2 capture register, offset address: 0x0020. */
    unsigned int  ECR3;           /**< Event3 capture register, offset address: 0x0024. */
    ECSEQR_REG ECSEQR;            /**< Event capture sequence register, offset address: 0x0028. */
    FTCR_REG  FTCR;               /**< Filter control register, offset address: 0x002C. */
    CCR0_REG CCR0;                /**< CCR0 register, offset address: 0x0030. */
    CCR1_REG CCR1;                /**< CCR1 register, offset address: 0x0034. */
    CCR2_REG CCR2;                /**< CCR2 register, offset address: 0x0038. */
    unsigned int reserve;
    INTENR_REG  INTENR;           /**< CAPM interrupt enable register, offset address: 0x0040. */
    INTRAWR_REG INTRAWR;          /**< CAPM initial interrupt register, offset address: 0x0044. */
    INTINJR_REG INTINJR;          /**< CAPM interrupt injection register, offset address: 0x0048. */
    INTFLGR_REG INTFLGR;          /**< CAPM interrupt status register, offset address: 0x004C. */
} volatile CAPM_RegStruct;

/**
  * @brief Capture module general control registers structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int restart_capm0 : 1;     /**< CAPM0 start a new single round capture. */
        unsigned int restart_capm1 : 1;     /**< CAPM1 start a new single round capture. */
        unsigned int restart_capm2 : 1;     /**< CAPM2 start a new single round capture. */
        unsigned int reserved1 : 5;
        unsigned int tsr_stop_capm0 : 1;    /**< CAPM0 TSR stop count enable. */
        unsigned int tsr_stop_capm1 : 1;    /**< CAPM1 TSR stop count enable. */
        unsigned int tsr_stop_capm2 : 1;    /**< CAPM2 TSR stop count enable. */
        unsigned int reserved2 : 5;
        unsigned int stat_rst_capm0 : 1;    /**< CAPM0 work state reset. */
        unsigned int stat_rst_capm1 : 1;    /**< CAPM1 work state reset. */
        unsigned int stat_rst_capm2 : 1;    /**< CAPM2 work state reset. */
        unsigned int reserve3 : 5;
        unsigned int sync_sw_capm0 : 1;     /**< Triggle CAPM0 sync, TSR reset, capture sequence reset. */
        unsigned int sync_sw_capm1 : 1;     /**< Triggle CAPM1 sync, TSR reset, capture sequence reset. */
        unsigned int sync_sw_capm2 : 1;     /**< Triggle CAPM2 sync, TSR reset, capture sequence reset. */
        unsigned int reserve4 : 5;
        } BIT;
} volatile CAPM_GENE_CR_REG;

/**
  * @brief Sync selection register for CAPM0 structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int capm0_sync_sel : 4;      /**< CAPM0 hardware sync source selection. */
        unsigned int capm0_synci_en : 1;      /**< CAPM0 sync enable. */
        unsigned int reserved : 27;
    }BIT;
} volatile SYNC_SELR0_REG;

/**
  * @brief Sync selection register for CAPM1 structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int capm1_sync_sel : 4;      /**< CAPM1 hardware sync source selection. */
        unsigned int capm1_synci_en : 1;      /**< CAPM1 sync enable. */
        unsigned int reserved : 27;
    }BIT;
} volatile SYNC_SELR1_REG;

/**
  * @brief Sync selection register for CAPM2 structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int capm2_sync_sel : 4;      /**< CAPM2 hardware sync source selection. */
        unsigned int capm2_synci_en : 1;      /**< CAPM2 sync enable. */
        unsigned int reserved : 27;
    }BIT;
} volatile SYNC_SELR2_REG;

/**
  * @brief Input source selection register for CAPM0 structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int capm0_in_sel : 1;      /**< CAPM0 input source selection. */
        unsigned int reserved : 31;
    }BIT;
} volatile INPUT_SELR0_REG;

/**
  * @brief Input source selection register for CAPM1 structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int capm1_in_sel : 1;      /**< CAPM1 input source selection. */
        unsigned int reserved : 31;
    }BIT;
} volatile INPUT_SELR1_REG;

/**
  * @brief Input source selection register for CAPM2 structure definition
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int capm2_in_sel : 1;      /**< CAPM2 input source selection. */
        unsigned int reserved : 31;
    }BIT;
} volatile INPUT_SELR2_REG;

/**
  * @brief Define the CAPM common register struct.
  */
typedef struct {
    REV_INFO_REG REV_INFO;              /**< Revision information, offset address: 0x0000. */
    CAPM_GENE_CR_REG CAPM_GENE_CR;      /**< Capture module general control register, offset address: 0x0004. */
    SYNC_SELR0_REG SYNC_SELR0;          /**< Sync selection register for CAPM0, offset address: 0x0008. */
    SYNC_SELR1_REG SYNC_SELR1;          /**< Sync selection register for CAPM1, offset address: 0x000C. */
    SYNC_SELR2_REG SYNC_SELR2;          /**< Sync selection register for CAPM2, offset address: 0x0010. */
    unsigned char reserved[20];
    INPUT_SELR0_REG INPUT_SELR0;        /**< Input source selection register for CAPM0, offset address: 0x0028. */
    INPUT_SELR1_REG INPUT_SELR1;        /**< Input source selection register for CAPM1, offset address: 0x002C. */
    INPUT_SELR2_REG INPUT_SELR2;        /**< Input source selection register for CAPM2, offset address: 0x0030. */
} volatile CAPM_COMM_RegStruct;


/**
  * @brief Get CAPM IP's version.
  * @param capmx: CAPM register base address.
  * @retval CAPM IP's version.
  */
static inline unsigned int DCL_CAPM_GetIPVer(const CAPM_RegStruct *capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    return (capmx->REV_INFO.reg) & CAPM_IP_VER_MASK;
}

/**
  * @brief Get TSR value.
  * @param capmx: CAPM register base address.
  * @retval TSR value.
  */
static inline unsigned int DCL_CAPM_GetTSR(const CAPM_RegStruct *capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    return capmx->tsr;
}
/**
  * @brief Set TSR divide value.
  * @param capmx: CAPM register base address.
  * @param divValue: Divide value. Range: 0~65535
  * @retval None.
  */
static inline void DCL_CAPM_SetTSRDiv(CAPM_RegStruct * const capmx, unsigned short divValue)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->TSR_DIV.BIT.tscnt_div = divValue;
    return;
}

/**
  * @brief Get EAR value.
  * @param capmx: CAPM register base address.
  * @retval EAR value.
  */
static inline unsigned int DCL_CAPM_GetEar(const CAPM_RegStruct *capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    return capmx->EAR.BIT.ear;
}

/**
  * @brief Get EAR_CMP value.
  * @param capmx: CAPM register base address.
  * @retval EAR_CMP value.
  */
static inline unsigned int DCL_CAPM_GetEarCmp(const CAPM_RegStruct *capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    return capmx->EAR_CMP.BIT.earcmp;
}

/**
  * @brief Set sync phase value.
  * @param capmx: CAPM register base address.
  * @param syncPhs: Phase value. Range: 0~0xFFFF FFFF.
  * @retval None.
  */
static inline void DCL_CAPM_SetSyncPhase(CAPM_RegStruct * const capmx, unsigned int syncPhs)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->SYNC_PHS = syncPhs;
    return;
}

/**
  * @brief Get sync phase value.
  * @param capmx: CAPM register base address.
  * @retval Phase value.
  */
static inline unsigned int DCL_CAPM_GetSyncPhase(const CAPM_RegStruct *capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    return capmx->SYNC_PHS;
}

/**
  * @brief Get ECR0 value.
  * @param capmx: CAPM register base address.
  * @retval ECR0 value.
  */
static inline unsigned int DCL_CAPM_GetECR0(const CAPM_RegStruct *capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    return capmx->ECR0;
}

/**
  * @brief Get ECR1 value.
  * @param capmx: CAPM register base address.
  * @retval ECR1 value.
  */
static inline unsigned int DCL_CAPM_GetECR1(const CAPM_RegStruct *capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    return capmx->ECR1;
}

/**
  * @brief Get ECR2 value.
  * @param capmx: CAPM register base address.
  * @retval ECR2 value.
  */
static inline unsigned int DCL_CAPM_GetECR2(const CAPM_RegStruct *capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    return capmx->ECR2;
}

/**
  * @brief Get ECR3 value.
  * @param capmx: CAPM register base address.
  * @retval ECR3 value.
  */
static inline unsigned int DCL_CAPM_GetECR3(const CAPM_RegStruct *capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    return capmx->ECR3;
}

/**
  * @brief Get current signal level.
  * @param capmx: CAPM register base address.
  * @retval Signal level.
  */
static inline unsigned char DCL_CAPM_GetCRTEdge(const CAPM_RegStruct *capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    return (capmx->ECSEQR.BIT.crt_edge);
}

/**
  * @brief Get next ECR number.
  * @param capmx: CAPM register base address.
  * @retval Next ECR number.
  */
static inline unsigned char DCL_CAPM_GetNextECRNum(const CAPM_RegStruct *capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    return (capmx->ECSEQR.BIT.nxtldecr);
}

/**
  * @brief Set capture rising edge register.
  * @param capmx: CAPM register base address.
  * @param capReg: Capture rising edge register.
  *                Input argument value: 0(CapReg 1),1(CapReg 2),2(CapReg 3),3(CapReg 4).Register set value:1,4,16,64.
  * @retval None.
  */
static inline void DCL_CAPM_RisingCap(CAPM_RegStruct * const capmx, unsigned int capReg)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    if (capReg > CAPM_MAX_CAP_REG_NUM) {
        return;
    }
    capmx->CCR0.reg |= (unsigned int)((1 << capReg) * (1 << capReg));
    return;
}

/**
  * @brief Set capture falling edge register.
  * @param capmx: CAPM register base address.
  * @param capReg: Capture falling edge register.
  *                Input argument value:0(CapReg 1),1(CapReg 2),2(CapReg 3),3(CapReg 4).Register set value:1,4,16,64.
  * @retval None.
  */
static inline void DCL_CAPM_FallingCap(CAPM_RegStruct * const capmx, unsigned int capReg)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    if (capReg > CAPM_MAX_CAP_REG_NUM) {
        return;
    }
    capmx->CCR0.reg &= (~(unsigned int)((1 << capReg) * (1 << capReg)));
    return;
}

/**
  * @brief Enable capture register reset TSR function.
  * @param capmx: CAPM register base address.
  * @param capReg: Reset TSR's capture register.
  *                Input argument value:0(CapReg 1),1(CapReg 2),2(CapReg 3),3(CapReg 4).Register set value:2,8,32,128.
  * @retval None.
  */
static inline void DCL_CAPM_EnableCapReset(CAPM_RegStruct * const capmx, unsigned int capReg)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    if (capReg > CAPM_MAX_CAP_REG_NUM) {
        return;
    }
    capmx->CCR0.reg |= (unsigned int)((CAPM_BIT_SHIFT_TWO * (1 << capReg) * (1 << capReg)));
    return;
}

/**
  * @brief Disable capture register reset TSR function.
  * @param capmx: CAPM register base address.
  * @param capReg: Non-reset TSR's capture register.
  *                Input argument value:0(CapReg 1),1(CapReg 2),2(CapReg 3),3(CapReg 4).Register set value:2,8,32,128.
  * @retval None.
  */
static inline void DCL_CAPM_DisableCapReset(CAPM_RegStruct * const capmx, unsigned int capReg)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    if (capReg > CAPM_MAX_CAP_REG_NUM) {
        return;
    }
    capmx->CCR0.reg &= ~(unsigned int)(CAPM_BIT_SHIFT_TWO * (1 << capReg) * (1 << capReg));
    return;
}

/**
  * @brief Set ECR0 capture falling edge.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_ECR0FallingCap(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR0.BIT.evt0pol = CAPM_FALLING_EDGE;
    return;
}

/**
  * @brief Set ECR0 capture rising edge.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_ECR0RisingCap(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR0.BIT.evt0pol = CAPM_RISING_EDGE;
    return;
}

/**
  * @brief Enable ECR0 reset after each capture action.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_EnableECR0CapReset(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR0.BIT.evt0rst = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Disable ECR0 reset after each capture action.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableECR0CapReset(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR0.BIT.evt0rst = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Set ECR1 capture falling edge.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_ECR1FallingCap(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR0.BIT.evt1pol = CAPM_FALLING_EDGE;
    return;
}

/**
  * @brief Set ECR1 capture rising edge.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_ECR1RisingCap(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR0.BIT.evt1pol = CAPM_RISING_EDGE;
    return;
}

/**
  * @brief Enable ECR1 reset after each capture action.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_EnableECR1CapReset(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR0.BIT.evt1rst = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Disable ECR1 reset after each capture action.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableECR1CapReset(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR0.BIT.evt1rst = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Set ECR2 capture falling edge.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_ECR2FallingCap(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR0.BIT.evt2pol = CAPM_FALLING_EDGE;
    return;
}

/**
  * @brief Set ECR2 capture rising edge.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_ECR2RisingCap(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR0.BIT.evt2pol = CAPM_RISING_EDGE;
    return;
}

/**
  * @brief Enable ECR2 reset after each capture action.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_EnableECR2CapReset(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR0.BIT.evt2rst = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Disable ECR2 reset after each capture action.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableECR2CapReset(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR0.BIT.evt2rst = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Set ECR2 capture Falling edge.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_ECR3FallingCap(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR0.BIT.evt3pol = CAPM_FALLING_EDGE;
    return;
}

/**
  * @brief Set ECR2 capture Rising edge.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_ECR3RisingCap(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR0.BIT.evt3pol = CAPM_RISING_EDGE;
    return;
}

/**
  * @brief Enable ECR3 after each capture action.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_EnableECR3CapReset(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR0.BIT.evt3rst = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Disable ECR3 after each capture action.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableECR3CapReset(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR0.BIT.evt3rst = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Enable capture register load.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_EnableCapRegLoad(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR0.BIT.ecrlden = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Disable capture register load.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableCapRegLoad(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR0.BIT.ecrlden = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Set the capture register's number which trggle DMA interrupt.
  * @param capmx: CAPM register base address.
  * @param capNum: Capture register number.
  * @retval None.
  */
static inline void DCL_CAPM_SetDMATriggleReg(CAPM_RegStruct * const capmx, CampConfigCapRegNum capNum)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    CAPM_PARAM_CHECK_NO_RET(capNum >= 0);
    CAPM_PARAM_CHECK_NO_RET(capNum < CAPM_MAX_CAP_REG_NUM);
    capmx->CCR0.BIT.dmaevt_sel = capNum;
    return;
}

/**
  * @brief Set prescale value.
  * @param base: CAPM register base address.
  * @param preScale PreScale value. Range: 0, 1, 2, 3 ... 127.
  * @retval None.
  */
static inline void DCL_CAPM_SetPreScale(CAPM_RegStruct * const capmx, unsigned short preScale)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    CAPM_PARAM_CHECK_NO_RET(preScale <= CAPM_MAX_PRESCALE);
    capmx->CCR0.BIT.psc = preScale;
    return;
}

/**
  * @brief Set count edge type.
  * @param capmx: CAPM register base address.
  * @param countType: Count edge type.
  * @retval None.
  */
static inline void DCL_CAPM_SetCountType(CAPM_RegStruct * const capmx, CAPM_CountType countType)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR0.BIT.cnt_edge_sel = countType;
    return;
}

/**
  * @brief Set filer value.
  * @param capmx: CAPM register base address.
  * @param filterValue: Filter value. Range: 0 ~ 8191.
  * @retval None.
  */
static inline void DCL_CAPM_SetFilterLevel(CAPM_RegStruct * const capmx, unsigned short filterValue)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->FTCR.BIT.ft_lev = filterValue;
    return;
}

/**
  * @brief Get filer value.
  * @param capmx: CAPM register base address.
  * @retval Filer value.
  */
static inline unsigned int DCL_CAPM_GetFilterLevel(const CAPM_RegStruct *capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    return capmx->FTCR.BIT.ft_lev;
}

/**
  * @brief Enable input filter.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_EnableFilter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->FTCR.BIT.ft_en = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Disable input filter.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableFilter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->FTCR.BIT.ft_en = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Restart CAPM0 one-shot capture.
  * @param capmComm: CAPM COMM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_RestartOneShotCap0(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->CAPM_GENE_CR.BIT.restart_capm0 = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Restart CAPM1 one-shot capture.
  * @param capmComm: CAPM COMM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_RestartOneShotCap1(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->CAPM_GENE_CR.BIT.restart_capm1 = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Restart CAPM2 one-shot capture.
  * @param capmComm: CAPM COMM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_RestartOneShotCap2(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->CAPM_GENE_CR.BIT.restart_capm2 = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Suspend capm0 TSR count.
  * @param capmComm: CAPM COMM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_SuspendTSRCount0(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->CAPM_GENE_CR.BIT.tsr_stop_capm0 = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Resume capm0 TSR counter.
  * @param capmx: CAPM COMM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_ResumeTSRCount0(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->CAPM_GENE_CR.BIT.tsr_stop_capm0 = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Suspend capm1 TSR count.
  * @param capmComm: CAPM COMM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_SuspendTSRCount1(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->CAPM_GENE_CR.BIT.tsr_stop_capm1 = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Resume capm1 TSR counter.
  * @param capmx: CAPM COMM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_ResumeTSRCount1(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->CAPM_GENE_CR.BIT.tsr_stop_capm1 = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Suspend capm2 TSR count.
  * @param capmComm: CAPM COMM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_SuspendTSRCount2(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->CAPM_GENE_CR.BIT.tsr_stop_capm2 = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Resume capm0 TSR counter.
  * @param capmx: CAPM COMM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_ResumeTSRCount2(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->CAPM_GENE_CR.BIT.tsr_stop_capm2 = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Reset capm0 TSR value.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_ResetTSRCount0(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->CAPM_GENE_CR.BIT.stat_rst_capm0 = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Reset capm1 TSR value.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_ResetTSRCount1(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->CAPM_GENE_CR.BIT.stat_rst_capm1 = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Reset capm2 TSR value.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_ResetTSRCount2(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->CAPM_GENE_CR.BIT.stat_rst_capm2 = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Set capture mode.
  * @param capmx: CAPM register base address.
  * @param capMode: Capture mode.
  * @retval None.
  */
static inline void DCL_CAPM_SetCapMode(CAPM_RegStruct * const capmx, CAPM_CapMode capMode)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    CAPM_PARAM_CHECK_NO_RET(capMode == CAPM_CONTINUECAP || capMode == CAPM_ONESHOTCAP);
    capmx->CCR2.BIT.cap_mode = capMode;
    return;
}

/**
  * @brief Set capture stop on which register's capture event.
  * @param capmx: CAPM register base address.
  * @param capNum: Stop capture register number.
  * @retval None.
  */
static inline void DCL_CAPM_SetStopSeq(CAPM_RegStruct * const capmx, CampConfigCapRegNum capNum)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    CAPM_PARAM_CHECK_NO_RET(capNum >= 0);
    CAPM_PARAM_CHECK_NO_RET(capNum < CAPM_MAX_CAP_REG_NUM);
    capmx->CCR2.BIT.seq_stop = capNum;
    return;
}

/**
  * @brief Enable capm0 sync input.
  * @param capmComm: CAPM COMM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_EnableSyncIn0(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->SYNC_SELR0.BIT.capm0_synci_en = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Enable capm1 sync input.
  * @param capmComm: CAPM COMM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_EnableSyncIn1(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->SYNC_SELR1.BIT.capm1_synci_en = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Enable capm2 sync input.
  * @param capmComm: CAPM COMM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_EnableSyncIn2(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->SYNC_SELR2.BIT.capm2_synci_en = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Disable capm0 sync input.
  * @param capmComm: CAPM COMM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableSyncIn0(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->SYNC_SELR0.BIT.capm0_synci_en = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Disable capm1 sync input.
  * @param capmComm: CAPM COMM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableSyncIn1(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->SYNC_SELR1.BIT.capm1_synci_en = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Disable capm2 sync input.
  * @param capmComm: CAPM COMM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableSyncIn2(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->SYNC_SELR2.BIT.capm2_synci_en = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Triggle a software sync event for capm0.
  * @param capmComm: CAPM COMM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_TriggleSoftSync0(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->CAPM_GENE_CR.BIT.sync_sw_capm0 = BASE_CFG_SET;
    return;
}

/**
  * @brief Triggle a software sync event for capm1.
  * @param capmComm: CAPM COMM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_TriggleSoftSync1(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->CAPM_GENE_CR.BIT.sync_sw_capm1 = BASE_CFG_SET;
    return;
}

/**
  * @brief Triggle a software sync event for capm2.
  * @param capmComm: CAPM COMM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_TriggleSoftSync2(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->CAPM_GENE_CR.BIT.sync_sw_capm2 = BASE_CFG_SET;
    return;
}

/**
  * @brief Clear all CAPM interrupt flags.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_ClearAllInter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTRAWR.reg = 0x1FF;
    return;
}

/**
  * @brief Clear specific interrupt.
  * @param capmx: CAPM register base address.
  * @param eventNumber: Specific interrupt.
  * @retval None.
  */
static inline void DCL_CAPM_ClearInter(CAPM_RegStruct * const capmx, CAPM_Interrupt eventNumber)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTRAWR.reg |= (unsigned int)eventNumber;
    return;
}

/**
  * @brief Enable specific interrupt.
  * @param capmx: CAPM register base address.
  * @param eventNumber: Specific interrupt.
  * @retval None.
  */
static inline void DCL_CAPM_EnableInter(CAPM_RegStruct * const capmx, CAPM_Interrupt eventNumber)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.reg |= (unsigned int)eventNumber;
    return;
}

/**
  * @brief Disable specific interrupt.
  * @param capmx: CAPM register base address.
  * @param eventNumber: Specific interrupt.
  * @retval None.
  */
static inline void DCL_CAPM_DisableInter(CAPM_RegStruct * const capmx, CAPM_Interrupt eventNumber)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.reg &= (~(unsigned int)eventNumber);
    return;
}

/**
  * @brief Enable event1 interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_EnableEvt1Inter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.BIT.evt0_en = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Disable event1 interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableEvt1Inter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.BIT.evt0_en = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Enable event2 interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_EnableEvt2Inter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.BIT.evt1_en = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Disable event2 interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableEvt2Inter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.BIT.evt1_en = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Enable event3 interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_EnableEvt3Inter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.BIT.evt2_en = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Disable event3 interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableEvt3Inter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.BIT.evt2_en = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Enable event4 interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_EnableEvt4Inter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.BIT.evt3_en = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Disable event4 interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableEvt4Inter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.BIT.evt3_en = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Enable TSR overflow interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_EnableTsrovfInter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.BIT.tsr_ovf_en = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Disable TSR overflow interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableTsrovfInter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.BIT.tsr_ovf_en = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Enable ECR overflow interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_EnableEcrovfInter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.BIT.ecr_ovf_en = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Disable ECR overflow interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableEcrovfInter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.BIT.ecr_ovf_en = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Enable EAR compare match interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_EnableEARCMPMatchInter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.BIT.earcmp_match_en = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Disable EAR compare match interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableEARCMPMatchInter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.BIT.earcmp_match_en = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Enable EAR overflow interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_EnableEarovfInter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.BIT.ear_ovf_en = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Disable EAR overflow interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableEarovfInter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.BIT.ear_ovf_en = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Enable DMA overflow interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_EnableDmaovfInter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.BIT.dmareq_ovf_en = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Disable DMA overflow interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableDmaovfInter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTENR.BIT.dmareq_ovf_en = BASE_CFG_DISABLE;
    return;
}

/**
  * @brief Get all interrupt flags.
  * @param capmx: CAPM register base address.
  * @retval Interrupt flags.
  */
static inline unsigned int DCL_CAPM_GetInterFlag(const CAPM_RegStruct *capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    return capmx->INTFLGR.reg;
}

/**
  * @brief Inject interrupts by software.
  * @param capmx: CAPM register base address.
  * @param eventNumber: Inject interrupt.
  * @retval None.
  */
static inline void DCL_CAPM_InjectInter(CAPM_RegStruct * const capmx, CAPM_Interrupt eventNumber)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTFLGR.reg |= (unsigned int)eventNumber;
    return;
}

/**
  * @brief Inject event1 interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_InjectEvt1Inter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTINJR.BIT.evt0_inj |= 0x01;
    return;
}

/**
  * @brief Inject event2 interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_InjectEvt2Inter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTINJR.BIT.evt1_inj |= 0x01;
    return;
}

/**
  * @brief Inject event3 interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_InjectEvt3Inter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTINJR.BIT.evt2_inj |= 0x01;
    return;
}

/**
  * @brief Inject event4 interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_InjectEvt4Inter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTINJR.BIT.evt3_inj |= 0x01;
    return;
}

/**
  * @brief Inject TSR overflow interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_IngectTsrovfInter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTINJR.BIT.tsr_ovf_inj |= 0x01;
    return;
}

/**
  * @brief Inject ECR overflow interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_InjectEcrovfInter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTINJR.BIT.ecr_ovf_inj |= 0x01;
    return;
}

/**
  * @brief Inject EAR overflow interrupt.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_InjectEarOvfInter(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->INTINJR.BIT.ear_ovf_inj |= 0x01;
    return;
}

/**
  * @brief Enable emulation stop TSR count.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_EnableEmuStopTSR(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR1.BIT.emu_stop_en = BASE_CFG_ENABLE;
    return;
}

/**
  * @brief Disable emulation stop TSR count.
  * @param capmx: CAPM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableEmuStopTSR(CAPM_RegStruct * const capmx)
{
    CAPM_ASSERT_PARAM(IsCAPMInstance(capmx));
    capmx->CCR1.BIT.emu_stop_en = BASE_CFG_DISABLE;
}

/**
  * @brief Disable TSR count stop control
  * @param capmComm: CAPM_COMM register base address.
  * @retval None.
  */
static inline void DCL_CAPM_DisableTSRStop(CAPM_COMM_RegStruct * const capmComm)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->CAPM_GENE_CR.BIT.tsr_stop_capm0 = BASE_CFG_DISABLE;
    capmComm->CAPM_GENE_CR.BIT.tsr_stop_capm1 = BASE_CFG_DISABLE;
    capmComm->CAPM_GENE_CR.BIT.tsr_stop_capm2 = BASE_CFG_DISABLE;
}

/**
  * @brief Set CAPM0 input source
  * @param capmComm: CAPM_COMM register base address.
  * @param src: source selection
  * @retval None.
  */
static inline void DCL_CAPM_SetInputSEL0(CAPM_COMM_RegStruct * const capmComm, CAPM_InputSrc src)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->INPUT_SELR0.BIT.capm0_in_sel = src;
}

/**
  * @brief Set CAPM1 input source
  * @param capmComm: CAPM_COMM register base address.
  * @param src: source selection
  * @retval None.
  */
static inline void DCL_CAPM_SetInputSEL1(CAPM_COMM_RegStruct * const capmComm, CAPM_InputSrc src)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->INPUT_SELR1.BIT.capm1_in_sel = src;
}

/**
  * @brief Set CAPM2 input source
  * @param capmComm: CAPM_COMM register base address.
  * @param src: source selection
  * @retval None.
  */
static inline void DCL_CAPM_SetInputSEL2(CAPM_COMM_RegStruct * const capmComm, CAPM_InputSrc src)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    capmComm->INPUT_SELR2.BIT.capm2_in_sel = src;
}

/**
  * @brief Set CAPM0 sync input source
  * @param capmComm: CAPM_COMM register base address.
  * @param src: apt source selection
  * @retval None.
  */
static inline void DCL_CAPM_SetSyncInput0(CAPM_COMM_RegStruct * const capmComm, CAPM_SyncSrc src)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    CAPM_PARAM_CHECK_NO_RET(src >= CAPM_SYNC_SRC_NONE);
    CAPM_PARAM_CHECK_NO_RET(src <= CAPM_SYNC_SRC_APT8);
    capmComm->SYNC_SELR0.BIT.capm0_sync_sel = src;
}

/**
  * @brief Set CAPM1 sync input source
  * @param capmComm: CAPM_COMM register base address.
  * @param src: apt source selection
  * @retval None.
  */
static inline void DCL_CAPM_SetSyncInput1(CAPM_COMM_RegStruct * const capmComm, CAPM_SyncSrc src)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    CAPM_PARAM_CHECK_NO_RET(src >= CAPM_SYNC_SRC_NONE);
    CAPM_PARAM_CHECK_NO_RET(src <= CAPM_SYNC_SRC_APT8);
    capmComm->SYNC_SELR1.BIT.capm1_sync_sel = src;
}

/**
  * @brief Set CAPM2 sync input source
  * @param capmComm: CAPM_COMM register base address.
  * @param src: apt source selection
  * @retval None.
  */
static inline void DCL_CAPM_SetSyncInput2(CAPM_COMM_RegStruct * const capmComm, CAPM_SyncSrc src)
{
    CAPM_ASSERT_PARAM(IsCAPMCOMMInstance(capmComm));
    CAPM_PARAM_CHECK_NO_RET(src >= CAPM_SYNC_SRC_NONE);
    CAPM_PARAM_CHECK_NO_RET(src <= CAPM_SYNC_SRC_APT8);
    capmComm->SYNC_SELR2.BIT.capm2_sync_sel = src;
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
