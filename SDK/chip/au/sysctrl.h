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
  * @file      sysctrl.h
  * @author    MCU Driver Team
  * @brief     This file provides firmware functions to manage the following
  *            functionalities of the system control register.
  *                + Register Struct of SYSCTRL
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_SYSCTRL_H
#define McuMagicTag_SYSCTRL_H

/* Includes ------------------------------------------------------------------ */
#include "baseaddr.h"
#include "typedefs.h"

/* Macro definitions ---------------------------------------------------------*/
#define SC_LOCKEN_VALID_HIGH_BIT 0xEA510000U /**< Upper 16 active bits of the SC_LOCKEN register */
#define SC_LOW_BIT_MASK 0x0000FFFFU /**< Obtains the mask of the lower 16 bits. */
#define SC_LOCKEN_CRG_DISABLE_MASK 0x0000FFFEU /**< CRG write protection disable mask in SC_LOCKEN */
#define SC_LOCKEN_CRG_ENABLE_MASK 0x00000001U /**< CRG write protection enable mask in SC_LOCKEN */
#define SC_LOCKEN_SC_DISABLE_MASK 0x0000FFFDU /**< SC write protection disable mask in SC_LOCKEN */
#define SC_LOCKEN_SC_ENABLE_MASK 0x00000002U /**< SC write protection enable mask in SC_LOCKEN */


/**
  * @brief Records the offsets of various states in the CPU status register.
  */
typedef enum {
    SYSCTRL_NMI_BIT        = 0x00000000U,
    SYSCTRL_LOCKUP_BIT     = 0x00000002U,
    SYSCTRL_HARD_FAULT_BIT = 0x00000003U,
    SYSCTRL_DEBUG_BIT      = 0x00000004U,
    SYSCTRL_SLEEP_BIT      = 0x00000005U,
    SYSCTRL_PC_VALID_BIT   = 0x0000001FU
} SYSCTRL_CPU_Status;

/**
  * @brief FUNC_JTAG_SEL_REG register function item.
  */
typedef enum {
    SYSCTRL_FUNC_JTAG_CORESIGHT = 0x00000000U,
    SYSCTRL_FUNC_JYAG_EFLASH    = 0x00000001U
} SYSCTRL_FUNC_JTAG_Status;

/**
  * @brief System soft reset register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int softresreq : 1;    /**< Soft reset of the system. */
        unsigned int reserved : 31;
    } BIT;
} volatile SC_SYS_RES_REG;

/**
  * @brief Record the number of resets(soft reset, pin reset).
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soft_rst_cnt : 16;     /**< Number of soft reset. */
        unsigned int ext_rst_cnt : 16;      /**< Number of the RESETN pin reset. */
    } BIT;
} volatile SC_RST_CNT0_REG;

/**
  * @brief Record the number of resets(wdg reset, iwdg reset).
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int wdg_rst_cnt : 16;      /**< Number of wdg reset. */
        unsigned int iwdg_rst_cnt : 16;     /**< Number of iwdg reset. */
    } BIT;
} volatile SC_RST_CNT1_REG;

/**
  * @brief System status register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int update_mode_clear : 1;     /**< System update mark clear register. */
        unsigned int reserved0 : 3;
        unsigned int update_mode : 1;           /**< System update mark. */
        unsigned int reserved1 : 27;
    } BIT;
} volatile SC_SYS_STAT_REG;

/**
  * @brief Software interrupt register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int software_int : 1;          /**< Software interrupt register. */
        unsigned int reserved : 31;
    } BIT;
} volatile SC_SOFT_INT_REG;

/**
  * @brief Software interrupt event ID register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int swint_evt_id : 32;         /**< Software interrupt event ID. */
    } BIT;
} volatile SC_SOFT_EVT_ID_REG;

/**
  * @brief Lock register of key registers.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int crg_cfg_lock : 1;          /**< CRG register write-protection. */
        unsigned int sc_cfg_lock : 1;           /**< SYSCTRL register write-protection. */
        unsigned int reserved : 30;
    } BIT;
} volatile SC_LOCKEN_REG;

/**
  * @brief SC dedicated hard reset register 0. (CH) This register is not reset by a system soft reset.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int sc_hrst_reg0 : 32;         /**< JTAG/SWD interface debug or write system. */
    } BIT;
} volatile SC_HRST_REG0_REG;

/**
  * @brief User dedicated hard reset register 0. (CH) This register is not reset by a system soft reset.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int user_hrst_reg0 : 32;       /**< User dedicated hard reset register 0. */
    } BIT;
} volatile USER_HRST_REG0_REG;

/**
  * @brief User dedicated hard reset register 1. (CH) This register is not reset by a system soft reset.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int user_hrst_reg1 : 32;       /**< User dedicated hard reset register 1. */
    } BIT;
} volatile USER_HRST_REG1_REG;

/**
  * @brief User dedicated POR reset register 0. (CH) This register is reset only by a POR reset.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int user_por_reg0 : 32;        /**< User dedicated POR reset register 0. */
    } BIT;
} volatile USER_POR_REG0_REG;

/**
  * @brief User dedicated POR reset register 1. (CH) This register is reset only by a POR reset.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int user_por_reg1 : 32;        /**< User dedicated POR reset register 1. */
    } BIT;
} volatile USER_POR_REG1_REG;

/**
  * @brief User dedicated register 0.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int user_reg0 : 32;        /**< User dedicated register 0. */
    } BIT;
} volatile USER_REG0_REG;

/**
  * @brief User dedicated register 1.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int user_reg1 : 32;        /**< User dedicated register 1. */
    } BIT;
} volatile USER_REG1_REG;

/**
  * @brief SYSCTRL0 register.
  */
typedef struct _SYSCTRL0_Regstruct {
    char space0[4];
    SC_SYS_RES_REG SC_SYS_RES;              /**< System soft reset register. */
    SC_RST_CNT0_REG SC_RST_CNT0;            /**< Record the number of resets(soft reset, pin reset). */
    SC_RST_CNT1_REG SC_RST_CNT1;            /**< Record the number of resets(wdg reset, iwdg reset). */
    char space1[8];
    SC_SYS_STAT_REG SC_SYS_STAT;            /**< System status register. */
    char space2[4];
    SC_SOFT_INT_REG SC_SOFT_INT;            /**< Software interrupt register. */
    SC_SOFT_EVT_ID_REG SC_SOFT_EVT_ID;      /**< Software interrupt event ID register. */
    char space3[28];
    SC_LOCKEN_REG SC_LOCKEN;                /**< Lock register of key registers. */
    char space4[440];
    SC_HRST_REG0_REG SC_HRST_REG0;          /**< SC dedicated hard reset register 0. */
    char space5[3068];
    USER_HRST_REG0_REG USER_HRST_REG0;      /**< User dedicated hard reset register 0. */
    USER_HRST_REG1_REG USER_HRST_REG1;      /**< User dedicated hard reset register 1. */
    char space6[56];
    USER_POR_REG0_REG USER_POR_REG0;        /**< User dedicated POR reset register 0. */
    USER_POR_REG1_REG USER_POR_REG1;        /**< User dedicated POR reset register 1. */
    char space7[56];
    USER_REG0_REG USER_REG0;                /**< User dedicated register 0. */
    USER_REG1_REG USER_REG1;                /**< User dedicated register 1. */
} volatile SYSCTRL0_RegStruct;

/**
  * @brief APT enable control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int apt0_run : 1;          /**< APT0 enable. */
        unsigned int apt1_run : 1;          /**< APT1 enable. */
        unsigned int apt2_run : 1;          /**< APT2 enable. */
        unsigned int apt3_run : 1;          /**< APT3 enable. */
        unsigned int apt4_run : 1;          /**< APT4 enable. */
        unsigned int apt5_run : 1;          /**< APT5 enable. */
        unsigned int apt6_run : 1;          /**< APT6 enable. */
        unsigned int apt7_run : 1;          /**< APT7 enable. */
        unsigned int apt8_run : 1;          /**< APT8 enable. */
        unsigned int reserved : 23;
    } BIT;
} volatile APT_RUN_REG;

/**
  * @brief APT POE filter control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int poe0_filter_level : 8;     /**< Number of POE0 filter period. */
        unsigned int poe1_filter_level : 8;     /**< Number of POE1 filter period. */
        unsigned int poe2_filter_level : 8;     /**< Number of POE2 filter period. */
        unsigned int poe0_filter_en : 1;        /**< POE0 filter enable. */
        unsigned int poe1_filter_en : 1;        /**< POE1 filter enable. */
        unsigned int poe2_filter_en : 1;        /**< POE2 filter enable. */
        unsigned int reserved : 5;
    } BIT;
} volatile APT_POE_FILTER_REG;

/**
  * @brief APT_EVTIO filter control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int apt_evtio4_filter_level : 8;       /**< Number of APT EVTIO4 filter period. */
        unsigned int apt_evtio5_filter_level : 8;       /**< Number of APT EVTIO5 filter period. */
        unsigned int reserved0 : 8;
        unsigned int apt_evtio4_filter_en : 1;          /**< APT EVTIO4 filter enable. */
        unsigned int apt_evtio5_filter_en : 1;          /**< APT EVTIO5 filter enable. */
        unsigned int reserved1 : 6;
    } BIT;
} volatile APT_EVTIO_FILTER_REG;

/**
  * @brief APT_EVTMP filter control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int apt_evtmp4_filter_level : 8;       /**< Number of APT EVTMP4 filter period. */
        unsigned int apt_evtmp5_filter_level : 8;       /**< Number of APT EVTMP5 filter period. */
        unsigned int apt_evtmp6_filter_level : 8;       /**< Number of APT EVTMP6 filter period. */
        unsigned int apt_evtmp4_filter_en : 1;          /**< APT EVTMP4 filter enable. */
        unsigned int apt_evtmp5_filter_en : 1;          /**< APT EVTMP5 filter enable. */
        unsigned int apt_evtmp6_filter_en : 1;          /**< APT EVTMP6 filter enable. */
        unsigned int reserved : 5;
    } BIT;
} volatile APT_EVTMP_FILTER_REG;

/**
  * @brief CAPM synchronization source select register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int capm0_sync_sel : 2;
        unsigned int reserved0 : 2;
        unsigned int capm1_sync_sel : 2;
        unsigned int reserved1 : 2;
        unsigned int capm2_sync_sel : 2;
        unsigned int reserved2 : 22;
    } BIT;
} volatile CAPM_SYNC_SEL_REG;

/**
  * @brief DMA request source select register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int dma_req0_sel : 1;      /**< DMA request line0 source selection. */
        unsigned int dma_req1_sel : 1;      /**< DMA request line1 source selection. */
        unsigned int reserved0 : 3;
        unsigned int dma_req5_sel : 1;      /**< DMA request line5 source selection. */
        unsigned int dma_req6_sel : 1;      /**< DMA request line6 source selection. */
        unsigned int dma_req7_sel : 1;      /**< DMA request line7 source selection. */
        unsigned int dma_req8_sel : 1;      /**< DMA request line8 source selection. */
        unsigned int dma_req9_sel : 1;      /**< DMA request line9 source selection. */
        unsigned int dma_req10_sel : 1;     /**< DMA request line10 source selection. */
        unsigned int dma_req11_sel : 1;     /**< DMA request line11 source selection. */
        unsigned int dma_req12_sel : 1;     /**< DMA request line12 source selection. */
        unsigned int dma_req13_sel : 1;     /**< DMA request line13 source selection. */
        unsigned int dma_req14_sel : 1;     /**< DMA request line14 source selection. */
        unsigned int dma_req15_sel : 1;     /**< DMA request line15 source selection. */
        unsigned int reserved1 : 16;
    } BIT;
} volatile DMA_REQ_SEL_REG;

/**
  * @brief Sysram odd-even check status register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int sysram_parity_err_clr : 1;     /**< SYSRAM odd-even check error status clear. */
        unsigned int sysram_parity_err : 1;         /**< SYSRAM odd-even check error status. */
        unsigned int reserved : 30;
    } BIT;
} volatile SYSRAM_ERR_REG;

/**
  * @brief CPU status register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cpu_in_nmi_hdlr : 1;
        unsigned int cpu_ra_wr_en : 1;
        unsigned int cpu_lockup_mode : 1;       /**< CPU lockup status. */
        unsigned int cpu_hard_fault_mode : 1;   /**< CPU hard fault status.*/
        unsigned int cpu_debug_mode : 1;        /**< CPU debug status.*/
        unsigned int cpu_sleep_mode : 1;        /**< CPU sleep status. */
        unsigned int reserved : 25;
        unsigned int cpu_pc_valid : 1;          /**< CPU PC value effective status. */
    } BIT;
} volatile CPU_STATUS_REG;

/**
  * @brief CPU IRF_X1 value register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cpu_irf_x1 : 32;       /**< CPU IRF_X1 value.*/
    } BIT;
} volatile CPU_IRF_X1_REG;

/**
  * @brief CPU IRF_X2 value register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cpu_irf_x2 : 32;       /**< CPU IRF_X2 value.*/
    } BIT;
} volatile CPU_IRF_X2_REG;

/**
  * @brief Tsensor enable control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int tsensor_en : 1;        /**< Tsensor enable.*/
        unsigned int reserved : 31;
    } BIT;
} volatile TSENSOR_EN_REG;

/**
  * @brief ADCVREF bandgap control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adcvref_bg_en : 1;     /**< ADC0 VREF enable. */
        unsigned int reserved0 : 15;
        unsigned int adcvref_bg_trim : 5;   /**< ADC0 VREF voltage selection. */
        unsigned int reserved1 : 11;
    } BIT;
} volatile ADCVREF_CTRL0_REG;

/**
  * @brief ADCLDO control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adcvref_adcldo_en : 1;         /**< ADCLDO enable. */
        unsigned int adcvref_adcldo_bypss : 1;      /**< Reserved. */
        unsigned int reserved0 : 2;
        unsigned int adcvref_adcldo_s : 4;          /**< ADCLDO output voltage tap adjustment. */
        unsigned int reserved1 : 8;
        unsigned int adcvref_adcldo_trim : 5;       /**< ADCLDO calibration value. */
        unsigned int reserved2 : 3;
        unsigned int adcvref_adcldo_ib_sel : 1;     /**< ADCLDO low power comsumption mode enable. */
        unsigned int reserved3 : 6;
        unsigned int adcvref_adcldo_ok : 1;         /**< ADCLDO voltage stability mark. */
    } BIT;
} volatile ADCVREF_CTRL1_REG;

/**
  * @brief ADC0 control registe.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adcvref_refbuf_en0 : 1;            /**< ADC0 VREF enable. */
        unsigned int reserved0 : 3;
        unsigned int adcvref_refbuf_s0 : 1;             /**< ADC0 VREF voltage selection. */
        unsigned int reserved1 : 3;
        unsigned int adcvref_refbuf_sel0 : 1;           /**< ADC0 VREF source selection. */
        unsigned int reserved2 : 7;
        unsigned int adcvref_refbuf_trim0_2p0v : 5;     /**< Trim value when ADC0 VREF is 2.0v. */
        unsigned int reserved3 : 3;
        unsigned int adcvref_refbuf_trim0_2p5v : 5;     /**< Trim value when ADC0 VREF is 2.5v. */
        unsigned int reserved4 : 3;
    } BIT;
} volatile ADC0_VREF_CTRL_REG;

/**
  * @brief ADC1 control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adcvref_refbuf_en1 : 1;            /**< ADC1 VREF enable. */
        unsigned int reserved0 : 3;
        unsigned int adcvref_refbuf_s1 : 1;             /**< ADC1 VREF voltage selection. */
        unsigned int reserved1 : 3;
        unsigned int adcvref_refbuf_sel1 : 1;           /**< ADC1 VREF source selection. */
        unsigned int reserved2 : 7;
        unsigned int adcvref_refbuf_trim1_2p0v : 5;     /**< Trim value when ADC1 VREF is 2.0v. */
        unsigned int reserved3 : 3;
        unsigned int adcvref_refbuf_trim1_2p5v : 5;     /**< Trim value when ADC1 VREF is 2.5v. */
        unsigned int reserved4 : 3;
    } BIT;
} volatile ADC1_VREF_CTRL_REG;

/**
  * @brief ADC2 control register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adcvref_refbuf_en2 : 1;            /**< ADC2 VREF enable. */
        unsigned int reserved0 : 3;
        unsigned int adcvref_refbuf_s2 : 1;             /**< ADC2 VREF voltage selection. */
        unsigned int reserved1 : 3;
        unsigned int adcvref_refbuf_sel2 : 1;           /**< ADC2 VREF source selection. */
        unsigned int reserved2 : 7;
        unsigned int adcvref_refbuf_trim2_2p0v : 5;     /**< Trim value when ADC2 VREF is 2.0v. */
        unsigned int reserved3 : 3;
        unsigned int adcvref_refbuf_trim2_2p5v : 5;     /**< Trim value when ADC2 VREF is 2.5v. */
        unsigned int reserved4 : 3;
    } BIT;
} volatile ADC2_VREF_CTRL_REG;

/**
  * @brief ADC2CORE control register 6.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adcvref_test_en : 1;       /**< ADC VREF test function enable. */
        unsigned int reserved0 : 3;
        unsigned int adcvref_test_sel : 4;      /**< ADC VREF test function selection. */
        unsigned int reserved1 : 8;
        unsigned int adcvref_rsv : 8;           /**< ADC VREF reserved. */
        unsigned int reserved2 : 8;
    } BIT;
} volatile ADCVREF_CTRL6_REG;

/**
  * @brief ADC2CORE_ISO register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc2core_iso : 1;      /**< ADC power feild and CORE power feild isolation control. */
        unsigned int reserved : 31;
    } BIT;
} volatile ADC2CORE_ISO_REG;

/**
  * @brief ADC serial observation selection register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_ob_sel : 2;        /**< ADC serial output side selection. */
        unsigned int reserved : 30;
    } BIT;
} volatile ADC_OB_SEL_REG;

/**
  * @brief SYSCTRL1 register.
  */
typedef struct _SYSCTRL1_RegStruct {
    char space0[0x8000];
    APT_RUN_REG APT_RUN;                        /**< APT enable control register, offset address: 0x8000. */
    char space1[12];
    APT_POE_FILTER_REG APT_POE_FILTER;          /**< APT POE filter control register, offset address: 0x8010. */
    APT_EVTIO_FILTER_REG APT_EVTIO_FILTER;      /**< APT_EVTIO filter control register, offset address: 0x8014. */
    APT_EVTMP_FILTER_REG APT_EVTMP_FILTER;      /**< APT_EVTMP filter control register, offset address: 0x8018. */
    char space2[484];
    DMA_REQ_SEL_REG DMA_REQ_SEL;                /**< DMA request source select register, offset address: 0x8200. */
    char space3[252];
    SYSRAM_ERR_REG SYSRAM_ERR;                  /**< Sysram odd-even check status register, offset address: 0x8300. */
    char space4[3324];
    CPU_STATUS_REG CPU_STATUS;                  /**< CPU status register, offset address: 0x9000. */
    char space5[4092];
    TSENSOR_EN_REG TSENSOR_EN;                  /**< Tsensor enable control register, offset address: 0xA000. */
    char space6[4092];
    ADCVREF_CTRL0_REG ADCVREF_CTRL0;            /**< ADCVREF bandgap control register, offset address: 0xB000. */
    ADCVREF_CTRL1_REG ADCVREF_CTRL1;            /**< ADCLD0 control register, offset address: 0xB004. */
    ADC0_VREF_CTRL_REG ADC0_VREF_CTRL;          /**< ADC0 control register, offset address: 0xB008. */
    ADC1_VREF_CTRL_REG ADC1_VREF_CTRL;          /**< ADC1 control register, offset address: 0xB00C. */
    ADC2_VREF_CTRL_REG ADC2_VREF_CTRL;          /**< ADC2 control register, offset address: 0xB010. */
    char space7[4];
    ADCVREF_CTRL6_REG ADCVREF_CTRL6;            /**< ADC2CORE control register 6, offset address: 0xB018. */
    char space8[20444];
    ADC_OB_SEL_REG ADC_OB_SEL;                  /**< ADC observation selection register, offset address: 0xFFF8. */
} volatile SYSCTRL1_RegStruct;

/**
  * @brief Make system soft reset.
  * @param None.
  * @retval None.
  */
static inline void DCL_SYSCTRL_SoftReset(void)
{
    SYSCTRL0->SC_SYS_RES.BIT.softresreq = 1;
}

/**
  * @brief Get number of soft resets.
  * @param None.
  * @retval Number of soft resets.
  */
static inline unsigned short DCL_SYSCTRL_GetSoftResetConut(void)
{
    return SYSCTRL0->SC_RST_CNT0.BIT.soft_rst_cnt;
}

/**
  * @brief Get number of reset times of the RESETN pin.
  * @param None.
  * @retval Number of reset times of the RESETN pin.
  */
static inline unsigned short DCL_SYSCTRL_GetPinResetConut(void)
{
    return SYSCTRL0->SC_RST_CNT0.BIT.ext_rst_cnt;
}

/**
  * @brief Get number of WDG resets.
  * @param None.
  * @retval Number of WDG resets.
  */
static inline unsigned short DCL_SYSCTRL_GetWdgResetConut(void)
{
    return SYSCTRL0->SC_RST_CNT1.BIT.wdg_rst_cnt;
}

/**
  * @brief Get number of IWDG resets.
  * @param None.
  * @retval Number of IWDG resets.
  */
static inline unsigned short DCL_SYSCTRL_GetIWdgResetConut(void)
{
    return SYSCTRL0->SC_RST_CNT1.BIT.iwdg_rst_cnt;
}

/**
  * @brief Set the write protection for SYSCTRL registers disable.
  * @param None.
  * @retval None.
  */
static inline void DCL_SYSCTRL_ScWriteProtectionDisable(void)
{
    /* Set the corresponding bit without affecting the other bits and set the high 16 bits to EA51 to write to */
    SYSCTRL0->SC_LOCKEN.reg = (SYSCTRL0->SC_LOCKEN.reg & SC_LOCKEN_SC_DISABLE_MASK) + SC_LOCKEN_VALID_HIGH_BIT;
}

/**
  * @brief Set the write protection for SYSCTRL registers enable.
  * @param None.
  * @retval None.
  */
static inline void DCL_SYSCTRL_ScWriteProtectionEnable(void)
{
    /* Set the corresponding bit without affecting the other bits and set the high 16 bits to EA51 to write to */
    SYSCTRL0->SC_LOCKEN.reg = ((SYSCTRL0->SC_LOCKEN.reg & SC_LOW_BIT_MASK) | SC_LOCKEN_SC_ENABLE_MASK) +
                              SC_LOCKEN_VALID_HIGH_BIT;
}

/**
  * @brief Set the write protection for CRG-related registers disable.
  * @param None.
  * @retval None.
  */
static inline void DCL_SYSCTRL_CrgWriteProtectionDisable(void)
{
    /* Set the corresponding bit without affecting the other bits and set the high 16 bits to EA51 to write to */
    SYSCTRL0->SC_LOCKEN.reg = (SYSCTRL0->SC_LOCKEN.reg & SC_LOCKEN_CRG_DISABLE_MASK) + SC_LOCKEN_VALID_HIGH_BIT;
}

/**
  * @brief Set the Set the write protection for CRG-related registers enable.
  * @param None.
  * @retval None.
  */
static inline void DCL_SYSCTRL_CrgWriteProtectionEnable(void)
{
    /* Set the corresponding bit without affecting the other bits and set the high 16 bits to EA51 to write to */
    SYSCTRL0->SC_LOCKEN.reg = ((SYSCTRL0->SC_LOCKEN.reg & SC_LOW_BIT_MASK) | SC_LOCKEN_CRG_ENABLE_MASK) +
                              SC_LOCKEN_VALID_HIGH_BIT;
}

/**
  * @brief Set software interrupt register, writing 1 generates a software interrupt.
  * @param None.
  * @retval None.
  */
static inline void DCL_SYSCTRL_GenerateSoftInterrupt(void)
{
    SYSCTRL0->SC_SOFT_INT.BIT.software_int = 1;
}

/**
  * @brief Set Software interrupt event ID.
  * @param id the software interrupt event ID.
  * @retval None.
  */
static inline void DCL_SYSCTRL_SetSoftInterruptEventId(unsigned int id)
{
    SYSCTRL0->SC_SOFT_EVT_ID.BIT.swint_evt_id = id;
}

/**
  * @brief Get Software interrupt event ID.
  * @param None.
  * @retval The value of software interrupt event ID.
  */
static inline unsigned int DCL_SYSCTRL_GetSoftInterruptEventId(void)
{
    return SYSCTRL0->SC_SOFT_EVT_ID.BIT.swint_evt_id;
}

/**
  * @brief Get SYSRAM Parity Error Status.
  * @param None.
  * @retval 0:no error, 1:error.
  */
static inline unsigned int DCL_SYSCTRL_GetSysramParityErrorStatus(void)
{
    return SYSCTRL1->SYSRAM_ERR.BIT.sysram_parity_err;
}

/**
  * @brief Set SYSRAM parity error status clear.
  * @param None.
  * @retval None.
  */
static inline void DCL_SYSCTRL_ClearSysramParityError(void)
{
    SYSCTRL1->SYSRAM_ERR.BIT.sysram_parity_err_clr = 1; /* Write any value to clear. */
}

/**
  * @brief Get CPU status.
  * @param offset Bit offset of CPU status.
  * @retval true or false
  */
static inline bool DCL_SYSCTRL_CheckCpuStatus(SYSCTRL_CPU_Status offset)
{
    return ((SYSCTRL1->CPU_STATUS.reg) & (1 << offset)) == 0 ? false : true;
}

/**
  * @brief Set the tsensor function enable.
  * @param None.
  * @retval None
  */
static inline void DCL_SYSCTRL_EnableTsensor(void)
{
    SYSCTRL1->TSENSOR_EN.BIT.tsensor_en = BASE_CFG_ENABLE;
}

/**
  * @brief Set the tsensor function disable.
  * @param None.
  * @retval None
  */
static inline void DCL_SYSCTRL_DisableTsensor(void)
{
    SYSCTRL1->TSENSOR_EN.BIT.tsensor_en = BASE_CFG_DISABLE;
}

#endif /* McuMagicTag_SYSCTRL_H */