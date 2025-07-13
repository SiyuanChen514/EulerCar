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
#define SC_LOCKEN_SC_ENABLE_MASK 0x00000002U /**< SC write protection enbale mask in SC_LOCKEN */


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
        unsigned int softresreq : 1; /**< Set any value to make system soft reset. */
        unsigned int reserved : 31;
    } BIT;
} volatile SC_SYS_RES_REG;

/**
  * @brief Record the number of resets(soft reset, pin reset).
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soft_rst_cnt : 16; /**< Number of soft resets. */
        unsigned int ext_rst_cnt : 16;  /**< Number of reset times of the RESETN pin. */
    } BIT;
} volatile SC_RST_CNT0_REG;

/**
  * @brief Record the number of resets(wdg reset, iwdg reset).
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int wdg_rst_cnt : 16;  /**< Number of WDG resets. */
        unsigned int iwdg_rst_cnt : 16; /**< Number of IWDG resets. */
    } BIT;
} volatile SC_RST_CNT1_REG;

/**
  * @brief System status register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int update_mode_clear : 1; /**< System upgrade flag clear register, 0:not clear, 1:clear. */
        unsigned int reserved0 : 3;
        unsigned int update_mode : 1;       /**< System upgrade flag, 0:not upgrade, 1:upgrade. */
        unsigned int reserved1 : 27;
    } BIT;
} volatile SC_SYS_STAT_REG;

/**
  * @brief Software interrupt register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int software_int : 1; /**< Software interrupt register, writing 1 generates a software interrupt. */
        unsigned int reserved : 31;
    } BIT;
} volatile SC_SOFT_INT_REG;

/**
  * @brief Software interrupt event ID register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int swint_evt_id : 32; /**< Software interrupt event ID. */
    } BIT;
} volatile SC_SOFT_EVT_ID_REG;

/**
  * @brief Lock register of key registers.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int crg_cfg_lock : 1; /**< Write protection for CRG, 0: write enabled, 1: write disabled. */
        unsigned int sc_cfg_lock : 1;  /**< Write protection for SYSCTRL, 0: write enabled, 1: write disabled. */
        unsigned int reserved : 30;
    } BIT;
} volatile SC_LOCKEN_REG;

/**
  * @brief SC dedicated hard reset register 0. (CH) This register is not reset by a system soft reset.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int sc_hrst_reg0 : 32;  /**< If the value is 0xA5A5A5, the CPU stops starting the system. */
    } BIT;
} volatile SC_HRST_REG0_REG;

/**
  * @brief User dedicated hard reset register 0. (CH) This register is not reset by a system soft reset.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int user_hrst_reg0 : 32;  /**< User-dedicated hard reset register 0. */
    } BIT;
} volatile USER_HRST_REG0_REG;

/**
  * @brief User dedicated hard reset register 1. (CH) This register is not reset by a system soft reset.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int user_hrst_reg1 : 32;  /**< User-dedicated hard reset register 1. */
    } BIT;
} volatile USER_HRST_REG1_REG;

/**
  * @brief User dedicated POR reset register 0. (CH) This register is reset only by a POR reset.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int user_por_reg0 : 32;  /**< User dedicated POR reset register 0. */
    } BIT;
} volatile USER_POR_REG0_REG;

/**
  * @brief User dedicated POR reset register 1. (CH) This register is reset only by a POR reset.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int user_por_reg1 : 32;  /**< User dedicated POR reset register 1. */
    } BIT;
} volatile USER_POR_REG1_REG;

/**
  * @brief User dedicated register 0.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int user_reg0 : 32;  /**< User dedicated register 0. */
    } BIT;
} volatile USER_REG0_REG;

/**
  * @brief User dedicated register 1.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int user_reg1 : 32;  /**< User dedicated register 1. */
    } BIT;
} volatile USER_REG1_REG;

/**
  * @brief SYSCTRL0 register.
  */
typedef struct _SYSCTRL0_Regstruct {
    char space0[4];
    SC_SYS_RES_REG SC_SYS_RES;         /**< System soft reset register, offset address: 0x0004. */
    SC_RST_CNT0_REG SC_RST_CNT0;       /**< Reset count register 0, offset address: 0x0008. */
    SC_RST_CNT1_REG SC_RST_CNT1;       /**< Reset count register 1, offset address: 0x000C. */
    char space1[8];
    SC_SYS_STAT_REG SC_SYS_STAT;       /**< System boot mode register, offset address: 0x0018. */
    char space2[4];
    SC_SOFT_INT_REG SC_SOFT_INT;       /**< Software interrupt register, offset address: 0x0020. */
    SC_SOFT_EVT_ID_REG SC_SOFT_EVT_ID; /**< Software interrupt event ID register, offset address: 0x0024. */
    char space3[28];
    SC_LOCKEN_REG SC_LOCKEN;           /**< Lock register of key registers, offset address: 0x0044. */
    char space4[440];
    SC_HRST_REG0_REG SC_HRST_REG0;     /**< SC dedicated hard reset register 0, offset address: 0x0200. */
    char space5[3068];
    USER_POR_REG0_REG USER_POR_REG0;   /**< User dedicated POR reset register 0, offset address: 0x0E00. */
    USER_POR_REG1_REG USER_POR_REG1;   /**< User dedicated POR reset register 1, offset address: 0x0E04. */
    char space6[56];
    USER_HRST_REG0_REG USER_HRST_REG0; /**< User dedicated hard reset register 0, offset address: 0x0E40. */
    USER_HRST_REG1_REG USER_HRST_REG1; /**< User dedicated hard reset register 1, offset address: 0x0E44. */
    char space7[56];
    USER_REG0_REG USER_REG0;           /**< User dedicated register 0, offset address: 0x0E80. */
    USER_REG1_REG USER_REG1;           /**< User dedicated register 1, offset address: 0x0E84. */
} volatile SYSCTRL0_RegStruct;

/**
  * @brief APT enable register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int apt0_run : 1;   /**< APT0 enable control, 0：enable, 1:enable. */
        unsigned int apt1_run : 1;   /**< APT1 enable control, 0：enable, 1:enable. */
        unsigned int apt2_run : 1;   /**< APT2 enable control, 0：enable, 1:enable. */
        unsigned int apt3_run : 1;   /**< APT3 enable control, 0：enable, 1:enable. */
        unsigned int reserved : 28;
    } BIT;
} volatile APT_RUN_REG;

/**
  * @brief Poe filter register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int poe0_filter_level : 8;   /**< Number of POE0 Filter Cycles. */
        unsigned int poe1_filter_level : 8;   /**< Number of POE1 Filter Cycles. */
        unsigned int poe2_filter_level : 8;   /**< Number of POE2 Filter Cycles. */
        unsigned int poe0_filter_en : 1;      /**< POE0 filter enable, 0：enable, 1:enable. */
        unsigned int poe1_filter_en : 1;      /**< POE1 filter enable, 0：enable, 1:enable. */
        unsigned int poe2_filter_en : 1;      /**< POE2 filter enable, 0：enable, 1:enable. */
        unsigned int reserved : 5;
    } BIT;
} volatile APT_POE_FILTER_REG;

/**
  * @brief APT_EVTIO_FILTER register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int apt_evtio4_filter_level : 8;  /**< Number of APT EVTIO4 Filter Cycles. */
        unsigned int apt_evtio5_filter_level : 8;  /**< Number of APT EVTIO5 Filter Cycles. */
        unsigned int reserved0 : 8;
        unsigned int apt_evtio4_filter_en : 1;     /**< APT EVTIO4 FILTER enable, 0：enable, 1:enable. */
        unsigned int apt_evtio5_filter_en : 1;     /**< APT EVTIO5 FILTER enable, 0：enable, 1:enable. */
        unsigned int reserved1 : 6;
    } BIT;
} volatile APT_EVTIO_FILTER_REG;

/**
  * @brief APT_EVTMP_FILTER register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int apt_evtmp4_filter_level : 8;  /**< Number of APT EVTMP4 Filter Periods. */
        unsigned int apt_evtmp5_filter_level : 8;  /**< Number of APT EVTMP5 Filter Periods. */
        unsigned int apt_evtmp6_filter_level : 8;  /**< Number of APT EVTMP6 Filter Periods. */
        unsigned int apt_evtmp4_filter_en : 1;     /**< APT EVTMP4 FILTER enable, 0：enable, 1:enable. */
        unsigned int apt_evtmp5_filter_en : 1;     /**< APT EVTMP5 FILTER enable, 0：enable, 1:enable. */
        unsigned int apt_evtmp6_filter_en : 1;     /**< APT EVTMP6 FILTER enable, 0：enable, 1:enable. */
        unsigned int reserved : 5;
    } BIT;
} volatile APT_EVTMP_FILTER_REG;

/**
 * @brief XTAL_CFG register.
 *
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int osc_ds : 4;     /**< Crystal I/O Drive Capability Configuration. */
        unsigned int ose_e : 1;      /**< Crystal I/O resonance buffer enable, 0:disable, 1:enable. */
        unsigned int osc_ie : 1;     /**< Crystal I/O clock input enable, 0:disable, 1:enable. */
        unsigned int reserved : 26;
    } BIT;
} volatile XTAL_CFG_REG;

/**
  * @brief Sysram parity check register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int sysram_parity_err_clr : 1;  /**< SYSRAM parity error status clear, write any value to clear. */
        unsigned int sysram0_parity_err : 1;     /**< SYSRAM Parity Error Status, 0:no error, 1:error. */
        unsigned int sysram1_parity_err : 1;     /**< SYSRAM Parity Error Status, 0:no error, 1:error. */
        unsigned int reserved : 29;
    } BIT;
} volatile SYSRAM_ERR_REG;

/**
 * @brief SYSRAM_MAP register.
 *
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int sysram_split : 3;     /**< SYSRAM space allocation. */
        unsigned int reserved : 29;
    } BIT;
} volatile SYSRAM_MAP_REG;

/**
 * @brief PVD_CFG register.
 *
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pvd_en : 1;        /**< PVD enable flag, 0:disable, 1:enable. */
        unsigned int pvd_rise_thd : 3;  /**< PVD rising edge threshold. */
        unsigned int pvd_fall_thd : 3;  /**< PVD falling edge threshold. */
        unsigned int reserved : 25;
    } BIT;
} volatile PVD_CFG_REG;

/**
 * @brief PVD_STATUS register.
 *
 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int pvd_toggle : 1;  /**< PVD triggering flag. */
        unsigned int reserved : 31;
    } BIT;
} volatile PVD_STATUS_REG;

/**
  * @brief CPU status register.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cpu_in_nmi_hdlr : 1;     /**< CPU NMI processing status. 0: non-NMI processing state,
                                                   1: NMI processing status. */
        unsigned int cpu_ra_wr_en : 1;        /**< cpu_ra_wr enable. */
        unsigned int cpu_lockup_mode : 1;     /**< CPU LOCKUP status. 0: non-lockup state, 1: lockup state. */
        unsigned int cpu_hard_fault_mode : 1; /**< Indicates the hard fault status of the CPU. 0: non-hard_fault state,
                                                   1: hard_fault. */
        unsigned int cpu_debug_mode : 1;      /**< Indicates the CPU debug status. 0: non-debug state,
                                                   1: debug state. */
        unsigned int cpu_sleep_mode : 1;      /**< CPU sleep status.  0: non-sleep state, 1: sleep state. */
        unsigned int reserved : 25;
        unsigned int cpu_pc_valid : 1;        /**< Valid status of the CPU PC value. 0: The PC value is invalid,
                                                   1: The PC value is valid. */
    } BIT;
} volatile CPU_STATUS_REG;

/**
  * @brief SYSCTRL1 register.
  */
typedef struct _SYSCTRL1_RegStruct {
    char                 space0[0x8000];
    APT_RUN_REG          APT_RUN;            /**< APT enable control register, offset address: 0x8000. */
    char                 space1[12];
    APT_POE_FILTER_REG   APT_POE_FILTER;     /**< APT PoE filtering control register, offset address: 0x8010. */
    APT_EVTIO_FILTER_REG APT_EVTIO_FILTER;   /**< APT EVTIO filtering control register, offset address: 0x8014. */
    APT_EVTMP_FILTER_REG APT_EVTMP_FILTER;   /**< APT EVTMP filtering control register, offset address: 0x8018. */
    char                 space2[228];
    XTAL_CFG_REG         XTAL_CFG;           /**< Crystal I/O control register, offset address: 0x8100. */
    char                 space3[508];
    SYSRAM_ERR_REG       SYSRAM_ERR;         /**< SYSRAM parity check status register, offset address: 0x8300. */
    SYSRAM_MAP_REG       SYS_MAP;            /**< SYSRAM space allocation register, offset address: 0x8304. */
    char                 space4[248];
    PVD_CFG_REG          PVD_CFG;            /**< PVD configuration register, offset address: 0x8400. */
    PVD_STATUS_REG       PVD_STATUS;         /**< PVD status register, offset address: 0x8404. */
    char                 space5[3064];
    CPU_STATUS_REG       CPU_STATUS;         /**< CPU status register, offset address: 0x9000. */
} volatile SYSCTRL1_RegStruct;

/**
  * @brief Make system soft reset.
  * @param None
  * @retval None.
  */
static inline void DCL_SYSCTRL_SoftReset(void)
{
    SYSCTRL0->SC_SYS_RES.BIT.softresreq = 1;
}

/**
  * @brief Get number of soft resets.
  * @param None
  * @retval Number of soft resets.
  */
static inline unsigned short DCL_SYSCTRL_GetSoftResetConut(void)
{
    return SYSCTRL0->SC_RST_CNT0.BIT.soft_rst_cnt;
}

/**
  * @brief Get number of reset times of the RESETN pin.
  * @param None
  * @retval Number of reset times of the RESETN pin.
  */
static inline unsigned short DCL_SYSCTRL_GetPinResetConut(void)
{
    return SYSCTRL0->SC_RST_CNT0.BIT.ext_rst_cnt;
}

/**
  * @brief Get number of WDG resets.
  * @param None
  * @retval Number of WDG resets.
  */
static inline unsigned short DCL_SYSCTRL_GetWdgResetConut(void)
{
    return SYSCTRL0->SC_RST_CNT1.BIT.wdg_rst_cnt;
}

/**
  * @brief Get number of IWDG resets.
  * @param None
  * @retval Number of IWDG resets.
  */
static inline unsigned short DCL_SYSCTRL_GetIWdgResetConut(void)
{
    return SYSCTRL0->SC_RST_CNT1.BIT.iwdg_rst_cnt;
}

/**
  * @brief Set the write protection for SYSCTRL registers disable.
  * @param None
  * @retval None.
  */
static inline void DCL_SYSCTRL_ScWriteProtectionDisable(void)
{
    /* Set the corresponding bit without affecting the other bits and set the high 16 bits to EA51 to write to. */
    SYSCTRL0->SC_LOCKEN.reg = (SYSCTRL0->SC_LOCKEN.reg & SC_LOCKEN_SC_DISABLE_MASK) + SC_LOCKEN_VALID_HIGH_BIT;
}

/**
  * @brief Set the write protection for SYSCTRL registers enable.
  * @param None
  * @retval None.
  */
static inline void DCL_SYSCTRL_ScWriteProtectionEnable(void)
{
    /* Set the corresponding bit without affecting the other bits and set the high 16 bits to EA51 to write to. */
    SYSCTRL0->SC_LOCKEN.reg = ((SYSCTRL0->SC_LOCKEN.reg & SC_LOW_BIT_MASK) | SC_LOCKEN_SC_ENABLE_MASK) +
                              SC_LOCKEN_VALID_HIGH_BIT;
}

/**
  * @brief Set the write protection for CRG-related registers disable.
  * @param None
  * @retval None.
  */
static inline void DCL_SYSCTRL_CrgWriteProtectionDisable(void)
{
    /* Set the corresponding bit without affecting the other bits and set the high 16 bits to EA51 to write to. */
    SYSCTRL0->SC_LOCKEN.reg = (SYSCTRL0->SC_LOCKEN.reg & SC_LOCKEN_CRG_DISABLE_MASK) + SC_LOCKEN_VALID_HIGH_BIT;
}

/**
  * @brief Set the Set the write protection for CRG-related registers enable.
  * @param None
  * @retval None.
  */
static inline void DCL_SYSCTRL_CrgWriteProtectionEnable(void)
{
    /* Set the corresponding bit without affecting the other bits and set the high 16 bits to EA51 to write to. */
    SYSCTRL0->SC_LOCKEN.reg = ((SYSCTRL0->SC_LOCKEN.reg & SC_LOW_BIT_MASK) | SC_LOCKEN_CRG_ENABLE_MASK) +
                              SC_LOCKEN_VALID_HIGH_BIT;
}

/**
  * @brief Set software interrupt register, writing 1 generates a software interrupt.
  * @param None
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
  * @param None
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
    return SYSCTRL1->SYSRAM_ERR.BIT.sysram0_parity_err;
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
#endif /* McuMagicTag_SYSCTRL_H */