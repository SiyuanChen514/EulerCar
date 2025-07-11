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
  * @file    adc_ip.h
  * @author  MCU Driver Team
  * @brief   ADC module driver
  * @details This file provides DCL functions to manage ADC and Definition of
  *          specific parameters.
  *           + Definition of ADC configuration parameters.
  *           + ADC register mapping structure.
  *           + Parameters check functions.
  *           + Direct configuration layer interface.
  */

/* Macro definitions */
#ifndef McuMagicTag_ADC_IP_H
#define McuMagicTag_ADC_IP_H

#include "baseinc.h"
#include "dma.h"

extern bool g_trimEnable;
extern unsigned int g_versionId;
#define SOC_MAX_NUM 16
#define INT_MAX_NUM 4
#define HOLDTIME_MIN 2
#define HOLDTIME_MAX 28
#define ACQPSTIME_MIN 3
#define ACQPSTIME_MAX 127
#define SYNCGROUP_NUM 8
#define DMA_OVER_MASK 0x00000010
#define INT_OVER_MASK 0x0000000F
#define ADC_MIN_FREQ  2350000
#define ADC_MAX_FREQ  40000000
#ifdef ADC_PARAM_CHECK
#define ADC_ASSERT_PARAM BASE_FUNC_ASSERT_PARAM
#define ADC_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define ADC_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define ADC_ASSERT_PARAM(para) ((void)0U)
#define ADC_PARAM_CHECK_NO_RET(para) ((void)0U)
#define ADC_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif

/**
  * @addtogroup ADC
  * @{
  */

/**
  * @defgroup ADC_IP ADC_IP
  * @brief ADC_IP: adc_v0.
  * @{
  */

/**
  * @defgroup ADC_REG_Definition ADC Register Structure.
  * @brief ADC Register Structure Definition.
  * @{
  */

/**
  * @brief Define the union ADC_RESULT0_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_result0 : 12;  /**< SOC0 Results */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_RESULT0_REG;

/**
  * @brief Define the union ADC_RESULT1_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_result1 : 12;  /**< SOC1 Results */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_RESULT1_REG;

/**
  * @brief Define the union ADC_RESULT2_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_result2 : 12;  /**< SOC2 Results */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_RESULT2_REG;

/**
  * @brief Define the union ADC_RESULT3_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_result3 : 12;  /**< SOC3 Results */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_RESULT3_REG;

/**
  * @brief Define the union ADC_RESULT4_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_result4 : 12;  /**< SOC4 Results */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_RESULT4_REG;

/**
  * @brief Define the union ADC_RESULT5_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_result5 : 12;  /**< SOC5 Results */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_RESULT5_REG;

/**
  * @brief Define the union ADC_RESULT6_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_result6 : 12;  /**< SOC6 Results */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_RESULT6_REG;

/**
  * @brief Define the union ADC_RESULT7_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_result7 : 12;  /**< SOC7 Results */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_RESULT7_REG;

/**
  * @brief Define the union ADC_RESULT8_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_result8 : 12;  /**< SOC8 Results */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_RESULT8_REG;

/**
  * @brief Define the union ADC_RESULT9_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_result9 : 12;  /**< SOC9 Results */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_RESULT9_REG;

/**
  * @brief Define the union ADC_RESULT10_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_result10 : 12;  /**< SOC10 Results */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_RESULT10_REG;

/**
  * @brief Define the union ADC_RESULT11_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_result11 : 12;  /**< SOC11 Results */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_RESULT11_REG;

/**
  * @brief Define the union ADC_RESULT12_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_result12 : 12;  /**< SOC12 Results */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_RESULT12_REG;

/**
  * @brief Define the union ADC_RESULT13_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_result13 : 12;  /**< SOC13 Results */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_RESULT13_REG;

/**
  * @brief Define the union ADC_RESULT14_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_result14 : 12;  /**< SOC14 Results */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_RESULT14_REG;

/**
  * @brief Define the union ADC_RESULT15_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_result15 : 12;  /**< SOC15 Results */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_RESULT15_REG;

/**
  * @brief Define the union ADC_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0 : 1;
        unsigned int one_shot : 1;        /**< Block round robin mode enable */
        unsigned int reserved1 : 1;
        unsigned int adc_cal_mode : 2;    /**< Calibration mode selection */
        unsigned int adc_cal_en : 1;      /**< ADC calibration enable */
        unsigned int reserved2 : 3;
        unsigned int rstn_ana_num : 8;    /**< Delay count value for analog reset */
        unsigned int reserved3 : 3;
        unsigned int saen_ini_num : 8;    /**< Duration when the simulated SAEN signal is high */
        unsigned int reserved4 : 4;
    } BIT;
} volatile ADC_CTRL_REG;

/**
  * @brief Define the union ADC_STATUS_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_bsy_chn : 4;    /**< ADC busy channel */
        unsigned int adc_bsy : 1;        /**< Calibration mode selection */
        unsigned int reserved0 : 3;
        unsigned int rr_pointer : 5;     /**< Calibration mode selection */
        unsigned int reserved1 : 19;
    } BIT;
} volatile ADC_STATUS_REG;

/**
  * @brief Define the union ADC_SOC_PRICTL_SET_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int rr_set : 1;    /**< Polling pointer reset. Set to 1 to reset */
        unsigned int reserved0 : 31;
    } BIT;
} volatile ADC_SOC_PRICTL_SET_REG;

/**
  * @brief Define the union ADC_SAMPLE_MODE_REG. If this bit is set to 0, two SOC perform independent sampling.
  * 1: Two SOC perform synchronous sampling.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int simul_en0 : 1;  /**< Sampling mode selection for SOC0 and SOC1 */
        unsigned int simul_en1 : 1;  /**< Sampling mode selection for SOC2 and SOC3 */
        unsigned int simul_en2 : 1;  /**< Sampling mode selection for SOC4 and SOC5 */
        unsigned int simul_en3 : 1;  /**< Sampling mode selection for SOC6 and SOC7 */
        unsigned int simul_en4 : 1;  /**< Sampling mode selection for SOC8 and SOC9 */
        unsigned int simul_en5 : 1;  /**< Sampling mode selection for SOC10 and SOC11 */
        unsigned int simul_en6 : 1;  /**< Sampling mode selection for SOC12 and SOC13 */
        unsigned int simul_en7 : 1;  /**< Sampling mode selection for SOC14 and SOC15 */
        unsigned int reserved0 : 24;
    } BIT;
} volatile ADC_SAMPLE_MODE_REG;

/**
  * @brief Define the union ADC_INT_FLAG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int int1_flag : 1;  /**< Interrupt 1 status. Value 0: No interrupt. Value 1: Interrupt is generated */
        unsigned int int2_flag : 1;  /**< Interrupt 2 status. Value 0: No interrupt. Value 1: Interrupt is generated */
        unsigned int int3_flag : 1;  /**< Interrupt 3 status. Value 0: No interrupt. Value 1: Interrupt is generated */
        unsigned int int4_flag : 1;  /**< Interrupt 4 status. Value 0: No interrupt. Value 1: Interrupt is generated */
        unsigned int reserved0 : 28;
    } BIT;
} volatile ADC_INT_FLAG_REG;

/**
  * @brief Define the union ADC_EOC_FLAG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int eoc0_flag : 1;   /**< Status of eoc0. 0: conversion is not complete. 1: conversion is complete */
        unsigned int eoc1_flag : 1;   /**< Status of eoc1. 0: conversion is not complete. 1: conversion is complete */
        unsigned int eoc2_flag : 1;   /**< Status of eoc2. 0: conversion is not complete. 1: conversion is complete */
        unsigned int eoc3_flag : 1;   /**< Status of eoc3. 0: conversion is not complete. 1: conversion is complete */
        unsigned int eoc4_flag : 1;   /**< Status of eoc4. 0: conversion is not complete. 1: conversion is complete */
        unsigned int eoc5_flag : 1;   /**< Status of eoc5. 0: conversion is not complete. 1: conversion is complete */
        unsigned int eoc6_flag : 1;   /**< Status of eoc6. 0: conversion is not complete. 1: conversion is complete */
        unsigned int eoc7_flag : 1;   /**< Status of eoc7. 0: conversion is not complete. 1: conversion is complete */
        unsigned int eoc8_flag : 1;   /**< Status of eoc8. 0: conversion is not complete. 1: conversion is complete */
        unsigned int eoc9_flag : 1;   /**< Status of eoc9. 0: conversion is not complete. 1: conversion is complete */
        unsigned int eoc10_flag : 1;  /**< Status of eoc10. 0: conversion is not complete. 1: conversion is complete */
        unsigned int eoc11_flag : 1;  /**< Status of eoc11. 0: conversion is not complete. 1: conversion is complete */
        unsigned int eoc12_flag : 1;  /**< Status of eoc12. 0: conversion is not complete. 1: conversion is complete */
        unsigned int eoc13_flag : 1;  /**< Status of eoc13. 0: conversion is not complete. 1: conversion is complete */
        unsigned int eoc14_flag : 1;  /**< Status of eoc14. 0: conversion is not complete. 1: conversion is complete */
        unsigned int eoc15_flag : 1;  /**< Status of eoc15. 0: conversion is not complete. 1: conversion is complete */
        unsigned int reserved0 : 16;
    } BIT;
} volatile ADC_EOC_FLAG_REG;

/**
  * @brief Define the union ADC_INT_OVFL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int int1_ovflg : 1;        /**< Interrupt 1 overflow flag */
        unsigned int int2_ovflg : 1;        /**< Interrupt 2 overflow flag */
        unsigned int int3_ovflg : 1;        /**< Interrupt 3 overflow flag */
        unsigned int int4_ovflg : 1;        /**< Interrupt 4 overflow flag */
        unsigned int dma_req_ovflg : 1;     /**< DMA request overflow flag */
        unsigned int reserved0 : 27;
    } BIT;
} volatile ADC_INT_OVFL_REG;

/**
  * @brief Define the union ADC_EOC_OVFL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int eoc0_ovflg : 1;        /**< Overflow status of eoc0 */
        unsigned int eoc1_ovflg : 1;        /**< Overflow status of eoc1 */
        unsigned int eoc2_ovflg : 1;        /**< Overflow status of eoc2 */
        unsigned int eoc3_ovflg : 1;        /**< Overflow status of eoc3 */
        unsigned int eoc4_ovflg : 1;        /**< Overflow status of eoc4 */
        unsigned int eoc5_ovflg : 1;        /**< Overflow status of eoc5 */
        unsigned int eoc6_ovflg : 1;        /**< Overflow status of eoc6 */
        unsigned int eoc7_ovflg : 1;        /**< Overflow status of eoc7 */
        unsigned int eoc8_ovflg : 1;        /**< Overflow status of eoc8 */
        unsigned int eoc9_ovflg : 1;        /**< Overflow status of eoc9 */
        unsigned int eoc10_ovflg : 1;       /**< Overflow status of eoc10 */
        unsigned int eoc11_ovflg : 1;       /**< Overflow status of eoc11 */
        unsigned int eoc12_ovflg : 1;       /**< Overflow status of eoc12 */
        unsigned int eoc13_ovflg : 1;       /**< Overflow status of eoc13 */
        unsigned int eoc14_ovflg : 1;       /**< Overflow status of eoc14 */
        unsigned int eoc15_ovflg : 1;       /**< Overflow status of eoc15 */
        unsigned int reserved0 : 16;
    } BIT;
} volatile ADC_EOC_OVFL_REG;

/**
  * @brief Define the union ADC_INT1_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int int1_en : 1;       /**< Interrupt 1 enable */
        unsigned int int1_cont : 1;     /**< Interrupt 1 pulse enable */
        unsigned int int1_pos : 1;      /**< Early interrupt mode */
        unsigned int int1_oven : 1;     /**< Interrupt 1 overflow flag enable */
        unsigned int int1_offset : 12;  /**< Interrupt 1 offset enable */
        unsigned int int1_eoc_en : 16;  /**< Enables the eoc that can trigger interrupt 1 */
    } BIT;
} volatile ADC_INT1_CTRL_REG;

/**
  * @brief Define the union ADC_INT2_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int int2_en : 1;       /**< Interrupt 2 enable */
        unsigned int int2_cont : 1;     /**< Interrupt 2 pulse enable */
        unsigned int int2_pos : 1;      /**< Early interrupt mode */
        unsigned int int2_oven : 1;     /**< Interrupt 2 overflow flag enable */
        unsigned int int2_offset : 12;  /**< Interrupt 2 offset enable */
        unsigned int int2_eoc_en : 16;  /**< Enables the eoc that can trigger interrupt 2 */
    } BIT;
} volatile ADC_INT2_CTRL_REG;

/**
  * @brief Define the union ADC_INT3_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int int3_en : 1;       /**< Interrupt 3 enable */
        unsigned int int3_cont : 1;     /**< Interrupt 3 pulse enable */
        unsigned int int3_pos : 1;      /**< Early interrupt mode */
        unsigned int int3_oven : 1;     /**< Interrupt 3 overflow flag enable */
        unsigned int int3_offset : 12;  /**< Interrupt 3 offset enable */
        unsigned int int3_eoc_en : 16;  /**< Enables the eoc that can trigger interrupt 3 */
    } BIT;
} volatile ADC_INT3_CTRL_REG;

/**
  * @brief Define the union ADC_INT4_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int int4_en : 1;       /**< Interrupt 4 enable */
        unsigned int int4_cont : 1;     /**< Interrupt 4 pulse enable */
        unsigned int int4_pos : 1;      /**< Early interrupt mode */
        unsigned int int4_oven : 1;     /**< Interrupt 4 overflow flag enable */
        unsigned int int4_offset : 12;  /**< Interrupt 4 offset enable */
        unsigned int int4_eoc_en : 16;  /**< Enables the eoc that can trigger interrupt 4 */
    } BIT;
} volatile ADC_INT4_CTRL_REG;

/**
  * @brief Define the union ADC_INT_OFFSET_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int dma_sing_req_sel : 1;       /**< DMA single request enable */
        unsigned int dma_brst_req_sel : 1;       /**< DMA burst request enable */
        unsigned int dma_int_sel : 4;            /**< Selecting the DMA interrupt source */
        unsigned int reserved0 : 26;
    } BIT;
} volatile ADC_INT_OFFSET_REG;

/**
  * @brief Define the union ADC_SOC_PRICTL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soc_priority : 16;         /**< High-priority configuration */
        unsigned int reserved0 : 16;
    } BIT;
} volatile ADC_SOC_PRICTL_REG;

/**
  * @brief Define the union ADC_SOFT_TRIG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soc0_soft_trig : 1;        /**< SOC0 triggered by software */
        unsigned int soc1_soft_trig : 1;        /**< SOC1 triggered by software */
        unsigned int soc2_soft_trig : 1;        /**< SOC2 triggered by software */
        unsigned int soc3_soft_trig : 1;        /**< SOC3 triggered by software */
        unsigned int soc4_soft_trig : 1;        /**< SOC4 triggered by software */
        unsigned int soc5_soft_trig : 1;        /**< SOC5 triggered by software */
        unsigned int soc6_soft_trig : 1;        /**< SOC6 triggered by software */
        unsigned int soc7_soft_trig : 1;        /**< SOC7 triggered by software */
        unsigned int soc8_soft_trig : 1;        /**< SOC8 triggered by software */
        unsigned int soc9_soft_trig : 1;        /**< SOC9 triggered by software */
        unsigned int soc10_soft_trig : 1;       /**< SOC10 triggered by software */
        unsigned int soc11_soft_trig : 1;       /**< SOC11 triggered by software */
        unsigned int soc12_soft_trig : 1;       /**< SOC12 triggered by software */
        unsigned int soc13_soft_trig : 1;       /**< SOC13 triggered by software */
        unsigned int soc14_soft_trig : 1;       /**< SOC14 triggered by software */
        unsigned int soc15_soft_trig : 1;       /**< SOC15 triggered by software */
        unsigned int reserved0 : 16;
    } BIT;
} volatile ADC_SOFT_TRIG_REG;

/**
  * @brief Define the union ADC_TRIG_OVFL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soc0_trig_ovflg : 1;       /**< Trigger overflow flag of SOC0 */
        unsigned int soc1_trig_ovflg : 1;       /**< Trigger overflow flag of SOC1 */
        unsigned int soc2_trig_ovflg : 1;       /**< Trigger overflow flag of SOC2 */
        unsigned int soc3_trig_ovflg : 1;       /**< Trigger overflow flag of SOC3 */
        unsigned int soc4_trig_ovflg : 1;       /**< Trigger overflow flag of SOC4 */
        unsigned int soc5_trig_ovflg : 1;       /**< Trigger overflow flag of SOC5 */
        unsigned int soc6_trig_ovflg : 1;       /**< Trigger overflow flag of SOC6 */
        unsigned int soc7_trig_ovflg : 1;       /**< Trigger overflow flag of SOC7 */
        unsigned int soc8_trig_ovflg : 1;       /**< Trigger overflow flag of SOC8 */
        unsigned int soc9_trig_ovflg : 1;       /**< Trigger overflow flag of SOC9 */
        unsigned int soc10_trig_ovflg : 1;      /**< Trigger overflow flag of SOC10 */
        unsigned int soc11_trig_ovflg : 1;      /**< Trigger overflow flag of SOC11 */
        unsigned int soc12_trig_ovflg : 1;      /**< Trigger overflow flag of SOC12 */
        unsigned int soc13_trig_ovflg : 1;      /**< Trigger overflow flag of SOC13 */
        unsigned int soc14_trig_ovflg : 1;      /**< Trigger overflow flag of SOC14 */
        unsigned int soc15_trig_ovflg : 1;      /**< Trigger overflow flag of SOC15 */
        unsigned int reserved0 : 16;
    } BIT;
} volatile ADC_TRIG_OVFL_REG;

/**
  * @brief Define the union ADC_SOC0_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soc0_chsel : 4;        /**< Channel selection */
        unsigned int soc0_trig_sel : 5;     /**< Trigger source selection */
        unsigned int soc0_int_trig_sel : 2; /**< Feedback interrupt trigger configuration */
        unsigned int soc0_acqps : 7;        /**< Capacitor charging time */
        unsigned int soc0_rslt_rclr : 1;    /**< Read-clear enable */
        unsigned int reserved0 : 1;
        unsigned int soc0_sh_hold : 6;      /**< Sample and Hold Circuit Hold Time */
        unsigned int reserved1 : 6;
    } BIT;
} volatile ADC_SOC0_CTRL_REG;

/**
  * @brief Define the union ADC_SOC1_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soc1_chsel : 4;        /**< Channel selection */
        unsigned int soc1_trig_sel : 5;     /**< Trigger source selection */
        unsigned int soc1_int_trig_sel : 2; /**< Feedback interrupt trigger configuration */
        unsigned int soc1_acqps : 7;        /**< Capacitor charging time */
        unsigned int soc1_rslt_rclr : 1;    /**< Read-clear enable */
        unsigned int reserved0 : 1;
        unsigned int soc1_sh_hold : 6;      /**< Sample and Hold Circuit Hold Time */
        unsigned int reserved1 : 6;
    } BIT;
} volatile ADC_SOC1_CTRL_REG;

/**
  * @brief Define the union ADC_SOC2_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soc2_chsel : 4;        /**< Channel selection */
        unsigned int soc2_trig_sel : 5;     /**< Trigger source selection */
        unsigned int soc2_int_trig_sel : 2; /**< Feedback interrupt trigger configuration */
        unsigned int soc2_acqps : 7;        /**< Capacitor charging time */
        unsigned int soc2_rslt_rclr : 1;    /**< Read-clear enable */
        unsigned int reserved0 : 1;
        unsigned int soc2_sh_hold : 6;      /**< Sample and Hold Circuit Hold Time */
        unsigned int reserved1 : 6;
    } BIT;
} volatile ADC_SOC2_CTRL_REG;

/**
  * @brief Define the union ADC_SOC3_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soc3_chsel : 4;        /**< Channel selection */
        unsigned int soc3_trig_sel : 5;     /**< Trigger source selection */
        unsigned int soc3_int_trig_sel : 2; /**< Feedback interrupt trigger configuration */
        unsigned int soc3_acqps : 7;        /**< Capacitor charging time */
        unsigned int soc3_rslt_rclr : 1;    /**< Read-clear enable */
        unsigned int reserved0 : 1;
        unsigned int soc3_sh_hold : 6;      /**< Sample and Hold Circuit Hold Time */
        unsigned int reserved1 : 6;
    } BIT;
} volatile ADC_SOC3_CTRL_REG;

/**
  * @brief Define the union ADC_SOC4_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soc4_chsel : 4;        /**< Channel selection */
        unsigned int soc4_trig_sel : 5;     /**< Trigger source selection */
        unsigned int soc4_int_trig_sel : 2; /**< Feedback interrupt trigger configuration */
        unsigned int soc4_acqps : 7;        /**< Capacitor charging time */
        unsigned int soc4_rslt_rclr : 1;    /**< Read-clear enable */
        unsigned int reserved0 : 1;
        unsigned int soc4_sh_hold : 6;      /**< Sample and Hold Circuit Hold Time */
        unsigned int reserved1 : 6;
    } BIT;
} volatile ADC_SOC4_CTRL_REG;

/**
  * @brief Define the union ADC_SOC5_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soc5_chsel : 4;        /**< Channel selection */
        unsigned int soc5_trig_sel : 5;     /**< Trigger source selection */
        unsigned int soc5_int_trig_sel : 2; /**< Feedback interrupt trigger configuration */
        unsigned int soc5_acqps : 7;        /**< Capacitor charging time */
        unsigned int soc5_rslt_rclr : 1;    /**< Read-clear enable */
        unsigned int reserved0 : 1;
        unsigned int soc5_sh_hold : 6;      /**< Sample and Hold Circuit Hold Time */
        unsigned int reserved1 : 6;
    } BIT;
} volatile ADC_SOC5_CTRL_REG;

/**
  * @brief Define the union ADC_SOC6_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soc6_chsel : 4;        /**< Channel selection */
        unsigned int soc6_trig_sel : 5;     /**< Trigger source selection */
        unsigned int soc6_int_trig_sel : 2; /**< Feedback interrupt trigger configuration */
        unsigned int soc6_acqps : 7;        /**< Capacitor charging time */
        unsigned int soc6_rslt_rclr : 1;    /**< Read-clear enable */
        unsigned int reserved0 : 1;
        unsigned int soc6_sh_hold : 6;      /**< Sample and Hold Circuit Hold Time */
        unsigned int reserved1 : 6;
    } BIT;
} volatile ADC_SOC6_CTRL_REG;

/**
  * @brief Define the union ADC_SOC7_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soc7_chsel : 4;        /**< Channel selection */
        unsigned int soc7_trig_sel : 5;     /**< Trigger source selection */
        unsigned int soc7_int_trig_sel : 2; /**< Feedback interrupt trigger configuration */
        unsigned int soc7_acqps : 7;        /**< Capacitor charging time */
        unsigned int soc7_rslt_rclr : 1;    /**< Read-clear enable */
        unsigned int reserved0 : 1;
        unsigned int soc7_sh_hold : 6;      /**< Sample and Hold Circuit Hold Time */
        unsigned int reserved1 : 6;
    } BIT;
} volatile ADC_SOC7_CTRL_REG;

/**
  * @brief Define the union ADC_SOC8_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soc8_chsel : 4;        /**< Channel selection */
        unsigned int soc8_trig_sel : 5;     /**< Trigger source selection */
        unsigned int soc8_int_trig_sel : 2; /**< Feedback interrupt trigger configuration */
        unsigned int soc8_acqps : 7;        /**< Capacitor charging time */
        unsigned int soc8_rslt_rclr : 1;    /**< Read-clear enable */
        unsigned int reserved0 : 1;
        unsigned int soc8_sh_hold : 6;      /**< Sample and Hold Circuit Hold Time */
        unsigned int reserved1 : 6;
    } BIT;
} volatile ADC_SOC8_CTRL_REG;

/**
  * @brief Define the union ADC_SOC9_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soc9_chsel : 4;        /**< Channel selection */
        unsigned int soc9_trig_sel : 5;     /**< Trigger source selection */
        unsigned int soc9_int_trig_sel : 2; /**< Feedback interrupt trigger configuration */
        unsigned int soc9_acqps : 7;        /**< Capacitor charging time */
        unsigned int soc9_rslt_rclr : 1;    /**< Read-clear enable */
        unsigned int reserved0 : 1;
        unsigned int soc9_sh_hold : 6;      /**< Sample and Hold Circuit Hold Time */
        unsigned int reserved1 : 6;
    } BIT;
} volatile ADC_SOC9_CTRL_REG;

/**
  * @brief Define the union ADC_SOC10_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soc10_chsel : 4;        /**< Channel selection */
        unsigned int soc10_trig_sel : 5;     /**< Trigger source selection */
        unsigned int soc10_int_trig_sel : 2; /**< Feedback interrupt trigger configuration */
        unsigned int soc10_acqps : 7;        /**< Capacitor charging time */
        unsigned int soc10_rslt_rclr : 1;    /**< Read-clear enable */
        unsigned int reserved0 : 1;
        unsigned int soc10_sh_hold : 6;      /**< Sample and Hold Circuit Hold Time */
        unsigned int reserved1 : 6;
    } BIT;
} volatile ADC_SOC10_CTRL_REG;

/**
  * @brief Define the union ADC_SOC11_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soc11_chsel : 4;        /**< Channel selection */
        unsigned int soc11_trig_sel : 5;     /**< Trigger source selection */
        unsigned int soc11_int_trig_sel : 2; /**< Feedback interrupt trigger configuration */
        unsigned int soc11_acqps : 7;        /**< Capacitor charging time */
        unsigned int soc11_rslt_rclr : 1;    /**< Read-clear enable */
        unsigned int reserved0 : 1;
        unsigned int soc11_sh_hold : 6;      /**< Sample and Hold Circuit Hold Time */
        unsigned int reserved1 : 6;
    } BIT;
} volatile ADC_SOC11_CTRL_REG;

/**
  * @brief Define the union ADC_SOC12_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soc12_chsel : 4;        /**< Channel selection */
        unsigned int soc12_trig_sel : 5;     /**< Trigger source selection */
        unsigned int soc12_int_trig_sel : 2; /**< Feedback interrupt trigger configuration */
        unsigned int soc12_acqps : 7;        /**< Capacitor charging time */
        unsigned int soc12_rslt_rclr : 1;    /**< Read-clear enable */
        unsigned int reserved0 : 1;
        unsigned int soc12_sh_hold : 6;      /**< Sample and Hold Circuit Hold Time */
        unsigned int reserved1 : 6;
    } BIT;
} volatile ADC_SOC12_CTRL_REG;

/**
  * @brief Define the union ADC_SOC13_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soc13_chsel : 4;        /**< Channel selection */
        unsigned int soc13_trig_sel : 5;     /**< Trigger source selection */
        unsigned int soc13_int_trig_sel : 2; /**< Feedback interrupt trigger configuration */
        unsigned int soc13_acqps : 7;        /**< Capacitor charging time */
        unsigned int soc13_rslt_rclr : 1;    /**< Read-clear enable */
        unsigned int reserved0 : 1;
        unsigned int soc13_sh_hold : 6;      /**< Sample and Hold Circuit Hold Time */
        unsigned int reserved1 : 6;
    } BIT;
} volatile ADC_SOC13_CTRL_REG;

/**
  * @brief Define the union ADC_SOC14_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soc14_chsel : 4;        /**< Channel selection */
        unsigned int soc14_trig_sel : 5;     /**< Trigger source selection */
        unsigned int soc14_int_trig_sel : 2; /**< Feedback interrupt trigger configuration */
        unsigned int soc14_acqps : 7;        /**< Capacitor charging time */
        unsigned int soc14_rslt_rclr : 1;    /**< Read-clear enable */
        unsigned int reserved0 : 1;
        unsigned int soc14_sh_hold : 6;      /**< Sample and Hold Circuit Hold Time */
        unsigned int reserved1 : 6;
    } BIT;
} volatile ADC_SOC14_CTRL_REG;

/**
  * @brief Define the union ADC_SOC15_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int soc15_chsel : 4;        /**< Channel selection */
        unsigned int soc15_trig_sel : 5;     /**< Trigger source selection */
        unsigned int soc15_int_trig_sel : 2; /**< Feedback interrupt trigger configuration */
        unsigned int soc15_acqps : 7;        /**< Capacitor charging time */
        unsigned int soc15_rslt_rclr : 1;    /**< Read-clear enable */
        unsigned int reserved0 : 1;
        unsigned int soc15_sh_hold : 6;      /**< Sample and Hold Circuit Hold Time */
        unsigned int reserved1 : 6;
    } BIT;
} volatile ADC_SOC15_CTRL_REG;

/**
  * @brief Define the union ADC_PPB0_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ppb0_ch_sel : 4;        /**< PPB channel selection */
        unsigned int ppb0_en : 1;            /**< PPB Enable */
        unsigned int reserved0 : 27;
    } BIT;
} volatile ADC_PPB0_CTRL_REG;

/**
  * @brief Define the union ADC_PPB1_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ppb1_ch_sel : 4;        /**< PPB channel selection */
        unsigned int ppb1_en : 1;            /**< PPB Enable */
        unsigned int reserved0 : 27;
    } BIT;
} volatile ADC_PPB1_CTRL_REG;

/**
  * @brief Define the union ADC_PPB2_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ppb2_ch_sel : 4;        /**< PPB channel selection */
        unsigned int ppb2_en : 1;            /**< PPB Enable */
        unsigned int reserved0 : 27;
    } BIT;
} volatile ADC_PPB2_CTRL_REG;

/**
  * @brief Define the union ADC_PPB3_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ppb3_ch_sel : 4;        /**< PPB channel selection */
        unsigned int ppb3_en : 1;            /**< PPB Enable */
        unsigned int reserved0 : 27;
    } BIT;
} volatile ADC_PPB3_CTRL_REG;

/**
  * @brief Define the union ADC_PPB0_PPB1_DLY_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ppb0_dly_stamp : 16;   /**< Sampling delay of PPB1 */
        unsigned int ppb1_dly_stamp : 16;   /**< Sampling delay of PPB0 */
    } BIT;
} volatile ADC_PPB0_PPB1_DLY_REG;

/**
  * @brief Define the union ADC_PPB2_PPB3_DLY_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ppb2_dly_stamp : 16;   /**< Sampling delay of PPB2 */
        unsigned int ppb3_dly_stamp : 16;   /**< Sampling delay of PPB3 */
    } BIT;
} volatile ADC_PPB2_PPB3_DLY_REG;

/**
  * @brief Define the union ADC_ANA_CTRL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0 : 1;
        unsigned int ana_logic_mode : 1;    /**< ADC Analog Operating Modes */
        unsigned int reserved1 : 1;
        unsigned int adc_ana_gsh0 : 2;      /**< S/H0 Attenuation Configuration */
        unsigned int adc_ana_gsh1 : 2;      /**< S/H1 Attenuation Configuration */
        unsigned int adc_ana_dish : 2;      /**< OFFSET Calibration Control */
        unsigned int reserved2 : 23;
    } BIT;
} volatile ADC_ANA_CTRL_REG;

/**
  * @brief Define the union ADC_SAR_CTRL0_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0 : 3;
        unsigned int cap_start_index : 4;   /**< Selecte Calibration Capacitor */
        unsigned int reserved1 : 25;
    } BIT;
} volatile ADC_SAR_CTRL0_REG;

/**
  * @brief Define the union ADC_SAR_CTRL3_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cap_start : 1;         /**< Capacitive Calibration Trigger */
        unsigned int reserved0 : 31;
    } BIT;
} volatile ADC_SAR_CTRL3_REG;

/**
  * @brief Define the union ADC_OFF_CALI1_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ofst_calana_cfg : 6;   /**< Analog offset configured by software */
        unsigned int reserved0 : 1;
        unsigned int ofst_ana_sel : 1;      /**< Select analog offset input source */
        unsigned int ofst_digcal_cfg : 14;  /**< Software configuration digital offset */
        unsigned int ofst_digcal_sel : 1;   /**< Select digital offset input source */
        unsigned int reserved1 : 9;
    } BIT;
} volatile ADC_OFF_CALI1_REG;

/**
  * @brief Define the union ADC_OFF_CALI2_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_avg_start : 1;   /**< Starts to calculate the average offset value */
        unsigned int reserved0 : 31;
    } BIT;
} volatile ADC_OFF_CALI2_REG;

/**
  * @brief Define the union ADC_OFF_CALI4_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ofst_caliana : 6;          /**< Analog offset value */
        unsigned int reserved0 : 1;
        unsigned int ofst_calidig : 14;         /**< Digital offset */
        unsigned int adc_ofst_calib_done : 1;   /**< Offset calibration end flag */
        unsigned int adc_ofst_calib_error : 1;  /**< Offset calibration error flag */
        unsigned int reserved1 : 9;
    } BIT;
} volatile ADC_OFF_CALI4_REG;

/**
  * @brief Define the union ADC_CAP_CALI0_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0 : 21;
        unsigned int adc_weight_ini_sel : 1;    /**< Weight selection */
        unsigned int cap_cal1_finish : 1;       /**< Capacitor calibration completion flag */
        unsigned int reserved1 : 9;
    } BIT;
} volatile ADC_CAP_CALI0_REG;

/**
  * @brief Define the union ADC_DATA_PROCS0_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0 : 1;
        unsigned int gain_digcal_cfg : 13;  /**< Calibration gain value configured by software */
        unsigned int reserved1 : 18;
    } BIT;
} volatile ADC_DATA_PROCS0_REG;

/**
  * @brief Define the union ADC_ANA_PD_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_pwdnz : 1;  /**< ADC analog power supply */
        unsigned int reserved0 : 31;
    } BIT;
} volatile ADC_ANA_PD_REG;

/**
  * @brief Define the ADC resistor struct.
  */
typedef struct {
    ADC_RESULT0_REG ADC_RESULT0;                /**< Offset address: 0x00000000U */
    ADC_RESULT1_REG ADC_RESULT1;                /**< Offset address: 0x00000004U */
    ADC_RESULT2_REG ADC_RESULT2;                /**< Offset address: 0x00000008U */
    ADC_RESULT3_REG ADC_RESULT3;                /**< Offset address: 0x0000000CU */
    ADC_RESULT4_REG ADC_RESULT4;                /**< Offset address: 0x00000010U */
    ADC_RESULT5_REG ADC_RESULT5;                /**< Offset address: 0x00000014U */
    ADC_RESULT6_REG ADC_RESULT6;                /**< Offset address: 0x00000018U */
    ADC_RESULT7_REG ADC_RESULT7;                /**< Offset address: 0x0000001CU */
    ADC_RESULT8_REG ADC_RESULT8;                /**< Offset address: 0x00000020U */
    ADC_RESULT9_REG ADC_RESULT9;                /**< Offset address: 0x00000024U */
    ADC_RESULT10_REG ADC_RESULT10;              /**< Offset address: 0x00000028U */
    ADC_RESULT11_REG ADC_RESULT11;              /**< Offset address: 0x0000002CU */
    ADC_RESULT12_REG ADC_RESULT12;              /**< Offset address: 0x00000030U */
    ADC_RESULT13_REG ADC_RESULT13;              /**< Offset address: 0x00000034U */
    ADC_RESULT14_REG ADC_RESULT14;              /**< Offset address: 0x00000038U */
    ADC_RESULT15_REG ADC_RESULT15;              /**< Offset address: 0x0000003CU */
    ADC_CTRL_REG ADC_CTRL;                      /**< Offset address: 0x00000040U */
    ADC_STATUS_REG ADC_STATUS;                  /**< Offset address: 0x00000044U */
    ADC_SOC_PRICTL_SET_REG ADC_SOC_PRICTL_SET;  /**< Offset address: 0x00000048U */
    ADC_SAMPLE_MODE_REG ADC_SAMPLE_MODE;        /**< Offset address: 0x0000004CU */
    ADC_INT_FLAG_REG ADC_INT_FLAG;              /**< Offset address: 0x00000050U */
    ADC_EOC_FLAG_REG ADC_EOC_FLAG;              /**< Offset address: 0x00000054U */
    ADC_INT_OVFL_REG ADC_INT_OVFL;              /**< Offset address: 0x00000058U */
    ADC_EOC_OVFL_REG ADC_EOC_OVFL;              /**< Offset address: 0x0000005CU */
    ADC_INT1_CTRL_REG ADC_INT1_CTRL;            /**< Offset address: 0x00000060U */
    ADC_INT2_CTRL_REG ADC_INT2_CTRL;            /**< Offset address: 0x00000064U */
    ADC_INT3_CTRL_REG ADC_INT3_CTRL;            /**< Offset address: 0x00000068U */
    ADC_INT4_CTRL_REG ADC_INT4_CTRL;            /**< Offset address: 0x0000006CU */
    ADC_INT_OFFSET_REG ADC_INT_OFFSET;          /**< Offset address: 0x00000070U */
    ADC_SOC_PRICTL_REG ADC_SOC_PRICTL;          /**< Offset address: 0x00000074U */
    ADC_SOFT_TRIG_REG ADC_SOFT_TRIG;            /**< Offset address: 0x00000078U */
    ADC_TRIG_OVFL_REG ADC_TRIG_OVFL;            /**< Offset address: 0x0000007CU */
    ADC_SOC0_CTRL_REG ADC_SOC0_CTRL;            /**< Offset address: 0x00000080U */
    ADC_SOC1_CTRL_REG ADC_SOC1_CTRL;            /**< Offset address: 0x00000084U */
    ADC_SOC2_CTRL_REG ADC_SOC2_CTRL;            /**< Offset address: 0x00000088U */
    ADC_SOC3_CTRL_REG ADC_SOC3_CTRL;            /**< Offset address: 0x0000008CU */
    ADC_SOC4_CTRL_REG ADC_SOC4_CTRL;            /**< Offset address: 0x00000090U */
    ADC_SOC5_CTRL_REG ADC_SOC5_CTRL;            /**< Offset address: 0x00000094U */
    ADC_SOC6_CTRL_REG ADC_SOC6_CTRL;            /**< Offset address: 0x00000098U */
    ADC_SOC7_CTRL_REG ADC_SOC7_CTRL;            /**< Offset address: 0x0000009CU */
    ADC_SOC8_CTRL_REG ADC_SOC8_CTRL;            /**< Offset address: 0x000000A0U */
    ADC_SOC9_CTRL_REG ADC_SOC9_CTRL;            /**< Offset address: 0x000000A4U */
    ADC_SOC10_CTRL_REG ADC_SOC10_CTRL;          /**< Offset address: 0x000000A8U */
    ADC_SOC11_CTRL_REG ADC_SOC11_CTRL;          /**< Offset address: 0x000000ACU */
    ADC_SOC12_CTRL_REG ADC_SOC12_CTRL;          /**< Offset address: 0x000000B0U */
    ADC_SOC13_CTRL_REG ADC_SOC13_CTRL;          /**< Offset address: 0x000000B4U */
    ADC_SOC14_CTRL_REG ADC_SOC14_CTRL;          /**< Offset address: 0x000000B8U */
    unsigned char space0[4];
    ADC_SOC15_CTRL_REG ADC_SOC15_CTRL;          /**< Offset address: 0x000000C0U */
    ADC_PPB0_CTRL_REG ADC_PPB0_CTRL;            /**< Offset address: 0x000000C4U */
    ADC_PPB1_CTRL_REG ADC_PPB1_CTRL;            /**< Offset address: 0x000000C8U */
    unsigned char space1[4];
    ADC_PPB2_CTRL_REG ADC_PPB2_CTRL;            /**< Offset address: 0x000000D0U */
    ADC_PPB3_CTRL_REG ADC_PPB3_CTRL;            /**< Offset address: 0x000000D4U */
    ADC_PPB0_PPB1_DLY_REG ADC_PPB0_PPB1_DLY;    /**< Offset address: 0x000000D8U */
    ADC_PPB2_PPB3_DLY_REG ADC_PPB2_PPB3_DLY;    /**< Offset address: 0x000000DCU */
    ADC_ANA_CTRL_REG ADC_ANA_CTRL;              /**< Offset address: 0x000000E0U */
    ADC_SAR_CTRL0_REG ADC_SAR_CTRL0;            /**< Offset address: 0x000000E4U */
    unsigned char space2[8];
    ADC_SAR_CTRL3_REG ADC_SAR_CTRL3;            /**< Offset address: 0x000000F0U */
    unsigned char space3[8];
    ADC_OFF_CALI1_REG ADC_OFF_CALI1;            /**< Offset address: 0x000000FCU */
    ADC_OFF_CALI2_REG ADC_OFF_CALI2;            /**< Offset address: 0x00000100U */
    unsigned char space4[4];
    ADC_OFF_CALI4_REG ADC_OFF_CALI4;            /**< Offset address: 0x00000108U */
    ADC_CAP_CALI0_REG ADC_CAP_CALI0;            /**< Offset address: 0x0000010CU */
    unsigned char space5[188];
    ADC_DATA_PROCS0_REG ADC_DATA_PROCS0;        /**< Offset address: 0x000001CCU */
    unsigned char space6[4];
    ADC_ANA_PD_REG      ADC_ANA_PD;             /**< Offset address: 0x000001D4U */
} volatile ADC_RegStruct;
/**
  * @}
  */


/**
  * @defgroup ADC_Param_Def ADC Parameters Definition
  * @brief Description of ADC configuration parameters.
  * @{
  */
/**
  * @brief Synchronous sample group classification.
  * @details Syncsample group type:
  *          + Group1 -- Synchronous sample group: SOC0 and SOC1
  *          + Group2 -- Synchronous sample group: SOC2 and SOC3
  *          + Group3 -- Synchronous sample group: SOC4 and SOC5
  *          + Group4 -- Synchronous sample group: SOC6 and SOC7
  *          + Group5 -- Synchronous sample group: SOC8 and SOC9
  *          + Group6 -- Synchronous sample group: SOC10 and SOC11
  *          + Group7 -- Synchronous sample group: SOC12 and SOC13
  *          + Group8 -- Synchronous sample group: SOC14 and SOC15
  */
typedef enum {
    ADC_SYNCSAMPLE_GROUP_1 = 0x00000001U,
    ADC_SYNCSAMPLE_GROUP_2 = 0x00000002U,
    ADC_SYNCSAMPLE_GROUP_3 = 0x00000004U,
    ADC_SYNCSAMPLE_GROUP_4 = 0x00000008U,
    ADC_SYNCSAMPLE_GROUP_5 = 0x00000010U,
    ADC_SYNCSAMPLE_GROUP_6 = 0x00000020U,
    ADC_SYNCSAMPLE_GROUP_7 = 0x00000040U,
    ADC_SYNCSAMPLE_GROUP_8 = 0x00000080U
} ADC_SyncSampleGroup;

/**
  * @brief ADC sample input.
  * @details Input type:
  *          + ADC_CH_ADCINA0 -- ADCINA0 is converted, number 0
  *          + ADC_CH_ADCINA1 -- ADCINA1 is converted, number 1
  *          + ADC_CH_ADCINA2 -- ADCINA2 is converted, number 2
  *          + ADC_CH_ADCINA3 -- ADCINA3 is converted, number 3
  *          + ADC_CH_ADCINA4 -- ADCINA4 is converted, number 4
  *          + ADC_CH_ADCINA5 -- ADCINA5 is converted, number 5
  *          + ADC_CH_ADCINA6 -- ADCINA6 is converted, number 6
  *          + ADC_CH_ADCINA7 -- ADCINA7 is converted, number 7
  *          + ADC_CH_ADCINB0 -- ADCINB0 is converted, number 8
  *          + ADC_CH_ADCINB1 -- ADCINB1 is converted, number 9
  *          + ADC_CH_ADCINB2 -- ADCINB2 is converted, number 10
  *          + ADC_CH_ADCINB3 -- ADCINB3 is converted, number 11
  *          + ADC_CH_ADCINB4 -- ADCINB4 is converted, number 12
  *          + ADC_CH_ADCINB5 -- ADCINB5 is converted, number 13
  *          + ADC_CH_ADCINB6 -- ADCINB6 is converted, number 14
  *          + ADC_CH_ADCINB7 -- ADCINB7 is converted, number 15
  */
typedef enum {
    ADC_CH_ADCINA0 = 0x00000000U,
    ADC_CH_ADCINA1 = 0x00000001U,
    ADC_CH_ADCINA2 = 0x00000002U,
    ADC_CH_ADCINA3 = 0x00000003U,
    ADC_CH_ADCINA4 = 0x00000004U,
    ADC_CH_ADCINA5 = 0x00000005U,
    ADC_CH_ADCINA6 = 0x00000006U,
    ADC_CH_ADCINA7 = 0x00000007U,
    ADC_CH_ADCINB0 = 0x00000008U,
    ADC_CH_ADCINB1 = 0x00000009U,
    ADC_CH_ADCINB2 = 0x0000000AU,
    ADC_CH_ADCINB3 = 0x0000000BU,
    ADC_CH_ADCINB4 = 0x0000000CU,
    ADC_CH_ADCINB5 = 0x0000000DU,
    ADC_CH_ADCINB6 = 0x0000000EU,
    ADC_CH_ADCINB7 = 0x0000000FU
} ADC_Input;

/**
  * @brief ADC SOC(start of conversion) classification.
  */
typedef enum {
    ADC_SOC_NUM0 = 0x00000000U,
    ADC_SOC_NUM1 = 0x00000001U,
    ADC_SOC_NUM2 = 0x00000002U,
    ADC_SOC_NUM3 = 0x00000003U,
    ADC_SOC_NUM4 = 0x00000004U,
    ADC_SOC_NUM5 = 0x00000005U,
    ADC_SOC_NUM6 = 0x00000006U,
    ADC_SOC_NUM7 = 0x00000007U,
    ADC_SOC_NUM8 = 0x00000008U,
    ADC_SOC_NUM9 = 0x00000009U,
    ADC_SOC_NUM10 = 0x0000000AU,
    ADC_SOC_NUM11 = 0x0000000BU,
    ADC_SOC_NUM12 = 0x0000000CU,
    ADC_SOC_NUM13 = 0x0000000DU,
    ADC_SOC_NUM14 = 0x0000000EU,
    ADC_SOC_NUM15 = 0x0000000FU
} ADC_SOCNumber;

/**
  * @brief ADC four interrupt classification.
  * @details Interrupt type:
  *          + ADC_INT_NUMBER1 -- ADCINT1 interrupt
  *          + ADC_INT_NUMBER2 -- ADCINT2 interrupt
  *          + ADC_INT_NUMBER3 -- ADCINT3 interrupt
  *          + ADC_INT_NUMBER4 -- ADCINT4 interrupt
  */
typedef enum {
    ADC_INT_NUMBER1 = 0x00000000U,
    ADC_INT_NUMBER2 = 0x00000001U,
    ADC_INT_NUMBER3 = 0x00000002U,
    ADC_INT_NUMBER4 = 0x00000003U
} ADC_IntNumber;

/**
  * @brief ADC supports three internal interrupt feedback trigger soc sample.
  * @details Interrupt trigger source type:
  *          + ADC_TRIGSOC_NONEINT -- NONE
  *          + ADC_TRIGSOC_INT1 -- ADCINT1 interrupt trigger soc
  *          + ADC_TRIGSOC_INT2 -- ADCINT2 interrupt trigger soc
  *          + ADC_TRIGSOC_INT3 -- ADCINT3 interrupt trigger soc
  */
typedef enum {
    ADC_TRIGSOC_NONEINT = 0x00000000U,
    ADC_TRIGSOC_INT1 = 0x00000001U,
    ADC_TRIGSOC_INT2 = 0x00000002U,
    ADC_TRIGSOC_INT3 = 0x00000003U
} ADC_IntTrigSoc;

/**
  * @brief ADC supports peripherals trigger source.
  */
typedef enum {
    ADC_TRIGSOC_NONEPERIPH = 0x00000000U,
    ADC_TRIGSOC_APT0_SOCA = 0x00000001U,
    ADC_TRIGSOC_APT0_SOCB = 0x00000002U,
    ADC_TRIGSOC_APT1_SOCA = 0x00000003U,
    ADC_TRIGSOC_APT1_SOCB = 0x00000004U,
    ADC_TRIGSOC_APT2_SOCA = 0x00000005U,
    ADC_TRIGSOC_APT2_SOCB = 0x00000006U,
    ADC_TRIGSOC_APT3_SOCA = 0x00000007U,
    ADC_TRIGSOC_APT3_SOCB = 0x00000008U,
    ADC_TRIGSOC_APT4_SOCA = 0x00000009U,
    ADC_TRIGSOC_APT4_SOCB = 0x0000000AU,
    ADC_TRIGSOC_APT5_SOCA = 0x0000000BU,
    ADC_TRIGSOC_APT5_SOCB = 0x0000000CU,
    ADC_TRIGSOC_APT6_SOCA = 0x0000000DU,
    ADC_TRIGSOC_APT6_SOCB = 0x0000000EU,
    ADC_TRIGSOC_APT7_SOCA = 0x0000000FU,
    ADC_TRIGSOC_APT7_SOCB = 0x00000010U,
    ADC_TRIGSOC_APT8_SOCA = 0x00000011U,
    ADC_TRIGSOC_APT8_SOCB = 0x00000012U,
    ADC_TRIGSOC_TIMER0 = 0x00000013U,
    ADC_TRIGSOC_TIMER1 = 0x00000014U,
    ADC_TRIGSOC_TIMER2 = 0x00000015U,
    ADC_TRIGSOC_TIMER3 = 0x00000016U,
    ADC_TRIGSOC_GPIO = 0x00000017U,
} ADC_PeriphTrigSoc;

/**
  * @brief The type of software trigger source.
  */
typedef enum {
    ADC_TRIGSOC_NONESOFT = 0x00000000U,
    ADC_TRIGSOC_SOFT = 0x00000001U,
} ADC_SoftTrigSoc;

/**
  * @brief The type of interrupt.
  * @details Interrupt type:
  *          + ADC_INTMODE_NORMAL   -- ADCINT normal interrupt
  *          + ADC_INTMODE_EARLYINT -- ADCINT early interrupt
  */
typedef enum {
    ADC_INTMODE_NORMAL = 0x00000000U,
    ADC_INTMODE_EARLYINT = 0x00000001U
} ADC_IntMode;

/**
  * @brief The type of DMA request.
  * @details DMA request type:
  *          + ADC_DMA_SINGLEREQ -- single request
  *          + ADC_DMA_BURSTREQ  -- burst request
  */
typedef enum {
    ADC_DMA_SINGLEREQ = 0x00000000U,
    ADC_DMA_BURSTREQ = 0x00000001U
} ADC_DMARequestType;

/**
  * @brief The type of DMA vref power.
  * @details Internal referencevoltage power type:
  *          + ADC_VREF_2P0V -- 2.0v, select when VDDA < 2.9v
  *          + ADC_VREF_2P5V -- 2.5v, select when VDDA >= 2.9v
  */
typedef enum {
    ADC_VREF_2P0V = 0x00000000U,
    ADC_VREF_2P5V = 0x00000001U
} ADC_VrefType;

/**
  * @brief The priority mode of SOCs sample simultaneously.
  * @details Priority mode:
  *          + ADC_PRIMODE_ALL_ROUND -- Round robin mode is used for all
  *          + ADC_PRIMODE_SOC0      -- SOC0 higher priority, others in round
  *          + ADC_PRIMODE_TO_SOC1   -- SOC 0-1 higher priority, others in round
  *          + ADC_PRIMODE_TO_SOC2   -- SOC 0-2 higher priority, others in round
  *          + ADC_PRIMODE_TO_SOC3   -- SOC 0-3 higher priority, others in round
  *          + ADC_PRIMODE_TO_SOC4   -- SOC 0-4 higher priority, others in round
  *          + ADC_PRIMODE_TO_SOC5   -- SOC 0-5 higher priority, others in round
  *          + ADC_PRIMODE_TO_SOC6   -- SOC 0-6 higher priority, others in round
  *          + ADC_PRIMODE_TO_SOC7   -- SOC 0-7 higher priority, others in round
  *          + ADC_PRIMODE_TO_SOC8   -- SOC 0-8 higher priority, others in round
  *          + ADC_PRIMODE_TO_SOC9   -- SOC 0-9 higher priority, others in round
  *          + ADC_PRIMODE_TO_SOC10  -- SOC 0-10 higher priority, others in round
  *          + ADC_PRIMODE_TO_SOC11  -- SOC 0-11 higher priority, others in round
  *          + ADC_PRIMODE_TO_SOC12  -- SOC 0-12 higher priority, others in round
  *          + ADC_PRIMODE_TO_SOC13  -- SOC 0-13 higher priority, others in round
  *          + ADC_PRIMODE_TO_SOC14  -- SOC 0-14 higher priority, others in round
  *          + ADC_PRIMODE_ALL_PRIORITY -- SOC 0-15 higher priority, others in round
  */
typedef enum {
    ADC_PRIMODE_ALL_ROUND = 0x00000000U,
    ADC_PRIMODE_SOC0 = 0x00000001U,
    ADC_PRIMODE_TO_SOC1 = 0x00000003U,
    ADC_PRIMODE_TO_SOC2 = 0x00000007U,
    ADC_PRIMODE_TO_SOC3 = 0x0000000FU,
    ADC_PRIMODE_TO_SOC4 = 0x0000001FU,
    ADC_PRIMODE_TO_SOC5 = 0x0000003FU,
    ADC_PRIMODE_TO_SOC6 = 0x0000007FU,
    ADC_PRIMODE_TO_SOC7 = 0x000000FFU,
    ADC_PRIMODE_TO_SOC8 = 0x000001FFU,
    ADC_PRIMODE_TO_SOC9 = 0x000003FFU,
    ADC_PRIMODE_TO_SOC10 = 0x000007FFU,
    ADC_PRIMODE_TO_SOC11 = 0x00000FFFU,
    ADC_PRIMODE_TO_SOC12 = 0x00001FFFU,
    ADC_PRIMODE_TO_SOC13 = 0x00003FFFU,
    ADC_PRIMODE_TO_SOC14 = 0x00007FFFU,
    ADC_PRIMODE_ALL_PRIORITY = 0x0000FFFFU
} ADC_PriorityMode;

/**
  * @brief The number of PPB(post processing block).
  */
typedef enum {
    ADC_PPB_NUM0 = 0x00000000U,
    ADC_PPB_NUM1 = 0x00000001U,
    ADC_PPB_NUM2 = 0x00000002U,
    ADC_PPB_NUM3 = 0x00000003U
} ADC_PPBNumber;

/**
  * @brief The mode of SOCs finish sample and conversion.
  * @details Priority mode:
  *          + ADC_SOCFINISH_NONE -- Interruption and DMA are not reported when sampling is complete
  *          + ADC_SOCFINISH_DMA  -- DMA is reported when sampling is complete
  *          + ADC_SOCFINISH_INT1 -- Interruption 1 is reported when sampling is complete
  *          + ADC_SOCFINISH_INT2 -- Interruption 2 is reported when sampling is complete
  *          + ADC_SOCFINISH_INT3 -- Interruption 3 is reported when sampling is complete
  *          + ADC_SOCFINISH_INT4 -- Interruption 4 is reported when sampling is complete
  */
typedef enum {
    ADC_SOCFINISH_NONE = 0x00000001U,
    ADC_SOCFINISH_DMA = 0x00000002U,
    ADC_SOCFINISH_INT1 = 0x00000003U,
    ADC_SOCFINISH_INT2 = 0x00000004U,
    ADC_SOCFINISH_INT3 = 0x00000005U,
    ADC_SOCFINISH_INT4 = 0x00000006U
}ADC_SOCFinishMode;

/**
  * @brief The type of interrupt call back functions.
  */
typedef enum {
    ADC_CALLBACK_INT1 = 0x00000000U,
    ADC_CALLBACK_INT2 = 0x00000001U,
    ADC_CALLBACK_INT3 = 0x00000002U,
    ADC_CALLBACK_INT4 = 0x00000003U,
    ADC_CALLBACK_DMA = 0x000000004U,
    ADC_CALLBACK_INTOVER = 0x00000005U,
    ADC_CALLBACK_DMAOVER = 0x00000006U,
    ADC_CALLBACK_TRIGOVER = 0x00000007U,
    ADC_CALLBACK_EOCOVER = 0x00000008U,
    ADC_CALLBACK_DMAERROR = 0x00000009U,
} ADC_CallbackFunType;

/**
  * @brief The type of gain.
  */
typedef enum {
    ADC_GAIN_1 = 0x00000000U,
    ADC_GAIN_0P75 = 0x00000001U,
    ADC_GAIN_0P6 = 0x00000002U,
} ADC_GainType;

/*
 * Each bit indicates the software triggering status of the SOC. The value 1 indicates enable
 * and the value 0 indicates disable.
 */
typedef union {
    unsigned int softTrigVal;
    struct {
        unsigned int trigSoc0 : 1;
        unsigned int trigSoc1 : 1;
        unsigned int trigSoc2 : 1;
        unsigned int trigSoc3 : 1;
        unsigned int trigSoc4 : 1;
        unsigned int trigSoc5 : 1;
        unsigned int trigSoc6 : 1;
        unsigned int trigSoc7 : 1;
        unsigned int trigSoc8 : 1;
        unsigned int trigSoc9 : 1;
        unsigned int trigSoc10 : 1;
        unsigned int trigSoc11 : 1;
        unsigned int trigSoc12 : 1;
        unsigned int trigSoc13 : 1;
        unsigned int trigSoc14 : 1;
        unsigned int trigSoc15 : 1;
        unsigned int reserved : 16;
    } BIT;
} ADC_SoftMultiTrig;

/**
  * @brief The definition of ADC overflow status.
  */
typedef union {
    unsigned int eocOver : 16;
    unsigned int trigOver : 16;
    unsigned int intOver : 16;
} ADC_OverState;

extern unsigned int HAL_ADC_ActiveCalibrateRetEx(ADC_RegStruct * const adcx, unsigned int soc,
                                                 unsigned int originalRet);

/**
  * @brief The definition of SOC parameter structure.
  * Note:
  * (1) sampleTotalTime: Sum of hold time and sampling time, unit: adc_clk. sampleTotalTime affects the sampling rate.
  * (2) sampleHoldTime: hold time, unit: adc_clk. Generally, the default value 2 is used.
  * (3) The sampling time is related to the input circuit model. Sampling time need meet input circuit design.
  * The model and formula of sampling time are as follows. Please refer to the Analog-to-Digital Converter (ADC) Section
  * of the Chip Data Sheet for details.
  * sampling time = (ln(2^n / setting_error) - ln(Cin / Csh_s)) * ((Rs + Ron_s) * Csh_s + Rs * Cin)
  *  +-----^^^^^^-------+--*----^^^^^---- +----------+
  *  |       Rs         |       Ron_s     |          |
  * VIN            Cin ===         Csh_s ===      ADC_CONVERT
  *  |                  |                 |          |
  *  +------------------+--*-------------------------+
  */
typedef struct {
    ADC_Input           adcInput;           /**< SOC specified input */
    unsigned int        sampleTotalTime;    /**< SOC specified input sample total time */
    unsigned int        sampleHoldTime;     /**< SOC specified input hold time */
    ADC_SoftTrigSoc     softTrigSource;     /**< SOC specified input software trigger mode */
    ADC_PeriphTrigSoc   periphTrigSource;   /**< SOC specified input periph trigger source */
    ADC_IntTrigSoc      intTrigSource;      /**< SOC specified input interrupt trigger source */
    ADC_SOCFinishMode   finishMode;         /**< SOC specified input mode of finishing sample and conversion */
} SOC_Param;

/**
  * @brief The definition of synchronous sampling parameter structure.
  * Note:
  * (1) sampleTotalTime: Sum of hold time and sampling time, unit: adc_clk. sampleTotalTime affects the sampling rate.
  * (2) sampleHoldTime: hold time, unit: adc_clk. Generally, the default value 2 is used.
  * (3) The sampling time is related to the input circuit model. Sampling time need meet input circuit design.
  * The model and formula of sampling time are as follows. Please refer to the Analog-to-Digital Converter (ADC) Section
  * of the Chip Data Sheet for details.
  * sampling time = (ln(2^n / setting_error) - ln(Cin / Csh_s)) * ((Rs + Ron_s) * Csh_s + Rs * Cin)
  *  +-----^^^^^^-------+--*----^^^^^---- +----------+
  *  |       Rs         |       Ron_s     |          |
  * VIN            Cin ===         Csh_s ===      ADC_CONVERT
  *  |                  |                 |          |
  *  +------------------+--*-------------------------+
  */
typedef struct {
    ADC_Input           ChannelA;           /**< The selection range is ADC_CH_ADCINA0 to ADC_CH_ADCINA7 */
    ADC_Input           ChannelB;           /**< The selection range is ADC_CH_ADCINB0 to ADC_CH_ADCINB7 */
    ADC_SyncSampleGroup group;              /**< Syncsample group param */
    unsigned int        sampleTotalTime;    /**< Generally, the default value 3 is selected */
    unsigned int        sampleHoldTime;     /**< Generally, the default value 2 is selected */
    ADC_SoftTrigSoc     softTrigSource;     /**< Software trigger mode */
    ADC_PeriphTrigSoc   periphTrigSource;   /**< Peripheral trigger source selection */
    ADC_IntTrigSoc      intTrigSource;      /**< Internal interrupt trigger source select */
    ADC_SOCFinishMode   finishMode;         /**< Sample finish mode */
    unsigned int       *dmaSaveAddr;        /**< When DMA is used, configure destination address for data transfer */
    DMA_ChannelParam   *dmaParam;           /**< When DMA is used, configure DMA transmission parameters */
} SOC_SyncParam;

/**
  * @brief The definition of extend handle structure.
  */
typedef struct _ADC_ExtendHandle {
    ADC_VrefType            vrefBuf;            /**< ADC vrefBuf type, configuration options affected by VDDA */
} ADC_ExtendHandle;

/**
  * @brief The definition of callback.
  */
typedef struct {
    void (* Int1FinishCallBack)(void *handle); /**< ADC interrupt one complete callback function for users */
    void (* Int2FinishCallBack)(void *handle); /**< ADC interrupt two complete callback function for users */
    void (* Int3FinishCallBack)(void *handle); /**< ADC interrupt three complete callback function for users */
    void (* Int4FinishCallBack)(void *handle); /**< ADC interrupt four complete callback function for users */
    void (* DmaFinishCallBack)(void *handle);  /**< ADC DMA finish callback function for users */
    void (* IntxOverCallBack)(void *handle);   /**< ADC interrupt overflow callback function for users */
    void (* DmaOverCallBack)(void *handle);    /**< ADC DMA overflow callback function for users */
    void (* TrigOverCallBack)(void *handle);   /**< ADC trigger overflow callback function for users */
    void (* EocOverCallBack)(void *handle);    /**< ADC eoc overflow callback function for users */
    void (* DmaErrorCallBack)(void *handle);   /**< ADC DMA transmission error callback function for users */
} ADC_UserCallBack;
/**
  * @}
  */

/* ADC DCL Functions */
/**
  * @brief Check ADC Operating Frequency.
  * @param  freq Configured frequency.
  * @retval bool
  */
static inline bool IsADCWorkFreq(unsigned int freq)
{
    return (freq >= ADC_MIN_FREQ) && (freq <= ADC_MAX_FREQ);
}

/**
  * @brief Check ADC synchronous sample group parameter.
  * @param group Group id of SOC.
  * @retval bool
  */
static inline bool IsADCSyncGroup(ADC_SyncSampleGroup group)
{
    return (group >= ADC_SYNCSAMPLE_GROUP_1) && (group <= ADC_SYNCSAMPLE_GROUP_8);
}

/**
  * @brief Check ADC sample input. Input are classified into group A inputs and group B inputs.
  * @param input Number of input.
  * @retval bool
  */
static inline bool IsADCSampleChannel(ADC_Input input)
{
    return (input >= ADC_CH_ADCINA0) && (input <= ADC_CH_ADCINB7);
}

/**
  * @brief Check sample input in group A.
  * @param input Number of input.
  * @retval bool
  */
static inline bool IsADCGroupAChannel(ADC_Input input)
{
    return (input >= ADC_CH_ADCINA0) && (input <= ADC_CH_ADCINA7);
}

/**
  * @brief Check sample input in group B.
  * @param input Number of input.
  * @retval bool
  */
static inline bool IsADCGroupBChannel(ADC_Input input)
{
    return (input >= ADC_CH_ADCINB0) && (input <= ADC_CH_ADCINB7);
}

/**
  * @brief Check ADC SOC(start of conversion). Each SOC selects a unique input for sampling. The sample parameters
  * are configured through the SOC.
  * @param soc Number of SOC.
  * @retval bool
  */
static inline bool IsADCSOCx(ADC_SOCNumber soc)
{
    return (soc >= ADC_SOC_NUM0) && (soc <= ADC_SOC_NUM15);
}

/**
  * @brief Check ADC interrupt parameter.
  * @param intx Number of interrupt.
  * @retval bool
  */
static inline bool IsADCIntx(ADC_IntNumber intx)
{
    return (intx >= ADC_INT_NUMBER1) && (intx <= ADC_INT_NUMBER4);
}

/**
  * @brief Check SOC interrupt trigger source.
  * @param intTrig Type of interrupt trigger source.
  * @retval bool
  */
static inline bool IsADCIntTrig(ADC_IntTrigSoc intTrig)
{
    return (intTrig >= ADC_TRIGSOC_NONEINT) && (intTrig <= ADC_TRIGSOC_INT3);
}

/**
  * @brief Check SOC interrupt trigger mode.
  * @param intMode Type of interrupt mode.
  * @retval bool
  */
static inline bool IsADCIntMode(ADC_IntMode intMode)
{
    return (intMode == ADC_INTMODE_NORMAL) || (intMode == ADC_INTMODE_EARLYINT);
}

/**
  * @brief Check SOC peripherals trigger source.
  * @param periphTrig Type of peripherals trigger source.
  * @retval bool
  */
static inline bool IsADCPeriphTrig(ADC_PeriphTrigSoc periphTrig)
{
    return (periphTrig >= ADC_TRIGSOC_NONEPERIPH) && (periphTrig <= ADC_TRIGSOC_GPIO);
}

/**
  * @brief Check SOC software trigger source.
  * @param softTrig Type of software trigger source.
  * @retval bool
  */
static inline bool IsADCSoftTrig(ADC_SoftTrigSoc softTrig)
{
    return (softTrig == ADC_TRIGSOC_NONESOFT) || (softTrig == ADC_TRIGSOC_SOFT);
}

/**
  * @brief Check SOC software trigger source.
  * @param dmaType Type of software trigger source.
  * @retval bool
  */
static inline bool IsADCReqDMAType(ADC_DMARequestType dmaType)
{
    return (dmaType == ADC_DMA_SINGLEREQ) || (dmaType == ADC_DMA_BURSTREQ);
}

/**
  * @brief Check mode of completion of SOC sample
  * @param mode Type of completion.
  * @retval bool
  */
static inline bool IsADCFinishMode(ADC_SOCFinishMode mode)
{
    return (mode >= ADC_SOCFINISH_NONE) && (mode <= ADC_SOCFINISH_INT4);
}

/**
  * @brief Check ADC sample priority parameter.
  * @param mode Priority mode of SOC.
  * @retval bool
  */
static inline bool IsADCPriorityMode(ADC_PriorityMode mode)
{
    return (mode >= ADC_PRIMODE_ALL_ROUND) && (mode <= ADC_PRIMODE_ALL_PRIORITY);
}

/**
  * @brief Check time of capacitor hold time after charging.
  * @param shHold Priority mode of SOC.
  * @retval bool
  */
static inline bool IsADCHodeTime(unsigned int shHold)
{
    return (shHold >= HOLDTIME_MIN) && (shHold <= HOLDTIME_MAX);
}

/**
  * @brief Check time of capacitor charging.
  * @param acqps Time of capacitor charging.
  * @retval bool
  */
static inline bool IsADCChargeTime(unsigned int acqps)
{
    return (acqps >= ACQPSTIME_MIN) && (acqps <= ACQPSTIME_MAX);
}

/**
  * @brief Check adc vrefbuf.
  * @param vrefBuf Type of vrefbuf.
  * @retval bool
  */
static inline bool IsADCVrefBufType(ADC_VrefType vrefBuf)
{
    return (vrefBuf == ADC_VREF_2P0V) || (vrefBuf == ADC_VREF_2P5V);
}

/**
  * @brief Check adc gain of SH0, SH1.
  * @param gain Type of gain of SH0, SH1.
  * @retval bool
  */
static inline bool IsADCGainType(ADC_GainType gain)
{
    return (gain >= ADC_GAIN_1) || (gain <= ADC_GAIN_0P6);
}

/**
  * @brief ADC Configuration Synchronous Sample Group.
  * @param adcx ADC register base address.
  * @param group Number of Synchronous sample group.
  * @retval None.
  */
static inline void DCL_ADC_SetSyncSample(ADC_RegStruct * const adcx, ADC_SyncSampleGroup group)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSyncGroup(group));
    adcx->ADC_SAMPLE_MODE.reg |= (unsigned int)group;
}

/**
  * @brief ADC uses ONE-SHOT sampling mode.
  * @param adcx ADC register base address.
  * @retval None.
  */
static inline void DCL_ADC_EnableOneShot(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_CTRL.BIT.one_shot = BASE_CFG_ENABLE;
}

/**
  * @brief ADC can not use ONE-SHOT sampling mode.
  * @param adcx ADC register base address.
  * @retval None.
  */
static inline void DCL_ADC_DisableOneShot(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_CTRL.BIT.one_shot = BASE_CFG_DISABLE;
}

/**
  * @brief Configuring the ADC eraly interrupt offset.
  * @param adcx ADC register base address.
  * @param intx Number of ADC interrupt controller, @ref ADC_IntNumber.
  * @param offset Offset of eraly interrupt.
  * @retval None.
  */
static inline void DCL_ADC_SetIntxOffset(ADC_RegStruct * const adcx, ADC_IntNumber intx, unsigned int offset)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCIntx(intx) && (offset < 4096));  /* The upper limit of offset is 4096 */
    uintptr_t addr = (uintptr_t)(void *)adcx;
    addr = (addr + 0x60 + (unsigned int)intx * 4);  /* Register base address difference 4, 0x60 CTRL_REG addr */
    ADC_INT1_CTRL_REG *intReg;
    intReg = (ADC_INT1_CTRL_REG *)(void *)addr;
    intReg->BIT.int1_offset = offset;
    intReg->BIT.int1_en = BASE_CFG_ENABLE;
}

/**
  * @brief Configure the interrupt mode: normal interrupt and early interrupt.
  * @param adcx ADC register base address.
  * @param intx Number of ADC interrupt controller, @ref ADC_IntNumber.
  * @param mode ADC interrupt Mode.
  * @retval None.
  */
static inline void DCL_ADC_SetIntxMode(ADC_RegStruct * const adcx, ADC_IntNumber intx, ADC_IntMode mode)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCIntx(intx) && IsADCIntMode(mode));
    uintptr_t addr = (uintptr_t)(void *)adcx;
    addr = (addr + 0x60 + (unsigned int)intx * 4);  /* Register base address difference 4, 0x60 CTRL_REG addr */
    ADC_INT1_CTRL_REG *intReg;
    intReg = (ADC_INT1_CTRL_REG *)(void *)addr;
    intReg->BIT.int1_pos = mode;
}

/**
  * @brief Configuring the interrupt source used by the SOC.
  * @param adcx ADC register base address.
  * @param intx Number of ADC interrupt controller, @ref ADC_IntNumber.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @retval None.
  */
static inline void DCL_ADC_SetSOCxBlindIntx(ADC_RegStruct * const adcx, ADC_IntNumber intx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCIntx(intx) && IsADCSOCx(socx));
    unsigned int shiftBit = (unsigned int)socx + 16;  /* Offset 16 bits configuration */
    uintptr_t addr = (uintptr_t)(void *)adcx;
    addr = (addr + 0x60 + (unsigned int)intx * 4);  /* Register base address difference 4, 0x60 CTRL_REG addr */
    ADC_INT1_CTRL_REG *intReg;
    intReg = (ADC_INT1_CTRL_REG *)(void *)addr;
    intReg->reg |= (1U << shiftBit);
}

/**
  * @brief Enable ADC interrupt.
  * @param adcx ADC register base address.
  * @param intx Number of ADC interrupt controller, @ref ADC_IntNumber.
  * @retval None.
  */
static inline void DCL_ADC_EnableIntx(ADC_RegStruct * const adcx, ADC_IntNumber intx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCIntx(intx));
    uintptr_t addr = (uintptr_t)(void *)adcx;
    addr = (addr + 0x60 + (unsigned int)intx * 4);  /* Register base address difference 4, 0x60 CTRL_REG addr */
    ADC_INT1_CTRL_REG *intReg;
    intReg = (ADC_INT1_CTRL_REG *)(void *)addr;
    intReg->BIT.int1_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable ADC interrupt.
  * @param adcx ADC register base address.
  * @param intx Number of ADC interrupt controller, @ref ADC_IntNumber.
  * @retval None.
  */
static inline void DCL_ADC_DisableIntx(ADC_RegStruct * const adcx, ADC_IntNumber intx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCIntx(intx));
    uintptr_t addr = (uintptr_t)(void *)adcx;
    addr = (addr + 0x60 + (unsigned int)intx * 4);  /* Register base address difference 4, 0x60 CTRL_REG addr */
    ADC_INT1_CTRL_REG *intReg;
    intReg = (ADC_INT1_CTRL_REG *)(void *)addr;
    intReg->BIT.int1_en = BASE_CFG_DISABLE;
}

/**
  * @brief Obtains the interrupt state.
  * @param adcx ADC register base address.
  * @retval unsigned int.
  */
static inline unsigned int DCL_ADC_GetStateOfIntx(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    return adcx->ADC_INT_FLAG.reg;
}

/**
  * @brief ADC clear interruption.
  * @param adcx ADC register base address.
  * @param intx Number of ADC interrupt controller, @ref ADC_IntNumber.
  * @retval None.
  */
static inline void DCL_ADC_ClearIntx(ADC_RegStruct * const adcx, ADC_IntNumber intx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCIntx(intx));
    adcx->ADC_INT_FLAG.reg = (1U << (unsigned int)intx);
}

/**
  * @brief Enable ADC interrupt pulse.
  * @param adcx ADC register base address.
  * @param intx Number of ADC interrupt controller, @ref ADC_IntNumber.
  * @retval None.
  */
static inline void DCL_ADC_EnableIntxPulse(ADC_RegStruct * const adcx, ADC_IntNumber intx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCIntx(intx));
    uintptr_t addr = (uintptr_t)(void *)adcx;
    addr = (addr + 0x60 + (unsigned int)intx * 4);  /* Register base address difference 4, 0x60 CTRL_REG addr */
    ADC_INT1_CTRL_REG *intReg;
    intReg = (ADC_INT1_CTRL_REG *)(void *)addr;
    intReg->BIT.int1_cont = BASE_CFG_ENABLE;
}

/**
  * @brief Disable ADC interrupt pulse.
  * @param adcx ADC register base address.
  * @param intx Number of ADC interrupt controller, @ref ADC_IntNumber.
  * @retval None.
  */
static inline void DCL_ADC_DisableIntxPulse(ADC_RegStruct * const adcx, ADC_IntNumber intx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCIntx(intx));
    uintptr_t addr = (uintptr_t)(void *)adcx;
    addr = (addr + 0x60 + (unsigned int)intx * 4);  /* Register base address difference 4, 0x60 CTRL_REG addr */
    ADC_INT1_CTRL_REG *intReg;
    intReg = (ADC_INT1_CTRL_REG *)(void *)addr;
    intReg->BIT.int1_cont = BASE_CFG_DISABLE;
}
/**
  * @brief Calculate the base address of the SOC registers with different numbers.This interface is invoked by the DCL,
  * and parameter verification has been completed at the DCL functions.
  * @param adcx ADC register base address.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @retval addr, the base address of the SOC registers.
  */
static unsigned int ADC_GetCTRLAddr(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    unsigned int addr;
    if (socx == ADC_SOC_NUM15) {
        addr = (uintptr_t)(void *)&(adcx->ADC_SOC15_CTRL);
    } else {
        addr = (uintptr_t)(void *)&(adcx->ADC_SOC0_CTRL);
        addr += ((unsigned int)socx * 4);   /* Register base address difference 4 */
    }
    return addr;
}

/**
  * @brief Configure the corresponding input for the SOC.
  * @param adcx ADC register base address.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @param input ADC input, @ref ADC_Input.
  * @retval None.
  */
static inline void DCL_ADC_SOCxSelcetChannel(ADC_RegStruct * const adcx, ADC_SOCNumber socx, ADC_Input input)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx) && IsADCSampleChannel(input));
    ADC_SOC0_CTRL_REG *soc = NULL;
    unsigned int addr = ADC_GetCTRLAddr(adcx, socx); /* get the Address After Translation */
    soc = (ADC_SOC0_CTRL_REG *)(void *)(uintptr_t)addr;
    soc->reg &= 0xFFFFFFF0;
    soc->reg |= (unsigned int)input;
}

/**
  * @brief Configure the trigger source for the SOC.
  * @param adcx ADC register base address.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @param trig Source of trigger, @ref ADC_PeriphTrigSoc.
  * @retval None.
  */
static inline void DCL_ADC_SOCxSelcetTrigSource(ADC_RegStruct * const adcx, ADC_SOCNumber socx, ADC_PeriphTrigSoc trig)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx) && IsADCPeriphTrig(trig));
    unsigned int addr = ADC_GetCTRLAddr(adcx, socx);
    ADC_SOC0_CTRL_REG *soc = NULL;
    soc = (ADC_SOC0_CTRL_REG *)(void *)(uintptr_t)addr;
    soc->reg &= 0xFFFFFE0F;
    soc->reg |= ((unsigned int)trig << 4);  /* Registers 4 through 8 bit to configure the SOC trigger source */
}

/**
  * @brief Configure the feedback interrupt for the SOC.
  * @param adcx ADC register base address.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @param Intxtrig Source of trigger, @ref ADC_IntTrigSoc.
  * @retval None.
  */
static inline void DCL_ADC_SOCxSelcetIntxTrig(ADC_RegStruct * const adcx, ADC_SOCNumber socx, ADC_IntTrigSoc Intxtrig)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx) && IsADCIntTrig(Intxtrig));
    unsigned int addr = ADC_GetCTRLAddr(adcx, socx);
    ADC_SOC0_CTRL_REG *soc = NULL;
    soc = (ADC_SOC0_CTRL_REG *)(void *)(uintptr_t)addr;
    soc->reg &= 0xFFFFF9FF;
    soc->reg |= ((unsigned int)Intxtrig << 9);  /* Shift left 9 bit to configure the SOC interrupt trigger source */
}

/**
  * @brief Configure the capacitor charging time for the SOC.
  * @param adcx ADC register base address.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @param acqps Capacitor charging time.
  * @retval None.
  */
static inline void DCL_ADC_SOCxSetAcqps(ADC_RegStruct * const adcx, ADC_SOCNumber socx, unsigned int acqps)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    ADC_PARAM_CHECK_NO_RET(acqps >= 3 && acqps <= 127);  /* The value of acqps ranges from 3 to 127 */
    unsigned int addr = ADC_GetCTRLAddr(adcx, socx);
    ADC_SOC0_CTRL_REG *soc = NULL;
    soc = (ADC_SOC0_CTRL_REG *)(void *)(uintptr_t)addr;
    soc->reg &= 0xFFFC07FF;
    soc->reg |= (acqps << 11);  /* Registers 11 bit to configure the capacitor charging time */
}

/**
  * @brief Configure the capacitor hold time for the SOC.
  * @param adcx ADC register base address.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @param shHold Charge hold time.
  * @retval None.
  */
static inline void DCL_ADC_SOCxSetShHold(ADC_RegStruct * const adcx, ADC_SOCNumber socx, unsigned int shHold)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    ADC_PARAM_CHECK_NO_RET(shHold >= 2 && shHold <= 28);  /* The value of shHold ranges from 2 to 28 */
    unsigned int addr = ADC_GetCTRLAddr(adcx, socx);
    ADC_SOC0_CTRL_REG *soc = NULL;
    soc = (ADC_SOC0_CTRL_REG *)(void *)(uintptr_t)addr;
    soc->reg &= 0xFC0FFFFF;
    soc->reg |= (shHold << 20);  /* Registers 20 bit to configure the capacitor charge hold time */
}

/**
  * @brief Enables the read-clear function of the result register for SOC.
  * @param adcx ADC register base address.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @retval None.
  */
static inline void DCL_ADC_SOCxEnableRC(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    ADC_SOC0_CTRL_REG *soc = NULL;
    unsigned int addr = ADC_GetCTRLAddr(adcx, socx); /* get the Address After Translation */
    soc = (ADC_SOC0_CTRL_REG *)(void *)(uintptr_t)addr;
    soc->BIT.soc0_rslt_rclr = BASE_CFG_ENABLE;
}

/**
  * @brief Disable the read-clear function of the result register for SOC.
  * @param adcx ADC register base address.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @retval None.
  */
static inline void DCL_ADC_SOCxDisableRC(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    unsigned int addr = ADC_GetCTRLAddr(adcx, socx); /* get the Address After Translation */
    ADC_SOC0_CTRL_REG *soc = NULL;
    soc = (ADC_SOC0_CTRL_REG *)(void *)(uintptr_t)addr;
    soc->BIT.soc0_rslt_rclr = BASE_CFG_DISABLE;
}

/**
  * @brief ADC uses software-triggered sampling.
  * @param adcx ADC register base address.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @retval None.
  */
static inline void DCL_ADC_SOCxSoftTrigger(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    adcx->ADC_SOFT_TRIG.reg |= (1U << (unsigned int)socx);
}

/**
  * @brief Multiple inputs trigger software sampling.
  * @param adcx ADC register base address.
  * @param val The val bits range from 0 to 0xFFFF. Writing 1 indicates triggering.
  * @retval None.
  */
static inline void DCL_ADC_SOCxMultiSoftTrigger(ADC_RegStruct * const adcx, unsigned int val)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(val <= 0xFFFF);  /* The value of val ranges from 0 to 0xFFFF */
    adcx->ADC_SOFT_TRIG.reg = val;
}

/**
  * @brief Configuring the SOC Priority.
  * @param adcx ADC register base address.
  * @param priorityMode Mode of SOC priority, @ref ADC_PriorityMode.
  * @retval None.
  */
static inline void DCL_ADC_SOCxSetPriority(ADC_RegStruct * const adcx, ADC_PriorityMode priorityMode)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCPriorityMode(priorityMode));
    adcx->ADC_SOC_PRICTL.reg = priorityMode;
}

/**
  * @brief Get current poll pointer. This pointer holds the last converted poll SOC.
  * @param adcx ADC register base address.
  * @retval None.
  */
static inline unsigned int DCL_ADC_QueryPollPoint(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    return adcx->ADC_STATUS.BIT.rr_pointer;
}

/**
  * @brief The poll pointer is reset by software. After the software is set to 1, the rr_pointer is set to 16.
  * @param adcx ADC register base address.
  * @retval None.
  */
static inline void DCL_ADC_ResetPollPoint(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_SOC_PRICTL_SET.BIT.rr_set = BASE_CFG_SET;
}


/**
  * @brief Set the specified SOC as the DAM request trigger source.
  * @param adcx ADC register base address.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @retval None.
  */
static inline void DCL_ADC_DMARequestSource(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    adcx->ADC_INT_OFFSET.BIT.dma_int_sel = socx;
}

/**
  * @brief ADC enable DMA burst request.
  * @param adcx ADC register base address.
  * @retval None.
  */
static inline void DCL_ADC_EnableDMABurstReq(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_INT_OFFSET.BIT.dma_brst_req_sel = BASE_CFG_ENABLE;
}

/**
  * @brief ADC disable DMA burst request.
  * @param adcx ADC register base address.
  * @retval None.
  */
static inline void DCL_ADC_DisableDMABurstReq(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_INT_OFFSET.BIT.dma_brst_req_sel = BASE_CFG_DISABLE;
}

/**
  * @brief ADC enable DMA single request.
  * @param adcx ADC register base address.
  * @retval None.
  */
static inline void DCL_ADC_EnableDMASingleReq(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_INT_OFFSET.BIT.dma_sing_req_sel = BASE_CFG_ENABLE;
}

/**
  * @brief ADC disable DMA single request.
  * @param adcx ADC register base address.
  * @retval None.
  */
static inline void DCL_ADC_DisableDMASingleReq(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_INT_OFFSET.BIT.dma_sing_req_sel = BASE_CFG_DISABLE;
}

/**
  * @brief Configure post processing module(PPB) for the SOC.
  * @param adcx ADC register base address.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @retval None.
  */
static inline void DCL_ADC_SOCxSelectPPB0(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    adcx->ADC_PPB0_CTRL.reg |= (unsigned int)socx;
}

/**
  * @brief Configure post processing module(PPB) for the SOC.
  * @param adcx ADC register base address.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @retval None.
  */
static inline void DCL_ADC_SOCxSelectPPB1(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    adcx->ADC_PPB1_CTRL.reg |= (unsigned int)socx;
}

/**
  * @brief Configure post processing module(PPB) for the SOC.
  * @param adcx ADC register base address.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @retval None.
  */
static inline void DCL_ADC_SOCxSelectPPB2(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    adcx->ADC_PPB2_CTRL.reg |= (unsigned int)socx;
}

/**
  * @brief Configure post processing module(PPB) for the SOC.
  * @param adcx ADC register base address.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @retval None.
  */
static inline void DCL_ADC_SOCxSelectPPB3(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    adcx->ADC_PPB3_CTRL.reg |= (unsigned int)socx;
}

/**
  * @brief Enable specified PPB.
  * @param adcx ADC register base address.
  * @retval None.
  */
static inline void DCL_ADC_EnablePPB0(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_PPB0_CTRL.BIT.ppb0_en = BASE_CFG_ENABLE;
}

/**
  * @brief Enable specified PPB.
  * @param adcx ADC register base address.
  * @retval None.
  */
static inline void DCL_ADC_EnablePPB1(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_PPB1_CTRL.BIT.ppb1_en = BASE_CFG_ENABLE;
}

/**
  * @brief Enable specified PPB.
  * @param adcx ADC register base address.
  * @retval None.
  */
static inline void DCL_ADC_EnablePPB2(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_PPB2_CTRL.BIT.ppb2_en = BASE_CFG_ENABLE;
}

/**
  * @brief Enable specified PPB.
  * @param adcx ADC register base address.
  * @retval None.
  */
static inline void DCL_ADC_EnablePPB3(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_PPB3_CTRL.BIT.ppb3_en = BASE_CFG_ENABLE;
}

/**
  * @brief Disable specified PPB.
  * @param adcx ADC register base address.
  * @retval None.
  */
static inline void DCL_ADC_DisablePPB0(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_PPB0_CTRL.BIT.ppb0_en = BASE_CFG_DISABLE;
}

/**
  * @brief Disable specified PPB.
  * @param adcx ADC register base address.
  * @retval None.
  */
static inline void DCL_ADC_DisablePPB1(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_PPB1_CTRL.BIT.ppb1_en = BASE_CFG_DISABLE;
}

/**
  * @brief Disable specified PPB.
  * @param adcx ADC register base address.
  * @retval None.
  */
static inline void DCL_ADC_DisablePPB2(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_PPB2_CTRL.BIT.ppb2_en = BASE_CFG_DISABLE;
}

/**
  * @brief Disable specified PPB.
  * @param adcx ADC register base address.
  * @retval None.
  */
static inline void DCL_ADC_DisablePPB3(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_PPB3_CTRL.BIT.ppb3_en = BASE_CFG_DISABLE;
}

/**
  * @brief Read ADC conversion result.
  * @param adcx ADC register base address.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @retval None.
  */
static inline unsigned int DCL_ADC_ReadSOCxResult(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_ASSERT_PARAM(IsADCSOCx(socx) == true);
    ADC_RESULT0_REG *result;
    uintptr_t addr = (uintptr_t)(void *)adcx;
    /* The address interval of the result register is 4 */
    result = (ADC_RESULT0_REG *)(void *)(addr + 4 * (unsigned int)socx);
    if (g_trimEnable == false) {
        return result->reg;
    }
    return HAL_ADC_ActiveCalibrateRetEx(adcx, socx, result->reg);
}

/**
  * @brief Setting vrefbuf Parameters.
  * @param vrefBuf Type of verfbuf, @ref ADC_VrefType.
  * @retval None.
  */
static inline void DCL_ADC0_SetVrefBuf(ADC_VrefType vrefBuf)
{
    ADC_ASSERT_PARAM(IsADCVrefBufType(vrefBuf));
    unsigned int val = BASE_CFG_ENABLE;
    val |= ((unsigned int)vrefBuf << 4);  /* Shift left 4 bit to configurate the type of vrefbuf */
    SYSCTRL1->ADC0_VREF_CTRL.reg |= val;
}

/**
  * @brief Setting vrefbuf Parameters.
  * @param vrefBuf Type of verfbuf, @ref ADC_VrefType.
  * @retval None.
  */
static inline void DCL_ADC1_SetVrefBuf(ADC_VrefType vrefBuf)
{
    ADC_ASSERT_PARAM(IsADCVrefBufType(vrefBuf));
    unsigned int val = BASE_CFG_ENABLE;
    val |= ((unsigned int)vrefBuf << 4);  /* Shift left 4 bit to configurate the type of vrefbuf */
    SYSCTRL1->ADC1_VREF_CTRL.reg |= val;
}

/**
  * @brief Setting vrefbuf Parameters.
  * @param vrefBuf Type of verfbuf, @ref ADC_VrefType.
  * @retval None.
  */
static inline void DCL_ADC2_SetVrefBuf(ADC_VrefType vrefBuf)
{
    ADC_ASSERT_PARAM(IsADCVrefBufType(vrefBuf));
    unsigned int val = BASE_CFG_ENABLE;
    val |= ((unsigned int)vrefBuf << 4);  /* Shift left 4 bit to configurate the type of vrefbuf */
    SYSCTRL1->ADC2_VREF_CTRL.reg |= val;
}

/**
  * @brief Analog Power Enable.
  * @param adcx ADC register base address.
  * @retval None.
  */
static inline void DCL_ADC_PowerEnable(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_ANA_PD.BIT.adc_pwdnz = BASE_CFG_ENABLE;
}

/**
  * @brief Analog Power Disable.
  * @param adcx ADC register base address.
  * @retval None.
  */
static inline void DCL_ADC_PowerDisable(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_ANA_PD.BIT.adc_pwdnz = BASE_CFG_DISABLE;
}

/**
  * @brief Obtain the SOC conversion status.
  * @param adcx ADC register base address.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @retval unsigned int, Not 0: Finish, 0: Not finish.
  */
static inline unsigned int DCL_ADC_GetConvState(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_ASSERT_PARAM(IsADCSOCx(socx) == true);
    unsigned int ret = adcx->ADC_EOC_FLAG.reg;
    return (ret & ((1U << (unsigned int)socx)));
}

/**
  * @brief Clears the SOC completion flag.
  * @param adcx ADC register base address.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @retval None.
  */
static inline void DCL_ADC_ResetConvState(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_ASSERT_PARAM(IsADCSOCx(socx) == true);
    unsigned int ret = (1U << (unsigned int)socx);
    adcx->ADC_EOC_FLAG.reg = ret;
}

/**
  * @brief Obtains the input ID currently configured for the SOC.
  * @param adcx ADC register base address.
  * @param socx Number of SOC, @ref ADC_SOCNumber.
  * @retval unisgned int, input number of soc.
  */
static inline unsigned int DCL_ADC_GetSOCxInputChannel(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_ASSERT_PARAM(IsADCSOCx(socx) == true);
    unsigned int addr = ADC_GetCTRLAddr(adcx, socx);  /* Obtains the SOC base address  */
    ADC_SOC0_CTRL_REG *soc = NULL;
    soc = (ADC_SOC0_CTRL_REG *)(void *)(uintptr_t)addr;
    return soc->BIT.soc0_chsel;
}

/**
  * @brief Setting the gain of SH0.
  * @param adcx ADC register base address.
  * @param gain Type of gain, @ref ADC_GainType.
  * @retval None.
  */
static inline void DCL_ADC_SetGainOfSampleHold0(ADC_RegStruct * const adcx, ADC_GainType gain)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_ASSERT_PARAM(IsADCGainType(gain) == true);
    adcx->ADC_ANA_CTRL.BIT.adc_ana_gsh0 = gain;
}

/**
  * @brief Setting the gain of SH1.
  * @param adcx ADC register base address.
  * @param gain Type of gain, @ref ADC_GainType.
  * @retval None.
  */
static inline void DCL_ADC_SetGainOfSampleHold1(ADC_RegStruct * const adcx, ADC_GainType gain)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_ASSERT_PARAM(IsADCGainType(gain) == true);
    adcx->ADC_ANA_CTRL.BIT.adc_ana_gsh1 = gain;
}

/**
  * @brief Obtains the gain of SH0.
  * @param adcx ADC register base address.
  * @retval unsigned int, gain of SH0.
  */
static inline unsigned int DCL_ADC_GetGainOfSampleHold0(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    return adcx->ADC_ANA_CTRL.BIT.adc_ana_gsh0;
}

/**
  * @brief Obtains the gain of SH1.
  * @param adcx ADC register base address.
  * @retval unsigned int, gain of SH1.
  */
static inline unsigned int DCL_ADC_GetGainOfSampleHold1(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    return adcx->ADC_ANA_CTRL.BIT.adc_ana_gsh1;
}

/**
  * @brief Enter ADC Calibration Mode.
  * @param adcx ADC register base address.
  * @retval void.
  */
static inline void DCL_ADC_EnterCalibrationMode(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_CTRL.BIT.adc_cal_en = 0x01;
    adcx->ADC_CTRL.BIT.adc_cal_mode = 0x01;
    adcx->ADC_ANA_CTRL.BIT.ana_logic_mode = 0x01;
}

/**
  * @brief Enter ADC Work Mode.
  * @param adcx ADC register base address.
  * @retval void.
  */
static inline void DCL_ADC_EnterWorkMode(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_CTRL.BIT.adc_cal_en = 0x00;
    adcx->ADC_CTRL.BIT.adc_cal_mode = 0x00;
    adcx->ADC_ANA_CTRL.BIT.ana_logic_mode = 0x00;
}

/**
  * @brief Configure the calibration capacitor and weight and enable calibration.
  * @param adcx ADC register base address.
  * @retval void.
  */
static inline void DCL_ADC_StartCalibrationMode(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_CAP_CALI0.BIT.adc_weight_ini_sel = 0x01; /* Select AutoCalibration Weight Value */
    adcx->ADC_SAR_CTRL0.BIT.cap_start_index = 0xF;     /* Select Calibration Capacitor */
    adcx->ADC_SAR_CTRL3.BIT.cap_start = 0x01;          /* Select Calibration Capacitor */
}

/**
  * @brief Clear the interrupt overflow flag.
  * @param adcx ADC register base address.
  * @retval void.
  */
static inline void DCL_ADC_ClearIntOver(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    unsigned int intOver = adcx->ADC_INT_OVFL.reg;
    adcx->ADC_INT_OVFL.reg = intOver;
}

/**
  * @brief Clear the EOC overflow flag.
  * @param adcx ADC register base address.
  * @retval void.
  */
static inline void DCL_ADC_ClearEocOver(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    unsigned int eocOver = adcx->ADC_EOC_OVFL.reg;
    adcx->ADC_EOC_OVFL.reg = eocOver;
}

/**
  * @brief Clear the trigger overflow flag.
  * @param adcx ADC register base address.
  * @retval void.
  */
static inline void DCL_ADC_ClearTrigOver(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    unsigned int trigOver = adcx->ADC_TRIG_OVFL.reg;
    adcx->ADC_TRIG_OVFL.reg = trigOver;
}

/**
  * @}
  */

/**
  * @}
  */
#endif  /* McuMagicTag_ADC_IP_H */