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
 * @details This file provides DCL functions to manage ADC and Definition of specific parameters.
 *          + Definition of ADC configuration parameters.
 *          + ADC register mapping structure.
 *          + Parameters check functions.
 *          + Direct configuration layer interface.
 */

/* Macro definitions */
#ifndef McuMagicTag_ADC_IP_H
#define McuMagicTag_ADC_IP_H

#include "baseinc.h"

#define SOC_MAX_NUM 16
#define INT_MAX_NUM 4
#define DMA_OVER_MASK 0x00010000
#define INT_OVER_MASK 0x0000FFFF
#define EVENT_TYPE 16

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
 * @brief ADC_IP: adc_v1.
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
  * @brief Define the union ADC_SOC0_CFG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_soc0_ch_sel : 5;       /**< Channel selection */
        unsigned int cfg_soc0_samptime_sel : 4; /**< Sampling Period Selection */
        unsigned int cfg_soc0_trig_sel : 5;     /**< Trigger source selection */
        unsigned int cfg_soc0_cont_en : 1;      /**< Continuous conversion mode enable bit */
        unsigned int reserved0 : 17;
    } BIT;
} volatile ADC_SOC0_CFG_REG;

/**
  * @brief Define the union ADC_SOC1_CFG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_soc1_ch_sel : 5;       /**< Channel selection */
        unsigned int cfg_soc1_samptime_sel : 4; /**< Sampling Period Selection */
        unsigned int cfg_soc1_trig_sel : 5;     /**< Trigger source selection */
        unsigned int cfg_soc1_cont_en : 1;      /**< Continuous conversion mode enable bit */
        unsigned int reserved0 : 17;
    } BIT;
} volatile ADC_SOC1_CFG_REG;

/**
  * @brief Define the union ADC_SOC2_CFG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_soc2_ch_sel : 5;       /**< Channel selection */
        unsigned int cfg_soc2_samptime_sel : 4; /**< Sampling Period Selection */
        unsigned int cfg_soc2_trig_sel : 5;     /**< Trigger source selection */
        unsigned int cfg_soc2_cont_en : 1;      /**< Continuous conversion mode enable bit */
        unsigned int reserved0 : 17;
    } BIT;
} volatile ADC_SOC2_CFG_REG;

/**
  * @brief Define the union ADC_SOC3_CFG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_soc3_ch_sel : 5;       /**< Channel selection */
        unsigned int cfg_soc3_samptime_sel : 4; /**< Sampling Period Selection */
        unsigned int cfg_soc3_trig_sel : 5;     /**< Trigger source selection */
        unsigned int cfg_soc3_cont_en : 1;      /**< Continuous conversion mode enable bit */
        unsigned int reserved0 : 17;
    } BIT;
} volatile ADC_SOC3_CFG_REG;

/**
  * @brief Define the union ADC_SOC4_CFG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_soc4_ch_sel : 5;       /**< Channel selection */
        unsigned int cfg_soc4_samptime_sel : 4; /**< Sampling Period Selection */
        unsigned int cfg_soc4_trig_sel : 5;     /**< Trigger source selection */
        unsigned int cfg_soc4_cont_en : 1;      /**< Continuous conversion mode enable bit */
        unsigned int reserved0 : 17;
    } BIT;
} volatile ADC_SOC4_CFG_REG;

/**
  * @brief Define the union ADC_SOC5_CFG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_soc5_ch_sel : 5;       /**< Channel selection */
        unsigned int cfg_soc5_samptime_sel : 4; /**< Sampling Period Selection */
        unsigned int cfg_soc5_trig_sel : 5;     /**< Trigger source selection */
        unsigned int cfg_soc5_cont_en : 1;      /**< Continuous conversion mode enable bit */
        unsigned int reserved0 : 17;
    } BIT;
} volatile ADC_SOC5_CFG_REG;

/**
  * @brief Define the union ADC_SOC6_CFG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_soc6_ch_sel : 5;       /**< Channel selection */
        unsigned int cfg_soc6_samptime_sel : 4; /**< Sampling Period Selection */
        unsigned int cfg_soc6_trig_sel : 5;     /**< Trigger source selection */
        unsigned int cfg_soc6_cont_en : 1;      /**< Continuous conversion mode enable bit */
        unsigned int reserved0 : 17;
    } BIT;
} volatile ADC_SOC6_CFG_REG;

/**
  * @brief Define the union ADC_SOC7_CFG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_soc7_ch_sel : 5;       /**< Channel selection */
        unsigned int cfg_soc7_samptime_sel : 4; /**< Sampling Period Selection */
        unsigned int cfg_soc7_trig_sel : 5;     /**< Trigger source selection */
        unsigned int cfg_soc7_cont_en : 1;      /**< Continuous conversion mode enable bit */
        unsigned int reserved0 : 17;
    } BIT;
} volatile ADC_SOC7_CFG_REG;

/**
  * @brief Define the union ADC_SOC8_CFG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_soc8_ch_sel : 5;       /**< Channel selection */
        unsigned int cfg_soc8_samptime_sel : 4; /**< Sampling Period Selection */
        unsigned int cfg_soc8_trig_sel : 5;     /**< Trigger source selection */
        unsigned int cfg_soc8_cont_en : 1;      /**< Continuous conversion mode enable bit */
        unsigned int reserved0 : 17;
    } BIT;
} volatile ADC_SOC8_CFG_REG;

/**
  * @brief Define the union ADC_SOC9_CFG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_soc9_ch_sel : 5;       /**< Channel selection */
        unsigned int cfg_soc9_samptime_sel : 4; /**< Sampling Period Selection */
        unsigned int cfg_soc9_trig_sel : 5;     /**< Trigger source selection */
        unsigned int cfg_soc9_cont_en : 1;      /**< Continuous conversion mode enable bit */
        unsigned int reserved0 : 17;
    } BIT;
} volatile ADC_SOC9_CFG_REG;

/**
  * @brief Define the union ADC_SOC10_CFG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_soc10_ch_sel : 5;       /**< Channel selection */
        unsigned int cfg_soc10_samptime_sel : 4; /**< Sampling Period Selection */
        unsigned int cfg_soc10_trig_sel : 5;     /**< Trigger source selection */
        unsigned int cfg_soc10_cont_en : 1;      /**< Continuous conversion mode enable bit */
        unsigned int reserved0 : 17;
    } BIT;
} volatile ADC_SOC10_CFG_REG;

/**
  * @brief Define the union ADC_SOC11_CFG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_soc11_ch_sel : 5;       /**< Channel selection */
        unsigned int cfg_soc11_samptime_sel : 4; /**< Sampling Period Selection */
        unsigned int cfg_soc11_trig_sel : 5;     /**< Trigger source selection */
        unsigned int cfg_soc11_cont_en : 1;      /**< Continuous conversion mode enable bit */
        unsigned int reserved0 : 17;
    } BIT;
} volatile ADC_SOC11_CFG_REG;

/**
  * @brief Define the union ADC_SOC12_CFG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_soc12_ch_sel : 5;       /**< Channel selection */
        unsigned int cfg_soc12_samptime_sel : 4; /**< Sampling Period Selection */
        unsigned int cfg_soc12_trig_sel : 5;     /**< Trigger source selection */
        unsigned int cfg_soc12_cont_en : 1;      /**< Continuous conversion mode enable bit */
        unsigned int reserved0 : 17;
    } BIT;
} volatile ADC_SOC12_CFG_REG;

/**
  * @brief Define the union ADC_SOC13_CFG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_soc13_ch_sel : 5;       /**< Channel selection */
        unsigned int cfg_soc13_samptime_sel : 4; /**< Sampling Period Selection */
        unsigned int cfg_soc13_trig_sel : 5;     /**< Trigger source selection */
        unsigned int cfg_soc13_cont_en : 1;      /**< Continuous conversion mode enable bit */
        unsigned int reserved0 : 17;
    } BIT;
} volatile ADC_SOC13_CFG_REG;

/**
  * @brief Define the union ADC_SOC14_CFG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_soc14_ch_sel : 5;       /**< Channel selection */
        unsigned int cfg_soc14_samptime_sel : 4; /**< Sampling Period Selection */
        unsigned int cfg_soc14_trig_sel : 5;     /**< Trigger source selection */
        unsigned int cfg_soc14_cont_en : 1;      /**< Continuous conversion mode enable bit */
        unsigned int reserved0 : 17;
    } BIT;
} volatile ADC_SOC14_CFG_REG;

/**
  * @brief Define the union ADC_SOC15_CFG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_soc15_ch_sel : 5;       /**< Channel selection */
        unsigned int cfg_soc15_samptime_sel : 4; /**< Sampling Period Selection */
        unsigned int cfg_soc15_trig_sel : 5;     /**< Trigger source selection */
        unsigned int cfg_soc15_cont_en : 1;      /**< Continuous conversion mode enable bit */
        unsigned int reserved0 : 17;
    } BIT;
} volatile ADC_SOC15_CFG_REG;

/**
  * @brief Define the union ADC_SOFT_TRIG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_soc0_soft_trig : 1;    /**< SOC0 triggered by software */
        unsigned int cfg_soc1_soft_trig : 1;    /**< SOC1 triggered by software */
        unsigned int cfg_soc2_soft_trig : 1;    /**< SOC2 triggered by software */
        unsigned int cfg_soc3_soft_trig : 1;    /**< SOC3 triggered by software */
        unsigned int cfg_soc4_soft_trig : 1;    /**< SOC4 triggered by software */
        unsigned int cfg_soc5_soft_trig : 1;    /**< SOC5 triggered by software */
        unsigned int cfg_soc6_soft_trig : 1;    /**< SOC6 triggered by software */
        unsigned int cfg_soc7_soft_trig : 1;    /**< SOC7 triggered by software */
        unsigned int cfg_soc8_soft_trig : 1;    /**< SOC8 triggered by software */
        unsigned int cfg_soc9_soft_trig : 1;    /**< SOC9 triggered by software */
        unsigned int cfg_soc10_soft_trig : 1;   /**< SOC10 triggered by software */
        unsigned int cfg_soc11_soft_trig : 1;   /**< SOC11 triggered by software */
        unsigned int cfg_soc12_soft_trig : 1;   /**< SOC12 triggered by software */
        unsigned int cfg_soc13_soft_trig : 1;   /**< SOC13 triggered by software */
        unsigned int cfg_soc14_soft_trig : 1;   /**< SOC14 triggered by software */
        unsigned int cfg_soc15_soft_trig : 1;   /**< SOC15 triggered by software */
        unsigned int reserved0 : 16;
    } BIT;
} volatile ADC_SOFT_TRIG_REG;

/**
  * @brief Define the union ADC_ARBT0_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_soc_priority : 16;     /**< Priority configuration */
        unsigned int reserved0 : 16;
    } BIT;
} volatile ADC_ARBT0_REG;

/**
  * @brief Define the union ADC_ARBT1_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_rr_pointer_reset : 1;  /**< Reset Poll Pointer */
        unsigned int reserved0 : 31;
    } BIT;
} volatile ADC_ARBT1_REG;

/**
  * @brief Define the union ADC_ARBT2_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int rr_pointer : 4;            /**< Priority polling pointer */
        unsigned int reserved0 : 28;
    } BIT;
} volatile ADC_ARBT2_REG;

/**
  * @brief Define the union ADC_OVERSAMP_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_oversamp_en : 1;       /**< Oversampling enable bit  */
        unsigned int reserved0 : 3;
        unsigned int cfg_oversamp_soc_sel : 4;  /**< Selecting a specified SoC for oversampling */
        unsigned int cfg_oversamp_n : 4;        /**< Configuring the Oversampling Multiple */
        unsigned int cfg_oversamp_m : 4;        /**< Oversampling precision truncation */
        unsigned int reserved1 : 16;
    } BIT;
} volatile ADC_OVERSAMP_REG;

/**
  * @brief Define the union ADC_OVERSAMP_RESULT_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int oversamp_data : 16;        /**< Oversampling result. The lower bits (12 to 16 bits) are valid */
        unsigned int reserved0 : 16;
    } BIT;
} volatile ADC_OVERSAMP_RESULT_REG;

/**
  * @brief Define the union ADC_PPB0_CTRL0_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0 : 1;
        unsigned int cfg_ppb0_dly_en : 1;       /**< Sampling delay count enable bit */
        unsigned int cfg_ppb0_offset_en : 1;    /**< Offset result count enable */
        unsigned int cfg_ppb0_detect_en : 1;    /**< Threshold detection enable */
        unsigned int cfg_ppb0_soc_sel : 4;      /**< Select soc */
        unsigned int reserved1 : 12;
        unsigned int cfg_ppb0_offset : 12;      /**< set offset value. 1-bit sign bit, 11-bit integer bit */
    } BIT;
} volatile ADC_PPB0_CTRL0_REG;

/**
  * @brief Define the union ADC_PPB0_CTRL1_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_ppb0_dnlimit : 13;     /**< Lower threshold of error detection */
        unsigned int cfg_ppb0_uplimit : 13;     /**< Upper threshold of error detection */
        unsigned int reserved0 : 6;
    } BIT;
} volatile ADC_PPB0_CTRL1_REG;

/**
  * @brief Define the union ADC_PPB0_CTRL2_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_ppb0_ref : 12;         /**< Error reference value (unsigned number, 12-bit integer) */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_PPB0_CTRL2_REG;

/**
  * @brief Define the union ADC_PPB0_RESULT_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ppb0_error_data : 13;      /**< Error calculation result */
        unsigned int reserved0 : 3;
        unsigned int ppb0_dly_stamp : 16;       /**< Sample delay count value */
    } BIT;
} volatile ADC_PPB0_RESULT_REG;

/**
  * @brief Define the union ADC_PPB1_CTRL0_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0 : 1;
        unsigned int cfg_ppb1_dly_en : 1;       /**< Sampling delay count enable bit */
        unsigned int cfg_ppb1_offset_en : 1;    /**< Offset result count enable */
        unsigned int cfg_ppb1_detect_en : 1;    /**< Threshold detection enable */
        unsigned int cfg_ppb1_soc_sel : 4;      /**< Select soc */
        unsigned int reserved1 : 12;
        unsigned int cfg_ppb1_offset : 12;      /**< set offset value. 1-bit sign bit, 11-bit integer bit */
    } BIT;
} volatile ADC_PPB1_CTRL0_REG;

/**
  * @brief Define the union ADC_PPB1_CTRL1_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_ppb1_dnlimit : 13;     /**< Lower threshold of error detection */
        unsigned int cfg_ppb1_uplimit : 13;     /**< Upper threshold of error detection */
        unsigned int reserved0 : 6;
    } BIT;
} volatile ADC_PPB1_CTRL1_REG;

/**
  * @brief Define the union ADC_PPB1_CTRL2_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_ppb1_ref : 12;         /**< Error reference value (unsigned number, 12-bit integer) */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_PPB1_CTRL2_REG;

/**
  * @brief Define the union ADC_PPB1_RESULT_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ppb1_error_data : 13;      /**< Error calculation result */
        unsigned int reserved0 : 3;
        unsigned int ppb1_dly_stamp : 16;       /**< Sample delay count value */
    } BIT;
} volatile ADC_PPB1_RESULT_REG;

/**
  * @brief Define the union ADC_PPB2_CTRL0_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0 : 1;
        unsigned int cfg_ppb2_dly_en : 1;       /**< Sampling delay count enable bit */
        unsigned int cfg_ppb2_offset_en : 1;    /**< Offset result count enable */
        unsigned int cfg_ppb2_detect_en : 1;    /**< Threshold detection enable */
        unsigned int cfg_ppb2_soc_sel : 4;      /**< Select soc */
        unsigned int reserved1 : 12;
        unsigned int cfg_ppb2_offset : 12;      /**< set offset value. 1-bit sign bit, 11-bit integer bit */
    } BIT;
} volatile ADC_PPB2_CTRL0_REG;

/**
  * @brief Define the union ADC_PPB2_CTRL1_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_ppb2_dnlimit : 13;     /**< Lower threshold of error detection */
        unsigned int cfg_ppb2_uplimit : 13;     /**< Upper threshold of error detection */
        unsigned int reserved0 : 6;
    } BIT;
} volatile ADC_PPB2_CTRL1_REG;

/**
  * @brief Define the union ADC_PPB2_CTRL2_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_ppb2_ref : 12;         /**< Error reference value (unsigned number, 12-bit integer) */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_PPB2_CTRL2_REG;

/**
  * @brief Define the union ADC_PPB2_RESULT_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ppb2_error_data : 13;      /**< Error calculation result */
        unsigned int reserved0 : 3;
        unsigned int ppb2_dly_stamp : 16;       /**< Sample delay count value */
    } BIT;
} volatile ADC_PPB2_RESULT_REG;

/**
  * @brief Define the union ADC_PPB3_CTRL0_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int reserved0 : 1;
        unsigned int cfg_ppb3_dly_en : 1;       /**< Sampling delay count enable bit */
        unsigned int cfg_ppb3_offset_en : 1;    /**< Offset result count enable */
        unsigned int cfg_ppb3_detect_en : 1;    /**< Threshold detection enable */
        unsigned int cfg_ppb3_soc_sel : 4;      /**< Select soc */
        unsigned int reserved1 : 12;
        unsigned int cfg_ppb3_offset : 12;      /**< set offset value. 1-bit sign bit, 11-bit integer bit */
    } BIT;
} volatile ADC_PPB3_CTRL0_REG;

/**
  * @brief Define the union ADC_PPB3_CTRL1_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_ppb3_dnlimit : 13;     /**< Lower threshold of error detection */
        unsigned int cfg_ppb3_uplimit : 13;     /**< Upper threshold of error detection */
        unsigned int reserved0 : 6;
    } BIT;
} volatile ADC_PPB3_CTRL1_REG;

/**
  * @brief Define the union ADC_PPB3_CTRL2_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_ppb3_ref : 12;         /**< Error reference value (unsigned number, 12-bit integer) */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_PPB3_CTRL2_REG;

/**
  * @brief Define the union ADC_PPB3_RESULT_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int ppb3_error_data : 13;      /**< Error calculation result */
        unsigned int reserved0 : 3;
        unsigned int ppb3_dly_stamp : 16;       /**< Sample delay count value */
    } BIT;
} volatile ADC_PPB3_RESULT_REG;

/**
  * @brief Define the union ADC_INT_DATA_0_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_intr_data_sel0 : 16;       /**< Configuration SoC selection data completion interrupt 0 */
        unsigned int cfg_intr_data_sel1 : 16;       /**< Configuration SoC selection data completion interrupt 1 */
    } BIT;
} volatile ADC_INT_DATA_0_REG;

/**
  * @brief Define the union ADC_INT_DATA_1_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_intr_data_sel2 : 16;       /**< Configuration SoC selection data completion interrupt 2 */
        unsigned int cfg_intr_data_sel3 : 16;       /**< Configuration SoC selection data completion interrupt 3 */
    } BIT;
} volatile ADC_INT_DATA_1_REG;

/**
  * @brief Define the union ADC_INT_DATA_FLAG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int intr_data_flag0 : 1;           /**< Raw status of data completion interrupt 0 */
        unsigned int intr_data_flag1 : 1;           /**< Raw status of data completion interrupt 1 */
        unsigned int intr_data_flag2 : 1;           /**< Raw status of data completion interrupt 2 */
        unsigned int intr_data_flag3 : 1;           /**< Raw status of data completion interrupt 3 */
        unsigned int reserved0 : 28;
    } BIT;
} volatile ADC_INT_DATA_FLAG_REG;

/**
  * @brief Define the union ADC_INT_DATA_MSK_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int intr_data_flag0_msk : 1;       /**< Masked interrupt status of data completion interrupt 0 */
        unsigned int intr_data_flag1_msk : 1;       /**< Masked interrupt status of data completion interrupt 1 */
        unsigned int intr_data_flag2_msk : 1;       /**< Masked interrupt status of data completion interrupt 2 */
        unsigned int intr_data_flag3_msk : 1;       /**< Masked interrupt status of data completion interrupt 3 */
        unsigned int reserved0 : 28;
    } BIT;
} volatile ADC_INT_DATA_MSK_REG;

/* Define the union ADC_DATA_FLAG_MASK_REG */
/**
  * @brief Define the union ADC_INT_DATA_FLAG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int intr_data_flag0_mask : 1;      /**< Interrupt mask flag of data completion interrupt 0 */
        unsigned int intr_data_flag1_mask : 1;      /**< Interrupt mask flag of data completion interrupt 1 */
        unsigned int intr_data_flag2_mask : 1;      /**< Interrupt mask flag of data completion interrupt 2 */
        unsigned int intr_data_flag3_mask : 1;      /**< Interrupt mask flag of data completion interrupt 3 */
        unsigned int reserved0 : 28;
    } BIT;
} volatile ADC_DATA_FLAG_MASK_REG;

/**
  * @brief Define the union ADC_EVENT_INT_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int intr_ppb0_zero_det : 1;        /**< Raw status of PPB0 zero-crossing interrupt */
        unsigned int intr_ppb0_uplimit_det : 1;     /**< Raw status of PPB0 up threshold interrupt */
        unsigned int intr_ppb0_dnlimit_det : 1;     /**< Raw status of PPB0 down threshold interrupt */
        unsigned int intr_ppb0_error_data_vld : 1;  /**< Raw status of PPB0 error calculation interrupt */
        unsigned int intr_ppb1_zero_det : 1;        /**< Raw status of PPB1 zero-crossing interrupt */
        unsigned int intr_ppb1_uplimit_det : 1;     /**< Raw status of PPB1 up threshold interrupt */
        unsigned int intr_ppb1_dnlimit_det : 1;     /**< Raw status of PPB1 down threshold interrupt */
        unsigned int intr_ppb1_error_data_vld : 1;  /**< Raw status of PPB1 error calculation interrupt */
        unsigned int intr_ppb2_zero_det : 1;        /**< Raw status of PPB2 zero-crossing interrupt */
        unsigned int intr_ppb2_uplimit_det : 1;     /**< Raw status of PPB2 up threshold interrupt */
        unsigned int intr_ppb2_dnlimit_det : 1;     /**< Raw status of PPB2 down threshold interrupt */
        unsigned int intr_ppb2_error_data_vld : 1;  /**< Raw status of PPB2 error calculation interrupt */
        unsigned int intr_ppb3_zero_det : 1;        /**< Raw status of PPB3 zero-crossing interrupt */
        unsigned int intr_ppb3_uplimit_det : 1;     /**< Raw status of PPB3 up threshold interrupt */
        unsigned int intr_ppb3_dnlimit_det : 1;     /**< Raw status of PPB3 down threshold interrupt */
        unsigned int intr_ppb3_error_data_vld : 1;  /**< Raw status of PPB3 error calculation interrupt */
        unsigned int intr_oversamp_data_vld : 1;    /**< Raw status of oversampling completion interrupt */
        unsigned int intr_cali_done : 1;            /**< Raw status of calibration completed interrupt */
        unsigned int reserved0 : 14;
    } BIT;
} volatile ADC_EVENT_INT_REG;

/**
  * @brief Define the union ADC_EVENT_INT_MSK_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int intr_ppb0_zero_det_msk : 1;        /**< Masked status of PPB0 zero-crossing interrupt */
        unsigned int intr_ppb0_uplimit_det_msk : 1;     /**< Masked status of PPB0 up threshold interrupt */
        unsigned int intr_ppb0_dnlimit_det_msk : 1;     /**< Masked status of PPB0 down threshold interrupt */
        unsigned int intr_ppb0_error_data_vld_msk : 1;  /**< Masked status of PPB0 error calculation interrupt */
        unsigned int intr_ppb1_zero_det_msk : 1;        /**< Masked status of PPB1 zero-crossing interrupt */
        unsigned int intr_ppb1_uplimit_det_msk : 1;     /**< Masked status of PPB1 up threshold interrupt */
        unsigned int intr_ppb1_dnlimit_det_msk : 1;     /**< Masked status of PPB1 down threshold interrupt */
        unsigned int intr_ppb1_error_data_vld_msk : 1;  /**< Masked status of PPB1 error calculation interrupt */
        unsigned int intr_ppb2_zero_det_msk : 1;        /**< Masked status of PPB2 zero-crossing interrupt */
        unsigned int intr_ppb2_uplimit_det_msk : 1;     /**< Masked status of PPB2 up threshold interrupt */
        unsigned int intr_ppb2_dnlimit_det_msk : 1;     /**< Masked status of PPB2 down threshold interrupt */
        unsigned int intr_ppb2_error_data_vld_msk : 1;  /**< Masked status of PPB2 error calculation interrupt */
        unsigned int intr_ppb3_zero_det_msk : 1;        /**< Masked status of PPB3 zero-crossing interrupt */
        unsigned int intr_ppb3_uplimit_det_msk : 1;     /**< Masked status of PPB3 up threshold interrupt */
        unsigned int intr_ppb3_dnlimit_det_msk : 1;     /**< Masked status of PPB3 down threshold interrupt */
        unsigned int intr_ppb3_error_data_vld_msk : 1;  /**< Masked status of PPB3 error calculation interrupt */
        unsigned int intr_oversamp_data_vld_msk : 1;    /**< Masked status of oversampling completion interrupt */
        unsigned int intr_cali_done_msk : 1;            /**< Masked status of calibration completed interrupt */
        unsigned int reserved0 : 14;
    } BIT;
} volatile ADC_EVENT_INT_MSK_REG;

/**
  * @brief Define the union ADC_EVENT_INT_MASK_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int intr_ppb0_zero_det_mask : 1;       /**< Mask flag of PPB0 zero-crossing interrupt */
        unsigned int intr_ppb0_uplimit_det_mask : 1;    /**< Mask flag of PPB0 up threshold interrupt */
        unsigned int intr_ppb0_dnlimit_det_mask : 1;    /**< Mask flag of PPB0 down threshold interrupt */
        unsigned int intr_ppb0_error_data_vld_mask : 1; /**< Mask flag of PPB0 error calculation interrupt */
        unsigned int intr_ppb1_zero_det_mask : 1;       /**< Mask flag of PPB1 zero-crossing interrupt */
        unsigned int intr_ppb1_uplimit_det_mask : 1;    /**< Mask flag of PPB1 up threshold interrupt */
        unsigned int intr_ppb1_dnlimit_det_mask : 1;    /**< Mask flag of PPB1 down threshold interrupt */
        unsigned int intr_ppb1_error_data_vld_mask : 1; /**< Mask flag of PPB1 error calculation interrupt */
        unsigned int intr_ppb2_zero_det_mask : 1;       /**< Mask flag of PPB2 zero-crossing interrupt */
        unsigned int intr_ppb2_uplimit_det_mask : 1;    /**< Mask flag of PPB2 up threshold interrupt */
        unsigned int intr_ppb2_dnlimit_det_mask : 1;    /**< Mask flag of PPB2 down threshold interrupt */
        unsigned int intr_ppb2_error_data_vld_mask : 1; /**< Mask flag of PPB2 error calculation interrupt */
        unsigned int intr_ppb3_zero_det_mask : 1;       /**< Mask flag of PPB3 zero-crossing interrupt */
        unsigned int intr_ppb3_uplimit_det_mask : 1;    /**< Mask flag of PPB3 up threshold interrupt */
        unsigned int intr_ppb3_dnlimit_det_mask : 1;    /**< Mask flag of PPB3 down threshold interrupt */
        unsigned int intr_ppb3_error_data_vld_mask : 1; /**< Mask flag of PPB3 error calculation interrupt */
        unsigned int intr_oversamp_data_vld_mask : 1;   /**< Mask flag of oversampling completion interrupt */
        unsigned int intr_cali_done_mask : 1;           /**< Mask flag of calibration completed interrupt */
        unsigned int reserved0 : 14;
    } BIT;
} volatile ADC_EVENT_INT_MASK_REG;

/**
  * @brief Define the union ADC_ERR_INT_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int intr_soc0_trig_over_flag : 1;  /**< Raw status of soc0 trigger overflow */
        unsigned int intr_soc1_trig_over_flag : 1;  /**< Raw status of soc1 trigger overflow */
        unsigned int intr_soc2_trig_over_flag : 1;  /**< Raw status of soc2 trigger overflow */
        unsigned int intr_soc3_trig_over_flag : 1;  /**< Raw status of soc3 trigger overflow */
        unsigned int intr_soc4_trig_over_flag : 1;  /**< Raw status of soc4 trigger overflow */
        unsigned int intr_soc5_trig_over_flag : 1;  /**< Raw status of soc5 trigger overflow */
        unsigned int intr_soc6_trig_over_flag : 1;  /**< Raw status of soc6 trigger overflow */
        unsigned int intr_soc7_trig_over_flag : 1;  /**< Raw status of soc7 trigger overflow */
        unsigned int intr_soc8_trig_over_flag : 1;  /**< Raw status of soc8 trigger overflow */
        unsigned int intr_soc9_trig_over_flag : 1;  /**< Raw status of soc9 trigger overflow */
        unsigned int intr_soc10_trig_over_flag : 1; /**< Raw status of soc10 trigger overflow */
        unsigned int intr_soc11_trig_over_flag : 1; /**< Raw status of soc11 trigger overflow */
        unsigned int intr_soc12_trig_over_flag : 1; /**< Raw status of soc12 trigger overflow */
        unsigned int intr_soc13_trig_over_flag : 1; /**< Raw status of soc13 trigger overflow */
        unsigned int intr_soc14_trig_over_flag : 1; /**< Raw status of soc14 trigger overflow */
        unsigned int intr_soc15_trig_over_flag : 1; /**< Raw status of soc15 trigger overflow */
        unsigned int intr_dma_req_over_flag : 1;    /**< Raw status of dma request overflow */
        unsigned int reserved0 : 15;
    } BIT;
} volatile ADC_ERR_INT_REG;

/**
  * @brief Define the union ADC_ERR_INT_MSK_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int intr_soc0_trig_over_flag_msk : 1;  /**< Masked status of soc0 trigger overflow */
        unsigned int intr_soc1_trig_over_flag_msk : 1;  /**< Masked status of soc1 trigger overflow */
        unsigned int intr_soc2_trig_over_flag_msk : 1;  /**< Masked status of soc2 trigger overflow */
        unsigned int intr_soc3_trig_over_flag_msk : 1;  /**< Masked status of soc3 trigger overflow */
        unsigned int intr_soc4_trig_over_flag_msk : 1;  /**< Masked status of soc4 trigger overflow */
        unsigned int intr_soc5_trig_over_flag_msk : 1;  /**< Masked status of soc5 trigger overflow */
        unsigned int intr_soc6_trig_over_flag_msk : 1;  /**< Masked status of soc6 trigger overflow */
        unsigned int intr_soc7_trig_over_flag_msk : 1;  /**< Masked status of soc7 trigger overflow */
        unsigned int intr_soc8_trig_over_flag_msk : 1;  /**< Masked status of soc8 trigger overflow */
        unsigned int intr_soc9_trig_over_flag_msk : 1;  /**< Masked status of soc9 trigger overflow */
        unsigned int intr_soc10_trig_over_flag_msk : 1; /**< Masked status of soc10 trigger overflow */
        unsigned int intr_soc11_trig_over_flag_msk : 1; /**< Masked status of soc11 trigger overflow */
        unsigned int intr_soc12_trig_over_flag_msk : 1; /**< Masked status of soc12 trigger overflow */
        unsigned int intr_soc13_trig_over_flag_msk : 1; /**< Masked status of soc13 trigger overflow */
        unsigned int intr_soc14_trig_over_flag_msk : 1; /**< Masked status of soc14 trigger overflow */
        unsigned int intr_soc15_trig_over_flag_msk : 1; /**< Masked status of soc15 trigger overflow */
        unsigned int intr_dma_req_over_flag_msk : 1;    /**< Masked status of dma request overflow */
        unsigned int reserved0 : 15;
    } BIT;
} volatile ADC_ERR_INT_MSK_REG;

/**
  * @brief Define the union ADC_ERR_INT_MASK_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int intr_soc0_trig_over_flag_mask : 1;     /**< Mask flag of soc0 trigger overflow */
        unsigned int intr_soc1_trig_over_flag_mask : 1;     /**< Mask flag of soc1 trigger overflow */
        unsigned int intr_soc2_trig_over_flag_mask : 1;     /**< Mask flag of soc2 trigger overflow */
        unsigned int intr_soc3_trig_over_flag_mask : 1;     /**< Mask flag of soc3 trigger overflow */
        unsigned int intr_soc4_trig_over_flag_mask : 1;     /**< Mask flag of soc4 trigger overflow */
        unsigned int intr_soc5_trig_over_flag_mask : 1;     /**< Mask flag of soc5 trigger overflow */
        unsigned int intr_soc6_trig_over_flag_mask : 1;     /**< Mask flag of soc6 trigger overflow */
        unsigned int intr_soc7_trig_over_flag_mask : 1;     /**< Mask flag of soc7 trigger overflow */
        unsigned int intr_soc8_trig_over_flag_mask : 1;     /**< Mask flag of soc8 trigger overflow */
        unsigned int intr_soc9_trig_over_flag_mask : 1;     /**< Mask flag of soc9 trigger overflow */
        unsigned int intr_soc10_trig_over_flag_mask : 1;    /**< Mask flag of soc10 trigger overflow */
        unsigned int intr_soc11_trig_over_flag_mask : 1;    /**< Mask flag of soc11 trigger overflow */
        unsigned int intr_soc12_trig_over_flag_mask : 1;    /**< Mask flag of soc12 trigger overflow */
        unsigned int intr_soc13_trig_over_flag_mask : 1;    /**< Mask flag of soc13 trigger overflow */
        unsigned int intr_soc14_trig_over_flag_mask : 1;    /**< Mask flag of soc14 trigger overflow */
        unsigned int intr_soc15_trig_over_flag_mask : 1;    /**< Mask flag of soc15 trigger overflow */
        unsigned int intr_dma_req_over_flag_mask : 1;       /**< Mask flag of dma request overflow */
        unsigned int reserved0 : 15;
    } BIT;
} volatile ADC_ERR_INT_MASK_REG;

/**
  * @brief Define the union ADC_DMA_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_dma_soc_sel : 4;       /**< Configuring the DMA function for a specified soc */
        unsigned int cfg_dma_sing_req_sel : 1;  /**< DMA single request signal enable */
        unsigned int cfg_dma_brst_req_sel : 1;  /**< DMA burst request signal enable */
        unsigned int reserved0 : 26;
    } BIT;
} volatile ADC_DMA_REG;

/**
  * @brief Define the union ADC_EN_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_adc_en : 1;       /**< ADC Controller Enable */
        unsigned int reserved0 : 31;
    } BIT;
} volatile ADC_EN_REG;

/**
  * @brief Define the union ADC_EN_DLY_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_en_dly : 8;        /**< Delay time after ADC is enabled (us) */
        unsigned int reserved0 : 24;
    } BIT;
} volatile ADC_EN_DLY_REG;

/**
  * @brief Define the union ADC_MODE_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_adc_mode : 2;      /**< ADC Operating Mode */
        unsigned int reserved0 : 30;
    } BIT;
} volatile ADC_MODE_REG;

/**
  * @brief Define the union ADC_OEGE_CH_SEL_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_oege_ch_sel0 : 5;  /**< Connection channel select, compensated by cfg_gain0/cfg_ofst0 */
        unsigned int reserved0 : 3;
        unsigned int cfg_oege_ch_sel1 : 5;  /**< Connection channel select, compensated by cfg_gain1/cfg_ofst1 */
        unsigned int reserved1 : 3;
        unsigned int cfg_oege_ch_sel2 : 5;  /**< Connection channel select, compensated by cfg_gain2/cfg_ofst2 */
        unsigned int reserved2 : 11;
    } BIT;
} volatile ADC_OEGE_CH_SEL_REG;

/**
  * @brief Define the union ADC_OEGE_CTRL0_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_ofst0 : 12;    /**< Set OE value. 1-bit sign bit, 10-bit integer bit, 1-bit decimal bit */
        unsigned int reserved0 : 4;
        unsigned int cfg_gain0 : 13;    /**< Set GE value. Unsigned number, 1-bit integer, 12-bit decimal */
        unsigned int reserved1 : 3;
    } BIT;
} volatile ADC_OEGE_CTRL0_REG;

/**
  * @brief Define the union ADC_OEGE_CTRL1_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_ofst1 : 12;    /**< Set OE value. 1-bit sign bit, 10-bit integer bit, 1-bit decimal bit */
        unsigned int reserved0 : 4;
        unsigned int cfg_gain1 : 13;    /**< Set GE value. Unsigned number, 1-bit integer, 12-bit decimal */
        unsigned int reserved1 : 3;
    } BIT;
} volatile ADC_OEGE_CTRL1_REG;

/**
  * @brief Define the union ADC_OEGE_CTRL2_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_ofst2 : 12;    /**< Set OE value. 1-bit sign bit, 10-bit integer bit, 1-bit decimal bit */
        unsigned int reserved0 : 4;
        unsigned int cfg_gain2 : 13;    /**< Set GE value. Unsigned number, 1-bit integer, 12-bit decimal */
        unsigned int reserved1 : 3;
    } BIT;
} volatile ADC_OEGE_CTRL2_REG;

/**
  * @brief Define the union ADC_PROCESS0_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_ofst_cali : 12; /**< Level-2 offset compensation, 1-bit sign, 10-bit integer, 1-bit decimal */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_PROCESS0_REG;

/**
  * @brief Define the union ADC_PROCESS1_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_noise_add_en : 1;   /**< Digital noise addition enable */
        unsigned int cfg_w_norm_sel : 1;
        unsigned int cfg_noise_add_bits : 3; /**< Digital noise size */
        unsigned int reserved0 : 27;
    } BIT;
} volatile ADC_PROCESS1_REG;

/**
  * @brief Define the union ADC_STATUS_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int adc_busy : 1;      /**< ADC working status */
        unsigned int reserved0 : 31;
    } BIT;
} volatile ADC_STATUS_REG;

/**
  * @brief Define the union ADC_WEIGHT_CFG15_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_weight15 : 19; /**< Weight configuration value of capacitor 15 */
        unsigned int reserved0 : 13;
    } BIT;
} volatile ADC_WEIGHT_CFG15_REG;

/**
  * @brief Define the union ADC_WEIGHT_CFG14_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_weight14 : 18; /**< Weight configuration value of capacitor 14 */
        unsigned int reserved0 : 14;
    } BIT;
} volatile ADC_WEIGHT_CFG14_REG;

/**
  * @brief Define the union ADC_WEIGHT_CFG13_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_weight13 : 17; /**< Weight configuration value of capacitor 13 */
        unsigned int reserved0 : 15;
    } BIT;
} volatile ADC_WEIGHT_CFG13_REG;

/**
  * @brief Define the union ADC_WEIGHT_CFG12_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_weight12 : 16; /**< Weight configuration value of capacitor 12 */
        unsigned int reserved0 : 16;
    } BIT;
} volatile ADC_WEIGHT_CFG12_REG;

/**
  * @brief Define the union ADC_WEIGHT_CFG11_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_weight11 : 15; /**< Weight configuration value of capacitor 11 */
        unsigned int reserved0 : 17;
    } BIT;
} volatile ADC_WEIGHT_CFG11_REG;

/**
  * @brief Define the union ADC_WEIGHT_CFG10_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_weight10 : 15; /**< Weight configuration value of capacitor 10 */
        unsigned int reserved0 : 17;
    } BIT;
} volatile ADC_WEIGHT_CFG10_REG;

/**
  * @brief Define the union ADC_WEIGHT_CFG9_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_weight9 : 14; /**< Weight configuration value of capacitor 9 */
        unsigned int reserved0 : 18;
    } BIT;
} volatile ADC_WEIGHT_CFG9_REG;

/**
  * @brief Define the union ADC_WEIGHT_CFG8_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_weight8 : 13; /**< Weight configuration value of capacitor 8 */
        unsigned int reserved0 : 19;
    } BIT;
} volatile ADC_WEIGHT_CFG8_REG;

/**
  * @brief Define the union ADC_WEIGHT_CFG7_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_weight7 : 12; /**< Weight configuration value of capacitor 7 */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_WEIGHT_CFG7_REG;

/**
  * @brief Define the union ADC_WEIGHT_CFG6_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_weight6 : 12; /**< Weight configuration value of capacitor 6 */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_WEIGHT_CFG6_REG;

/**
  * @brief Define the union ADC_WEIGHT_CFG5_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_weight5 : 11; /**< Weight configuration value of capacitor 5 */
        unsigned int reserved0 : 21;
    } BIT;
} volatile ADC_WEIGHT_CFG5_REG;

/**
  * @brief Define the union ADC_WEIGHT_CFG4_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_weight4 : 10; /**< Weight configuration value of capacitor 4 */
        unsigned int reserved0 : 22;
    } BIT;
} volatile ADC_WEIGHT_CFG4_REG;

/**
  * @brief Define the union ADC_WEIGHT_CFG3_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_weight3 : 9; /**< Weight configuration value of capacitor 3 */
        unsigned int reserved0 : 23;
    } BIT;
} volatile ADC_WEIGHT_CFG3_REG;

/**
  * @brief Define the union ADC_CAP_TRG_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_mode1_start : 1; /**< Trigger logic auto-calibration weight value */
        unsigned int cfg_weight_ini : 1;  /**< Initialize the weight and load the weight value */
        unsigned int reserved0 : 30;
    } BIT;
} volatile ADC_CAP_TRG_REG;

/**
  * @brief Define the union ADC_CAP_M1_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_mode1_acc_sel : 3;         /**< Mode 1 Accumulated Times Selection */
        unsigned int cfg_cap_index : 4;             /**< Mode 1 Start Capacitor Configuration */
        unsigned int cfg_weight_limit_sel : 3;      /**< Weight upper and lower limit gear selection */
        unsigned int cfg_mode1_caplsb_sel : 1;      /**< Mode 1 low-bit capacitor enable */
        unsigned int cfg_weight_limit_bypass : 1;   /**< Weight upper and lower threshold bypass enable */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_CAP_M1_REG;

/**
  * @brief Define the union ADC_ANA_CTRL0_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_inmux_en : 1;          /**< Channel selection control enable signal */
        unsigned int cfg_comp_chop_en : 1;      /**< Chopper enable signal of the ADC comparator */
        unsigned int cfg_latch_dly_sel : 1;     /**< Sets whether to delay one cycle. */
        unsigned int cfg_muxtime_sel : 2;       /**< Channel early switch period selection */
        unsigned int reserved0 : 11;
        unsigned int cfg_sar_comp : 4;          /**< ADC COMP reserved register */
        unsigned int cfg_sar_vcm : 4;           /**< ADC VCM reserved register */
        unsigned int cfg_sar_vref : 4;          /**< ADC VREF reserved register */
        unsigned int cfg_sar_samp_cap_sel : 4;  /**< Number of ADC sampling capacitors */
    } BIT;
} volatile ADC_ANA_CTRL0_REG;

/**
  * @brief Define the union ADC_TSENSOR_TRIM_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_tsensor_ofst_trim : 12; /**< Tsensor offset compensation trim value */
        unsigned int reserved0 : 20;
    } BIT;
} volatile ADC_TSENSOR_TRIM_REG;

/**
  * @brief Define the union ADC_OEGE_TRIM_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_ofst_cali_trim : 12; /**< ADC General Gain Calibration Trim Value */
        unsigned int reserved0 : 4;
        unsigned int cfg_gain_cali_trim : 13; /**< ADC general offset calibration trim value */
        unsigned int reserved1 : 3;
    } BIT;
} volatile ADC_OEGE_TRIM_REG;

/**
  * @brief Define the union ADC_PGA0_OEGE_TRIM0_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_pga0_ofst_trim2 : 12; /**< Gain calibration trim value */
        unsigned int reserved0 : 4;
        unsigned int cfg_pga0_gain_trim2 : 13; /**< Offset calibration trim value */
        unsigned int reserved1 : 3;
    } BIT;
} volatile ADC_PGA0_OEGE_TRIM0_REG;

/**
  * @brief Define the union ADC_PGA0_OEGE_TRIM1_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_pga0_ofst_trim4 : 12; /**< Gain calibration trim value */
        unsigned int reserved0 : 4;
        unsigned int cfg_pga0_gain_trim4 : 13; /**< Offset calibration trim value */
        unsigned int reserved1 : 3;
    } BIT;
} volatile ADC_PGA0_OEGE_TRIM1_REG;

/**
  * @brief Define the union ADC_PGA0_OEGE_TRIM2_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_pga0_ofst_trim8 : 12;  /**< Gain calibration trim value */
        unsigned int reserved0 : 4;
        unsigned int cfg_pga0_gain_trim8 : 13;  /**< Offset calibration trim value */
        unsigned int reserved1 : 3;
    } BIT;
} volatile ADC_PGA0_OEGE_TRIM2_REG;

/**
  * @brief Define the union ADC_PGA0_OEGE_TRIM3_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_pga0_ofst_trim16 : 12; /**< Gain calibration trim value */
        unsigned int reserved0 : 4;
        unsigned int cfg_pga0_gain_trim16 : 13; /**< Offset calibration trim value */
        unsigned int reserved1 : 3;
    } BIT;
} volatile ADC_PGA0_OEGE_TRIM3_REG;

/**
  * @brief Define the union ADC_PGA1_OEGE_TRIM0_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_pga1_ofst_trim2 : 12;  /**< Gain calibration trim value */
        unsigned int reserved0 : 4;
        unsigned int cfg_pga1_gain_trim2 : 13;  /**< Offset calibration trim value */
        unsigned int reserved1 : 3;
    } BIT;
} volatile ADC_PGA1_OEGE_TRIM0_REG;

/**
  * @brief Define the union ADC_PGA1_OEGE_TRIM1_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_pga1_ofst_trim4 : 12;  /**< Gain calibration trim value */
        unsigned int reserved0 : 4;
        unsigned int cfg_pga1_gain_trim4 : 13;  /**< Offset calibration trim value */
        unsigned int reserved1 : 3;
    } BIT;
} volatile ADC_PGA1_OEGE_TRIM1_REG;

/**
  * @brief Define the union ADC_PGA1_OEGE_TRIM2_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_pga1_ofst_trim8 : 12;  /**< Gain calibration trim value */
        unsigned int reserved0 : 4;
        unsigned int cfg_pga1_gain_trim8 : 13;  /**< Offset calibration trim value */
        unsigned int reserved1 : 3;
    } BIT;
} volatile ADC_PGA1_OEGE_TRIM2_REG;

/**
  * @brief Define the union ADC_PGA1_OEGE_TRIM3_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_pga1_ofst_trim16 : 12; /**< Gain calibration trim value */
        unsigned int reserved0 : 4;
        unsigned int cfg_pga1_gain_trim16 : 13; /**< Offset calibration trim value */
        unsigned int reserved1 : 3;
    } BIT;
} volatile ADC_PGA1_OEGE_TRIM3_REG;

/**
  * @brief Define the union ADC_ANA_TRIM_REG
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cfg_sar_trim0 : 8; /**< ADC analog trim register 0 */
        unsigned int cfg_sar_trim1 : 8; /**< ADC analog trim register 1 */
        unsigned int reserved0 : 16;
    } BIT;
} volatile ADC_ANA_TRIM_REG;

/**
  * @brief Define the ADC_RegStruct
  */
typedef struct {
    ADC_RESULT0_REG ADC_RESULT0;                 /**< Offset address: 0x00000000U, Result register0 */
    ADC_RESULT1_REG ADC_RESULT1;                 /**< Offset address: 0x00000004U, Result register1 */
    ADC_RESULT2_REG ADC_RESULT2;                 /**< Offset address: 0x00000008U, Result register2 */
    ADC_RESULT3_REG ADC_RESULT3;                 /**< Offset address: 0x0000000CU, Result register3 */
    ADC_RESULT4_REG ADC_RESULT4;                 /**< Offset address: 0x00000010U, Result register4 */
    ADC_RESULT5_REG ADC_RESULT5;                 /**< Offset address: 0x00000014U, Result register5 */
    ADC_RESULT6_REG ADC_RESULT6;                 /**< Offset address: 0x00000018U, Result register6 */
    ADC_RESULT7_REG ADC_RESULT7;                 /**< Offset address: 0x0000001CU, Result register7 */
    ADC_RESULT8_REG ADC_RESULT8;                 /**< Offset address: 0x00000020U, Result register8 */
    ADC_RESULT9_REG ADC_RESULT9;                 /**< Offset address: 0x00000024U, Result register9 */
    ADC_RESULT10_REG ADC_RESULT10;               /**< Offset address: 0x00000028U, Result register10 */
    ADC_RESULT11_REG ADC_RESULT11;               /**< Offset address: 0x0000002CU, Result register11 */
    ADC_RESULT12_REG ADC_RESULT12;               /**< Offset address: 0x00000030U, Result register12 */
    ADC_RESULT13_REG ADC_RESULT13;               /**< Offset address: 0x00000034U, Result register13 */
    ADC_RESULT14_REG ADC_RESULT14;               /**< Offset address: 0x00000038U, Result register14 */
    ADC_RESULT15_REG ADC_RESULT15;               /**< Offset address: 0x0000003CU, Result register15 */
    unsigned int space0[12];
    ADC_EOC_FLAG_REG ADC_EOC_FLAG;               /**< Offset address: 0x00000070U, EOC status register */
    unsigned int space1[35];
    ADC_SOC0_CFG_REG ADC_SOC0_CFG;               /**< Offset address: 0x00000100U, SOC0  configuration register */
    ADC_SOC1_CFG_REG ADC_SOC1_CFG;               /**< Offset address: 0x00000104U, SOC1  configuration register */
    ADC_SOC2_CFG_REG ADC_SOC2_CFG;               /**< Offset address: 0x00000108U, SOC2  configuration register */
    ADC_SOC3_CFG_REG ADC_SOC3_CFG;               /**< Offset address: 0x0000010CU, SOC3  configuration register */
    ADC_SOC4_CFG_REG ADC_SOC4_CFG;               /**< Offset address: 0x00000110U, SOC4  configuration register */
    ADC_SOC5_CFG_REG ADC_SOC5_CFG;               /**< Offset address: 0x00000114U, SOC5  configuration register */
    ADC_SOC6_CFG_REG ADC_SOC6_CFG;               /**< Offset address: 0x00000118U, SOC6  configuration register */
    ADC_SOC7_CFG_REG ADC_SOC7_CFG;               /**< Offset address: 0x0000011CU, SOC7  configuration register */
    ADC_SOC8_CFG_REG ADC_SOC8_CFG;               /**< Offset address: 0x00000120U, SOC8  configuration register */
    ADC_SOC9_CFG_REG ADC_SOC9_CFG;               /**< Offset address: 0x00000124U, SOC9  configuration register */
    ADC_SOC10_CFG_REG ADC_SOC10_CFG;             /**< Offset address: 0x00000128U, SOC10 configuration register */
    ADC_SOC11_CFG_REG ADC_SOC11_CFG;             /**< Offset address: 0x0000012CU, SOC11 configuration register */
    ADC_SOC12_CFG_REG ADC_SOC12_CFG;             /**< Offset address: 0x00000130U, SOC12 configuration register */
    ADC_SOC13_CFG_REG ADC_SOC13_CFG;             /**< Offset address: 0x00000134U, SOC13 configuration register */
    ADC_SOC14_CFG_REG ADC_SOC14_CFG;             /**< Offset address: 0x00000138U, SOC14 configuration register */
    ADC_SOC15_CFG_REG ADC_SOC15_CFG;             /**< Offset address: 0x0000013CU, SOC15 configuration register */
    unsigned int space2[8];
    ADC_SOFT_TRIG_REG ADC_SOFT_TRIG;             /**< Offset address: 0x00000160U, Software trigger register */
    unsigned int space3[39];
    ADC_ARBT0_REG ADC_ARBT0;                     /**< Offset address: 0x00000200U, Priority register0 */
    ADC_ARBT1_REG ADC_ARBT1;                     /**< Offset address: 0x00000204U, Priority register1 */
    ADC_ARBT2_REG ADC_ARBT2;                     /**< Offset address: 0x00000208U, Priority register2 */
    unsigned int space4[5];
    ADC_OVERSAMP_REG ADC_OVERSAMP;               /**< Offset address: 0x00000220U, Oversampling setting register */
    ADC_OVERSAMP_RESULT_REG ADC_OVERSAMP_RESULT; /**< Offset address: 0x00000224U, Oversampling result register */
    unsigned int space5[10];
    ADC_PPB0_CTRL0_REG ADC_PPB0_CTRL0;           /**< Offset address: 0x00000250U, PPB0 configuration register0 */
    ADC_PPB0_CTRL1_REG ADC_PPB0_CTRL1;           /**< Offset address: 0x00000254U, PPB0 configuration register1 */
    ADC_PPB0_CTRL2_REG ADC_PPB0_CTRL2;           /**< Offset address: 0x00000258U, PPB0 configuration register2 */
    ADC_PPB0_RESULT_REG ADC_PPB0_RESULT;         /**< Offset address: 0x0000025CU, PPB0 result register */
    ADC_PPB1_CTRL0_REG ADC_PPB1_CTRL0;           /**< Offset address: 0x00000260U, PPB1 configuration register0 */
    ADC_PPB1_CTRL1_REG ADC_PPB1_CTRL1;           /**< Offset address: 0x00000264U, PPB1 configuration register1 */
    ADC_PPB1_CTRL2_REG ADC_PPB1_CTRL2;           /**< Offset address: 0x00000268U, PPB1 configuration register2 */
    ADC_PPB1_RESULT_REG ADC_PPB1_RESULT;         /**< Offset address: 0x0000026CU, PPB1 result register */
    ADC_PPB2_CTRL0_REG ADC_PPB2_CTRL0;           /**< Offset address: 0x00000270U, PPB2 configuration register0 */
    ADC_PPB2_CTRL1_REG ADC_PPB2_CTRL1;           /**< Offset address: 0x00000274U, PPB2 configuration register1 */
    ADC_PPB2_CTRL2_REG ADC_PPB2_CTRL2;           /**< Offset address: 0x00000278U, PPB2 configuration register2 */
    ADC_PPB2_RESULT_REG ADC_PPB2_RESULT;         /**< Offset address: 0x0000027CU, PPB2 result register */
    ADC_PPB3_CTRL0_REG ADC_PPB3_CTRL0;           /**< Offset address: 0x00000280U, PPB3 configuration register0 */
    ADC_PPB3_CTRL1_REG ADC_PPB3_CTRL1;           /**< Offset address: 0x00000284U, PPB3 configuration register1 */
    ADC_PPB3_CTRL2_REG ADC_PPB3_CTRL2;           /**< Offset address: 0x00000288U, PPB3 configuration register2 */
    ADC_PPB3_RESULT_REG ADC_PPB3_RESULT;         /**< Offset address: 0x0000028CU, PPB3 result register */
    unsigned int space6[8];
    ADC_INT_DATA_0_REG ADC_INT_DATA_0;           /**< Offset address: 0x000002B0U, Data interrupt register0 */
    ADC_INT_DATA_1_REG ADC_INT_DATA_1;           /**< Offset address: 0x000002B4U, Data interrupt register1 */
    ADC_INT_DATA_FLAG_REG ADC_INT_DATA_FLAG;     /**< Offset address: 0x000002B8U, Raw data interrupt register */
    ADC_INT_DATA_MSK_REG ADC_INT_DATA_MSK;       /**< Offset address: 0x000002BCU, Masked data interrupt register */
    ADC_DATA_FLAG_MASK_REG ADC_DATA_FLAG_MASK;   /**< Offset address: 0x000002C0U, Data interrupt mask register */
    ADC_EVENT_INT_REG ADC_EVENT_INT;             /**< Offset address: 0x000002C4U, Raw event interrupt register */
    ADC_EVENT_INT_MSK_REG ADC_EVENT_INT_MSK;     /**< Offset address: 0x000002C8U, Masked event interrupt register */
    ADC_EVENT_INT_MASK_REG ADC_EVENT_INT_MASK;   /**< Offset address: 0x000002CCU, Event interrupt mask register */
    ADC_ERR_INT_REG ADC_ERR_INT;                 /**< Offset address: 0x000002D0U, Raw error interrupt register */
    ADC_ERR_INT_MSK_REG ADC_ERR_INT_MSK;         /**< Offset address: 0x000002D4U, Masked error interrupt register */
    ADC_ERR_INT_MASK_REG ADC_ERR_INT_MASK;       /**< Offset address: 0x000002D8U, Error interrupt mask register */
    unsigned int space7[5];
    ADC_DMA_REG ADC_DMA;                         /**< Offset address: 0x000002F0U, DMA configuration register */
    unsigned int space8[3];
    ADC_EN_REG ADC_EN;                           /**< Offset address: 0x00000300U, Enable Register */
    unsigned int space9[3];
    ADC_EN_DLY_REG ADC_EN_DLY;                   /**< Offset address: 0x00000310U, Enable Delay Register */
    unsigned int space10[59];
    ADC_MODE_REG ADC_MODE;                       /**< Offset address: 0x00000400U, Mode configuration register */
    unsigned int space11[7];
    ADC_OEGE_CH_SEL_REG ADC_OEGE_CH_SEL;         /**< Offset address: 0x00000420U, OE and GE channel register */
    unsigned int space12[7];
    ADC_OEGE_CTRL0_REG ADC_OEGE_CTRL0;           /**< Offset address: 0x00000440U, OE and GE configuration register0 */
    ADC_OEGE_CTRL1_REG ADC_OEGE_CTRL1;           /**< Offset address: 0x00000444U, OE and GE configuration register1 */
    ADC_OEGE_CTRL2_REG ADC_OEGE_CTRL2;           /**< Offset address: 0x00000448U, OE and GE configuration register2 */
    unsigned int space13[5];
    ADC_PROCESS0_REG ADC_PROCESS0;               /**< Offset address: 0x00000460U, Data processing register0 */
    ADC_PROCESS1_REG ADC_PROCESS1;               /**< Offset address: 0x00000464U, Data processing register1 */
    unsigned int space14[6];
    ADC_STATUS_REG ADC_STATUS;                   /**< Offset address: 0x00000480U, ADC status register */
    unsigned int space15[31];
    ADC_WEIGHT_CFG15_REG ADC_WEIGHT_CFG15;       /**< Offset address: 0x00000500U, Capacitor weight register */
    ADC_WEIGHT_CFG14_REG ADC_WEIGHT_CFG14;       /**< Offset address: 0x00000504U, Capacitor weight register */
    ADC_WEIGHT_CFG13_REG ADC_WEIGHT_CFG13;       /**< Offset address: 0x00000508U, Capacitor weight register */
    ADC_WEIGHT_CFG12_REG ADC_WEIGHT_CFG12;       /**< Offset address: 0x0000050CU, Capacitor weight register */
    ADC_WEIGHT_CFG11_REG ADC_WEIGHT_CFG11;       /**< Offset address: 0x00000510U, Capacitor weight register */
    ADC_WEIGHT_CFG10_REG ADC_WEIGHT_CFG10;       /**< Offset address: 0x00000514U, Capacitor weight register */
    ADC_WEIGHT_CFG9_REG ADC_WEIGHT_CFG9;         /**< Offset address: 0x00000518U, Capacitor weight register */
    ADC_WEIGHT_CFG8_REG ADC_WEIGHT_CFG8;         /**< Offset address: 0x0000051CU, Capacitor weight register */
    ADC_WEIGHT_CFG7_REG ADC_WEIGHT_CFG7;         /**< Offset address: 0x00000520U, Capacitor weight register */
    ADC_WEIGHT_CFG6_REG ADC_WEIGHT_CFG6;         /**< Offset address: 0x00000524U, Capacitor weight register */
    ADC_WEIGHT_CFG5_REG ADC_WEIGHT_CFG5;         /**< Offset address: 0x00000528U, Capacitor weight register */
    ADC_WEIGHT_CFG4_REG ADC_WEIGHT_CFG4;         /**< Offset address: 0x0000052CU, Capacitor weight register */
    ADC_WEIGHT_CFG3_REG ADC_WEIGHT_CFG3;         /**< Offset address: 0x00000530U, Capacitor weight register */
    unsigned int space16[3];
    ADC_CAP_TRG_REG ADC_CAP_TRG;                 /**< Offset address: 0x00000540U, Calibration enable register */
    unsigned int space17[3];
    ADC_CAP_M1_REG ADC_CAP_M1;                   /**< Offset address: 0x00000550U, Capacitor calibration register */
    unsigned int space18[64];
    ADC_ANA_CTRL0_REG ADC_ANA_CTRL0;             /**< Offset address: 0x00000654U, Analog register0 */
    unsigned int space19[106];
    ADC_TSENSOR_TRIM_REG ADC_TSENSOR_TRIM;       /**< Offset address: 0x00000800U, Tsensor trim register */
    ADC_OEGE_TRIM_REG ADC_OEGE_TRIM;             /**< Offset address: 0x00000804U, OE and GE common trim register */
    unsigned int space20[2];
    ADC_PGA0_OEGE_TRIM0_REG ADC_PGA0_OEGE_TRIM0; /**< Offset address: 0x00000810U, PGA0 OE and GE trim register0 */
    ADC_PGA0_OEGE_TRIM1_REG ADC_PGA0_OEGE_TRIM1; /**< Offset address: 0x00000814U, PGA0 OE and GE trim register1 */
    ADC_PGA0_OEGE_TRIM2_REG ADC_PGA0_OEGE_TRIM2; /**< Offset address: 0x00000818U, PGA0 OE and GE trim register2 */
    ADC_PGA0_OEGE_TRIM3_REG ADC_PGA0_OEGE_TRIM3; /**< Offset address: 0x0000081CU, PGA0 OE and GE trim register3 */
    ADC_PGA1_OEGE_TRIM0_REG ADC_PGA1_OEGE_TRIM0; /**< Offset address: 0x00000820U, PGA1 OE and GE trim register0 */
    ADC_PGA1_OEGE_TRIM1_REG ADC_PGA1_OEGE_TRIM1; /**< Offset address: 0x00000824U, PGA1 OE and GE trim register1 */
    ADC_PGA1_OEGE_TRIM2_REG ADC_PGA1_OEGE_TRIM2; /**< Offset address: 0x00000828U, PGA1 OE and GE trim register2 */
    ADC_PGA1_OEGE_TRIM3_REG ADC_PGA1_OEGE_TRIM3; /**< Offset address: 0x0000082CU, PGA1 OE and GE trim register3 */
    unsigned int space21[4];
    ADC_ANA_TRIM_REG ADC_ANA_TRIM;               /**< Offset address: 0x00000840U, Analog trim register */
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
 * @brief ADC sample input.
 * @details Channel type:
 * + ADC_CH_ADCINA0 -- ADCIN0 is converted, number 0
 * + ADC_CH_ADCINA1 -- ADCIN1 is converted, number 1
 * + ADC_CH_ADCINA2 -- ADCIN2 is converted, number 2
 * + ADC_CH_ADCINA3 -- ADCIN3 is converted, number 3
 * + ADC_CH_ADCINA4 -- ADCIN4 is converted, number 4
 * + ADC_CH_ADCINA5 -- ADCIN5 is converted, number 5
 * + ADC_CH_ADCINA6 -- ADCIN6 is converted, number 6
 * + ADC_CH_ADCINA7 -- ADCIN7 is converted, number 7
 * + ADC_CH_ADCINA8 -- ADCIN8 is converted, number 8
 * + ADC_CH_ADCINA9 -- ADCIN9 is converted, number 9
 * + ADC_CH_ADCINA10 -- ADCIN10 is converted, number 10
 * + ADC_CH_ADCINA11 -- ADCIN11 is converted, number 11
 * + ADC_CH_ADCINA12 -- ADCIN12 is converted, number 12
 * + ADC_CH_ADCINA13 -- ADCIN13 is converted, number 13
 * + ADC_CH_ADCINA14 -- ADCIN14 is converted, number 14
 * + ADC_CH_ADCINA15 -- ADCIN15 is converted, number 15
 * + ADC_CH_ADCINA16 -- ADCIN16 is converted, number 16
 * + ADC_CH_ADCINA17 -- ADCIN17 is converted, number 17
 * + ADC_CH_ADCINA18 -- ADCIN18 is converted, number 18
 * + ADC_CH_ADCINA19 -- ADCIN19 is converted, number 19
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
    ADC_CH_ADCINA8 = 0x00000008U,
    ADC_CH_ADCINA9 = 0x00000009U,
    ADC_CH_ADCINA10 = 0x0000000AU,
    ADC_CH_ADCINA11 = 0x0000000BU,
    ADC_CH_ADCINA12 = 0x0000000CU,
    ADC_CH_ADCINA13 = 0x0000000DU,
    ADC_CH_ADCINA14 = 0x0000000EU,
    ADC_CH_ADCINA15 = 0x0000000FU,
    ADC_CH_ADCINA16 = 0x00000010U,
    ADC_CH_ADCINA17 = 0x00000011U,
    ADC_CH_ADCINA18 = 0x00000012U,
    ADC_CH_ADCINA19 = 0x00000013U,
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
 * + ADC_INT_NUMBER0 -- ADCINT0 interrupt
 * + ADC_INT_NUMBER1 -- ADCINT1 interrupt
 * + ADC_INT_NUMBER2 -- ADCINT2 interrupt
 * + ADC_INT_NUMBER3 -- ADCINT3 interrupt
 */
typedef enum {
    ADC_INT_NUMBER0 = 0x00000000U,
    ADC_INT_NUMBER1 = 0x00000001U,
    ADC_INT_NUMBER2 = 0x00000002U,
    ADC_INT_NUMBER3 = 0x00000003U
} ADC_IntNumber;

/**
 * @brief ADC supports peripherals trigger source.
 */
typedef enum {
    ADC_TRIGSOC_SOFT = 0x00000000U,
    ADC_TRIGSOC_APT0_SOCA = 0x00000001U,
    ADC_TRIGSOC_APT0_SOCB = 0x00000002U,
    ADC_TRIGSOC_APT1_SOCA = 0x00000003U,
    ADC_TRIGSOC_APT1_SOCB = 0x00000004U,
    ADC_TRIGSOC_APT2_SOCA = 0x00000005U,
    ADC_TRIGSOC_APT2_SOCB = 0x00000006U,
    ADC_TRIGSOC_APT3_SOCA = 0x00000007U,
    ADC_TRIGSOC_APT3_SOCB = 0x00000008U,
    ADC_TRIGSOC_GPT0 = 0x00000009U,
    ADC_TRIGSOC_GPT1 = 0x0000000AU,
    ADC_TRIGSOC_GPT2 = 0x0000000BU,
    ADC_TRIGSOC_GPT3 = 0x0000000CU,
    ADC_TRIGSOC_TIMER0 = 0x000000DU,
    ADC_TRIGSOC_TIMER1 = 0x000000EU,
    ADC_TRIGSOC_TIMER2 = 0x000000FU,
    ADC_TRIGSOC_TIMER3 = 0x00000010U,
    ADC_TRIGSOC_GPIOPD5 = 0x00000011U,
    ADC_TRIGSOC_GPIOPF3 = 0x00000012U,
    ADC_TRIGSOC_GPIOPF2 = 0x00000013U,
    ADC_TRIGSOC_GPIOPF1 = 0x00000014U,
} ADC_TrigSource;

/**
 * @brief The type of DMA request.
 * @details DMA request type:
 * + ADC_DMA_SINGLEREQ -- single request
 * + ADC_DMA_BURSTREQ  -- burst request
 */
typedef enum {
    ADC_DMA_SINGLEREQ = 0x00000000U,
    ADC_DMA_BURSTREQ = 0x00000001U
} ADC_DMARequestType;

/**
 * @brief The priority mode of SOCs sample simultaneously.
 * @details Priority mode:
 * + ADC_PRIMODE_ALL_ROUND -- Round robin mode is used for all
 * + ADC_PRIMODE_SOC0      -- SOC0 higher priority, others in round
 * + ADC_PRIMODE_TO_SOC1   -- SOC 0-1 higher priority, others in round
 * + ADC_PRIMODE_TO_SOC2   -- SOC 0-2 higher priority, others in round
 * + ADC_PRIMODE_TO_SOC3   -- SOC 0-3 higher priority, others in round
 * + ADC_PRIMODE_TO_SOC4   -- SOC 0-4 higher priority, others in round
 * + ADC_PRIMODE_TO_SOC5   -- SOC 0-5 higher priority, others in round
 * + ADC_PRIMODE_TO_SOC6   -- SOC 0-6 higher priority, others in round
 * + ADC_PRIMODE_TO_SOC7   -- SOC 0-7 higher priority, others in round
 * + ADC_PRIMODE_TO_SOC8   -- SOC 0-8 higher priority, others in round
 * + ADC_PRIMODE_TO_SOC9   -- SOC 0-9 higher priority, others in round
 * + ADC_PRIMODE_TO_SOC10  -- SOC 0-10 higher priority, others in round
 * + ADC_PRIMODE_TO_SOC11  -- SOC 0-11 higher priority, others in round
 * + ADC_PRIMODE_TO_SOC12  -- SOC 0-12 higher priority, others in round
 * + ADC_PRIMODE_TO_SOC13  -- SOC 0-13 higher priority, others in round
 * + ADC_PRIMODE_TO_SOC14  -- SOC 0-14 higher priority, others in round
 * + ADC_PRIMODE_ALL_PRIORITY -- SOC 0-15 higher priority, others in round
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
 * @brief ADC Oversampling Right Shift Bits.
 * @details:
 * + ADC_RIGHTSHIFT_BIT0 -- Non-displacement
 * + ADC_RIGHTSHIFT_BIT1 -- Shift 1 bit to the right
 * + ADC_RIGHTSHIFT_BIT2 -- Shift 2 bit to the right
 * + ADC_RIGHTSHIFT_BIT3 -- Shift 3 bit to the right
 * + ADC_RIGHTSHIFT_BIT4 -- Shift 4 bit to the right
 * + ADC_RIGHTSHIFT_BIT5 -- Shift 5 bit to the right
 * + ADC_RIGHTSHIFT_BIT6 -- Shift 6 bit to the right
 * + ADC_RIGHTSHIFT_BIT7 -- Shift 7 bit to the right
 * + ADC_RIGHTSHIFT_BIT8 -- Shift 8 bit to the right
 */
typedef enum {
    ADC_RIGHTSHIFT_BIT0 = 0x00000000U,
    ADC_RIGHTSHIFT_BIT1 = 0x00000001U,
    ADC_RIGHTSHIFT_BIT2 = 0x00000002U,
    ADC_RIGHTSHIFT_BIT3 = 0x00000003U,
    ADC_RIGHTSHIFT_BIT4 = 0x00000004U,
    ADC_RIGHTSHIFT_BIT5 = 0x00000005U,
    ADC_RIGHTSHIFT_BIT6 = 0x00000006U,
    ADC_RIGHTSHIFT_BIT7 = 0x00000007U,
    ADC_RIGHTSHIFT_BIT8 = 0x00000008U,
} ADC_OversamplingRightShift;

/**
 * @brief ADC Oversampling Multiple.
 * @details:
 * + ADC_OVERSAMPLING_8X -- The sampling result is 8 times
 * + ADC_OVERSAMPLING_16X -- The sampling result is 16 times
 * + ADC_OVERSAMPLING_32X -- The sampling result is 32 times
 * + ADC_OVERSAMPLING_64X -- The sampling result is 64 times
 * + ADC_OVERSAMPLING_128X -- The sampling result is 128 times
 * + ADC_OVERSAMPLING_256X -- The sampling result is 256 times
 */
typedef enum {
    ADC_OVERSAMPLING_8X = 0x00000003U,
    ADC_OVERSAMPLING_16X = 0x00000004U,
    ADC_OVERSAMPLING_32X = 0x00000005U,
    ADC_OVERSAMPLING_64X = 0x00000006U,
    ADC_OVERSAMPLING_128X = 0x00000007U,
    ADC_OVERSAMPLING_256X = 0x00000008U,
} ADC_OversamplingMultiple;

/**
 * @brief ADC sampling time, unit: adc_clk.
 */
typedef enum {
    ADC_SOCSAMPLE_5CLK = 0x00000000U,
    ADC_SOCSAMPLE_7CLK = 0x00000001U,
    ADC_SOCSAMPLE_10CLK = 0x00000002U,
    ADC_SOCSAMPLE_12CLK = 0x00000003U,
    ADC_SOCSAMPLE_15CLK = 0x00000004U,
    ADC_SOCSAMPLE_22CLK = 0x00000005U,
    ADC_SOCSAMPLE_30CLK = 0x00000006U,
    ADC_SOCSAMPLE_50CLK = 0x00000007U,
    ADC_SOCSAMPLE_75CLK = 0x00000008U,
    ADC_SOCSAMPLE_100CLK = 0x00000009U,
    ADC_SOCSAMPLE_125CLK = 0x0000000AU,
    ADC_SOCSAMPLE_150CLK = 0x0000000BU,
    ADC_SOCSAMPLE_200CLK = 0x0000000CU,
    ADC_SOCSAMPLE_300CLK = 0x0000000DU,
    ADC_SOCSAMPLE_400CLK = 0x0000000EU,
    ADC_SOCSAMPLE_500CLK = 0x0000000FU
} ADC_SOCSampleCycle;

/**
 * @brief The mode of SOCs finish sample and conversion.
 * @details Priority mode:
 * + ADC_SOCFINISH_NONE -- Interruption and DMA are not reported when sampling is complete
 * + ADC_SOCFINISH_DMA  -- DMA is reported when sampling is complete
 * + ADC_SOCFINISH_INT0 -- Interruption 0 is reported when sampling is complete
 * + ADC_SOCFINISH_INT1 -- Interruption 1 is reported when sampling is complete
 * + ADC_SOCFINISH_INT2 -- Interruption 2 is reported when sampling is complete
 * + ADC_SOCFINISH_INT3 -- Interruption 3 is reported when sampling is complete
 */
typedef enum {
    ADC_SOCFINISH_NONE = 0x00000001U,
    ADC_SOCFINISH_DMA = 0x00000002U,
    ADC_SOCFINISH_INT0 = 0x00000003U,
    ADC_SOCFINISH_INT1 = 0x00000004U,
    ADC_SOCFINISH_INT2 = 0x00000005U,
    ADC_SOCFINISH_INT3 = 0x00000006U
} ADC_SOCFinishMode;

/**
 * @brief The mode of ADC work.
 * @details Priority mode:
 * + ADC_WORKMODE_NORMAL     -- Normal Work
 * + ADC_WORKMODE_CAPACITY1  -- Calibration Mode 1
 */
typedef enum {
    ADC_WORKMODE_NORMAL = 0x00000000U,
    ADC_WORKMODE_CAPACITY1 = 0x00000001U,
} ADC_WorkMode;

/**
 * @brief The type of interrupt call back functions.
 */
typedef enum {
    ADC_CALLBACK_INT0 = 0x00000000U,
    ADC_CALLBACK_INT1 = 0x00000001U,
    ADC_CALLBACK_INT2 = 0x00000002U,
    ADC_CALLBACK_INT3 = 0x00000003U,
    ADC_CALLBACK_DMA = 0x000000004U,
    ADC_CALLBACK_DMAERROR = 0x000000005U,
    ADC_CALLBACK_DMAOVER = 0x00000006U,
    ADC_CALLBACK_TRIGOVER = 0x00000007U,
    ADC_CALLBACK_EVENT_OVERSAMPLING = 0x000000008U,
    ADC_CALLBACK_EVENT_PPB0_ZERO = 0x00000010U,
    ADC_CALLBACK_EVENT_PPB0_UP = 0x00000011U,
    ADC_CALLBACK_EVENT_PPB0_DOWN = 0x00000012U,
    ADC_CALLBACK_EVENT_PPB0_ERROR = 0x00000013U,
    ADC_CALLBACK_EVENT_PPB1_ZERO = 0x000000014U,
    ADC_CALLBACK_EVENT_PPB1_UP = 0x00000015U,
    ADC_CALLBACK_EVENT_PPB1_DOWN = 0x00000016U,
    ADC_CALLBACK_EVENT_PPB1_ERROR = 0x00000017U,
    ADC_CALLBACK_EVENT_PPB2_ZERO = 0x00000018U,
    ADC_CALLBACK_EVENT_PPB2_UP = 0x00000019U,
    ADC_CALLBACK_EVENT_PPB2_DOWN = 0x0000001AU,
    ADC_CALLBACK_EVENT_PPB2_ERROR = 0x0000001BU,
    ADC_CALLBACK_EVENT_PPB3_ZERO = 0x0000001CU,
    ADC_CALLBACK_EVENT_PPB3_UP = 0x0000001DU,
    ADC_CALLBACK_EVENT_PPB3_DOWN = 0x0000001EU,
    ADC_CALLBACK_EVENT_PPB3_ERROR = 0x0000001FU,
} ADC_CallbackFunType;

/**
 * @brief PPB function enable bit.
 */
typedef struct {
    bool detect;    /**< Function: zero-crossing detection, upper threshold detection, and lower threshold detection */
    bool offset;    /**< Result Data Offset */
    bool delay;     /**< Recording the sampling delay */
} PPB_Function;

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
  * @brief The definition of synchronous sampling parameter structure.
  */
typedef struct {
    ADC_OversamplingMultiple    multiple;           /**< Multiplier of Oversampling Accumulation */
    ADC_OversamplingRightShift  rightShift;         /**< Select sampling accuracy by shifting right bits */
    bool                        oversamplingInt;    /**< Select sampling accuracy by shifting right bits */
} ADC_OversamplingParam;

/**
  * @brief The definition of SOC parameter structure.
  */
typedef struct {
    ADC_Input           adcInput;           /**< SOC specified input */
    ADC_SOCSampleCycle  sampleTotalTime;    /**< SOC specified input sample total time */
    ADC_TrigSource      trigSource;         /**< SOC specified input periph trigger source */
    bool                continueMode;       /**< SOC specified input interrupt trigger source */
    ADC_SOCFinishMode   finishMode;         /**< SOC specified input mode of finishing sample and conversion */
} SOC_Param;

/**
  * @brief The definition of ADC overflow status.
  */
typedef union {
    unsigned int trigOver;
    unsigned int dmaReqOver;
} ADC_OverState;

/**
  * @brief The definition of extend handle structure.
  */
typedef struct _ADC_ExtendHandle {
} ADC_ExtendHandle;

/**
  * @brief The definition of callback.
  */
typedef struct {
    void (* Int0FinishCallBack)(void *handle); /**< ADC interrupt complete callback function for users */
    void (* Int1FinishCallBack)(void *handle); /**< ADC interrupt complete callback function for users */
    void (* Int2FinishCallBack)(void *handle); /**< ADC interrupt complete callback function for users */
    void (* Int3FinishCallBack)(void *handle); /**< ADC interrupt complete callback function for users */
    void (* DmaFinishCallBack)(void *handle);  /**< ADC DMA finish callback function for users */
    void (* OverSamplingFinishCallBack)(void *handle);  /**< ADC DMA finish callback function for users */
    void (* DmaErrorCallBack)(void *handle);   /**< ADC DMA transmission error callback function for users */
    void (* DmaOverCallBack)(void *handle);    /**< ADC DMA overflow callback function for users */
    void (* TrigOverCallBack)(void *handle);   /**< ADC DMA overflow callback function for users */
    void (* PPBEventCallBack[EVENT_TYPE])(void *handle); /**< (PPB0~PPB3) PPBx_ZRRO, PPBx_UP,PPBx_DOWN, PPBx_ERROR */
} ADC_UserCallBack;
/**
 * @}
 */

/* ADC DCL Functions */
/**
 * @brief Check ADC PPB.
 * @param ppb PPB number of ADC.
 * @retval bool
 */
static inline bool IsADCPostProcessingBlock(ADC_PPBNumber ppb)
{
    return (ppb >= ADC_PPB_NUM0) && (ppb <= ADC_PPB_NUM3);
}

/**
 * @brief Check ADC work mode.
 * @param mode work mode of ADC.
 * @retval bool
 */
static inline bool IsADCWorkMode(ADC_WorkMode mode)
{
    return (mode >= ADC_WORKMODE_NORMAL) && (mode <= ADC_WORKMODE_CAPACITY1);
}

/**
 * @brief Check ADC oversampling multiple parameter.
 * @param multiple oversampling multiple of SOC.
 * @retval bool
 */
static inline bool IsADCOversamplingMultiple(ADC_OversamplingMultiple multiple)
{
    return (multiple >= ADC_OVERSAMPLING_8X) && (multiple <= ADC_OVERSAMPLING_256X);
}

/**
 * @brief Check bit of right shift in oversampling.
 * @param bit bit of right shift.
 * @retval bool
 */
static inline bool IsADCOversamplingRightShift(ADC_OversamplingRightShift bit)
{
    return (bit >= ADC_RIGHTSHIFT_BIT0) && (bit <= ADC_RIGHTSHIFT_BIT8);
}

/**
 * @brief Check ADC sample input.
 * @param input Number of input.
 * @retval bool
 */
static inline bool IsADCSampleChannel(ADC_Input input)
{
    return (input >= ADC_CH_ADCINA0) && (input <= ADC_CH_ADCINA19);
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
    return (intx >= ADC_INT_NUMBER0) && (intx <= ADC_INT_NUMBER3);
}

/**
 * @brief Check SOC trigger source.
 * @param trig Type of trigger source.
 * @retval bool
 */
static inline bool IsADCTrigSource(ADC_TrigSource trig)
{
    return (trig >= ADC_TRIGSOC_SOFT) && (trig <= ADC_TRIGSOC_GPIOPF1);
}

/**
 * @brief Check SOC DMA Request Type.
 * @param dmaType Type of DMA Request.
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
    return (mode >= ADC_SOCFINISH_NONE) && (mode <= ADC_SOCFINISH_INT3);
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
 * @brief Check time of total ADC sampling time.
 * @param acqps Time of total ADC sampling time.
 * @retval bool
 */
static inline bool IsADCTotalTime(unsigned int acqps)
{
    return (acqps <= ADC_SOCSAMPLE_500CLK);
}

/**
 * @brief Configuring the interrupt source used by the SOC.
 * @param adcx ADC register base address.
 * @param socx Number of SOC, @ref ADC_SOCNumber.
 * @retval None.
 */
static inline void DCL_ADC_SetSOCxBlindInt0(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    unsigned int shiftBit = (unsigned int)socx;
    adcx->ADC_INT_DATA_0.reg |= (1U << shiftBit);
}

/**
 * @brief Obtains the SOC ID that use interrupt.
 * @param adcx ADC register base address.
 * @retval unsigned int, Obtains the SOC ID that uses this interrupt.
 */
static inline unsigned int DCL_ADC_GetSOCxBlindInt0(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_INT_DATA_0_REG value;
    value.reg = adcx->ADC_INT_DATA_0.reg;
    return value.BIT.cfg_intr_data_sel0;
}

/**
 * @brief Configuring the interrupt source used by the SOC.
 * @param adcx ADC register base address.
 * @param socx Number of SOC, @ref ADC_SOCNumber.
 * @retval None.
 */
static inline void DCL_ADC_SetSOCxBlindInt1(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    unsigned int shiftBit = (unsigned int)socx + 16; /* Offset 16 bits configuration */
    adcx->ADC_INT_DATA_0.reg |= (1U << shiftBit);
}

/**
 * @brief Obtains the SOC ID that use interrupt.
 * @param adcx ADC register base address.
 * @retval unsigned int, Obtains the SOC ID that uses this interrupt.
 */
static inline unsigned int DCL_ADC_GetSOCxBlindInt1(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_INT_DATA_0_REG value;
    value.reg = adcx->ADC_INT_DATA_0.reg;
    return value.BIT.cfg_intr_data_sel1;
}

/**
 * @brief Configuring the interrupt source used by the SOC.
 * @param adcx ADC register base address.
 * @param socx Number of SOC, @ref ADC_SOCNumber.
 * @retval None.
 */
static inline void DCL_ADC_SetSOCxBlindInt2(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    unsigned int shiftBit = (unsigned int)socx;
    adcx->ADC_INT_DATA_1.reg |= (1U << shiftBit);
}

/**
 * @brief Obtains the SOC ID that use interrupt.
 * @param adcx ADC register base address.
 * @retval unsigned int, Obtains the SOC ID that uses this interrupt.
 */
static inline unsigned int DCL_ADC_GetSOCxBlindInt2(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_INT_DATA_1_REG value;
    value.reg = adcx->ADC_INT_DATA_1.reg;
    return value.BIT.cfg_intr_data_sel2;
}

/**
 * @brief Configuring the interrupt source used by the SOC.
 * @param adcx ADC register base address.
 * @param socx Number of SOC, @ref ADC_SOCNumber.
 * @retval None.
 */
static inline void DCL_ADC_SetSOCxBlindInt3(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    unsigned int shiftBit = (unsigned int)socx + 16; /* Offset 16 bits configuration */
    adcx->ADC_INT_DATA_1.reg |= (1U << shiftBit);
}

/**
 * @brief Obtains the SOC ID that use interrupt.
 * @param adcx ADC register base address.
 * @retval unsigned int, Obtains the SOC ID that uses this interrupt.
 */
static inline unsigned int DCL_ADC_GetSOCxBlindInt3(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_INT_DATA_1_REG value;
    value.reg = adcx->ADC_INT_DATA_1.reg;
    return value.BIT.cfg_intr_data_sel3;
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
    adcx->ADC_DATA_FLAG_MASK.reg |= (1U << (unsigned int)intx);
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
    adcx->ADC_DATA_FLAG_MASK.reg &= ~(1U << (unsigned int)intx);
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
    adcx->ADC_INT_DATA_FLAG.reg = (1U << (unsigned int)intx);
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
    addr = (uintptr_t)(void *)&(adcx->ADC_SOC0_CFG);
    addr += ((unsigned int)socx * 4); /* Register base address difference 4 */
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
    ADC_SOC0_CFG_REG *soc = NULL;
    unsigned int addr = ADC_GetCTRLAddr(adcx, socx); /* Get the Address After Translation */
    soc = (ADC_SOC0_CFG_REG *)(void *)(uintptr_t)addr;
    soc->BIT.cfg_soc0_ch_sel = (unsigned int)input;
}

/**
 * @brief Configure the trigger source for the SOC.
 * @param adcx ADC register base address.
 * @param socx Number of SOC, @ref ADC_SOCNumber.
 * @param trig Source of trigger, @ref ADC_TrigSource.
 * @retval None.
 */
static inline void DCL_ADC_SOCxSelcetTrigSource(ADC_RegStruct * const adcx, ADC_SOCNumber socx, ADC_TrigSource trig)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx) && IsADCTrigSource(trig));
    unsigned int addr = ADC_GetCTRLAddr(adcx, socx); /* Obtaining the Address for Configuring the SOC */
    ADC_SOC0_CFG_REG *soc = NULL;
    soc = (ADC_SOC0_CFG_REG *)(void *)(uintptr_t)addr;
    soc->BIT.cfg_soc0_trig_sel = (unsigned int)trig;
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
    ADC_PARAM_CHECK_NO_RET(acqps <= 15); /* The value of acqps ranges from 0 to 15 */
    unsigned int addr = ADC_GetCTRLAddr(adcx, socx);
    ADC_SOC0_CFG_REG *soc = NULL;
    soc = (ADC_SOC0_CFG_REG *)(void *)(uintptr_t)addr;
    soc->BIT.cfg_soc0_samptime_sel = acqps;
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
 * @brief Multiple channels trigger software sampling.
 * @param adcx ADC register base address.
 * @param val The val bits range from 0 to 0xFFFF. Writing 1 indicates triggering.
 * @retval None.
 */
static inline void DCL_ADC_SOCxMultiSoftTrigger(ADC_RegStruct * const adcx, unsigned int val)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(val <= 0xFFFF); /* The value of val ranges from 0 to 0xFFFF */
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
    adcx->ADC_ARBT0.reg = priorityMode;
}

/**
 * @brief Get current poll pointer. This pointer holds the last converted poll SOC.
 * @param adcx ADC register base address.
 * @retval None.
 */
static inline unsigned int DCL_ADC_QueryPollPoint(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    return adcx->ADC_ARBT2.reg;
}

/**
 * @brief The poll pointer is reset by software. After the software is set to 1, the rr_pointer is set to 16.
 * @param adcx ADC register base address.
 * @retval None.
 */
static inline void DCL_ADC_ResetPollPoint(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_ARBT1.BIT.cfg_rr_pointer_reset = BASE_CFG_SET;
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
    adcx->ADC_DMA.BIT.cfg_dma_soc_sel = socx;
}

/**
 * @brief ADC enable DMA burst request.
 * @param adcx ADC register base address.
 * @retval None.
 */
static inline void DCL_ADC_EnableDMABurstReq(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_DMA.BIT.cfg_dma_brst_req_sel = BASE_CFG_ENABLE;
}

/**
 * @brief ADC disable DMA burst request.
 * @param adcx ADC register base address.
 * @retval None.
 */
static inline void DCL_ADC_DisableDMABurstReq(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_DMA.BIT.cfg_dma_brst_req_sel = BASE_CFG_DISABLE;
}

/**
 * @brief ADC enable DMA single request.
 * @param adcx ADC register base address.
 * @retval None.
 */
static inline void DCL_ADC_EnableDMASingleReq(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_DMA.BIT.cfg_dma_sing_req_sel = BASE_CFG_ENABLE;
}

/**
 * @brief ADC disable DMA single request.
 * @param adcx ADC register base address.
 * @retval None.
 */
static inline void DCL_ADC_DisableDMASingleReq(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_DMA.BIT.cfg_dma_sing_req_sel = BASE_CFG_DISABLE;
}

/**
 * @brief Configure post processing module(PPB) for the SOC.
 * @param adcx ADC register base address.
 * @param num Number of PPB.
 * @param socx Number of SOC, @ref ADC_SOCNumber.
 * @retval None.
 */
static inline void DCL_ADC_SOCxSelectPPBx(ADC_RegStruct * const adcx, ADC_PPBNumber num, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCPostProcessingBlock(num));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    ADC_PPB0_CTRL0_REG *ppb;
    uintptr_t addr = (uintptr_t)(void *)&(adcx->ADC_PPB0_CTRL0);
    ppb = (ADC_PPB0_CTRL0_REG *)(void *)(addr + 0x10U * (unsigned int)num); /* Get PPB configuration base address */
    ppb->BIT.cfg_ppb0_soc_sel = (unsigned int)socx;
}

/**
 * @brief Set the compensation offset.
 * @param adcx ADC register base address.
 * @param num Number of PPB.
 * @param offset Offset compensation value.
 * @retval None.
 */
static inline void DCL_ADC_SetPPBxOffset(ADC_RegStruct * const adcx, ADC_PPBNumber num, unsigned int offset)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCPostProcessingBlock(num));
    ADC_PARAM_CHECK_NO_RET(offset <= 0xFFF);
    ADC_PPB0_CTRL0_REG *ppb;
    uintptr_t addr = (uintptr_t)(void *)&(adcx->ADC_PPB0_CTRL0);
    ppb = (ADC_PPB0_CTRL0_REG *)(void *)(addr + 0x10U * (unsigned int)num); /* Get PPB configuration base address */
    ppb->BIT.cfg_ppb0_offset = offset;
}

/**
 * @brief Setting the PPB Function.
 * @param adcx ADC register base address.
 * @param num Number of PPB.
 * @param fun PPB function configuration.
 * @retval None.
 */
static inline void DCL_ADC_SetPPBxFunction(ADC_RegStruct * const adcx, ADC_PPBNumber num, PPB_Function *fun)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCPostProcessingBlock(num));
    ADC_ASSERT_PARAM(fun != NULL);
    ADC_PPB0_CTRL0_REG *ppb;
    uintptr_t addr = (uintptr_t)(void *)&(adcx->ADC_PPB0_CTRL0);
    ppb = (ADC_PPB0_CTRL0_REG *)(void *)(addr + 0x10U * (unsigned int)num);  /* Get PPB configuration base address */
    ppb->BIT.cfg_ppb0_detect_en = fun->detect;
    ppb->BIT.cfg_ppb0_offset_en = fun->offset;
    ppb->BIT.cfg_ppb0_dly_en = fun->delay;
}

/**
 * @brief Set the upper and down thresholds.
 * @param adcx ADC register base address.
 * @param num Number of PPB.
 * @param up Upper threshold, the most significant bit is the sign bit.
 * @param dn Down threshold, the most significant bit is the sign bit.
 * @retval None.
 */
static inline void DCL_ADC_SetPPBxThreshold(ADC_RegStruct * const adcx, ADC_PPBNumber num,
                                            unsigned int up, unsigned int dn)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCPostProcessingBlock(num));
    ADC_PARAM_CHECK_NO_RET(up <= 0x1FFF);
    ADC_PARAM_CHECK_NO_RET(dn <= 0x1FFF);
    ADC_PPB0_CTRL1_REG *ppb;
    uintptr_t addr = (uintptr_t)(void *)&(adcx->ADC_PPB0_CTRL1);
    ppb = (ADC_PPB0_CTRL1_REG *)(void *)(addr + 0x10U * (unsigned int)num);  /* Get PPB configuration base address */
    ppb->BIT.cfg_ppb0_uplimit = up;
    ppb->BIT.cfg_ppb0_dnlimit = dn;
}

/**
 * @brief Setting the Error reference value.
 * @param adcx ADC register base address.
 * @param num Number of PPB.
 * @param ref Error reference value.
 * @retval None.
 */
static inline void DCL_ADC_SetPPBxErrorRef(ADC_RegStruct * const adcx, ADC_PPBNumber num, unsigned int ref)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCPostProcessingBlock(num));
    ADC_PARAM_CHECK_NO_RET(ref <= 0xFFF);
    ADC_PPB0_CTRL2_REG *ppb;
    uintptr_t addr = (uintptr_t)(void *)&(adcx->ADC_PPB0_CTRL2);
    ppb = (ADC_PPB0_CTRL2_REG *)(void *)(addr + 0x10U * (unsigned int)num);  /* Get PPB configuration base address */
    ppb->BIT.cfg_ppb0_ref = ref;
}

/**
 * @brief Read sample delay count value.
 * @param adcx ADC register base address.
 * @param num Number of PPB.
 * @retval unsigned int, delay count value. The unit is the system frequency period.
 */
static inline unsigned int DCL_ADC_GetPPBxDelayCnt(ADC_RegStruct * const adcx, ADC_PPBNumber num)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_ASSERT_PARAM(IsADCPostProcessingBlock(num));
    ADC_PPB0_RESULT_REG *ppb;
    uintptr_t addr = (uintptr_t)(void *)&(adcx->ADC_PPB0_RESULT);
    ppb = (ADC_PPB0_RESULT_REG *)(void *)(addr + 0x10U * (unsigned int)num);
    unsigned int dly = ppb->reg;
    return (dly >> 16U);  /* dly_stamp is in the upper 16 bits */
}

/**
 * @brief Obtain the error calculation result.
 * @param adcx ADC register base address.
 * @param num Number of PPB.
 * @retval unsigned int, Error Result, the most significant bit is the sign bit.
 */
static inline unsigned int DCL_ADC_GetPPBxErrorResult(ADC_RegStruct * const adcx, ADC_PPBNumber num)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_ASSERT_PARAM(IsADCPostProcessingBlock(num));
    ADC_PPB0_RESULT_REG *ppb;
    uintptr_t addr = (uintptr_t)(void *)&(adcx->ADC_PPB0_RESULT);
    ppb = (ADC_PPB0_RESULT_REG *)(void *)(addr + 0x10U * (unsigned int)num);  /* Get PPB configuration base address */
    return ppb->BIT.ppb0_error_data;
}

/**
 * @brief Check whether the error calculation result is complete.
 * @param adcx ADC register base address.
 * @param num Number of PPB.
 * @retval unsigned int,   Not 0: Finish, 0: Not finish.
 */
static inline unsigned int DCL_ADC_CheckPPBxErrorResultFinish(ADC_RegStruct * const adcx, ADC_PPBNumber num)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_ASSERT_PARAM(IsADCPostProcessingBlock(num));
    unsigned int value = adcx->ADC_EVENT_INT.reg;
    unsigned int shfit = 3 + 4 * num;  /* 3 and 4 are used to convert the error result status bits */
    value = (value & (1U << shfit));
    return value;
}

/**
  * @brief Enable PPB interrupts.
  * @param adcx ADC register base address.
  * @retval void.
  */
static inline void DCL_ADC_EnablePPBxEventInt(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_EVENT_INT_MASK.reg |= 0xFFFF;
}

/**
  * @brief Disable PPB interrupts.
  * @param adcx ADC register base address.
  * @retval void.
  */
static inline void DCL_ADC_DisablePPBxEventInt(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    unsigned int value = adcx->ADC_EVENT_INT_MASK.reg;
    value = (value & 0xFFFF0000);
    adcx->ADC_EVENT_INT_MASK.reg = value;
}

/**
  * @brief Clear PPB interrupt.
  * @param adcx ADC register base address.
  * @retval void.
  */
static inline void DCL_ADC_ClearPPBxEventInt(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_EVENT_INT.reg |= 0xFFFF;
}

/**
  * @brief Get the status of event interrupt.
  * @param adcx ADC register base address.
  * @retval unsigned int, status of event interrupt.
  */
static inline unsigned int DCL_ADC_GetEventIntStatus(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    return adcx->ADC_EVENT_INT_MSK.reg;
}

/**
 * @brief Read ADC conversion result.
 * @param adcx ADC register base address.
 * @param socx Number of SOC, @ref ADC_SOCNumber.
 * @retval unsigned int, result.
 */
static inline unsigned int DCL_ADC_ReadSOCxResult(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_ASSERT_PARAM(IsADCSOCx(socx));
    ADC_RESULT0_REG *result;
    uintptr_t addr = (uintptr_t)(void *)adcx;
    /* The address interval of the result register is 4 */
    result = (ADC_RESULT0_REG *)(void *)(addr + 4 * (unsigned int)socx);
    return result->reg;
}

/**
 * @brief Obtain the ADC oversampling status.
 * @param adcx ADC register base address.
 * @retval unsigned int, 1: Finish, 0: Not finish.
 */
static inline unsigned int DCL_ADC_GetOversamplingState(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    return adcx->ADC_EVENT_INT.BIT.intr_oversamp_data_vld;
}

/**
 * @brief Reset the ADC oversampling status, also clear oversampling interrupt.
 * @param adcx ADC register base address.
 * @retval None.
 */
static inline void DCL_ADC_ResetOversamplingState(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_EVENT_INT.BIT.intr_oversamp_data_vld = BASE_CFG_SET;
}

/**
 * @brief Read ADC oversampling conversion result.
 * @param adcx ADC register base address.
 * @retval None.
 */
static inline unsigned int DCL_ADC_ReadOversamplingResult(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    return adcx->ADC_OVERSAMP_RESULT.reg;
}

/**
 * @brief Select SOC Progress Oversampling.
 * @param adcx ADC register base address.
 * @param socx Number of SOC, @ref ADC_SOCNumber.
 * @retval None.
 */
static inline void DCL_ADC_SoCSelectOversampling(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    adcx->ADC_OVERSAMP.BIT.cfg_oversamp_soc_sel = socx;
}

/**
 * @brief Setting Oversampling Parameters.
 * @param adcx ADC register base address.
 * @param multiple ADC oversampling Accumulation Multiple.
 * @param rightShift Number of bits shifted right in the oversampling result, used for truncation precision.
 * @retval None.
 */
static inline void DCL_ADC_SetOversamplingParam(ADC_RegStruct * const adcx, ADC_OversamplingMultiple multiple,
                                                ADC_OversamplingRightShift rightShift)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCOversamplingMultiple(multiple));
    ADC_PARAM_CHECK_NO_RET(IsADCOversamplingMultiple(rightShift));
    adcx->ADC_OVERSAMP.BIT.cfg_oversamp_n = multiple;   /* Configuring the Oversampling Multiple */
    adcx->ADC_OVERSAMP.BIT.cfg_oversamp_m = rightShift; /* Configuring the Bits Shifted Right in Oversampling */
}

/**
 * @brief Enable oversampling function.
 * @param adcx ADC register base address.
 * @retval None.
 */
static inline void DCL_ADC_EnableOversampling(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_OVERSAMP.BIT.cfg_oversamp_en = BASE_CFG_ENABLE;
}

/**
 * @brief Disbale oversampling function.
 * @param adcx ADC register base address.
 * @retval None.
 */
static inline void DCL_ADC_DisableOversampling(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_OVERSAMP.BIT.cfg_oversamp_en = BASE_CFG_DISABLE;
}

/**
 * @brief Enable ADC oversampling interrupt.
 * @param adcx ADC register base address.
 * @retval None.
 */
static inline void DCL_ADC_EnableOversamplingInt(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_EVENT_INT_MASK.BIT.intr_oversamp_data_vld_mask = BASE_CFG_ENABLE;
}

/**
 * @brief Disable ADC oversampling interrupt.
 * @param adcx ADC register base address.
 * @retval None.
 */
static inline void DCL_ADC_DisableOversamplingInt(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_EVENT_INT_MASK.BIT.intr_oversamp_data_vld_mask = BASE_CFG_DISABLE;
}

/**
 * @brief Enable Analog Power.
 * @param adcx ADC register base address.
 * @retval None.
 */
static inline void DCL_ADC_Enable(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_EN.BIT.cfg_adc_en = BASE_CFG_ENABLE;
}

/**
 * @brief Disable Analog Power.
 * @param adcx ADC register base address.
 * @retval None.
 */
static inline void DCL_ADC_Disable(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_EN.BIT.cfg_adc_en = BASE_CFG_DISABLE;
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
    ADC_ASSERT_PARAM(IsADCSOCx(socx));
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
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
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
    ADC_ASSERT_PARAM(IsADCSOCx(socx));
    unsigned int address = (uintptr_t)(void *)&(adcx->ADC_SOC0_CFG);
    address += ((unsigned int)socx * 4); /* Register base address difference 4 */
    ADC_SOC0_CFG_REG *soc = NULL;
    soc = (ADC_SOC0_CFG_REG *)(void *)(uintptr_t)address;
    return soc->BIT.cfg_soc0_ch_sel;
}

/**
 * @brief Enable SOC for continuous conversion.
 * @param adcx ADC register base address.
 * @param socx Number of SOC, @ref ADC_SOCNumber.
 * @retval None.
 */
static inline void DCL_ADC_EnableSOCxContinue(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    unsigned int addr = ADC_GetCTRLAddr(adcx, socx); /* Obtains the SOC base address  */
    ADC_SOC0_CFG_REG *soc = NULL;
    soc = (ADC_SOC0_CFG_REG *)(void *)(uintptr_t)addr;
    soc->BIT.cfg_soc0_cont_en = BASE_CFG_ENABLE;
}

/**
 * @brief Disable SOC for continuous conversion.
 * @param adcx ADC register base address.
 * @param socx Number of SOC, @ref ADC_SOCNumber.
 * @retval None.
 */
static inline void DCL_ADC_DisableSOCxContinue(ADC_RegStruct * const adcx, ADC_SOCNumber socx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    unsigned int addr = ADC_GetCTRLAddr(adcx, socx); /* Obtains the SOC base address  */
    ADC_SOC0_CFG_REG *soc = NULL;
    soc = (ADC_SOC0_CFG_REG *)(void *)(uintptr_t)addr;
    soc->BIT.cfg_soc0_cont_en = BASE_CFG_DISABLE;
}

/**
 * @brief Set the working mode.
 * @param adcx ADC register base address.
 * @param mode Work mode of ADC.
 * @retval None.
 */
static inline void DCL_ADC_SetWorkMode(ADC_RegStruct * const adcx, ADC_WorkMode mode)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCWorkMode(mode));
    adcx->ADC_MODE.reg = mode;
}

/**
 * @brief Enable error interrupt.
 * @param adcx ADC register base address.
 * @retval None.
 */
static inline void DCL_ADC_EnableErrorInt(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_ERR_INT_MASK.reg = 0x1FFFF;
}

/**
 * @brief Disbale error interrupt.
 * @param adcx ADC register base address.
 * @retval None.
 */
static inline void DCL_ADC_DisbaleErrorInt(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    adcx->ADC_ERR_INT_MASK.reg = BASE_CFG_DISABLE;
}

/**
 * @brief Clear error interrupt.
 * @param adcx ADC register base address.
 * @retval None.
 */
static inline void DCL_ADC_ClearErrorInt(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    unsigned int overFlag = adcx->ADC_ERR_INT.reg;
    adcx->ADC_ERR_INT.reg = overFlag;
}

/**
 * @brief Get status of error interrupt.
 * @param adcx ADC register base address.
 * @retval unsigned int, status of error interrupt.
 */
static inline unsigned int DCL_ADC_GetErrorIntStatus(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    return adcx->ADC_ERR_INT_MSK.reg;
}

/**
 * @brief Obtains the ADC controller status.
 * @param adcx ADC register base address.
 * @retval unsigned int, ADC controller status.
 */
static inline unsigned int DCL_ADC_GetControllerStatus(ADC_RegStruct * const adcx)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    return adcx->ADC_STATUS.reg;
}

/**
 * @brief Obtains the ADC controller status.
 * @param adcx ADC register base address.
 * @param socx Number of SOC, @ref ADC_SOCNumber.
 * @param offset Customized calibration offset parameter.12-bit signed decimal number. The most significant bit is the
 * sign bit, and the least significant bit is the one-bit decimal place. The value is stored in twos complement format.
 * @param gain Customized calibration gain parameter.13-bit unsigned decimal number. The most significant bit is an
 * integer bit, and the other bits are 12-bit decimal places.
 * @retval None.
 */
static inline void DCL_ADC_CalibrationGroup0(ADC_RegStruct * const adcx, ADC_SOCNumber socx,
                                             unsigned int offset, unsigned int gain)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    ADC_PARAM_CHECK_NO_RET(offset <= 0xFFF);
    ADC_PARAM_CHECK_NO_RET(gain <= 0x1FFF);
    adcx->ADC_OEGE_CH_SEL.BIT.cfg_oege_ch_sel0 = socx;
    adcx->ADC_OEGE_CTRL0.BIT.cfg_ofst0 = offset;    /* Configure the offset */
    adcx->ADC_OEGE_CTRL0.BIT.cfg_gain0 = gain;      /* Configure the gain */
}


/**
 * @brief Obtains the ADC controller status.
 * @param adcx ADC register base address.
 * @param socx Number of SOC, @ref ADC_SOCNumber.
 * @param offset Customized calibration offset parameter.12-bit signed decimal number. The most significant bit is the
 * sign bit, and the least significant bit is the one-bit decimal place. The value is stored in twos complement format.
 * @param gain Customized calibration gain parameter.13-bit unsigned decimal number. The most significant bit is an
 * integer bit, and the other bits are 12-bit decimal places.
 * @retval None.
 */
static inline void DCL_ADC_CalibrationGroup1(ADC_RegStruct * const adcx, ADC_SOCNumber socx,
                                             unsigned int offset, unsigned int gain)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    ADC_PARAM_CHECK_NO_RET(offset <= 0xFFF);
    ADC_PARAM_CHECK_NO_RET(gain <= 0x1FFF);
    adcx->ADC_OEGE_CH_SEL.BIT.cfg_oege_ch_sel1 = socx;
    adcx->ADC_OEGE_CTRL1.BIT.cfg_ofst1 = offset;    /* Configure the offset */
    adcx->ADC_OEGE_CTRL1.BIT.cfg_gain1 = gain;      /* Configure the gain */
}

/**
 * @brief Obtains the ADC controller status.
 * @param adcx ADC register base address.
 * @param socx Number of SOC, @ref ADC_SOCNumber.
 * @param offset Customized calibration offset parameter.12-bit signed decimal number. The most significant bit is the
 * sign bit, and the least significant bit is the one-bit decimal place. The value is stored in twos complement format.
 * @param gain Customized calibration gain parameter.13-bit unsigned decimal number. The most significant bit is an
 * integer bit, and the other bits are 12-bit decimal places.
 * @retval None.
 */
static inline void DCL_ADC_CalibrationGroup2(ADC_RegStruct * const adcx, ADC_SOCNumber socx,
                                             unsigned int offset, unsigned int gain)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_NO_RET(IsADCSOCx(socx));
    ADC_PARAM_CHECK_NO_RET(offset <= 0xFFF);
    ADC_PARAM_CHECK_NO_RET(gain <= 0x1FFF);
    adcx->ADC_OEGE_CH_SEL.BIT.cfg_oege_ch_sel2 = socx;
    adcx->ADC_OEGE_CTRL2.BIT.cfg_ofst2 = offset;    /* Configure the offset */
    adcx->ADC_OEGE_CTRL2.BIT.cfg_gain2 = gain;      /* Configure the gain */
}
/**
 * @}
 */

/**
 * @}
 */
#endif /* McuMagicTag_ADC_IP_H */