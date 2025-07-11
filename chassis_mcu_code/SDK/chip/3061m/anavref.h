/**
 * @copyright Copyright (c) 2023, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
 * @file    anavref.h
 * @author  MCU Driver Team
 * @brief   anavref register mapping structure
 */

/* Macro definitions */
#ifndef McuMagicTag_ANAVREF_IP_H
#define McuMagicTag_ANAVREF_IP_H

#include "baseinc.h"

/**
  * @brief Define the union VREF_CTRL_REG0
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int da_ref_enh : 1; /**< vref enable */
        unsigned int reserved0 : 31;
    } BIT;
} volatile VREF_CTRL_REG0;

/**
  * @brief Define the union VREF_CTRL_REG1
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int da_ref_chop_enh : 1;      /**< vref chopping enable */
        unsigned int reserved0 : 15;
        unsigned int da_ref_temp_trim_enh : 1; /**< vref High-precision mode enable */
        unsigned int reserved1 : 15;
    } BIT;
} volatile VREF_CTRL_REG1;


/**
  * @brief Define the union VREF_TRIM_REG0
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int da_iref_trim : 8;     /**< iref trim */
        unsigned int da_ref_vref_trim : 8; /**< IBIAS voltage trim */
        unsigned int da_ref_vbg_trim : 8;  /**< vref voltage trim */
        unsigned int da_ref_buf_trim : 8;  /**< vref buffer trim */
    } BIT;
} volatile VREF_TRIM_REG0;

/**
  * @brief Define the union VREF_TRIM_REG1
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int da_ref_temp_trim3 : 8; /**< Benchmark temperature drift trim information */
        unsigned int da_ref_temp_trim2 : 8; /**< Benchmark temperature drift trim information */
        unsigned int da_ref_temp_trim1 : 8; /**< Benchmark temperature drift trim information */
        unsigned int da_ref_temp_trim0 : 8; /**< Benchmark temperature drift trim information */
    } BIT;
} volatile VREF_TRIM_REG1;

/**
  * @brief Define the VREF_RegStruct
  */
typedef struct {
    VREF_CTRL_REG0      VREF_CTRL0;     /**< Offset address: 0x0000000U*/
    unsigned int space0[7];
    VREF_CTRL_REG1      VREF_CTRL1;     /**< Offset address: 0x0000020U*/
    unsigned int space1[7];
    VREF_TRIM_REG0      VREF_TRIM0;     /**< Offset address: 0x0000040U*/
    VREF_TRIM_REG1      VREF_TRIM1;     /**< Offset address: 0x0000044U*/
} volatile VREF_RegStruct;

#endif /* McuMagicTag_ANAVREF_IP_H */