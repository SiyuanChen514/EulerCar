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
  * @file      fotp.h
  * @author    MCU Driver Team
  * @brief     This file provides firmware functions to manage the following
  *            functionalities of the system control register.
  *                + Register Struct of FOTP RNG0 and FOTP RNG1
  */
#ifndef McuMagicTag_FOTP_H
#define McuMagicTag_FOTP_H

#define FOTP_INFO_REG_MAX_ID   511 /* Max index of fotp info rng 0 and rng 1 */

typedef enum {
    FOTP_INFO_RNG0,
    FOTP_INFO_RNG1,
    FOTP_INFO_MAXTYPE,
} FOTP_InfoRngType;

typedef struct {
    unsigned int data[4];
} FOTP_CommonData;

/*
 * FOTP INFO RNG0
 */
typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    DIEID_STD_VER : 3;
            unsigned int    LOTID0        : 6;
            unsigned int    LOTID1        : 6;
            unsigned int    LOTID2        : 6;
            unsigned int    LOTID3        : 6;
            unsigned int    LOTID4        : 5;
        } data0;
        struct {
            unsigned int    LOTID4   : 1;
            unsigned int    LOTID5   : 6;
            unsigned int    WAFERID  : 5;
            unsigned int    DIEX     : 8;
            unsigned int    DIEY     : 8;
            unsigned int    reserved : 4;
        } data1;
        unsigned int        reserved;
        struct {
            unsigned int    ts_ref_t0_cp_rt : 11;
            unsigned int    ts_ref_v0_cp_rt : 11;
            unsigned int    reserved        : 2;
            unsigned int    CRC8_CP_RT      : 8;
        } data3;
    } REG;
} FOTP_INFO_RGN0_NUMBER_0;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    YEAR : 6;
            unsigned int    MON  : 4;
            unsigned int    DAY  : 5;
            unsigned int    HOUR : 5;
            unsigned int    MIN  : 6;
            unsigned int    SEC  : 6;
        } data0;
        unsigned int        reserved;
        struct {
            unsigned int    reserved         : 10;
            unsigned int    losc_ctrim_ft_rt : 8;
            unsigned int    hosc_ctrim_ft_rt : 9;
            unsigned int    SITE_NUM_FT      : 5;
        } data2;
        struct {
            unsigned int    ts_ref_t0_ft_rt : 11;
            unsigned int    ts_ref_v0_ft_rt : 11;
            unsigned int    reserved        : 10;
        } data3;
    } REG;
} FOTP_INFO_RGN0_NUMBER_1;

typedef union {
    FOTP_CommonData    comData;
    struct {
        unsigned int        chip_id;
        unsigned int        reserved;
        struct {
            unsigned int    version_id : 8;
            unsigned int    reserved   : 24;
        } data2;
        unsigned int        customer_id;
    } REG;
} FOTP_INFO_RGN0_NUMBER_2;

typedef union {
    FOTP_CommonData    comData;
    struct {
        unsigned int        fotp_empty_flag;
        unsigned int        reserved[3];
    } REG;
} FOTP_INFO_RGN0_NUMBER_4;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    info_rgn0_unlock : 1;
            unsigned int    reserved         : 31;
        } data0;
        unsigned int        reserved[3];
    } REG;
} FOTP_INFO_RGN0_NUMBER_5;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    bootrom_debug_enable : 1;
            unsigned int    reserved             : 31;
        } data0;
        struct {
            unsigned int    bootrom_hide_disable : 1;
            unsigned int    reserved             : 31;
        } data1;
        struct {
            unsigned int    ef_bist_jtag_enable : 1;
            unsigned int    reserved            : 31;
        } data2;
        unsigned int        reserved;
    } REG;
} FOTP_INFO_RGN0_NUMBER_6;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    cpu_fpu_enable  : 1;
            unsigned int    sysram_size_cfg : 1;
            unsigned int    eflash_size_cfg : 1;
            unsigned int    cpu_maxfreq_cfg : 1;
            unsigned int    reserved        : 28;
        } data0;
        unsigned int        reserved[3];
    } REG;
} FOTP_INFO_RGN0_NUMBER_7;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    adc0_enable : 1;
            unsigned int    adc1_enable : 1;
            unsigned int    adc2_enable : 1;
            unsigned int    reserved    : 29;
        } data0;
        struct {
            unsigned int    pga0_enable : 1;
            unsigned int    pga1_enable : 1;
            unsigned int    pga2_enable : 1;
            unsigned int    reserved    : 29;
        } data1;
        struct {
            unsigned int    dac0_enable : 1;
            unsigned int    dac1_enable : 1;
            unsigned int    dac2_enable : 1;
            unsigned int    reserved    : 29;
        } data2;
        struct {
            unsigned int    acmp0_enable : 1;
            unsigned int    acmp1_enable : 1;
            unsigned int    acmp2_enable : 1;
            unsigned int    reserved     : 29;
        } data3;
    } REG;
} FOTP_INFO_RGN0_NUMBER_8;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    apt0_enable : 1;
            unsigned int    apt1_enable : 1;
            unsigned int    apt2_enable : 1;
            unsigned int    apt3_enable : 1;
            unsigned int    apt4_enable : 1;
            unsigned int    apt5_enable : 1;
            unsigned int    apt6_enable : 1;
            unsigned int    apt7_enable : 1;
            unsigned int    apt8_enable : 1;
            unsigned int    reserved    : 23;
        } data0;
        unsigned int        reserved[3];
    } REG;
} FOTP_INFO_RGN0_NUMBER_9;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    IDDQ_DVDD : 8;
            unsigned int    IDDQ_AVDD : 8;
            unsigned int    reserved  : 16;
        } data0;
        unsigned int        reserved[3];
    } REG;
} FOTP_INFO_RGN0_NUMBER_10;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    hpm_core : 16;
            unsigned int    reserved : 16;
        } data0;
        unsigned int        reserved[3];
    } REG;
} FOTP_INFO_RGN0_NUMBER_11;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    PASSFLAG_CP_RT : 1;
            unsigned int    reserved       : 31;
        } data0;
        unsigned int        reserved[3];
    } REG;
} FOTP_INFO_RGN0_NUMBER_13;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    ts_ref_t0_ft_ht : 11;
            unsigned int    ts_ref_v0_ft_ht : 11;
            unsigned int    reserved        : 10;
        } data0;
        struct {
            unsigned int    pmu_bg_vo_h00_ht     : 12;
            unsigned int    pmu_bg_vo_h1f_ht     : 12;
            unsigned int    adcvref_bg_vo_h00_ht : 8;
        } data1;
        struct {
            unsigned int    adcvref_bg_vo_h00_ht : 4;
            unsigned int    adcvref_bg_vo_h08_ht : 12;
            unsigned int    adcvref_bg_vo_h10_ht : 12;
            unsigned int    adcvref_bg_vo_h18_ht : 4;
        } data2;
        struct {
            unsigned int    adcvref_bg_vo_h18_ht : 8;
            unsigned int    adcvref_bg_vo_h1f_ht : 12;
            unsigned int    reserved             : 4;
            unsigned int    CRC8_FT_RT           : 8;
        } data3;
    } REG;
} FOTP_INFO_RGN0_NUMBER_15;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    PASSFLAG_FT_HT : 1;
            unsigned int    reserved       : 31;
        } data0;
        unsigned int        reserved[3];
    } REG;
} FOTP_INFO_RGN0_NUMBER_16;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    DVS_FLOW_FLAG : 1;
            unsigned int    DVS_PASS_FLAG : 1;
            unsigned int    reserved      : 30;
        } data0;
        unsigned int        reserved[3];
    } REG;
} FOTP_INFO_RGN0_NUMBER_17;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    FAILFLAG_ALL : 2;
            unsigned int    reserved     : 30;
        } data0;
        unsigned int        reserved[3];
    } REG;
} FOTP_INFO_RGN0_NUMBER_18;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    PASSFLAG_FT_RT : 1;
            unsigned int    reserved       : 31;
        } data0;
        unsigned int        reserved[3];
    } REG;
} FOTP_INFO_RGN0_NUMBER_19;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    pmu_bg_vo_h00_rt     : 12;
            unsigned int    pmu_bg_vo_h1f_rt     : 12;
            unsigned int    adcvref_bg_vo_h00_rt : 8;
        } data0;
        struct {
            unsigned int    adcvref_bg_vo_h00_rt : 4;
            unsigned int    adcvref_bg_vo_h08_rt : 12;
            unsigned int    adcvref_bg_vo_h10_rt : 12;
            unsigned int    adcvref_bg_vo_h18_rt : 4;
        } data1;
        struct {
            unsigned int    adcvref_bg_vo_h18_rt : 8;
            unsigned int    adcvref_bg_vo_h1f_rt : 12;
            unsigned int    reserved             : 12;
        } data2;
        unsigned int        reserved;
    } REG;
} FOTP_INFO_RGN0_NUMBER_20;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    pmu_bg_trim    : 5;
            unsigned int    pmu_aoldo_trim : 5;
            unsigned int    pmu_cldo_trim  : 4;
            unsigned int    reserved       : 18;
        } data0;
        struct {
            unsigned int    adcvref_bg_trim     : 5;
            unsigned int    adcvref_adcldo_trim : 5;
            unsigned int    reserved            : 22;
        } data1;
        struct {
            unsigned int    adcvref_refbuf0_trim_2p0v : 5;
            unsigned int    adcvref_refbuf1_trim_2p0v : 5;
            unsigned int    adcvref_refbuf2_trim_2p0v : 5;
            unsigned int    reserved                  : 17;
        } data2;
        struct {
            unsigned int    adcvref_refbuf0_trim_2p5v : 5;
            unsigned int    adcvref_refbuf1_trim_2p5v : 5;
            unsigned int    adcvref_refbuf2_trim_2p5v : 5;
            unsigned int    reserved                  : 17;
        } data3;
    } REG;
} FOTP_INFO_RGN0_NUMBER_21;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    adc0_sh0_g0p6_ge_trim  : 13;
            unsigned int    adc0_sh0_g0p6_oe_trim  : 14;
            unsigned int    adc0_sh0_g0p6_trim_1d8 : 5;
        } data0;
        struct {
            unsigned int    adc0_sh0_g0p6_trim_1d8 : 2;
            unsigned int    adc0_sh0_g0p6_trim_7d8 : 7;
            unsigned int    adc0_sh0_g0p6_trim_3d8 : 7;
            unsigned int    adc0_sh0_g0p6_trim_4d8 : 7;
            unsigned int    adc0_sh0_g0p6_trim_5d8 : 7;
            unsigned int    reserved               : 2;
        } data1;
        struct {
            unsigned int    adc0_sh0_g0p75_ge_trim  : 13;
            unsigned int    adc0_sh0_g0p75_oe_trim  : 14;
            unsigned int    adc0_sh0_g0p75_trim_1d8 : 5;
        } data2;
        struct {
            unsigned int    adc0_sh0_g0p75_trim_1d8 : 2;
            unsigned int    adc0_sh0_g0p75_trim_7d8 : 7;
            unsigned int    adc0_sh0_g0p75_trim_3d8 : 7;
            unsigned int    adc0_sh0_g0p75_trim_4d8 : 7;
            unsigned int    adc0_sh0_g0p75_trim_5d8 : 7;
            unsigned int    reserved                : 2;
        } data3;
    } REG;
} FOTP_INFO_RGN0_NUMBER_22;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    adc0_sh1_g0p6_ge_trim  : 13;
            unsigned int    adc0_sh1_g0p6_oe_trim  : 14;
            unsigned int    adc0_sh1_g0p6_trim_1d8 : 5;
        } data0;
        struct {
            unsigned int    adc0_sh1_g0p6_trim_1d8 : 2;
            unsigned int    adc0_sh1_g0p6_trim_7d8 : 7;
            unsigned int    adc0_sh1_g0p6_trim_3d8 : 7;
            unsigned int    adc0_sh1_g0p6_trim_4d8 : 7;
            unsigned int    adc0_sh1_g0p6_trim_5d8 : 7;
            unsigned int    reserved               : 2;
        } data1;
        struct {
            unsigned int    adc0_sh1_g0p75_ge_trim  : 13;
            unsigned int    adc0_sh1_g0p75_oe_trim  : 14;
            unsigned int    adc0_sh1_g0p75_trim_1d8 : 5;
        } data2;
        struct {
            unsigned int    adc0_sh1_g0p75_trim_1d8 : 2;
            unsigned int    adc0_sh1_g0p75_trim_7d8 : 7;
            unsigned int    adc0_sh1_g0p75_trim_3d8 : 7;
            unsigned int    adc0_sh1_g0p75_trim_4d8 : 7;
            unsigned int    adc0_sh1_g0p75_trim_5d8 : 7;
            unsigned int    reserved                : 2;
        } data3;
    } REG;
} FOTP_INFO_RGN0_NUMBER_23;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    adc1_sh0_g0p6_ge_trim  : 13;
            unsigned int    adc1_sh0_g0p6_oe_trim  : 14;
            unsigned int    adc1_sh0_g0p6_trim_1d8 : 5;
        } data0;
        struct {
            unsigned int    adc1_sh0_g0p6_trim_1d8 : 2;
            unsigned int    adc1_sh0_g0p6_trim_7d8 : 7;
            unsigned int    adc1_sh0_g0p6_trim_3d8 : 7;
            unsigned int    adc1_sh0_g0p6_trim_4d8 : 7;
            unsigned int    adc1_sh0_g0p6_trim_5d8 : 7;
            unsigned int    reserved               : 2;
        } data1;
        struct {
            unsigned int    adc1_sh0_g0p75_ge_trim  : 13;
            unsigned int    adc1_sh0_g0p75_oe_trim  : 14;
            unsigned int    adc1_sh0_g0p75_trim_1d8 : 5;
        } data2;
        struct {
            unsigned int    adc1_sh0_g0p75_trim_1d8 : 2;
            unsigned int    adc1_sh0_g0p75_trim_7d8 : 7;
            unsigned int    adc1_sh0_g0p75_trim_3d8 : 7;
            unsigned int    adc1_sh0_g0p75_trim_4d8 : 7;
            unsigned int    adc1_sh0_g0p75_trim_5d8 : 7;
            unsigned int    reserved                : 2;
        } data3;
    } REG;
} FOTP_INFO_RGN0_NUMBER_24;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    adc1_sh1_g0p6_ge_trim  : 13;
            unsigned int    adc1_sh1_g0p6_oe_trim  : 14;
            unsigned int    adc1_sh1_g0p6_trim_1d8 : 5;
        } data0;
        struct {
            unsigned int    adc1_sh1_g0p6_trim_1d8 : 2;
            unsigned int    adc1_sh1_g0p6_trim_7d8 : 7;
            unsigned int    adc1_sh1_g0p6_trim_3d8 : 7;
            unsigned int    adc1_sh1_g0p6_trim_4d8 : 7;
            unsigned int    adc1_sh1_g0p6_trim_5d8 : 7;
            unsigned int    reserved               : 2;
        } data1;
        struct {
            unsigned int    adc1_sh1_g0p75_ge_trim  : 13;
            unsigned int    adc1_sh1_g0p75_oe_trim  : 14;
            unsigned int    adc1_sh1_g0p75_trim_1d8 : 5;
        } data2;
        struct {
            unsigned int    adc1_sh1_g0p75_trim_1d8 : 2;
            unsigned int    adc1_sh1_g0p75_trim_7d8 : 7;
            unsigned int    adc1_sh1_g0p75_trim_3d8 : 7;
            unsigned int    adc1_sh1_g0p75_trim_4d8 : 7;
            unsigned int    adc1_sh1_g0p75_trim_5d8 : 7;
            unsigned int    reserved                : 2;
        } data3;
    } REG;
} FOTP_INFO_RGN0_NUMBER_25;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    adc2_sh0_g0p6_ge_trim  : 13;
            unsigned int    adc2_sh0_g0p6_oe_trim  : 14;
            unsigned int    adc2_sh0_g0p6_trim_1d8 : 5;
        } data0;
        struct {
            unsigned int    adc2_sh0_g0p6_trim_1d8 : 2;
            unsigned int    adc2_sh0_g0p6_trim_7d8 : 7;
            unsigned int    adc2_sh0_g0p6_trim_3d8 : 7;
            unsigned int    adc2_sh0_g0p6_trim_4d8 : 7;
            unsigned int    adc2_sh0_g0p6_trim_5d8 : 7;
            unsigned int    reserved               : 2;
        } data1;
        struct {
            unsigned int    adc2_sh0_g0p75_ge_trim  : 13;
            unsigned int    adc2_sh0_g0p75_oe_trim  : 14;
            unsigned int    adc2_sh0_g0p75_trim_1d8 : 5;
        } data2;
        struct {
            unsigned int    adc2_sh0_g0p75_trim_1d8 : 2;
            unsigned int    adc2_sh0_g0p75_trim_7d8 : 7;
            unsigned int    adc2_sh0_g0p75_trim_3d8 : 7;
            unsigned int    adc2_sh0_g0p75_trim_4d8 : 7;
            unsigned int    adc2_sh0_g0p75_trim_5d8 : 7;
            unsigned int    reserved                : 2;
        } data3;
    } REG;
} FOTP_INFO_RGN0_NUMBER_26;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    adc2_sh1_g0p6_ge_trim  : 13;
            unsigned int    adc2_sh1_g0p6_oe_trim  : 14;
            unsigned int    adc2_sh1_g0p6_trim_1d8 : 5;
        } data0;
        struct {
            unsigned int    adc2_sh1_g0p6_trim_1d8 : 2;
            unsigned int    adc2_sh1_g0p6_trim_7d8 : 7;
            unsigned int    adc2_sh1_g0p6_trim_3d8 : 7;
            unsigned int    adc2_sh1_g0p6_trim_4d8 : 7;
            unsigned int    adc2_sh1_g0p6_trim_5d8 : 7;
            unsigned int    reserved               : 2;
        } data1;
        struct {
            unsigned int    adc2_sh1_g0p75_ge_trim  : 13;
            unsigned int    adc2_sh1_g0p75_oe_trim  : 14;
            unsigned int    adc2_sh1_g0p75_trim_1d8 : 5;
        } data2;
        struct {
            unsigned int    adc2_sh1_g0p75_trim_1d8 : 2;
            unsigned int    adc2_sh1_g0p75_trim_7d8 : 7;
            unsigned int    adc2_sh1_g0p75_trim_3d8 : 7;
            unsigned int    adc2_sh1_g0p75_trim_4d8 : 7;
            unsigned int    adc2_sh1_g0p75_trim_5d8 : 7;
            unsigned int    reserved                : 2;
        } data3;
    } REG;
} FOTP_INFO_RGN0_NUMBER_27;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    pga0_g1_error_trim_v0 : 8;
            unsigned int    pga0_g1_error_trim_v1 : 8;
            unsigned int    pga0_g1_error_trim_v2 : 8;
            unsigned int    pga0_g2_error_trim_v0 : 8;
        } data0;
        struct {
            unsigned int    pga0_g2_error_trim_v1 : 8;
            unsigned int    pga0_g2_error_trim_v2 : 8;
            unsigned int    pga0_g4_error_trim_v0 : 8;
            unsigned int    pga0_g4_error_trim_v1 : 8;
        } data1;
        struct {
            unsigned int    pga0_g4_error_trim_v2 : 8;
            unsigned int    pga0_g8_error_trim_v0 : 8;
            unsigned int    pga0_g8_error_trim_v1 : 8;
            unsigned int    pga0_g8_error_trim_v2 : 8;
        } data2;
        struct {
            unsigned int    pga0_g16_error_trim_v0 : 8;
            unsigned int    pga0_g16_error_trim_v1 : 8;
            unsigned int    pga0_g16_error_trim_v2 : 8;
            unsigned int    reserved               : 8;
        } data3;
    } REG;
} FOTP_INFO_RGN0_NUMBER_28;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    pga1_g1_error_trim_v0 : 8;
            unsigned int    pga1_g1_error_trim_v1 : 8;
            unsigned int    pga1_g1_error_trim_v2 : 8;
            unsigned int    pga1_g2_error_trim_v0 : 8;
        } data0;
        struct {
            unsigned int    pga1_g2_error_trim_v1 : 8;
            unsigned int    pga1_g2_error_trim_v2 : 8;
            unsigned int    pga1_g4_error_trim_v0 : 8;
            unsigned int    pga1_g4_error_trim_v1 : 8;
        } data1;
        struct {
            unsigned int    pga1_g4_error_trim_v2 : 8;
            unsigned int    pga1_g8_error_trim_v0 : 8;
            unsigned int    pga1_g8_error_trim_v1 : 8;
            unsigned int    pga1_g8_error_trim_v2 : 8;
        } data2;
        struct {
            unsigned int    pga1_g16_error_trim_v0 : 8;
            unsigned int    pga1_g16_error_trim_v1 : 8;
            unsigned int    pga1_g16_error_trim_v2 : 8;
            unsigned int    reserved               : 8;
        } data3;
    } REG;
} FOTP_INFO_RGN0_NUMBER_29;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    pga2_g1_error_trim_v0 : 8;
            unsigned int    pga2_g1_error_trim_v1 : 8;
            unsigned int    pga2_g1_error_trim_v2 : 8;
            unsigned int    pga2_g2_error_trim_v0 : 8;
        } data0;
        struct {
            unsigned int    pga2_g2_error_trim_v1 : 8;
            unsigned int    pga2_g2_error_trim_v2 : 8;
            unsigned int    pga2_g4_error_trim_v0 : 8;
            unsigned int    pga2_g4_error_trim_v1 : 8;
        } data1;
        struct {
            unsigned int    pga2_g4_error_trim_v2 : 8;
            unsigned int    pga2_g8_error_trim_v0 : 8;
            unsigned int    pga2_g8_error_trim_v1 : 8;
            unsigned int    pga2_g8_error_trim_v2 : 8;
        } data2;
        struct {
            unsigned int    pga2_g16_error_trim_v0 : 8;
            unsigned int    pga2_g16_error_trim_v1 : 8;
            unsigned int    pga2_g16_error_trim_v2 : 8;
            unsigned int    reserved               : 8;
        } data3;
    } REG;
} FOTP_INFO_RGN0_NUMBER_30;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    pga0_poffset_trim : 5;
            unsigned int    pga0_noffset_trim : 5;
            unsigned int    pga1_poffset_trim : 5;
            unsigned int    pga1_noffset_trim : 5;
            unsigned int    pga2_poffset_trim : 5;
            unsigned int    pga2_noffset_trim : 5;
            unsigned int    reserved          : 2;
        } data0;
        struct {
            unsigned int    reserved      : 15;
            unsigned int    CRC8_P1_RT_FT : 8;
            unsigned int    CRC8_P2_RT_FT : 8;
            unsigned int    reserved1     : 1;
        } data1;
        unsigned int        reserved[2];
    } REG;
} FOTP_INFO_RGN0_NUMBER_31;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    DIEX    : 8;
            unsigned int    DIEY    : 8;
            unsigned int    WAFERID : 8;
            unsigned int    LOTID0  : 8;
        } data0;
        struct {
            unsigned int    LOTID1 : 8;
            unsigned int    LOTID2 : 8;
            unsigned int    LOTID3 : 8;
            unsigned int    LOTID4 : 8;
        } data1;
        struct {
            unsigned int    LOTID5          : 8;
            unsigned int    PASSFLAG_RT_CPB : 1;
            unsigned int    PASSFLAG_RT_FTC : 1;
            unsigned int    reserved        : 22;
        } data2;
        unsigned int        reserved;
    } REG;
} FOTP_INFO_RGN0_NUMBER_507;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    ef_cp1_stress_flag : 16;
            unsigned int    reserved           : 16;
        } data0;
        unsigned int        reserved[3];
    } REG;
} FOTP_INFO_RGN0_NUMBER_508;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    ef_cp2_dr_flag : 16;
            unsigned int    reserved       : 16;
        } data0;
        unsigned int        reserved[3];
    } REG;
} FOTP_INFO_RGN0_NUMBER_509;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    ef_cp2_gdr : 16;
            unsigned int    reserved   : 16;
        } data0;
        unsigned int        reserved[3];
    } REG;
} FOTP_INFO_RGN0_NUMBER_510;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    ef_cp1_gdr : 16;
            unsigned int    reserved   : 16;
        } data0;
        unsigned int        reserved[3];
    } REG;
} FOTP_INFO_RGN0_NUMBER_511;

/*
 * FOTP INFO RNG1
 */
typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    lifecycle_status : 1;
            unsigned int    reserved         : 31;
        } data0;
        unsigned int        reserved[3];
    } REG;
} FOTP_INFO_RGN1_NUMBER_0;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    uart0_enable : 1;
            unsigned int    uart1_enable : 1;
            unsigned int    uart2_enable : 1;
            unsigned int    reserved     : 29;
        } data0;
        struct {
            unsigned int    func_jtag_enable : 1;
            unsigned int    reserved         : 31;
        } data1;
        struct {
            unsigned int    uart0_boot_enable : 1;
            unsigned int    reserved          : 31;
        } data2;
        struct {
            unsigned int    func_jtag_boot_enable : 1;
            unsigned int    reserved              : 31;
        } data3;
    } REG;
} FOTP_INFO_RGN1_NUMBER_1;

typedef union {
    FOTP_CommonData    comData;
    struct {
        struct {
            unsigned int    info_rgn1_unlock : 1;
            unsigned int    reserved         : 31;
        } data0;
        unsigned int        reserved[3];
    } REG;
} FOTP_INFO_RGN1_NUMBER_2;

#endif