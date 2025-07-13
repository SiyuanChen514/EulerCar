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
 * @file    ioconfig.h
 * @author  MCU Driver Team
 * @brief   ioconfig module driver
 * @details This file provides IOConfig register mapping structure.
 */

/* Macro definitions */
#ifndef McuMagicTag_IOCONFIG_H
#define McuMagicTag_IOCONFIG_H
typedef union {
    unsigned int reg;
    struct {
        unsigned int func : 4;           /**< IO function selection. */
        unsigned int ds : 2;                /**< Pin drive capability selection. */
        unsigned int reserved0 : 1;
        unsigned int pd : 1;                /**< Pin pull down control. */
        unsigned int pu : 1;                /**< Pin pull up control. */
        unsigned int sr : 1;                /**< Electrical level shift speed control. */
        unsigned int se : 1;                /**< Schmidt input control. */
        unsigned int osc_e : 1;             /**< XOUT/XIN pin crystal oscillator function enable. */
        unsigned int osc_ds : 2;            /**< XOUT/XIN pin crystal oscillator function drive capability selection. */
        unsigned int osc_ie : 1;            /**< XOUT/XIN pin crystal oscillator function clock output enable. */
        unsigned int reserved1 : 17;
    } BIT;
} volatile IOCMG_OSC_REG;

typedef union {
    unsigned int reg;
    struct {
        unsigned int func : 4;           /**< IO function selection. */
        unsigned int ds : 2;                /**< Pin drive capability selection. */
        unsigned int reserved0 : 1;
        unsigned int pd : 1;                /**< Pin pull down control. */
        unsigned int pu : 1;                /**< Pin pull up control. */
        unsigned int sr : 1;                /**< Electrical level shift speed control. */
        unsigned int se : 1;                /**< Schmidt input control. */
        unsigned int reserved1 : 21;
    } BIT;
} volatile IOCMG_REG;

typedef struct {
    IOCMG_REG iocmg_0; /**< Pin TCK/SWDCK IO Config Register, offset address:0x0U */
    IOCMG_REG iocmg_1; /**< Pin TMS/SWDIO IO Config Register, offset address:0x4U */
    IOCMG_REG iocmg_66; /**< Pin I2C0_SCL/UART2_TX IO Config Register, offset address:0x8U */
    IOCMG_REG iocmg_67; /**< Pin I2C0_SDA/UART2_RX IO Config Register, offset address:0xCU */
    unsigned char space0[65524];
    IOCMG_OSC_REG iocmg_6; /**< Pin XTAL_OUT IO Config Register, offset address:0x10004U */
    IOCMG_REG iocmg_7; /**< Pin XTAL_IN IO Config Register, offset address:0x10008U */
#if defined (AU301LDF51)
    unsigned char space1[4];
#else
    IOCMG_REG iocmg_8; /**< Pin GPT0 IO Config Register, offset address:0x1000CU */
#endif
#if defined (AU302NDF51) || defined (AU301LDF51)
    unsigned char space2[16];
#else
    IOCMG_REG iocmg_9; /**< Pin CAN_RX IO Config Register, offset address:0x10010U */
    IOCMG_REG iocmg_10; /**< Pin CAN_TX IO Config Register, offset address:0x10014U */
    IOCMG_REG iocmg_11; /**< Pin QDM_A IO Config Register, offset address:0x10018U */
    IOCMG_REG iocmg_12; /**< Pin QDM_B IO Config Register, offset address:0x1001CU */
#endif
    IOCMG_REG iocmg_13; /**< Pin BOOT/GPIO1_2 IO Config Register, offset address:0x10020U */
    IOCMG_REG iocmg_14; /**< Pin TDO/GPT1/UART1_RX IO Config Register, offset address:0x10024U */
    IOCMG_REG iocmg_15; /**< Pin TDI/CAPM0/UART1_TX IO Config Register, offset address:0x10028U */
    unsigned char space3[2097108];
#if defined (AU302NDF51) || defined (AU301LDF51)
    unsigned char space4[4];
#else
    IOCMG_REG iocmg_17; /**< Pin QDM_INDEX/POE1 IO Config Register, offset address:0x210000U */
#endif
    IOCMG_REG iocmg_18; /**< Pin TRSTN/ADC1_B3/CAPM1 IO Config Register, offset address:0x210004U */
#if defined (AU301LDF51)
    unsigned char space5[8];
#else
    IOCMG_REG iocmg_19; /**< Pin ADC1_B4/CAPM2 IO Config Register, offset address:0x210008U */
    IOCMG_REG iocmg_20; /**< Pin ADC1_B5 IO Config Register, offset address:0x21000CU */
#endif
    IOCMG_REG iocmg_21; /**< Pin ADC1_A1 IO Config Register, offset address:0x210010U */
    IOCMG_REG iocmg_22; /**< Pin ADC1_A2/PGA1IN_P0 IO Config Register, offset address:0x210014U */
    IOCMG_REG iocmg_23; /**< Pin ADC1_B2/PGA1IN_N0/APT6_A IO Config Register, offset address:0x210018U */
    IOCMG_REG iocmg_24; /**< Pin PGA1OUT0 IO Config Register, offset address:0x21001CU */
    IOCMG_REG iocmg_25; /**< Pin ADC1_A3 IO Config Register, offset address:0x210020U */
    IOCMG_REG iocmg_26; /**< Pin ADC1_A4/APT6_B IO Config Register, offset address:0x210024U */
#if defined (AU301LDF51)
    unsigned char space6[20];
#else
    IOCMG_REG iocmg_27; /**< Pin ADC1_A5/APT7_B IO Config Register, offset address:0x210028U */
    IOCMG_REG iocmg_28; /**< Pin ADC1_A6/APT8_B IO Config Register, offset address:0x21002CU */
    unsigned char space6[12];
#endif
    IOCMG_REG iocmg_32; /**< Pin APT0_A IO Config Register, offset address:0x21003CU */
    IOCMG_REG iocmg_33; /**< Pin APT1_A IO Config Register, offset address:0x210040U */
    IOCMG_REG iocmg_34; /**< Pin APT2_A IO Config Register, offset address:0x210044U */
    IOCMG_REG iocmg_35; /**< Pin APT0_B IO Config Register, offset address:0x210048U */
    IOCMG_REG iocmg_36; /**< Pin APT1_B IO Config Register, offset address:0x21004CU */
    IOCMG_REG iocmg_37; /**< Pin APT2_B IO Config Register, offset address:0x210050U */
#if defined (AU301LDF51)
    unsigned char space7[12];
#else
    IOCMG_REG iocmg_38; /**< Pin PGA0IN_P0 IO Config Register, offset address:0x210054U */
    IOCMG_REG iocmg_39; /**< Pin PGA0IN_N0 IO Config Register, offset address:0x210058U */
    IOCMG_REG iocmg_40; /**< Pin PGA0OUT IO Config Register, offset address:0x21005CU */
#endif
#if defined (AU302NDF51) || defined (AU301LDF51)
    unsigned char space8[4];
#else
    IOCMG_REG iocmg_41; /**< Pin GPIO4_6 IO Config Register, offset address:0x210060U */
#endif
#if defined (AU301LDF51)
    unsigned char space9[4];
#else
    IOCMG_REG iocmg_42; /**< Pin GPIO4_7 IO Config Register, offset address:0x210064U */
#endif
    IOCMG_REG iocmg_43; /**< Pin GPIO5_0/POE0 IO Config Register, offset address:0x210068U */
    unsigned char space10[16];
    IOCMG_REG iocmg_48; /**< Pin GPIO5_5 IO Config Register, offset address:0x21007CU */
    IOCMG_REG iocmg_49; /**< Pin GPIO5_6 IO Config Register, offset address:0x210080U */
    IOCMG_REG iocmg_50; /**< Pin GPIO5_7 IO Config Register, offset address:0x210084U */
#if defined (AU301LDF51)
    unsigned char space11[20];
#else
    IOCMG_REG iocmg_51; /**< Pin ADC2_A7 IO Config Register, offset address:0x210088U */
    IOCMG_REG iocmg_52; /**< Pin ADC2_B0 IO Config Register, offset address:0x21008CU */
    IOCMG_REG iocmg_53; /**< Pin ADC2_A1 IO Config Register, offset address:0x210090U */
    IOCMG_REG iocmg_54; /**< Pin ADC2_A2/POE2 IO Config Register, offset address:0x210094U */
    IOCMG_REG iocmg_55; /**< Pin ADC2_B2 IO Config Register, offset address:0x210098U */
#endif
#if defined (AU302NDF51) || defined (AU301LDF51)
    unsigned char space12[32];
#else
    unsigned char space12[8];
    IOCMG_REG iocmg_58; /**< Pin APT3_A IO Config Register, offset address:0x2100A4U */
    IOCMG_REG iocmg_59; /**< Pin APT4_A IO Config Register, offset address:0x2100A8U */
    IOCMG_REG iocmg_60; /**< Pin APT5_A IO Config Register, offset address:0x2100ACU */
    IOCMG_REG iocmg_61; /**< Pin APT3_B IO Config Register, offset address:0x2100B0U */
    IOCMG_REG iocmg_62; /**< Pin APT4_B IO Config Register, offset address:0x2100B4U */
    IOCMG_REG iocmg_63; /**< Pin APT5_B IO Config Register, offset address:0x2100B8U */
#endif
} volatile IOConfig_RegStruct;

#endif /* McuMagicTag_IOCONFIG_H */