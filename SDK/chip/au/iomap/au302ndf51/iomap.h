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
  * @file      iomap.h
  * @author    MCU Driver Team
  * @brief     Defines chip pin map and function mode.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_IOMAP_H
#define McuMagicTag_IOMAP_H

#define IOCMG_REG_OFFSET_MASK           0xFFFF0000
#define IOCMG_FUNC_NUM_MASK             0x0000000F
#define IOCMG_REG_VALUE_MASK            0x0000FFFF
/* get offset value of member in type struct */
#define OFFSET_OF(type, member)  (unsigned int)(&(((type *)0)->member))

#define IOCMG_PIN_MUX(regx, funcNum, regValueDefault) \
        (unsigned int)(((OFFSET_OF(IOConfig_RegStruct, regx) & 0x00FF0000) << 8) | \
                      ((OFFSET_OF(IOConfig_RegStruct, regx) & 0x000000FF) << 16) | \
                      ((regValueDefault & 0xFFFFFFF0) | funcNum))
/* pin function mode info ---------------------------------------------------- */
#define IO48_AS_GPIO1_7        IOCMG_PIN_MUX(iocmg_18, FUNC_MODE_0, 0x0081)
#define IO48_AS_JTAG_TRSTN     IOCMG_PIN_MUX(iocmg_18, FUNC_MODE_1, 0x0081)
#define IO48_AS_CAPM1_SRC1     IOCMG_PIN_MUX(iocmg_18, FUNC_MODE_3, 0x0081)
#define IO48_AS_UART1_CTSN     IOCMG_PIN_MUX(iocmg_18, FUNC_MODE_4, 0x0081)
#define IO48_AS_SSP0_CSN0      IOCMG_PIN_MUX(iocmg_18, FUNC_MODE_5, 0x0081)
#define IO48_AS_ADTRG1         IOCMG_PIN_MUX(iocmg_18, FUNC_MODE_6, 0x0081)
#define IO48_AS_ADC1_ANA       IOCMG_PIN_MUX(iocmg_18, FUNC_MODE_8, 0x0081)
#define IO48_AS_ACMP1_ANA_N2   IOCMG_PIN_MUX(iocmg_18, FUNC_MODE_9, 0x0081)

#define IO1_AS_GPIO2_0        IOCMG_PIN_MUX(iocmg_19, FUNC_MODE_0, 0x0000)
#define IO1_AS_CAPM2_SRC1     IOCMG_PIN_MUX(iocmg_19, FUNC_MODE_3, 0x0000)
#define IO1_AS_SSP0_TXD       IOCMG_PIN_MUX(iocmg_19, FUNC_MODE_5, 0x0000)
#define IO1_AS_ADC1_ANA_B4    IOCMG_PIN_MUX(iocmg_19, FUNC_MODE_8, 0x0000)

#define IO2_AS_GPIO2_1        IOCMG_PIN_MUX(iocmg_20, FUNC_MODE_0, 0x0000)
#define IO2_AS_CAPM0_SRC1     IOCMG_PIN_MUX(iocmg_20, FUNC_MODE_3, 0x0000)
#define IO2_AS_SSP0_RXD       IOCMG_PIN_MUX(iocmg_20, FUNC_MODE_5, 0x0000)
#define IO2_AS_ADC1_ANA_B5    IOCMG_PIN_MUX(iocmg_20, FUNC_MODE_8, 0x0000)

#define IO3_AS_GPIO2_2        IOCMG_PIN_MUX(iocmg_21, FUNC_MODE_0, 0x0000)
#define IO3_AS_APT_EVTMP4     IOCMG_PIN_MUX(iocmg_21, FUNC_MODE_2, 0x0000)
#define IO3_AS_UART1_RTSN     IOCMG_PIN_MUX(iocmg_21, FUNC_MODE_4, 0x0000)
#define IO3_AS_SSP0_CSN1      IOCMG_PIN_MUX(iocmg_21, FUNC_MODE_5, 0x0000)
#define IO3_AS_ADST1          IOCMG_PIN_MUX(iocmg_21, FUNC_MODE_6, 0x0000)
#define IO3_AS_ADC1_ANA_A1    IOCMG_PIN_MUX(iocmg_21, FUNC_MODE_8, 0x0000)
#define IO3_AS_ACMP1_ANA_P2   IOCMG_PIN_MUX(iocmg_21, FUNC_MODE_9, 0x0000)

#define IO4_AS_GPIO2_3        IOCMG_PIN_MUX(iocmg_22, FUNC_MODE_0, 0x0000)
#define IO4_AS_SSP0_CLK       IOCMG_PIN_MUX(iocmg_22, FUNC_MODE_5, 0x0000)
#define IO4_AS_ADC1_ANA_A2    IOCMG_PIN_MUX(iocmg_22, FUNC_MODE_8, 0x0000)
#define IO4_AS_PGA1_ANA_P0    IOCMG_PIN_MUX(iocmg_22, FUNC_MODE_9, 0x0000)

#define IO5_AS_GPIO2_4        IOCMG_PIN_MUX(iocmg_23, FUNC_MODE_0, 0x0000)
#define IO5_AS_APT6_PWMA      IOCMG_PIN_MUX(iocmg_23, FUNC_MODE_3, 0x0000)
#define IO5_AS_ADC1_ANA_B2    IOCMG_PIN_MUX(iocmg_23, FUNC_MODE_8, 0x0000)
#define IO5_AS_PGA1_ANA_N0    IOCMG_PIN_MUX(iocmg_23, FUNC_MODE_9, 0x0000)

#define IO6_AS_GPIO2_5        IOCMG_PIN_MUX(iocmg_24, FUNC_MODE_0, 0x0000)
#define IO6_AS_APT7_PWMA      IOCMG_PIN_MUX(iocmg_24, FUNC_MODE_3, 0x0000)
#define IO6_AS_PGA1_ANA_EXT0  IOCMG_PIN_MUX(iocmg_24, FUNC_MODE_8, 0x0000)

#define IO7_AS_GPIO2_6        IOCMG_PIN_MUX(iocmg_25, FUNC_MODE_0, 0x0000)
#define IO7_AS_APT8_PWMA      IOCMG_PIN_MUX(iocmg_25, FUNC_MODE_3, 0x0000)
#define IO7_AS_POE0           IOCMG_PIN_MUX(iocmg_25, FUNC_MODE_4, 0x0000)
#define IO7_AS_ADST0          IOCMG_PIN_MUX(iocmg_25, FUNC_MODE_6, 0x0000)
#define IO7_AS_ADC1_ANA_A3    IOCMG_PIN_MUX(iocmg_25, FUNC_MODE_8, 0x0000)

#define IO8_AS_GPIO2_7        IOCMG_PIN_MUX(iocmg_26, FUNC_MODE_0, 0x0000)
#define IO8_AS_ACMP1_OUT      IOCMG_PIN_MUX(iocmg_26, FUNC_MODE_2, 0x0000)
#define IO8_AS_APT6_PWMB      IOCMG_PIN_MUX(iocmg_26, FUNC_MODE_3, 0x0000)
#define IO8_AS_ADC1_ANA_A4    IOCMG_PIN_MUX(iocmg_26, FUNC_MODE_8, 0x0000)

#define IO9_AS_GPIO3_0        IOCMG_PIN_MUX(iocmg_27, FUNC_MODE_0, 0x0000)
#define IO9_AS_APT7_PWMB      IOCMG_PIN_MUX(iocmg_27, FUNC_MODE_3, 0x0000)
#define IO9_AS_ADC1_ANA_A5    IOCMG_PIN_MUX(iocmg_27, FUNC_MODE_8, 0x0000)
#define IO9_AS_ACMP1_ANA_N3   IOCMG_PIN_MUX(iocmg_27, FUNC_MODE_9, 0x0000)

#define IO10_AS_GPIO3_1        IOCMG_PIN_MUX(iocmg_28, FUNC_MODE_0, 0x0000)
#define IO10_AS_APT8_PWMB      IOCMG_PIN_MUX(iocmg_28, FUNC_MODE_3, 0x0000)
#define IO10_AS_ADC1_ANA_A6    IOCMG_PIN_MUX(iocmg_28, FUNC_MODE_8, 0x0000)
#define IO10_AS_ACMP1_ANA_P3   IOCMG_PIN_MUX(iocmg_28, FUNC_MODE_9, 0x0000)

#define IO11_AS_GPIO3_5        IOCMG_PIN_MUX(iocmg_32, FUNC_MODE_0, 0x0000)
#define IO11_AS_APT0_PWMA      IOCMG_PIN_MUX(iocmg_32, FUNC_MODE_3, 0x0000)

#define IO12_AS_GPIO3_6        IOCMG_PIN_MUX(iocmg_33, FUNC_MODE_0, 0x0000)
#define IO12_AS_APT1_PWMA      IOCMG_PIN_MUX(iocmg_33, FUNC_MODE_3, 0x0000)
#define IO12_AS_DAC0_ANA_OUT   IOCMG_PIN_MUX(iocmg_33, FUNC_MODE_8, 0x0000)

#define IO13_AS_GPIO3_7        IOCMG_PIN_MUX(iocmg_34, FUNC_MODE_0, 0x0000)
#define IO13_AS_APT2_PWMA      IOCMG_PIN_MUX(iocmg_34, FUNC_MODE_3, 0x0000)
#define IO13_AS_DAC1_ANA_OUT   IOCMG_PIN_MUX(iocmg_34, FUNC_MODE_8, 0x0000)

#define IO14_AS_GPIO4_0        IOCMG_PIN_MUX(iocmg_35, FUNC_MODE_0, 0x0000)
#define IO14_AS_APT0_PWMB      IOCMG_PIN_MUX(iocmg_35, FUNC_MODE_3, 0x0000)
#define IO14_AS_DAC2_ANA_OUT   IOCMG_PIN_MUX(iocmg_35, FUNC_MODE_8, 0x0000)

#define IO15_AS_GPIO4_1        IOCMG_PIN_MUX(iocmg_36, FUNC_MODE_0, 0x0000)
#define IO15_AS_APT1_PWMB      IOCMG_PIN_MUX(iocmg_36, FUNC_MODE_3, 0x0000)
#define IO15_AS_ADC_OB_CLK     IOCMG_PIN_MUX(iocmg_36, FUNC_MODE_4, 0x0000)

#define IO16_AS_GPIO4_2        IOCMG_PIN_MUX(iocmg_37, FUNC_MODE_0, 0x0000)
#define IO16_AS_APT2_PWMB      IOCMG_PIN_MUX(iocmg_37, FUNC_MODE_3, 0x0000)
#define IO16_AS_ADC_OB_DATA    IOCMG_PIN_MUX(iocmg_37, FUNC_MODE_4, 0x0000)

#define IO17_AS_GPIO4_3        IOCMG_PIN_MUX(iocmg_38, FUNC_MODE_0, 0x0000)
#define IO17_AS_ADC0_ANA_A7    IOCMG_PIN_MUX(iocmg_38, FUNC_MODE_8, 0x0000)
#define IO17_AS_PGA0_ANA_P0    IOCMG_PIN_MUX(iocmg_38, FUNC_MODE_9, 0x0000)

#define IO18_AS_GPIO4_4        IOCMG_PIN_MUX(iocmg_39, FUNC_MODE_0, 0x0000)
#define IO18_AS_ADC0_ANA_B0    IOCMG_PIN_MUX(iocmg_39, FUNC_MODE_8, 0x0000)
#define IO18_AS_PGA0_ANA_N0    IOCMG_PIN_MUX(iocmg_39, FUNC_MODE_9, 0x0000)

#define IO19_AS_GPIO4_5        IOCMG_PIN_MUX(iocmg_40, FUNC_MODE_0, 0x0000)
#define IO19_AS_PGA0_ANA_EXT0  IOCMG_PIN_MUX(iocmg_40, FUNC_MODE_8, 0x0000)

#define IO22_AS_GPIO4_7        IOCMG_PIN_MUX(iocmg_42, FUNC_MODE_0, 0x0000)
#define IO22_AS_ADC0_ANA_A4    IOCMG_PIN_MUX(iocmg_42, FUNC_MODE_8, 0x0000)
#define IO22_AS_PGA1_ANAN_N3   IOCMG_PIN_MUX(iocmg_42, FUNC_MODE_9, 0x0000)
#define IO22_AS_ACMP0_ANA_N1   IOCMG_PIN_MUX(iocmg_42, FUNC_MODE_10, 0x0000)

#define IO23_AS_GPIO5_0        IOCMG_PIN_MUX(iocmg_43, FUNC_MODE_0, 0x0000)
#define IO23_AS_ACMP0_OUT      IOCMG_PIN_MUX(iocmg_43, FUNC_MODE_2, 0x0000)
#define IO23_AS_APT_EVTMP4     IOCMG_PIN_MUX(iocmg_43, FUNC_MODE_3, 0x0000)
#define IO23_AS_POE0           IOCMG_PIN_MUX(iocmg_43, FUNC_MODE_4, 0x0000)
#define IO23_AS_ADST0          IOCMG_PIN_MUX(iocmg_43, FUNC_MODE_6, 0x0000)
#define IO23_AS_PGA1_ANA_EXT1  IOCMG_PIN_MUX(iocmg_43, FUNC_MODE_9, 0x0000)

#define IO24_AS_GPIO5_5        IOCMG_PIN_MUX(iocmg_48, FUNC_MODE_0, 0x0000)
#define IO24_AS_SYS_RSTN_OUT   IOCMG_PIN_MUX(iocmg_48, FUNC_MODE_1, 0x0000)
#define IO24_AS_ADC2_ANA_B1    IOCMG_PIN_MUX(iocmg_48, FUNC_MODE_8, 0x0000)
#define IO24_AS_PGA2_ANA_P0    IOCMG_PIN_MUX(iocmg_48, FUNC_MODE_9, 0x0000)

#define IO25_AS_GPIO5_6        IOCMG_PIN_MUX(iocmg_49, FUNC_MODE_0, 0x0000)
#define IO25_AS_ADC2_ANA_A6    IOCMG_PIN_MUX(iocmg_49, FUNC_MODE_8, 0x0000)
#define IO25_AS_PGA2_ANA_N0    IOCMG_PIN_MUX(iocmg_49, FUNC_MODE_9, 0x0000)

#define IO26_AS_GPIO5_7        IOCMG_PIN_MUX(iocmg_50, FUNC_MODE_0, 0x0000)
#define IO26_AS_PGA2_ANA_EXT0  IOCMG_PIN_MUX(iocmg_50, FUNC_MODE_9, 0x0000)

#define IO29_AS_GPIO6_0        IOCMG_PIN_MUX(iocmg_51, FUNC_MODE_0, 0x0000)
#define IO29_AS_UART2_TXD      IOCMG_PIN_MUX(iocmg_51, FUNC_MODE_3, 0x0000)
#define IO29_AS_ADC2_ANA_A7    IOCMG_PIN_MUX(iocmg_51, FUNC_MODE_8, 0x0000)
#define IO29_AS_ACMP2_ANA_N1   IOCMG_PIN_MUX(iocmg_51, FUNC_MODE_9, 0x0000)

#define IO30_AS_GPIO6_1        IOCMG_PIN_MUX(iocmg_52, FUNC_MODE_0, 0x0000)
#define IO30_AS_UART2_RXD      IOCMG_PIN_MUX(iocmg_52, FUNC_MODE_3, 0x0000)
#define IO30_AS_ADC2_ANA_B0    IOCMG_PIN_MUX(iocmg_52, FUNC_MODE_8, 0x0000)
#define IO30_AS_ACMP2_ANA_P1   IOCMG_PIN_MUX(iocmg_52, FUNC_MODE_9, 0x0000)

#define IO31_AS_GPIO6_2        IOCMG_PIN_MUX(iocmg_53, FUNC_MODE_0, 0x0000)
#define IO31_AS_ACMP2_OUT      IOCMG_PIN_MUX(iocmg_53, FUNC_MODE_2, 0x0000)
#define IO31_AS_ADST2          IOCMG_PIN_MUX(iocmg_53, FUNC_MODE_6, 0x0000)
#define IO31_AS_ADC2_ANA_A1    IOCMG_PIN_MUX(iocmg_53, FUNC_MODE_8, 0x0000)

#define IO32_AS_GPIO6_3        IOCMG_PIN_MUX(iocmg_54, FUNC_MODE_0, 0x0000)
#define IO32_AS_POE2           IOCMG_PIN_MUX(iocmg_54, FUNC_MODE_2, 0x0000)
#define IO32_AS_CAN_RX         IOCMG_PIN_MUX(iocmg_54, FUNC_MODE_3, 0x0000)
#define IO32_AS_ADTRG2         IOCMG_PIN_MUX(iocmg_54, FUNC_MODE_6, 0x0000)
#define IO32_AS_ADC2_ANA_A2    IOCMG_PIN_MUX(iocmg_54, FUNC_MODE_8, 0x0000)

#define IO33_AS_GPIO6_4        IOCMG_PIN_MUX(iocmg_55, FUNC_MODE_0, 0x0000)
#define IO33_AS_APT_EVTMP6     IOCMG_PIN_MUX(iocmg_55, FUNC_MODE_2, 0x0000)
#define IO33_AS_CAN_TX         IOCMG_PIN_MUX(iocmg_55, FUNC_MODE_3, 0x0000)
#define IO33_AS_ADC2_ANA_B2    IOCMG_PIN_MUX(iocmg_55, FUNC_MODE_8, 0x0000)

#define IO34_AS_GPIO7_6        IOCMG_PIN_MUX(iocmg_66, FUNC_MODE_0, 0x0000)
#define IO34_AS_UART2_TXD      IOCMG_PIN_MUX(iocmg_66, FUNC_MODE_1, 0x0000)
#define IO34_AS_I2C0_SCL       IOCMG_PIN_MUX(iocmg_66, FUNC_MODE_2, 0x0000)
#define IO34_AS_DS_WAKEUP2     IOCMG_PIN_MUX(iocmg_66, FUNC_MODE_3, 0x0000)
#define IO34_AS_APT_EVTIO4     IOCMG_PIN_MUX(iocmg_66, FUNC_MODE_4, 0x0000)
#define IO34_AS_ADC2_ANA_A3    IOCMG_PIN_MUX(iocmg_66, FUNC_MODE_8, 0x0000)

#define IO35_AS_GPIO7_7        IOCMG_PIN_MUX(iocmg_67, FUNC_MODE_0, 0x0000)
#define IO35_AS_UART2_RXD      IOCMG_PIN_MUX(iocmg_67, FUNC_MODE_1, 0x0000)
#define IO35_AS_I2C0_SDA       IOCMG_PIN_MUX(iocmg_67, FUNC_MODE_2, 0x0000)
#define IO35_AS_DS_WAKEUP3     IOCMG_PIN_MUX(iocmg_67, FUNC_MODE_3, 0x0000)
#define IO35_AS_ADC2_ANA_A4    IOCMG_PIN_MUX(iocmg_67, FUNC_MODE_8, 0x0000)

#define IO36_AS_GPIO0_0        IOCMG_PIN_MUX(iocmg_0, FUNC_MODE_0, 0x0081)
#define IO36_AS_JTAG_TCK       IOCMG_PIN_MUX(iocmg_0, FUNC_MODE_1, 0x0081)
#define IO36_AS_DS_WAKEUP0     IOCMG_PIN_MUX(iocmg_0, FUNC_MODE_3, 0x0081)

#define IO37_AS_GPIO0_1        IOCMG_PIN_MUX(iocmg_1, FUNC_MODE_0, 0x0331)
#define IO37_AS_JTAG_TMS       IOCMG_PIN_MUX(iocmg_1, FUNC_MODE_1, 0x0331)
#define IO37_AS_DS_WAKEUP1     IOCMG_PIN_MUX(iocmg_1, FUNC_MODE_3, 0x0331)

#define IO39_AS_GPIO0_3        IOCMG_PIN_MUX(iocmg_6, FUNC_MODE_0, 0x0000)
#define IO39_AS_GPT0_PWM       IOCMG_PIN_MUX(iocmg_6, FUNC_MODE_2, 0x0000)
#define IO39_AS_UART0_TXD      IOCMG_PIN_MUX(iocmg_6, FUNC_MODE_4, 0x0000)
#define IO39_AS_XTAL_OUT       IOCMG_PIN_MUX(iocmg_6, FUNC_MODE_8, 0x0000)

#define IO40_AS_GPIO0_4        IOCMG_PIN_MUX(iocmg_7, FUNC_MODE_0, 0x0000)
#define IO40_AS_UART0_RXD      IOCMG_PIN_MUX(iocmg_7, FUNC_MODE_4, 0x0000)
#define IO40_AS_XTAL_IN        IOCMG_PIN_MUX(iocmg_7, FUNC_MODE_8, 0x0000)

#define IO41_AS_GPIO0_5        IOCMG_PIN_MUX(iocmg_8, FUNC_MODE_0, 0x0000)
#define IO41_AS_GPT0_PWM       IOCMG_PIN_MUX(iocmg_8, FUNC_MODE_1, 0x0000)
#define IO41_AS_APT_EVTMP5     IOCMG_PIN_MUX(iocmg_8, FUNC_MODE_2, 0x0000)
#define IO41_AS_CAPM1_SRC0     IOCMG_PIN_MUX(iocmg_8, FUNC_MODE_3, 0x0000)

#define IO42_AS_GPIO1_2        IOCMG_PIN_MUX(iocmg_13, FUNC_MODE_0, 0x0080)
#define IO42_AS_UPDATE_MODE    IOCMG_PIN_MUX(iocmg_13, FUNC_MODE_1, 0x0080)
#define IO42_AS_TEST_CLK       IOCMG_PIN_MUX(iocmg_13, FUNC_MODE_2, 0x0080)
#define IO42_AS_UART1_RTSN     IOCMG_PIN_MUX(iocmg_13, FUNC_MODE_4, 0x0080)

#define IO43_AS_GPIO1_3        IOCMG_PIN_MUX(iocmg_14, FUNC_MODE_0, 0x0211)
#define IO43_AS_JTAG_TDO       IOCMG_PIN_MUX(iocmg_14, FUNC_MODE_1, 0x0211)
#define IO43_AS_GPT1_PWM       IOCMG_PIN_MUX(iocmg_14, FUNC_MODE_2, 0x0211)
#define IO43_AS_CAPM2_SRC0     IOCMG_PIN_MUX(iocmg_14, FUNC_MODE_3, 0x0211)
#define IO43_AS_UART1_RXD      IOCMG_PIN_MUX(iocmg_14, FUNC_MODE_4, 0x0211)
#define IO43_AS_I2C0_SCL       IOCMG_PIN_MUX(iocmg_14, FUNC_MODE_5, 0x0211)

#define IO44_AS_GPIO1_4        IOCMG_PIN_MUX(iocmg_15, FUNC_MODE_0, 0x0001)
#define IO44_AS_JTAG_TDI       IOCMG_PIN_MUX(iocmg_15, FUNC_MODE_1, 0x0001)
#define IO44_AS_CAPM0_SRC0     IOCMG_PIN_MUX(iocmg_15, FUNC_MODE_3, 0x0001)
#define IO44_AS_UART1_TXD      IOCMG_PIN_MUX(iocmg_15, FUNC_MODE_4, 0x0001)
#define IO44_AS_I2C0_SDA       IOCMG_PIN_MUX(iocmg_15, FUNC_MODE_5, 0x0001)

#endif /* McuMagicTag_IOMAP_H */