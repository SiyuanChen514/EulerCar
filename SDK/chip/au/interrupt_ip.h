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
  * @file    interrupt_ip.h
  * @author  MCU Driver Team
  * @brief   interrupt module driver.
  *          This file define the interrupt number
  */

#ifndef MCUMagicTag_INTERRUPT_IP_H
#define MCUMagicTag_INTERRUPT_IP_H

/* Typedef definitions -------------------------------------------------------*/
#define MSTATUS_MIE            0x00000008U       /**< mie in mstatus */
#define MSTATUS_MPIE           0x00000080U       /**< mpie in mstatus */
#define UINT32_CUT_MASK        0xFFFFFFFFU

#define IRQ_PRIO_HIGHEST 7    /**< Highest priority of a hardware interrupt. */
#define IRQ_PRIO_LOWEST  1    /**< Lowest priority of a hardware interrupt. */

/**
 * @brief Count of system interrupt vector.
 *        The number of standard interrupts inside the CPU. The interrupt number
 *        is 0~25. The software interrupt nesting scheme cannot use standard
 *        interrupts, which means that external system integration will ensure
 *        that no standard interrupts will be triggered.
 */
#define IRQ_VECTOR_CNT               26

/**
 * @brief Count of local interrupt vector 0 - 5, enabled by CSR mie 26 -31 bit.
 */
#define IRQ_MIE_VECTOR_CNT            6

/**
 * @brief Count of IRQ controlled by CSR mie
 */
#define IRQ_MIE_TOTAL_CNT        (IRQ_VECTOR_CNT + IRQ_MIE_VECTOR_CNT)
#define IRQ_LOCIEN1_OFFSET       64
#define IRQ_LOCIEN2_OFFSET       96
#define IRQ_LOCIEN3_OFFSET       128

/**
  * @brief rv_custom_csr
  *        locipri0~15 are registers that control the priority of interrupts,
  *        and every 4 bits control the priority of an interrupt
  */
#define LOCIPRI0                0xBC0
#define LOCIPRI1                0xBC1
#define LOCIPRI2                0xBC2
#define LOCIPRI3                0xBC3
#define LOCIPRI4                0xBC4
#define LOCIPRI5                0xBC5
#define LOCIPRI6                0xBC6
#define LOCIPRI7                0xBC7
#define LOCIPRI8                0xBC8
#define LOCIPRI9                0xBC9
#define LOCIPRI10               0xBCA
#define LOCIPRI11               0xBCB
#define LOCIPRI12               0xBCC
#define LOCIPRI13               0xBCD
#define LOCIPRI14               0xBCE
#define LOCIPRI15               0xBCF

#define LOCIPRI(x)              LOCIPRI##x

/**
  * @brief locien0~3 are registers that control interrupt enable
  */
#define LOCIEN0                 0xBE0
#define LOCIEN1                 0xBE1
#define LOCIEN2                 0xBE2
#define LOCIEN3                 0xBE3

/**
  * @brief locipd0~3 are registers that control the interrupt flag bit. Each bit
  *        controls an interrupt. If the corresponding bit bit is 1, it means the
  *        corresponding interrupt is triggered.
  */
#define LOCIPD0                 0xBE8
#define LOCIPD1                 0xBE9
#define LOCIPD2                 0xBEA
#define LOCIPD3                 0xBEB

/**
  * @brief Locipclr is the register that clears the interrupt flag bit, and the
  *        corresponding interrupt number is assigned to the locipclr register,
  *        and the hardware will clear the corresponding interrupt flag bit, that
  *        is, the corresponding locipd bit is set
  */
#define LOCIPCLR                0xBF0

/**
  * @brief The maximum number of interrupts supported, excluding 26 internal standard
  *          interrupts, up to 230 external non-standard interrupts can be supported
  */
#define IRQ_NUM                 256

/* ---------- Interrupt Number Definition ----------------------------------- */
typedef enum {
    IRQ_SOFTWARE          = 26,   /* The first 0~25 interrupts are the internal standard interrupts of the CPU,
	                                   and the customizable external non-standard interrupts start from 26 */
    IRQ_UART0             = 28,
    IRQ_UART1             = 29,
    IRQ_UART2             = 30,

    IRQ_TIMER0            = 32,
    IRQ_TIMER1            = 33,
    IRQ_TIMER2            = 34,
    IRQ_TIMER3            = 35,

    IRQ_WDG               = 40,
    IRQ_IWDG              = 41,
    IRQ_I2C               = 42,

    IRQ_SPI               = 44,

    IRQ_CAN               = 46,
    IRQ_CRC               = 47,
    IRQ_APT0_EVT          = 48,
    IRQ_APT0_TMR          = 49,
    IRQ_APT1_EVT          = 50,
    IRQ_APT1_TMR          = 51,
    IRQ_APT2_EVT          = 52,
    IRQ_APT2_TMR          = 53,
    IRQ_APT3_EVT          = 54,
    IRQ_APT3_TMR          = 55,
    IRQ_APT4_EVT          = 56,
    IRQ_APT4_TMR          = 57,
    IRQ_APT5_EVT          = 58,
    IRQ_APT5_TMR          = 59,
    IRQ_APT6_EVT          = 60,
    IRQ_APT6_TMR          = 61,
    IRQ_APT7_EVT          = 62,
    IRQ_APT7_TMR          = 63,
    IRQ_APT8_EVT          = 64,
    IRQ_APT8_TMR          = 65,

    IRQ_CMM               = 68,
    IRQ_CFD               = 69,
    IRQ_CAPM0             = 70,
    IRQ_CAPM1             = 71,
    IRQ_CAPM2             = 72,
    IRQ_QDM0              = 73,

    IRQ_DMA_TC            = 77,
    IRQ_DMA_ERR           = 78,
    IRQ_SYSRAM_PARITY_ERR = 79,

    IRQ_EFC               = 81,
    IRQ_EFC_ERR           = 82,

    IRQ_PMU_CLDO_OCP      = 84,
    IRQ_PVD               = 85,

    IRQ_ADC0_OVINT        = 92,
    IRQ_ADC0_INT1         = 93,
    IRQ_ADC0_INT2         = 94,
    IRQ_ADC0_INT3         = 95,
    IRQ_ADC0_INT4         = 96,
    IRQ_ADC1_OVINT        = 97,
    IRQ_ADC1_INT1         = 98,
    IRQ_ADC1_INT2         = 99,
    IRQ_ADC1_INT3         = 100,
    IRQ_ADC1_INT4         = 101,
    IRQ_ADC2_OVINT        = 102,
    IRQ_ADC2_INT1         = 103,
    IRQ_ADC2_INT2         = 104,
    IRQ_ADC2_INT3         = 105,
    IRQ_ADC2_INT4         = 106,

    IRQ_GPIO0             = 109,
    IRQ_GPIO1             = 110,
    IRQ_GPIO2             = 111,
    IRQ_GPIO3             = 112,
    IRQ_GPIO4             = 113,
    IRQ_GPIO5             = 114,
    IRQ_GPIO6             = 115,
    IRQ_GPIO7             = 116,

    IRQ_MAX,                 /**< The maximum number of interrupts currently supported */
} IRQ_ID;

#endif   /* MCUMagicTag_INTERRUPT_IP_H */