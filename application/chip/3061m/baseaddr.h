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
  * @file      baseaddr.h
  * @author    MCU Driver Team
  * @brief     Definition of MCU register baseaddress
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_BASEADDR_H
#define McuMagicTag_BASEADDR_H

#define CRG_BASE          (void *)0x10000000
#define CMM_BASE          (void *)0x10010000
#define CFD_BASE          (void *)0x10010000
#define SYSCTRL0_BASE     (void *)0x10100000
#define SYSCTRL1_BASE     (void *)0x10100000
#define UART0_BASE        (void *)0x14000000
#define UART1_BASE        (void *)0x14001000
#define UART2_BASE        (void *)0x14002000
#define UART3_BASE        (void *)0x14003000

#define I2C0_BASE         (void *)0x14100000
#define I2C1_BASE         (void *)0x14101000
#define SPI0_BASE         (void *)0x14200000
#define SPI1_BASE         (void *)0x14201000
#define TIMER0_BASE       (void *)0x14300000
#define TIMER1_BASE       (void *)0x14301000
#define TIMER2_BASE       (void *)0x14302000
#define TIMER3_BASE       (void *)0x14303000

#define SYSTICK_BASE      (void *)0x14380000
#define WWDG_BASE         (void *)0x14400000
#define IWDG_BASE         (void *)0x14401000
#define GPIO0_BASE        (void *)0x14500000
#define GPIO1_BASE        (void *)0x14501000
#define GPIO2_BASE        (void *)0x14502000
#define GPIO3_BASE        (void *)0x14503000
#define GPIO4_BASE        (void *)0x14504000
#define GPIO5_BASE        (void *)0x14505000
#define CAN_BASE          (void *)0x14600000
#define GPT0_BASE         (void *)0x14700000
#define GPT1_BASE         (void *)0x14701000
#define GPT2_BASE         (void *)0x14702000
#define GPT3_BASE         (void *)0x14703000

#define EFC_BASE          (void *)0x14710000
#define FOTP_BASE         (void *)0x14720000
#define HPM_BASE          (void *)0x147D0000
#define PMC_BASE          (void *)0x147E0000
#define IOCMG_BASE        (void *)0x147F0000
#define CRC_BASE          (void *)0x14800000
#define APT0_BASE         (void *)0x14A00000
#define APT1_BASE         (void *)0x14A01000
#define APT2_BASE         (void *)0x14A02000
#define APT3_BASE         (void *)0x14A03000

#define CAPM0_BASE        (void *)0x14B00000
#define CAPM1_BASE        (void *)0x14B01000
#define CAPM2_BASE        (void *)0x14B02000
#define CAPM_COMM_BASE    (void *)0x14B03000

#define QDM0_BASE         (void *)0x14C00000
#define QDM1_BASE         (void *)0x14C01000
#define ADC0_BASE         (void *)0x18000000
#define VREF_BASE         (void *)0x18100000
#define PGA0_BASE         (void *)0x18200000
#define PGA1_BASE         (void *)0x18201000
#define ACMP0_BASE        (void *)0x18300000
#define DAC0_BASE         (void *)0x18400000

#define TSENSOR_BASE      (void *)0x18500000
#define ANA_CTRL_TOP_BASE (void *)0x18600000

#define DMA_BASE          (void *)0x1C000000
#define DMA_CHANNEL0_BASE (void *)0x1C000100
#define DMA_CHANNEL1_BASE (void *)0x1C000120
#define DMA_CHANNEL2_BASE (void *)0x1C000140
#define DMA_CHANNEL3_BASE (void *)0x1C000160
#define DMA_CHANNEL4_BASE (void *)0x1C000180
#define DMA_CHANNEL5_BASE (void *)0x1C0001A0

#define CRG ((CRG_RegStruct *)CRG_BASE)
#define CMM ((CMM_RegStruct *)CMM_BASE)
#define CFD ((CFD_RegStruct *)CFD_BASE)
#define SYSCTRL0 ((SYSCTRL0_RegStruct *)SYSCTRL0_BASE)
#define SYSCTRL1 ((SYSCTRL1_RegStruct *)SYSCTRL1_BASE)
#define UART0 ((UART_RegStruct *)UART0_BASE)
#define UART1 ((UART_RegStruct *)UART1_BASE)
#define UART2 ((UART_RegStruct *)UART2_BASE)
#define UART3 ((UART_RegStruct *)UART3_BASE)
#define I2C0 ((I2C_RegStruct *)I2C0_BASE)
#define I2C1 ((I2C_RegStruct *)I2C1_BASE)
#define SPI0 ((SPI_RegStruct *)SPI0_BASE)
#define SPI1 ((SPI_RegStruct *)SPI1_BASE)
#define TIMER0 ((TIMER_RegStruct *)TIMER0_BASE)
#define TIMER1 ((TIMER_RegStruct *)TIMER1_BASE)
#define TIMER2 ((TIMER_RegStruct *)TIMER2_BASE)
#define TIMER3 ((TIMER_RegStruct *)TIMER3_BASE)
#define SYSTICK ((SYSTICK_RegStruct *)SYSTICK_BASE)
#define WWDG ((WWDG_RegStruct *)WWDG_BASE)
#define IWDG ((IWDG_RegStruct *)IWDG_BASE)
#define GPIO0 ((GPIO_RegStruct *)GPIO0_BASE)
#define GPIO1 ((GPIO_RegStruct *)GPIO1_BASE)
#define GPIO2 ((GPIO_RegStruct *)GPIO2_BASE)
#define GPIO3 ((GPIO_RegStruct *)GPIO3_BASE)
#define GPIO4 ((GPIO_RegStruct *)GPIO4_BASE)
#define GPIO5 ((GPIO_RegStruct *)GPIO5_BASE)

#define CAN ((CAN_RegStruct *)CAN_BASE)
#define GPT0 ((GPT_RegStruct *)GPT0_BASE)
#define GPT1 ((GPT_RegStruct *)GPT1_BASE)
#define GPT2 ((GPT_RegStruct *)GPT2_BASE)
#define GPT3 ((GPT_RegStruct *)GPT3_BASE)
#define EFC ((EFC_RegStruct *)EFC_BASE)
#define FOTP ((FOTP_RegStruct *)FOTP_BASE)
#define HPM ((HPM_RegStruct *)HPM_BASE)
#define PMC ((PMC_RegStruct *)PMC_BASE)
#define CRC ((CRC_RegStruct *)CRC_BASE)
#define APT0 ((APT_RegStruct *)APT0_BASE)
#define APT1 ((APT_RegStruct *)APT1_BASE)
#define APT2 ((APT_RegStruct *)APT2_BASE)
#define APT3 ((APT_RegStruct *)APT3_BASE)
#define CAPM0 ((CAPM_RegStruct *)CAPM0_BASE)
#define CAPM1 ((CAPM_RegStruct *)CAPM1_BASE)
#define CAPM2 ((CAPM_RegStruct *)CAPM2_BASE)
#define CAPM_COMM ((CAPM_COMM_RegStruct *)CAPM_COMM_BASE)
#define QDM0 ((QDM_RegStruct *)QDM0_BASE)
#define QDM1 ((QDM_RegStruct *)QDM1_BASE)
#define ADC0 ((ADC_RegStruct *)ADC0_BASE)
#define PGA0 ((PGA_RegStruct *)PGA0_BASE)
#define PGA1 ((PGA_RegStruct *)PGA1_BASE)
#define DAC0 ((DAC_RegStruct *)DAC0_BASE)
#define TSENSOR ((TSENSOR_RegStruct *)TSENSOR_BASE)
#define ACMP0 ((ACMP_RegStruct *)ACMP0_BASE)
#define DMA ((DMA_RegStruct *)DMA_BASE)
#define DMA_CHANNEL0 ((DMA_ChannelRegStruct *)DMA_CHANNEL0_BASE)
#define DMA_CHANNEL1 ((DMA_ChannelRegStruct *)DMA_CHANNEL1_BASE)
#define DMA_CHANNEL2 ((DMA_ChannelRegStruct *)DMA_CHANNEL2_BASE)
#define DMA_CHANNEL3 ((DMA_ChannelRegStruct *)DMA_CHANNEL3_BASE)
#define DMA_CHANNEL4 ((DMA_ChannelRegStruct *)DMA_CHANNEL4_BASE)
#define DMA_CHANNEL5 ((DMA_ChannelRegStruct *)DMA_CHANNEL5_BASE)
#define IOCMG ((IOConfig_RegStruct*)IOCMG_BASE)
#define VREF ((VREF_RegStruct *)VREF_BASE)

#define IsCRGInstance(instance) ((instance) == CRG)
#define IsCMMInstance(instance) ((instance) == CMM)
#define IsCFDInstance(instance) ((instance) == CFD)
#define IsSYSCTRLInstance(instance) (((instance) == SYSCTRL0) || ((instance) == SYSCTRL1))
#define IsUARTInstance(instance) (((instance) == UART0) || ((instance) == UART1) || \
                                  ((instance) == UART2) || ((instance) == UART3))
#define IsI2CInstance(instance) (((instance) == I2C0) || ((instance) == I2C1))
#define IsSPIInstance(instance) (((instance) == SPI0) || ((instance) == SPI1))
#define IsTIMERInstance(instance) (((instance) == TIMER0) || ((instance) == TIMER1) || \
                                   ((instance) == TIMER2) || ((instance) == TIMER3))
#define IsSYSTICKInstance(instance)  ((instance) == SYSTICK)
#define IsWWDGInstance(instance) ((instance) == WWDG)
#define IsIWDGInstance(instance) ((instance) == IWDG)
#define IsGPIOInstance(instance) (((instance) == GPIO0) || ((instance) == GPIO1) || \
                                  ((instance) == GPIO2) || ((instance) == GPIO3) || \
                                  ((instance) == GPIO4) || ((instance) == GPIO5))
#define IsCANInstance(instance) ((instance) == CAN)
#define IsGPTInstance(instance) (((instance) == GPT0) || ((instance) == GPT1) || \
                                 ((instance) == GPT2) || ((instance) == GPT3))
#define IsEFCInstance(instance) ((instance) == EFC)
#define IsFOTPInstance(instance) ((instance) == FOTP)
#define IsHPMInstance(instance) ((instance) == HPM)
#define IsPMCInstance(instance) ((instance) == PMC)
#define IsIOCMGInstance(instance) ((instance) == IOCMG)
#define IsCRCInstance(instance) ((instance) == CRC)
#define IsAPTInstance(instance) (((instance) == APT0) || ((instance) == APT1) || \
                                 ((instance) == APT2) || ((instance) == APT3))
#define IsCAPMInstance(instance) (((instance) == CAPM0) || ((instance) == CAPM1) || ((instance) == CAPM2))
#define IsCAPMCOMMInstance(instance) ((instance) == CAPM_COMM)
#define IsQDMInstance(instance) (((instance) == QDM0) || ((instance) == QDM1))
#define IsADCInstance(instance) ((instance) == ADC0)
#define IsPGAInstance(instance) (((instance) == PGA0) || ((instance) == PGA1))
#define IsDACInstance(instance) ((instance) == DAC0)
#define IsACMPInstance(instance) ((instance) == ACMP0)
#define IsDMAInstance(instance) ((instance) == DMA)
#define IsDMACHXInstance(instance) (((instance) == DMA_CHANNEL0) || ((instance) == DMA_CHANNEL1) || \
                                    ((instance) == DMA_CHANNEL2) || ((instance) == DMA_CHANNEL3) || \
                                    ((instance) == DMA_CHANNEL4) || ((instance) == DMA_CHANNEL5))
#define SRAM_START 0x4000000
#define SRAM_END 0x4007FFF
#define REGISTER_START 0x10000000
#define REGISTER_END 0x1C000FFF
#endif /* McuMagicTag_BASEADDR_H */