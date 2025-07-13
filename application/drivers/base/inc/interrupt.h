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
  * @file    interrupt.h
  * @author  MCU Driver Team
  * @brief   BASE module driver
  * @brief   Header file containing functions prototypes of Interrupt HAL library.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_INTERRUPT_H
#define McuMagicTag_INTERRUPT_H

/* Includes ------------------------------------------------------------------*/
#include "feature.h"
#include "interrupt_ip.h"

/* Macro definitions ---------------------------------------------------------*/
#define INTERRUPT_USE_ASSERT
#ifdef INTERRUPT_USE_ASSERT
#define INTERRUPT_ASSERT_PARAM          BASE_FUNC_ASSERT_PARAM
#define INTERRUPT_PARAM_CHECK_NO_RET    BASE_FUNC_PARAMCHECK_NO_RET
#define INTERRUPT_PARAM_CHECK_WITH_RET  BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define INTERRUPT_ASSERT_PARAM(para)    ((void)0U)
#define INTERRUPT_PARAM_CHECK_NO_RET    ((void)0U)
#define INTERRUPT_PARAM_CHECK_WITH_RET  ((void)0U)
#endif
/**
  * @defgroup BASE BASE
  * @brief BASE module.
  * @{
  */

/**
  * @defgroup INTERRUPT Interrupt Definition
  * @brief Definition of Interrupt Definition.
  * @{
  */

/**
  * @defgroup INTERRUPT_MACRO Macro Definition
  * @brief Definition of Interrupt Definition.
  * @{
  */

/**
  * @brief IRQ module error code
  */
#define IRQ_ERRNO_PROC_FUNC_NULL     1   /**< Non-interrupted callback function */
#define IRQ_ERRNO_NUM_INVALID        2   /**< Interrupt Number invalid */
#define IRQ_ERRNO_ALREADY_CREATED    3   /**< Interrupt function is created */
#define IRQ_ERRNO_NOT_CREATED        4   /**< Interrupt function not create */
#define IRQ_ERRNO_PRIORITY_INVALID   5   /**< Invalid priority */

#define RISCV_U_MODE                 0x8  /**< The Value in mcause for umode */
#define RISCV_M_MODE                 0xB  /**< The Value in mcause for mmode */
/**
  * @}
  */

/**
  * @defgroup ASM Interrupt ASM Function Definition
  * @brief Definition of Interrupt ASM Function Definition.
  * @{
  */

/**
  * @brief Read standard csr registers
  */
#define READ_CSR(csrReg) ({                             \
    unsigned int tmp_;                                  \
    asm volatile ("csrr %0, " #csrReg : "=r"(tmp_));    \
    tmp_;                                               \
})


/**
  * @brief Write standard csr registers
  */
#define WRITE_CSR(csrReg, csrVal) do {                              \
    if (__builtin_constant_p(csrVal) && ((unsigned int)(csrVal) < 32)) {  \
        asm volatile ("csrw " #csrReg ", %0" :: "i"(csrVal));       \
    } else {                                                        \
        asm volatile ("csrw " #csrReg ", %0" :: "r"(csrVal));       \
    }                                                               \
} while (0)

/**
  * @brief Set standard csr registers
  */
#define SET_CSR(csrReg, csrBit) do {                                           \
    unsigned int tmp_;                                                         \
    if (__builtin_constant_p(csrBit) && ((unsigned int)(csrBit) < 32)) {       \
        asm volatile ("csrrs %0, " #csrReg ", %1" : "=r"(tmp_) : "i"(csrBit)); \
    } else {                                                                   \
        asm volatile ("csrrs %0, " #csrReg ", %1" : "=r"(tmp_): "r"(csrBit));  \
    }                                                                          \
    (void)tmp_;                                                                \
} while (0)

/**
  * @brief Clear standard csr registers
  */
#define CLEAR_CSR(csrReg, csrBit) do {                                         \
    unsigned int tmp_;                                                         \
    if (__builtin_constant_p(csrBit) && ((unsigned int)(csrBit) < 32)) {       \
        asm volatile ("csrrc %0, " #csrReg ", %1" : "=r"(tmp_) : "i"(csrBit)); \
    } else {                                                                   \
        asm volatile ("csrrc %0, " #csrReg ", %1" : "=r"(tmp_) : "r"(csrBit)); \
    }                                                                          \
    (void)tmp_;                                                                \
} while (0)

/**
  * @brief Read the custom defined registers of the chip
  */
#define READ_CUSTOM_CSR(csrReg) ({                                              \
    unsigned int tmp_;                                                          \
    asm volatile ("csrr %0, %1" : "=r"(tmp_) : "i"(csrReg));                    \
    tmp_;                                                                       \
})

/**
  * @brief Write the custom defined registers of the chip
  */
#define WRITE_CUSTOM_CSR_VAL(csrRegAddr, csrVal) do {                           \
    if (__builtin_constant_p(csrVal))  {                                        \
        asm volatile("li t0," "%0" : : "i"(csrVal));                            \
    } else {                                                                    \
        asm volatile("mv t0," "%0" : : "r"(csrVal));                            \
    }                                                                           \
    asm volatile("csrw %0, t0" :: "i"(csrRegAddr));                             \
} while (0)

/**
  * @brief Set the custom defined registers of the chip
  */
#define SET_CUSTOM_CSR(csrRegAddr, csrBit) do {                                 \
    if (__builtin_constant_p(csrBit) && ((unsigned int)(csrBit) < 32)) {        \
        asm volatile("li t0," "%0" : : "i"(csrBit));                            \
    } else {                                                                    \
        asm volatile("mv t0," "%0" : : "r"(csrBit));                            \
    }                                                                           \
    asm volatile("csrs %0, t0" :: "i"(csrRegAddr));                             \
} while (0)

/**
  * @brief Clear the custom defined registers of the chip
  */
#define CLEAR_CUSTOM_CSR(csrRegAddr, csrBit) do {                               \
    if (__builtin_constant_p(csrBit) && ((unsigned int)(csrBit) < 32)) {        \
        asm volatile("li t0," "%0" : : "i"(csrBit));                            \
    } else {                                                                    \
        asm volatile("mv t0," "%0" : : "r"(csrBit));                            \
    }                                                                           \
    asm volatile("csrc %0, t0" :: "i"(csrRegAddr));                             \
} while (0)

/* Configure the locipri register, that is, configure the interrupt priority */
/**
  * @brief Get the local interrupt register number.
  */
#define GET_LOCAL_INTER_CONFIGREG_NUM(interIndex)  ((unsigned int)(interIndex) >> 3)

/**
  * @brief Set local interrupt registers priority.
  */
#define SET_LOCAL_INTER_NUM_PRI(configNum, priNum, pri) do {                                \
    unsigned int interPriVal = READ_CUSTOM_CSR(LOCIPRI(configNum));                         \
    /* clear the irqNum-th local interrupt priority */                                      \
    interPriVal &= (~((0xfU << (((unsigned int)(priNum) & 0x7U) << 2)) & UINT32_CUT_MASK)); \
    /* set the irqNum-th local interrupt priority */                                        \
    interPriVal |= ((unsigned int)(pri) << (((unsigned int)(priNum) & 0x7U) << 2));       \
    WRITE_CUSTOM_CSR_VAL(LOCIPRI(configNum), interPriVal);                                  \
} while (0)

/**
  * @brief Get local interrupt registers priority.
  */
#define GET_LOCAL_INTER_NUM_PRI(configNum, priNum, pri) do {                        \
    (pri) = READ_CUSTOM_CSR(LOCIPRI(configNum));                                    \
    /* Get the irqNum-th local interrupt priority */                                \
    (pri) >>= (((unsigned int)(priNum) & 0x7U) << 2);                               \
    (pri) &= 0x7U;                                                                  \
} while (0)

#if defined(USER_MODE_ENABLE) && (USER_MODE_ENABLE == 1)
/**
  * @brief Riscv mode switch in user mode
  */
#define RISCV_PRIV_MODE_SWITCH(priv) do {                                           \
    if ((priv) == RISCV_U_MODE) {                                                   \
        asm volatile ("ecall");                                                     \
    }                                                                               \
} while (0)
#else
#define RISCV_PRIV_MODE_SWITCH(priv) (void)(0)
#endif
/**
  * @}
  */

/**
  * @brief  Clear external interrupt
  * @param irqNum external interrupt number
  * @retval BASE_STATUS_OK or IRQ_ERRNO_NUM_INVALID or IRQ_ERRNO_NOT_CREATED
  */
static inline void IRQ_ClearN(unsigned int irqNum)
{
    asm volatile("fence");
    WRITE_CUSTOM_CSR_VAL(LOCIPCLR, irqNum);
}
/**
  * @defgroup INTERRUPT_STRUCTURE_DEFINITION Interrupt Structure Definition
  * @brief Definition of interrupt STRUCTURE.
  * @{
  */
typedef void (* IRQ_PROC_FUNC)(void *arg);

/**
  * @brief Interrupt Handle Structure
  */
typedef struct {
    IRQ_PROC_FUNC  pfnHandler;
    void          *param;
} IRQ_ARG_FUNC;

/**
  * @brief System error context Structure
  */
typedef struct {
    unsigned int ra;
    unsigned int t0;
    unsigned int t1;
    unsigned int t2;
    unsigned int a0;
    unsigned int a1;
    unsigned int a2;
    unsigned int a3;
    unsigned int a4;
    unsigned int a5;
    unsigned int a6;
    unsigned int a7;
    unsigned int t3;
    unsigned int t4;
    unsigned int t5;
    unsigned int t6;
    unsigned int s0;
    unsigned int s1;
    unsigned int s2;
    unsigned int s3;
    unsigned int s4;
    unsigned int s5;
    unsigned int s6;
    unsigned int s7;
    unsigned int s8;
    unsigned int s9;
    unsigned int s10;
    unsigned int s11;
    unsigned int sp;
    unsigned int gp;
    unsigned int tp;
    unsigned int mepc;
    unsigned int mstatus;
    unsigned int mtval;
    unsigned int mcause;
    unsigned int ccause;
} SyserrContext;
/**
  * @}
  */

/**
  * @defgroup INTERRUPT_API_DEFINITION Interrupt API
  * @brief Definition of interrupt API.
  * @{
  */
unsigned int IRQ_SetPriority(unsigned int irqNum, unsigned int priority);
unsigned int IRQ_GetPriority(unsigned int irqNum, unsigned int *priority);
void IRQ_Enable(void);
void IRQ_Disable(void);
unsigned int IRQ_EnableN(unsigned int irqNum);
unsigned int IRQ_DisableN(unsigned int irqNum);
void IRQ_Init(void);
unsigned int IRQ_Register(unsigned int irqNum, IRQ_PROC_FUNC func, void *arg);
unsigned int IRQ_Unregister(unsigned int irqNum);
unsigned int IRQ_ClearAll(void);
void SysErrNmiEntry(const SyserrContext *context);
void SysErrExcEntry(const SyserrContext *context);
void InterruptEntry(unsigned int irqNum);
void SysErrPrint(const SyserrContext *context);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* McuMagicTag_INTERRUPT_H */
