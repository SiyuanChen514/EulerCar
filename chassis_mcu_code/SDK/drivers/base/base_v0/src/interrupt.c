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
  * @file    interrupt.c
  * @author  MCU Driver Team
  * @brief   Provides the handle template functions for processing exceptions and interrupts supported by the current
  *          functionalities of the interrupt.
  *           + Initialization and de-initialization functions
  *           + Regester and de-regester interrupt
  *           + Enable and disable interrupt
  *           + Configure interrupt
  */

/* Includes ------------------------------------------------------------------ */
#include "interrupt.h"
#include "baseinc.h"

/* Macro definitions ---------------------------------------------------------*/

/* Typedef definitions -------------------------------------------------------*/
void IRQ_PriorityInit(void);
static void IRQ_DummyHandler(void *arg);
static void IRQ_SetCallBack(unsigned int irqNum, IRQ_PROC_FUNC func, void *arg);

#if defined(USER_MODE_ENABLE) && (USER_MODE_ENABLE == 1)
static inline unsigned int IRQ_GetCpuPrivilege(void);

static struct IRQ_Mask {
    unsigned int irqMie;
    unsigned int irqLocien0;
    unsigned int irqLocien1;
    unsigned int irqLocien2;
    unsigned int irqLocien3;
} g_irqMask;

volatile unsigned int g_RiscvPrivMode = 0;
#endif

/**
  * @brief Interrupt vector table, supports up to IRQ_MAX interrupts, except for IRQ_VECTOR_CNT internal
  *        standard interrupts, which can be configured according to actual conditions.
  */
IRQ_ARG_FUNC g_irqCallbackFunc[IRQ_MAX];

/* Initialization and de-initialization functions ----------------------------*/
/**
  * @brief Exception/Interrupt Handler Entry.
  * @param irqNum external interrupt number.
  * @retval None
  */
void InterruptEntry(unsigned int irqNum)
{
    g_irqCallbackFunc[irqNum].pfnHandler(g_irqCallbackFunc[irqNum].param);
    IRQ_ClearN(irqNum);
}

/**
  * @brief Irq initialization.
  * @param none.
  * @retval None
  */
void IRQ_Init(void)
{
    unsigned int index;

    for (index = 0; index < IRQ_MAX; index++) {
        g_irqCallbackFunc[index].pfnHandler = IRQ_DummyHandler;
        g_irqCallbackFunc[index].param = NULL;
    }
}

/* Register and Unregister interrupt -----------------------------------------*/
/**
  * @brief Register IRQ Callback function and parameter.
  * @param irqNum  External interrupt number.
  * @param func    Callback function.
  * @param arg     Parameter of callback function.
  * @retval BASE_STATUS_OK(success) or IRQ_ERRNO_ALREADY_CREATED(fail) or IRQ_ERRNO_NUM_INVALID.
  * @note    In the corresponding interrupt handler, manually clear the interrupt source and the corresponding interrupt
  *          flag bit (call the IRQ_ClearN function to clear the interrupt), otherwise the interrupt will always be
  *          triggered.
  */
unsigned int IRQ_Register(unsigned int irqNum, IRQ_PROC_FUNC func, void *arg)
{
    INTERRUPT_ASSERT_PARAM(func != NULL);
    INTERRUPT_PARAM_CHECK_WITH_RET(irqNum < IRQ_MAX, IRQ_ERRNO_NUM_INVALID);

    if (g_irqCallbackFunc[irqNum].pfnHandler != IRQ_DummyHandler) {
        return IRQ_ERRNO_ALREADY_CREATED;
    }
    IRQ_SetCallBack(irqNum, func, arg);
    return BASE_STATUS_OK;
}

/**
  * @brief Unregister IRQ Callback.
  * @param irqNum   External interrupt number.
  * @retval BASE_STATUS_OK or IRQ_ERRNO_NUM_INVALID.
  */
unsigned int IRQ_Unregister(unsigned int irqNum)
{
    INTERRUPT_PARAM_CHECK_WITH_RET(irqNum < IRQ_MAX, IRQ_ERRNO_NUM_INVALID);
    g_irqCallbackFunc[irqNum].pfnHandler = IRQ_DummyHandler;
    g_irqCallbackFunc[irqNum].param = NULL;
    return BASE_STATUS_OK;
}

/* Enable and disable interrupt ----------------------------------------------*/
/**
  * @brief  Global Interrupt Enable.
  * @retval None.
  */
void IRQ_Enable(void)
{
#if defined(USER_MODE_ENABLE) && (USER_MODE_ENABLE == 1)
    unsigned int priv = IRQ_GetCpuPrivilege();

    RISCV_PRIV_MODE_SWITCH(priv);

    g_irqMask.irqMie  |= READ_CSR(mie);
    g_irqMask.irqLocien0 |= READ_CUSTOM_CSR(LOCIEN0);
    g_irqMask.irqLocien1 |= READ_CUSTOM_CSR(LOCIEN1);
    g_irqMask.irqLocien2 |= READ_CUSTOM_CSR(LOCIEN2);
    g_irqMask.irqLocien3 |= READ_CUSTOM_CSR(LOCIEN3);

    WRITE_CSR(mie, g_irqMask.irqMie);
    WRITE_CUSTOM_CSR_VAL(LOCIEN0, g_irqMask.irqLocien0);
    WRITE_CUSTOM_CSR_VAL(LOCIEN1, g_irqMask.irqLocien1);
    WRITE_CUSTOM_CSR_VAL(LOCIEN2, g_irqMask.irqLocien2);
    WRITE_CUSTOM_CSR_VAL(LOCIEN3, g_irqMask.irqLocien3);

    RISCV_PRIV_MODE_SWITCH(priv);
#else
    SET_CSR(mstatus, MSTATUS_MIE);
#endif
}

/**
  * @brief Global Interrupt Disable.
  * @retval BASE_STATUS_OK.
  * @note   Must be called in Interrupt(Machine mode)
  */
void IRQ_Disable(void)
{
#if defined(USER_MODE_ENABLE) && (USER_MODE_ENABLE == 1)
    unsigned int priv = IRQ_GetCpuPrivilege();

    RISCV_PRIV_MODE_SWITCH(priv);

    g_irqMask.irqMie = READ_CSR(mie);
    g_irqMask.irqLocien0 = READ_CUSTOM_CSR(LOCIEN0);
    g_irqMask.irqLocien1 = READ_CUSTOM_CSR(LOCIEN1);
    g_irqMask.irqLocien2 = READ_CUSTOM_CSR(LOCIEN2);
    g_irqMask.irqLocien3 = READ_CUSTOM_CSR(LOCIEN3);

    WRITE_CSR(mie, 0);
    WRITE_CUSTOM_CSR_VAL(LOCIEN0, 0);
    WRITE_CUSTOM_CSR_VAL(LOCIEN1, 0);
    WRITE_CUSTOM_CSR_VAL(LOCIEN2, 0);
    WRITE_CUSTOM_CSR_VAL(LOCIEN3, 0);

    RISCV_PRIV_MODE_SWITCH(priv);
#else
    CLEAR_CSR(mstatus, MSTATUS_MIE | MSTATUS_MPIE);
#endif
}

/**
  * @brief Enable the specified interrupt.
  * @param irqNum  External interrupt number.
  * @retval BASE_STATUS_OK or IRQ_ERRNO_NUM_INVALID.
  */
unsigned int IRQ_EnableN(unsigned int irqNum)
{
    unsigned int irqOrder;
    unsigned int locienVal;
#if defined(USER_MODE_ENABLE) && (USER_MODE_ENABLE == 1)
    unsigned int priv = IRQ_GetCpuPrivilege();
#endif

    INTERRUPT_PARAM_CHECK_WITH_RET((irqNum >= IRQ_VECTOR_CNT && irqNum < IRQ_MAX), IRQ_ERRNO_NUM_INVALID);

    /* The interrupt enable bits that can be controlled in the mie register (32 bits), up to 32
       can be controlled, and each bit corresponds to an interrupt enable */

    RISCV_PRIV_MODE_SWITCH(priv);

    if (irqNum < IRQ_MIE_TOTAL_CNT) {
        irqOrder = 1U << irqNum;
        SET_CSR(mie, irqOrder);
    } else if (irqNum < IRQ_LOCIEN1_OFFSET) {
        irqOrder = irqNum - IRQ_MIE_TOTAL_CNT;
        locienVal = READ_CUSTOM_CSR(LOCIEN0);
        locienVal |= (1U << irqOrder);
        WRITE_CUSTOM_CSR_VAL(LOCIEN0, locienVal);
    } else if (irqNum < IRQ_LOCIEN2_OFFSET) {
        irqOrder = irqNum - IRQ_LOCIEN1_OFFSET;
        locienVal = READ_CUSTOM_CSR(LOCIEN1);
        locienVal |= (1U << irqOrder);
        WRITE_CUSTOM_CSR_VAL(LOCIEN1, locienVal);
    } else {
        irqOrder = irqNum - IRQ_LOCIEN2_OFFSET;
        locienVal = READ_CUSTOM_CSR(LOCIEN2);
        locienVal |= (1U << irqOrder);
        WRITE_CUSTOM_CSR_VAL(LOCIEN2, locienVal);
    }

    RISCV_PRIV_MODE_SWITCH(priv);

    return BASE_STATUS_OK;
}

/**
  * @brief Disable the specified interrupt.
  * @param irqNum  External interrupt number.
  * @retval BASE_STATUS_OK or IRQ_ERRNO_NUM_INVALID or IRQ_ERRNO_NOT_CREATED.
  */
unsigned int IRQ_DisableN(unsigned int irqNum)
{
    unsigned int irqOrder;
#if defined(USER_MODE_ENABLE) && (USER_MODE_ENABLE == 1)
    unsigned int priv = IRQ_GetCpuPrivilege();
#endif

    INTERRUPT_PARAM_CHECK_WITH_RET((irqNum >= IRQ_VECTOR_CNT && irqNum < IRQ_MAX), IRQ_ERRNO_NUM_INVALID);
    INTERRUPT_PARAM_CHECK_WITH_RET((g_irqCallbackFunc[irqNum].pfnHandler != IRQ_DummyHandler), IRQ_ERRNO_NOT_CREATED);

    RISCV_PRIV_MODE_SWITCH(priv);

    if (irqNum < IRQ_MIE_TOTAL_CNT) {
        irqOrder = 1U << irqNum;
        CLEAR_CSR(mie, irqOrder);
    } else if (irqNum < IRQ_LOCIEN1_OFFSET) {
        irqOrder = 1U << (irqNum - IRQ_MIE_TOTAL_CNT);
        CLEAR_CUSTOM_CSR(LOCIEN0, irqOrder);
    } else if (irqNum < IRQ_LOCIEN2_OFFSET) {
        irqOrder = 1U << (irqNum - IRQ_LOCIEN1_OFFSET);
        CLEAR_CUSTOM_CSR(LOCIEN1, irqOrder);
    } else {
        irqOrder = 1U << (irqNum - IRQ_LOCIEN2_OFFSET);
        CLEAR_CUSTOM_CSR(LOCIEN2, irqOrder);
    }

    RISCV_PRIV_MODE_SWITCH(priv);

    return BASE_STATUS_OK;
}

/**
  * @brief Print RISCV register.
  * @param context.
  * @note  The actual code is generated by IDE
  * @retval None.
  */
__weak void SysErrPrint(const SyserrContext *context)
{
    BASE_FUNC_UNUSED(context);
}

/**
  * @brief System error completion processing
  * @param None.
  * @retval None.
  */
static void SysErrFinish(void)
{
}

/**
  * @brief Exception Handler Entry.
  * @param context error context.
  * @retval None.
  */
void SysErrExcEntry(const SyserrContext *context)
{
    SysErrPrint(context);
    SysErrFinish();
}

/**
  * @brief NMI Interrupt Handler Entry.
  * @param context error context.
  * @retval None.
  */
void SysErrNmiEntry(const SyserrContext *context)
{
    INTERRUPT_ASSERT_PARAM(context != NULL);
    SysErrPrint(context);
    SysErrFinish();
}
/**
  * @brief Set the priority of local interrupt.
  * @param intNum GROUP NUM.
  * @param interPriNum Local interrupt number, which equals external interrupt number - IRQ_VECTOR_CN.
  * @param prior local int prioroty.
  * @retval None
  */
static void SetLocalIntNumPri(unsigned int intNum, unsigned int interPriNum, unsigned int prior)
{
    switch (intNum) {
        case 8:  /* GROUP8 */
            SET_LOCAL_INTER_NUM_PRI(8, interPriNum, prior);
            break;
        case 9:  /* GROUP9 */
            SET_LOCAL_INTER_NUM_PRI(9, interPriNum, prior);
            break;
        case 10:  /* GROUP10 */
            SET_LOCAL_INTER_NUM_PRI(10, interPriNum, prior);
            break;
        case 11:  /* GROUP11 */
            SET_LOCAL_INTER_NUM_PRI(11, interPriNum, prior);
            break;
        case 12:  /* GROUP12 */
            SET_LOCAL_INTER_NUM_PRI(12, interPriNum, prior);
            break;
        case 13:  /* GROUP13 */
            SET_LOCAL_INTER_NUM_PRI(13, interPriNum, prior);
            break;
        case 14:  /* GROUP14 */
            SET_LOCAL_INTER_NUM_PRI(14, interPriNum, prior);
            break;
        case 15:  /* GROUP15 */
            SET_LOCAL_INTER_NUM_PRI(15, interPriNum, prior);
            break;
        default:
            break;
    }
}
/**
  * @brief Set the priority of local interrupt.
  * @param interPriNum Local interrupt number, which equals external interrupt number - IRQ_VECTOR_CN.
  * @param prior       Priority of this local interrupt to be set.
  * @retval None.
  */
static void IRQ_SetLocalPriority(unsigned int interPriNum, unsigned int prior)
{
#if defined(USER_MODE_ENABLE) && (USER_MODE_ENABLE == 1)
    unsigned int priv = IRQ_GetCpuPrivilege();
#endif
    RISCV_PRIV_MODE_SWITCH(priv);
    unsigned int intNum = GET_LOCAL_INTER_CONFIGREG_NUM(interPriNum);
    switch (intNum) {
        case 0:  /* GROUP0 */
            SET_LOCAL_INTER_NUM_PRI(0, interPriNum, prior);
            break;
        case 1:  /* GROUP1 */
            SET_LOCAL_INTER_NUM_PRI(1, interPriNum, prior);
            break;
        case 2:  /* GROUP2 */
            SET_LOCAL_INTER_NUM_PRI(2, interPriNum, prior);
            break;
        case 3:  /* GROUP3 */
            SET_LOCAL_INTER_NUM_PRI(3, interPriNum, prior);
            break;
        case 4:  /* GROUP4 */
            SET_LOCAL_INTER_NUM_PRI(4, interPriNum, prior);
            break;
        case 5:  /* GROUP5 */
            SET_LOCAL_INTER_NUM_PRI(5, interPriNum, prior);
            break;
        case 6:  /* GROUP6 */
            SET_LOCAL_INTER_NUM_PRI(6, interPriNum, prior);
            break;
        case 7:  /* GROUP7 */
            SET_LOCAL_INTER_NUM_PRI(7, interPriNum, prior);
            break;
        default:
            SetLocalIntNumPri(intNum, interPriNum, prior);
            break;
    }
    RISCV_PRIV_MODE_SWITCH(priv);
}

/**
  * @brief Set the priority of external interrupt.
  * @param irqNum  External interrupt number.
  * @param priority.
  * @retval IRQ_ERRNO_NUM_INVALID or IRQ_ERRNO_PRIORITY_INVALID or BASE_STATUS_OK.
  */
unsigned int IRQ_SetPriority(unsigned int irqNum, unsigned int priority)
{
    INTERRUPT_PARAM_CHECK_WITH_RET((irqNum >= IRQ_VECTOR_CNT && irqNum < IRQ_MAX), IRQ_ERRNO_NUM_INVALID);
    INTERRUPT_PARAM_CHECK_WITH_RET((priority >= IRQ_PRIO_LOWEST && priority <= IRQ_PRIO_HIGHEST), \
                                   IRQ_ERRNO_PRIORITY_INVALID);

    /* The locipri register is specifically used to configure the priority of the
       external non-standard interrupts of the CPU, so the number of internal
       standard interrupts should be subtracted */
    IRQ_SetLocalPriority(irqNum - IRQ_VECTOR_CNT, priority);

    return BASE_STATUS_OK;
}
/**
  * @brief Get the priority of local interrupt.
  * @param intNum GROUP NUM.
  * @param interPriNum Local interrupt number, which equals external interrupt number - IRQ_VECTOR_CN.
  * @param prior local int prioroty.
  * @retval None
  */
static void GetLocaIntNumPri(unsigned int intNum, unsigned int interPriNum, unsigned int prior)
{
    switch (intNum) {
        case 8:  /* GROUP8 */
            GET_LOCAL_INTER_NUM_PRI(8, interPriNum, prior);
            break;
        case 9:  /* GROUP9 */
            GET_LOCAL_INTER_NUM_PRI(9, interPriNum, prior);
            break;
        case 10:  /* GROUP10 */
            GET_LOCAL_INTER_NUM_PRI(10, interPriNum, prior);
            break;
        case 11:  /* GROUP11 */
            GET_LOCAL_INTER_NUM_PRI(11, interPriNum, prior);
            break;
        case 12:  /* GROUP12 */
            GET_LOCAL_INTER_NUM_PRI(12, interPriNum, prior);
            break;
        case 13:  /* GROUP13 */
            GET_LOCAL_INTER_NUM_PRI(13, interPriNum, prior);
            break;
        case 14:  /* GROUP14 */
            GET_LOCAL_INTER_NUM_PRI(14, interPriNum, prior);
            break;
        case 15:  /* GROUP15 */
            GET_LOCAL_INTER_NUM_PRI(15, interPriNum, prior);
            break;
        default:
            break;
    }
}

/**
  * @brief Get the priority of local interrupt.
  * @param interPriNum Local interrupt number, which equals external interrupt number - IRQ_VECTOR_CN.
  * @retval prior      Priority of this local interrupt to be set.
  */
static unsigned int IRQ_GetLocalPriority(unsigned int interPriNum)
{
#if defined(USER_MODE_ENABLE) && (USER_MODE_ENABLE == 1)
    unsigned int priv = IRQ_GetCpuPrivilege();
#endif
    unsigned int prior = 0;
    RISCV_PRIV_MODE_SWITCH(priv);
    unsigned int intNum = GET_LOCAL_INTER_CONFIGREG_NUM(interPriNum);
    switch (intNum) {
        case 0:  /* GROUP0 */
            GET_LOCAL_INTER_NUM_PRI(0, interPriNum, prior);
            break;
        case 1:  /* GROUP1 */
            GET_LOCAL_INTER_NUM_PRI(1, interPriNum, prior);
            break;
        case 2:  /* GROUP2 */
            GET_LOCAL_INTER_NUM_PRI(2, interPriNum, prior);
            break;
        case 3:  /* GROUP3 */
            GET_LOCAL_INTER_NUM_PRI(3, interPriNum, prior);
            break;
        case 4:  /* GROUP4 */
            GET_LOCAL_INTER_NUM_PRI(4, interPriNum, prior);
            break;
        case 5:  /* GROUP5 */
            GET_LOCAL_INTER_NUM_PRI(5, interPriNum, prior);
            break;
        case 6:  /* GROUP6 */
            GET_LOCAL_INTER_NUM_PRI(6, interPriNum, prior);
            break;
        case 7:  /* GROUP7 */
            GET_LOCAL_INTER_NUM_PRI(7, interPriNum, prior);
            break;
        default:
            GetLocaIntNumPri(intNum, interPriNum, prior);
            break;
    }
    RISCV_PRIV_MODE_SWITCH(priv);
    return prior;
}
/**
  * @brief Get the priority of external interrupt.
  * @param irqNum  External interrupt number.
  * @output priority.
  * @retval IRQ_ERRNO_NUM_INVALID or IRQ_ERRNO_PRIORITY_INVALID or BASE_STATUS_OK.
  */
unsigned int IRQ_GetPriority(unsigned int irqNum, unsigned int *priority)
{
    INTERRUPT_PARAM_CHECK_WITH_RET(irqNum < IRQ_MAX, IRQ_ERRNO_NUM_INVALID);

    /* The locipri register is specifically used to configure the priority of the
       external non-standard interrupts of the CPU, so the number of internal
       standard interrupts should be subtracted */
    *priority = IRQ_GetLocalPriority(irqNum - IRQ_VECTOR_CNT);

    return BASE_STATUS_OK;
}

/**
  * @brief  Clear all external interrupts
  * @retval BASE_STATUS_OK or IRQ_ERRNO_NOT_CREATED
  */
unsigned int IRQ_ClearAll(void)
{
    unsigned int index;
    for (index = IRQ_VECTOR_CNT; index < IRQ_MAX; index++) {
        IRQ_ClearN(index);
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Interrupt dummy handler
  * @param arg     Not used
  * @retval None.
  */
static void IRQ_DummyHandler(void *arg)
{
    BASE_FUNC_UNUSED(arg);
}

/**
  * @brief Construct a new irq setcallback object
  * @param irqNum external interrupt number
  * @param func   callback function
  * @param arg    callback arg
  * @retval None.
  */
static inline void IRQ_SetCallBack(unsigned int irqNum, IRQ_PROC_FUNC func, void *arg)
{
    g_irqCallbackFunc[irqNum].param = arg;
    g_irqCallbackFunc[irqNum].pfnHandler = func;
}

/**
  * @brief Get CPU Privilege by ecall
  * @param none
  * @retval mcause value
  */
#if defined(USER_MODE_ENABLE) && (USER_MODE_ENABLE == 1)
static inline unsigned int IRQ_GetCpuPrivilege(void)
{
    return (g_RiscvPrivMode == 0) ? RISCV_U_MODE : RISCV_M_MODE;
}
#endif
