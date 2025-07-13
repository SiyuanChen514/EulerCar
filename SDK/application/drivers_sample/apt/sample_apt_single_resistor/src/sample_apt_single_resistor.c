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
  * @file    sample_apt_single_resistor.c
  * @author  MCU Driver Team
  * @brief   APT module sample of HAL API.
  *          This file provides some configuration example of APT module HAL API.
  *          + PWM waveform configuration and ADC trigger time configuration sample.
  *          + Output control protection configuration sample.
  *          + Interrupt callback function and user registration function sample.
  *          + APT module synchronization sample.
  */

#include "sample_apt_single_resistor.h"
#include "apt.h"
#include "debug.h"
#include "crg.h"
#include "gpio.h"
#include "main.h"
#include "debug.h"

/**
  * APTx module base definition of three phases.
  */
#define APT_U               APT0 /* Base address of U phase APT module */
#define APT_V               APT1 /* Base address of V phase APT module */
#define APT_W               APT2 /* Base address of W phase APT module */

/**
  * APT module run control definition.
  */
#define APT_RUN_U           RUN_APT0
#define APT_RUN_V           RUN_APT1
#define APT_RUN_W           RUN_APT2

/**
  * APT interrupt number definition of three phases.
  * IGBT - APT2, APT3, APT4 (U, V, W phase)
  * IPM  - APT5, APT6, ATP7 (U, V, W phase)
  */
#define APT_U_EVT_IRQ       IRQ_APT0_EVT
#define APT_U_TMR_IRQ       IRQ_APT0_TMR
#define APT_V_EVT_IRQ       IRQ_APT1_EVT
#define APT_V_TMR_IRQ       IRQ_APT1_TMR
#define APT_W_EVT_IRQ       IRQ_APT2_EVT
#define APT_W_TMR_IRQ       IRQ_APT2_TMR

/* APT module interrupt priority. */
#define EVT_INTERRUPT_PRIORITY 7
#define TMR_INTERRUPT_PRIORITY 6

/* Use sync-out pulse from APT_U as the sync-in source for slave APT module. */
#define APT_SYNC_IN_SRC     APT_SYNCIN_SRC_APT0_SYNCOUT

/* Some configuration values of APT modules. */
/* You can also use HAL_CRG_GetIpFreq(APT_U) to get the CPU clock frequency (In units of Hz). */
#define APT_CLK_FREQ        HAL_CRG_GetIpFreq((void *)APT_U)
#define APT_PWM_FREQ        5000U       /* Set PWM frequency to 5000Hz. */
#define APT_TIMER_PERIOD    (APT_CLK_FREQ / (APT_PWM_FREQ * 2)) /* Period value when using APT_COUNT_MODE_UP_DOWN. */
#define APT_DIVIDER_FACTOR  1U          /* The APT clock is not divided. */
#define DB_CNT_PER_US       (APT_CLK_FREQ / 1000000) /* Dead-Band delay counter period when Dead-Band time is 1us */
#define DB_US               3U          /* Dead-Band time, in units of us */


/**
  * @brief Timer interrupt callback function of U phase APT module.
  * @param aptHandle APT module handle.
  * @retval None.
  */
void AptUTimerCallback(void *aptHandle)
{
    APT_Handle *handle = (APT_Handle *)aptHandle;
    /* read counter direction */
    if (DCL_APT_GetCounterDirection(handle->baseAddress) == APT_COUNTER_STATUS_COUNT_DOWN) {
        DBG_PRINTF("Count Down\r\n");
    } else if (DCL_APT_GetCounterDirection(handle->baseAddress) == APT_COUNTER_STATUS_COUNT_UP) {
        DBG_PRINTF("Count Up\r\n");
    }
}

/**
  * @brief Event interrupt callback function of U phase APT module.
  * @param aptHandle APT module handle.
  * @retval None.
  */
void AptUEventCallback(void *aptHandle)
{
    BASE_FUNC_UNUSED(aptHandle);
}

/**
  * @brief Interrupt initialization of U phase APT.
  * @retval None.
  */
static void InterruptInitAptU(void)
{
    /* Timer interrupt and event interrupt of U phase APT module. */
    IRQ_SetPriority(APT_U_EVT_IRQ, EVT_INTERRUPT_PRIORITY);
    IRQ_SetPriority(APT_U_TMR_IRQ, TMR_INTERRUPT_PRIORITY);
    IRQ_Register(IRQ_APT0_EVT, HAL_APT_EventIrqHandler, &g_hAptU);
    IRQ_Register(IRQ_APT0_TMR, HAL_APT_TimerIrqHandler, &g_hAptU);
    IRQ_EnableN(APT_U_EVT_IRQ);
    IRQ_EnableN(APT_U_TMR_IRQ);
    HAL_APT_RegisterCallBack(&g_hAptU, APT_EVENT_INTERRUPT, AptUEventCallback);
    HAL_APT_RegisterCallBack(&g_hAptU, APT_TIMER_INTERRUPT, AptUTimerCallback);
}

/**
  * @brief APT Synchronize initialize.
  * @retval None.
  */
static void APT_SyncMasterInit(APT_Handle *aptHandle)
{
    HAL_APT_MasterSyncInit(aptHandle, APT_SYNC_OUT_ON_CNTR_ZERO);
}


static void APT_SyncSlaveInit(APT_Handle *aptHandle)
{
    APT_SlaveSyncIn aptSlave;
    aptSlave.divPhase = 0; /* divide phase value */
    aptSlave.cntPhase = 0; /* counter phase value */
    aptSlave.syncCntMode = APT_COUNT_MODE_AFTER_SYNC_UP;
    aptSlave.syncInSrc = APT_SYNC_IN_SRC; /* sync source selection */
    aptSlave.cntrSyncSrc = APT_CNTR_SYNC_SRC_SYNCIN;
    HAL_APT_SlaveSyncInit(aptHandle, &aptSlave);
}

/**
  * @brief Initialize and start the APT modules of U, V, W phases.
  * @param mode: ADC sampling mode.
  * @retval None.
  */
void APT_PWMInitHALSample(void)
{
    /* Initialize GPIO pin for timer interrupt test. */
    SystemInit();
    IRQ_Enable();
    InterruptInitAptU();
    /* Initial APT module synchronization. */
    APT_SyncMasterInit(&g_hAptU);
    APT_SyncSlaveInit(&g_hAptV);
    APT_SyncSlaveInit(&g_hAptW);

    /* Start APT module of U, V, W phases. */
    HAL_APT_StartModule(APT_RUN_U | APT_RUN_V | APT_RUN_W);
}

/**
  * @brief Modify the duty ratio of PWM waveform for U phase APT.
  * @param duty Duty of PWM waveform.
  * @retval None.
  */
void APT_SetPWMDutyU(unsigned int duty)
{
    HAL_APT_SetPWMDutyByNumber(&g_hAptU, duty);
}

/**
  * @brief Modify the duty ratio of PWM waveform for V phase APT.
  * @param duty Duty of PWM waveform.
  * @retval None.
  */
void APT_SetPWMDutyV(unsigned int duty)
{
    HAL_APT_SetPWMDutyByNumber(&g_hAptV, duty);
}

/**
  * @brief Modify the duty ratio of PWM waveform for W phase APT.
  * @param duty Duty of PWM waveform.
  * @retval None.
  */
void APT_SetPWMDutyW(unsigned int duty)
{
    HAL_APT_SetPWMDutyByNumber(&g_hAptW, duty);
}

/**
  * @brief Modify the ADC trigger time of master APT module (U phase).
  * @param cntCmpSOCA Count compare value of SOCA.
  * @param cntCmpSOCB Counnt compare value of SOCB.
  * @retval None.
  */
void APT_SetADCTrgTime(unsigned short cntCmpSOCA, unsigned short cntCmpSOCB, ADC_SampleMode mode)
{
    /* AptU use CMPA and CMPB as the trigger source of SOCA and SOCB. */
    /* SOCA is used to trigger 1st ADC sampling when using single resistor sampling. */
    /* SOCB is used to trigger 2nd ADC sampling when using single resistor sampling. */
    if (mode == ADC_SINGLE_RESISTOR) {
        HAL_APT_SetADCTriggerTime(&g_hAptU, cntCmpSOCA, cntCmpSOCB);
    } else {
        HAL_APT_SetADCTriggerTime(&g_hAptU, cntCmpSOCA, cntCmpSOCB);
        HAL_APT_SetADCTriggerTime(&g_hAptV, cntCmpSOCA, cntCmpSOCB);
        HAL_APT_SetADCTriggerTime(&g_hAptW, cntCmpSOCA, cntCmpSOCB);
    }
}

/**
  * @brief PWM waveform output control.
  * @param enable pwm waveform output enable.
  * @retval None.
  */
void APT_PhaseOut(bool enable)
{
    if (enable == true) {
        /* Enable PWM U waveform output. */
        DCL_APT_DisableSwContPWMAction(APT_U, APT_PWM_CHANNEL_A);
        DCL_APT_DisableSwContPWMAction(APT_U, APT_PWM_CHANNEL_B);
        /* Enable PWM V waveform output. */
        DCL_APT_DisableSwContPWMAction(APT_V, APT_PWM_CHANNEL_A);
        DCL_APT_DisableSwContPWMAction(APT_V, APT_PWM_CHANNEL_B);
        /* Enable PWM W waveform output. */
        DCL_APT_DisableSwContPWMAction(APT_W, APT_PWM_CHANNEL_A);
        DCL_APT_DisableSwContPWMAction(APT_W, APT_PWM_CHANNEL_B);
    } else {
        /* Disable PWM  U waveform output. */
        DCL_APT_EnableSwContPWMAction(APT_U, APT_PWM_CHANNEL_A);
        DCL_APT_EnableSwContPWMAction(APT_U, APT_PWM_CHANNEL_B);
        DCL_APT_ForcePWMOutputLow(APT_U);
        /* Disable PWM  V waveform output. */
        DCL_APT_EnableSwContPWMAction(APT_V, APT_PWM_CHANNEL_A);
        DCL_APT_EnableSwContPWMAction(APT_V, APT_PWM_CHANNEL_B);
        DCL_APT_ForcePWMOutputLow(APT_V);
        /* Disable PWM W waveform output. */
        DCL_APT_EnableSwContPWMAction(APT_W, APT_PWM_CHANNEL_A);
        DCL_APT_EnableSwContPWMAction(APT_W, APT_PWM_CHANNEL_B);
        DCL_APT_ForcePWMOutputLow(APT_W);
    }
}
