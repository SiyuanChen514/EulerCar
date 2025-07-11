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
  * @file      timer.c
  * @author    MCU Driver Team
  * @brief     TIMER module driver.
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the TIMER.
  *              + Initialization and de-initialization functions
  *              + config the register of timer
  */

/* Includes ------------------------------------------------------------------*/
#include "timer.h"
#include "interrupt.h"

/**
  * @brief   Init the timer
  * @param   handle  Timer Handle
  * @retval  BASE_STATUS_OK    Success
  * @retval  BASE_STATUS_ERROR parameter check fail
  */
BASE_StatusType HAL_TIMER_Init(TIMER_Handle *handle)
{
    TIMER_ASSERT_PARAM(handle != NULL);
    TIMER_ASSERT_PARAM(IsTIMERInstance(handle->baseAddress));

    TIMER_PARAM_CHECK_WITH_RET(IsTimerPeriod(handle->load), BASE_STATUS_ERROR);
    TIMER_PARAM_CHECK_WITH_RET(IsTimerPeriod(handle->bgLoad), BASE_STATUS_ERROR);
    TIMER_PARAM_CHECK_WITH_RET(IsTimerMode(handle->mode), BASE_STATUS_ERROR);
    TIMER_PARAM_CHECK_WITH_RET(IsTimerSize(handle->size), BASE_STATUS_ERROR);
    TIMER_PARAM_CHECK_WITH_RET(IsTimerDiv(handle->prescaler), BASE_STATUS_ERROR);
    TIMER_PARAM_CHECK_WITH_RET(IsTimerCntMode(handle->cntMode), BASE_STATUS_ERROR);
    /* Initialize the configuration parameters of the timer */
    handle->baseAddress->timer_intclr = BASE_CFG_SET;  /* Writing to this register clears interrupt output of timer */
    handle->baseAddress->timer_load = handle->load;
    handle->baseAddress->timerbgload = handle->bgLoad;
    handle->baseAddress->TIMERx_CONTROL.BIT.timeren = BASE_CFG_DISABLE;
    handle->baseAddress->TIMERx_CONTROL.BIT.intenable = handle->interruptEn;
    /* Sets the frequency divider and size of the timer module. */
    handle->baseAddress->TIMERx_CONTROL.BIT.timerpre = handle->prescaler;
    handle->baseAddress->TIMERx_CONTROL.BIT.timersize = handle->size;
    /* Sets the running mode of the timer. */
    if (handle->mode == TIMER_MODE_RUN_ONTSHOT) {
        handle->baseAddress->TIMERx_CONTROL.BIT.oneshot = BASE_CFG_SET;
    } else {
        handle->baseAddress->TIMERx_CONTROL.BIT.oneshot = BASE_CFG_UNSET;
        handle->baseAddress->TIMERx_CONTROL.BIT.timermode =
            (handle->mode == TIMER_MODE_RUN_FREE) ? BASE_CFG_UNSET : BASE_CFG_SET;
    }
    /* Request for setting the DMA and ADC. */
    handle->baseAddress->TIMERx_CONTROLB.BIT.dmasreqen = handle->adcSocReqEnable ||
                                                         handle->dmaReqEnable;
    handle->baseAddress->TIMERx_CONTROLB.BIT.dmabreqen = handle->dmaReqEnable;
    return BASE_STATUS_OK;
}

/**
  * @brief   DeInit the timer
  * @param   handle  Timer Handle
  * @retval  None
  */
void HAL_TIMER_DeInit(TIMER_Handle *handle)
{
    TIMER_ASSERT_PARAM(handle != NULL);
    TIMER_ASSERT_PARAM(IsTIMERInstance(handle->baseAddress));
    /* Clears interrupts and masks interrupts. */
    handle->baseAddress->timer_intclr = BASE_CFG_SET;
    handle->baseAddress->TIMERx_CONTROL.BIT.intenable = BASE_CFG_DISABLE;
    handle->userCallBack.TimerPeriodFinCallBack = NULL;    /* Clear all user call back function. */
    /* The counter loading value is set to 0, and the timer is disabled. */
    handle->baseAddress->timer_load = 0;
    handle->baseAddress->timerbgload = 0;
    handle->baseAddress->TIMERx_CONTROL.reg = 0;
    handle->baseAddress->TIMERx_CONTROLB.reg = 0;
}

/**
  * @brief   Config Timer
  * @param   handle  Timer Handle
  * @param   cfgType  Timer configures, @ref TIMER_CFG_TYPE
  * @retval  BASE_STATUS_OK    Success
  * @retval  BASE_STATUS_ERROR parameter check fail
  */
BASE_StatusType HAL_TIMER_Config(TIMER_Handle *handle, TIMER_CFG_TYPE cfgType)
{
    TIMER_ASSERT_PARAM(handle != NULL);
    TIMER_ASSERT_PARAM(IsTIMERInstance(handle->baseAddress));

    switch (cfgType) {
        /* Configure timer count. */
        case TIMER_CFG_LOAD:
            TIMER_PARAM_CHECK_WITH_RET(IsTimerPeriod(handle->load), BASE_STATUS_ERROR);
            handle->baseAddress->timer_load = handle->load;
            handle->bgLoad = handle->load;
            break;
        /* Configure timer reload count. */
        case TIMER_CFG_BGLOAD:
            TIMER_PARAM_CHECK_WITH_RET(IsTimerPeriod(handle->bgLoad), BASE_STATUS_ERROR);
            handle->baseAddress->timerbgload = handle->bgLoad;
            break;
        /* Configure timer work mode. */
        case TIMER_CFG_MODE:
            DCL_TIMER_SetTimerMode(handle->baseAddress, handle->mode);
            break;
        /* Configure timer interrupt. */
        case TIMER_CFG_INTERRUPT:
            handle->baseAddress->TIMERx_CONTROL.BIT.intenable = handle->interruptEn;
            break;
        /* Configure timer prescaler. */
        case TIMER_CFG_PRESCALER:
            TIMER_PARAM_CHECK_WITH_RET(IsTimerDiv(handle->prescaler), BASE_STATUS_ERROR);
            handle->baseAddress->TIMERx_CONTROL.BIT.timerpre = handle->prescaler;
            break;
        /* Configure the size of the timer counter. */
        case TIMER_CFG_SIZE:
            TIMER_PARAM_CHECK_WITH_RET(IsTimerSize(handle->size), BASE_STATUS_ERROR);
            handle->baseAddress->TIMERx_CONTROL.BIT.timersize = handle->size;
            break;
        /* Configure the ADC request. */
        case TIMER_CFG_DMAADC_SINGLE_REQ:
            handle->baseAddress->TIMERx_CONTROLB.BIT.dmasreqen = handle->adcSocReqEnable ||
                                                                 handle->dmaReqEnable;
            break;
        /* Configure the DMA request. */
        case TIMER_CFG_DMA_BURST_REQ:
            handle->baseAddress->TIMERx_CONTROLB.BIT.dmabreqen = handle->dmaReqEnable;
            break;
        default:
            return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief   Get Timer Config
  * @param   handle  Timer Handle
  * @retval  BASE_STATUS_OK  Success
  * @retval  BASE_STATUS_ERROR Parameter check fail
  */
BASE_StatusType HAL_TIMER_GetConfig(TIMER_Handle *handle)
{
    TIMER_ASSERT_PARAM(handle != NULL);
    TIMER_ASSERT_PARAM(IsTIMERInstance(handle->baseAddress));

    handle->load   = handle->baseAddress->timer_load;
    handle->bgLoad = handle->baseAddress->timerbgload;
    handle->mode   = handle->baseAddress->TIMERx_CONTROL.BIT.timermode;
    handle->size   = handle->baseAddress->TIMERx_CONTROL.BIT.timersize;
    handle->prescaler   = handle->baseAddress->TIMERx_CONTROL.BIT.timerpre;
    handle->interruptEn = handle->baseAddress->TIMERx_CONTROL.BIT.intenable;
    handle->adcSocReqEnable = handle->baseAddress->TIMERx_CONTROLB.BIT.dmasreqen;
    handle->dmaReqEnable = handle->baseAddress->TIMERx_CONTROLB.BIT.dmasreqen ||
                           handle->baseAddress->TIMERx_CONTROLB.BIT.dmabreqen;
    handle->mode = DCL_TIMER_GetTimerMode(handle->baseAddress);

    return BASE_STATUS_OK;
}

/**
  * @brief   Start timer.
  * @param   handle Timer Handle
  * @retval  None
  */
void HAL_TIMER_Start(TIMER_Handle *handle)
{
    TIMER_ASSERT_PARAM(handle != NULL);
    TIMER_ASSERT_PARAM(IsTIMERInstance(handle->baseAddress));

    handle->baseAddress->TIMERx_CONTROL.BIT.timeren = BASE_CFG_SET;
}

/**
  * @brief   Stop timer.
  * @param   handle Timer Handle
  * @retval  None
  * @note    Timer in OneShot Mode also need stop
  */
void HAL_TIMER_Stop(TIMER_Handle *handle)
{
    TIMER_ASSERT_PARAM(handle != NULL);
    TIMER_ASSERT_PARAM(IsTIMERInstance(handle->baseAddress));

    handle->baseAddress->TIMERx_CONTROL.BIT.timeren = BASE_CFG_UNSET;
    handle->baseAddress->timer_intclr = BASE_CFG_SET;
}

/**
  * @brief TIMER Interrupt service processing function.
  * @param handle TIMER handle.
  * @retval None.
  */
void HAL_TIMER_IrqHandler(void *handle)
{
    TIMER_ASSERT_PARAM(handle != NULL);
    TIMER_Handle *timerHandle = (TIMER_Handle *)handle;
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerHandle->baseAddress));
    /* Check interrupt type */
    if (timerHandle->baseAddress->TIMERx_MIS.timermis == 0x1) {
        /* DMA overflow interrupt */
        timerHandle->baseAddress->timer_intclr = BASE_CFG_ENABLE;
        /* Call the period finish callback function of the user. */
        if (timerHandle->userCallBack.TimerPeriodFinCallBack != NULL) {
            timerHandle->userCallBack.TimerPeriodFinCallBack(timerHandle);
        }
    }
    return;
}

/**
  * @brief   Register the callback function of TIMER handle.
  * @param   handle       Timer Handle
  * @param   typeID       Timer interrupt typr, @ref TIMER_InterruptType
  * @param   callBackFunc CallBack function of user, @ref TIMER_CallBackFunc
  * @retval  BASE_STATUS_OK  Success
  * @retval  BASE_STATUS_ERROR Parameter check fail
  */
BASE_StatusType HAL_TIMER_RegisterCallback(TIMER_Handle *handle, TIMER_InterruptType typeID,
                                           TIMER_CallBackFunc callBackFunc)
{
    TIMER_ASSERT_PARAM(handle != NULL);
    TIMER_ASSERT_PARAM(callBackFunc != NULL);

    switch (typeID) {
        case TIMER_PERIOD_FIN:
            handle->userCallBack.TimerPeriodFinCallBack = callBackFunc;  /* User timer period finish call back. */
            break;
        default:
            return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief   Unregister the callback function of TIMER handle.
  * @param   handle   Timer Handle
  * @param   typeID   CallBack function of user, @ref TIMER_InterruptType
  * @retval  BASE_STATUS_OK  Success
  * @retval  BASE_STATUS_ERROR Parameter check fail
  */
BASE_StatusType HAL_TIMER_UnRegisterCallback(TIMER_Handle *handle, TIMER_InterruptType typeID)
{
    TIMER_ASSERT_PARAM(handle != NULL);
    /* Determine the callback function type. */
    switch (typeID) {
        case TIMER_PERIOD_FIN:
            handle->userCallBack.TimerPeriodFinCallBack = NULL;   /* Periodic callback for timer period finish. */
            break;
        default:
            return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}