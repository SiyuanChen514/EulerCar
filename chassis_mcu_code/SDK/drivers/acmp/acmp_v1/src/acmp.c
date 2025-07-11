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
 * @file    acmp.c
 * @author  MCU Driver Team.
 * @brief   ACMP HAL level module driver.
 *          This file provides firmware functions to manage the following
 *          functionalities of ACMP.
 *           + Comparator's Initialization and de-initialization functions
 *           + Set Comparator's hysteresis voltage function
 *           + Set software blking valid function
 *           + Set software blking invalid function
 */
#include "acmp.h"
#include "assert.h"


/* Define -------------- */
#define ACMP_INTERRUPT_ENABLE   0b111
#define ACMP_POS_INTERUPT       0b100
#define ACMP_NEG_INTERUPT       0b010
#define ACMP_EDGE_INTERRUPT     0b001


/**
  * @brief Input and output initialization of comparator
  * @param acmpHandle: ACMP handle.
  * @retval None.
  */
static void ACMP_InputOutputInit(ACMP_Handle *acmpHandle)
{
    ACMP_ASSERT_PARAM(acmpHandle != NULL);
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpHandle->baseAddress));
    ACMP_PARAM_CHECK_NO_RET(IsACMPOutputPolarity(acmpHandle->inOutConfig.polarity));
    /* Check input multiplexing selection and input switch selection */
    ACMP_PARAM_CHECK_NO_RET(IsACMPInputPNumber(acmpHandle->inOutConfig.inputPNum));
    ACMP_PARAM_CHECK_NO_RET(IsACMPInputNNumber(acmpHandle->inOutConfig.inputNNum));
    /* input positive selection */
    acmpHandle->baseAddress->ACMP_CTRL1.BIT.da_acmp_input_psel = acmpHandle->inOutConfig.inputPNum;
    /* input negative selection */
    acmpHandle->baseAddress->ACMP_CTRL1.BIT.da_acmp_input_nsel = acmpHandle->inOutConfig.inputNNum;
    /* output polarity selection */
    acmpHandle->baseAddress->ACMP_CTRL2.BIT.cfg_acmp_out_inv = acmpHandle->inOutConfig.polarity;
}

/**
  * @brief Filter initialization of comparator
  * @param acmpHandle: ACMP handle.
  * @retval None.
  */
static void ACMP_FilterInit(ACMP_Handle *acmpHandle)
{
    ACMP_ASSERT_PARAM(acmpHandle != NULL);
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpHandle->baseAddress));
    unsigned short blkingSrc;
    switch (acmpHandle->filterCtrl.filterMode) {
        case ACMP_FILTER_NONE: /* The filtering function is not applicable. */
            acmpHandle->baseAddress->ACMP_CTRL3.BIT.cfg_acmp_filter_en = BASE_CFG_DISABLE; /* Disable filtering */
            acmpHandle->baseAddress->ACMP_CTRL4.BIT.cfg_acmp_blk_en = BASE_CFG_DISABLE;    /* Disable blocking */

            acmpHandle->baseAddress->ACMP_CTRL2.BIT.cfg_acmp_out_sel = 0x0; /* 0x0: Output raw comparison result. */
            break;
        case ACMP_FILTER_BLOCK:  /* Use the blockinng function. */
            acmpHandle->baseAddress->ACMP_CTRL4.BIT.cfg_acmp_blk_en = BASE_CFG_ENABLE;  /* Enable blocking. */
            blkingSrc = acmpHandle->filterCtrl.blkingSrcSelect;
            acmpHandle->baseAddress->ACMP_CTRL4.BIT.cfg_acmp_blk_sel = blkingSrc; /* Setting Blking source */
            if (blkingSrc == ACMP_BLKING_SRC_SOFT) {
                /* Sets the polarity of the window.. */
                acmpHandle->baseAddress->ACMP_CTRL4.BIT.cfg_acmp_blk_pol_sel = acmpHandle->filterCtrl.blkingPorty;
            } else {
                /* Blocking source from apt window. */
                acmpHandle->baseAddress->ACMP_CTRL4.BIT.cfg_acmp_blk_pol_sel = BASE_CFG_ENABLE;
                acmpHandle->baseAddress->ACMP_CTRL4.BIT.cfg_acmp_blk_rslt_pol = BASE_CFG_DISABLE;
            }
            acmpHandle->baseAddress->ACMP_CTRL2.BIT.cfg_acmp_out_sel = 0x2; /* 0x2: Outputs digital filtered and
                                                                            masked comparison results */
            break;
        case ACMP_FILTER_FILTER: /* Set the filtering function. */
            acmpHandle->baseAddress->ACMP_CTRL3.BIT.cfg_acmp_filter_en = BASE_CFG_ENABLE; /* Enable filtering. */
            /* Filter length setting. */
            acmpHandle->baseAddress->ACMP_CTRL3.BIT.cfg_acmp_filter_step = acmpHandle->filterCtrl.filterStep;

            acmpHandle->baseAddress->ACMP_CTRL2.BIT.cfg_acmp_out_sel = 0x1; /* 0x1: Outputs filtering result. */
            break;
        case ACMP_FILTER_BOTH: /* Use filtering and shielding functions. */
            acmpHandle->baseAddress->ACMP_CTRL3.BIT.cfg_acmp_filter_en = BASE_CFG_ENABLE; /* Enable filtering. */
            acmpHandle->baseAddress->ACMP_CTRL3.BIT.cfg_acmp_filter_step = acmpHandle->filterCtrl.filterStep;
  
            acmpHandle->baseAddress->ACMP_CTRL4.BIT.cfg_acmp_blk_en = BASE_CFG_ENABLE; /* Enable blocking. */
            blkingSrc = acmpHandle->filterCtrl.blkingSrcSelect;
            acmpHandle->baseAddress->ACMP_CTRL4.BIT.cfg_acmp_blk_sel = blkingSrc;      /* Setting blocking source. */
            if (blkingSrc == ACMP_BLKING_SRC_SOFT) {
                /* Setting Blking source from software. */
                acmpHandle->baseAddress->ACMP_CTRL4.BIT.cfg_acmp_blk_pol_sel = acmpHandle->filterCtrl.blkingPorty;
            } else {
                /* Blocking source from apt window. */
                acmpHandle->baseAddress->ACMP_CTRL4.BIT.cfg_acmp_blk_pol_sel = BASE_CFG_ENABLE;
                acmpHandle->baseAddress->ACMP_CTRL4.BIT.cfg_acmp_blk_rslt_pol = BASE_CFG_DISABLE;
            }
            acmpHandle->baseAddress->ACMP_CTRL2.BIT.cfg_acmp_out_sel = 0x2; /* 0x2: Outputs digital filtered and
                                                                                    masked comparison results */
            break;
        default:
            acmpHandle->baseAddress->ACMP_CTRL3.BIT.cfg_acmp_filter_en = BASE_CFG_DISABLE; /* Disable filtering. */
            acmpHandle->baseAddress->ACMP_CTRL4.BIT.cfg_acmp_blk_en = BASE_CFG_DISABLE;    /* Disable blocking. */
            break;
    }
}

/**
  * @brief Comparator HAL Init
  * @param acmpHandle: ACMP handle.
  * @retval BASE_StatusType: OK, ERROR
  */
BASE_StatusType HAL_ACMP_Init(ACMP_Handle *acmpHandle)
{
    ACMP_ASSERT_PARAM(acmpHandle != NULL);
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpHandle->baseAddress));
    /* Parameter macro check. */
    ACMP_PARAM_CHECK_WITH_RET(acmpHandle->hysteresisVol >=  ACMP_HYS_VOL_ZERO, BASE_STATUS_ERROR);
    ACMP_PARAM_CHECK_WITH_RET(acmpHandle->hysteresisVol <= ACMP_HYS_VOL_30MV, BASE_STATUS_ERROR);
    ACMP_PARAM_CHECK_WITH_RET(acmpHandle->filterCtrl.filterStep >= 0, BASE_STATUS_ERROR);
    ACMP_PARAM_CHECK_WITH_RET(acmpHandle->filterCtrl.filterStep <= ACMP_FILTER_STEP_MAX_VALUE, BASE_STATUS_ERROR);
    ACMP_PARAM_CHECK_WITH_RET(IsACMPBlkingSrcType(acmpHandle->filterCtrl.blkingSrcSelect), BASE_STATUS_ERROR);
    /* Enable ACMP. */
    acmpHandle->baseAddress->ACMP_CTRL0.BIT.da_acmp_enh = BASE_CFG_ENABLE;
    /* Enable ACMP interrupt. */
    if (acmpHandle->interruptEn == BASE_CFG_SET) {
        acmpHandle->baseAddress->ACMP_INTR_MASK.reg = ACMP_INTERRUPT_ENABLE;  /* Configure acmp interrupt. */
    } else {
        acmpHandle->baseAddress->ACMP_INTR_MASK.reg = BASE_CFG_UNSET;         /* Disable acmp interrupt. */
    }
    /* ACMP input and output settings. */
    ACMP_InputOutputInit(acmpHandle);
    /* ACMP comparison filtering function. */
    ACMP_FilterInit(acmpHandle);
    /* Set hysteresis voltage */
    HAL_ACMP_SetHystVol(acmpHandle, acmpHandle->hysteresisVol);

    BASE_FUNC_DELAY_US(150);    /* After the configuration is complete, a delay of 150 us is required. */
    return BASE_STATUS_OK;
}

/**
  * @brief Comparator HAL DeInit
  * @param acmpHandle: ACMP handle.
  * @retval BASE_StatusType: OK, ERROR
  */
BASE_StatusType HAL_ACMP_DeInit(ACMP_Handle *acmpHandle)
{
    ACMP_ASSERT_PARAM(acmpHandle != NULL);
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpHandle->baseAddress));
    acmpHandle->baseAddress->ACMP_CTRL0.reg = BASE_CFG_DISABLE;     /* Disable ACMP. */
    acmpHandle->baseAddress->ACMP_CTRL1.reg = BASE_CFG_DISABLE;     /* Clears the input and output status. */
    acmpHandle->baseAddress->ACMP_CTRL2.reg = BASE_CFG_DISABLE;     /* Clears the comparison result selection. */
    acmpHandle->baseAddress->ACMP_INTR.reg = BASE_CFG_DISABLE;      /* Clear all interrrupt. */
    acmpHandle->userCallBack.AcmpEdgedCallBack = NULL;              /* Clears all user callback functions. */
    acmpHandle->userCallBack.AcmpNegativeCallBack = NULL;
    acmpHandle->userCallBack.AcmpPositiveCallBack = NULL;
    return BASE_STATUS_OK;
}

/**
  * @brief Set hysteresis Voltage
  * @param acmpHandle: ACMP handle.
  * @param voltage: hysteresis voltage to be set. @ref ACMP_HystVol
  * @retval None.
  */
void HAL_ACMP_SetHystVol(ACMP_Handle *acmpHandle, ACMP_HystVol voltage)
{
    ACMP_ASSERT_PARAM(acmpHandle != NULL);
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpHandle->baseAddress));
    ACMP_PARAM_CHECK_NO_RET(voltage >=  ACMP_HYS_VOL_ZERO);
    ACMP_PARAM_CHECK_NO_RET(voltage <= ACMP_HYS_VOL_30MV);
    acmpHandle->baseAddress->ACMP_RSV.BIT.da_acmp_rsv = voltage; /* Hysteresis voltage setting. */
}

/**
  * @brief Set blocking valid
  * @param acmpHandle: ACMP handle.
  * @retval None.
  */
void HAL_ACMP_BlkingValid(ACMP_Handle *acmpHandle)
{
    ACMP_ASSERT_PARAM(acmpHandle != NULL);
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpHandle->baseAddress));
    /* Enable Blocking function. */
    acmpHandle->baseAddress->ACMP_CTRL4.BIT.cfg_acmp_blk_en = BASE_CFG_ENABLE;
}

/**
  * @brief Set blocking invalid
  * @param acmpHandle: ACMP handle.
  * @retval None.
  */
void HAL_ACMP_BlkingInvalid(ACMP_Handle *acmpHandle)
{
    ACMP_ASSERT_PARAM(acmpHandle != NULL);
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpHandle->baseAddress));
    acmpHandle->baseAddress->ACMP_CTRL4.BIT.cfg_acmp_blk_en = BASE_CFG_DISABLE;  /* Disable blocking function. */
}

/**
  * @brief Sets the output result of ACMP.
  * @param acmpHandle: ACMP handle.
  * @param resultSelect: ACMP result output options.
  * @retval BASE_StatusType: OK, ERROR.
  */
BASE_StatusType HAL_ACMP_ResultSelect(ACMP_Handle *acmpHandle, ACMP_ResultSelect resultSelect)
{
    ACMP_ASSERT_PARAM(acmpHandle != NULL);
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpHandle->baseAddress));
    ACMP_PARAM_CHECK_WITH_RET(IsACMPResultSeletion(resultSelect), BASE_STATUS_ERROR);
    /* Output result selection of the comparator. */
    switch (resultSelect) {
        case ACMP_RESULT_SIMULATION:
            acmpHandle->baseAddress->ACMP_CTRL2.BIT.cfg_acmp_out_sel = 0x0; /* 0x0: Original comparison results. */
            break;
        case ACMP_RESULT_FILTER:
            acmpHandle->baseAddress->ACMP_CTRL2.BIT.cfg_acmp_out_sel = 0x1; /* 0x1: Resulter after filtering. */
            break;
        case ACMP_RESULT_FILTER_BLOCK:
            acmpHandle->baseAddress->ACMP_CTRL2.BIT.cfg_acmp_out_sel = 0x2; /* 0x2: Resulter after filtering
                                                                                    and blocking. */
            break;
        default:
            return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief ACMP Interrupt service processing function.
  * @param handle ACMP handle.
  * @retval None.
  */
void HAL_ACMP_IrqHandler(void *handle)
{
    ACMP_ASSERT_PARAM(handle != NULL);
    ACMP_Handle *acmpHandle = (ACMP_Handle *)handle;
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpHandle->baseAddress));
    /* Check interrupt type */
    if (acmpHandle->baseAddress->ACMP_INTR_MSK.BIT.intr_acmp_pos_msk == BASE_CFG_ENABLE) {
        /* Rising Edge Interrupt. */
        acmpHandle->baseAddress->ACMP_INTR.reg = ACMP_POS_INTERUPT;  /* Clears the rising edge interrupt. */
        /* Call the rising edge user interrupt function. */
        if (acmpHandle->userCallBack.AcmpPositiveCallBack != NULL) {
            acmpHandle->userCallBack.AcmpPositiveCallBack(acmpHandle);
        }
    }
    if (acmpHandle->baseAddress->ACMP_INTR_MSK.BIT.intr_acmp_neg_msk == BASE_CFG_ENABLE) {
        /* Falling Edge Interrupt. */
        acmpHandle->baseAddress->ACMP_INTR.reg = ACMP_NEG_INTERUPT;  /* Clears falling Edge Interrupt. */
        /* Call the falling edge user interrupt function. */
        if (acmpHandle->userCallBack.AcmpNegativeCallBack != NULL) {
            acmpHandle->userCallBack.AcmpNegativeCallBack(acmpHandle);
        }
    }
    if (acmpHandle->baseAddress->ACMP_INTR_MSK.BIT.intr_acmp_edge_msk == BASE_CFG_ENABLE) {
        /* Flip edge interrupt. */
        acmpHandle->baseAddress->ACMP_INTR.reg = ACMP_EDGE_INTERRUPT;  /* Clears Flip edge interrupt. */
        /* Call flip edge user interrupt function. */
        if (acmpHandle->userCallBack.AcmpEdgedCallBack != NULL) {
            acmpHandle->userCallBack.AcmpEdgedCallBack(acmpHandle);
        }
    }
    return;
}

/**
  * @brief   Register the callback function of ACMP handle.
  * @param   acmpHandle   Acmp Handle
  * @param   typeID       CallBack function type of user, @ref ACMP_CallBackFun_Type
  * @param   callBackFunc CallBack function of user, @ref ACMM_CallBackType
  * @retval  BASE_STATUS_OK  Success
  * @retval  BASE_STATUS_ERROR Parameter check fail
  */
BASE_StatusType HAL_ACMP_RegisterCallBack(ACMP_Handle *acmpHandle, ACMP_CallBackFun_Type typeID,
                                          ACMP_CallBackType callBackFunc)
{
    ACMP_ASSERT_PARAM(acmpHandle != NULL);
    ACMP_ASSERT_PARAM(callBackFunc != NULL);
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpHandle->baseAddress));
    /* Registers user callback function. */
    switch (typeID) {
        case ACMP_POS_INT:   /* Register rising edge user callback function */
            acmpHandle->userCallBack.AcmpPositiveCallBack = callBackFunc;
            break;
        case ACMP_NEG_INT:   /* Register failing edge user callback function */
            acmpHandle->userCallBack.AcmpNegativeCallBack = callBackFunc;
            break;
        case ACMP_EDGE_INT: /* Register fliping edge user callback function */
            acmpHandle->userCallBack.AcmpEdgedCallBack = callBackFunc;
            break;
        default:
            return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}