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
  * @file     mcs_carrier.h
  * @author   MCU Algorithm Team
  * @brief    This file provides functions declaration for carrier interrupt processing function.
  */
#ifndef McuMagicTag_MCS_CARRIER_H
#define McuMagicTag_MCS_CARRIER_H

#include "mcs_sys_status.h"
#include "mcs_mtr_param.h"
#include "mcs_fsm.h"
#include "mcs_svpwm.h"
#include "mcs_curr_ctrl.h"
#include "mcs_if_ctrl.h"
#include "mcs_ramp_mgmt.h"
#include "mcs_spd_ctrl.h"
#include "mcs_fosmo.h"
#include "mcs_pll.h"
#include "mcs_startup.h"
#include "mcs_r1_svpwm.h"
#include "mcs_adcCalibr.h"

typedef struct {
    unsigned int iuAdcCalibr; /* iu adc temperature calibration */
    unsigned int ivAdcCalibr; /* iv adc temperature calibration */
    unsigned int iwAdcCalibr; /* iw adc temperature calibration */
    unsigned int iBusAdcCalibr; /* ibus adc temperature calibration */
} IBIAS_Handle; /* Current Bias */

typedef void (*MCS_ReadCurrUvwCb)(UvwAxis *CurrUvw, IBIAS_Handle *adcBiasIuvw);
typedef void (*MCS_SetPwmDutyCb)(UvwAxis *dutyUvwLeft, UvwAxis *dutyUvwRight);
typedef void (*MCS_SetADCTriggerTimeCb)(unsigned short cntCmpSOCA, unsigned short cntCmpSOCB);
typedef void (*MCS_AdcCalibrCurrUvwCb)(IBIAS_Handle *adcCalibrCurrUvw);

/**
  * @brief Sampling mode.
  */
typedef enum {
    THREE_RESISTORS = 0,
    DUAL_RESISTORS = 1,
    SINGLE_RESISTOR = 2,
    SAMPLE_MODE_END,
} MCS_SampleMode;

/**
  * @brief Motor control data structure
  */
typedef struct {
    float spdCmd;       /**< External input speed command value */
    float axisAngle;    /**< Angle of the synchronous coordinate system, used for coordinate transformation */
    float spdRef;       /**< Command value after speed ramp management */
    float spdFbk;       /**< Motor speed feedback (Hz). */
    float ts;           /**< current loop control period */
    unsigned short aptMaxcntCmp; /**< Apt Maximum Comparison Count */

    unsigned short sysTickCnt;       /**< System Timer Tick Count */
    unsigned short capChargeTickNum; /**< Bootstrap Capacitor Charge Tick Count */
    volatile unsigned int msTickCnt; /**< Millisecond-level counter, which can be used in 1-ms and 5-ms tasks. */

    SysStatusReg statusReg; /**< System status */
    FsmState stateMachine;  /**< Motor Control State Machine */

    MCS_SampleMode sampleMode;   /**< sample mode */
    MOTOR_Param mtrParam;        /**< Motor parameters */
    FOSMO_Handle smo;            /**< SMO observer handle */
    IF_Handle ifCtrl;            /**< I/F control handle */
    SVPWM_Handle sv;             /**< SVPWM Handle */
    R1SVPWM_Handle r1Sv;         /**< Single-resistance phase-shifted SVPWM handld */
    RMG_Handle spdRmg;           /**< Ramp management struct for the speed controller input reference */
    SPDCTRL_Handle spdCtrl;      /**< Speed loop Control Handle */
    CURRCTRL_Handle currCtrl;    /**< Current loop control handle */
    STARTUP_Handle startup;      /**< Startup Switch Handle */
    IBIAS_Handle adcCalibrCurrUvw;  /**< Phase current ADC calibration handle. */

    DqAxis idqRef;          /**< Command value of the dq axis current */
    UvwAxis iuvw;           /**< Three-phase current sampling value */
    AlbeAxis iabFbk;        /**< αβ-axis current feedback value */
    DqAxis idqFbk;          /**< Current feedback value of the dq axis */
    DqAxis vdqRef;          /**< Current loop output dq voltage */
    AlbeAxis vabRef;        /**< Current loop output voltage αβ */
    UvwAxis  dutyUvw;       /**< UVW three-phase duty cycle */
    UvwAxis  dutyUvwLeft;   /**< Single Resistor UVW Three-Phase Left Duty Cycle */
    UvwAxis  dutyUvwRight;  /**< Single Resistor UVW Three-Phase Right Duty Cycle*/

    MCS_ReadCurrUvwCb readCurrUvwCb;             /**< Read current callback function */
    MCS_SetPwmDutyCb setPwmDutyCb;	             /**< Set the duty cycle callback function. */
    MCS_SetADCTriggerTimeCb setADCTriggerTimeCb; /**< Sets the ADC trigger point callback function. */
    MCS_AdcCalibrCurrUvwCb readCurrBiasCb;       /**< Phase current ADC calibration function pointer. */
} MTRCTRL_Handle;

void MCS_CarrierProcess(MTRCTRL_Handle *mtrCtrl);

#endif