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
  * @file      mcs_motor_process.c
  * @author    MCU Algorithm Team
  * @brief     This file provides motor application for AD105HDMA board.
  * @details   Dual FOC application based on the AD105HDMA VER.A board
  *            1) Both motor model is JMC 42JSF630AS-1000.
  *            2) Select the Motorcontrolsystem example in the sample column of chipConfig and click Generate Code.
  *            3) The AD105HDMA VER.A board is a high-voltage board, its power supply must be changed to 24V.
  */
#include "mcs_motor_process.h"
#include "mcs_user_config.h"
#include "mcs_math_const.h"
#include "mcs_math.h"
#include "mcs_carrier.h"

#define APT_FULL_DUTY 1.0f

/* Motor parameters */
/* Np, Rs, Ld, Lq, Psif, J, Nmax, Currmax, PPMR, zShift */
static MOTOR_Param g_motorParamCp = {
    MOTOR_PARAM_NP,
    MOTOR_PARAM_RS,
    MOTOR_PARAM_LD,
    MOTOR_PARAM_LQ,
    MOTOR_PARAM_LS,
    MOTOR_PARAM_PSIF,
    MOTOR_PARAM_JS,
    MOTOR_PARAM_MAX_SPD,
    MOTOR_PARAM_MAX_CURR,
    MOTOR_PARAM_MAX_TRQ,
    MOTOR_PARAM_ENCODER_PPMR,
    MOTOR_PARAM_ENCODER_ZSHIFT
};

/* Np, Rs, Ld, Lq, Psif, J, Nmax, Currmax, PPMR, zShift */
static MOTOR_Param g_motorParamFan = {
    MOTOR_PARAM_NP,
    MOTOR_PARAM_RS,
    MOTOR_PARAM_LD,
    MOTOR_PARAM_LQ,
    MOTOR_PARAM_LS,
    MOTOR_PARAM_PSIF,
    MOTOR_PARAM_JS,
    MOTOR_PARAM_MAX_SPD,
    MOTOR_PARAM_MAX_CURR,
    MOTOR_PARAM_MAX_TRQ,
    MOTOR_PARAM_ENCODER_PPMR,
    MOTOR_PARAM_ENCODER_ZSHIFT
};

static APT_RegStruct *g_aptCp[PHASE_MAX_NUM] = {APT_U_CP, APT_V_CP, APT_W_CP};
static APT_RegStruct *g_aptFan[PHASE_MAX_NUM] = {APT_U_FAN, APT_V_FAN, APT_W_FAN};

/* Motor control handle for compressor */
static MTRCTRL_Handle g_mc;

/* Motor control handle for fan */
static MTRCTRL_Handle g_fan;

/**
  * @brief Initialzer of system tick.
  * @param mtrCtrl Motor control struct handle.
  * @retval None.
  */
static void TimerTickInit(MTRCTRL_Handle *mtrCtrl)
{
    mtrCtrl->sysTickCnt = 0;
    mtrCtrl->capChargeTickNum = (INV_CAP_CHARGE_MS * MICROSECOND_NUM_PER_MILLISECOND / SYSTICK_PERIOD_US);
}

/**
  * @brief Initialzer of smo control struct handle.
  * @param fosmo first-order smo struct handle.
  * @param ts first-order ts.
  * @retval None.
  */
static void FOSMO_InitWrapper(FOSMO_Handle *fosmo, float ts)
{
    /* Configuring first-order smo Parameters. */
    FOSMO_Param fosmoParam = {
        .gain = FOSMO_GAIN,
        .lambda = FOSMO_LAMBDA,
        .fcEmf = FOSMO_EMF_CUTOFF_FREQ,
        .fcLpf = FOSMO_SPD_CUTOFF_FREQ,
        .pllBdw = FOSMO_PLL_BDW,
    };
    FOSMO_Init(fosmo, fosmoParam, g_motorParamCp, ts);
}

/**
  * @brief Initialzer of speed controller handle.
  * @param spdHandle speed controller struct handle.
  * @param ts speed controller ts.
  * @retval None.
  */
static void SPDCTRL_InitWrapper(SPDCTRL_Handle *spdHandle, float ts)
{
    /* Configuring speed Controller Parameters. */
    PI_Param spdPi = {
        .kp = SPD_KP,
        .ki = SPD_KI,
        .lowerLim = SPD_LOWERLIM,
        .upperLim = SPD_UPPERLIM,
    };
    SPDCTRL_Init(spdHandle, &g_motorParamCp, spdPi, ts);
}

/**
  * @brief Initialzer of current controller handle.
  * @param fosmo current controller struct handle.
  * @param idqRef DQ-axis current reference value.
  * @param idqFbk DQ-axis current feedback value.
  * @param ts current controller ts.
  * @retval None.
  */
static void CURRCTRL_InitWrapper(CURRCTRL_Handle *currHandle, DqAxis *idqRef, DqAxis *idqFbk, float ts)
{
    /* Configuring Current Controller Parameters. */
    PI_Param dqCurrPi = {
        .kp = CURR_KP,
        .ki = CURR_KI,
        .lowerLim = CURR_LOWERLIM,
        .upperLim = CURR_UPPERLIM,
    };
    CURRCTRL_Init(currHandle, &g_motorParamCp, idqRef, idqFbk, dqCurrPi, dqCurrPi, ts);
}

/**
  * @brief Init motor control task.
  * @retval None.
  */
static void TSK_InitCp(void)
{
    g_mc.stateMachine = FSM_IDLE;
    g_mc.spdCmd = USER_TARGET_SPD_HZ;
    g_mc.aptMaxcntCmp = APT_DUTY_MAX;
    g_mc.sampleMode = SINGLE_RESISTOR;
    g_mc.ts = CTRL_CURR_PERIOD; /* Init current controller */

    IF_Init(&g_mc.ifCtrl, CTRL_IF_CURR_AMP_A, USER_CURR_SLOPE, CTRL_SYSTICK_PERIOD, CTRL_CURR_PERIOD);
    /* Init speed slope */
    RMG_Init(&g_mc.spdRmg, CTRL_SYSTICK_PERIOD, USER_SPD_SLOPE);
    /* Init motor param */
    MtrParamInit(&g_mc.mtrParam, g_motorParamCp);

    TimerTickInit(&g_mc);

    SVPWM_Init(&g_mc.sv, INV_VOLTAGE_BUS * ONE_DIV_SQRT3);

    R1SVPWM_Init(&g_mc.r1Sv, INV_VOLTAGE_BUS * ONE_DIV_SQRT3, SAMPLE_POINT_SHIFT, SAMPLE_WINDOW_DUTY);

    STARTUP_Init(&g_mc.startup, USER_SWITCH_SPDBEGIN_HZ, USER_SWITCH_SPDEND_HZ);

    SPDCTRL_InitWrapper(&g_mc.spdCtrl, CTRL_SYSTICK_PERIOD);

    CURRCTRL_InitWrapper(&g_mc.currCtrl, &g_mc.idqRef, &g_mc.idqFbk, CTRL_CURR_PERIOD);

    FOSMO_InitWrapper(&g_mc.smo, CTRL_CURR_PERIOD);
    /* Total Current Sampling AD Bias Calibration Initialization */
    ADCCALIBR_Init(&g_mc.adcCalibrIbus);

    OTD_Init(&g_mc.otd, OTD_TEMP_THR, OTD_TIME_THR, CTRL_SYSTICK_PERIOD);
}

/**
  * @brief Initialzer of smo control struct handle.
  * @param fosmo first-order smo struct handle.
  * @param ts first-order ts.
  * @retval None.
  */
static void FOSMO_InitWrapperFan(FOSMO_Handle *fosmo, float ts)
{
    /* Configuring first-order smo Parameters. */
    FOSMO_Param fosmoParam = {
        .gain = FOSMO_GAIN_FAN,
        .lambda = FOSMO_LAMBDA_FAN,
        .fcEmf = FOSMO_EMF_CUTOFF_FREQ_FAN,
        .fcLpf = FOSMO_SPD_CUTOFF_FREQ_FAN,
        .pllBdw = FOSMO_PLL_BDW_FAN,
    };
    FOSMO_Init(fosmo, fosmoParam, g_motorParamFan, ts);
}

/**
  * @brief Initialzer of speed controller handle.
  * @param spdHandle speed controller struct handle.
  * @param ts speed controller ts.
  * @retval None.
  */
static void SPDCTRL_InitWrapperFan(SPDCTRL_Handle *spdHandle, float ts)
{
    /* Configuring speed Controller Parameters. */
    PI_Param spdPi = {
        .kp = SPD_KP_FAN,
        .ki = SPD_KI_FAN,
        .lowerLim = SPD_LOWERLIM_FAN,
        .upperLim = SPD_UPPERLIM_FAN,
    };
    SPDCTRL_Init(spdHandle, &g_motorParamFan, spdPi, ts);
}

/**
  * @brief Initialzer of current controller handle.
  * @param fosmo current controller struct handle.
  * @param idqRef DQ-axis current reference value.
  * @param idqFbk DQ-axis current feedback value.
  * @param ts current controller ts.
  * @retval None.
  */
static void CURRCTRL_InitWrapperFan(CURRCTRL_Handle *currHandle, DqAxis *idqRef, DqAxis *idqFbk, float ts)
{
    /* Configuring Current Controller Parameters. */
    PI_Param dqCurrPi = {
        .kp = CURR_KP_FAN,
        .ki = CURR_KI_FAN,
        .lowerLim = CURR_LOWERLIM_FAN,
        .upperLim = CURR_UPPERLIM_FAN,
    };
    CURRCTRL_Init(currHandle, &g_motorParamFan, idqRef, idqFbk, dqCurrPi, dqCurrPi, ts);
}

/**
  * @brief Init motor control task.
  * @retval None.
  */
static void TSK_InitFan(void)
{
    g_fan.stateMachine = FSM_IDLE;
    g_fan.spdCmd = USER_TARGET_SPD_HZ_FAN;
    g_fan.aptMaxcntCmp = APT_DUTY_MAX;
    g_fan.sampleMode = SINGLE_RESISTOR;
    g_fan.ts = CTRL_CURR_PERIOD; /* Init current controller */

    IF_Init(&g_fan.ifCtrl, CTRL_IF_CURR_AMP_A, USER_CURR_SLOPE, CTRL_SYSTICK_PERIOD, CTRL_CURR_PERIOD);
    /* Init speed slope */
    RMG_Init(&g_fan.spdRmg, CTRL_SYSTICK_PERIOD, USER_SPD_SLOPE);
    /* Init motor param */
    MtrParamInit(&g_fan.mtrParam, g_motorParamFan);

    TimerTickInit(&g_fan);

    SVPWM_Init(&g_fan.sv, INV_VOLTAGE_BUS * ONE_DIV_SQRT3);

    R1SVPWM_Init(&g_fan.r1Sv, INV_VOLTAGE_BUS * ONE_DIV_SQRT3, SAMPLE_POINT_SHIFT, SAMPLE_WINDOW_DUTY);

    STARTUP_Init(&g_fan.startup, USER_SWITCH_SPDBEGIN_HZ, USER_SWITCH_SPDEND_HZ);

    SPDCTRL_InitWrapperFan(&g_fan.spdCtrl, CTRL_SYSTICK_PERIOD);

    CURRCTRL_InitWrapperFan(&g_fan.currCtrl, &g_fan.idqRef, &g_fan.idqFbk, CTRL_CURR_PERIOD);

    FOSMO_InitWrapperFan(&g_fan.smo, CTRL_CURR_PERIOD);
    /* Total Current Sampling AD Bias Calibration Initialization */
    ADCCALIBR_Init(&g_fan.adcCalibrIbus);
}

/**
  * @brief Clear historical values of all controller before start-up.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void ClearBeforeStartup(MTRCTRL_Handle *mtrCtrl)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    /* The initial angle is 0. */
    mtrCtrl->axisAngle = 0;

    mtrCtrl->spdRef = 0.0f;
    /* The initial dq-axis reference current is 0. */
    mtrCtrl->idqRef.d = 0.0f;
    mtrCtrl->idqRef.q = 0.0f;

    mtrCtrl->vdqRef.d = 0.0f;
    mtrCtrl->vdqRef.q = 0.0f;
    /* Clear Duty Cycle Value. The initial duty cycle is 0.5. */
    mtrCtrl->dutyUvwLeft.u = 0.5f;
    mtrCtrl->dutyUvwLeft.v = 0.5f;
    mtrCtrl->dutyUvwLeft.w = 0.5f;
    mtrCtrl->dutyUvwRight.u = 0.5f;
    mtrCtrl->dutyUvwRight.v = 0.5f;
    mtrCtrl->dutyUvwRight.w = 0.5f;

    RMG_Clear(&mtrCtrl->spdRmg); /* Clear the history value of speed slope control */
    CURRCTRL_Clear(&mtrCtrl->currCtrl);
    IF_Clear(&mtrCtrl->ifCtrl);
    SPDCTRL_Clear(&mtrCtrl->spdCtrl);
    FOSMO_Clear(&mtrCtrl->smo);
    STARTUP_Clear(&mtrCtrl->startup);
    R1SVPWM_Clear(&mtrCtrl->r1Sv);
}

/**
  * @brief To set the comparison value of the IGBT single-resistance ADC sampling trigger position.
  * @param aptx The APT register struct handle.
  * @param cntCmpA A Count compare reference of time-base counter.
  * @param cntCmpB B Count compare reference of time-base counter.
  * @param maxCntCmp Maximum Comparison Value
  * @retval None.
  */
static void MCS_SetAdcCompareR1(APT_RegStruct *aptx, unsigned short cntCmpA,
                                unsigned short cntCmpB, unsigned short maxCntCmp)
{
    MCS_ASSERT_PARAM(aptx != NULL);
    MCS_ASSERT_PARAM(maxCntCmp != 0);
    unsigned short tmp;
    /* Sets the A Count compare reference of time-base counter. */
    tmp = (cntCmpA >= maxCntCmp) ? (maxCntCmp - 1) : cntCmpA;
    DCL_APT_SetCounterCompare(aptx, APT_COMPARE_REFERENCE_A, tmp);
    /* Sets the B Count compare reference of time-base counter. */
    tmp = (cntCmpB >= maxCntCmp) ? (maxCntCmp - 1) : cntCmpB;
    DCL_APT_SetCounterCompare(aptx, APT_COMPARE_REFERENCE_B, tmp);
}

/**
  * @brief Open the three-phase lower pipe.
  * @param aptAddr Three-phase APT address pointer.
  * @param maxDutyCnt Max duty count.
  * @retval None.
  */
static void AptTurnOnLowSidePwm(APT_RegStruct **aptAddr, unsigned int maxDutyCnt)
{
    unsigned int dutyCnt;
    dutyCnt = maxDutyCnt * APT_FULL_DUTY;
    /* Open the three-phase lower pipe */
    for (unsigned int i = 0; i < PHASE_MAX_NUM; i++) {
        APT_RegStruct *aptx = (APT_RegStruct *)(aptAddr[i]);
        DCL_APT_SetCounterCompare(aptx, APT_COMPARE_REFERENCE_C, dutyCnt);
        DCL_APT_SetCounterCompare(aptx, APT_COMPARE_REFERENCE_D, dutyCnt);
    }
}

/**
  * @brief Enable three-phase pwm output.
  * @param aptAddr Three-phase APT address pointer.
  * @retval None.
  */
static void MotorPwmOutputEnable(APT_RegStruct **aptAddr)
{
    for (unsigned int i = 0; i < PHASE_MAX_NUM; i++) {
        APT_RegStruct *aptx = (APT_RegStruct *)(aptAddr[i]);
        /* channel A/B force output action disable */
        DCL_APT_DisableSwContPWMAction(aptx, APT_PWM_CHANNEL_A);
        DCL_APT_DisableSwContPWMAction(aptx, APT_PWM_CHANNEL_B);
    }
}

/**
  * @brief Disable three-phase pwm output.
  * @param aptAddr Three-phase APT address pointer.
  * @retval None.
  */
static void MotorPwmOutputDisable(APT_RegStruct **aptAddr)
{
    /* Disable three-phase pwm output. */
    for (unsigned int i = 0; i < PHASE_MAX_NUM; i++) {
        APT_RegStruct *aptx = (APT_RegStruct *)(aptAddr[i]);
        DCL_APT_EnableSwContPWMAction(aptx, APT_PWM_CHANNEL_A);
        DCL_APT_EnableSwContPWMAction(aptx, APT_PWM_CHANNEL_B);
        DCL_APT_ForcePWMOutputLow(aptx);
    }
}

/**
  * @brief Construct a new mcs startupswitch object
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void MCS_StartupSwitch(MTRCTRL_Handle *mtrCtrl)
{
    STARTUP_Handle *startup = &mtrCtrl->startup;
    DqAxis *idqRef = &mtrCtrl->idqRef;
    float iftargetAmp = mtrCtrl->ifCtrl.targetAmp;
    float spdRef = mtrCtrl->spdRef;

    switch (startup->stage) {
        case STARTUP_STAGE_CURR:
            if (mtrCtrl->ifCtrl.curAmp >= iftargetAmp) {
                /* Stage change */
                idqRef->q = iftargetAmp;
                startup->stage = STARTUP_STAGE_SPD;
            } else {
                /* current Amplitude increase */
                idqRef->q = IF_CurrAmpCalc(&mtrCtrl->ifCtrl);
                spdRef = 0.0f;
            }
            break;
        case STARTUP_STAGE_SPD:
            /* current frequency increase */
            if (Abs(spdRef) >= startup->spdBegin) {
                /* Stage change */
                startup->stage = STARTUP_STAGE_SWITCH;
                TrigVal localTrigVal;
                TrigCalc(&localTrigVal, AngleSub(mtrCtrl->smo.elecAngle, mtrCtrl->ifCtrl.angle));
                idqRef->d = iftargetAmp * localTrigVal.sin;
                mtrCtrl->startup.initCurr = idqRef->d;
                idqRef->q = iftargetAmp;
                mtrCtrl->spdCtrl.spdPi.integral = iftargetAmp * localTrigVal.cos;
            } else {
                /* Speed rmg */
                spdRef = RMG_Exec(&mtrCtrl->spdRmg, mtrCtrl->spdCmd);
            }
            break;

        case STARTUP_STAGE_SWITCH:
            /* Switch from IF to SMO */
            spdRef = RMG_Exec(&mtrCtrl->spdRmg, mtrCtrl->spdCmd);
            idqRef->d = STARTUP_CurrCal(&mtrCtrl->startup, spdRef);
            idqRef->q = SPDCTRL_Exec(&mtrCtrl->spdCtrl, spdRef, mtrCtrl->spdFbk);
            if (spdRef >= startup->spdEnd) {
                /* Stage change */
                idqRef->d = 0.0f;
                mtrCtrl->stateMachine = FSM_RUN;
            }
            break;

        default:
            break;
    }

    mtrCtrl->spdRef = spdRef;
}

/**
  * @brief Pre-processing of motor status.
  * @param statusReg System status.
  * @param stateMachine Motor Control Status.
  * @retval None.
  */
static void MotorStatePerProc(SysStatusReg *statusReg, volatile FsmState *stateMachine)
{
    MCS_ASSERT_PARAM(statusReg != NULL);
    MCS_ASSERT_PARAM(stateMachine != NULL);
    /* Get system status */
    if (SysIsError(statusReg)) {
        *stateMachine = FSM_FAULT;
    }
    if (SysGetCmdStop(statusReg)) {
        SysCmdStopClr(statusReg);
        *stateMachine = FSM_STOP;
    }
}

/**
  * @brief Actiong for FSM_CAP_CHARGE status.
  * @param mtrCtrl The motor control handle.
  * @param stateMachine Motor Control Status.
  * @retval None.
  */
static void MCS_CAPChage(MTRCTRL_Handle *mtrCtrl, FsmState *stateMachine)
{
    mtrCtrl->sysTickCnt++;
    if (mtrCtrl->sysTickCnt == mtrCtrl->capChargeTickNum) {
        *stateMachine = FSM_CLEAR;
    }
}


/**
  * @brief Actiong for FSM_FAlT status.
  * @param statusReg System status.
  * @param stateMachine Motor Control Status.
  * @retval None.
  */
static void MCS_Fault(SysStatusReg *statusReg, FsmState *stateMachine)
{
    if (SysIsError(statusReg) == false) {
        *stateMachine = FSM_IDLE;
    }
}

/**
  * @brief Check systerm cmd start signal.
  * @param mtrCtrl The motor control handle.
  * @param aptAddr Three-phase APT address pointer.
  * @param statusReg System status.
  * @param stateMachine Motor Control Status.
  * @retval None.
  */
static void CheckSysCmdStart(MTRCTRL_Handle *mtrCtrl,
                             APT_RegStruct **aptAddr,
                             SysStatusReg *statusReg,
                             FsmState *stateMachine)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    MCS_ASSERT_PARAM(aptAddr != NULL);
    MCS_ASSERT_PARAM(statusReg != NULL);
    MCS_ASSERT_PARAM(stateMachine != NULL);
    /* check start cmd */
    if (SysGetCmdStart(statusReg)) {
        SysRunningSet(statusReg);
        SysCmdStartClr(statusReg);
        mtrCtrl->sysTickCnt = 0;
        *stateMachine = FSM_CAP_CHARGE;
        /* Preparation for charging the bootstrap capacitor. */
        AptTurnOnLowSidePwm(aptAddr, mtrCtrl->aptMaxcntCmp);
        /* Out put pwm */
        MotorPwmOutputEnable(aptAddr);
    }
}

/**
  * @brief Check that the total current sampling AD bias calibration is complete.
  * @param stateMachine Motor Control Status.
  * @retval None.
  */
static void AdcCalibrIsFinish(MTRCTRL_Handle *mtrCtrl, volatile FsmState *stateMachine)
{
    /* Calibrate ADC shift due to temperature */
    if (ADCCALIBR_IsFinish(&mtrCtrl->adcCalibrIbus)) {
        *stateMachine = FSM_STARTUP;
    }
}

/**
  * @brief System timer tick task.
  * @param mtrCtrl The motor control handle.
  * @param aptAddr Three-phase APT address pointer.
  * @retval None.
  */
static void TSK_SystickIsr(MTRCTRL_Handle *mtrCtrl, APT_RegStruct **aptAddr)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    MCS_ASSERT_PARAM(aptAddr != NULL);
    SysStatusReg *statusReg = &mtrCtrl->statusReg;
    FsmState *stateMachine = &mtrCtrl->stateMachine;
    mtrCtrl->msTickCnt++;
    /* Pre-processing of motor status. */
    MotorStatePerProc(statusReg, stateMachine);
    /* statemachine */
    switch (*stateMachine) {
        case FSM_IDLE:
            CheckSysCmdStart(mtrCtrl, aptAddr, statusReg, stateMachine);
            break;

        case FSM_CAP_CHARGE:
            /* Bootstrap Capacitor Charging Timing */
            MCS_CAPChage(mtrCtrl, stateMachine);
            break;

        case FSM_CLEAR:
            ClearBeforeStartup(mtrCtrl);
            *stateMachine = FSM_OFFSET_CALIB;
            break;

        case FSM_OFFSET_CALIB:
            /* Switch to FSM_STARTUP after AD offset calibration */
            AdcCalibrIsFinish(mtrCtrl, stateMachine);
            break;

        case FSM_STARTUP:
            MCS_StartupSwitch(mtrCtrl);
            break;

        case FSM_RUN:
            /* Speed ramp control */
            mtrCtrl->spdRef = RMG_Exec(&mtrCtrl->spdRmg, mtrCtrl->spdCmd);
            /* speed loop control */
            mtrCtrl->idqRef.q = SPDCTRL_Exec(&mtrCtrl->spdCtrl, mtrCtrl->spdRef, mtrCtrl->spdFbk);
            break;

        case FSM_STOP:
            MotorPwmOutputDisable(aptAddr);
            SysRunningClr(statusReg);
            *stateMachine = FSM_IDLE;
            break;

        case FSM_FAULT: /* Overcurrent state */
            MCS_Fault(statusReg, stateMachine);
            break;
        default:
            break;
    }
}

/**
  * @brief Get ADC result when ADC conversion completes.
  * @param adcHandle ADC handle.
  * @param soc Number of SOC, @ref ADC_SOCNumber.
  * @param adcCoeff Coefficient of ADC sample circuit.
  * @param adcCalibr ADC calibration value.
  * @retval ADC result.
  */
static float GetAdcResult(ADC_Handle *adcHandle, unsigned int soc, float adcCoeff, float adcCalibr)
{
    /* wait for ADC conversion complete */
    while (1) {
        /**
         * Check ADC conversion if completes.
         * The conversion will definitely be completed due to the MCU work principle.
         * So the while (1) loop will definitely break.
         */
        if (HAL_ADC_CheckSocFinish(adcHandle, soc) == BASE_STATUS_OK) {
            break;
        }
    }

    /* ADC result = (calibration_value - SOCxResult) * hardware_sampling_coefficient */
    return (float)((float)(DCL_ADC_ReadSOCxResult(adcHandle->baseAddress, soc)) - adcCalibr) * adcCoeff;
}

/**
  * @brief read bus current bias calibration callback function.
  * @param IBIAS_Handle Adc calibration struct.
  * @retval None.
  */
static void readCurrBiasCpCb(IBIAS_Handle *adcCalibrCurrUvw)
{
    MCS_ASSERT_PARAM(adcCalibrCurrUvw != NULL);
    adcCalibrCurrUvw->iBusAdcCalibr = ADCCALIBR_Exec(&g_mc.adcCalibrIbus, &g_adc2, ADC_SOC_NUM8);
}

/**
  * @brief Read the ADC current sampling value of the compressor.
  * @param CurrUvw Three-phase current.
  * @retval None.
  */
static void ReadCurrUvwCp(UvwAxis *iuvw, IBIAS_Handle *adcCalibr)
{
    MCS_ASSERT_PARAM(iuvw != NULL);
    MCS_ASSERT_PARAM(adcCalibr != NULL);
    float iBusSocA, iBusSocB;

    /* Zero sampling value of hardware circuit is iBusAdcCalibr. */
    iBusSocA = GetAdcResult(&g_adc2, ADC_SOC_NUM8, ADC_CURR_COFFI_CP, adcCalibr->iBusAdcCalibr);
    iBusSocB = GetAdcResult(&g_adc2, ADC_SOC_NUM9, ADC_CURR_COFFI_CP, adcCalibr->iBusAdcCalibr);
    R1CurrReconstruct(g_mc.r1Sv.voltIndexLast, iBusSocA, iBusSocB, iuvw);
}

/**
  * @brief Setting the APT Output Duty Cycle.
  * @param aptx APT register base address.
  * @param leftDuty Left duty cycle.
  * @param rightDuty Right duty cycle.
  * @retval None.
  */
static void SetPwmDuty(APT_Handle *aptx, float leftDuty, float rightDuty)
{
    MCS_ASSERT_PARAM(aptx != NULL);
    unsigned short maxPeriodCnt = aptx->waveform.timerPeriod;
    unsigned short cntCmpLeftEdge = (unsigned short)(leftDuty * maxPeriodCnt);
    unsigned short cntCmpRightEdge = (unsigned short)(rightDuty * maxPeriodCnt);
    /* avoid overflowing */
    cntCmpLeftEdge = (cntCmpLeftEdge > maxPeriodCnt) ? maxPeriodCnt : cntCmpLeftEdge;
    cntCmpRightEdge = (cntCmpRightEdge > maxPeriodCnt) ? maxPeriodCnt : cntCmpRightEdge;
    HAL_APT_SetPWMDuty(aptx, cntCmpLeftEdge, cntCmpRightEdge);
}

/**
  * @brief Compressor Duty Cycle Setting.
  * @param dutyUvwLeft Three-phase left duty cycle.
  * @param dutyUvwRight Three-phase right duty cycle.
  * @retval None.
  */
static void SetPwmDutyCp(UvwAxis *dutyUvwLeft, UvwAxis *dutyUvwRight)
{
    MCS_ASSERT_PARAM(dutyUvwLeft != NULL);
    MCS_ASSERT_PARAM(dutyUvwRight != NULL);
    /* Setting the Three-Phase Duty Cycle */
    SetPwmDuty(&g_apt3, dutyUvwLeft->u, dutyUvwRight->u);
    SetPwmDuty(&g_apt4, dutyUvwLeft->v, dutyUvwRight->v);
    SetPwmDuty(&g_apt5, dutyUvwLeft->w, dutyUvwRight->w);
}

/**
  * @brief To set the ADC sampling trigger comparison value.
  * @param cntCmpSOCA Soca Compare Count Value.
  * @param cntCmpSOCB Socb Compare Count Value.
  * @retval None.
  */
static void SetADCTriggerTimeCp(unsigned short cntCmpSOCA, unsigned short cntCmpSOCB)
{
    MCS_SetAdcCompareR1(APT_U_CP, cntCmpSOCA, cntCmpSOCB, g_mc.aptMaxcntCmp);
}

/**
  * @brief read bus current bias calibration callback function.
  * @param IBIAS_Handle Adc calibration struct.
  * @retval None.
  */
static void readCurrBiasFanCb(IBIAS_Handle *adcCalibrCurrUvw)
{
    MCS_ASSERT_PARAM(adcCalibrCurrUvw != NULL);
    adcCalibrCurrUvw->iBusAdcCalibr = ADCCALIBR_Exec(&g_fan.adcCalibrIbus, &g_adc0, ADC_SOC_NUM0);
}

/**
  * @brief Obtaining the Three-Phase Current of the Fan.
  * @param iuvw Three-phase current data return pointer.
  * @param IBIAS_Handle Adc bias struct.
  * @retval None.
  */
static void ReadCurrUvwFanCb(UvwAxis *iuvw, IBIAS_Handle *adcCalibr)
{
    MCS_ASSERT_PARAM(iuvw != NULL);
    MCS_ASSERT_PARAM(adcCalibr != NULL);
    float iBusSocA, iBusSocB;

    iBusSocA = GetAdcResult(&g_adc0, ADC_SOC_NUM0, ADC_CURR_COFFI_FAN, adcCalibr->iBusAdcCalibr);
    iBusSocB = GetAdcResult(&g_adc0, ADC_SOC_NUM1, ADC_CURR_COFFI_FAN, adcCalibr->iBusAdcCalibr);
    /* reconstructed three-phase current */
    R1CurrReconstruct(g_fan.r1Sv.voltIndexLast, iBusSocA, iBusSocB, iuvw);
}

/**
  * @brief Fan Duty Cycle Setting.
  * @param dutyUvwLeft Three-phase left duty cycle.
  * @param dutyUvwRight Three-phase right duty cycle.
  * @retval None.
  */
static void SetPwmDutyFanCb(UvwAxis *dutyUvwLeft, UvwAxis *dutyUvwRight)
{
    MCS_ASSERT_PARAM(dutyUvwLeft != NULL);
    MCS_ASSERT_PARAM(dutyUvwRight != NULL);
    /* Setting the Three-Phase Duty Cycle */
    SetPwmDuty(&g_apt0, dutyUvwLeft->u, dutyUvwRight->u);
    SetPwmDuty(&g_apt1, dutyUvwLeft->v, dutyUvwRight->v);
    SetPwmDuty(&g_apt2, dutyUvwLeft->w, dutyUvwRight->w);
}

/**
  * @brief To set the ADC sampling trigger comparison value.
  * @param cntCmpSOCA Soca Compare Count Value.
  * @param cntCmpSOCB Socb Compare Count Value.
  * @retval None.
  */
static void SetADCTriggerTimeFanCb(unsigned short cntCmpSOCA, unsigned short cntCmpSOCB)
{
    MCS_SetAdcCompareR1((APT_RegStruct *)g_aptFan[PHASE_U], cntCmpSOCA, cntCmpSOCB, g_fan.aptMaxcntCmp);
}

/**
  * @brief Event interrupt callback function of APT module.
  * @param para APT module handle.
  * @retval None.
  */
void APT0EventCallback(void *aptHandle)
{
    MCS_ASSERT_PARAM(aptHandle != NULL);
    APT_Handle *handle = (APT_Handle *)aptHandle;
    /* The fan IPM overcurrent triggers and disables the three-phase PWM output. */
    MotorPwmOutputDisable(g_aptFan);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptFan[PHASE_U], APT_OC_COMBINE_EVENT_A1);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptFan[PHASE_V], APT_OC_COMBINE_EVENT_A1);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptFan[PHASE_W], APT_OC_COMBINE_EVENT_A1);
    /* Status setting error */
    SysErrorSet(&g_fan.statusReg);
    DBG_PRINTF("APT_EVT_IRQ_FAN\r\n");
    BASE_FUNC_UNUSED(handle);
}

/**
  * @brief Event interrupt callback function of APT module.
  * @param para APT module handle.
  * @retval None.
  */
void APT3EventCallback(void *aptHandle)
{
    MCS_ASSERT_PARAM(aptHandle != NULL);
    APT_Handle *handle = (APT_Handle *)aptHandle;
    /* The compressor IPM overcurrent triggers and disables the three-phase PWM output. */
    MotorPwmOutputDisable(g_aptCp);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptCp[PHASE_U], APT_OC_COMBINE_EVENT_A1);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptCp[PHASE_V], APT_OC_COMBINE_EVENT_A1);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptCp[PHASE_W], APT_OC_COMBINE_EVENT_A1);
    /* Status setting error */
    SysErrorSet(&g_mc.statusReg);
    DBG_PRINTF("APT_EVT_IRQ_CP\r\n");
    BASE_FUNC_UNUSED(handle);
}

/**
  * @brief Init motor controller's data structure.
  * @retval None.
  */
static void InitSoftware(void)
{
    /* Initializing Compressor and Fan Control Tasks */
    TSK_InitCp();
    TSK_InitFan();

    /* MCU peripheral configuration function used for initial motor control */
    g_mc.readCurrUvwCb = ReadCurrUvwCp;
    g_mc.setPwmDutyCb = SetPwmDutyCp;
    g_mc.setADCTriggerTimeCb = SetADCTriggerTimeCp;
    g_mc.readCurrBiasCb = readCurrBiasCpCb;

    g_fan.readCurrUvwCb = ReadCurrUvwFanCb;
    g_fan.setPwmDutyCb = SetPwmDutyFanCb;
    g_fan.setADCTriggerTimeCb = SetADCTriggerTimeFanCb;
    g_fan.readCurrBiasCb = readCurrBiasFanCb;
}

/**
  * @brief Disable PWM output when IPM temperature too high.
  * @retval None.
  */
static void OverTempProtProcess(void)
{
    float tempV;

    HAL_ADC_SoftTrigSample(&g_adc2, ADC_SOC_NUM7);
    tempV = HAL_ADC_GetConvResult(&g_adc2, ADC_SOC_NUM7) * IPM_VOLT_COEFFI;
    bool overTempFlag;
    overTempFlag = OTD_Exec(&g_mc.otd, tempV);
    /* The PWM output is disabled when the voltage corresponding to the temperature exceeds the preset threshold. */
    if (overTempFlag) {
        SysErrorSet(&g_mc.statusReg);
        MotorPwmOutputDisable(g_aptCp);
        DBG_PRINTF("OverTemp\r\n");
    }
}

/**
  * @brief System timer ISR function.
  * @param param The systick timer handle.
  * @retval None.
  */
void Timer1ITCallBack(void *param)
{
    MCS_ASSERT_PARAM(param != NULL);
    BASE_FUNC_UNUSED(param);
    /* Motor speed loop execution and state control. */
    TSK_SystickIsr(&g_mc, g_aptCp);
    TSK_SystickIsr(&g_fan, g_aptFan);
    /* Temperature protection process. */
    OverTempProtProcess();
}

/* global variables for variable trace */
volatile float g_mc_u, g_mc_v, g_mc_w;
volatile float g_fan_u, g_fan_v, g_fan_w;
volatile float g_currLoopExeTime;
/**
  * @brief The carrier ISR wrapper function, entry for both compressor and fan.
  * @param aptHandle The APT handle.
  * @retval None.
  */
void APT3TimerCallback(void *aptHandle)
{
    int start = SYSTICK_GetTimeStampUs();
    MCS_ASSERT_PARAM(aptHandle != NULL);
    /* the carrierprocess of comp */
    MCS_CarrierProcess(&g_mc);
    g_mc_u = g_mc.iuvw.u;
    g_mc_v = g_mc.iuvw.v;
    g_mc_w = g_mc.iuvw.w;
    
    /* the carrierprocess of fan */
    MCS_CarrierProcess(&g_fan);
    g_fan_u = g_fan.iuvw.u;
    g_fan_v = g_fan.iuvw.v;
    g_fan_w = g_fan.iuvw.w;
    BASE_FUNC_UNUSED(aptHandle);
    g_currLoopExeTime = SYSTICK_GetTimeStampUs() - start;
}

/**
  * @brief User application entry.
  * @retval BSP_OK.
  */
int MotorMain(void)
{
    unsigned int tickNum500Ms = 1000; /* 500ms tick */
    static unsigned int tickCnt500Ms = 0;

    SystemInit();

    HAL_TIMER_Start(&g_timer1);

    /* Disable PWM output before startup. */
    MotorPwmOutputDisable(g_aptCp);
    MotorPwmOutputDisable(g_aptFan);

    /* Software initialization. */
    InitSoftware();
    /* Start the PWM clock. */
    HAL_APT_StartModule(RUN_APT0 | RUN_APT1 | RUN_APT2 | RUN_APT3 | RUN_APT4 | RUN_APT5);

    BASE_FUNC_DELAY_S(PTC_RELAY_DELAY);
    /* Open PTC relay */
    HAL_GPIO_SetValue(&g_gpio2, GPIO_PIN_0, GPIO_HIGH_LEVEL);

    BASE_FUNC_DELAY_S(MOTOR_START_DELAY);
    /* Starting motor. */
    SysCmdStartSet(&g_mc.statusReg);
    SysCmdStartSet(&g_fan.statusReg);

    while (1) {
        if (g_mc.msTickCnt - tickCnt500Ms >= tickNum500Ms) {
            if (SysIsError(&g_mc.statusReg) != true && SysIsError(&g_fan.statusReg) != true) {
               /* The LED blinks when no status is not error. */
                HAL_GPIO_TogglePin(&g_gpio0, GPIO_PIN_5);
            } else {
                HAL_GPIO_SetValue(&g_gpio0, GPIO_PIN_5, GPIO_LOW_LEVEL);
            }
            tickCnt500Ms = g_mc.msTickCnt;
        }
    }
    return 0;
}