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
  * @brief     This file provides motor application for AD101LDMA board.
  * @details   Single FOC application based on the AD101LDMA VER.A board
  *            1) The motor model is JMC 42JSF630AS-1000.
  *            2) Select the Motorcontrolsystem example in the sample column of chipConfig and click Generate Code.
  *            3) The AD101LDMA VER.A board is a high-voltage board, its power supply must be changed to 24V.
  */
#include "mcs_motor_process.h"
#include "mcs_user_config.h"
#include "mcs_math_const.h"
#include "mcs_math.h"
#include "mcs_carrier.h"

#define APT_FULL_DUTY 1.0f

void ISR_Systick(void *param);
void ISR_Carrier(void *aptHandle);
void ISR_OverCurrProt(void *aptHandle);

/* Motor parameters */
/* Np, Rs, Ld, Lq, Psif, J, Nmax, Currmax, PPMR, zShift */
static MOTOR_Param g_motorParam = {
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

/* Motor control handle for compressor */
static MTRCTRL_Handle g_mc;

/* ADC calibration. */
static ADC_CALIBR_Handle g_adcCalibrIbus;

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
    FOSMO_Init(fosmo, fosmoParam, g_motorParam, ts);
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
    SPDCTRL_Init(spdHandle, &g_motorParam, spdPi, ts);
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
    CURRCTRL_Init(currHandle, &g_motorParam, idqRef, idqFbk, dqCurrPi, dqCurrPi, ts);
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
    MtrParamInit(&g_mc.mtrParam, g_motorParam);

    TimerTickInit(&g_mc);

    SVPWM_Init(&g_mc.sv, INV_VOLTAGE_BUS * ONE_DIV_SQRT3);

    R1SVPWM_Init(&g_mc.r1Sv, INV_VOLTAGE_BUS * ONE_DIV_SQRT3, SAMPLE_POINT_SHIFT, SAMPLE_WINDOW_DUTY);

    STARTUP_Init(&g_mc.startup, USER_SWITCH_SPDBEGIN_HZ, USER_SWITCH_SPDEND_HZ);

    SPDCTRL_InitWrapper(&g_mc.spdCtrl, CTRL_SYSTICK_PERIOD);

    CURRCTRL_InitWrapper(&g_mc.currCtrl, &g_mc.idqRef, &g_mc.idqFbk, CTRL_CURR_PERIOD);

    FOSMO_InitWrapper(&g_mc.smo, CTRL_CURR_PERIOD);
    /* Total Current Sampling AD Bias Calibration Initialization */
    ADCCALIBR_Init(&g_adcCalibrIbus);
}

/**
  * @brief Clear historical values of all controller before start-up.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void ClearBeforeStartup(MTRCTRL_Handle *mtrCtrl)
{
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
  * @brief To set the comparison value of the IGBT single-resistance ADC sampling trigger position
  * @param aptx The APT register struct handle.
  * @param cntCmpA A Count compare reference of time-base counter.
  * @param cntCmpB B Count compare reference of time-base counter.
  * @param maxCntCmp Maximum Comparison Value
  * @retval None.
  */
static void MCS_SetAdcCompareR1(APT_RegStruct *aptx, unsigned short cntCmpA,
                                unsigned short cntCmpB, unsigned short maxCntCmp)
{
    unsigned short tmp;
    /* Sets the A Count compare reference of time-base counter. */
    tmp = (cntCmpA >= maxCntCmp) ? (maxCntCmp - 1) : cntCmpA;
    DCL_APT_SetCounterCompare(aptx, APT_COMPARE_REFERENCE_A, tmp);
    /* Sets the B Count compare reference of time-base counter. */
    tmp = (cntCmpB >= maxCntCmp) ? (maxCntCmp - 1) : cntCmpB;
    DCL_APT_SetCounterCompare(aptx, APT_COMPARE_REFERENCE_B, tmp);
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
    unsigned short maxPeriodCnt = aptx->waveform.timerPeriod;
    unsigned short cntCmpLeftEdge = (unsigned short)(leftDuty * maxPeriodCnt);
    unsigned short cntCmpRightEdge = (unsigned short)(rightDuty * maxPeriodCnt);
    /* avoid overflowing */
    cntCmpLeftEdge = (cntCmpLeftEdge > maxPeriodCnt) ? maxPeriodCnt : cntCmpLeftEdge;
    cntCmpRightEdge = (cntCmpRightEdge > maxPeriodCnt) ? maxPeriodCnt : cntCmpRightEdge;
    HAL_APT_SetPWMDuty(aptx, cntCmpLeftEdge, cntCmpRightEdge);
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
static void AdcCalibrIsFinish(volatile FsmState *stateMachine)
{
    /* Calibrate ADC shift due to temperature */
    if (ADCCALIBR_IsFinish(&g_adcCalibrIbus)) {
        *stateMachine = FSM_STARTUP;
    }
}

/**
  * @brief System timer tick task.+
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
    /* Pre-processing of motor status */
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
            AdcCalibrIsFinish(stateMachine);
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
  * @brief Three-phase current bias calibration.
  * @param IBIAS_Handle Adc calibration struct.
  * @retval None.
  */
static void readCurrBiasCb(IBIAS_Handle *adcCalibrCurrUvw)
{
    MCS_ASSERT_PARAM(adcCalibrCurrUvw != NULL);
    adcCalibrCurrUvw->iBusAdcCalibr = ADCCALIBR_Exec(&g_adcCalibrIbus, &g_adc, ADC_SOC_NUM8);
}

/**
  * @brief Read the ADC current sampling value.
  * @param iuvw Three-phase current.
  * @param IBIAS_Handle Adc bias struct.
  * @retval None.
  */
static void ReadCurrUvwCb(UvwAxis *iuvw, IBIAS_Handle *adcCalibr)
{
    float iBusSocA, iBusSocB;
    iBusSocA = GetAdcResult(&g_adc, ADC_SOC_NUM8, ADC_CURR_COFFI_CP, adcCalibr->iBusAdcCalibr);
    iBusSocB = GetAdcResult(&g_adc, ADC_SOC_NUM9, ADC_CURR_COFFI_CP, adcCalibr->iBusAdcCalibr);
    R1CurrReconstruct(g_mc.r1Sv.voltIndexLast, iBusSocA, iBusSocB, iuvw);
}

/**
  * @brief Compressor Duty Cycle Setting.
  * @param dutyUvwLeft Three-phase left duty cycle.
  * @param dutyUvwRight Three-phase right duty cycle.
  * @retval None.
  */
static void SetPwmDutyCb(UvwAxis *dutyUvwLeft, UvwAxis *dutyUvwRight)
{
    SetPwmDuty(&g_aptUcp, dutyUvwLeft->u, dutyUvwRight->u);
    SetPwmDuty(&g_aptVcp, dutyUvwLeft->v, dutyUvwRight->v);
    SetPwmDuty(&g_aptWcp, dutyUvwLeft->w, dutyUvwRight->w);
}

/**
  * @brief To set the ADC sampling trigger comparison value.
  * @param cntCmpSOCA Soca Compare Count Value.
  * @param cntCmpSOCB Socb Compare Count Value.
  * @retval None.
  */
static void SetADCTriggerTimeCb(unsigned short cntCmpSOCA, unsigned short cntCmpSOCB)
{
    MCS_SetAdcCompareR1(APT_U_CP, cntCmpSOCA, cntCmpSOCB, g_mc.aptMaxcntCmp);
}

/**
  * @brief Init motor controller's data structure.
  * @retval None.
  */
static void InitSoftware(void)
{
    /* Initializing Compressor and Fan Control Tasks */
    TSK_InitCp();
    /* MCU peripheral configuration function used for initial motor control */
    g_mc.readCurrUvwCb = ReadCurrUvwCb;
    g_mc.setPwmDutyCb = SetPwmDutyCb;
    g_mc.setADCTriggerTimeCb = SetADCTriggerTimeCb;
    g_mc.readCurrBiasCb = readCurrBiasCb;
}

/**
  * @brief Event interrupt callback function of APT module.
  * @param aptHandle APT module handle.
  * @retval None.
  */
void ISR_OverCurrProt(void *aptHandle)
{
    /* The IPM overcurrent triggers and disables the three-phase PWM output. */
    MotorPwmOutputDisable(g_aptCp);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptCp[PHASE_U], APT_OC_COMBINE_EVENT_A1);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptCp[PHASE_V], APT_OC_COMBINE_EVENT_A1);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptCp[PHASE_W], APT_OC_COMBINE_EVENT_A1);
    /* Status setting error */
    SysErrorSet(&g_mc.statusReg);
    DBG_PRINTF("APT_EVT_IRQ\r\n");
    HAL_GPIO_TogglePin(&g_Led, GPIO_PIN_6);
    BASE_FUNC_UNUSED(aptHandle);
}

/**
  * @brief System timer ISR function.
  * @param param The systick timer handle.
  * @retval None.
  */
void ISR_Systick(void *param)
{
    TSK_SystickIsr(&g_mc, g_aptCp);
    BASE_FUNC_UNUSED(param);
}

/**
  * @brief Timer interrupt callback function of U phase APT module.
  * @param aptHandle APT module handle.
  * @retval None.
  */
void ISR_Carrier(void *aptHandle)
{
    /* the carrierprocess */
    MCS_CarrierProcess(&g_mc);
    BASE_FUNC_UNUSED(aptHandle);
}

/**
  * @brief User application entry.
  * @retval BSP_OK.
  */
int MotorMain(void)
{
    SystemInit();
    HAL_TIMER_Start(&g_sysTickTimer);
    /* Disable PWM output before startup. */
    MotorPwmOutputDisable(g_aptCp);
    /* Software initialization. */
    InitSoftware();
    /* Start the PWM clock. */
    HAL_APT_StartModule(RUN_APT0 | RUN_APT1 | RUN_APT2);

    unsigned int tickNum_500ms = 1000; /* 500ms tick */
    static unsigned int tickCnt_500ms = 0;

    BASE_FUNC_DELAY_S(MOTOR_START_DELAY);
    
    SysCmdStartSet(&g_mc.statusReg); /* start motor */

    while (1) {
        if (g_mc.msTickCnt - tickCnt_500ms >= tickNum_500ms) {
            if (SysIsError(&g_mc.statusReg) != true) {
                /* The LED blinks when no status is not error. */
                HAL_GPIO_TogglePin(&g_Led, G_LED_PIN);
            } else {
                HAL_GPIO_SetValue(&g_Led, G_LED_PIN, GPIO_LOW_LEVEL);
            }
            tickCnt_500ms = g_mc.msTickCnt;
        }
    }
    return 0;
}