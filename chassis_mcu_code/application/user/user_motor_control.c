#include "main.h"
#include "debug.h"
#include "user_motor_control.h"

unsigned int g_aptDutyCur = 50;
unsigned int g_aptDutySet = 50;
unsigned int g_aptFreqCntCur =1500;
unsigned int g_aptFreqCntSet =1500;
unsigned int g_rotationDirectionStatus = 0;
unsigned int g_rotationDirectionStatusSet = 0;

unsigned int g_testCount = 0;
unsigned int g_testNum = 0;

void User_APTForcePWMAOutputLow(APT_RegStruct *aptx)
{
    /* Enable force output. */


    aptx->PG_OUT_FRC.BIT.rg_pga_frc_act = APT_PWM_CONTINUOUS_ACTION_LOW; /* if not invert, set low */
    aptx->PG_OUT_FRC.BIT.rg_pga_frc_en = BASE_CFG_ENABLE;
    /* if PWMA invert */
    return;
}

void User_APTForcePWMBOutputLow(APT_RegStruct *aptx)
{
    aptx->PG_OUT_FRC.BIT.rg_pgb_frc_act = APT_PWM_CONTINUOUS_ACTION_HIGH; /* if not invert, set low */
   /* Enable force output. */
    aptx->PG_OUT_FRC.BIT.rg_pgb_frc_en = BASE_CFG_ENABLE;
    return;
}


void User_APTPwmARecovery(APT_RegStruct *aptx)
{
    /* Enable force output. */
    aptx->PG_OUT_FRC.BIT.rg_pga_frc_en = BASE_CFG_DISABLE;
    return;
}

void User_APTPwmBRecovery(APT_RegStruct *aptx)
{
    /* Enable force output. */
    aptx->PG_OUT_FRC.BIT.rg_pgb_frc_en = BASE_CFG_DISABLE;
    return;
}

/* 电机启动 */
void startMotor(void)
{
    HAL_APT_StartModule(RUN_APT0|RUN_APT1);
}

/* 电机停止 */
void stopMotor(void)
{
    HAL_APT_StopModule(RUN_APT0|RUN_APT1);
}

/* 电机正转 */
void MotorForwardRotation(void)
{
    User_APTPwmARecovery(g_apt0.baseAddress); // 左转，右轮子前进  
    User_APTForcePWMBOutputLow(g_apt0.baseAddress);
    User_APTForcePWMAOutputLow(g_apt1.baseAddress); // 右转，左轮子前进
    User_APTPwmBRecovery(g_apt1.baseAddress);
}

/* 电机反转 */
void MotorReverse(void)
{
    User_APTForcePWMAOutputLow(g_apt0.baseAddress);
    User_APTPwmBRecovery(g_apt0.baseAddress);

    User_APTPwmARecovery(g_apt1.baseAddress);
    User_APTForcePWMBOutputLow(g_apt1.baseAddress);
}

/* 电机左转 */
void MotorLeft(void)
{
    User_APTPwmARecovery(g_apt0.baseAddress); // 左转，右轮子前进
    User_APTForcePWMBOutputLow(g_apt0.baseAddress);
    HAL_APT_StopModule(RUN_APT1);
}

/* 电机右转 */
void MotorRight(void)
{
    HAL_APT_StopModule(RUN_APT0);
    User_APTForcePWMAOutputLow(g_apt1.baseAddress); // 右转，左轮子前进
    User_APTPwmBRecovery(g_apt1.baseAddress);
}

/* 通过占空比调节电机速度 */
void MotorSpeedAdjustmentByDuty(unsigned int duty, unsigned int directions)
{
    /* 正转 */
    if (directions == 0) {
        HAL_APT_SetPWMDutyByNumber(&g_apt0, duty);
        HAL_APT_SetPWMDutyByNumber(&g_apt1, duty);
    } else {
        /* 反转 */
        HAL_APT_SetPWMDutyByNumber(&g_apt0, (100 - duty));
        HAL_APT_SetPWMDutyByNumber(&g_apt1, (100 - duty));
    }
}

/* 通过修改频率调节电机速度 */
void MotorSpeedAdjustmentByFreq(unsigned int freq)
{
    DCL_APT_SetTimeBasePeriod(g_apt0.baseAddress, freq);
    DCL_APT_SetTimeBasePeriod(g_apt1.baseAddress, freq);
}

void initMotor()
{
    MotorSpeedAdjustmentByDuty(g_aptDutyCur, g_rotationDirectionStatus); 
}

void MotorProcess()
{

    //MotorSpeedAdjustmentByFreq(g_aptFreqCntCur);
    // HAL_APT_StartModule(RUN_APT0|RUN_APT1);
    
    // while (1) {
        /* 改变转动方向 */
        g_testCount ++;

        /*if (g_rotationDirectionStatus != g_rotationDirectionStatusSet) {
            g_rotationDirectionStatus = g_rotationDirectionStatusSet;
            if (g_rotationDirectionStatus == 0) {    
                MotorForwardRotation();  //电机正转 
            } else {
                MotorReverse();    //电机反转
            } 
        } */

        if (g_testCount > 200) {
            if (g_rotationDirectionStatus == 1)
            {
                MotorForwardRotation();   /* 电机正转 */ 
                g_rotationDirectionStatus = 0;
            }
            else
            {
                MotorReverse();   /* 电机反转 */
                g_rotationDirectionStatus = 1;
            }
            
            DBG_PRINTF("Test Num = %d\r\n",g_testNum++);
            g_testCount =0;
        }    


        /* 改变速度 */
        if (g_aptDutyCur != g_aptDutySet) {
            g_aptDutyCur = g_aptDutySet;
            MotorSpeedAdjustmentByDuty(g_aptDutyCur, g_rotationDirectionStatus);
        }

        if (g_aptFreqCntCur != g_aptFreqCntSet) {
            g_aptFreqCntCur = g_aptFreqCntSet;
            MotorSpeedAdjustmentByFreq(g_aptFreqCntCur);
        }
    // }

}