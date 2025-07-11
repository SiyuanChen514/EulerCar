#include "qdm.h"
#include "debug.h"
#include "encoder.h"
#include "main.h"
#include "eulercar_control.h"


//系统10ms中断计数器
extern unsigned int g_TimerInterruptCount;

/**
  * @brief QDM sample use M method
  * @param none.
  * @retval none.
  */

//用于pid算法的速度，即MOTOR_PID_CONTROL_PERIOD周期内的速度
Gear_Motor_handle pidRightMotor;
Gear_Motor_handle pidLeftMotor;

//用于ODOM里程计的平均速度，即EULER_CAR_DATA_SEND_PERIOD周期内的速度
Gear_Motor_handle aveRightMotor;
Gear_Motor_handle aveLeftMotor;

//编码器读取周期内最大差值
unsigned int g_maxEncodePidDeltaValue;
unsigned int g_maxEncodeAveDeltaValue;

void InitGearMotor(void)
{
    pidRightMotor.motorSide  = MOTOR_RIGHT;    
    pidRightMotor.curNumber = 0;            /* 当前计数器 */
    pidRightMotor.lastNumber = 0;           /* 上一次计数器 */
    pidRightMotor.speedRps = 0;             /* 电机转速，单位每秒多少圈 */
    pidRightMotor.speed = 0;                /* 电机速度，单位mm/s，由转速和轮胎直径计算 */
    pidRightMotor.calPeriod = MOTOR_PID_CONTROL_PERIOD; 

    pidLeftMotor.motorSide  = MOTOR_LEFT;    
    pidLeftMotor.curNumber = 0;             /* 当前计数器 */
    pidLeftMotor.lastNumber = 0;            /* 上一次计数器 */
    pidLeftMotor.speedRps = 0;              /* 电机转速，单位每秒多少圈 */    
    pidLeftMotor.speed = 0;                 /* 电机速度，单位mm/s，由转速和轮胎直径计算 */
    pidLeftMotor.calPeriod = MOTOR_PID_CONTROL_PERIOD;    


    aveRightMotor.motorSide  = MOTOR_RIGHT;    
    aveRightMotor.curNumber = 0;            /* 当前计数器 */
    aveRightMotor.lastNumber = 0;           /* 上一次计数器 */
    aveRightMotor.speedRps = 0;             /* 电机转速，单位每秒多少圈 */
    aveRightMotor.speed = 0;                /* 电机速度，单位mm/s，由转速和轮胎直径计算 */
    aveRightMotor.calPeriod = EULER_CAR_DATA_SEND_PERIOD; 

    aveLeftMotor.motorSide  = MOTOR_LEFT;    
    aveLeftMotor.curNumber = 0;             /* 当前计数器 */
    aveLeftMotor.lastNumber = 0;            /* 上一次计数器 */
    aveLeftMotor.speedRps = 0;              /* 电机转速，单位每秒多少圈 */    
    aveLeftMotor.speed = 0;                 /* 电机速度，单位mm/s，由转速和轮胎直径计算 */
    aveLeftMotor.calPeriod = EULER_CAR_DATA_SEND_PERIOD;    


    //编码器在一个PID周期内最大差值，电机转速的2倍，主要用于容错处理
    g_maxEncodePidDeltaValue = (int)(g_motorMaxSpeed / MOTOR_TIRE_DIAMETER / PI * g_motorLineNum / (1000.0/MOTOR_PID_CONTROL_PERIOD) * 2.0);
    //编码器在一个底盘数据上报周期内最大差值，电机转速的2倍，主要用于容错处理
    g_maxEncodeAveDeltaValue = (int)(g_motorMaxSpeed / MOTOR_TIRE_DIAMETER / PI * g_motorLineNum / (1000.0/EULER_CAR_DATA_SEND_PERIOD) * 2.0);

    //DBG_PRINTF("g_maxEncodeDeltaValue = %d\r\n", g_maxEncodeDeltaValue);
}



void QDM_CalMotorSpeed(Gear_Motor_handle *pMotor)
{
    unsigned int qdm_cnt, qdm_dir;
    unsigned int deltaValue;
    if(pMotor->motorSide == MOTOR_RIGHT )
        HAL_QDM_ReadPosCountAndDir(&g_qdm0, &qdm_cnt, &qdm_dir);
    else    
        HAL_QDM_ReadPosCountAndDir(&g_qdm1, &qdm_cnt, &qdm_dir);
    
    /* 计算车轮速度 */
    pMotor->curNumber = qdm_cnt;
    if(qdm_dir == 1){ //电机正转
        if(pMotor->curNumber >= pMotor->lastNumber) {
             deltaValue = pMotor->curNumber - pMotor->lastNumber;
        }else{ //过零点后的计算编码器差值 
             deltaValue = (pMotor->curNumber +g_motorLineNum) - pMotor->lastNumber; 
        }
    }else{  //电机反转
        if(pMotor->curNumber <= pMotor->lastNumber) {
             deltaValue = pMotor->lastNumber - pMotor->curNumber;
        }else{ //过零点后的计算编码器差值 
             deltaValue = (pMotor->lastNumber + g_motorLineNum) - pMotor->curNumber ; 
        }
    }

    /* 电机正反转切换时，会出现编码器差值计算错误，设置编码器差值为0 */
    if(pMotor->calPeriod == MOTOR_PID_CONTROL_PERIOD){
        if(deltaValue > g_maxEncodePidDeltaValue) {
             DBG_PRINTF("===Motor Encode Error = %d, motor side:%d, calculate period %dms===\r\n", deltaValue,pMotor->motorSide,pMotor->calPeriod);
             deltaValue = 0;
        }
    }else{
        if(deltaValue > g_maxEncodeAveDeltaValue) {
             DBG_PRINTF("===Motor Encode Error = %d, motor side:%d, calculate period %dms===\r\n", deltaValue,pMotor->motorSide,pMotor->calPeriod);
             deltaValue = 0;
        }       
    }


    /* 计算右轮电机每秒转速 */
    pMotor->speedRps = ((float)deltaValue * (1000.0 / (float)pMotor->calPeriod))/g_motorLineNum;   
    pMotor->speed = pMotor->speedRps * MOTOR_TIRE_DIAMETER * PI;
    pMotor->lastNumber = pMotor->curNumber;
    pMotor->deltaValue = deltaValue;
    if(qdm_dir == 0){   //电机反转
        pMotor->speed = -pMotor->speed;
    } 
 
    /*每10ms打印一次编码器实际数值和计算出来的速度
    if(pMotor->motorSide == MOTOR_LEFT){
        if((g_TimerInterruptCount % 100) == 0) {
            if((deltaValue != 0) ) {   //编码器数值有变化才进行打印
                DBG_PRINTF("T:%d|qc:%d,d:%d,dr:%d,crs:%.02f,rs:%.01f\r\n",
                                          g_TimerInterruptCount,qdm_cnt,pMotor->deltaValue,qdm_dir,pMotor->speedRps,pMotor->speed);
            }
        }
    }*/

}
