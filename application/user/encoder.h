#ifndef ENCODER_H
#define ENCODER_H

#define PI 3.1415927

#define MOTOR_LEFT    0
#define MOTOR_RIGHT   1

#define MOTOR_PID_CONTROL_PERIOD    10     //编码器读取周期，单位ms
#define EULER_CAR_DATA_SEND_PERIOD  100    //底盘数据上报上位机的周期，单位ms

typedef struct _Gear_Motor_handle {
    unsigned int     motorSide;            /* 当前是那个轮子*/
    unsigned int     curNumber;            /* 当前计数器 */
    unsigned int     lastNumber;           /* 上一次计数器 */
    float            speedRps;             /* 电机转速，单位每秒多少圈 */
    float            speed;                /* 电机速度，单位mm/s，由转速和轮胎直径计算 */
    unsigned int     calPeriod;            /* 轮速计算周期, 单位ms */
    unsigned int     deltaValue;           /* 编码器差值 */ 
} Gear_Motor_handle;


void InitGearMotor(void);
void QDM_CalMotorSpeed(Gear_Motor_handle *pMotor);

#endif