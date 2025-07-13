#ifndef PID_H
#define PID_H

//增量式PID控制算法结构体
typedef struct {
    float SetSpeed;        //上位机设定速度
    float ActualSpeed;     //编码器实际测试速度
    float err;             //偏差值
    float err_next;        //上一个偏差值
    float err_last;        //上上一个偏差值
    float IncSpeed;        //增量式PID算法计算出的增量速度
    float TargetIncSpeed;  //增量式PID算法目标增量速度    
    unsigned int duty;     //电机PWM占空比 
} PidEulerCar;

void Pid_Init(void);
float Pid_Ctrl(PidEulerCar *pMotor);

#endif