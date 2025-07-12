#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include <securec.h>
#include "user_motor_control.h"
#include "eulercar_control.h"
#include "uart.h"
#include "pid.h"
#include "button.h"
#include "encoder.h"
#include "grab.h"
#include "can_interfaces.h"

unsigned char g_MotorState;                              //电机状态

EulerCarSendData g_SendData;                             //串口发送数据
EulerCarRecvData g_ReceiveData;                          //串口接收数据
static unsigned char g_ReceiveDataCount = 0;   

extern unsigned int g_pid_count;
extern PidEulerCar g_pidEulerCarRight;
extern PidEulerCar g_pidEulerCarLeft;
extern Gear_Motor_handle pidRightMotor;
extern Gear_Motor_handle pidLeftMotor;
extern Gear_Motor_handle aveRightMotor;
extern Gear_Motor_handle aveLeftMotor;


//按键状态
extern int g_button1State;
extern int g_button2State;

//按键滤波计数器
extern int g_button1_count;
extern int g_button2_count;

//通过按键调整PID参数的步进参数
extern float g_KP_Step;
extern float g_KI_Step;
extern float g_KD_Step;


extern bool isRecv;
extern bool isWrite;
extern unsigned char ServoMsg;
extern unsigned char BackMsg;
extern bool is_servo_done;
extern bool Action;
/* 接收完成标志 */
static volatile bool g_RxInterruptflag = true;
/* 发送完成标志 */
static volatile bool g_TxInterruptFlag = true;

/* 定时器中断计数器 */
unsigned int g_TimerInterruptCount = 0;

/* 上报数据给上位机计数器 */
static unsigned int g_SendCount = 0;

/* 上位机下发命令计数器 */
static unsigned int g_RecvCount = 0;

//小车三轴目标运动速度，单位：m/s
float Move_X, Move_Y, Move_Z;

#define EULERCAR_WHEEL_TRACK  120.0    //EulerCar小车两个轮子之间的轮距，单位mm
float modelCalLeftSpeed = 0.0;         //根据两轮差速小车模型计算出的左轮速度
float modelCalRightSpeed = 0.0;        //根据两轮差速小车模型计算出的右轮速度

#define IS_SUPPORT_PID 1

/* 串口发送的数据进行赋值 */
static void data_transition(void)
{
    float A_Move_X,A_Move_Z;
  
    g_SendData.SensorStr.Frame_Header = FRAME_HEADER; //帧头
    g_SendData.SensorStr.Frame_Tail = FRAME_TAIL;     //帧尾

    //从各车轮当前速度求出三轴当前速度
    A_Move_X = (aveRightMotor.speed + aveLeftMotor.speed) / 2.0; //小车x轴速度，单位mm/s
    A_Move_Z = (aveRightMotor.speed - aveLeftMotor.speed) / (EULERCAR_WHEEL_TRACK * 2.0) * 1000.0; //小车角速度, 单位mrad/s

    g_SendData.SensorStr.X_speed = (short)(round(A_Move_X)); //小车x轴速度，单位mm/s
    g_SendData.SensorStr.Y_speed = 0;
    g_SendData.SensorStr.Z_speed = (short)(round(A_Move_Z)); //小车角速度，单位mm/s

    DBG_PRINTF("ActualeftSpeed:%.2fmm/s,ActualRightSpeed:%.2fmm/s,A_Move_X=%.2fmm/s,A_Move_Z=%.2fmrad/s\r\n", 
                                                          aveLeftMotor.speed,aveRightMotor.speed,A_Move_X,A_Move_Z);

    //加速度计三轴加速度
    g_SendData.SensorStr.Accelerometer.X_data = 1;  //加速度计Y轴转换到ROS坐标X轴
    g_SendData.SensorStr.Accelerometer.Y_data = 1;  //加速度计X轴转换到ROS坐标Y轴
    g_SendData.SensorStr.Accelerometer.Z_data = 1;  //加速度计Z轴转换到ROS坐标Z轴

    //角速度计三轴角速度
    g_SendData.SensorStr.Gyroscope.X_data = 2;     //角速度计Y轴转换到ROS坐标X轴
    g_SendData.SensorStr.Gyroscope.Y_data = 2;     //角速度计X轴转换到ROS坐标Y轴
    if (g_MotorState == 0)
        //如果电机控制位使能状态，那么正常发送Z轴角速度
        g_SendData.SensorStr.Gyroscope.Z_data = 1;
    else
        //如果机器人是静止的（电机控制位失能），那么发送的Z轴角速度为0
        g_SendData.SensorStr.Gyroscope.Z_data = 0;

    //电池电压(这里将浮点数放大一千倍传输，相应的在接收端在接收到数据后也会缩小一千倍)
    g_SendData.SensorStr.Power_Voltage = 90;

    g_SendData.buffer[0] = g_SendData.SensorStr.Frame_Header; // 帧头
    g_SendData.buffer[1] = g_MotorState;                      // 小车软件失能标志位

    //小车三轴速度,各轴都拆分为两个8位数据再发送
    g_SendData.buffer[2] = g_SendData.SensorStr.X_speed >> 8;
    g_SendData.buffer[3] = g_SendData.SensorStr.X_speed;
    g_SendData.buffer[4] = g_SendData.SensorStr.Y_speed >> 8;
    g_SendData.buffer[5] = g_SendData.SensorStr.Y_speed;
    g_SendData.buffer[6] = g_SendData.SensorStr.Z_speed >> 8;
    g_SendData.buffer[7] = g_SendData.SensorStr.Z_speed;

    //IMU加速度计三轴加速度,各轴都拆分为两个8位数据再发送
    g_SendData.buffer[8] = g_SendData.SensorStr.Accelerometer.X_data >> 8;
    g_SendData.buffer[9] = g_SendData.SensorStr.Accelerometer.X_data;
    g_SendData.buffer[10] = g_SendData.SensorStr.Accelerometer.Y_data >> 8;
    g_SendData.buffer[11] = g_SendData.SensorStr.Accelerometer.Y_data;
    g_SendData.buffer[12] = g_SendData.SensorStr.Accelerometer.Z_data >> 8;
    g_SendData.buffer[13] = g_SendData.SensorStr.Accelerometer.Z_data;

    //IMU角速度计三轴角速度,各轴都拆分为两个8位数据再发送
    g_SendData.buffer[14] = g_SendData.SensorStr.Gyroscope.X_data >> 8;
    g_SendData.buffer[15] = g_SendData.SensorStr.Gyroscope.X_data;
    g_SendData.buffer[16] = g_SendData.SensorStr.Gyroscope.Y_data >> 8;
    g_SendData.buffer[17] = g_SendData.SensorStr.Gyroscope.Y_data;
    g_SendData.buffer[18] = g_SendData.SensorStr.Gyroscope.Z_data >> 8;
    g_SendData.buffer[19] = g_SendData.SensorStr.Gyroscope.Z_data;

    //电池电压,拆分为两个8位数据发送
    g_SendData.buffer[20] = g_SendData.SensorStr.Power_Voltage >> 8;
    g_SendData.buffer[21] = g_SendData.SensorStr.Power_Voltage;

    //数据校验位计算
    unsigned char check_sum = 0, k;
    for (k = 0; k < 22; k++) {
        check_sum = check_sum ^ g_SendData.buffer[k];
    }
    g_SendData.buffer[22] = check_sum;
    g_SendData.buffer[23] = g_SendData.SensorStr.Frame_Tail; //帧尾
}

void debug_send_buffer(unsigned int count)
{
    DBG_PRINTF("-----------------send_count=%u-----------------\r\n", count);
    for (size_t i = 0; i < SEND_DATA_SIZE; i++) {
        DBG_PRINTF("0x%x ", g_SendData.buffer[i]);
    }
    DBG_PRINTF("  \r\n");
}

void UART3WriteInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    //DBG_PRINTF("\r\nUART Write Finish\r\n");
    g_TxInterruptFlag = true;
    return;
}

void UART3ReadInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    g_RxInterruptflag = 1;
    return;
}

// 定时器10ms产生一个中断
void TIMER0_InterruptProcess(void *handle)
{
    BASE_StatusType ret;
    /* USER CODE BEGIN TIMER0_InterruptProcess */
    TIMER_Handle *timerHandle = (TIMER_Handle *)handle;
    BASE_FUNC_UNUSED(timerHandle);
    g_TimerInterruptCount++;

    //按键滤波，500毫秒内算一次按键
    if(g_button1_count > 0)
    {
        g_button1_count ++;
        if(g_button1_count > 50){
           g_button1_count = 0;
           //DBG_PRINTF("Button1 Count Reset 0\r\n");
        }
    }

    if(g_button2_count > 0)
    {
        g_button2_count ++;
        if(g_button2_count > 50){
           g_button2_count = 0;
           //DBG_PRINTF("Button2 Count Reset 0\r\n");
        }
    }

    //按设置周期上报底盘数据
    if ((g_TimerInterruptCount % (EULER_CAR_DATA_SEND_PERIOD/10)) == 0) {
        if (g_TxInterruptFlag) {
            g_TxInterruptFlag = false;
            //获取小车要发送的数据
            QDM_CalMotorSpeed(&aveLeftMotor);    //计算左轮平均速度           
            QDM_CalMotorSpeed(&aveRightMotor);   //计算右轮平均速度
            data_transition();

            ret = HAL_UART_WriteIT(&g_uart3, g_SendData.buffer, SEND_DATA_SIZE);
           
            if(ret == BASE_STATUS_OK) {
                //DBG_PRINTF("Data send OK, send count = %d\r\n",g_SendCount);
            }else{
                DBG_PRINTF("Data send error, code = %d\r\n",ret);
            }
            g_SendCount++;
        }
    }

    //每间隔10毫秒进行一次PID电机控制
    if (g_TimerInterruptCount % (MOTOR_PID_CONTROL_PERIOD / 10) == 0) {
         //获取小车运行实际速度
         QDM_CalMotorSpeed(&pidLeftMotor);    //计算一个PID周期内左轮速度           
         QDM_CalMotorSpeed(&pidRightMotor);   //计算一个PID周期内右轮速度
       
         g_pidEulerCarLeft.ActualSpeed = pidLeftMotor.speed;
         g_pidEulerCarRight.ActualSpeed = pidRightMotor.speed;
         
         //如果上位机下发速度大于0.1mm/s，则启动PID控制
         if ((fabsf(modelCalLeftSpeed) > 0.1) || (fabsf(modelCalRightSpeed) > 0.1)) {
             Pid_Process();
         }
    }

}

static float XYZ_transition(unsigned char High, unsigned char Low)
{
    short transition = ((High << 8) + Low);                    //将高8位和低8位整合成一个16位的short型数据
    //return transition / 1000 + (transition % 1000) * 0.001;  //单位转换, mm/s->m/s
    return (float)transition;
}

int EulerCarSpeedCtrlLeft(float LeftSpeed)
{

    unsigned int duty = (abs)((int)(LeftSpeed/g_motorMaxSpeed * 100)); 
    if (duty > 99) {
        duty = 35;
    }
    //DBG_PRINTF("left wheel duty:%d\r\n", duty);
    HAL_APT_SetPWMDutyByNumber(&g_apt1, duty);

    //如果电机小于0.1mm/s，则停止电机转动
    if (fabsf(LeftSpeed) < 0.1) {
        HAL_APT_StopModule(RUN_APT1);
        return 0;
    }
    if (LeftSpeed > 0) {
        User_APTPwmARecovery(g_apt1.baseAddress);  
        User_APTForcePWMBOutputLow(g_apt1.baseAddress);
    } else {
        User_APTPwmBRecovery(g_apt1.baseAddress);
        User_APTForcePWMAOutputLow(g_apt1.baseAddress);
    }
    HAL_APT_StartModule(RUN_APT1);
    //DBG_PRINTF("Left wheel set duty:%d, run ok!\r\n",duty);

    return 0;
}

int EulerCarSpeedCtrlRight(float rightSpeed)
{
    unsigned int duty = (abs)((int)(rightSpeed/g_motorMaxSpeed * 100));   
    if (duty > 99) {
        duty = 35;
    }
    HAL_APT_SetPWMDutyByNumber(&g_apt0, duty);

    //如果电机小于0.1mm/s，则停止电机转动
    if (fabsf(rightSpeed) < 0.1) {
        HAL_APT_StopModule(RUN_APT0);
        return 0;
    }
    if (rightSpeed > 0) { 
        User_APTPwmARecovery(g_apt0.baseAddress); 
        User_APTForcePWMBOutputLow(g_apt0.baseAddress);
    } else {
        User_APTPwmBRecovery(g_apt0.baseAddress);
        User_APTForcePWMAOutputLow(g_apt0.baseAddress);
    }
    HAL_APT_StartModule(RUN_APT0);
    //DBG_PRINTF("Right wheel set duty:%d, run ok!\r\n",duty);

    return 0;
}

void Pid_Process(void)
{
    float pidTargetSpeed;
    g_pid_count++;

    /* 通过PID算法计算右轮目标速度 */
    pidTargetSpeed = Pid_Ctrl(&g_pidEulerCarRight);
    EulerCarSpeedCtrlRight(pidTargetSpeed);   
    
    /* 通过PID算法计算左轮目标速度 */
    pidTargetSpeed = Pid_Ctrl(&g_pidEulerCarLeft);
    EulerCarSpeedCtrlLeft(pidTargetSpeed); 
   
    /* 
    //左右轮同时打印
    if((g_TimerInterruptCount % 10) == 0) {
        DBG_PRINTF("N=%d,P=%.02f,I=%.02f,D=%.02f|L-SS:%.1f,AS:%.1f,err:%.1f,nr:%.1f,lr:%.1f,in:%.1f,TS:%.1f,dt:%d|R-SS:%.1f,AS:%.1f,err:%.1f,nr:%.1f,lr:%.1f,in:%.f,TS:%.1f,dt:%d\r\n", 
                         g_pid_count,g_KP,g_KI,g_KD,g_pidEulerCarRight.SetSpeed,g_pidEulerCarRight.ActualSpeed,
                               g_pidEulerCarRight.err,g_pidEulerCarRight.err_next,g_pidEulerCarRight.err_last,g_pidEulerCarRight.IncSpeed,
                               g_pidEulerCarRight.TargetIncSpeed,g_pidEulerCarRight.duty,
                         g_pidEulerCarLeft.SetSpeed,g_pidEulerCarLeft.ActualSpeed,
                               g_pidEulerCarLeft.err,g_pidEulerCarLeft.err_next,g_pidEulerCarLeft.err_last,g_pidEulerCarLeft.IncSpeed,
                               g_pidEulerCarLeft.TargetIncSpeed,g_pidEulerCarLeft.duty);
    } 

    //打印电机运行情况
    if((g_TimerInterruptCount % 10) == 0) {
        DBG_PRINTF("N=%d|L-SS:%.1f,AS:%.1f,err:%.1f,nr:%.1f,in:%.1f,TS:%.1f,dt:%d|R-SS:%.1f,AS:%.1f,err:%.1f,nr:%.1f,in:%.f,TS:%.1f,dt:%d\r\n", 
                         g_pid_count,g_pidEulerCarRight.SetSpeed,g_pidEulerCarRight.ActualSpeed,
                               g_pidEulerCarRight.err,g_pidEulerCarRight.err_next,g_pidEulerCarRight.IncSpeed,
                               g_pidEulerCarRight.TargetIncSpeed,g_pidEulerCarRight.duty,
                         g_pidEulerCarLeft.SetSpeed,g_pidEulerCarLeft.ActualSpeed,
                               g_pidEulerCarLeft.err,g_pidEulerCarLeft.err_next,g_pidEulerCarLeft.IncSpeed,
                               g_pidEulerCarLeft.TargetIncSpeed,g_pidEulerCarLeft.duty);
    } 
   
 
    //左右轮同时打印
    if((g_TimerInterruptCount % 10) == 0) {
        DBG_PRINTF("N=%d,P=%.02f,I=%.02f,D=%.02f|L-SS:%.1f,AS:%.1f,err:%.1f,nr:%.1f,lr:%.1f,in:%.1f,TS:%.1f,dt:%d|R-SS:%.1f,AS:%.1f,err:%.1f,nr:%.1f,lr:%.1f,in:%.f,TS:%.1f,dt:%d\r\n", 
                         g_pid_count,g_KP,g_KI,g_KD,g_pidEulerCarRight.SetSpeed,g_pidEulerCarRight.ActualSpeed,
                               g_pidEulerCarRight.err,g_pidEulerCarRight.err_next,g_pidEulerCarRight.err_last,g_pidEulerCarRight.IncSpeed,
                               g_pidEulerCarRight.TargetIncSpeed,g_pidEulerCarRight.duty,
                         g_pidEulerCarLeft.SetSpeed,g_pidEulerCarLeft.ActualSpeed,
                               g_pidEulerCarLeft.err,g_pidEulerCarLeft.err_next,g_pidEulerCarLeft.err_last,g_pidEulerCarLeft.IncSpeed,
                               g_pidEulerCarLeft.TargetIncSpeed,g_pidEulerCarLeft.duty);
    } 
 
    //只打印右轮
    if((g_TimerInterruptCount % 10) == 0) {
        DBG_PRINTF("N=%d,P=%.02f,I=%.02f,D=%.02f|R-SS:%.1f,AS:%.1f,err:%.1f,nr:%.1f,lr:%.1f,in:%.1f,TS:%.1f,dt:%d\r\n", 
                         g_pid_count,g_KP,g_KI,g_KD,g_pidEulerCarRight.SetSpeed,
                         g_pidEulerCarRight.ActualSpeed,g_pidEulerCarRight.err,g_pidEulerCarRight.err_next,g_pidEulerCarRight.err_last,
                                   g_pidEulerCarRight.IncSpeed,g_pidEulerCarRight.TargetIncSpeed,g_pidEulerCarRight.duty);
    } 

    //只打印左轮
    if((g_TimerInterruptCount % 100) == 0) {
        DBG_PRINTF("N=%d,P=%.02f,I=%.02f,D=%.02f|L-SS:%.1f,AS:%.1f,err:%.1f,nr:%.1f,lr:%.1f,in:%.1f,TS:%.1f,dt:%d\r\n", 
                         g_pid_count,g_KP,g_KI,g_KD,g_pidEulerCarLeft.SetSpeed,
                         g_pidEulerCarLeft.ActualSpeed,g_pidEulerCarLeft.err,g_pidEulerCarLeft.err_next,g_pidEulerCarLeft.err_last,
                                   g_pidEulerCarLeft.IncSpeed,g_pidEulerCarLeft.TargetIncSpeed,g_pidEulerCarLeft.duty);
    } */
}

void recv_data_cal(void)
{   
    float x_linear,z_angular;
    
    g_ReceiveData.ControlStr.X_speed = Move_X;
    g_ReceiveData.ControlStr.Y_speed = Move_Y;
    g_ReceiveData.ControlStr.Z_speed = Move_Z;
    x_linear = g_ReceiveData.ControlStr.X_speed; 
    z_angular = g_ReceiveData.ControlStr.Z_speed;
    //DBG_PRINTF("x_speed:%.02f, Z_speed:%.02f\r\n", g_ReceiveData.ControlStr.X_speed, g_ReceiveData.ControlStr.Z_speed);
    //差分轮运动学模型求解
    modelCalLeftSpeed  = x_linear - z_angular * EULERCAR_WHEEL_TRACK / 2.0 / 1000.0;    //左轮速度，单位mm/s
    modelCalRightSpeed = x_linear + z_angular * EULERCAR_WHEEL_TRACK / 2.0 / 1000.0;    //右轮速度，单位mm/s
    DBG_PRINTF("modelCalLeftSpeed:%.2fmm/s, modelCalRightSpeed:%.2fmm/s\r\n", modelCalLeftSpeed, modelCalRightSpeed);
   
    //将上位机设置轮速存到PID控制结构体
    g_pidEulerCarLeft.SetSpeed = modelCalLeftSpeed;
    g_pidEulerCarRight.SetSpeed = modelCalRightSpeed;

    //如果上位机下发速度小于0.1mm/s，小车停止运动
    if((fabsf(modelCalLeftSpeed) < 0.1) && (fabsf(modelCalRightSpeed) < 0.1))
    {
        EulerCarSpeedCtrlRight(0);
        EulerCarSpeedCtrlLeft(0);
        //DBG_PRINTF("EulerCar stoped!!!\r\n");
    }
}

void UART3_INTRxSimultaneously(void)
{
    float tmp_X,tmp_Y,tmp_Z; 
    unsigned char k,Usart_Receive,check_sum;

    while (1) {
        if (g_RxInterruptflag) {
            g_RxInterruptflag = false;
            HAL_UART_ReadIT(&g_uart3, &Usart_Receive, 1); // 读取数据
            g_ReceiveData.buffer[g_ReceiveDataCount] = Usart_Receive;
            //确保数组第一个数据为FRAME_HEADER
            if (Usart_Receive == FRAME_HEADER || g_ReceiveDataCount > 0) {
                g_ReceiveDataCount++;
            }
            else {
                g_ReceiveDataCount = 0;
            }
            // 验证数据包的长度
            if (g_ReceiveDataCount == RECEIVE_DATA_SIZE) {
                g_ReceiveDataCount = 0; //为串口数据重新填入数组做准备
                //验证数据包的帧尾
                if (g_ReceiveData.buffer[10] == FRAME_TAIL) {
                    check_sum = 0;
                    for (k = 0; k < 9; k++) {
                        check_sum = check_sum ^ g_ReceiveData.buffer[k];
                    }
                    //数据异或位校验计算，模式0是发送数据校验
                    if (g_ReceiveData.buffer[9] == check_sum) {  
                        g_RecvCount++;
                        //从串口数据求三轴目标速度， 单位mm/s
                        tmp_X = XYZ_transition(g_ReceiveData.buffer[3], g_ReceiveData.buffer[4]);
                        tmp_Y = XYZ_transition(g_ReceiveData.buffer[5], g_ReceiveData.buffer[6]);
                        tmp_Z = XYZ_transition(g_ReceiveData.buffer[7], g_ReceiveData.buffer[8]);
                        //合法性检查，设置速度必须小于当前电机支持的最高速度
                        if( (fabsf(tmp_X) < g_motorMaxSpeed) && (fabsf(tmp_Y) < g_motorMaxSpeed) && (fabsf(tmp_Z) < MOTOR_MAX_ANGULAR_SPEED) ) {  
                            Move_X = tmp_X;
                            Move_Y = tmp_Y;
                            Move_Z = tmp_Z;
                            DBG_PRINTF("\r\ng_TimerCount=%d,g_RecvCount=%d,Move_X=%.2fmm/s,Move_Y=%.2fmm/s,Move_Z=%.2fmrad/s\r\n",
                                                                              g_TimerInterruptCount,g_RecvCount, Move_X, Move_Y, Move_Z);
                            recv_data_cal();
						}
                        else{
                            DBG_PRINTF("\r\ng_TimerCount=%d,g_RecvCount=%d",g_TimerInterruptCount,g_RecvCount);
                            if((fabsf(tmp_X) > g_motorMaxSpeed) || (fabsf(tmp_Y) > g_motorMaxSpeed))
                                DBG_PRINTF("\r\nLinear speed set error: X=%.2fmm/s Y=%.2fmm/s, must less %.2fmm/s\r\n", tmp_X, tmp_Y,g_motorMaxSpeed);
                            if((fabsf(tmp_Z) > MOTOR_MAX_ANGULAR_SPEED))
                                DBG_PRINTF("\r\nAngular speed set error: Z=%.2fmrad/s, must less %.1fmrad/s\r\n", tmp_Z, MOTOR_MAX_ANGULAR_SPEED);
                        }
                    }
                }
            }
        }

        ServoMsg = ReceiveMsg(RxID);
        if(isRecv){
            DBG_PRINTF("Receieved!\r\n");
            isRecv = false;
            DBG_PRINTF("Receive message: %x \r\n", ServoMsg);
            HAL_GPIO_SetValue(&g_gpio2, GPIO_PIN_5, GPIO_LOW_LEVEL);
            if(is_servo_done){
                is_servo_done = false;
                if(ServoMsg & CLOSE){
                    _close(200, 700);
                    DBG_PRINTF("CLOSE!\r\n");
                }
                else{
                    open(100, 300);
                    DBG_PRINTF("OPEN!\r\n");
                }
                if(ServoMsg & UP){
                    up(50, 650);
                    DBG_PRINTF("UP!\r\n");
                }
                else{
                    down(50, 400);
                    DBG_PRINTF("DOWN!\r\n");
                }
                is_servo_done = true;
            }
            else{
                BackMsg = (BackMsg << 1) & 0;    // 第7位表示动作类型: 0表示举起 1表示放下. 第八位表示动作是否完成: 1表示完成 0表示失败
            }
            
            TransmitMsg(&BackMsg, TxID);
            
            HAL_GPIO_SetValue(&g_gpio2, GPIO_PIN_5, GPIO_HIGH_LEVEL);
        }

        //检测按键是否按下，修改PID参数
        if (g_button1State == 1) {
            g_button1State = 0;
            g_KP = g_KP + g_KP_Step;
            DBG_PRINTF("Button1 Evnet, g_KP + %f\r\n",g_KP_Step);            
            DBG_PRINTF("g_KP=%f g_KI=%f g_KD=%f\r\n", g_KP, g_KI, g_KD);
        }
        
        if (g_button2State == 1) {
            g_button2State = 0;
            g_KI = g_KI + g_KI_Step;           
            DBG_PRINTF("Button2 Evnet, g_KI + %f\r\n",g_KI_Step);               
            DBG_PRINTF("g_KP=%f g_KI=%f g_KD=%f\r\n", g_KP, g_KI, g_KD);
        }
    }
}