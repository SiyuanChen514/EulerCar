#ifndef EULERCAR_TIMER_H
#define EULERCAR_TIMER_H

#include "debug.h"
#include "timer.h"
#include "main.h"

#define REQUIRE_TIME_IT 5000

#define FRAME_HEADER 0X7B // Frame_header //帧头
#define FRAME_TAIL 0X7D   // Frame_tail   //帧尾

#define SEND_DATA_SIZE 24
#define RECEIVE_DATA_SIZE 11

#define TxID 0x187  // 发送的数据帧ID为0x187
#define RxID 0x287  // 接收的数据帧ID为0x287
/* 用于存放陀螺仪加速度计三轴数据的结构体 */
typedef struct __Mpu6050_Data_
{
  short X_data; // 2 bytes //2个字节
  short Y_data; // 2 bytes //2个字节
  short Z_data; // 2 bytes //2个字节
} Mpu6050_Data;


/*
  一个数据包200个bit
  起始位 : 偏移 : 定义
  1 : 8 : 帧头
  9 : 16 : X轴
  25 : 16 : Y轴
  41 : 16 : Z轴
  57 : 16 : 电压
  73 : 48 : 三轴加速度
  121 : 48 : 三轴角速度
  169 : 6 : Reserved
  175 : 1 : 动作类型
  176 : 1 : 动作运行结果
  177 : 8 : checksum
  185 : 8 : 帧尾
  
*/

typedef struct _EulerCarSendData_
{
  unsigned char buffer[SEND_DATA_SIZE];
  struct _SensorStr_
  {
    unsigned char Frame_Header; // 帧头
    short X_speed;              // 运动模型的X轴
    short Y_speed;              // 运动模型的Y轴
    short Z_speed;              // 运动模型的Z轴
    short Power_Voltage;        // 电池电压
    Mpu6050_Data Accelerometer; // 加速度计的三轴加速度
    Mpu6050_Data Gyroscope;     // 角速度计的三轴角速度
    unsigned char Frame_Tail;   // 帧尾
  } SensorStr;
} EulerCarSendData;

typedef struct _MotorData_
{
  float leftSpeed; // 使用M法获取当前左轮的旋转速度
  float rightSpeed; //使用M法获取当前右轮的旋转速度
  unsigned int count; // 当前位置计数器的值
  unsigned int dir;// 和旋转的方向
} Motor_Data;

typedef struct _RECEIVE_DATA_
{
  unsigned char buffer[RECEIVE_DATA_SIZE];
  struct _ControlStr_
  {
    unsigned char Frame_Header; // 1 bytes //1个字节
    float X_speed;              // 4 bytes //4个字节
    float Y_speed;              // 4 bytes //4个字节
    float Z_speed;              // Z轴角速度4 bytes //4个字节
    unsigned char Frame_Tail;   // 1 bytes //1个字节
  } ControlStr;
} EulerCarRecvData;

void Pid_Process(void);
void recv_data_cal(void);
int  EulerCarSpeedCtrlLeft(float LeftSpeed);
int  EulerCarSpeedCtrlRight(float rightSpeed);
void UART3_INTRxSimultaneously(void);
void debug_send_buffer(unsigned int count);

#endif
