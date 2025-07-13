#include <math.h>
#include <stdlib.h>
#include <string.h>  // For memcpy
#include <stdio.h>
#include <unistd.h>

#include "main.h"
#include "debug.h"
#include "i2c.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "gyro.h"

#define gyroKp            (20.0f)     // 比例增益支配率收敛到加速度计/磁强计
#define gyroKi            (0.0004f)   // 积分增益支配率的陀螺仪偏见的衔接
#define gyroHalfT         (0.005f)    // 采样周期的一半
#define PAI               3.14
#define DEGREES           180

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;   // 四元数的元素，代表估计方向
float exInt = 0, eyInt = 0, ezInt = 0;  // 按比例缩小积分误差
float g_gyro_yaw, g_gyro_pitch, g_gyro_roll;                 // 偏航角，俯仰角，翻滚角
static float yaw_conv = 0.0f;


/**
 * @berf i2c read
 * @param hi_u8 reg_high_8bit_cmd:Transmit register value 8 bits high
 * @param hi_u8 reg_low_8bit_cmd:Transmit register value low 8 bits
 * @param hi_u8* recv_data:Receive data buff
 * @param hi_u8 send_len:Sending data length
 * @param hi_u8 read_len:Length of received data
*/
uint32_t LSM6DS_WriteRead(uint8_t reg_high_8bit_cmd, uint8_t send_len, uint8_t read_len)
{
    uint8_t recvData[12] = {0};
    uint32_t ret = 0;
    uint8_t send_user_cmd[1] = {reg_high_8bit_cmd};
    ret= HAL_I2C_MasterWriteBlocking(&g_i2c0, LSM6DS_WRITE_ADDR, send_user_cmd, send_len, 10000);
    if (ret != BASE_STATUS_OK) {
            DBG_PRINTF("LINE:%d,write Data Fail,ret:%d\r\n", __LINE__, ret);
            return ret;
        }
        /* Read data from eeprom. */
    ret = HAL_I2C_MasterReadBlocking(&g_i2c0, LSM6DS_READ_ADDR, recvData, read_len, 10000);
    if (ret != BASE_STATUS_OK) {
            DBG_PRINTF("LINE:%d,Read Data Fail,ret:%d\r\n", __LINE__, ret);
            return ret;
        }
    DBG_PRINTF("LSM6DS read ndef data\r\n");
    for (int i = 0; i < read_len; i++) {
         DBG_PRINTF("0x%x ", recvData[i]);
    }
    ret = recvData[0];
    return ret;
}

uint32_t LSM6DS_ReadCont(uint8_t reg_addr, uint8_t* buffer, uint16_t read_len)
{
    uint32_t ret = 0;
    uint8_t send_user_cmd[1] = {reg_addr};
    ret= HAL_I2C_MasterWriteBlocking(&g_i2c0, LSM6DS_WRITE_ADDR, send_user_cmd, 1, 10000);
    if (ret != BASE_STATUS_OK) {
            DBG_PRINTF("LINE:%d,write Data Fail,ret:%d\r\n", __LINE__, ret);
            return ret;
        }
        /* Read data from eeprom. */
    ret = HAL_I2C_MasterReadBlocking(&g_i2c0, LSM6DS_READ_ADDR, buffer, read_len, 10000);
    if (ret != BASE_STATUS_OK) {
            DBG_PRINTF("LINE:%d,Read Data Fail,ret:%d\r\n", __LINE__, ret);
            return ret;
        }
    DBG_PRINTF("LSM6DS read ndef data\r\n");
    for (int i = 0; i < read_len; i++) {
         DBG_PRINTF("0x%x ", buffer[i]);
    }
    return ret;
}

uint32_t LSM6DS_Write(uint8_t addr, uint8_t writedata, uint32_t buffLen)
{
    uint8_t buffer[2] = {addr, writedata};
    uint32_t retval = HAL_I2C_MasterWriteBlocking(&g_i2c0, LSM6DS_WRITE_ADDR, buffer, buffLen, 10000);
    if (retval != BASE_STATUS_OK) {
        DBG_PRINTF("LSM6DS_Write: IoTI2cWrite(%02X) failed, %0X!\n", buffer[0], retval);
        return retval;
    }
    DBG_PRINTF("IoTI2cWrite(%02X)\r\n", buffer[0]);
    return BASE_STATUS_OK;
}

void IMU_YAW_CAL(float gyroZ)
{
    int ret = 0;
    static char Pitchline[32] = { 0 };
    static char Rollline[32] = { 0 };
    static char Yawline[32] = { 0 };
    static float dt = 0.03; // 0.03代表300ms读取陀螺仪数据
    static float yaw = 0.0f, temp = 0.0f;
    // 除去零偏
    #if 0
    static int a = 0;
    a++;
    if (hi_get_seconds() <= 5) { // 5s
        printf("---------times-----------:%d\n", a);
    }
    #endif
    if (fabs(gyroZ) < 0.04) { // 0.04标准值
        temp = 0;
    } else {
        temp = gyroZ * dt;
    }
    yaw += temp;
    yaw_conv = yaw * 57.32; // 57.32 初始值
    // 360°一个循环
    if (fabs(yaw_conv) > 360.0f) {
        if ((yaw_conv) < 0) {
            yaw_conv += 360.0f;
        } else {
            yaw_conv -= 360.0f;
        }
    }
    DBG_PRINTF("Pitch:%.02f, Roll:%.02f, yaw:%.2f\n", g_gyro_pitch, g_gyro_roll, yaw_conv);
    ssd1306_SetCursor(0, 15); // 0为横坐标，15为纵坐标
    ret = sprintf(Pitchline, "Pitch: %.2f", g_gyro_pitch);
    if (ret < 0) {
        printf("Pitch failed\r\n");
    }
    ssd1306_DrawString(Pitchline, Font_7x10, White);
    ssd1306_SetCursor(0, 30); // 0为横坐标，30为纵坐标
    ret = sprintf(Rollline, "roll: %.2f", g_gyro_roll);
    if (ret < 0) {
        printf("roll failed\r\n");
    }
    ssd1306_DrawString(Rollline, Font_7x10, White);
    ssd1306_SetCursor(0, 0); // 0为横坐标，0为纵坐标
    ret = sprintf(Yawline, "roll: %.2f", yaw_conv);
    if (ret < 0) {
        printf("yaw failed\r\n");
    }
    ssd1306_DrawString(Yawline, Font_7x10, White);
    ssd1306_UpdateScreen();
}

void GetRoll(float atan2x, float atan2y)
{
    float atan2_x = atan2x;
    float atan2_y = atan2y;
    if (atan2_x > 0) {
        g_gyro_roll = atan(atan2_y / atan2_x) * DEGREES / PAI;
    } else if (atan2_x < 0 && atan2_y >= 0) {
        g_gyro_roll = atan(atan2_y / atan2_x) * DEGREES / PAI + DEGREES;
    } else if (atan2_x < 0 && atan2_y < 0) {
        g_gyro_roll = atan(atan2_y / atan2_x) * DEGREES / PAI - DEGREES;
    } else if (atan2_y > 0 && fabsf(atan2_x) < 0.001) {
        g_gyro_roll = 90; // 90°
    } else if (atan2_y < 0 && fabsf(atan2_x) < 0.001) {
        g_gyro_roll = -90; // -90°
    } else {
        printf("undefined\n");
    }
}

void GetPitch(float atan2x, float atan2y)
{
    float atan2_x = atan2x;
    float atan2_y_pitch = atan2y;
    if (atan2_x > 0) {
        g_gyro_pitch = atan(atan2_y_pitch / atan2_x) * DEGREES / PAI;
    } else if (atan2_x < 0 && atan2_y_pitch >= 0) {
        g_gyro_pitch = atan(atan2_y_pitch / atan2_x) * DEGREES / PAI + DEGREES;
    } else if (atan2_x < 0 && atan2_y_pitch < 0) {
        g_gyro_pitch = atan(atan2_y_pitch / atan2_x) * DEGREES / PAI - DEGREES;
    } else if (atan2_y_pitch > 0 && fabsf(atan2_x) < 0.001) {
        g_gyro_pitch = 90; // 90°
    } else if (atan2_y_pitch <  0 && fabsf(atan2_x) < 0.001) {
        g_gyro_pitch = -90; // -90°
    } else {
        printf("undefined\n");
    }
}

void IMU_Attitude_cal(float gcx, float gcy, float gcz, float acx, float acy, float acz)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float atan2_x, atan2_y;
    float atan2_y_pitch;
    float ax = acx, ay = acy, az = acz;
    float gx = gcx, gy = gcy, gz = gcz;

    // 把采集到的三轴加速度转化为单位向量，即向量除以模
    norm = (float)sqrt((float)(ax * ax + ay * ay + az * az));
    if (fabsf(norm) < 0.001) {
        printf("norm = 0,failed\n");
    }
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;

    // 把四元素换算成方向余弦中的第三行的三个元素
    // vx、vy、vz其实就是上一次的欧拉角(四元数)机体参考坐标系换算出来的重力的单位向量
    vx = 2 * (q1 * q3 - q0 * q2); // 2计算系数
    vy = 2 * (q0 * q1 + q2 * q3); // 2计算系数
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 对向量叉乘，求出姿态误差
    // ex、ey、ez为三轴误差元素
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // 叉乘向量仍旧是机体坐标系上的，而陀螺仪积分误差也是机体坐标系
    // 而且叉积的大小与陀螺仪误差成正比，正好拿来纠正陀螺
    exInt = exInt + ex * gyroKi;
    eyInt = eyInt + ey * gyroKi;
    ezInt = ezInt + ez * gyroKi;

    // 调整后的陀螺仪测量
    gx = gx + gyroKp * ex + exInt;
    gy = gy + gyroKp * ey + eyInt;
    gz = gz + gyroKp * ez + ezInt;

    // 使用一阶龙格库塔解四元数微分方程
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * gyroHalfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * gyroHalfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * gyroHalfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * gyroHalfT;

    // 四元数归一化
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (fabsf(norm) < 0.001) {
        printf("norm = 0,failed\n");
    }
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
    
    // 计算姿态角，本文Roll为横滚角，Pitch为俯仰角
    atan2_x = -2 * q1 * q1 - 2 * q2 * q2 + 1; // 2 计算参数
    atan2_y = 2 * q2 * q3 + 2 * q0 * q1; // 2 计算参数
    GetRoll(atan2_x, atan2_y);
    // 俯仰角
    atan2_y_pitch = -2 * q1 * q3 + 2 * q0 * q2; // 2 计算参数
    GetPitch(atan2_x, atan2_y_pitch);
}

void Lsm_Get_RawAcc(void)
{
    uint8_t buf[12] = {0};
    int16_t acc_x = 0, acc_y = 0, acc_z = 0;
    float acc_x_conv = 0, acc_y_conv = 0, acc_z_conv = 0;
    int16_t ang_rate_x = 0, ang_rate_y = 0, ang_rate_z = 0;
    float ang_rate_x_conv = 0, ang_rate_y_conv = 0, ang_rate_z_conv = 0;

    if ((LSM6DS_WriteRead(LSM6DSL_STATUS_REG, 1, 1) & 0x03)!=0) {
        if (BASE_STATUS_OK != LSM6DS_ReadCont(LSM6DSL_OUTX_L_G, buf, 12)) {
            DBG_PRINTF("i2c read error!\n");
        }
        else {
            ang_rate_x = (buf[1] << 8) + buf[0];
            ang_rate_y = (buf[3] << 8) + buf[2];
            ang_rate_z = (buf[5] << 8) + buf[4];
            acc_x = (buf[7] << 8) + buf[6];
            acc_y = (buf[9] << 8) + buf[8];
            acc_z = (buf[11] << 8) + buf[10];

            // DBG_PRINTF("lsm acc: %d, %d, %d \n ang: %d, %d, %d\n ",
            //     acc_x, acc_y, acc_z, ang_rate_x, ang_rate_y, ang_rate_z);
            ang_rate_x_conv = 3.14 / 180.0 * ang_rate_x / 14.29;
            ang_rate_y_conv = 3.14 / 180.0 * ang_rate_y / 14.29;
            ang_rate_z_conv = 3.14 / 180.0 * ang_rate_z / 14.29;

            acc_x_conv = acc_x / 4098.36;
            acc_y_conv = acc_y / 4098.36;
            acc_z_conv = acc_z / 4098.36;
            // DBG_PRINTF("lsm trans acc: %.2f, %.2f, %.2f \n ang: %.2f, %.2f, %.2f\n ",
            //     acc_x_conv, acc_y_conv, acc_z_conv, ang_rate_x_conv, ang_rate_y_conv, ang_rate_z_conv);
            IMU_Attitude_cal(ang_rate_x_conv, ang_rate_y_conv, ang_rate_z_conv, acc_x_conv, acc_y_conv, acc_z_conv);
            IMU_YAW_CAL(ang_rate_z_conv);
        }
    }
}

void LSM6DS_Init()
{
    LSM6DS_Write(LSM6DSL_CTRL3_C, 0x34, 2);
    LSM6DS_Write(LSM6DSL_CTRL2_G , 0X4C, 2); // 角速度陀螺仪配置2000dps ,104Hz
    LSM6DS_Write(LSM6DSL_CTRL10_C, 0x38, 2); // timer en, pedo en, tilt en ??
    LSM6DS_Write(LSM6DSL_CTRL1_XL, 0x4F, 2); // 加速度配置量程为8g,104Hz, lpf1_bw_sel=1, bw0_xl=1;
    
    LSM6DS_Write(LSM6DSL_TAP_CFG, 0x10, 2);
    LSM6DS_Write(LSM6DSL_WAKE_UP_DUR, 0x00, 2);
    LSM6DS_Write(LSM6DSL_WAKE_UP_THS, 0x02, 2);
    LSM6DS_Write(LSM6DSL_TAP_THS_6D, 0x40, 2);
    LSM6DS_Write(LSM6DSL_CTRL8_XL, 0x01, 2);
}

void InitGyro(void)
{
    uint32_t ret;
    ret = LSM6DS_WriteRead(LSM6DSL_WHO_AM_I, 1, 1);
    DBG_PRINTF("who am i: %X\n", ret);
    LSM6DS_Init();
    while (1) {
        Lsm_Get_RawAcc();
        BASE_FUNC_DELAY_MS(10); // 延时10ms
    }
}