#ifndef USER_MOTOR_CONTROL_H
#define USER_MOTOR_CONTROL_H

#include <apt_ip.h>

void initMotor(void);
void startMotor(void);
void stopMotor(void);
void MotorForwardRotation(void);
void MotorReverse(void);
void MotorSpeedAdjustmentByDuty(unsigned int duty, unsigned int directions);
void MotorSpeedAdjustmentByFreq(unsigned int freq);
void MotorProcess(void);
void MotorRight(void);
void MotorLeft(void);
void User_APTForcePWMAOutputLow(APT_RegStruct *aptx);
void User_APTForcePWMBOutputLow(APT_RegStruct *aptx);
void User_APTPwmARecovery(APT_RegStruct *aptx);
void User_APTPwmBRecovery(APT_RegStruct *aptx);

#endif