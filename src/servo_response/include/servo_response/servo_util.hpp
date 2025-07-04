#ifndef SERVO_UTIL_H
#define SERVO_UTIL_H

// The file is mainly for the response of servo (Rotate up and down for example)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <cstdint>

#define PWM_CHIP_PATH "/sys/class/pwm/pwmchip0/"
#define PWM_EXPORT_PATH PWM_CHIP_PATH "export"
#define PWM_UNEXPORT_PATH PWM_CHIP_PATH "unexport"

#define PWM1_PERIOD_PATH PWM_CHIP_PATH "pwm1/period"
#define PWM1_DUTY_CYCLE_PATH PWM_CHIP_PATH "pwm1/duty_cycle"
#define PWM1_ENABLE_PATH PWM_CHIP_PATH "pwm1/enable"

#define PWM15_PERIOD_PATH PWM_CHIP_PATH "pwm15/period"
#define PWM15_DUTY_CYCLE_PATH PWM_CHIP_PATH "pwm15/duty_cycle"
#define PWM15_ENABLE_PATH PWM_CHIP_PATH "pwm15/enable"

#define COUNTERCLOCKWISE 1800000
#define CLOCKWISE 1200000

#define STATE_HIGH 1
#define STATE_LOW 0
#define STATE_OPEN 1
#define STATE_CLOSE 0

#define HORIZONTAL_SERVO_ID 15
#define VERTICAL_SERVO_ID 1

#define DELAY_TIME_MS 500


void set_period(int id, int period);
void set_duty_cycle(int id, int dc);
void run_servo(int id, int dc);
void close_servo(int id);
void disable_pwm(int id);
#endif