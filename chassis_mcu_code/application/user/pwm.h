#ifndef PWM_H
#define PWM_H

#include "gpt.h"
#include "main.h"


void set_duty(GPT_Handle* gpt_handle, int duty);


// void clockwise(GPT_Handle* gpt_handle, unsigned short time, unsigned short duty);
void Rotate(GPT_Handle* gpt_handle, unsigned short time, unsigned short duty);

void up(unsigned short time, unsigned short duty);
void down(unsigned short time, unsigned short duty);
void open(unsigned short time, unsigned short duty);
void _close(unsigned short time, unsigned short duty);

#endif