#ifndef BUTTON_H
#define BUTTON_H

#define ADJUST_MINUS_90   0
#define ADJUST_MINUS_45   1
#define ADJUST_MINUS_0    2
#define ADJUST_PLUS_45    3
#define ADJUST_PLUS_90    4

void AngleAdjustment(unsigned int index, unsigned int angles);
void AngleAdjuseProcess(void);
void InitButtonFunction(void);
BASE_StatusType GPIO_KeySample(void);
BASE_StatusType ButtonPrintSample(void);

#endif /* GPIO_KEY_SAMPLE_H */