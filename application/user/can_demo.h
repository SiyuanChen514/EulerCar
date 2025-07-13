#ifndef CAN_DEMO_H
#define CAN_DEMO_H

int CAN_ReceiveFilter(void);
void CAN_ReadFinish(void *handle);
void CAN_WriteFinish(void *handle);

#endif /* GPIO_KEY_SAMPLE_H */