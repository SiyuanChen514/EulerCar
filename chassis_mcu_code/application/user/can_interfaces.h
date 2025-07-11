#ifndef CAN_INTERFACES_H
#define CAN_INTERFACES_H


void ReceiveMsg(unsigned char* msg, unsigned int CANRxID);
void TransmitMsg(unsigned char* msg, unsigned int CANTxID);
void test_can(void);


#endif