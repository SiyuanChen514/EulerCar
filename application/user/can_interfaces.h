#ifndef CAN_INTERFACES_H
#define CAN_INTERFACES_H


#define UP 0x08	// 0000 1000
#define DOWN 0x04	//	0000 0100
#define CLOSE 0x02	// 	0000 0010
#define OPEN 0x01	// 	0000 0001

void ReceiveMsg(unsigned char* msg,unsigned int CANRxID);
void TransmitMsg(unsigned char* msg, unsigned int CANTxID);
void test_can(void);


#endif