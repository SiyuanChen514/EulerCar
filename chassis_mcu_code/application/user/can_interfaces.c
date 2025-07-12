#include "can_interfaces.h"
#include "can.h"
#include "debug.h"
#include "main.h"
#include "grab.h"

extern CANFrame g_sendFrame;
extern CANFrame g_receiveFrame;
extern CAN_Handle g_can;
extern bool isRecv;
extern bool isWrite;
unsigned char message = '\0';

void CANWriteCallbackFunc(void* handle){
    DBG_PRINTF("Write Callback\r\n");
    BASE_FUNC_UNUSED(handle);
    isWrite = true;
}
void CANReadCallbackFunc(void* handle){
    DBG_PRINTF("Read Callback!\r\n");
    BASE_FUNC_UNUSED(handle);
    isRecv = true;
}


unsigned char ReceiveMsg(unsigned int CANRxID){
    unsigned char msg;
    CAN_FilterConfigure rxFilter;
    g_can.rxFrame = &g_receiveFrame;
    rxFilter.receiveType = CAN_FILTERFRAME_STD_DATA;
    rxFilter.filterID = CANRxID;        /* 0x1014 and 0xFFFFF0FF constitute filtering rules */
    rxFilter.filterMask = 0x7FF;  /* 0xFFFFF0FF is filter ID mask */
    HAL_CAN_ReadIT(&g_can, &g_receiveFrame, &rxFilter);
    msg = g_receiveFrame.frame[0];
    
    return msg;
}

void TransmitMsg(unsigned char* msg, unsigned int CANTxID){   /* Address for storing received frame data */
    // DBG_PRINTF("CAN interrupt register \r\n");
    g_sendFrame.type = CAN_TYPEFRAME_STD_DATA; /* Transmit extended data frame */
    g_sendFrame.CANId = CANTxID;       /* 0x1314 is ID of transmitted data frames */
    g_sendFrame.dataLength = 1;       /* 1 is length of the sent frame */
    g_sendFrame.frame[0] = *msg;
    HAL_CAN_Write(&g_can, &g_sendFrame);
}


void test_can(void){
    DBG_PRINTF("TEST CAN");
    message = 0x00;
    while(1){
        message = ReceiveMsg(0x287);
        //DBG_PRINTF("TEST CAN");
        if(isRecv){
            if(message & 1){
                lift();
                DBG_PRINTF("Lift\r\n");
            }
            else{
                DBG_PRINTF("Lay\r\n");
                lay();
            }
        }
        // TransmitMsg(&message);
        // DBG_PRINTF("%d\r\n", i++);
    }
}