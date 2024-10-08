#ifndef __IOT_PROTOL_H
#define __IOT_PROTOL_H
#include "system.h"

enum {
    CMD_CAN_TRANS = 0X0C,
    CMD_GPS_START = 0X0E,
    CMD_GPS_STOP = 0X0D,
};

void IOT_Rec_Parse();
void CAN_Rec_Prase(stc_can_rxframe_t stcRxFrame);



#endif