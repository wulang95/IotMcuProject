#ifndef __IOT_PROTOL_H
#define __IOT_PROTOL_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "can.h"


enum {
    CMD_CAN_TRANS = 0X0C,
    CMD_UP_ASK = 0xfe,
		CMD_UP_NOASK = 0Xfd
};




void IOT_Rec_Parse();
void CAN_Rec_Prase(stc_can_rxframe_t stcRxFrame);
void IOT_cmd_data_send(uint8_t cmd, uint8_t *data, uint16_t len);


#endif