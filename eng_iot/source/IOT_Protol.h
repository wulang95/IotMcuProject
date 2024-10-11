#ifndef __IOT_PROTOL_H
#define __IOT_PROTOL_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "can.h"
enum {
    CMD_CAN_TRANS = 0X0C,
    CMD_GPS_POWERON = 0X0E,
    CMD_GPS_POWEROFF = 0X0D,
		CMD_GPS_DATA = 0X0F,
		CMD_GPS_TRANS = 0X0B,
		CMD_GPS_DEEPSLEEP = 0X0A,
		CMD_GPS_HOST_START = 0X09,
};

void IOT_Rec_Parse();
void CAN_Rec_Prase(stc_can_rxframe_t stcRxFrame);
void IOT_cmd_data_send(uint8_t cmd, uint8_t *data, uint16_t len);


#endif