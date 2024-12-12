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
		CMD_CAT_REPOWERON = 0X08,
		CMD_CRC_ERROR = 0X07,
		CMD_CAN_OTA_DATA = 0X06,
		CMD_CAN_OTA_START = 0X05,
    CMD_CAN_OTA_END = 0X04,
		CMD_CAN_OTA_DATA_FINISH = 0X03,
};

struct can_ota_stu{
	uint16_t buf_len;
	uint8_t dev_id;
	uint8_t pack_num;
	uint8_t last_pack_num;
	uint8_t ota_data_finish_flag;
	uint8_t pack_count;
	uint8_t ota_state;
};
extern struct can_ota_stu can_ota;
void can_ota_data();
void IOT_Rec_Parse();
void CAN_Rec_Prase(stc_can_rxframe_t stcRxFrame);
void IOT_cmd_data_send(uint8_t cmd, uint8_t *data, uint16_t len);


#endif