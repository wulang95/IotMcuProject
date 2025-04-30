#include "system.h"
#include "app_ota.h"
#define   DATA_DEBUG 0

#if DATA_DEBUG==1
#define PRINT_DATA(A, B, C) Data_Print(A, B, C)
#else
#define PRINT_DATA(A, B, C)
#endif

uint8_t rec[256];
uint16_t reclen;
#define DATA_BODY  sizeof(stc_can_txframe_t)
#define DATALEN		2+DATA_BODY+2


/****************************************************
** 功能    :计算校验值
********************************************************/
uint16_t ble_Package_CheckSum(uint8_t* pdata, uint32_t len)
{
    uint16_t sum = 0;
    uint32_t i;

    for(i = 0; i < len; i++)
        sum += pdata[i];
    sum = sum ^ 0xFFFF;             //异或0xFFFFF = 取反后取低16位
    return sum;
}

void Data_Print(char *string, uint8_t *buff, uint16_t len)
{
		uint16_t i = 0;
		printf("%s[%d]:", string, len);
		while(len--){
			printf("%02x\t", buff[i]);
			i++;
		}
		printf("\r\n");
}

#define CAN_DATA_LEN   sizeof(stc_can_rxframe_t)
struct can_ota_stu can_ota;
uint8_t mcu_adc_flag;
void can_ota_data()
{
		uint8_t res;
		uint16_t ota_len;
		uint8_t data[8];
		uint32_t can_id = 0;
		stc_can_txframe_t can_fame;
		ota_len = FIFO_Valid_Size(CAN_OTA);
		if(ota_len == 0) {
				if(can_ota.ota_data_finish_flag == 1){
					can_ota.ota_data_finish_flag = 0;
					res = 1;
					IOT_cmd_data_send(CMD_CAN_OTA_DATA_FINISH, &res, 1);
				}
			return;
		}
		else if(ota_len <= 8) {
				FIFO_Rece_Buf(CAN_OTA, data, ota_len);
				can_fame.Control_f.DLC = ota_len;
		} else {
				FIFO_Rece_Buf(CAN_OTA, data, 8);
				can_fame.Control_f.DLC = 8;
		}
		can_id = 0x21 | can_ota.dev_id<< 8 | 0xd0 << 16;
		can_fame.Control_f.IDE = 1;
		can_fame.Control_f.RTR = 0;
		memcpy(can_fame.Data, data, can_fame.Control_f.DLC);
		if((can_ota.pack_count/16)%2) {
				can_id |= ((can_ota.pack_count%16)+ 16) << 24;		
		} else {
				can_id |= ((can_ota.pack_count%16)) << 24;	
		}
		can_fame.ExtID = can_id;
		Iot_Can_Send(can_fame);
		can_ota.pack_count++;
}

static uint8_t can_check_sum(uint8_t *dat, uint8_t len)
{
    uint8_t check_sum = 0;
    uint8_t i = 0;
    for(i = 0; i < len; i++)
    {
        check_sum +=dat[i];
    }
    return check_sum;

}
uint8_t lock_sta;
void car_jump_password()
{
		stc_can_txframe_t can_fame;
		uint8_t data[8];
		memset(data, 0, 8);
		can_fame.ExtID = 0x18142821;
		data[0] = 0x06;
		data[1] = 0x00;
		data[2] = 0x01;
		data[3] = 0x56;
		data[7] = can_check_sum(data, 7);
		can_fame.Control_f.DLC = 8;
		can_fame.Control_f.IDE = 1;
		can_fame.Control_f.RTR = 0;
		memcpy(can_fame.Data, data, 8);
		Iot_Can_Send(can_fame);
}
void car_unlock_send()
{
		stc_can_txframe_t can_fame;
		uint8_t data[8];
		memset(data, 0, 8);
		can_fame.ExtID = 0x18146021;
		data[0] = 0x01;
		data[1] = 0;
		data[2] = 0x01;
		data[3] = 0xa9;
		data[7] = can_check_sum(data, 7);
		can_fame.Control_f.DLC = 8;
		can_fame.Control_f.IDE = 1;
		can_fame.Control_f.RTR = 0;
		memcpy(can_fame.Data, data, 8);
		Iot_Can_Send(can_fame);
		Iot_Can_Send(can_fame);
	
		memset(data, 0, 8);
		can_fame.ExtID = 0x18142821;
		data[0] = 0x08;
		data[1] = 0x00;
		data[2] = 0x01;
		data[3] = 0xa9;
		data[7] = can_check_sum(data, 7);
		memcpy(can_fame.Data, data, 8);
		Iot_Can_Send(can_fame);
		Iot_Can_Send(can_fame);
		
		memset(data, 0, 8);
		can_fame.ExtID = 0x18142821;
		data[0] = 0x06;
		data[1] = 0x00;
		data[2] = 0x01;
		data[3] = 0x56;
		data[7] = can_check_sum(data, 7);
		memcpy(can_fame.Data, data, 8);
		Iot_Can_Send(can_fame);
		Iot_Can_Send(can_fame);
}



void car_lock_send()
{
		stc_can_txframe_t can_fame;
		uint8_t data[8];
		memset(data, 0, 8);
		can_fame.ExtID = 0x18146021;
		data[0] = 0x01;
		data[1] = 0;
		data[2] = 0x01;
		data[3] = 0x56;
		data[7] = can_check_sum(data, 7);
		can_fame.Control_f.DLC = 8;
		can_fame.Control_f.IDE = 1;
		can_fame.Control_f.RTR = 0;
		memcpy(can_fame.Data, data, 8);
		Iot_Can_Send(can_fame);
		Iot_Can_Send(can_fame);
	
		memset(data, 0, 8);
		can_fame.ExtID = 0x18142821;
		data[0] = 0x07;
		data[1] = 0x00;
		data[2] = 0x01;
		data[3] = 0x56;
		data[7] = can_check_sum(data, 7);
		memcpy(can_fame.Data, data, 8);
		Iot_Can_Send(can_fame);	
		Iot_Can_Send(can_fame);	
}
extern uint8_t unlock_cnt;
void IOT_rcv_data_handler(uint8_t cmd, uint8_t *data, uint16_t data_len)
{
		uint8_t res;
		uint8_t buf[128] = {0};
		stc_can_txframe_t can_tx_body; 
		stc_can_rxframe_t can_rx_body;
		printf("cmd:%02x\r\n", cmd);
		switch(cmd){
			case CMD_CAN_TRANS:
					memcpy(&can_rx_body, data, data_len);
					if(can_rx_body.Cst.Control_f.IDE == 1) {
							can_tx_body.ExtID = can_rx_body.ExtID;
					} else {
							can_tx_body.StdID = can_rx_body.StdID;
					}
					can_tx_body.Control_f.IDE = can_rx_body.Cst.Control_f.IDE;
					can_tx_body.Control_f.RTR = can_rx_body.Cst.Control_f.RTR;
					can_tx_body.Control_f.DLC = can_rx_body.Cst.Control_f.DLC;
					memcpy(can_tx_body.Data, can_rx_body.Data, 8);
					Iot_Can_Send(can_tx_body);
			break;
		}
}


void IOT_Rec_Parse()
{
	uint8_t res;
	static uint8_t cmd;
	static uint8_t buf[256];
	static uint16_t check_sum,rev_sum;
	static uint8_t step = 0;
	static uint16_t len, i;
	
	if(FIFO_Rece_Buf(IOT_UART, &res, 1) == 0){
			switch(step){
				case 0:
					if(res == 0xAA) {
							memset(buf, 0, sizeof(buf));
							step = 1;
							check_sum = 0;
							len = 0;
							i = 0;
							cmd = 0;
							rev_sum = 0;
							SET_SYS_TIME(WEEK_TIME, 30000);
							SET_SYS_TIME(IOT_PROTO_TM, 2000);
					}
					break;
				case 1:
					if(res == 0x55) {
							step = 2;
					} else {
						step = 0;
					}
					break;
				case 2:
					check_sum += res;
					cmd = res;
					step = 3;
					break;
				case 3:
					check_sum += res;
					len = res << 8;
					step = 4;
					break;
				case 4:
					len |= res;
					check_sum += res;
					if(len) step  = 5;
					else step = 6;
					break;
				case 5:
					buf[i++] = res;
					check_sum += res;
					if(i == len){
						step = 6;
					}
					break;
				case 6:
					rev_sum = res;
					step = 7;
					break;
				case 7:
					rev_sum |= res << 8;
					check_sum = check_sum ^ 0xFFFF; 
					if(rev_sum == check_sum) {
						IOT_rcv_data_handler(cmd, buf, len);		
					} else {
						printf("check error, rev_sum:%04x, check_sum:%04x\r\n", rev_sum, check_sum);
						FIFO_Clean_Buf(IOT_UART);
					}
					step = 0;
					break;
			}
	}	
	if(CHECK_SYS_TIME(IOT_PROTO_TM) == 0) {
		step = 0;
	}
}


void can_test(stc_can_rxframe_t can_rx_body)
{
		stc_can_txframe_t can_tx_body; 
		if(can_rx_body.Cst.Control_f.IDE == 1) {
				can_tx_body.ExtID = can_rx_body.ExtID;
		} else {
				can_tx_body.StdID = can_rx_body.StdID;
		}
		can_tx_body.Control_f.IDE = can_rx_body.Cst.Control_f.IDE;
		can_tx_body.Control_f.RTR = can_rx_body.Cst.Control_f.RTR;
		can_tx_body.Control_f.DLC = can_rx_body.Cst.Control_f.DLC;
		memcpy(can_tx_body.Data, can_rx_body.Data, 8);
		Iot_Can_Send(can_tx_body);
}
uint8_t buf[256] = {0};
void IOT_cmd_data_send(uint8_t cmd, uint8_t *data, uint16_t len)
{
//		printf("data:%s,len:%d", (char *)data, len);
		uint16_t crc_val;
		uint16_t lenth = 0;
		buf[lenth++] = 0xAA;
		buf[lenth++] = 0x55;
		buf[lenth++] = cmd;
		buf[lenth++] = len >> 8;
		buf[lenth++] = len&0xff;
		if(data != NULL) {
			 memcpy(&buf[lenth], data, len);
		}
		lenth += len;
		crc_val = ble_Package_CheckSum(&buf[2], lenth - 2);
		buf[lenth++] = crc_val &0xff;
		buf[lenth++] = crc_val >> 8;
		if(CMD_GPS_DATA != cmd)
			PRINT_DATA("s:", buf, lenth);
		Uart0_Send_Iot(buf, lenth);
}

void CAN_Rec_Prase(stc_can_rxframe_t stcRxFrame)
{
	uint16_t len = 0;
	uint16_t crc_val;
	uint8_t tx_data[50];
	tx_data[len++] = 0xAA;
	tx_data[len++] = 0x55;
	tx_data[len++] = CMD_CAN_TRANS;
	tx_data[len++] = (sizeof(stc_can_rxframe_t) >> 8) &0xff;
	tx_data[len++] = sizeof(stc_can_rxframe_t) & 0xff;
	memcpy(&tx_data[len], &stcRxFrame, sizeof(stc_can_rxframe_t));
	len += sizeof(stc_can_rxframe_t);
	crc_val = ble_Package_CheckSum(&tx_data[2], len - 2);
	tx_data[len++] = crc_val &0xff;
	tx_data[len++] = crc_val >> 8;
//	PRINT_DATA("IOT_UART_SEND", tx_data, len);
	SET_SYS_TIME(WEEK_TIME, 30000);
//	UART0_DMA_Send(tx_data, len);
	Uart0_Send_Iot(tx_data, len);
}