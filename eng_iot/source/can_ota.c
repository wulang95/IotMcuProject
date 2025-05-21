#include "system.h"
#include "can_ota.h"
#include "app_ota.h"

struct can_ota_con_stu can_ota_con;
	

void ota_flash_data_task()
{
		if(can_ota_con.flash_con.flash_pack_len*128 >= can_ota_con.flash_con.flash_total_len){
				printf("flash data total_len:%d\r\n", can_ota_con.flash_con.flash_total_len);
				
				return;
		}		
		IOT_cmd_data_send(CMD_CAN_OTA_DATA_REQ, (uint8_t *)&can_ota_con.flash_con.flash_pack_len, 2);
}

void ota_flash_init()
{
		can_ota_con.flash_con.flash_total_len = MIN(can_ota_con.rmlen, CAN_OTA_FLASH_SIZE);
		ota_flash_erase(BACK_APP_ADR, BACK_APP_SIZE);
		can_ota_con.flash_con.falsh_offset = 0;
		can_ota_con.ota_step = CAN_FLASH_DATA_STEP;	
		can_ota_con.flash_con.flash_pack_len = 0;
}

void can_ota_rec_prase(stc_can_rxframe_t rx_can_fame)
{
		CAN_PDU_STU can_pdu;
		can_pdu.can_id = rx_can_fame.ExtID;
		if(can_pdu.pdu.da == IOT_ADR && can_pdu.src == can_ota_con.dev_id){
				switch(can_pdu.pdu.pdu1){
					case ENTER_OTA_REQ_CMD:
							if(rx_can_fame.Data[0] == 0x55) {
									if(can_ota_con.ota_step == CAN_OTA_ENTER_STEP) {
											can_ota_con.ota_step = CAN_FLASH_DATA_STEP;
									}
							}
					break;
				}
		}
}

void can_ota_enter()
{
		CAN_PDU_STU can_pdu;
		stc_can_txframe_t can_fame = {0};
		can_pdu.src = IOT_ADR;
		can_pdu.pdu.da = can_ota_con.dev_id;
		can_pdu.pdu.pdu1 = ENTER_OTA_CMD;
		can_pdu.dp = 0;
    can_pdu.r = 0;
    can_pdu.p = 1;
    can_pdu.res = 0;
		can_fame.ExtID = can_pdu.can_id;
		
		can_fame.Control_f.DLC = 8;
		can_fame.Control_f.IDE = 1;
		can_fame.Control_f.RTR = 0;
		
		can_fame.Data[0] = can_ota_con.dev_id;
		can_fame.Data[1] = 0;
		can_fame.Data[2] = 0;
		can_fame.Data[3] = 0;
		can_fame.Data[4] = 0x4E;
		can_fame.Data[5] = 0x4f;
		can_fame.Data[6] = 0x54;
		can_fame.Data[7] = 0x41;
		
		Iot_Can_Send(can_fame);
		
		can_pdu.pdu.da = 0x00;
		can_fame.ExtID = can_pdu.can_id;
		Iot_Can_Send(can_fame);
}	
	
void can_ota_state_check()
{
		switch(can_ota_con.ota_step)
		{
			case CAN_OTA_IDEL_STEP:
				
			break;
			case CAN_OTA_ENTER_STEP:
				can_ota_enter();
			break;
			case CAN_FLASH_INIT_STEP:
				ota_flash_init();
			break;
			case CAN_FLASH_DATA_STEP:
				
			break;
		}
}