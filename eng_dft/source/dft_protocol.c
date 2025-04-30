#include "dft_protocol.h"
#include "system.h"
#include "stdlib.h"

struct dft_mcu_con_stu dft_mcu_con;
void mcu_dft_can_res_send(uint16_t cmd, uint8_t *data, uint8_t len);
void IOT_cmd_data_send(uint8_t cmd, uint8_t *data, uint16_t len);
uint8_t g_cat1_power_flag, g_con_cat1_gpio_flag;

static void dft_gps_get_star_num()
{
		uint8_t buf[256] = {0};
		uint16_t rx_len;
		uint8_t i;
		uint8_t gps_vaild_flag,star_num;
		char *p_str, *last_str;
		if(sys_time[DTF_TM] == 0) {
				star_num = 0xff;
				dft_mcu_con.dft_state = DFT_STATE_IDEL;
				mcu_dft_can_res_send(dft_mcu_table[CAN_DFT_GPS_STAR_NUM].dft_cmd_ask_index, &star_num, dft_mcu_table[CAN_DFT_GPS_STAR_NUM].data_len);
		}
		
		rx_len = FIFO_Valid_Size(GPS_UART);
		if(rx_len > 0) {
				rx_len = MIN(rx_len, 256);
				FIFO_Rece_Buf(GPS_UART, buf, rx_len);
				p_str = strstr((char *)buf, "GGA");
				if(p_str == NULL) return;
			
				p_str = strchr(p_str, ',');
				if(p_str == NULL) return;
	
				p_str = strchr(p_str, ',');
				if(p_str == NULL) return;
			
				p_str = strchr(p_str, ',');
				if(p_str == NULL) return;
			
				p_str = strchr(p_str, ',');
				if(p_str == NULL) return;
			
				p_str = strchr(p_str, ',');
				if(p_str == NULL) return;
			
				p_str = strchr(p_str, ',');
				if(p_str == NULL) return;
				
				last_str = p_str;
				
				p_str = strchr(p_str, ',');
				if(p_str == NULL) return;
				
				gps_vaild_flag = atoi(last_str + 1);
				if(gps_vaild_flag == 0) return;
				
				last_str = p_str;
				
				p_str = strchr(p_str, ',');
				if(p_str == NULL) return;
				star_num = atoi(last_str + 1);
				dft_mcu_con.dft_state = DFT_STATE_IDEL;
				mcu_dft_can_res_send(dft_mcu_table[CAN_DFT_GPS_STAR_NUM].dft_cmd_ask_index, &star_num, dft_mcu_table[CAN_DFT_GPS_STAR_NUM].data_len);
		}
}



static void dft_get_sys_power_vol()
{
		uint8_t data[4] ={0};
		data[0] = (power48v_adc_val >> 24)&0xff;
		data[1] = (power48v_adc_val >> 16)&0xff;
		data[2] = (power48v_adc_val >> 8)&0xff;
		data[3] = power48v_adc_val&0xff;
 		mcu_dft_can_res_send(dft_mcu_table[CAN_DFT_SYSPOWER_ADC].dft_cmd_ask_index, data, dft_mcu_table[CAN_DFT_SYSPOWER_ADC].data_len);
		dft_mcu_con.dft_state = DFT_STATE_IDEL;
}

static void dft_get_backbat_vol()
{
		uint8_t data[4] ={0};
		data[0] = (bat_adc_val >> 24)&0xff;
		data[1] = (bat_adc_val >> 16)&0xff;
		data[2] = (bat_adc_val >> 8)&0xff;
		data[3] = bat_adc_val&0xff;
 		mcu_dft_can_res_send(dft_mcu_table[CAN_DFT_BACKBAT_ADC].dft_cmd_ask_index, data, dft_mcu_table[CAN_DFT_BACKBAT_ADC].data_len);
		dft_mcu_con.dft_state = DFT_STATE_IDEL;
}

static void dft_get_bat_temp_adc()
{
		uint8_t data[4] ={0};
		data[0] = (bat_temp_adc_val >> 24)&0xff;
		data[1] = (bat_temp_adc_val >> 16)&0xff;
		data[2] = (bat_temp_adc_val >> 8)&0xff;
		data[3] = bat_temp_adc_val&0xff;
 		mcu_dft_can_res_send(dft_mcu_table[CAN_DFT_BACKTEMP_ADC].dft_cmd_ask_index, data, dft_mcu_table[CAN_DFT_BACKTEMP_ADC].data_len);
		dft_mcu_con.dft_state = DFT_STATE_IDEL;
}
	
static void dft_mcu_cat1_con_gpio()
{
		static uint8_t step = 0;
		uint8_t res  = DFT_FAIL;
		if(CHECK_SYS_TIME(DTF_TM)== 0) {
				step = 0;
				res = DFT_FAIL;
				dft_mcu_con.dft_state = DFT_STATE_IDEL;
				mcu_dft_can_res_send(dft_mcu_table[CAN_DFT_CAT1_CONN_GPIO].dft_cmd_ask_index, &res, dft_mcu_table[CAN_DFT_CAT1_CONN_GPIO].data_len);
		}
		switch(step) {
			case 0:
					g_con_cat1_gpio_flag = 0;
					IOT_cmd_data_send(CMD_DFT_CON_GPIO, NULL, 0);
					SET_SYS_TIME(DFT_FUN_TM, 500);
				  step = 1;
			break;
			case 1:
				if(CHECK_SYS_TIME(DFT_FUN_TM) == 0) {
						step = 0;
						res = DFT_FAIL;
						dft_mcu_con.dft_state = DFT_STATE_IDEL;
						mcu_dft_can_res_send(dft_mcu_table[CAN_DFT_CAT1_CONN_GPIO].dft_cmd_ask_index, &res, dft_mcu_table[CAN_DFT_CAT1_CONN_GPIO].data_len);
				} else if(g_con_cat1_gpio_flag == 1) {
					 step = 2;
				}
				break;
			case 2:
				Gpio_SetIO(GpioPortA, GpioPin5);
				SET_SYS_TIME(DFT_FUN_TM, 100);
				step = 3;
			break;
			case 3:
				if(CHECK_SYS_TIME(DFT_FUN_TM) == 0) {
						step = 0;
						res = DFT_FAIL;
						dft_mcu_con.dft_state = DFT_STATE_IDEL;
						mcu_dft_can_res_send(dft_mcu_table[CAN_DFT_CAT1_CONN_GPIO].dft_cmd_ask_index, &res, dft_mcu_table[CAN_DFT_CAT1_CONN_GPIO].data_len);
				} else if(Gpio_GetInputIO(GpioPortA, GpioPin6) == 1) {
						Gpio_ClrIO(GpioPortA, GpioPin5);
						SET_SYS_TIME(DFT_FUN_TM, 100);
					  step = 4;
				}
			break;
			case 4:
					if(CHECK_SYS_TIME(DFT_FUN_TM) == 0) {
						step = 0;
						res = DFT_FAIL;
						dft_mcu_con.dft_state = DFT_STATE_IDEL;
						mcu_dft_can_res_send(dft_mcu_table[CAN_DFT_CAT1_CONN_GPIO].dft_cmd_ask_index, &res, dft_mcu_table[CAN_DFT_CAT1_CONN_GPIO].data_len);
				} else if(Gpio_GetInputIO(GpioPortA, GpioPin6) == 1) {						
					  step = 0;
						res = DFT_OK;
						dft_mcu_con.dft_state = DFT_STATE_IDEL;
						mcu_dft_can_res_send(dft_mcu_table[CAN_DFT_CAT1_CONN_GPIO].dft_cmd_ask_index, &res, dft_mcu_table[CAN_DFT_CAT1_CONN_GPIO].data_len);
				}
			break;
		}
}

void dft_sys_power_det()
{
		uint8_t power_sta;
		power_sta =  Gpio_GetInputIO(GpioPortA, GpioPin1);
		mcu_dft_can_res_send(dft_mcu_table[CAN_DFT_SYSPOWER_DET].dft_cmd_ask_index, &power_sta, dft_mcu_table[CAN_DFT_SYSPOWER_DET].data_len);
		dft_mcu_con.dft_state = DFT_STATE_IDEL;
}


struct dft_mcu_item_stu dft_mcu_table[] = {
	{0x0000, 0x0000, 1, 20000, dft_gps_get_star_num},
	{0x0001, 0x0001, 4, 3000, dft_get_sys_power_vol},
	{0x0002, 0x0002, 4, 3000, dft_get_backbat_vol},
	{0x0003, 0x0003, 4, 3000, dft_get_bat_temp_adc},
	{0x0004, 0x0004, 1, 3000, dft_mcu_cat1_con_gpio},
	{0x0005, 0x0005, 1, 3000, dft_sys_power_det},
};


int dft_find_item(uint16_t cmd_index)
{
		uint8_t i;
		for(i = 0; i < CAN_DFT_MAX; i++){
				if(cmd_index == dft_mcu_table[i].dft_cmd_index) {
						break;
				}
		}
		if(i == CAN_DFT_MAX) {
			return DFT_FAIL;
		} else {
			dft_mcu_con.dft_cmd = i;
			return DFT_OK;
		}
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


void IOT_cmd_data_send(uint8_t cmd, uint8_t *data, uint16_t len)
{
//		printf("data:%s,len:%d", (char *)data, len);
		uint8_t buf[256] = {0};
		uint16_t crc_val;
		uint16_t lenth = 0;
		buf[lenth++] = 0xAA;
		buf[lenth++] = 0x55;
		buf[lenth++] = cmd;
		buf[lenth++] = len >> 8;
		buf[lenth++] = len&0xff;
		if(data != NULL) {
				memcpy(&buf[lenth], data, len);
				lenth += len;
		}
		crc_val = ble_Package_CheckSum(&buf[2], lenth - 2);
		buf[lenth++] = crc_val &0xff;
		buf[lenth++] = crc_val >> 8;
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
			case CMD_DFT_CAT1_SIGN:
					g_cat1_power_flag = 1;
			break;
			case CMD_DFT_CON_GPIO:
					g_con_cat1_gpio_flag = 1;
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

uint8_t gps_ver_cmd[] = {0x42, 0x4b, 0x68, 0xf8, 0x02, 0x07, 0x00, 0x00};




void mcu_dft_can_res_send(uint16_t cmd, uint8_t *data, uint8_t len)
{
		stc_can_txframe_t stcTxFrame;
		CAN_PDU_STU can_id;
		can_id.src = MCU_ID;
    can_id.p = 6;
    can_id.r = 0;
    can_id.dp = 0;
    can_id.res = 0;
    can_id.pdu.da = DFT_ID;
		can_id.pdu.pdu1 = DFT_CAN_CMD_ACK;
		
		memset(&stcTxFrame, 0, sizeof(stcTxFrame));
		stcTxFrame.ExtID = can_id.can_id;
		stcTxFrame.Data[0] = (cmd >> 8)&0xff;
		stcTxFrame.Data[1] = cmd & 0xff;
		stcTxFrame.Data[2] = len;
		memcpy(&stcTxFrame.Data[3], data, len);
		stcTxFrame.Data[7] = can_check_sum(&stcTxFrame.Data[0], 7);
		Iot_Can_Send(stcTxFrame);
}

void dft_can_png_rqust(uint16_t png)
{
		switch(png){
			case CAN_DFT_GPS_VAR:
					
			break;
		}
}

void mcu_rx_can_handle(stc_can_rxframe_t stcRxFrame)
{
		uint8_t res;
		CAN_PDU_STU can_id;
		uint8_t check_u8;
		uint16_t cmd_index;
		uint16_t can_pgn;
		can_id.can_id = stcRxFrame.ExtID;
		if(can_id.pdu.da != MCU_ID || can_id.pdu.pdu2 >=  0xf000) return;
		if(can_id.pdu.pdu1 == DFT_CAN_CMD) {
				check_u8 = can_check_sum(stcRxFrame.Data, 7);
				if(check_u8 == stcRxFrame.Data[7]) {
						cmd_index = stcRxFrame.Data[0] << 8 | stcRxFrame.Data[1];
					  if(dft_mcu_con.dft_state == DFT_STATE_IDEL) {
								if(dft_find_item(cmd_index) == DFT_OK) {
										dft_mcu_con.dft_state = DFT_STATE_MCU;
										SET_SYS_TIME(DTF_TM, dft_mcu_table[dft_mcu_con.dft_cmd].dft_timeout);
								}
						}
				}
		} else if(can_id.pdu.pdu1 == DFT_CAN_R_PNG) {
				can_pgn = stcRxFrame.Data[1] << 8 | stcRxFrame.Data[0];
		}
}

void cat1_rx_can_handle(stc_can_rxframe_t stcRxFrame)
{
		CAN_Rec_Prase(stcRxFrame);
}


void can_rec_data_handle(stc_can_rxframe_t stcRxFrame)
{
		CAN_PDU_STU can_id;
		can_id.can_id = stcRxFrame.ExtID;
		if(can_id.pdu.pdu2 >= 0xf000){
			
		} else {
				switch(can_id.pdu.da) {
					case MCU_ID:
						mcu_rx_can_handle(stcRxFrame);
					break;
					case CAT1_ID:
						cat1_rx_can_handle(stcRxFrame);
					break;
				}
		}
}