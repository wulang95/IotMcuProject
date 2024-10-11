#include "system.h"


uint8_t gps_data_check_res(uint8_t *s, uint16_t len)
{
		uint16_t i = 0;
		uint8_t check_res; 
		check_res = s[0];
		for(i = 1; i < len; i++){
				check_res = check_res^s[i];
		}
		return check_res;
}
int hextoint(const char *hexStr)
{
		int res;
		sscanf(hexStr, "%x", &res);
		return res;
}
char pos_data[80];
GPS_DATA gps_info_buf;

uint8_t GGA_info_prase(char *pstart, uint16_t len, GPS_DATA *gps_info)
{
		/*$GNGGA,093314.00,3110.4880379,N,12135.9872231,E,1,37,0.5,17.362,M,0.000,M,,*74<CR><LF>*/
		char *p_s, *p_e;
		if(len <= 40) return ERROR;
		p_s = strchr(pstart, ',');
		p_s = p_s + 1;
		p_e = strchr(p_s, ',');
		memcpy(gps_info->Time1, p_s, p_e - p_s);
		p_s = p_e+1;
		p_e = strchr(p_s, ',');
		p_e = strchr(p_e + 1, ',');
		p_e = strchr(p_e + 1, ',');
		p_e = strchr(p_e + 1, ',');	
		memcpy(gps_info->LatLongData, p_s, p_e - p_s);
		p_s = strchr(p_e + 1, ',');
		p_s = p_s + 1;
		p_e = strchr(p_s, ',');
		memcpy(gps_info->SateNumStr, p_s, p_e - p_s);
		p_s = strchr(p_e + 1, ',');
		p_s = p_s + 1;
		p_e = strchr(p_s, ',');
		memcpy(gps_info->HDOP, p_s, p_e - p_s);
		p_s = strchr(p_e + 1, ',');
		p_s = strchr(p_s + 1, ',');
	
		p_s = p_s + 1; 
		p_e = strchr(p_s, ',');
		memcpy(gps_info->SeaLevelH, p_s, p_e - p_s);
		p_s = strchr(p_e + 1, ',');
		p_s = p_s + 1; 
		p_e = strchr(p_s, ',');
		strcat(gps_info->SeaLevelH, ",");
		strncat(gps_info->SeaLevelH, p_s, p_e - p_s);
		return SUCCESS;
}

uint8_t RMC_info_prase(char *pstart, uint16_t len, GPS_DATA *gps_info)
{
		/*$GNRMC,093314.00,A,3110.4880379,N,12135.9872231,E,3.09,30.61,090222,,,A,V*09<CR><LF>*/
		char *p_s, *p_e;
		if(len <= 32) return ERROR;
		p_s = strchr(pstart, ',');
		p_s = p_s + 1;
		p_e = strchr(p_s, ',');
		memcpy(gps_info->Time1, p_s, p_e - p_s); 
		
		p_s = p_e + 1;
		p_e = strchr(p_s, ',');
		if(p_s[0] == 'A') gps_info->GPSValidFlag = 1;
		p_s = p_e + 1;
		p_e = strchr(p_s, ',');
		p_e = strchr(p_e+1, ',');
		p_e = strchr(p_e+1, ',');
		p_e = strchr(p_e+1, ',');
		memcpy(gps_info->LatLongData, p_s, p_e - p_s);
		p_s = strchr(p_e+1, ',');
		p_s = strchr(p_e+1, ',');
		p_s = p_s + 1;
		p_e = strchr(p_s, ',');
		memcpy(gps_info->Time2, p_s, p_e - p_s);
		p_s = strchr(p_e + 1, ',');
		p_s = strchr(p_s + 1, ',');
		p_s = p_s + 1;
		p_e = strchr(p_s, ',');
		memcpy(gps_info->Mode, p_s, p_e - p_s);
		return SUCCESS;
}

uint8_t GLL_info_prase(char *pstart, uint16_t len, GPS_DATA *gps_info)
{
		/*$GPGLL, 3110.4880379,N,00833.91565,E,093314.00,A,A*60\r\n*/
		char *p_s, *p_e;
		if(len <= 25) return ERROR;
		p_s = strchr(pstart, ',');
		p_s = p_s + 1;
		p_e = strchr(p_s, ',');
		p_e = strchr(p_e+1, ',');
		p_e = strchr(p_e+1, ',');
		p_e = strchr(p_e+1, ',');
		memcpy(gps_info->LatLongData, p_s, p_e - p_s);
		p_s = p_e + 1;
		p_e = strchr(p_s, ',');
		memcpy(gps_info->Time1, p_s, p_e - p_s);
		p_s = p_e+1;
		p_s = strchr(p_s, ',');
		p_s = p_s + 1;
		p_e = strchr(p_s, ',');
		if(p_s[0] == 'A')gps_info->GPSValidFlag = 1;
		return SUCCESS;
}

char *cmd_NAME_OFF = "$POLCFGMSG,1,0\r\n";
char *cmd_GGA_ON = "$POLCFGMSG,0,0,1\r\n";
char *cmd_SAVE = "$POLCFGSAVE\r\n";
char *cmd_NAME_5HZ = "$POLCFGMSG,1,5\r\n";
char *cmd_NAME_1HZ = "$POLCFGMSG,1,1\r\n";
char *cmd_GGA_OFF = "$POLCFGMSG,0,0,0\r\n";
char *cmd_RMC_ON = "$POLCFGMSG,0,5,1\r\n";
char *cmd_RMC_OFF = "$POLCFGMSG,0,5,0\r\n";
char *cmd_GLL_ON = "$POLCFGMSG,0,13,1\r\n";
char *cmd_GLL_OFF = "$POLCFGMSG,0,13,0\r\n";

static uint8_t gps_cmd_pwr_deepsleep[] = {0x42, 0x4b, 0x51, 0x05, 0x00, 0x03, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t gps_cmd_ctrl_start[] = {0x42, 0x4b, 0x0c, 0x4a, 0x02, 0x05, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};

uint8_t gps_send_cmd_str(char *cmd_str)
{
		uint8_t buf[100] = {0};
		uint16_t rx_len;
		uint16_t real_len;
		uint8_t i = 0;
		FIFO_Clean_Buf(GPS_UART);
		Uart1_Send_gps((uint8_t *)cmd_str, strlen(cmd_str));
		for(i = 0; i < 10; i++){
				delay1ms(200);
				rx_len = FIFO_Valid_Size(GPS_UART);
				real_len = MIN(rx_len, 100);
				if(real_len) {
						FIFO_Rece_Buf(GPS_UART, buf, real_len);
						printf("rec[%d]:%s\r\n", real_len, (char *)buf);
						if(strstr((char *)buf, "$OK") != NULL) {
								break;
						}
				}
				FIFO_Clean_Buf(GPS_UART);
				Uart1_Send_gps((uint8_t *)cmd_str, strlen(cmd_str));
		}
		if(i == 10){
			printf("cmd:%s gps cmd send error\r\n", cmd_str);
		}
}

void GPS_init()
{
		uint8_t buf[100] = {0};
		uint16_t rx_len;
		uint16_t real_len;
		uint8_t i = 0;
		stc_gpio_cfg_t stcGpioCfg;
		Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
	
		stcGpioCfg.enDir = GpioDirOut;
		stcGpioCfg.enDrv = GpioDrvL;
		stcGpioCfg.enPu = GpioPuDisable;
		stcGpioCfg.enPd = GpioPdDisable;
		stcGpioCfg.enOD = GpioOdDisable;
		stcGpioCfg.enCtrlMode = GpioAHB;
		
		Gpio_Init(GpioPortB, GpioPin11, &stcGpioCfg); //GPSPOWER
		Gpio_Init(GpioPortB, GpioPin10, &stcGpioCfg); //GPSRST
		Gpio_ClrIO(GpioPortB, GpioPin11);
		FIFO_Clean_Buf(GPS_UART);
		while(i < 10){
				rx_len = FIFO_Valid_Size(GPS_UART);
				printf("rx_len:%d\r\n", rx_len);
				real_len = MIN(rx_len, 100);
				if(real_len){
					FIFO_Rece_Buf(GPS_UART, buf, real_len);
					printf("rec[%d]:%s\r\n", real_len, buf);
					if(strstr((char *)buf, "1660") != NULL){
							printf("GPS init ok\r\n");
							break;
					}
				}
				Gpio_SetIO(GpioPortB, GpioPin11);
				delay1ms(200);
				Gpio_ClrIO(GpioPortB, GpioPin11);
				FIFO_Clean_Buf(GPS_UART);
				delay1ms(300);
				i++;
				printf("i:%d\r\n", i);
				Wdt_Feed();
		}
		if(i == 10) {
			Gpio_SetIO(GpioPortB, GpioPin11);
			printf("gps init is fail\r\n");
			return;
		}
		Wdt_Feed();
		gps_send_cmd_str(cmd_NAME_OFF);
		Wdt_Feed();
		gps_send_cmd_str(cmd_NAME_1HZ);
		Wdt_Feed();
		gps_send_cmd_str(cmd_NAME_OFF);
		Wdt_Feed();
		gps_send_cmd_str(cmd_GGA_ON);
		Wdt_Feed();
		gps_send_cmd_str(cmd_SAVE);
}

void gps_cmd_name_switch()
{
		static uint8_t step = 0;
				switch(step)
				{
					case 0:
						Uart1_Send_gps((uint8_t *)cmd_GGA_ON, strlen(cmd_GGA_ON));
						Uart1_Send_gps((uint8_t *)cmd_RMC_OFF, strlen(cmd_RMC_OFF));
						Uart1_Send_gps((uint8_t *)cmd_GLL_OFF, strlen(cmd_GLL_OFF));
						break;
					case 1:
						Uart1_Send_gps((uint8_t *)cmd_GGA_OFF, strlen(cmd_GGA_OFF));
						Uart1_Send_gps((uint8_t *)cmd_RMC_ON, strlen(cmd_RMC_ON));
						Uart1_Send_gps((uint8_t *)cmd_GLL_OFF, strlen(cmd_GLL_OFF));
						break;
					case 2:
						Uart1_Send_gps((uint8_t *)cmd_GGA_OFF, strlen(cmd_GGA_OFF));
						Uart1_Send_gps((uint8_t *)cmd_RMC_OFF, strlen(cmd_RMC_OFF));
						Uart1_Send_gps((uint8_t *)cmd_GLL_ON, strlen(cmd_GLL_ON));
						break;
				}
				step++;
				if(step == 3) step = 0;
}


void GPS_host_start_cmd()
{
		Uart1_Send_gps(gps_cmd_ctrl_start, sizeof(gps_cmd_ctrl_start));
		FIFO_Clean_Buf(GPS_UART);
}

void GPS_deep_sleep_cmd()
{
		Uart1_Send_gps(gps_cmd_pwr_deepsleep, sizeof(gps_cmd_pwr_deepsleep));
		FIFO_Clean_Buf(GPS_UART);
}
void GPS_power_on()
{
		Gpio_ClrIO(GpioPortB, GpioPin11);
}

void GPS_power_off()
{
		Gpio_SetIO(GpioPortB, GpioPin11);
}

void GPS_Control()
{
		uint8_t data[256] = {0};
		uint16_t rx_len;
		uint16_t real_len;
		uint16_t tx_len = 0;
		char *p_start, *p_end, check_str[2];
		char *type;
		uint8_t vaild_flag = 0;
		uint8_t check_res, rec_check_res;
		static GPS_DATA gps_info = {0};
		if(CHECK_SYS_TIME(GPS_TM)) return;
		SET_SYS_TIME(GPS_TM, 500);
		rx_len = FIFO_Valid_Size(GPS_UART);
		if(rx_len <= 0) return;
		rx_len = MIN(rx_len, 256);
		FIFO_Rece_Buf(GPS_UART, data, rx_len);
		SET_SYS_TIME(WEEK_TIME, 10000);
		printf("GPS rec[%d]:%s\r\n", rx_len, data);
		p_start = strchr((char *)data, '$');
		if(p_start == NULL) return;
		p_end = strchr((char *)data, '*');
		if(p_end == NULL)	return;
		check_res = gps_data_check_res((uint8_t *)p_start + 1, p_end - p_start-1);
		memcpy(check_str, p_end+1, 2);
		rec_check_res = hextoint(check_str);
		if(rec_check_res != check_res) return;
		type = strstr(p_start, "GGA");
		if(type != NULL) {
			 if(GGA_info_prase(p_start, p_end - p_start, &gps_info)== SUCCESS) 
					vaild_flag = 1;
		} 
		
		type = strstr(p_start, "RMC");
		if(type != NULL) {
				if(RMC_info_prase(p_start, p_end - p_start, &gps_info) == SUCCESS)
					vaild_flag = 1;
		}
	
		type = strstr(p_start, "GLL");
		if(type != NULL) {
				if(GLL_info_prase(p_start, p_end - p_start, &gps_info) == SUCCESS)
					vaild_flag = 1;
		}	
		gps_cmd_name_switch();
		if(vaild_flag == 1) 
		{
				tx_len += sprintf(pos_data, "%s,", gps_info.Time1);
				if(gps_info.GPSValidFlag) {
						tx_len += sprintf(pos_data + tx_len, "%c,", 'A');
				} else {
						tx_len += sprintf(pos_data + tx_len, "%c,", 'V');
				}
				if(strlen(gps_info.LatLongData) == 0) 
						tx_len += sprintf(pos_data + tx_len, "%s,", ",,,");
				else {
						tx_len += sprintf(pos_data + tx_len, "%s,", gps_info.LatLongData);
				}
				tx_len += sprintf(pos_data + tx_len, "%s,", gps_info.SateNumStr);
				tx_len += sprintf(pos_data + tx_len, "%s,", gps_info.HDOP);
				tx_len += sprintf(pos_data + tx_len, "%s,", gps_info.Time2);
				tx_len += sprintf(pos_data + tx_len, "%s,", gps_info.SeaLevelH);
				tx_len += sprintf(pos_data + tx_len, "%s", gps_info.Mode);
		} else {
				memset(&gps_info, 0, sizeof(gps_info));
				tx_len += sprintf(pos_data, "%s", ",V,,,,,,,,,,N");
		}
		IOT_cmd_data_send(CMD_GPS_DATA, (uint8_t *)pos_data, tx_len);
}

