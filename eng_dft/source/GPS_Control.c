#include "system.h"
#include "stdlib.h"

uint8_t star_num;
uint8_t gps_data_check_res(uint8_t *s, uint16_t len)
{
		uint16_t i = 0;
		uint8_t check_res; 
		check_res = s[0];
		char buf[128]= {0};
		memcpy(buf, (char *)s, len);
		printf("buf:%s\r\n", buf);
//		for(i = 1; i < len; i++){
//				check_res = check_res^s[i];
//		}
//		return check_res;
}
int hextoint(const char *hexStr)
{
		int res;
		sscanf(hexStr, "%x", &res);
		return res;
}

GPS_DATA gps_info_buf;
//len:74
uint8_t GGA_info_prase(char *pstart, uint16_t len, GPS_DATA *gps_info)
{
		/*$GNGGA,093314.00,3110.4880379,N,12135.9872231,E,1,37,0.5,17.362,M,0.000,M,,*74<CR><LF>*/
		
		char *p_s, *p_e;
		if(len <= 60) return ERROR;
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

		p_s = p_e + 1;
		p_e = strchr(p_s, ',');
		memcpy(gps_info->HDOP, p_s, p_e - p_s);
		p_s = p_e + 1;
		p_e = strchr(p_s, ',');
		p_e = strchr(p_e+1, ',');
		p_s = p_e + 1;
		p_e = strchr(p_s+1, ',');
		p_e = strchr(p_e+1, ','); 
		memcpy(gps_info->SeaLevelH, p_s, p_e - p_s);
		return SUCCESS;
}
//71
uint8_t RMC_info_prase(char *pstart, uint16_t len, GPS_DATA *gps_info)
{
		/*$GNRMC,093314.00,A,3110.4880379,N,12135.9872231,E,3.09,30.61,090222,,,A,V*09<CR><LF>*/
		char *p_s, *p_e;
		if(len <= 60) return ERROR;
		p_s = strchr(pstart, ',');
		p_s = p_s + 1;
		p_e = strchr(p_s, ',');
		memcpy(gps_info->Time1, p_s, p_e - p_s); 
		p_s = p_e + 1;
		p_e = strchr(p_s, ',');
		gps_info->GPSValidFlag = p_s[0];
		p_s = p_e + 1;
		p_e = strchr(p_s, ',');
		p_e = strchr(p_e+1, ',');
		p_e = strchr(p_e+1, ',');
		p_e = strchr(p_e+1, ',');
		memcpy(gps_info->LatLongData, p_s, p_e - p_s);
		p_s = strchr(p_e+1, ',');
		p_s = strchr(p_s+1, ',');
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
//73
uint8_t GLL_info_prase(char *pstart, uint16_t len, GPS_DATA *gps_info)
{
		/*$GPGLL, 3110.4880379,N,00833.91565,E,093314.00,A,A*60\r\n*/
		char *p_s, *p_e;
		if(len <= 35) return ERROR;
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
		gps_info->GPSValidFlag = p_s[0];
		return SUCCESS;
}
char *cmd_NAME_ON = "$POLCFGMSG,1,1\r\n";
char *cmd_NAME_OFF = "$POLCFGMSG,1,0\r\n";
char *cmd_GGA_ON = "$POLCFGMSG,0,0,1\r\n";
char *cmd_GSA_ON = "$POLCFGMSG,0,1,1\r\n";
char *cmd_GSV_ON = "$POLCFGMSG,0,2,1\r\n";
char *cmd_VTG_ON = "$POLCFGMSG,0,3,1\r\n";
char *cmd_CNR_ON = "$POLCFGMSG,0,4,1\r\n";
char *cmd_CLK_ON = "$POLCFGMSG,0,6,1\r\n";
char *cmd_POL_ON = "$POLCFGMSG,0,7,1\r\n";
char *cmd_THS_ON = "$POLCFGMSG,0,8,1\r\n";
char *cmd_ANT_ON = "$POLCFGMSG,0,9,1\r\n";
char *cmd_JAM_ON = "$POLCFGMSG,0,10,1\r\n";
char *cmd_INS_ON = "$POLCFGMSG,0,11,1\r\n";
char *cmd_GST_ON = "$POLCFGMSG,0,12,1\r\n";
char *cmd_AID_ON = "$POLCFGMSG,0,14,1\r\n";
char *cmd_MSM_ON = "$POLCFGMSG,0,15,1\r\n";
char *cmd_GMP_ON = "$POLCFGMSG,0,16,1\r\n";
char *cmd_SAVE = "$POLCFGSAVE\r\n";
char *cmd_NAME_5HZ = "$POLCFGMSG,1,5\r\n";
char *cmd_NAME_1HZ = "$POLCFGMSG,1,1\r\n";
char *cmd_GGA_OFF = "$POLCFGMSG,0,0,0\r\n";
char *cmd_RMC_ON = "$POLCFGMSG,0,5,1\r\n";
char *cmd_RMC_OFF = "$POLCFGMSG,0,5,0\r\n";
char *cmd_GLL_ON = "$POLCFGMSG,0,13,1\r\n";
char *cmd_GLL_OFF = "$POLCFGMSG,0,13,0\r\n";
void GPS_factory_cmd();
static uint8_t gps_cmd_pwr_deepsleep[] = {0x42, 0x4b, 0x51, 0x05, 0x00, 0x03, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t gps_cmd_ctrl_start[] = {0x42, 0x4b, 0x0c, 0x4a, 0x02, 0x05, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
static uint8_t gps_cmd_factory[] = {0x42, 0x4b, 0x49, 0xc4, 0x02, 0x0c, 0x00, 0x04, 0x00, 0x00, 0xff, 0xff};
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
char gps_ver[16];
void GPS_get_ver()
{
		uint8_t buf[100] = {0}, check_res, check_str[2] = {0},rec_check_res;
		uint16_t rx_len;
		uint16_t real_len;
		uint8_t i = 0;
		char *p_start,*p_end,*str;
		uint8_t gps_ver_table[] = {0x42, 0x4b, 0x68, 0xf8, 0x02, 0x07, 0x00, 0x00};
		FIFO_Clean_Buf(GPS_UART);
		Uart1_Send_gps(gps_ver_table, sizeof(gps_ver_table));
		for(i = 0; i < 10; i++){
				delay1ms(200);
				rx_len = FIFO_Valid_Size(GPS_UART);
				real_len = MIN(rx_len, 100);
				if(real_len) {
						FIFO_Rece_Buf(GPS_UART, buf, real_len);
						p_start = strstr((char *)buf,"POLRS"); 
						check_res = p_start[0];
					  i = 1;
						if(p_start != NULL){
								p_end = strchr(p_start, '*');
								while(p_start[i] != *p_end) {
									check_res = check_res^p_start[i];
									i++;
								}
								memcpy(check_str, p_end+1, 2);
								rec_check_res = hextoint((char *)check_str);
								if(rec_check_res == check_res) {
										str = strchr(p_start, ',');
										str++;
										i=0;
										while(*str != ',') {
												gps_ver[i++] = *str;
												str++;
										}
										printf("gps_ver:%s\r\n", gps_ver);
										break;
								}
						}
						printf("============ver: rec[%d]:%s\r\n", real_len, (char *)buf);
						if(strstr((char *)buf, "$OK") != NULL) {
								break;
						}
				}
				FIFO_Clean_Buf(GPS_UART);
				Uart1_Send_gps(gps_ver_table, sizeof(gps_ver_table));
		}
		if(i == 10){
			printf("cmd:%s gps cmd send error\r\n", gps_ver_table);
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
		star_num = 0;
		stcGpioCfg.enDir = GpioDirOut;
		stcGpioCfg.enDrv = GpioDrvL;
		stcGpioCfg.enPu = GpioPuDisable;
		stcGpioCfg.enPd = GpioPdDisable;
		stcGpioCfg.enOD = GpioOdDisable;
		stcGpioCfg.enCtrlMode = GpioAHB;
		
		Gpio_Init(GpioPortB, GpioPin11, &stcGpioCfg); //GPSPOWER
		Gpio_Init(GpioPortB, GpioPin10, &stcGpioCfg); //GPSRST
		Gpio_SetIO(GpioPortB, GpioPin11);
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
				Gpio_ClrIO(GpioPortB, GpioPin11);
				delay1ms(200);
				Gpio_SetIO(GpioPortB, GpioPin11);
				FIFO_Clean_Buf(GPS_UART);
				delay1ms(600);
				i++;
				printf("i:%d\r\n", i);
				Wdt_Feed();
		}
		if(i == 10) {
			Gpio_ClrIO(GpioPortB, GpioPin11);
			printf("gps init is fail\r\n");
			return;
		}
		memset(gps_ver, 0, 16);
		GPS_get_ver();
		Wdt_Feed();
		gps_send_cmd_str(cmd_NAME_OFF);
		Wdt_Feed();
		gps_send_cmd_str(cmd_NAME_1HZ);
		Wdt_Feed();
		gps_send_cmd_str(cmd_NAME_OFF);
		Wdt_Feed();
//		gps_send_cmd_str(cmd_GSV_ON);
//		Wdt_Feed();
		gps_send_cmd_str(cmd_GGA_ON);
		Wdt_Feed();
//		gps_send_cmd_str(cmd_RMC_ON);
//		Wdt_Feed();
		gps_send_cmd_str(cmd_SAVE);
		#if GPS_TEST==1 
		gps_send_cmd_str(cmd_NAME_ON);
		gps_send_cmd_str(cmd_GSA_ON);
		gps_send_cmd_str(cmd_GSV_ON);
		Wdt_Feed();
		gps_send_cmd_str(cmd_VTG_ON);
		gps_send_cmd_str(cmd_CNR_ON);
		gps_send_cmd_str(cmd_CLK_ON);
		gps_send_cmd_str(cmd_POL_ON);
		Wdt_Feed();
		gps_send_cmd_str(cmd_THS_ON);
		gps_send_cmd_str(cmd_ANT_ON);
		gps_send_cmd_str(cmd_JAM_ON);
		Wdt_Feed();
		gps_send_cmd_str(cmd_INS_ON);
		gps_send_cmd_str(cmd_GST_ON);
		Wdt_Feed();
		gps_send_cmd_str(cmd_AID_ON);
		gps_send_cmd_str(cmd_MSM_ON);
		gps_send_cmd_str(cmd_GMP_ON);
		Wdt_Feed();
		gps_send_cmd_str(cmd_SAVE);
		GPS_factory_cmd();
		#endif
	//	GPS_deep_sleep_cmd();
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

void GPS_factory_cmd()
{
		Uart1_Send_gps(gps_cmd_factory, sizeof(gps_cmd_factory));
		FIFO_Clean_Buf(GPS_UART);
}
void GPS_power_on()
{
		Gpio_SetIO(GpioPortB, GpioPin11);
}

void GPS_power_off()
{
		Gpio_ClrIO(GpioPortB, GpioPin11);
		FIFO_Clean_Buf(GPS_UART);
}

static uint8_t GPS_GGA_Proces(char *Data)
{
    uint8_t  DataLen, i;
    char *ps, *pe, Buf[64], LatBuf[64];
 //   printf("\n**********GPS_GGA_Proces***********\n");

    //	        0           1  2            3  4  5   6     7      8  9     10 11 12  13
    //063717.00, 2238.07773, N, 11407.55384, E, 1, 09, 1.02, 101.3,  M, -2.2, M,  ,  *
    //$GNGGA,160957.00,,,,,0,00,99.99,,,,,,*74
    ps = Data;                                       
    for(i = 0; i < 12; i++)
    {
        pe = strstr(ps, ",");
        if(pe == NULL)
        {
            pe = strstr(ps, "*");                      
            if(pe == NULL)  return -1;
        }

        #if 0                                         
        if(pe == ps)                                  
        {
            ps = pe + 1;                             
            continue;
        }
        #endif


        DataLen = pe - ps;
        memset(Buf, 0, sizeof(Buf));
        if(DataLen < sizeof(Buf))	memcpy(Buf, ps, DataLen); 
        else return -1;                          

    //    LOG_I("i:%d  DataLen:%d  Buf:%s\n", i, DataLen, Buf);

        switch(i)
        {
            case 1:                                        
            case 2:
            case 3:
            case 4:
            case 5:
							break;
            case 6:
                if(Buf[0] == '0') {
									star_num = 0;
									return -1;
								}
              break;
            case 7:                                            
                if(DataLen >= 6) 
                    return -1;
                star_num = atoi((char *)Buf);
							//	printf("star_num:%d\r\n", star_num);
                return 0;                                          

        }
        ps = pe + 1;                                      
    }
    return 0;
}

uint16_t  GPS_FloatStr_To_Num(char *Data)
{
    uint16_t  Num = 0;
    char Buf1[8], Buf2[8];
    //               字符串长度错误                    字符串为空        返回最大值
    if((strlen(Data) == 0) || (strlen(Data) >= 8) || (Data == NULL))   return 0xFFFF;

    memset(Buf1, 0, sizeof(Buf1));
    memset(Buf2, 0, sizeof(Buf2));
    while(*Data)
    {
        if(*Data == '.')
        {
            strcpy(Buf2, Data + 1);
            Num  = atoi(Buf1) * 100;
            if((Buf2[0] >= '0') && (Buf2[0] <= '9'))
                Num += (Buf2[0] - '0') * 10;
            if((Buf2[1] >= '0') && (Buf2[1] <= '9'))
                Num += Buf2[1] - '0';
            return Num;
        }
        else
        {
            Buf1[Num++] = *Data;
        }
        Data++;
    }
    return 0xFFFF;
}

char gps_buff[512];
uint16_t gps_data_offset;
void GPS_data_task()
{
		uint8_t check_res, rec_check_res, i =0;
		char *p_start, *p_end, check_str[2] ={0};
		p_start = strchr(&gps_buff[gps_data_offset], '$');
		if(p_start == NULL) return;
		p_end = strchr(p_start, '*');
		if(p_end == NULL)	return;
		p_start = p_start +1;
		check_res = p_start[i++];
//		gps_data_offset = gps_data_offset + (p_end - p_start) + (p_start - gps_buff)+3;
		gps_data_offset = p_end - gps_buff + 3;
//		printf("gps_data_offset:%d\r\n", gps_data_offset);
		while(p_start[i] != *p_end){
				check_res = check_res^p_start[i];
				i++;
		}
		memcpy(check_str, p_end+1, 2);
		rec_check_res = hextoint(check_str);
		if(rec_check_res != check_res) {
			printf("p_start:%s, p_end:%s\r\n", p_start, p_end);
			printf("gps check error!,rec_check_res:%02x, check_res:%02x\r\n", rec_check_res, check_res);
			return;
		}
		GPS_GGA_Proces(p_start);
	//	IOT_cmd_data_send(CMD_GPS_DATA, (uint8_t *)p_start, p_end - p_start+1);	
}

void GPS_Control()
{
		uint8_t data[256] = {0};
		static uint16_t last_len = 0, rx_len = 0;
		uint16_t gps_data_len;
		char  check_str[2],*p_data,*p_ldata;
		last_len = rx_len;
		if(CHECK_SYS_TIME(GPS_TM)) return;
		SET_SYS_TIME(GPS_TM, 100);
		rx_len = FIFO_Valid_Size(GPS_UART);
		if(rx_len <= 0) return;
		rx_len = MIN(rx_len, 256);
		if(last_len != rx_len) return;
		FIFO_Rece_Buf(GPS_UART, data, rx_len);   //确保数据接收
		SET_SYS_TIME(WEEK_TIME, 30000);
		p_data = strstr((char *)data, "$OK");
		p_ldata = (char *)data;
		if(p_data){
				do{
						p_ldata = p_data + 1;
						p_data = strstr((char *)p_ldata, "$OK");
				}while(p_data);
		}
		memset(gps_buff, 0, sizeof(gps_buff));
		gps_data_len = rx_len - (p_ldata - (char *)data);
		memcpy(gps_buff, p_ldata, gps_data_len);
//		printf("GPS rec[%d]:%s\r\n", gps_data_len, gps_buff);
		gps_data_offset = 0;	
		last_len = 0;
		rx_len = 0;
}


