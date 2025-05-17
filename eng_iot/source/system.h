#ifndef   __SYSTEM_H
#define   __SYSTEM_H
#include "ddl.h"
#include "gpio.h"
#include "sysctrl.h"
#include "can.h"
#include "uart.h"
#include "FIFO.h"
#include "wdt.h"
#include "rtc.h"
#include "lpm.h"
#include "flash.h"
#include "GPS_Control.h"
#include "IOT_Protol.h"
#include "adc.h"
#include "bgr.h"
#define BOOTLOADER_ADR   0X00000000
#define BOOTLOADER_SIZE	 0X4000
#define APP_ADR		0X00004000
#define APP_SIZE	 0X8000
#define BACK_APP_CONFIG_ADR 0XC000
#define BACK_APP_CONFIG_SIZE 0X400
#define BACK_APP_ADR  0XC400
#define BACK_APP_SIZE 0X8000
#define MAGIC 0XFAFA5858



#define  GPS_TEST   0
#define MIN(x, y)	x>y?y:x

#define  SOFT_VERSION		0x0101

#define  HW_VERSION			0x0101

#define CAN250K			250000
#define CAN500K			500000
#define CAN_BAUD			CAN250K




#define UART_IOT_BAUD   115200
#define UART_GPS_BAUD		115200
#define OTA_BAUD  115200
#define CAN_RX_FIFO_SIZE		1
#define IOT_BUFF_SIZE  			512
#define GPS_BUFF_SIZE  			512
#define OTA_DATA_SIZE       4096

enum {
	CLK_TYPE_INT,
	CLK_TYPE_EXT
};

enum {
	WEEK_TIME	=0,
	GPS_TM,
	CAT1_TM,
	CAT1_ERROR_TM,
	TEST_TM,
	IOT_PROTO_TM,
	IOT_OTA_TM,
	LOCK_TM,
	TIME_MAX,
};
enum {
	CAT1_POWERON,
	CAT1_POWEROFF,
};
extern uint32_t sys_time[TIME_MAX];
#define SET_SYS_TIME(x, t) sys_time[x]=t
#define CHECK_SYS_TIME(x)	sys_time[x]

#define IOT_UART    FIFO_INDEX0
#define GPS_UART		FIFO_INDEX1
#define CAN_OTA			FIFO_INDEX2
extern volatile uint8_t  ship_mode_flag;
extern uint8_t gps_rx_buff[GPS_BUFF_SIZE];
extern uint8_t g_cat1_state;
extern uint32_t cur_tick;
typedef void(*handel)(stc_can_rxframe_t stcRxFrame);
typedef  void (*iapfun)(void);
uint32_t systick_diff(uint32_t time);
void iap_load_app(uint32_t appxaddr);
void Sys_Init();
void Iot_Can_Send(stc_can_txframe_t stcTxFrame);
void Uart0_Send_Iot(uint8_t *buf, uint16_t len);
void can_rx_dispitch(handel process);
void Sys_Check_Sleep();
void UART0_DMA_Send(uint8_t *buf, uint16_t len);
void Uart1_Send_gps(uint8_t *buf, uint16_t len);
void cat1_power_control();
void UART0_SWITCH_BAUD(uint32_t baud);
void mcu_adc_data_check_get();
#endif