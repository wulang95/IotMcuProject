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
#define  GPS_TEST   0
#define MIN(x, y)	x>y?y:x

#define  SOFT_VERSION_H		0x00
#define  SOFT_VERSION_L		0x00

#define  HW_VERSION_H			0x00
#define	 HW_VERSION_L			0x00

#define CAN250K			250000
#define CAN500K			500000
#define CAN_BAUD			CAN250K
#define UART_IOT_BAUD   115200
#define UART_GPS_BAUD		115200

#define CAN_RX_FIFO_SIZE		50
#define IOT_BUFF_SIZE  			256
#define GPS_BUFF_SIZE  			512

enum {
	WEEK_TIME	=0,
	GPS_TM,
	CAT1_TM,
	TEST_TM,
	TIME_MAX,
};

extern uint32_t sys_time[TIME_MAX];
#define SET_SYS_TIME(x, t) sys_time[x]=t
#define CHECK_SYS_TIME(x)	sys_time[x]

#define IOT_UART    FIFO_INDEX0
#define GPS_UART		FIFO_INDEX1

extern uint8_t gps_rx_buff[GPS_BUFF_SIZE];

extern uint32_t cur_tick;
typedef void(*handel)(stc_can_rxframe_t stcRxFrame);
uint32_t systick_diff(uint32_t time);
void Sys_Init();
void Iot_Can_Send(stc_can_txframe_t stcTxFrame);
void Uart0_Send_Iot(uint8_t *buf, uint16_t len);
void can_rx_dispitch(handel process);
void Sys_Check_Sleep();
void UART0_DMA_Send(uint8_t *buf, uint16_t len);
void cat1_power_on();
void Uart1_Send_gps(uint8_t *buf, uint16_t len);





#endif