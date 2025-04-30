#ifndef __DFT_PROTOCOL_H
#define __DFT_PROTOCOL_H
#include "ddl.h"
#include "can.h"


#define DFT_OK   	0
#define	DFT_FAIL	1

enum{
	DFT_STATE_IDEL,   /* 测试空闲 */
	DFT_STATE_CAT1,		/* 测试CAT1 */
	DFT_STATE_MCU,		/* 测试MCU  */
};

enum{
		CAT1_ID = 0X24,
		DFT_ID = 0X22,
		MCU_ID = 0X23,
};

enum{
		DFT_CAN_R_PNG = 0XEA,
		DFT_CAN_CMD = 0X21,
		DFT_CAN_CMD_ACK = 0X22,
};

enum {
	CAN_DFT_GPS_STAR_NUM = 0,
	CAN_DFT_SYSPOWER_ADC,
	CAN_DFT_BACKBAT_ADC,
	CAN_DFT_BACKTEMP_ADC,
	CAN_DFT_CAT1_CONN_GPIO,
	CAN_DFT_SYSPOWER_DET,
	CAN_DFT_MAX,
};

enum {
	CAN_DFT_GPS_VAR = 0XF3F0,
};


enum {
		CMD_CAN_TRANS = 0X0C,
		CMD_DFT_CAT1_SIGN = 0X0D,
		CMD_DFT_CON_GPIO = 0X0B,
};


#pragma pack(1)
typedef struct {
    union 
    {
        uint16_t pdu2;
        struct {
            uint8_t da;
            uint8_t pdu1;
        };
    }; 
}PDU_STU;
typedef struct {
    union 
    {
        uint32_t can_id;
        struct {
            uint8_t src;
            PDU_STU pdu;
            uint8_t dp  :1;
            uint8_t r   :1;
            uint8_t p   :3;
            uint8_t res :3;
        };
    };   
} CAN_PDU_STU;


#pragma pack()

typedef void (*dft_item_call_func)();

struct dft_mcu_item_stu {
	uint16_t dft_cmd_index;   					/* 当前测试项 */
	uint16_t dft_cmd_ask_index;					/* 当前测试项应答*/
	uint8_t data_len;										/* 数据长度  */
	uint32_t dft_timeout;				 				/*  测试超时  */
	dft_item_call_func dft_mcu_func;    /* 测试函数   */
};

struct dft_mcu_con_stu {
		uint8_t dft_state;
		uint8_t dft_cmd;
};

extern struct dft_mcu_item_stu dft_mcu_table[];
extern struct dft_mcu_con_stu dft_mcu_con;
void IOT_Rec_Parse();
void can_rec_data_handle(stc_can_rxframe_t stcRxFrame);





















#endif