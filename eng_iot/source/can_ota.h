#ifndef __CAN_OTA_H
#define __CAN_OTA_H
#include "ddl.h"

#define CAN_OTA_FLASH_SIZE  0X8000


#define ENTER_OTA_CMD   		0XEB
#define ENTER_OTA_REQ_CMD		0XEC
#define QUIT_OTA_CMD 				0XEE
#define QUIT_OTA_REQ_CMD		0XEF
#define PACK_OTA_CMD        0XD3
#define DATA_OTA_CMD        0XD0

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

enum {
   HMI_ADR =  0X28,
   CONTROL_ADR = 0XEF,
   BMS_ADR = 0XF4,
   SECOND_BMS_ADR = 0XF7,
   IOT_ADR = 0X21,
   CHARGER_ADR = 0X56,
   LOCK_ADR = 0X60,
};

enum {
		CAN_OTA_IDEL_STEP = 0,
		CAN_OTA_ENTER_STEP,
		CAN_FLASH_INIT_STEP,
		CAN_FLASH_DATA_STEP,
		CAN_OTA_HEADER_STEP,
		CAN_OTA_DATA_STEP,
		CAN_OTA_TAIL_STEP,
		CAN_OTA_QUIT_STEP,
};

struct flash_con_stu {
	uint32_t crc;
	uint32_t flash_total_len;
	uint32_t falsh_offset;
	uint8_t flash_buf[128];
	uint16_t flash_pack_len;
};

struct can_ota_con_stu{
		uint8_t dev_id;
		uint8_t ota_step;
		uint8_t last_ota_step;
		uint16_t pack_crc;
		uint8_t pack_buf[4096];
		uint32_t total_len;
		uint32_t offset;
		uint32_t rmlen;
		struct flash_con_stu flash_con;
};





















#endif