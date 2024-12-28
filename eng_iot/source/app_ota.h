#ifndef APP_OTA_H
#define APP_OTA_H
#include "ddl.h"
#include "flash.h"

#pragma pack(1)
struct ota_config_stu{
		uint32_t magic;
		uint8_t ota_flag;
		uint16_t soft_ver;
		uint16_t hw_ver;
		uint32_t total_len;
		uint32_t file_crc32;
		uint32_t config_crc32;
};
#pragma pack()

struct ota_control_stu{
		uint16_t pack_num;
		uint16_t last_num; 
		uint8_t data_len;
		uint32_t offset;
		uint8_t data[128];
};
extern struct ota_config_stu ota_config;
extern struct ota_control_stu ota_control;
unsigned int GetCrc32(const unsigned char* pData, unsigned int Len);
unsigned int GetCrc32_cum(const unsigned char* pData, unsigned int Len, unsigned int CRC32);
void ota_flash_write(uint32_t addr, void *data, uint32_t len);
void ota_flash_read(uint32_t addr, void *buffer, uint32_t len);
void ota_flash_erase(uint32_t addr, uint32_t len);
int ota_data_check();








#endif
