#ifndef   __SYSTEM_H
#define   __SYSTEM_H
#include "ddl.h"
#define BOOTLOADER_ADR   0X00000000
#define BOOTLOADER_SIZE	 0X4000
#define APP_ADR		0X00004000
#define APP_SIZE	 0X8000
#define BACK_APP_CONFIG_ADR 0XC000
#define BACK_APP_CONFIG_SIZE 0X400
#define BACK_APP_ADR  0XC400
#define BACK_APP_SIZE 0X8000
#define MAGIC 0XFAFA5858

#define MIN(x, y)	x>y?y:x



typedef  void (*iapfun)(void);
void Sys_Init();
void iap_load_app(uint32_t appxaddr);
















#endif