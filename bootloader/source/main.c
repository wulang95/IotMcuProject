/******************************************************************************
 * Copyright (C) 2021, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************/

/******************************************************************************
 * @file   main.c
 *
 * @brief  Source file for FLASH example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "system.h"
#include "app_ota.h"
#include "wdt.h"
#include "reset.h"
/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
//#define RUN_IN_RAM 1    //need to cfg *.icf

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/


/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** This sample
 **
 ******************************************************************************/

void print_reset_src()
{
		if(TRUE == Reset_GetFlag(ResetFlagMskPor5V)) {
				printf("ResetFlagMskPor5V\r\n");
		} else if(TRUE == Reset_GetFlag(ResetFlagMskPor1_5V)) {
				printf("ResetFlagMskPor1_5V\r\n");
		} else if(TRUE == Reset_GetFlag(ResetFlagMskLvd)) {
				printf("ResetFlagMskLvd\r\n");
		}else if(TRUE == Reset_GetFlag(ResetFlagMskWdt)) {
				printf("ResetFlagMskWdt\r\n");
		}else if(TRUE == Reset_GetFlag(ResetFlagMskPca)) {
				printf("ResetFlagMskPca\r\n");
		}else if(TRUE == Reset_GetFlag(ResetFlagMskLockup)) {
				printf("ResetFlagMskLockup\r\n");
		}else if(TRUE == Reset_GetFlag(ResetFlagMskSysreq)) {
				printf("ResetFlagMskSysreq\r\n");
		}else if(TRUE == Reset_GetFlag(ResetFlagMskRstb)) {
				printf("ResetFlagMskRstb\r\n");
		}
		Reset_ClearFlagAll();
}
int32_t main(void)
{
		INTX_DISABLE();
		SCB->VTOR = BOOTLOADER_ADR;
    Sys_Init();
		print_reset_src();
		if(check_bacp_app_config() == Ok && ota_config.ota_flag == 1) {
			Wdt_Feed();
			if(ota_data_check() == Ok) {
					Wdt_Feed();
					back_app_to_app();
			} else {
				printf("ota_data_check is fail\r\n");
				printf("jump app adr:%x\r\n", APP_ADR);
				iap_load_app(APP_ADR);
			}
		} else {
			printf("check_bacp_app_config is fail\r\n");
			printf("jump app adr:%x\r\n", APP_ADR);
			iap_load_app(APP_ADR);
		}
    while (1) {
			delay1ms(1000);
			printf("bootloader heart\r\n");
			Wdt_Feed();
		}
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
