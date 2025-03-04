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
 * @brief  Source file for CAN example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "system.h"
#include "IOT_Protol.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief  Main function of can tx rx Irq mode project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
extern  void crc32_test();
uint8_t unlock_cnt;
int32_t main(void)
{
	SCB->VTOR = APP_ADR;
	Sys_Init();
	crc32_test();
	while(1)
	{
			if(CHECK_SYS_TIME(TEST_TM) == 0) {
					SET_SYS_TIME(TEST_TM, 1000);
					printf("mcu heart\r\n");
			}
			if(g_cat1_state == CAT1_POWERON && ship_mode_flag == 0) {
				if(Gpio_GetInputIO(GpioPortA, GpioPin6) == 1) {
					
						SET_SYS_TIME(CAT1_ERROR_TM, 30000);
						Sys_Check_Sleep();
				} else {
						if(CHECK_SYS_TIME(CAT1_ERROR_TM) == 0) {
							g_cat1_state = CAT1_POWEROFF;
						}
				}
			}
			if(lock_sta == CAR_UNLOCK_ATA && (CHECK_SYS_TIME(LOCK_TM)==0)) {   //持续发2S，加快开机速度
					car_jump_password();
					SET_SYS_TIME(LOCK_TM, 20);
					if(++unlock_cnt >= 100) {
						 lock_sta = CAR_LOCK_STA;
					}
			} 
			if(ship_mode_flag == 0) {
					cat1_power_control();
			} else {
					Sys_Check_Sleep();	
			}
		#if GPS_TEST == 0 
			GPS_Control();
			GPS_data_task();
		#endif
			IOT_Rec_Parse();
			can_rx_dispitch(CAN_Rec_Prase);
			if(CHECK_SYS_TIME(IOT_OTA_TM) == 0){
				can_ota_data();
				SET_SYS_TIME(IOT_OTA_TM, 3);
			}
			Wdt_Feed();
	}
}



