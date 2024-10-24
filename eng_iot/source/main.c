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
int32_t main(void)
{
	Sys_Init();
	while(1)
	{
			if(CHECK_SYS_TIME(TEST_TM) == 0) {
					SET_SYS_TIME(TEST_TM, 1000);
					printf("mcu heart\r\n");
			}
		#if GPS_TEST == 0 
			GPS_Control();
			GPS_data_task();
		#endif
			IOT_Rec_Parse();
			can_rx_dispitch(CAN_Rec_Prase);
	//		Sys_Check_Sleep();
			Wdt_Feed();
	}
}



