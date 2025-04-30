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
#include "dft_protocol.h"
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
				SET_SYS_TIME(TEST_TM, 2000);
	//			printf("mcu eng_dft heart\r\n");
				if(TRUE == Adc_GetIrqStatus(AdcMskIrqSqr)){
					Adc_ClrIrqStatus(AdcMskIrqSqr);
					power48v_adc_val = Adc_GetSqrResult(AdcSQRCH0MUX);
					bat_adc_val = Adc_GetSqrResult(AdcSQRCH1MUX);
					bat_temp_adc_val = Adc_GetSqrResult(AdcSQRCH2MUX);
					printf("power48v_adc_val:%d\r\n", power48v_adc_val);
					printf("bat_adc_val:%d\r\n",bat_adc_val);
					printf("bat_temp_adc_val:%d\r\n",bat_temp_adc_val);
					Adc_SQR_Start();
			}
		}
		IOT_Rec_Parse(); 
		can_rx_dispitch(can_rec_data_handle);
		if(dft_mcu_con.dft_state == DFT_STATE_MCU) {
				dft_mcu_table[dft_mcu_con.dft_cmd].dft_mcu_func();
		}
		Wdt_Feed();	
	}
			
}



