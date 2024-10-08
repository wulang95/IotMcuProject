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
 * @brief  Source file for OPA example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "gpio.h"
#include "opa.h"
#include "bgr.h"
/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
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

/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 ******************************************************************************
 ** \brief  主函数
 **
 ** \return 无
 ******************************************************************************/
int32_t main(void)
{   
    stc_gpio_cfg_t GpioInitStruct;
    stc_opa_oenx_config_t OpaOenxStruct;
    stc_opa_zconfig_t OpaInitStruct;
    
    DDL_ZERO_STRUCT(GpioInitStruct);
    DDL_ZERO_STRUCT(OpaOenxStruct);
    DDL_ZERO_STRUCT(OpaInitStruct);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralOpa, TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE); /* 使用OPA,必须使能BGR时钟 */
    Bgr_BgrEnable(); /* 使用OPA,必须使能BGR */
    
    Gpio_SetAnalogMode(GpioPortB, GpioPin15);      /* OPA_INN引脚： PB15 */
    Gpio_SetAnalogMode(GpioPortC, GpioPin6);       /* OPA_INP引脚： PC06 */
    Gpio_SetAnalogMode(GpioPortC, GpioPin7);       /* OPA_OUT引脚： PC07 */
    Gpio_SetAnalogMode(GpioPortD, GpioPin8);       /* OPA_OUT4引脚：PD08 */

    /* 配置Opa0的输出通道数 */
    OpaOenxStruct.opa_ch1 = FALSE;
    OpaOenxStruct.opa_ch2 = FALSE;
    OpaOenxStruct.opa_ch3 = FALSE;
    OpaOenxStruct.opa_ch4 = TRUE;
    Opa_OutChannelConfig(Opa0, OpaOenxStruct);

    /* 使能Opa0通道 */
    Opa_Cmd(Opa0, TRUE);

    /* 配置Opa教零 */
    OpaInitStruct.bAdctr_en = FALSE;                     /* 禁止DAC启动时自动校准*/
    OpaInitStruct.bAz_pulse = FALSE;                     /* 禁用软件校准，使用软件触发校准*/
    OpaInitStruct.bClk_sw_set = FALSE;                   /* 软件校准无效*/
    OpaInitStruct.enClksrc = Opa_M1024Pclk;              /* 自动校准脉冲宽度设置*/
    Opa_CalConfig(&OpaInitStruct);
    
    /* 配置Opa0教零使能，使能OPA教零之前，必须提前配置教零控制寄存器OPA_CR */
    Opa_CalCmd(Opa0, TRUE);
    
    /* 软件触发校准 */
    Opa_CalSwTrig();
    
    
    while (1)
    {   
    }
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


