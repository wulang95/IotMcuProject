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
 * @brief  Source file for GPIO example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "gpio.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void App_UserKeyInit(void);
static void App_LedInit(void);
  
/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*****************************************************************************
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
int32_t main(void)
{
    ///< 按键端口初始化
    App_UserKeyInit();
    
    ///< LED端口初始化
    App_LedInit();
    
    ///< 打开并配置USER KEY为下降沿中断
    Gpio_EnableIrq(EVB_KEY1_PORT, EVB_KEY1_PIN, GpioIrqFalling);
    ///< 使能端口PORTC系统中断
    EnableNvic(PORTC_E_IRQn, IrqLevel3, TRUE);
    
    while(1)
    {
        ;
    }
}

///< PortC中断服务函数
void PortC_IRQHandler(void)
{
    if(TRUE == Gpio_GetIrqStatus(EVB_KEY1_PORT, EVB_KEY1_PIN))
    {            
        ///< LED点亮
        Gpio_SetIO(EVB_LEDR_PORT, EVB_LEDR_PIN);
        
        delay1ms(2000);
        
        ///< LED关闭
        Gpio_ClrIO(EVB_LEDR_PORT, EVB_LEDR_PIN);  

        Gpio_ClearIrq(EVB_KEY1_PORT, EVB_KEY1_PIN);    
    }

}  

static void App_UserKeyInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    ///< 端口方向配置->输入
    stcGpioCfg.enDir = GpioDirIn;
    ///< 端口驱动能力配置->高驱动能力
    stcGpioCfg.enDrv = GpioDrvL;
    ///< 端口上下拉配置->无
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    ///< 端口开漏输出配置->开漏输出关闭
    stcGpioCfg.enOD = GpioOdDisable;
    ///< 端口输入/输出值寄存器总线控制模式配置->AHB
    stcGpioCfg.enCtrlMode = GpioAHB;
    ///< GPIO IO USER KEY初始化
    Gpio_Init(EVB_KEY1_PORT, EVB_KEY1_PIN, &stcGpioCfg); 
}


static void App_LedInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 
    
    ///< 端口方向配置->输出(其它参数与以上（输入）配置参数一致)
    stcGpioCfg.enDir = GpioDirOut;
    ///< 端口上下拉配置->下拉
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdEnable;
    
    ///< LED关闭
    Gpio_ClrIO(EVB_LEDR_PORT, EVB_LEDR_PIN);
    
    ///< GPIO IO LED端口初始化
    Gpio_Init(EVB_LEDR_PORT, EVB_LEDR_PIN, &stcGpioCfg);
    

}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


