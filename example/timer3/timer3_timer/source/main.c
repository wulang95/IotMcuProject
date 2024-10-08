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
 * @brief  Source file for TIMER3 example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "timer3.h"
#include "gpio.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
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

/*******************************************************************************
 * TIM3中断服务函数
 ******************************************************************************/
void Tim3_IRQHandler(void)
{
    static uint8_t i;
    
    //Timer3 模式0 计时溢出中断
    if(TRUE == Tim3_GetIntFlag(Tim3UevIrq))
    {
        if(0 == i)
        {
            Gpio_WriteOutputIO(EVB_LEDR_PORT, EVB_LEDR_PIN, TRUE);  //LED翻转
            i++;
        }
        else
        {
            Gpio_WriteOutputIO(EVB_LEDR_PORT, EVB_LEDR_PIN, FALSE);  //LED翻转
            i = 0;
        }
        
        Tim3_ClearIntFlag(Tim3UevIrq);  //Timer3模式0 中断标志清除
    }
}


//LED端口配置
void App_LEDPortCfg(void)
{
    stc_gpio_cfg_t           stcLEDPortCfg;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcLEDPortCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //GPIO 外设时钟使能
    
    stcLEDPortCfg.enDir  = GpioDirOut;
    Gpio_Init(EVB_LEDR_PORT, EVB_LEDR_PIN, &stcLEDPortCfg);   //PD14设置为LED输出
}

//Timer3 配置
void App_Timer3Cfg(uint16_t u16Period)
{
    uint16_t                    u16ArrValue;
    uint16_t                    u16CntValue;
    stc_tim3_mode0_cfg_t     stcTim3BaseCfg;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcTim3BaseCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralTim3, TRUE); //Base Timer外设时钟使能
    
    stcTim3BaseCfg.enWorkMode = Tim3WorkMode0;              //定时器模式
    stcTim3BaseCfg.enCT       = Tim3Timer;                  //定时器功能，计数时钟为内部PCLK
    stcTim3BaseCfg.enPRS      = Tim3PCLKDiv16;              //PCLK/16
    stcTim3BaseCfg.enCntMode  = Tim316bitArrMode;           //自动重载16位计数器/定时器
    stcTim3BaseCfg.bEnTog     = FALSE;
    stcTim3BaseCfg.bEnGate    = FALSE;
    stcTim3BaseCfg.enGateP    = Tim3GatePositive;
    
    Tim3_Mode0_Init(&stcTim3BaseCfg);                       //TIM3 的模式0功能初始化
        
    u16ArrValue = 0x10000 - u16Period ;
    
    Tim3_M0_ARRSet(u16ArrValue);                            //设置重载值(ARR = 0x10000 - 周期)
    
    u16CntValue = 0x10000 - u16Period;
    
    Tim3_M0_Cnt16Set(u16CntValue);                          //设置计数初值
    
    Tim3_ClearIntFlag(Tim3UevIrq);                          //清中断标志
    Tim3_Mode0_EnableIrq();                                 //使能TIM3中断(模式0时只有一个中断)
    EnableNvic(TIM3_IRQn, IrqLevel3, TRUE);                 //TIM3 开中断 
}


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
    App_LEDPortCfg();//LED端口配置

    App_Timer3Cfg(25000); //Timer3 配置; 16分频,周期25000-->25000*(1/4M) * 16 = 100000us = 100ms
    
    Tim3_M0_Run();   //TIM3 运行。
    
    while (1);
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


