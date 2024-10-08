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
 * @brief  Source file for LPTIMER example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "lptim.h"
#include "lpm.h"
#include "gpio.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
//LED
#define LED_PORT     GpioPortD
#define LED_PIN      GpioPin14
//按键
#define Button_PORT  GpioPortA
#define Button_PIN   GpioPin7

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
__IO uint8_t ItFlag;
/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static volatile uint32_t u32LptTestFlag = 0;


/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/


/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
static void App_SysClkInit(void);     
static void App_GPIOInit(void);
static void App_LPTimer0Init(void);
static void App_LowPowerModeGpioSet(void);

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
    ///< 系统时钟初始化
    App_SysClkInit();     
    ///< GPIO初始化
    App_GPIOInit();
    ///< LPTimer0初始化
    App_LPTimer0Init(); 
    
    while (Gpio_GetInputIO(Button_PORT,Button_PIN) == 1);   //等待user key按下
    Lptim_Cmd(M0P_LPTIMER0, TRUE);                          //启动lptimer0运行   
    Gpio_WriteOutputIO(LED_PORT, LED_PIN, FALSE);           //熄灭LED

    ///< 休眠模式GPIO配置
    App_LowPowerModeGpioSet();
    Lpm_GotoDeepSleep(TRUE);                                //进入深度睡眠模式
    while (1)
    {
        if(ItFlag == 1)
        {
            ItFlag = 0;
        }
    }
}



/**
 ******************************************************************************
 ** \brief  LPTIMER0中断服务函数
 **
 ** \return 无
 ******************************************************************************/
void LpTim0_IRQHandler(void)
{
    if (TRUE == Lptim_GetItStatus(M0P_LPTIMER0))
    {
        ItFlag = 1;
        Lptim_ClrItStatus(M0P_LPTIMER0);//清除LPTIMER0的中断标志位      

        Gpio_WriteOutputIO(EVB_LEDR_PORT, EVB_LEDR_PIN, TRUE);     //输出高，点亮LED
        delay1ms(500);
        Gpio_WriteOutputIO(EVB_LEDR_PORT, EVB_LEDR_PIN, FALSE);     //输出高，点亮LED
    }
}


static void App_SysClkInit(void)
{
    stc_sysctrl_clk_cfg_t  stcClkCfg;
    
    //CLK INIT
    stcClkCfg.enClkSrc  = SysctrlClkRCH;
    stcClkCfg.enHClkDiv = SysctrlHclkDiv1;
    stcClkCfg.enPClkDiv = SysctrlPclkDiv1;
    Sysctrl_ClkInit(&stcClkCfg);
    
    //使能RCL
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);
}

/**
 ******************************************************************************
 ** \brief  初始化外部GPIO引脚
 **
 ** \return 无
 ******************************************************************************/
static void App_GPIOInit(void)
{
    stc_gpio_cfg_t         GpioInitStruct;
    DDL_ZERO_STRUCT(GpioInitStruct);
    
    //使能GPIO外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 
    
    //PD14设置为GPIO-->LED控制脚
    GpioInitStruct.enDir  = GpioDirOut;
    Gpio_Init(EVB_LEDR_PORT, EVB_LEDR_PIN, &GpioInitStruct);

    //PA07作为按键输入
    GpioInitStruct.enDir  = GpioDirIn;
    Gpio_Init(EVB_KEY1_PORT, EVB_KEY1_PIN, &GpioInitStruct);
    
    Gpio_WriteOutputIO(EVB_LEDR_PORT, EVB_LEDR_PIN, TRUE);     //输出高，点亮LED    
}

/**
 ******************************************************************************
 ** \brief  初始化LPTIMER0
 **
 ** \return 无
 ******************************************************************************/
static void App_LPTimer0Init(void)
{
    stc_lptim_cfg_t    stcLptCfg;    
    DDL_ZERO_STRUCT(stcLptCfg);

    ///< 使能LPTIM0 外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralLpTim0, TRUE);
    
    stcLptCfg.enGate   = LptimGateLow;
    stcLptCfg.enGatep  = LptimGatePLow;
    stcLptCfg.enTcksel = LptimRcl;
    stcLptCfg.enTogen  = LptimTogEnLow;
    stcLptCfg.enCt     = LptimTimerFun;         //计数器功能
    stcLptCfg.enMd     = LptimMode1;            //工作模式为模式1：无自动重装载16位计数器/定时器
    stcLptCfg.u16Arr   = 0;                     //预装载寄存器值
    Lptim_Init(M0P_LPTIMER0, &stcLptCfg);
    
    Lptim_ClrItStatus(M0P_LPTIMER0);        //清除中断标志位
    Lptim_ConfIt(M0P_LPTIMER0, TRUE);       //允许LPTIMER中断    
    EnableNvic(LPTIM_0_1_IRQn, IrqLevel3, TRUE); 
}

static void App_LowPowerModeGpioSet(void)
{
    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    //swd as gpio
    Sysctrl_SetFunc(SysctrlSWDUseIOEn, TRUE);
    
    ///< 配置为数字端口
    M0P_GPIO->PAADS = 0;
    M0P_GPIO->PBADS = 0;
    M0P_GPIO->PCADS = 0;
    M0P_GPIO->PDADS = 0;
    M0P_GPIO->PEADS = 0;
    M0P_GPIO->PFADS = 0;
    
    ///< 配置为端口输入（除LED端口外）
    M0P_GPIO->PADIR = 0XFFFF;
    M0P_GPIO->PBDIR = 0XFFFF;
    M0P_GPIO->PCDIR = 0XFFFF;
    M0P_GPIO->PDDIR = 0XFFFF; 
    M0P_GPIO->PEDIR = 0XFFF7;
    M0P_GPIO->PFDIR = 0XFFFF;
    
}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


