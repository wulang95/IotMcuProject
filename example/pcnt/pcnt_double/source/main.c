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
 * @brief  Source file for PCNT example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "pca.h"
#include "lpm.h"
#include "gpio.h"
#include "pcnt.h"
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

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
 
/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
static void App_GpioInit(void);
static void App_PcntInit(void);

/**
 ******************************************************************************
 ** \brief  主函数
 **
 ** \return 无
 ******************************************************************************/
int32_t main(void)
{       
    App_GpioInit();
    App_PcntInit();

    Pcnt_Cmd(TRUE);                                      //使能PCNT
    while (1)
    {
        
    }
}

/**
 ******************************************************************************
 ** \brief  中断入口函数
 **
 ** \return 无
 ******************************************************************************/
void Pcnt_IRQHandler(void)
{
    Pcnt_ClrItStatus(PcntUF);
    if(Gpio_GetInputIO(EVB_LEDR_PORT, EVB_LEDR_PIN) == FALSE)
    {
        Gpio_WriteOutputIO(EVB_LEDR_PORT, EVB_LEDR_PIN, TRUE);
    }
    else if(Gpio_GetInputIO(EVB_LEDR_PORT, EVB_LEDR_PIN) == TRUE)
    {
        Gpio_WriteOutputIO(EVB_LEDR_PORT, EVB_LEDR_PIN, FALSE);
    }    
}

/**
 ******************************************************************************
 ** \brief  初始化外部GPIO引脚
 **
 ** \return 无
 ******************************************************************************/
static void App_GpioInit(void)
{
    stc_gpio_cfg_t GpioInitStruct;
    DDL_ZERO_STRUCT(GpioInitStruct);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    GpioInitStruct.enDrv = GpioDrvH;
    GpioInitStruct.enDir = GpioDirIn;
    Gpio_Init(GpioPortB,GpioPin5,&GpioInitStruct);
    Gpio_SetAfMode(GpioPortB,GpioPin5,GpioAf6);              //PB05作为PCNT_S0
    Gpio_Init(GpioPortB,GpioPin7,&GpioInitStruct);
    Gpio_SetAfMode(GpioPortB,GpioPin7,GpioAf7);              //PB07作为PCNT_S1
    
    GpioInitStruct.enDir = GpioDirOut;
    Gpio_Init(EVB_LEDR_PORT, EVB_LEDR_PIN, &GpioInitStruct);   //PD14配置成输出，控制板上蓝色LED
    Gpio_WriteOutputIO(EVB_LEDR_PORT, EVB_LEDR_PIN, TRUE);    
}

/**
 ******************************************************************************
 ** \brief  配置PCNT
 **
 ** \return 无
 ******************************************************************************/
static void App_PcntInit(void)
{
    stc_pcnt_initstruct_t PcntInitStruct;
    DDL_ZERO_STRUCT(PcntInitStruct);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralPcnt, TRUE);    
    
    PcntInitStruct.Pcnt_S0Sel = PcntS0PNoinvert;  //S0输入极性不取反
    PcntInitStruct.Pcnt_S1Sel = PcntS1PNoinvert;  //S1输入极性不取反 
    PcntInitStruct.Pcnt_Clk   = PcntCLKPclk;      //采样时钟
    PcntInitStruct.Pcnt_Mode  = PcntDoubleMode;   //双通道正交脉冲计数模式
    PcntInitStruct.Pcnt_FltEn = TRUE;          //滤波使能
    PcntInitStruct.Pcnt_DebTop = 5;            //滤波计数器阈值
    PcntInitStruct.Pcnt_ClkDiv = 5;            //滤波时钟分频系数
    PcntInitStruct.Pcnt_TocrEn = TRUE;         //超时控制使能
    PcntInitStruct.Pcnt_TocrTh = 2000;         //超时阈值
    Pcnt_Init(&PcntInitStruct);

    Pcnt_SetB2C(100);                                    //计数溢出值100
    Pcnt_ClrItStatus(PcntUF);                           //清除PCNT溢出中断
    Pcnt_ItCfg(PcntUF, TRUE);                        //使能溢出中断
    EnableNvic(PCNT_IRQn, IrqLevel3, TRUE);              //使能中断向量
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/

