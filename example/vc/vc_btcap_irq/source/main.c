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
 * @brief  Source file for VC example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "vc.h"
#include "gpio.h"
#include "adc.h"
#include "bt.h"
#include "flash.h"
#include "bgr.h"
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
 ** \brief  中断入口函数
 **
 ** \return 无
 ******************************************************************************/
void Tim2_IRQHandler(void)
{
    if(TRUE == Bt_GetIntFlag(TIM2, BtCA0Irq))
    {
        if(Gpio_GetInputIO(EVB_LEDR_PORT, EVB_LEDR_PIN) == FALSE)
        {
            Gpio_WriteOutputIO(EVB_LEDR_PORT, EVB_LEDR_PIN, TRUE);
        }
        else if(Gpio_GetInputIO(EVB_LEDR_PORT, EVB_LEDR_PIN) == TRUE)
        {
            Gpio_WriteOutputIO(EVB_LEDR_PORT, EVB_LEDR_PIN, FALSE);
        }
        Bt_ClearIntFlag(TIM2,BtCA0Irq);
    }
}

/**
 ******************************************************************************
 ** \brief  初始化外部GPIO引脚
 **
 ** \return 无
 ******************************************************************************/
void GPIO_Cfg(void)
{
    stc_gpio_cfg_t GpioInitStruct;
    DDL_ZERO_STRUCT(GpioInitStruct);
    
    GpioInitStruct.enDrv = GpioDrvH;
    GpioInitStruct.enDir = GpioDirOut;
    Gpio_Init(GpioPortA,GpioPin6,&GpioInitStruct);
    Gpio_SetAfMode(GpioPortA,GpioPin6,GpioAf5);              //PA06作为VC0_OUT

    Gpio_Init(EVB_LEDR_PORT, EVB_LEDR_PIN, &GpioInitStruct);   //PD14配置成输出，控制板上蓝色LED
    Gpio_WriteOutputIO(EVB_LEDR_PORT, EVB_LEDR_PIN, TRUE);    
    
    Gpio_SetAnalogMode(GpioPortC,GpioPin0);                  //模拟输入

    GpioInitStruct.enDir  = GpioDirOut;
    
    Gpio_SfTimCCfg(GpioSfTim2CA,GpioSf2);               //VC0_OUT连接TIM2_CHA
}

/**
 ******************************************************************************
 ** \brief  初始化VC0
 **
 ** \return 无
 ******************************************************************************/
void VC_Cfg(void)
{
    stc_vc_channel_cfg_t VcInitStruct;
    DDL_ZERO_STRUCT(VcInitStruct);
    VcInitStruct.enVcChannel = VcChannel0;
    VcInitStruct.enVcCmpDly  = VcDelay10mv;          //VC0迟滞电压约为10mV
    VcInitStruct.enVcBiasCurrent = VcBias10ua;       //VC0功耗为10uA
    VcInitStruct.enVcFilterTime  = VcFilter28us;     //VC输出滤波时间约为28us
    VcInitStruct.enVcInPin_P     = VcInPCh0;         //VC0_CH的P端连接PC00
    VcInitStruct.enVcInPin_N     = AiBg1p2;          //VC0_CH的N端连接内核1.2V
    VcInitStruct.enVcOutCfg   = VcOutTIM2RCLR;    //不输出给配置寄存器所定义的外设
    VcInitStruct.bFlten          = TRUE;             //是能滤波
    Vc_Init(&VcInitStruct);
    Vc_CfgItType(VcChannel0, VcIrqRise);          //配置VC0为上升沿中断    
}

/**
 ******************************************************************************
 ** \brief  初始化TIM2
 **
 ** \return 无
 ******************************************************************************/
void TIM2M23_Cfg(void)
{
    stc_bt_mode23_cfg_t        stcBtBaseCfg;
    stc_bt_m23_input_cfg_t     stcBtPortInputCfg;
    stc_bt_m23_bk_input_cfg_t  stcBtBkInputCfg;
    uint16_t                      u16CntValue;
    uint16_t                      u16ArrValue;
    uint8_t                       u8ValidPeriod;

    DDL_ZERO_STRUCT(stcBtBaseCfg);
    DDL_ZERO_STRUCT(stcBtPortInputCfg);   
    DDL_ZERO_STRUCT(stcBtBkInputCfg);    

    stcBtBaseCfg.enWorkMode    = BtWorkMode2;              //锯齿波模式
    stcBtBaseCfg.enCT          = BtTimer;                  //定时器功能，计数时钟为内部PCLK
    stcBtBaseCfg.enPRS         = BtPCLKDiv8;               //PCLK
    stcBtBaseCfg.enCntDir      = BtCntUp;                  //向上计数，在三角波模式时只读

    Bt_Mode23_Init(TIM2, &stcBtBaseCfg);                   //TIM0 的模式2/3功能初始化
        
    stcBtPortInputCfg.enCh0ACmpCap    = BtCHxCapMode;
    stcBtPortInputCfg.enCH0ACapSel    = BtCHxCapRise;
    stcBtPortInputCfg.enCH0AInFlt     = BtFltNone;
    stcBtPortInputCfg.enCH0APolarity  = BtPortPositive;

    Bt_M23_PortInput_Cfg(TIM2,&stcBtPortInputCfg);      
    
    u8ValidPeriod = 0;                                     //事件更新周期设置，0表示锯齿波每个周期更新一次，每+1代表延迟1个周期
    Bt_M23_SetValidPeriod(TIM2,u8ValidPeriod);             //间隔周期设置
    
    u16CntValue = 0;
    Bt_M23_Cnt16Set(TIM2, u16CntValue);                    //设置计数初值    

    u16ArrValue = 0xFFFF;
    Bt_M23_ARRSet(TIM2, u16ArrValue, TRUE);                //设置重载值,并使能缓存
}

/**
 ******************************************************************************
 ** \brief  主函数
 **
 ** \return 无
 ******************************************************************************/
int main(void)
{
    stc_vc_channel_cfg_t       stcChannelCfg;    
    stc_sysctrl_clk_cfg_t      stcClkCfg;    
    en_flash_waitcycle_t          enFlashWait;

    DDL_ZERO_STRUCT(stcChannelCfg);  
    DDL_ZERO_STRUCT(stcClkCfg);    
    enFlashWait = FlashWaitCycle1;                      //读等待周期设置为1（当HCLK大于24MHz时必须至少为1）
    Flash_WaitCycle(enFlashWait);                       // Flash 等待1个周期
    stcClkCfg.enClkSrc    = SysctrlClkXTH;           //使用外部高速晶振,32M
    stcClkCfg.enHClkDiv   = SysctrlHclkDiv1;         // HCLK = SystemClk/1
    stcClkCfg.enPClkDiv   = SysctrlPclkDiv1;         // PCLK = HCLK/1
    Sysctrl_SetXTHFreq(SysctrlXthFreq24_32MHz);         //设置外部高速频率为20~32M
    Sysctrl_ClkInit(&stcClkCfg);
        
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);//开GPIO时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralVcLvd, TRUE);//开LVD时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);//开adc时钟   
    Sysctrl_SetPeripheralGate(SysctrlPeripheralBaseTim, TRUE); //Base Timer外设时钟使能

    Bgr_BgrEnable();                 //BGR必须使能

    GPIO_Cfg();                                        //配置测试IO口
    VC_Cfg();                                          //配置VC
    TIM2M23_Cfg();                                     //配置TIM2的模式
    Vc_Cmd(VcChannel0, TRUE);                                //使能VC0
    
    Bt_ClearAllIntFlag(TIM2);                              //清中断标志
    EnableNvic(TIM2_IRQn, IrqLevel0, TRUE);                //TIM0中断使能
    Bt_Mode23_EnableIrq(TIM2,BtCA0Irq);                    //使能TIM0 UEV更新中断    
    Bt_M23_Run(TIM2);                                      //TIM0 运行    
    while (1)
    {

    }
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


