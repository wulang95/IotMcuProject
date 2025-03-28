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
 * @brief  Source file for GENERALTIMER example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "bt.h"
#include "gpio.h"
#include "flash.h"
#include "adc.h"

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
 * TIM0中断服务函数
 ******************************************************************************/
void Tim0_IRQHandler(void)
{
    static uint8_t i;
    
    //Timer0 模式23 更新中断
    if(TRUE == Bt_GetIntFlag(TIM0, BtUevIrq))
    {
        if(0 == i)
        {
            Gpio_WriteOutputIO(EVB_LEDR_PORT, EVB_LEDR_PIN, TRUE);  //LED 引脚输出高电平
            
            Bt_M23_CCR_Set(TIM0, BtCCR0A, 0x3000); //设置通道A比较值
            Bt_M23_CCR_Set(TIM0, BtCCR0B, 0x6000); //设置通道B比较值
            
            i++;
        }
        else if(1 == i)
        {
            Gpio_WriteOutputIO(EVB_LEDR_PORT, EVB_LEDR_PIN, FALSE);  //LED 引脚输出低电平
            
            Bt_M23_CCR_Set(TIM0, BtCCR0A, 0x6000); //设置通道A比较值
            Bt_M23_CCR_Set(TIM0, BtCCR0B, 0x3000); //设置通道B比较值
            
            i = 0;
        }
        
        Bt_ClearIntFlag(TIM0,BtUevIrq);  //清中断标志
    }
}

//时钟配置初始化
void App_ClockCfg(void)
{
    en_flash_waitcycle_t      enWaitCycle;
    stc_sysctrl_pll_cfg_t     stcPLLCfg;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcPLLCfg);
    
    enWaitCycle = FlashWaitCycle1;
    Flash_WaitCycle(enWaitCycle);
    
    stcPLLCfg.enInFreq    = SysctrlPllInFreq4_6MHz;     //RCH 4MHz
    stcPLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;  //PLL 输出48MHz
    stcPLLCfg.enPllClkSrc = SysctrlPllRch;              //输入时钟源选择RCH
    stcPLLCfg.enPllMul    = SysctrlPllMul12;            //4MHz x 12 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg);
    Sysctrl_SetPLLStableTime(SysctrlPllStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkPLL, TRUE);
    
    Sysctrl_SysClkSwitch(SysctrlClkPLL);  ///< 时钟切换
}

//端口配置初始化
void App_Timer0PortCfg(void)
{
    stc_gpio_cfg_t             stcTIM0Port;
    stc_gpio_cfg_t             stcLEDPort;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcTIM0Port);
    DDL_ZERO_STRUCT(stcLEDPort);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //GPIO 外设时钟使能
    
    stcLEDPort.enDir  = GpioDirOut;
    Gpio_Init(EVB_LEDR_PORT, EVB_LEDR_PIN, &stcLEDPort);     //PD14设置为LED输出
    
    stcTIM0Port.enDir  = GpioDirOut;
    
    Gpio_Init(GpioPortA, GpioPin0, &stcTIM0Port);
    Gpio_SetAfMode(GpioPortA,GpioPin0,GpioAf7);            //PA00设置为TIM0_CHA
    
    Gpio_Init(GpioPortA, GpioPin1, &stcTIM0Port);
    Gpio_SetAfMode(GpioPortA,GpioPin1,GpioAf3);            //PA01设置为TIM0_CHB
}

//Timer0 配置
void App_Timer0Cfg(uint16_t u16Period, uint16_t u16CHxACompare, uint16_t u16CHxBCompare)
{
    uint16_t                   u16CntValue;
    uint8_t                    u8ValidPeriod;
    stc_bt_mode23_cfg_t        stcBtBaseCfg;
    stc_bt_m23_compare_cfg_t   stcBtPortCmpCfg;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcBtBaseCfg);
    DDL_ZERO_STRUCT(stcBtPortCmpCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralBaseTim, TRUE);   //Base Timer外设时钟使能
        
    stcBtBaseCfg.enWorkMode    = BtWorkMode2;              //锯齿波模式
    stcBtBaseCfg.enCT          = BtTimer;                  //定时器功能，计数时钟为内部PCLK
    stcBtBaseCfg.enPRS         = BtPCLKDiv1;               //PCLK
    stcBtBaseCfg.enCntDir      = BtCntUp;                  //向上计数，在三角波模式时只读
    stcBtBaseCfg.enPWMTypeSel  = BtIndependentPWM;         //独立输出PWM
    stcBtBaseCfg.enPWM2sSel    = BtSinglePointCmp;         //单点比较功能
    stcBtBaseCfg.bOneShot      = FALSE;                    //循环计数
    stcBtBaseCfg.bURSSel       = FALSE;                    //上下溢更新
    Bt_Mode23_Init(TIM0, &stcBtBaseCfg);                   //TIM0 的模式23功能初始化
    
    Bt_M23_ARRSet(TIM0, u16Period, TRUE);                  //设置重载值,并使能缓存
    
    Bt_M23_CCR_Set(TIM0, BtCCR0A, u16CHxACompare);         //设置比较值A

    Bt_M23_CCR_Set(TIM0, BtCCR0B, u16CHxBCompare);         //设置比较值B
    
    
    stcBtPortCmpCfg.enCH0ACmpCtrl   = BtPWMMode2;          //OCREFA输出控制OCMA:PWM模式2
    stcBtPortCmpCfg.enCH0APolarity  = BtPortPositive;      //正常输出
    stcBtPortCmpCfg.bCh0ACmpBufEn   = TRUE;                //A通道缓存控制
    stcBtPortCmpCfg.enCh0ACmpIntSel = BtCmpIntNone;        //A通道比较控制:无
    
    stcBtPortCmpCfg.enCH0BCmpCtrl   = BtPWMMode2;          //OCREFB输出控制OCMB:PWM模式2
    stcBtPortCmpCfg.enCH0BPolarity  = BtPortPositive;      //正常输出
    stcBtPortCmpCfg.bCH0BCmpBufEn   = TRUE;                //B通道缓存控制使能
    stcBtPortCmpCfg.enCH0BCmpIntSel = BtCmpIntNone;        //B通道比较控制:无
    
    Bt_M23_PortOutput_Cfg(TIM0, &stcBtPortCmpCfg);         //比较输出端口配置
        
    u8ValidPeriod = 0;                                     //事件更新周期设置，0表示锯齿波每个周期更新一次，每+1代表延迟1个周期
    Bt_M23_SetValidPeriod(TIM0,u8ValidPeriod);             //间隔周期设置
        
    u16CntValue = 0;
    Bt_M23_Cnt16Set(TIM0, u16CntValue);                    //设置计数初值
    
    Bt_ClearAllIntFlag(TIM0);                              //清中断标志
    EnableNvic(TIM0_IRQn, IrqLevel0, TRUE);                //TIM0中断使能
    Bt_Mode23_EnableIrq(TIM0,BtUevIrq);                    //使能TIM0 UEV更新中断
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
    App_ClockCfg();                       //时钟初始化
    
    App_Timer0Cfg(48000, 24000, 24000);   //Timer0 配置:周期 48000(锯齿波: 1K); 通道A比较值24000; 通道B比较值24000
    
    App_Timer0PortCfg();                  //Timer3 Port端口配置
    
    Bt_M23_EnPWM_Output(TIM0, TRUE, FALSE);    //TIM0 端口输出使能
    
    Bt_M23_Run(TIM0);                          //TIM0 运行

    while (1);

}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


