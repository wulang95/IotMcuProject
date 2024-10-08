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
#include "ddl.h"
#include "sysctrl.h"
#include "gpio.h"
#include "can.h"

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
static void App_SysClkInit(void);
static void App_CanGpioInit(void);
static void App_CanInit(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
__IO uint32_t can_id, can_data[2]={0};
stc_can_txframe_t       stcTxFrame;
stc_can_rxframe_t       stcRxFrame;
/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief  Main function of can rx tx poll mode project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    uint8_t u8Idx = 0;

    ///< 系统时钟初始化(8MHz for CanClk)
    App_SysClkInit();

    ///< CAN GPIO 配置
    App_CanGpioInit();

    ///< CAN 初始化配置
    App_CanInit();

    while(1)
    {
        if(TRUE == CAN_IrqFlgGet(CanRxIrqFlg))
        {
            CAN_IrqFlgClr(CanRxIrqFlg);

            CAN_Receive(&stcRxFrame);

            if(1 == stcRxFrame.Cst.Control_f.RTR)
            {
                continue;
            }

            //<<Can Tx
            stcTxFrame.StdID         = stcRxFrame.StdID;
            stcTxFrame.Control_f.DLC = stcRxFrame.Cst.Control_f.DLC;
            stcTxFrame.Control_f.IDE = stcRxFrame.Cst.Control_f.IDE;
            stcTxFrame.Control_f.RTR = stcRxFrame.Cst.Control_f.RTR;

            for(u8Idx=0; u8Idx<stcRxFrame.Cst.Control_f.DLC; u8Idx++)
            {
                stcTxFrame.Data[u8Idx] = stcRxFrame.Data[u8Idx];
            }

            CAN_SetFrame(&stcTxFrame);
            CAN_TransmitCmd(CanPTBTxCmd);
        }

    }

}


static void App_SysClkInit(void)
{
    ///< 切换时钟前（根据外部高速晶振）设置XTH频率范围,配置晶振参数，使能目标时钟，此处为8MHz
    Sysctrl_SetXTHFreq(SysctrlXthFreq4_8MHz);
    Sysctrl_XTHDriverCfg(SysctrlXtalDriver1);
    Sysctrl_SetXTHStableTime(SysctrlXthStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTH, TRUE);
    delay1ms(10);

    ///< 时钟切换
    Sysctrl_SysClkSwitch(SysctrlClkXTH);

}

static void App_CanGpioInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;

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

    Gpio_Init(EVB_CAN_RX_PORT, EVB_CAN_RX_PIN, &stcGpioCfg);
    stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(EVB_CAN_TX_PORT, EVB_CAN_TX_PIN, &stcGpioCfg);
    Gpio_Init(EVB_CAN_STB_PORT, EVB_CAN_STB_PIN, &stcGpioCfg);

    ///<CAN RX\TX复用功能配置
    Gpio_SetAfMode(EVB_CAN_RX_PORT, EVB_CAN_RX_PIN, GpioAf1);
    Gpio_SetAfMode(EVB_CAN_TX_PORT, EVB_CAN_TX_PIN, GpioAf1);

    ///<STB 低-PHY有效
    Gpio_ClrIO(EVB_CAN_STB_PORT, EVB_CAN_STB_PIN);
}

static void App_CanInit(void)
{
    stc_can_init_config_t   stcCanInitCfg;
    stc_can_filter_t        stcFilter;


    Sysctrl_SetPeripheralGate(SysctrlPeripheralCan, TRUE);

    //<<CAN 波特率配置
    stcCanInitCfg.stcCanBt.PRESC = 1-1;
    stcCanInitCfg.stcCanBt.SEG_1 = 5-2;
    stcCanInitCfg.stcCanBt.SEG_2 = 3-1;
    stcCanInitCfg.stcCanBt.SJW   = 3-1;

    stcCanInitCfg.stcWarningLimit.CanErrorWarningLimitVal = 16-1;
    stcCanInitCfg.stcWarningLimit.CanWarningLimitVal = 10;

    stcCanInitCfg.enCanRxBufAll  = CanRxNormal;
    stcCanInitCfg.enCanRxBufMode = CanRxBufNotStored;
    stcCanInitCfg.enCanSTBMode   = CanSTBFifoMode;

    CAN_Init(&stcCanInitCfg);

    //<<CAN 滤波器配置
    stcFilter.enAcfFormat = CanAllFrames;
    stcFilter.enFilterSel = CanFilterSel1;
    stcFilter.u32CODE     = 0x00000000;
    stcFilter.u32MASK     = 0x1FFFFFFF;
    CAN_FilterConfig(&stcFilter, TRUE);

    //<<Can Irq Enable
    CAN_IrqCmd(CanRxIrqEn, TRUE);

}
/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
