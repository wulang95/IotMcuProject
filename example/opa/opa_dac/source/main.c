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
#include "dac.h"
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
    ** \brief  配置DAC
    ** 
    ** @param  无
    ** \retval 无
    **
******************************************************************************/ 
void DAC_Config(void)
{
    stc_dac_cfg_t  dac_initstruct;
    dac_initstruct.boff_t = DacBoffEnable;    //DAC0通道输出缓冲器使能
    dac_initstruct.ten_t  = DacTenEnable;     //DAC0通道触发使能
    dac_initstruct.sref_t = DacVoltageAvcc;
    dac_initstruct.mamp_t = DacMenp2047;
    dac_initstruct.wave_t = DacTrWaveEnable;
    dac_initstruct.tsel_t = DacSwTriger;      //软件触发方式
    dac_initstruct.align  = DacRightAlign;    //右对齐
    dac_initstruct.dhr12  = 50;           //三角波基值
    Dac0_Init(&dac_initstruct);
    Dac0_Cmd(TRUE);
}
/**
 ******************************************************************************
 ** \brief  主函数
 **
 ** \return 无
 ******************************************************************************/
int32_t main(void)
{   
    uint16_t tmp;
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);    /* 使能GPIO模块的时钟 */
    Sysctrl_SetPeripheralGate(SysctrlPeripheralOpa, TRUE);     /* 使能OPA模块的时钟 */
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);  /* 使能BGR模块的时钟 */
    Sysctrl_SetPeripheralGate(SysctrlPeripheralDac, TRUE);     /* 使能DAC模块的时钟 */
    Bgr_BgrEnable(); /* 使能BAR模块 */

    Gpio_SetAnalogMode(GpioPortA, GpioPin4);  /* PA04作为DAC的模拟输出口 */
    
    Opa_DacBufCmd(Opa_Dac0Buff, TRUE);        /* 使能DAC0缓存功能 */

    DAC_Config();                             /* DAC配置 */
    while (1)
    {
        Dac0_SoftwareTriggerCmd();             /* 软件触发输出 */
        for(tmp=0; tmp<100; tmp++);           /* 延时 */
    }
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


