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
 * @brief  Source file for LCD example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "lcd.h"
#include "lpm.h"
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

/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                             
 ******************************************************************************/

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
void App_PortCfg(void);
void App_LcdCfg(void);
/**
 ******************************************************************************
 ** \brief  主函数
 ** 
 ** @param  无
 ** \retval 无
 **
 ******************************************************************************/ 
int32_t main(void)
{
    Sysctrl_ClkSourceEnable(SysctrlClkRCL,TRUE);            ///< 使能RCL时钟
    Sysctrl_SetRCLTrim(SysctrlRclFreq32768);                ///< 配置内部低速时钟频率为32.768kHz

    Sysctrl_SetPeripheralGate(SysctrlPeripheralLcd,TRUE);   ///< 开启LCD时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);  ///< 开启GPIO时钟
    
    App_PortCfg();               ///< LCD端口配置
    App_LcdCfg();                ///< LCD模块配置

    Lcd_ClearDisp();             ///< 清屏
    Lcd_WriteRam(0,0x0f0f0f0f);  ///< 赋值寄存器LCDRAM0
    Lcd_WriteRam(1,0x0f0f0f0f);  ///< 赋值寄存器LCDRAM1

    while(1)
    {
        ;
    }
}

/**
 ******************************************************************************
 ** \brief  初始化外部GPIO引脚
 **
 ** \return 无
 ******************************************************************************/
void App_PortCfg(void)
{
    Gpio_SetAnalogMode(EVB_LCD_COM0_PORT, EVB_LCD_COM0_PIN);  //COM0
    Gpio_SetAnalogMode(EVB_LCD_COM1_PORT, EVB_LCD_COM1_PIN);  //COM1
    Gpio_SetAnalogMode(EVB_LCD_COM2_PORT, EVB_LCD_COM2_PIN);  //COM2
    Gpio_SetAnalogMode(EVB_LCD_COM3_PORT, EVB_LCD_COM3_PIN);  //COM3   
                                                        
    Gpio_SetAnalogMode(EVB_LCD_SEG0_PORT, EVB_LCD_SEG0_PIN);  //SEG0
    Gpio_SetAnalogMode(EVB_LCD_SEG1_PORT, EVB_LCD_SEG1_PIN);  //SEG1
    Gpio_SetAnalogMode(EVB_LCD_SEG2_PORT, EVB_LCD_SEG2_PIN);  //SEG2
    Gpio_SetAnalogMode(EVB_LCD_SEG3_PORT, EVB_LCD_SEG3_PIN);  //SEG3
    Gpio_SetAnalogMode(EVB_LCD_SEG4_PORT, EVB_LCD_SEG4_PIN);  //SEG4
    Gpio_SetAnalogMode(EVB_LCD_SEG5_PORT, EVB_LCD_SEG5_PIN);  //SEG5
    Gpio_SetAnalogMode(EVB_LCD_SEG6_PORT, EVB_LCD_SEG6_PIN);  //SEG6
    Gpio_SetAnalogMode(EVB_LCD_SEG7_PORT, EVB_LCD_SEG7_PIN);  //SEG7
    
    Gpio_SetAnalogMode(EVB_LCD_VLCDH_PORT, EVB_LCD_VLCDH_PIN);  //VLCDH
    Gpio_SetAnalogMode(EVB_LCD_VLCD3_PORT, EVB_LCD_VLCD3_PIN);  //VLCD3
    Gpio_SetAnalogMode(EVB_LCD_VLCD2_PORT, EVB_LCD_VLCD2_PIN);  //VLCD2
    Gpio_SetAnalogMode(EVB_LCD_VLCD1_PORT, EVB_LCD_VLCD1_PIN);  //VLCD1
    
}

/**
 ******************************************************************************
 ** \brief  配置LCD
 **
 ** \return 无
 ******************************************************************************/
void App_LcdCfg(void)
{
    stc_lcd_cfg_t LcdInitStruct;
    stc_lcd_segcom_t LcdSegCom;

    LcdSegCom.u32Seg0_31 = 0xffffff00;                              ///< 配置LCD_POEN0寄存器 开启SEG0~SEG7
    LcdSegCom.stc_seg32_51_com0_8_t.seg32_51_com0_8 = 0xffffffff;   ///< 初始化LCD_POEN1寄存器 全部关闭输出端口
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Com0_3 = 0;          ///< 使能COM0~COM3
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Mux = 0;             ///< Mux=0,Seg32_35=0,BSEL=1表示:选择外部电容工作模式，内部电阻断路
    LcdSegCom.stc_seg32_51_com0_8_t.segcom_bit.Seg32_35 = 0;
    Lcd_SetSegCom(&LcdSegCom);                                      ///< LCD COMSEG端口配置

    LcdInitStruct.LcdBiasSrc = LcdExtCap;                          ///< 电容分压模式，需要外部电路配合
    LcdInitStruct.LcdDuty = LcdDuty4;                              ///< 1/4duty
    LcdInitStruct.LcdBias = LcdBias3;                              ///< 1/3 BIAS
    LcdInitStruct.LcdCpClk = LcdClk2k;                             ///< 电压泵时钟频率选择2kHz
    LcdInitStruct.LcdScanClk = LcdClk128hz;                        ///< LCD扫描频率选择128Hz
    LcdInitStruct.LcdMode = LcdMode0;                              ///< 选择模式0
    LcdInitStruct.LcdClkSrc = LcdRCL;                              ///< LCD时钟选择RCL
    LcdInitStruct.LcdEn   = LcdEnable;                             ///< 使能LCD模块
    Lcd_Init(&LcdInitStruct);
}


/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


