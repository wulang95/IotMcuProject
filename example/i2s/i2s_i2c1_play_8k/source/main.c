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
 * @brief  Source file for I2S example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32l07x.h"
#include "sysctrl.h"
#include "wm8731.h"
#include "i2s.h"
#include "data_sound_8k_i2s.h"
#include "gpio.h"
#include "i2c.h"
#include "flash.h"

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
const uint16_t *pu16SoundData_l=au16PixieDustSoundI2s_8k;
const uint16_t *pu16SoundData_r=au16PixieDustSoundI2s_8k;

/**
 ******************************************************************************
 ** \brief 内部RCH倍频成48MHz
 **
 ** \param  none
 ** \return none
 ******************************************************************************/
 
void SystemClock_Config(void)
{
    stc_sysctrl_clk_cfg_t ClkInitStruct;
    stc_sysctrl_pll_cfg_t PLLCfg;    
    
    //开启FLASH外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralFlash, TRUE);    
    //因将要倍频的PLL作为系统时钟HCLK会达到48MHz：所以此处预先设置FLASH 读等待周期为1 cycle(默认值为0 cycle)
    Flash_WaitCycle(FlashWaitCycle2);    
    //时钟初始化前，优先设置要使用的时钟源：此处配置PLL
    Sysctrl_SetRCHTrim(SysctrlRchFreq4MHz);             //PLL使用RCH作为时钟源，因此需要先设置RCH    
    
    PLLCfg.enInFreq    = SysctrlPllInFreq4_6MHz;     //RCH 4MHz
    PLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;  //PLL 输出48MHz
    PLLCfg.enPllClkSrc = SysctrlPllRch;              //输入时钟源选择RCH
    PLLCfg.enPllMul    = SysctrlPllMul12;            //4MHz x 12 = 48MHz
    Sysctrl_SetPLLFreq(&PLLCfg);           
    //选择PLL作为HCLK时钟源;
    ClkInitStruct.enClkSrc  = SysctrlClkPLL;
    //HCLK SYSCLK
    ClkInitStruct.enHClkDiv = SysctrlHclkDiv1;
    //PCLK 为HCLK
    ClkInitStruct.enPClkDiv = SysctrlPclkDiv1;
    //系统时钟初始化
    Sysctrl_ClkInit(&ClkInitStruct);
}	

/**
 ******************************************************************************
    ** \brief  配置涉及到的GPIO引脚功能
    **
    ** \param  无
    **               
    ** \retval 无
    **         
 ******************************************************************************/
void GPIO_Config(void)
{  
    stc_gpio_cfg_t         GpioInitStruct;
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //开启GPIO模块时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralI2s0,TRUE);  //开启I2S0模块时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralI2c1,TRUE);  //开启I2C1模块时钟
    /*************************配置I2S0引脚***********************/
    M0P_SYSCTRL->PERI_CLKEN1_f.I2S0=1;         //使能I2S0模块时钟
    GpioInitStruct.enDir = GpioDirOut;              //引脚配置成输出
    GpioInitStruct.enPu  = GpioPuEnable;            //引脚使能上拉
    GpioInitStruct.enDrv = GpioDrvH;                //引脚强驱动
    Gpio_Init(GpioPortD, GpioPin8, &GpioInitStruct);
    Gpio_SetAfMode(GpioPortD, GpioPin8, GpioAf2);   //引脚服用为I2S0_SCK
    
    GpioInitStruct.enDir = GpioDirOut;              //引脚配置成输出
    GpioInitStruct.enPu  = GpioPuEnable;            //引脚使能上拉
    GpioInitStruct.enDrv = GpioDrvH;                //引脚强驱动
    Gpio_Init(GpioPortD, GpioPin10, &GpioInitStruct);
    Gpio_SetAfMode(GpioPortD, GpioPin10, GpioAf2);   //引脚服用为I2S0_SD   
    
    GpioInitStruct.enDir = GpioDirOut;              //引脚配置成输出
    GpioInitStruct.enPu  = GpioPuEnable;            //引脚使能上拉
    GpioInitStruct.enDrv = GpioDrvH;                //引脚强驱动
    Gpio_Init(GpioPortD, GpioPin11, &GpioInitStruct);
    Gpio_SetAfMode(GpioPortD, GpioPin11, GpioAf2);  //引脚服用为I2S0_WS  
    /*************************配置I2C1引脚***********************/    
    GpioInitStruct.enDir = GpioDirOut;              //引脚配置成输出
    GpioInitStruct.enOD  = GpioOdEnable;            //引脚开漏输出
    GpioInitStruct.enDrv = GpioDrvH;                //引脚强驱动
    Gpio_Init(GpioPortB, GpioPin10, &GpioInitStruct);
    Gpio_SetAfMode(GpioPortB, GpioPin10, GpioAf1);   //引脚服用为I2C1_SCL 

    GpioInitStruct.enDir = GpioDirOut;              //引脚配置成输出
    GpioInitStruct.enOD  = GpioOdEnable;            //引脚开漏输出
    GpioInitStruct.enDrv = GpioDrvH;                //引脚强驱动
    Gpio_Init(GpioPortB, GpioPin11, &GpioInitStruct);
    Gpio_SetAfMode(GpioPortB, GpioPin11, GpioAf1);   //引脚服用为I2C1_SDA
}

/**
 ******************************************************************************
    ** \brief  配置I2C1的引脚功能
    **
    ** \param  无
    **               
    ** \retval 无
    **         
 ******************************************************************************/
void I2C_Config(void)
{
    stc_i2c_cfg_t I2CInitStruct;
    I2CInitStruct.u32Pclk = Sysctrl_GetPClkFreq();              // 获取PCLK时钟
    I2CInitStruct.u32Baud = 100000;                             // 波特率100kHz
    I2CInitStruct.enMode  = I2cMasterMode;                      // I2C主机模式
    I2CInitStruct.bGc = FALSE;                                  // 广播地址应答使能关闭，主模式无效
    I2C_Init(M0P_I2C1, &I2CInitStruct);                         // 模块初始化 
}

/**
 ******************************************************************************
    ** \brief  配置中断向量
    **
    ** \param  无
    **               
    ** \retval 无
    **         
 ******************************************************************************/
void NVIC_Configuration(void)
{
	NVIC_ClearPendingIRQ(SPI0_I2S0_IRQn);      //清除I2S0对应的中断挂起标志位
	NVIC_SetPriority(SPI0_I2S0_IRQn,3);        //设置I2S0中断优先级
	NVIC_EnableIRQ(SPI0_I2S0_IRQn);            //使能I2S0中断
}

/**
 ******************************************************************************
    ** \brief  配置WM8731
    **
    ** \param  无
    **               
    ** \retval 无
    **         
 ******************************************************************************/
void WM8731_Config(void)
{
    stc_wm8731_reg_t wm8731_reg; 
    
	wm8731_reg.LLIN = 0;
	wm8731_reg.RLIN = 0;
	wm8731_reg.LHOUT_f.LHPVOL = 0x6f;
	wm8731_reg.RHOUT_f.RHPVOL = 0x6f;
	wm8731_reg.AAPC = 0x38;
	wm8731_reg.DAPC = 0;
	wm8731_reg.PDC  = 0x04;
	wm8731_reg.DAIF = 0x02;
	wm8731_reg.SC   = 0;
	wm8731_reg.AC   = 0x01;
	wm8731_reg.RESET = 0;	
	WM8731_Init(M0P_I2C1, &wm8731_reg);        //初始化WM8731    
}

/**
 ******************************************************************************
    ** \brief  配置I2S
    **
    ** \param  无
    **               
    ** \retval 无
    **         
 ******************************************************************************/
void I2S_Config(void)
{
    stc_i2s_config_t i2s_config;
	//配置波特率为8K
	i2s_config.i2s_Div   = 94;
	i2s_config.i2s_Fract = 0;
	i2s_config.i2s_Odd   = 0;
	i2s_config.i2s_Mckoe = DISABLE;
	//通信标准为philis，通道长度为16bit，数据长度为16bit，模式为主发送模式	
	i2s_config.i2s_Mode    = I2sMasterSend;       //主模式 发送
	i2s_config.i2s_Std     = i2sstdPhilips;       //I2S PHILIPS标准
	i2s_config.i2s_Cksel   = I2sPclk;             //时钟选择PCLK
	i2s_config.i2s_Datalen = i2sDatlen16Bit;      //16位数据长度
	i2s_config.i2s_Chlen   = i2sChlen16Bit;       //通道数据长度：16bit
	I2s_Init(M0P_I2S0, &i2s_config);              //初始化IIS0
}
/**
 ******************************************************************************
    ** \brief  主函数
    **
    ** \param  无
    **               
    ** \retval 无
    **         
 ******************************************************************************/
int32_t main(void)
{
	SystemClock_Config();              //配置系统时钟
	GPIO_Config();                     //配置使用到的GPIO
	I2C_Config();                      //配置IIC
    WM8731_Config();
    I2S_Config();
    NVIC_Configuration();                         //中断向量表配置
	I2S_ConfIt(M0P_I2S0, I2S_IT_TXE, ENABLE);     //使能IIS0输出中断  
	I2S_Cmd(M0P_I2S0, ENABLE);	                  //使能IIS0中断
	while(1)
	{	
        
	}
}	
	

/**
 ******************************************************************************
 ** \brief  向左声道发送寄存器写16位数据
 **
 ** \param  无
 **               
 ** \retval 无
 **         
 ******************************************************************************/
void i2s0_lsend(void)
{
    uint16_t u16Data;

    u16Data =  *pu16SoundData_l++;
    I2s_SendDataL(M0P_I2S0, u16Data);//将数据装入左声道数据寄存器
    pu16SoundData_l++;
    if(u32WavLen_8k <= (pu16SoundData_l - &au16PixieDustSoundI2s_8k[0]))
    {
        pu16SoundData_l = au16PixieDustSoundI2s_8k;
    }	
}

/**
 ******************************************************************************
 ** \brief  向右声道发送寄存器写16位数据
 **
 ** \param  无
 **               
 ** \retval 无
 ** 
 ******************************************************************************/
void i2s0_rsend(void)
{
    uint16_t u16Data;

    u16Data =  *pu16SoundData_r++;
    I2s_SendDataR(M0P_I2S0, u16Data);//将数据装入右声道数据寄存器
    pu16SoundData_r++;
    if(u32WavLen_8k <= (pu16SoundData_r - &au16PixieDustSoundI2s_8k[0]))
    {
        pu16SoundData_r = au16PixieDustSoundI2s_8k+1;
    }	
}

/**
 ******************************************************************************
 ** \brief  I2S0中断处理函数
 **
 ** \param  无
 **               
 ** \retval 无
 **         
 ******************************************************************************/
void I2s0_IRQHandler(void)
{
    if(I2s_GetStatus(M0P_I2S0, I2S_TXE_L) != RESET)
	{
		i2s0_lsend();
	}
    if(I2s_GetStatus(M0P_I2S0, I2S_TXE_R) != RESET)
	{
		i2s0_rsend();
	}	
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


