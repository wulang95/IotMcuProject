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
 * @brief  Source file for UART example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "uart.h"
#include "gpio.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                            
 ******************************************************************************/
#define SLAVEADDR   0xc0

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
volatile uint8_t u8TxData[2] = {0xaa,0x55};
volatile uint8_t u8RxData[2] = {0x00};
uint8_t u8TxCnt=0,u8RxCnt=0;

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
void App_PortInit(void);
void App_UartCfg(void);

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
    //串口引脚配置
    App_PortInit();
    
    //串口模块配置
    App_UartCfg();
    
    //发送从机地址，接下来中断中发送数据
    M0P_UART1->SBUF = (1<<8)|SLAVEADDR; 
    
    while(1)
    {
        ;
    }
}

//UART1中断函数
void Uart1_IRQHandler(void)
{
    if(Uart_GetStatus(M0P_UART1, UartRC))
    {
        Uart_ClrStatus(M0P_UART1, UartRC);  
        u8RxData[u8RxCnt]=Uart_ReceiveData(M0P_UART1);   //接收数据
        u8RxCnt++;
        if(u8RxCnt>1)                                    //如果已经接收到两个数据
        {
            Uart_DisableIrq(M0P_UART1,UartRxIrq);        //禁止接收中断
        }
    }
    
    if(Uart_GetStatus(M0P_UART1, UartTC))
    {
        Uart_ClrStatus(M0P_UART1, UartTC);
        Uart_SendDataIt(M0P_UART1, u8TxData[u8TxCnt++]); //发送数据
        if(u8TxCnt>1)                                    //如果已经发送两个数据字节
        {
            u8TxCnt = 0;
            u8RxCnt = 0;
            Uart_DisableIrq(M0P_UART1,UartTxIrq);        //禁止发送中断
            Uart_EnableIrq(M0P_UART1,UartRxIrq);         //使能接收中断
        } 
    }

}

//串口引脚配置
void App_PortInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    DDL_ZERO_STRUCT(stcGpioCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);  //GPIO模块时钟使能
    
    stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPortA,GpioPin2,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortA,GpioPin2,GpioAf1);//PA02 TX
    
    Gpio_Init(GpioPortC,GpioPin0,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortC,GpioPin0,GpioAf3);//PC00 CTS
    
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortA,GpioPin3,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortA,GpioPin3,GpioAf1);//PA03 RX
    
    Gpio_Init(GpioPortC,GpioPin1,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortC,GpioPin1,GpioAf3);//PC01 RTS
}

//串口模块配置
void App_UartCfg(void)
{
    stc_uart_cfg_t  stcCfg;
    stc_uart_multimode_t stcMulti;
    stc_uart_baud_t stcBaud;

    DDL_ZERO_STRUCT(stcCfg);
    DDL_ZERO_STRUCT(stcMulti);
    DDL_ZERO_STRUCT(stcBaud);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE); //UART1模块时钟使能
    
    ///<UART Init
    stcCfg.enRunMode        = UartMskMode3;           //模式3
    stcCfg.enStopBit        = UartMsk1bit;            //1bit停止位
    stcCfg.enMmdorCk        = UartMskDataOrAddr;      //多机模式时
    stcCfg.stcBaud.u32Baud  = 9600;                   //波特率9600
    stcCfg.stcBaud.enClkDiv = UartMsk8Or16Div;        //通道采样分频配置
    stcCfg.stcBaud.u32Pclk  = Sysctrl_GetPClkFreq();  //获得外设时钟（PCLK）频率值
    Uart_Init(M0P_UART1, &stcCfg);                    //<串口初始化
        
    Uart_EnableFunc(M0P_UART1,UartRtsFunc);           //使能Rts功能
    Uart_EnableFunc(M0P_UART1,UartCtsFunc);           //使能cts功能
    
    ///<UART中断使能
    Uart_ClrStatus(M0P_UART1,UartRC);                 //<清接收请求
    Uart_ClrStatus(M0P_UART1,UartTC);                 //<清接收请求
    Uart_EnableIrq(M0P_UART1,UartTxIrq);              //<使能串口接收中断    
    EnableNvic(UART1_3_IRQn, IrqLevel3, TRUE);        //<系统中断使能
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/

