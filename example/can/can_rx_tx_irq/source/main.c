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
#include "uart.h"
#include "wdt.h"
#include "rtc.h"
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
stc_can_rxframe_t       stcRxFrame;
stc_can_txframe_t       stcTxFrame;
uint8_t                 u8RxFlag = FALSE;
/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
void Can_IRQHandler(void)
{

    if(TRUE == CAN_IrqFlgGet(CanRxIrqFlg))
    {
        CAN_IrqFlgClr(CanRxIrqFlg);
					printf("Can_IRQHandler");
        CAN_Receive(&stcRxFrame);

        u8RxFlag = TRUE;
    }

}
void App_RtcCfg(void)
{
    stc_rtc_initstruct_t RtcInitStruct;
    Sysctrl_SetPeripheralGate(SysctrlPeripheralRtc,TRUE);//RTC模块时钟打开
    RtcInitStruct.rtcAmpm = RtcPm;                       //12小时制
    RtcInitStruct.rtcClksrc = RtcClkXtl;                 //外部低速时钟
    RtcInitStruct.rtcPrdsel.rtcPrdsel = RtcPrdx;         //周期中断类型PRDX
    RtcInitStruct.rtcPrdsel.rtcPrdx = 1u;                //周期中断时间间隔 1秒
    RtcInitStruct.rtcTime.u8Second = 0x55;               //配置RTC时间
    RtcInitStruct.rtcTime.u8Minute = 0x01;
    RtcInitStruct.rtcTime.u8Hour   = 0x10;
    RtcInitStruct.rtcTime.u8Day    = 0x17;
    RtcInitStruct.rtcTime.u8DayOfWeek = 0x04;
    RtcInitStruct.rtcTime.u8Month  = 0x04;
    RtcInitStruct.rtcTime.u8Year   = 0x19;
    RtcInitStruct.rtcCompen = RtcCompenEnable;           // 使能时钟误差补偿
    RtcInitStruct.rtcCompValue = 0;                      //补偿值  根据实际情况进行补偿
    Rtc_Init(&RtcInitStruct);
    Rtc_AlmIeCmd(TRUE);                                  //使能闹钟中断
    
    EnableNvic(RTC_IRQn, IrqLevel3, TRUE);               //使能RTC中断向量
    Rtc_Cmd(TRUE);                                       //使能RTC开始计数
}
void Rtc_IRQHandler(void)
{
    if(Rtc_GetPridItStatus() == TRUE)
    {
        Rtc_ClearPrdfItStatus();             //清除中断标志位
    }
		printf("213244");
}
uint8_t DDL_ConsoleOutputChar(char c)
{
		return Uart_SendDataPoll(M0P_UART1, c);
}

#if (defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)) || \
    (defined (__ICCARM__) && (__VER__ < 9000000)) || (defined (__CC_ARM))
/**
 * @brief  Re-target fputc function.
 * @param  [in] ch
 * @param  [in] f
 * @retval int32_t
 */
int32_t fputc(int32_t ch, FILE *f)
{
    (void)f;  /* Prevent unused argument compilation warning */

    return (Ok == DDL_ConsoleOutputChar((char)ch)) ? ch : -1;
}

#elif (defined (__ICCARM__) && (__VER__ >= 9000000))
#include <LowLevelIOInterface.h>
#pragma module_name = "?__write"
size_t __dwrite(int handle, const unsigned char *buffer, size_t size)
{
    size_t nChars = 0;
    size_t i;

    if (buffer == NULL) {
        /*
         * This means that we should flush internal buffers.  Since we
         * don't we just return.  (Remember, "handle" == -1 means that all
         * handles should be flushed.)
         */
        return 0;
    }

    /* This template only writes to "standard out" and "standard err",
     * for all other file handles it returns failure. */
    if (handle != _LLIO_STDOUT && handle != _LLIO_STDERR) {
        return _LLIO_ERROR;
    }

    for (i = 0; i < size; i++) {
        if (DDL_ConsoleOutputChar((char)buffer[i]) < 0) {
            return _LLIO_ERROR;
        }

        ++nChars;
    }

    return nChars;
}

#elif defined ( __GNUC__ ) && !defined (__CC_ARM)
/**
 * @brief  Re-target _write function.
 * @param  [in] fd
 * @param  [in] data
 * @param  [in] size
 * @retval int32_t
 */
int32_t _write(int fd, char data[], int32_t size)
{
    int32_t i = -1;

    if (NULL != data) {
        (void)fd;  /* Prevent unused argument compilation warning */

        for (i = 0; i < size; i++) {
            if (Ok != DDL_ConsoleOutputChar(data[i])) {
                break;
            }
        }
    }

    return i ? i : -1;
}
#endif


static void UART1_Print_Init(uint32_t baud)
{
		stc_gpio_cfg_t stcGpioCfg;
		stc_uart_cfg_t    stcCfg;
		DDL_ZERO_STRUCT(stcGpioCfg);
		Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
	
		stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPortA, GpioPin2, &stcGpioCfg);
    Gpio_SetAfMode(GpioPortA, GpioPin2, GpioAf1);
	
		DDL_ZERO_STRUCT(stcCfg);
		Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE);
		stcCfg.enRunMode        = UartMskMode3;
		stcCfg.enStopBit        = UartMsk1bit;           
    stcCfg.enMmdorCk        = UartMskEven;           
    stcCfg.stcBaud.u32Baud  = baud;                  
    stcCfg.stcBaud.enClkDiv = UartMsk8Or16Div;       
    stcCfg.stcBaud.u32Pclk  = Sysctrl_GetPClkFreq(); 
    Uart_Init(M0P_UART1, &stcCfg);     
}
static void App_WdtInit(void)
{
    ///< 开启WDT外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralWdt,TRUE);
    ///< WDT 初始化
    Wdt_Init(WdtResetEn, WdtT3s28);
}
/**
 *******************************************************************************
 ** \brief  Main function of can tx rx Irq mode project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    uint8_t u8Idx = 0;
		uint8_t buf[8] = {0x10, 0x23, 0x14,0x16,0x18,0x19,0x20,0x34};
		delay1ms(1000);
		App_WdtInit();
		Wdt_Start();
		Sysctrl_ClkSourceEnable(SysctrlClkXTL, TRUE);
		App_RtcCfg();
    ///< 系统时钟初始化(8MHz for CanClk)
    App_SysClkInit();
		UART1_Print_Init(115200);
    ///< CAN GPIO 配置
    App_CanGpioInit();

    ///< CAN 初始化配置
    App_CanInit();
		printf("App_CanInit\r\n");
		stcTxFrame.ExtID = 0x1314;
		stcTxFrame.Control_f.DLC = 8;
		stcTxFrame.Control_f.IDE = 1;
		stcTxFrame.Control_f.RTR = 0;
		memcpy(stcTxFrame.Data, buf, 8);
		CAN_SetFrame(&stcTxFrame);
    CAN_TransmitCmd(CanPTBTxCmd);
    while(1)
    {
				Wdt_Feed();
			  delay1ms(500);
				CAN_SetFrame(&stcTxFrame);
				CAN_TransmitCmd(CanPTBTxCmd);
//        if(TRUE == u8RxFlag)
//        {
//            u8RxFlag = FALSE;

//            if(1 == stcRxFrame.Cst.Control_f.RTR)
//            {
//                continue;
//            }

//            //<<Can Tx
//            stcTxFrame.StdID         = stcRxFrame.StdID;
//            stcTxFrame.Control_f.DLC = stcRxFrame.Cst.Control_f.DLC;
//            stcTxFrame.Control_f.IDE = stcRxFrame.Cst.Control_f.IDE;
//            stcTxFrame.Control_f.RTR = stcRxFrame.Cst.Control_f.RTR;

//            for(u8Idx=0; u8Idx<stcRxFrame.Cst.Control_f.DLC; u8Idx++)
//            {
//                stcTxFrame.Data[u8Idx] = stcRxFrame.Data[u8Idx];
//            }

//            CAN_SetFrame(&stcTxFrame);
//            CAN_TransmitCmd(CanPTBTxCmd);

//            CAN_IrqCmd(CanRxIrqEn, TRUE);
//        }

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
		printf("can_clk:%d\r\n", Sysctrl_GetPClkFreq());
		
    
 //   stcCanInitCfg.stcCanBt.PRESC = 2-1;
    stcCanInitCfg.stcCanBt.SEG_1 = 5-2;
    stcCanInitCfg.stcCanBt.SEG_2 = 3-1;
    stcCanInitCfg.stcCanBt.SJW   = 3-1;
	stcCanInitCfg.stcCanBt.PRESC = Sysctrl_GetPClkFreq()/((stcCanInitCfg.stcCanBt.SEG_1 + stcCanInitCfg.stcCanBt.SEG_2 + 3)*250000) - 1;
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

    EnableNvic(CAN_IRQn, IrqLevel0, TRUE);

}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
