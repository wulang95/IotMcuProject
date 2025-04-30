#include "system.h"
#include "timer3.h"
#include "flash.h"
#include "app_ota.h"
#include <stdio.h>
#include <stdbool.h>
#include <time.h>

static stc_can_rxframe_t       stcRxFrame;
uint32_t sys_time[TIME_MAX];
uint8_t ship_mode_flag;

struct can_rx_frame_s{
		stc_can_rxframe_t Can_RxFrame[CAN_RX_FIFO_SIZE];
		uint8_t len;
		uint8_t in;
		uint8_t out;
		uint8_t num;
		uint8_t sync;
} can_rx_frame;

static void can_rx_frame_init()
{
		can_rx_frame.len = CAN_RX_FIFO_SIZE;
		can_rx_frame.in = 0;
		can_rx_frame.out = 0;
		can_rx_frame.num = 0;
		memset(can_rx_frame.Can_RxFrame, 0, sizeof(stc_can_rxframe_t)*CAN_RX_FIFO_SIZE);
}

static void can_rx_frame_in(stc_can_rxframe_t stcRxFrame)
{
	if(can_rx_frame.num < can_rx_frame.len)
			can_rx_frame.num++;
	memcpy(&can_rx_frame.Can_RxFrame[can_rx_frame.in], &stcRxFrame, sizeof(stc_can_rxframe_t));
	can_rx_frame.in++;
	if(can_rx_frame.in == can_rx_frame.len) can_rx_frame.in = 0;
}

static void can_rx_frame_out(stc_can_rxframe_t *stcRxFrame)
{
	if(can_rx_frame.num)
		can_rx_frame.num--;
	else return;
	memcpy(stcRxFrame, &can_rx_frame.Can_RxFrame[can_rx_frame.out], sizeof(stc_can_rxframe_t));
	can_rx_frame.out++;
	if(can_rx_frame.out == can_rx_frame.len) can_rx_frame.out = 0;
}

void can_rx_dispitch(handel process)
{
	stc_can_rxframe_t RxFrame;
	if(can_rx_frame.num){
		can_rx_frame_out(&RxFrame);
		if(process) process(RxFrame);
	}
//	printf("can_num:%d, ota:%d\r\n", can_rx_frame.num, can_ota.ota_state);
}
void Can_IRQHandler(void)
{
		
	  if(TRUE == CAN_IrqFlgGet(CanRxIrqFlg))
    {
        CAN_IrqFlgClr(CanRxIrqFlg);
        CAN_Receive(&stcRxFrame);
				if(can_ota.ota_state == 1) {
					if(((stcRxFrame.ExtID>>16)&0xff)!= 0xd0)
						CAN_Rec_Prase(stcRxFrame);
				} else {
					can_rx_frame_in(stcRxFrame);
				}		
//				can_rx_frame_in(stcRxFrame);
    }
}



void App_Timer3Cfg(uint16_t u16Period)
{
    uint16_t                    u16ArrValue;
    uint16_t                    u16CntValue;
    stc_tim3_mode0_cfg_t     stcTim3BaseCfg;
    
    //????????
    DDL_ZERO_STRUCT(stcTim3BaseCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralTim3, TRUE); //Base Timer??????
    
    stcTim3BaseCfg.enWorkMode = Tim3WorkMode0;              //?????
    stcTim3BaseCfg.enCT       = Tim3Timer;                  //?????,???????PCLK
    stcTim3BaseCfg.enPRS      = Tim3PCLKDiv16;              //PCLK/16
    stcTim3BaseCfg.enCntMode  = Tim316bitArrMode;           //????16????/???
    stcTim3BaseCfg.bEnTog     = FALSE;
    stcTim3BaseCfg.bEnGate    = FALSE;
    stcTim3BaseCfg.enGateP    = Tim3GatePositive;
    
    Tim3_Mode0_Init(&stcTim3BaseCfg);                       //TIM3 ???0?????
        
    u16ArrValue = 0x10000 - u16Period ;
    
    Tim3_M0_ARRSet(u16ArrValue);                            //?????(ARR = 0x10000 - ??)
    
    u16CntValue = 0x10000 - u16Period;
    
    Tim3_M0_Cnt16Set(u16CntValue);                          //??????
    
    Tim3_ClearIntFlag(Tim3UevIrq);                          //?????
    Tim3_Mode0_EnableIrq();                                 //??TIM3??(??0???????)
    EnableNvic(TIM3_IRQn, IrqLevel3, TRUE);                 //TIM3 ??? 
}
void delay_us(uint32_t us)
{
    volatile uint32_t i = 0;
    volatile uint16_t j = 0;
    for(i = 0; i < us; i++)
    {
        __asm__ volatile ("nop");
        __asm__ volatile ("nop");
        __asm__ volatile ("nop");
        __asm__ volatile ("nop");
    }
}
void App_SystemClkInit_PLL48M_byXTH(void)
{
    stc_sysctrl_pll_cfg_t stcPLLCfg;    
    

    Sysctrl_SetXTHFreq(SysctrlXthFreq4_8MHz);
    Sysctrl_XTHDriverCfg(SysctrlXtalDriver3);
    Sysctrl_SetXTHStableTime(SysctrlXthStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTH, TRUE);
    delay1ms(10);
    
    stcPLLCfg.enInFreq    = SysctrlPllInFreq6_12MHz;    //XTH 8MHz
    stcPLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;  //PLL ??
    stcPLLCfg.enPllClkSrc = SysctrlPllXthXtal;          //???????XTH
    stcPLLCfg.enPllMul    = SysctrlPllMul6;             //8MHz x 6 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg); 
    

    Flash_WaitCycle(FlashWaitCycle1);    
//		delay100us(5);
		delay_us(500);

    Sysctrl_ClkSourceEnable(SysctrlClkPLL, TRUE);    

    Sysctrl_SysClkSwitch(SysctrlClkPLL);

}

void App_SystemClkInit_PLL32M_byXTH(void)
{
    stc_sysctrl_pll_cfg_t stcPLLCfg;    
    

    Sysctrl_SetXTHFreq(SysctrlXthFreq4_8MHz);
    Sysctrl_XTHDriverCfg(SysctrlXtalDriver3);
    Sysctrl_SetXTHStableTime(SysctrlXthStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTH, TRUE);
    delay1ms(10);
    
    stcPLLCfg.enInFreq    = SysctrlPllInFreq6_12MHz;    //XTH 8MHz
    stcPLLCfg.enOutFreq   = SysctrlPllOutFreq24_36MHz;  //PLL ??
    stcPLLCfg.enPllClkSrc = SysctrlPllXthXtal;          //???????XTH
    stcPLLCfg.enPllMul    = SysctrlPllMul4;             //8MHz x 6 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg); 
    

    Flash_WaitCycle(FlashWaitCycle1);    
		delay100us(5);

    Sysctrl_ClkSourceEnable(SysctrlClkPLL, TRUE);    

    Sysctrl_SysClkSwitch(SysctrlClkPLL);

}

static void App_SysClkInit(void)
{
		Sysctrl_SetHCLKDiv(SysctrlHclkDiv1);
    Sysctrl_SetPCLKDiv(SysctrlPclkDiv1);
		App_SystemClkInit_PLL48M_byXTH();
//		App_SystemClkInit_PLL32M_byXTH();
}

void jbd_sysclk_init(uint8_t clk_type)
{
    en_flash_waitcycle_t enWaitCycle;
    stc_sysctrl_pll_cfg_t stcPLLCfg;

    M0P_SYSCTRL->RCH_CR = *(volatile unsigned int *)(0x100C08); // 4 M
    // M0P_SYSCTRL->RCH_CR = *(volatile unsigned int *)(0x100C06);  //8 M
    // M0P_SYSCTRL->RCH_CR = *(volatile unsigned int *)(0x100C04);  //16 M
    // M0P_SYSCTRL->RCH_CR = *(volatile unsigned int *)(0x100C00);  //24 M

    // ????????
    DDL_ZERO_STRUCT(stcPLLCfg);

    // 8MHz x 4 = 32MHz
    if (clk_type == CLK_TYPE_INT)
    {
        Sysctrl_ClkSourceEnable(SysctrlClkRCH, TRUE);
        Sysctrl_SysClkSwitch(SysctrlClkRCH);
        Sysctrl_ClkSourceEnable(SysctrlClkXTH, FALSE);
        Sysctrl_ClkSourceEnable(SysctrlClkPLL, FALSE);
        Flash_WaitCycle(FlashWaitCycle0);
    } else
    {
			//	App_SystemClkInit_PLL32M_byXTH();
				App_SystemClkInit_PLL48M_byXTH();
		}
}


static void cat1_init(void)
{
		stc_gpio_cfg_t stcGpioCfg;
		Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
	
		stcGpioCfg.enDir = GpioDirOut;
		stcGpioCfg.enDrv = GpioDrvL;
		stcGpioCfg.enPu = GpioPuDisable;
		stcGpioCfg.enPd = GpioPdDisable;
		stcGpioCfg.enOD = GpioOdDisable;
		stcGpioCfg.enCtrlMode = GpioAHB;
	
		Gpio_Init(GpioPortB, GpioPin15, &stcGpioCfg); 	//4G_PWKEY
		Gpio_Init(GpioPortB, GpioPin14, &stcGpioCfg);		//4G_RST
		Gpio_Init(GpioPortB, GpioPin13, &stcGpioCfg);		//CAT1_POWER
		Gpio_Init(GpioPortA, GpioPin5, &stcGpioCfg);  //mcu_out_cat1
		
		Gpio_ClrIO(GpioPortB, GpioPin14);
		Gpio_ClrIO(GpioPortB, GpioPin13);

		Gpio_ClrIO(GpioPortB, GpioPin15);
		Gpio_SetIO(GpioPortA, GpioPin5);
	
		stcGpioCfg.enDir = GpioDirIn;
		Gpio_Init(GpioPortA, GpioPin6, &stcGpioCfg); 
		Gpio_EnableIrq(GpioPortA, GpioPin6, GpioIrqFalling);
		Gpio_ClearIrq(GpioPortA, GpioPin6); 
		EnableNvic(PORTA_IRQn, IrqLevel3, TRUE);
}


uint8_t g_cat1_state;
void cat1_power_on()
{
	Gpio_SetIO(GpioPortB, GpioPin13);
	delay1ms(500);
	Gpio_ClrIO(GpioPortB, GpioPin13);
	delay1ms(500);
	Gpio_ClrIO(GpioPortB, GpioPin15);
	delay1ms(500);
	Gpio_SetIO(GpioPortB, GpioPin15);
	delay1ms(3000);
	Gpio_ClrIO(GpioPortB, GpioPin15);
	printf("cat1 power ok\r\n");
//	static uint8_t state = 0;
//	if(g_cat1_state == CAT1_POWERON) return;
//	switch(state) {
//		case 0:
//				Gpio_ClrIO(GpioPortB, GpioPin15);
//				delay1ms(500);
//				SET_SYS_TIME(CAT1_TM, 500);
//				state = 1;
//		break;
//		case 1:
//				if(CHECK_SYS_TIME(CAT1_TM) == 0) {
//						Gpio_SetIO(GpioPortB, GpioPin15);
//						delay1ms(3000);
//						SET_SYS_TIME(CAT1_TM, 3000);
//						state = 2;
//				}
//		break;
//		case 2:
//			if(CHECK_SYS_TIME(CAT1_TM) == 0) {
//						Gpio_ClrIO(GpioPortB, GpioPin15);
//						state = 0;
//						g_cat1_state = CAT1_POWERON;
//						printf("cat1 power ok\r\n");
//			}	
//		break;
//	}
}

void cat1_power_control()
{
	static uint8_t state = 0;
	if(g_cat1_state == CAT1_POWERON) return;
	switch(state) {
		case 0:
				Gpio_ClrIO(GpioPortB, GpioPin13);
				SET_SYS_TIME(CAT1_TM, 500);
				state = 1;
		break;
		case 1:
				if(CHECK_SYS_TIME(CAT1_TM) == 0){
						Gpio_SetIO(GpioPortB, GpioPin13);
						SET_SYS_TIME(CAT1_TM, 500);
						state = 2;
				}
		break;
		case 2:
				if(CHECK_SYS_TIME(CAT1_TM) == 0){
						Gpio_ClrIO(GpioPortB, GpioPin15);
						SET_SYS_TIME(CAT1_TM, 500);
						state = 3;
				}
		break;
		case 3:
				if(CHECK_SYS_TIME(CAT1_TM) == 0) {
						Gpio_SetIO(GpioPortB, GpioPin15);
						SET_SYS_TIME(CAT1_TM, 3000);
						state = 4;
				}
		break;
		case 4:
			if(CHECK_SYS_TIME(CAT1_TM) == 0) {
						Gpio_ClrIO(GpioPortB, GpioPin15);
						state = 0;
						printf("cat1 power ok\r\n");
						g_cat1_state = CAT1_POWERON;
						SET_SYS_TIME(CAT1_ERROR_TM, 30000);
			}	
		break;
	}
}


static void App_Can_init(uint32_t baud)
{
		stc_gpio_cfg_t stcGpioCfg;
		stc_can_init_config_t   stcCanInitCfg;
		stc_can_filter_t        stcFilter;
		can_rx_frame_init();
		Sysctrl_SetPeripheralGate(SysctrlPeripheralCan, TRUE);
		Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
		stcGpioCfg.enDir = GpioDirIn;
		stcGpioCfg.enDrv = GpioDrvL;
		stcGpioCfg.enPu = GpioPuDisable;
		stcGpioCfg.enPd = GpioPdDisable;
		stcGpioCfg.enOD = GpioOdDisable;
		stcGpioCfg.enCtrlMode = GpioAHB;
	
		Gpio_Init(GpioPortB, GpioPin8, &stcGpioCfg);
    stcGpioCfg.enDir = GpioDirOut;
		Gpio_Init(GpioPortB, GpioPin9, &stcGpioCfg);
		Gpio_Init(GpioPortA, GpioPin8, &stcGpioCfg);
	
		Gpio_SetAfMode(GpioPortB, GpioPin8, GpioAf3);
    Gpio_SetAfMode(GpioPortB, GpioPin9, GpioAf5);

    Gpio_ClrIO(GpioPortA, GpioPin8);
    stcCanInitCfg.stcCanBt.SEG_1 = 5-2;
    stcCanInitCfg.stcCanBt.SEG_2 = 3-1;
    stcCanInitCfg.stcCanBt.SJW   = 3-1;
		stcCanInitCfg.stcCanBt.PRESC = Sysctrl_GetPClkFreq()/((stcCanInitCfg.stcCanBt.SEG_1 + stcCanInitCfg.stcCanBt.SEG_2 + 3)*baud) - 1;
    stcCanInitCfg.stcWarningLimit.CanErrorWarningLimitVal = 16-1;
    stcCanInitCfg.stcWarningLimit.CanWarningLimitVal = 10;

    stcCanInitCfg.enCanRxBufAll  = CanRxNormal;
    stcCanInitCfg.enCanRxBufMode = CanRxBufOverwritten;
    stcCanInitCfg.enCanSTBMode   = CanSTBPrimaryMode;

    CAN_Init(&stcCanInitCfg);
		
    stcFilter.enAcfFormat = CanAllFrames;
    stcFilter.enFilterSel = CanFilterSel1;
		stcFilter.u32CODE     = 0x00000000;
    stcFilter.u32MASK     = 0x1FFFFFFF;
    CAN_FilterConfig(&stcFilter, TRUE);
		CAN_ModeConfig(CanTxSignalPrimaryMode, CanSelfAckEnable, TRUE);
		
		CAN_IrqCmd(CanRxIrqEn, TRUE);
		EnableNvic(CAN_IRQn, IrqLevel0, TRUE);
}


uint8_t rx_buff[IOT_BUFF_SIZE];
uint8_t gps_rx_buff[GPS_BUFF_SIZE];
uint8_t can_ota_buf[OTA_DATA_SIZE];


void UART0_SWITCH_BAUD(uint32_t baud)
{
		stc_uart_cfg_t    stcCfg;
		DDL_ZERO_STRUCT(stcCfg);
		Sysctrl_SetPeripheralGate(SysctrlPeripheralUart0,TRUE);
		stcCfg.enRunMode        = UartMskMode1;
		stcCfg.enStopBit        = UartMsk1bit;           
    stcCfg.enMmdorCk        = UartMskDataOrAddr;           
    stcCfg.stcBaud.u32Baud  = baud;                  
    stcCfg.stcBaud.enClkDiv = UartMsk8Or16Div;       
    stcCfg.stcBaud.u32Pclk  = Sysctrl_GetPClkFreq(); 
		Uart_DisableIrq(M0P_UART0,UartRxIrq);
    Uart_Init(M0P_UART0, &stcCfg); 
		Uart_ClrStatus(M0P_UART0,UartRC);
		Uart_EnableIrq(M0P_UART0,UartRxIrq);
}

static void UART0_Iot_Init(uint32_t baud)
{
		stc_gpio_cfg_t stcGpioCfg;
		stc_uart_cfg_t    stcCfg;
		DDL_ZERO_STRUCT(stcGpioCfg);
		Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
	
		stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPortA, GpioPin9, &stcGpioCfg);
    Gpio_SetAfMode(GpioPortA, GpioPin9, GpioAf1);
	
		stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortA, GpioPin10, &stcGpioCfg);
    Gpio_SetAfMode(GpioPortA, GpioPin10, GpioAf1); 
	
		DDL_ZERO_STRUCT(stcCfg);
		Sysctrl_SetPeripheralGate(SysctrlPeripheralUart0,TRUE);
		stcCfg.enRunMode        = UartMskMode1;
		stcCfg.enStopBit        = UartMsk1bit;           
    stcCfg.enMmdorCk        = UartMskDataOrAddr;           
    stcCfg.stcBaud.u32Baud  = baud;                  
    stcCfg.stcBaud.enClkDiv = UartMsk8Or16Div;       
    stcCfg.stcBaud.u32Pclk  = Sysctrl_GetPClkFreq(); 
    Uart_Init(M0P_UART0, &stcCfg);     
		Uart_ClrStatus(M0P_UART0,UartRC);
		Uart_EnableIrq(M0P_UART0,UartRxIrq);
		EnableNvic(UART0_2_IRQn, IrqLevel3, TRUE);
		FIFO_Init(IOT_UART, rx_buff, IOT_BUFF_SIZE);
		FIFO_Init(CAN_OTA, can_ota_buf, OTA_DATA_SIZE);
}

static uint8_t u8RxData_0;

void Uart0_IRQHandler()
{
		if(Uart_GetStatus(M0P_UART0, UartRC))         
    {
        Uart_ClrStatus(M0P_UART0, UartRC);        
        u8RxData_0 = Uart_ReceiveData(M0P_UART0);   
				FIFO_Write_OneByte(IOT_UART, u8RxData_0);
    }
}

void Uart0_Send_Iot(uint8_t *buf, uint16_t len)
{
	uint16_t i = 0;
	while(len--){
		Uart_SendDataPoll(M0P_UART0, buf[i]);
		i++;
	}
}
static uint8_t u8RxData_1;
void Uart1_IRQHandler()
{
		if(Uart_GetStatus(M0P_UART1, UartRC))         
    {
				
        Uart_ClrStatus(M0P_UART1, UartRC);        
        u8RxData_1 = Uart_ReceiveData(M0P_UART1);   
				FIFO_Write_OneByte(GPS_UART, u8RxData_1);
    }
}

void Uart1_Send_gps(uint8_t *buf, uint16_t len)
{
	uint16_t i = 0;
	while(len--){
		Uart_SendDataPoll(M0P_UART1, buf[i]);
		i++;
	}
}
void simulation_uart_tx(uint8_t temp_data);

uint8_t DDL_ConsoleOutputChar(char c)
{
	simulation_uart_tx(c);
	return Ok;
	//	return Uart_SendDataPoll(M0P_UART1, c);
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

void simulation_uart_tx(uint8_t temp_data)
{
		uint8_t bit;
		for(bit = 0; bit < 10; bit++)
		{
				if(bit == 0){
						Gpio_ClrIO(GpioPortA, GpioPin2);
				} else if(bit == 9){
						Gpio_SetIO(GpioPortA, GpioPin2);
				} else {
						if(temp_data & (1 << (bit - 1)))
							Gpio_SetIO(GpioPortA, GpioPin2);
						else 
							Gpio_ClrIO(GpioPortA, GpioPin2);
				}
				delay100us(1);  //9600
		}
}

static void print_gpio_init()
{
		stc_gpio_cfg_t stcGpioCfg;
		DDL_ZERO_STRUCT(stcGpioCfg);
		Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
	
		stcGpioCfg.enDir = GpioDirOut;
		stcGpioCfg.enDrv = GpioDrvL;
		stcGpioCfg.enPu = GpioPuDisable;
		stcGpioCfg.enPd = GpioPdDisable;
		stcGpioCfg.enOD = GpioOdDisable;
		stcGpioCfg.enCtrlMode = GpioAHB;
	
		Gpio_Init(GpioPortA, GpioPin2, &stcGpioCfg); 	
}

static void UART1_GPS_Init(uint32_t baud)
{
		stc_gpio_cfg_t stcGpioCfg;
		stc_uart_cfg_t    stcCfg;
		DDL_ZERO_STRUCT(stcGpioCfg);
		Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
	
		stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortA, GpioPin3, &stcGpioCfg);
    Gpio_SetAfMode(GpioPortA, GpioPin3, GpioAf1);
	
		stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPortA, GpioPin4, &stcGpioCfg);
    Gpio_SetAfMode(GpioPortA, GpioPin4, GpioAf2); 

		Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE);
		stcCfg.enRunMode        = UartMskMode1;
		stcCfg.enStopBit        = UartMsk1bit;           
    stcCfg.enMmdorCk        = UartMskDataOrAddr;           
    stcCfg.stcBaud.u32Baud  = baud;                  
    stcCfg.stcBaud.enClkDiv = UartMsk8Or16Div;       
    stcCfg.stcBaud.u32Pclk  = Sysctrl_GetPClkFreq(); 
    Uart_Init(M0P_UART1, &stcCfg);   
		
		Uart_ClrStatus(M0P_UART1,UartRC);
		Uart_EnableIrq(M0P_UART1,UartRxIrq);
		EnableNvic(UART1_3_IRQn, IrqLevel3, TRUE);
		FIFO_Init(GPS_UART, gps_rx_buff, GPS_BUFF_SIZE);		
}

void Iot_Can_Send(stc_can_txframe_t stcTxFrame)
{
//		printf("%08x\r\n", stcTxFrame.ExtID);
		while(CAN_StatusGet(CanTxActive) != FALSE);
		stcTxFrame.enBufferSel = CanSTBSel;
	  CAN_IrqFlgClr(CanTxSecondaryIrqFlg);
		CAN_SetFrame(&stcTxFrame);
		CAN_TransmitCmd(CanSTBTxAllCmd);
		delay100us(3);
}


static void App_WdtInit(en_wdt_time_t W_Tm)
{
    Sysctrl_SetPeripheralGate(SysctrlPeripheralWdt,TRUE);
    Wdt_Init(WdtResetEn, W_Tm);
}

uint32_t cur_tick;
uint32_t systick_diff(uint32_t time)
{
		uint32_t dif;
		const uint32_t tick = cur_tick;
		if(tick > time) {
				dif = tick - time;
		} else {
				dif = 0xffffffff - time + tick;
		}
		return dif;
}


void SysTick_IRQHandler(void)
{
//	cur_tick++;
//	for(uint8_t i = 0; i < TIME_MAX; i++)
//	{
//			if(sys_time[i])sys_time[i]--;
//	}
}

void Tim3_IRQHandler(void)
{
	if(TRUE == Tim3_GetIntFlag(Tim3UevIrq)){	
			Tim3_ClearIntFlag(Tim3UevIrq); 
			cur_tick++;
			for(uint8_t i = 0; i < TIME_MAX; i++)
			{
					if(sys_time[i])sys_time[i]--;
			}
	}
}


static void Sys_Deinit()
{
		stc_gpio_cfg_t stcGpioCfg;
		Sysctrl_SetPeripheralGate(SysctrlPeripheralCan, FALSE);
		Sysctrl_SetPeripheralGate(SysctrlPeripheralUart0,FALSE);
		Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,FALSE);
		DDL_ZERO_STRUCT(stcGpioCfg);
		
		stcGpioCfg.enDir = GpioDirIn;
		Gpio_Init(GpioPortA, GpioPin9, &stcGpioCfg);
		Gpio_Init(GpioPortB, GpioPin8, &stcGpioCfg);
		Gpio_SetIO(GpioPortA, GpioPin8);		//关闭CAN
}
static uint8_t week_flag;
void PortA_IRQHandler(void)
{
		if(TRUE == Gpio_GetIrqStatus(GpioPortA, GpioPin10))
		{
			Gpio_ClearIrq(GpioPortA, GpioPin10); 		
		}
		if(TRUE == Gpio_GetIrqStatus(GpioPortA, GpioPin6))
		{
			Gpio_ClearIrq(GpioPortA, GpioPin6); 		
		}
		
		if(TRUE == Gpio_GetIrqStatus(GpioPortA, GpioPin1))  /* 外部电源接入  */
		{
			Gpio_ClearIrq(GpioPortA, GpioPin1); 		
		}
		
}
void PortB_IRQHandler(void)
{
		if(TRUE == Gpio_GetIrqStatus(GpioPortB, GpioPin8))
		{
				Gpio_ClearIrq(GpioPortB, GpioPin8); 
		}
}

static void Exit_Interrupt_Init()
{
		stc_gpio_cfg_t stcGpioCfg;
		stcGpioCfg.enDir = GpioDirIn;
		stcGpioCfg.enDrv = GpioDrvL;
		stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
		stcGpioCfg.enOD = GpioOdDisable;
		stcGpioCfg.enCtrlMode = GpioAHB;
		Gpio_Init(GpioPortA, GpioPin10, &stcGpioCfg); 	
		Gpio_Init(GpioPortB, GpioPin8, &stcGpioCfg); 
	
//		Gpio_EnableIrq(GpioPortA, GpioPin10, GpioIrqRising);
		Gpio_EnableIrq(GpioPortA, GpioPin10, GpioIrqFalling);
		Gpio_EnableIrq(GpioPortB, GpioPin8, GpioIrqRising);
//		Gpio_EnableIrq(GpioPortD, GpioPin0, GpioIrqRising);
		Gpio_ClearIrq(GpioPortB, GpioPin8); 
		Gpio_ClearIrq(GpioPortA, GpioPin10); 	
		EnableNvic(PORTA_IRQn, IrqLevel3, TRUE);
		EnableNvic(PORTB_IRQn, IrqLevel3, TRUE);
}


void Rtc_IRQHandler(void)
{
    if(Rtc_GetPridItStatus() == TRUE)
    {
        week_flag = 1;
        Rtc_ClearPrdfItStatus();             
    }
		if (Rtc_GetAlmfItStatus() == TRUE) //????
    {
        week_flag = 1;               //??mian????????LED 10?
        Rtc_ClearAlmfItStatus();       //??????
    }
}


bool is_valid_bcd(unsigned char bcd) {
    return ((bcd & 0x0F) <= 9) && ((bcd >> 4) <= 9);
}


unsigned char decimal_to_bcd(unsigned char decimal) {
    return ((decimal / 10) << 4) | (decimal % 10);
}

int bcd_to_decimal(unsigned char bcd) {
    if (!is_valid_bcd(bcd)) {
        return -1; 
    }
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}
static void App_Rtc_Alarm_init()
{
		stc_rtc_initstruct_t RtcInitStruct;
    stc_rtc_alarmtime_t RtcAlmStruct;
	
		DDL_ZERO_STRUCT(RtcInitStruct);                      //???????
    DDL_ZERO_STRUCT(RtcAlmStruct);
	
		Sysctrl_SetPeripheralGate(SysctrlPeripheralRtc,TRUE);//RTC??????    
    
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);        //????XTL????RTC??
    
    RtcInitStruct.rtcAmpm = RtcPm;                       //12???
    RtcInitStruct.rtcClksrc = RtcClkRcl;                 //??????
    RtcInitStruct.rtcPrdsel.rtcPrdsel = RtcPrds;         //??????PRDS
    RtcInitStruct.rtcPrdsel.rtcPrds = RtcNone;           //???????
    RtcInitStruct.rtcTime.u8Second = 0x55;               //??RTC??2019?4?17?10:01:55
    RtcInitStruct.rtcTime.u8Minute = 0x01;
    RtcInitStruct.rtcTime.u8Hour   = 0x10;
    RtcInitStruct.rtcTime.u8Day    = 0x17;
    RtcInitStruct.rtcTime.u8DayOfWeek = 0x04;
    RtcInitStruct.rtcTime.u8Month  = 0x04;
    RtcInitStruct.rtcTime.u8Year   = 0x19;
    RtcInitStruct.rtcCompen = RtcCompenEnable;           // ????????
    RtcInitStruct.rtcCompValue = 0;                      //???  ??????????
    Rtc_Init(&RtcInitStruct);
    
    RtcAlmStruct.RtcAlarmSec = 0x05;
    RtcAlmStruct.RtcAlarmMinute = 0x02;
    RtcAlmStruct.RtcAlarmHour = 0x10;
    RtcAlmStruct.RtcAlarmWeek = 0x7f;                    //??????,??10:02:05??????    
    Rtc_SetAlarmTime(&RtcAlmStruct);                     //??????    
    Rtc_AlmIeCmd(TRUE);                                  //??????
    
    EnableNvic(RTC_IRQn, IrqLevel3, TRUE);               //??RTC????
    Rtc_Cmd(TRUE);  
		Rtc_StartWait();
}

static void App_rtc_alarm_set(uint8_t sec)
{
		stc_rtc_alarmtime_t RtcAlmStruct;
		stc_rtc_time_t readtime;
		struct tm timeinfo;
		struct tm *tm_info_t;
		time_t timestamp;
	
		Rtc_ReadDateTime(&readtime);
		timeinfo.tm_year = bcd_to_decimal(readtime.u8Year) + 2000 - 1900;
		timeinfo.tm_mon = bcd_to_decimal(readtime.u8Month) - 1;
		timeinfo.tm_mday = bcd_to_decimal(readtime.u8Day);
		timeinfo.tm_hour = bcd_to_decimal(readtime.u8Hour);
		timeinfo.tm_min = bcd_to_decimal(readtime.u8Minute);
		timeinfo.tm_sec = bcd_to_decimal(readtime.u8Second);
		timeinfo.tm_isdst = -1;
		timestamp = mktime(&timeinfo);
		timestamp += sec;
		tm_info_t = localtime(&timestamp);
		
		RtcAlmStruct.RtcAlarmSec = decimal_to_bcd(tm_info_t->tm_sec);
		RtcAlmStruct.RtcAlarmMinute = decimal_to_bcd(tm_info_t->tm_min);
		RtcAlmStruct.RtcAlarmHour = decimal_to_bcd(tm_info_t->tm_hour);		
    RtcAlmStruct.RtcAlarmWeek = 0x7f; 
		Rtc_SetAlarmTime(&RtcAlmStruct);
		Rtc_StartWait();
}

void jbd_rtc_init(void)
{
    stc_rtc_initstruct_t RtcInitStruct;
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralRtc, TRUE); // RTC??????
    RtcInitStruct.rtcAmpm = RtcPm;                         // 24???
    RtcInitStruct.rtcClksrc = RtcClkRcl;                   // ??????
    RtcInitStruct.rtcPrdsel.rtcPrdsel = RtcPrdx;           // ??????PRDX
    RtcInitStruct.rtcPrdsel.rtcPrdx = 7u;                 // ???????? 1? 7 VS 4S  11 VS 6S
    RtcInitStruct.rtcTime.u8Second = 0x00;                 // ??RTC??
    RtcInitStruct.rtcTime.u8Minute = 0x00;
    RtcInitStruct.rtcTime.u8Hour = 0x00;
    RtcInitStruct.rtcTime.u8Day = 0x00;
    RtcInitStruct.rtcTime.u8DayOfWeek = 0x00;
    RtcInitStruct.rtcTime.u8Month = 0x00;
    RtcInitStruct.rtcTime.u8Year = 0x00;
    RtcInitStruct.rtcCompen = RtcCompenEnable; // ????????
    RtcInitStruct.rtcCompValue = 0;            // ???  ??????????
    Rtc_Init(&RtcInitStruct);
    Rtc_AlmIeCmd(false);                    // ??????
    EnableNvic(RTC_IRQn, IrqLevel3, false); // ??RTC????
    Rtc_Cmd(true);                          // ??RTC????
}

void jbd_rtc_control(bool rtc_enable)
{
    if (rtc_enable)
			
    {
        EnableNvic(RTC_IRQn, IrqLevel3, true); // ??RTC??
    }
    else
    {
        EnableNvic(RTC_IRQn, IrqLevel3, false); // ??RTC??
    }
}
void rtc_alarm_test()
{	
		while(1){
				jbd_rtc_control(true);
				Wdt_Feed();
			App_rtc_alarm_set(4);
		//		jbd_sysclk_init(CLK_TYPE_INT);
				M0P_SYSCTRL->SYSCTRL2_f.SYSCTRL2 = 0X5A5A;
				M0P_SYSCTRL->SYSCTRL2_f.SYSCTRL2 = 0XA5A5;
			  M0P_SYSCTRL->SYSCTRL0_f.WAKEUP_BYRCH=1;
				Lpm_GotoDeepSleep(FALSE); 
			  /// add switch to xth code
				jbd_sysclk_init(CLK_TYPE_EXT);
				if(week_flag) {
					printf("wake...\r\n");
					week_flag = 0;
				} else {		
						break;
				}
		}
		printf("----------------");
}


static void App_Rtc_Init(uint8_t sec)
{
    stc_rtc_initstruct_t RtcInitStruct;
    Sysctrl_SetPeripheralGate(SysctrlPeripheralRtc,TRUE);
    RtcInitStruct.rtcAmpm = RtcPm;                       
    RtcInitStruct.rtcClksrc = RtcClkXtl;                 
    RtcInitStruct.rtcPrdsel.rtcPrdsel = RtcPrdx;         
    RtcInitStruct.rtcPrdsel.rtcPrdx = sec;                
    RtcInitStruct.rtcTime.u8Second = 0x55;               
    RtcInitStruct.rtcTime.u8Minute = 0x01;
    RtcInitStruct.rtcTime.u8Hour   = 0x10;
    RtcInitStruct.rtcTime.u8Day    = 0x17;
    RtcInitStruct.rtcTime.u8DayOfWeek = 0x04;
    RtcInitStruct.rtcTime.u8Month  = 0x04;
    RtcInitStruct.rtcTime.u8Year   = 0x19;
    RtcInitStruct.rtcCompen = RtcCompenEnable;           // ????????
    RtcInitStruct.rtcCompValue = 0;                      //???  ??????????
    Rtc_Init(&RtcInitStruct);
    Rtc_AlmIeCmd(TRUE);                                  //??????
    week_flag = 0;
    EnableNvic(RTC_IRQn, IrqLevel3, TRUE);               //??RTC????
    Rtc_Cmd(TRUE); 
		Rtc_StartWait(); 	
}

static void App_Rtc_Deinit()
{
		Rtc_AlmIeCmd(FALSE); 
		EnableNvic(RTC_IRQn, IrqLevel3, FALSE);  
		Rtc_Cmd(FALSE);
}
static void sys_rest()
{
		Gpio_DisableIrq(GpioPortA, GpioPin10, GpioIrqFalling);
		Gpio_DisableIrq(GpioPortB, GpioPin8, GpioIrqRising);
		EnableNvic(PORTA_IRQn, IrqLevel3, FALSE);
		EnableNvic(PORTB_IRQn, IrqLevel3, FALSE);	
		App_Rtc_Deinit();
		Sysctrl_SetPeripheralGate(SysctrlPeripheralCan, TRUE);
		Sysctrl_SetPeripheralGate(SysctrlPeripheralUart0,TRUE);
		Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE);
	
		stc_gpio_cfg_t stcGpioCfg;
		Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
		stcGpioCfg.enDir = GpioDirIn;
		stcGpioCfg.enDrv = GpioDrvL;
		stcGpioCfg.enPu = GpioPuDisable;
		stcGpioCfg.enPd = GpioPdDisable;
		stcGpioCfg.enOD = GpioOdDisable;
		stcGpioCfg.enCtrlMode = GpioAHB;
	
		Gpio_Init(GpioPortB, GpioPin8, &stcGpioCfg);
    stcGpioCfg.enDir = GpioDirOut;
		Gpio_Init(GpioPortB, GpioPin9, &stcGpioCfg);
		Gpio_Init(GpioPortA, GpioPin8, &stcGpioCfg);
	
		Gpio_SetAfMode(GpioPortB, GpioPin8, GpioAf3);
    Gpio_SetAfMode(GpioPortB, GpioPin9, GpioAf5);

    Gpio_ClrIO(GpioPortA, GpioPin8);

	//	DDL_ZERO_STRUCT(stcGpioCfg);
	
		stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPortA, GpioPin9, &stcGpioCfg);
    Gpio_SetAfMode(GpioPortA, GpioPin9, GpioAf1);
	
		stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortA, GpioPin10, &stcGpioCfg);
    Gpio_SetAfMode(GpioPortA, GpioPin10, GpioAf1); 
}

void Sys_Check_Sleep()
{
	if(CHECK_SYS_TIME(WEEK_TIME)) return;
	printf("enter sleep\r\n");
	Sys_Deinit();
	App_Rtc_Alarm_init();
	Exit_Interrupt_Init();
	Wdt_Feed();
	while(1){
				Wdt_Feed();
				App_rtc_alarm_set(4);
				M0P_SYSCTRL->SYSCTRL2_f.SYSCTRL2 = 0X5A5A;
				M0P_SYSCTRL->SYSCTRL2_f.SYSCTRL2 = 0XA5A5;
			  M0P_SYSCTRL->SYSCTRL0_f.WAKEUP_BYRCH=1;
				Lpm_GotoDeepSleep(FALSE); 
				if(week_flag) {
					printf("wake...\r\n");
					week_flag = 0;
				} else {		
						break;
				}
		}	
		App_Rtc_Deinit();
		jbd_sysclk_init(CLK_TYPE_EXT);
		sys_rest();
		printf("sys week\r\n");
		SET_SYS_TIME(WEEK_TIME, 30000);
}

//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI  
__asm void WFI_SET(void)
{
	WFI;		  
}
//关闭所有中断(但是不包括fault和NMI中断)
__asm void INTX_DISABLE(void)
{
	CPSID   I
	BX      LR	  
}
//开启所有中断
__asm void INTX_ENABLE(void)
{
	CPSIE   I
	BX      LR  
}
//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(uint32_t addr) 
{
	MSR MSP, r0 			//set Main Stack value
	BX r14
}
iapfun jump2app; 

void iap_load_app(uint32_t appxaddr)
{
		jump2app=(iapfun)*(volatile uint32_t*)(appxaddr+4);		//用户代码区第二个字为程序开始地址(复位地址)		
		MSR_MSP(*(volatile uint32_t*)appxaddr);					//初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
		for(int i = 0; i < 8; i++)
		{			
			NVIC->ICER[i] = 0xFFFFFFFF;	/* 关闭中断*/
			NVIC->ICPR[i] = 0xFFFFFFFF;	/* 清除中断标志位 */
		}
		jump2app();									//跳转到APP.
}	

void App_AdcPortInit(void)
{    
    ///< ??ADC/BGR GPIO????
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    Gpio_SetAnalogMode(GpioPortA, GpioPin7);        //PA07 (AIN7)
    Gpio_SetAnalogMode(GpioPortB, GpioPin0);        //PB00 (AIN8)
    Gpio_SetAnalogMode(GpioPortB, GpioPin2);        //PB02 (AIN16)
}

void App_AdcInit(void)
{
    stc_adc_cfg_t              stcAdcCfg;

    DDL_ZERO_STRUCT(stcAdcCfg);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE); 
    Bgr_BgrEnable();        ///< ??BGR
    ///< ADC ?????
    stcAdcCfg.enAdcMode         = AdcScanMode;              ///<????-??
    stcAdcCfg.enAdcClkDiv       = AdcMskClkDiv1;            ///<????-1
    stcAdcCfg.enAdcSampCycleSel = AdcMskSampCycle8Clk;      ///<?????-8
    stcAdcCfg.enAdcRefVolSel    = AdcMskRefVolSelInBgr2p5;      ///<??????-VCC
    stcAdcCfg.enAdcOpBuf        = AdcMskBufDisable;         ///<OP BUF??-?
    stcAdcCfg.enInRef           = AdcMskInRefEnable;       ///<????????-?
    stcAdcCfg.enAdcAlign        = AdcAlignRight;               ///<????????-?
    Adc_Init(&stcAdcCfg);
}

void App_AdcSQRCfg(void)
{
    stc_adc_sqr_cfg_t          stcAdcSqrCfg;
    
    DDL_ZERO_STRUCT(stcAdcSqrCfg);
        
    stcAdcSqrCfg.bSqrDmaTrig = FALSE;
    stcAdcSqrCfg.enResultAcc = AdcResultAccDisable;
    stcAdcSqrCfg.u8SqrCnt    = 3;
    Adc_SqrModeCfg(&stcAdcSqrCfg);

    Adc_CfgSqrChannel(AdcSQRCH0MUX, AdcExInputCH7);
    Adc_CfgSqrChannel(AdcSQRCH1MUX, AdcExInputCH8);
    Adc_CfgSqrChannel(AdcSQRCH2MUX, AdcExInputCH16);
    
    Adc_SQR_Start();
}  

void sys_power_gpio_init()
{
		stc_gpio_cfg_t stcGpioCfg;
		Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
	
		stcGpioCfg.enDir = GpioDirOut;
		stcGpioCfg.enDrv = GpioDrvL;
		stcGpioCfg.enPu = GpioPuDisable;
		stcGpioCfg.enPd = GpioPdDisable;
		stcGpioCfg.enOD = GpioOdDisable;
		stcGpioCfg.enCtrlMode = GpioAHB;
	
		Gpio_Init(GpioPortA, GpioPin0, &stcGpioCfg);   /* ????  */
	  
	//	Gpio_SetIO(GpioPortA, GpioPin0);	
		Gpio_ClrIO(GpioPortA, GpioPin0);
		stcGpioCfg.enPu = GpioPuEnable;
		stcGpioCfg.enDir = GpioDirIn;   /*  ??????  */
		Gpio_Init(GpioPortA, GpioPin1, &stcGpioCfg); 
		Gpio_EnableIrq(GpioPortA, GpioPin1, GpioIrqFalling);
		Gpio_ClearIrq(GpioPortA, GpioPin1); 
		EnableNvic(PORTA_IRQn, IrqLevel3, TRUE);
}

void mcu_adc_data_check_get()
{
	volatile uint32_t power48v_adc_val;
	volatile uint32_t bat_adc_val;
	volatile uint32_t bat_temp_adc_val;
	uint8_t data[6] = {0};
	static uint8_t count = 0;
	static uint32_t power48v_adc_sum = 0, bat_adc_sum =0, bat_temp_adc_sum = 0;
	if(mcu_adc_flag && TRUE == Adc_GetIrqStatus(AdcMskIrqSqr)){
		Adc_ClrIrqStatus(AdcMskIrqSqr);
		power48v_adc_val = Adc_GetSqrResult(AdcSQRCH0MUX);
		bat_adc_val = Adc_GetSqrResult(AdcSQRCH1MUX);
		bat_temp_adc_val = Adc_GetSqrResult(AdcSQRCH2MUX);
		printf("power48v_adc_val:%d\r\n", power48v_adc_val);
		printf("bat_adc_val:%d\r\n",bat_adc_val);
		printf("bat_temp_adc_val:%d\r\n",bat_temp_adc_val);
		Adc_SQR_Start();
		power48v_adc_sum += power48v_adc_val;
		bat_adc_sum += bat_adc_val;
		bat_temp_adc_sum += bat_temp_adc_val;
		count++;
		if(count == 3) {
				power48v_adc_sum = power48v_adc_sum/3;
				bat_adc_sum = bat_adc_sum/3;
				bat_temp_adc_sum = bat_temp_adc_sum/3;
				mcu_adc_flag = 0;
				count = 0;
				data[0] = (power48v_adc_sum >> 8)&0xff;
				data[1] = power48v_adc_sum & 0xff;
				data[2] = (bat_adc_sum >> 8)&0xff;
				data[3] = bat_adc_sum&0xff;
				data[4] = (bat_temp_adc_sum>>8)&0xff;
				data[5] = bat_temp_adc_sum&0xff;
				IOT_cmd_data_send(CMD_MCU_ADC_DATA, data, 6);
				power48v_adc_sum = 0;
				bat_adc_sum = 0;
				bat_temp_adc_sum = 0;
		}
	}
}

void hmi_init()
{
		uint16_t lenth;
		uint8_t buf[64] = {0};
		char *dft = "dft test";
		SET_SYS_TIME(HMI_TM, 10);
		while(1){
				if(CHECK_SYS_TIME(HMI_TM) == 0) {
						printf("wait dtf test...\r\n");
						SET_SYS_TIME(HMI_TM, 1000);
						lenth = FIFO_Valid_Size(IOT_UART);
						lenth = MIN(64, lenth);
						if(lenth) {
								FIFO_Rece_Buf(IOT_UART, buf, lenth);
								printf("iot_rec:%s", buf);
								if(strstr((char *)buf, "dft test") != NULL) 
									Uart0_Send_Iot((uint8_t *)dft, strlen(dft));
									break;
						}
				}
		}
}

void Sys_Init()
{
		App_WdtInit(WdtT52s4);
		Wdt_Start();
		App_SysClkInit();
		Sysctrl_ClkSourceEnable(SysctrlClkXTL,TRUE);
		SysTick_Config(SystemCoreClock/1000);
		print_gpio_init();
		UART0_Iot_Init(UART_IOT_BAUD);
		App_Can_init(CAN_BAUD);
		Wdt_Feed();
		App_Timer3Cfg(3000);//32M时2000, 48M时3000
		Tim3_M0_Run();
		if(Flash_Init(12, TRUE) != Ok){
			printf("Flash_Init is fail\r\n");
		}
		hmi_init();
		printf("compile time:%s%s\r\n", __DATE__, __TIME__);
		printf("SOFTVISION:%0x, HWVISION:%0x\r\n", SOFT_VERSION, HW_VERSION);
		printf("SystemCoreClock:%d\r\n", SystemCoreClock);	
		printf("sys init\r\n");
}