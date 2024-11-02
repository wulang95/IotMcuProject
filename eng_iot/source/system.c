#include "system.h"
#include "dmac.h"
#include "timer3.h"



static stc_can_rxframe_t       stcRxFrame;
uint32_t sys_time[TIME_MAX];
struct can_rx_frame_s{
		stc_can_rxframe_t Can_RxFrame[CAN_RX_FIFO_SIZE];
		uint8_t len;
		uint8_t in;
		uint8_t out;
		uint8_t num;
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
}
void Can_IRQHandler(void)
{
	  if(TRUE == CAN_IrqFlgGet(CanRxIrqFlg))
    {
        CAN_IrqFlgClr(CanRxIrqFlg);
        CAN_Receive(&stcRxFrame);
				can_rx_frame_in(stcRxFrame);
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

void App_SystemClkInit_PLL48M_byXTH(void)
{
    stc_sysctrl_pll_cfg_t stcPLLCfg;    
    

    Sysctrl_SetXTHFreq(SysctrlXthFreq4_8MHz);
    Sysctrl_XTHDriverCfg(SysctrlXtalDriver1);
    Sysctrl_SetXTHStableTime(SysctrlXthStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTH, TRUE);
    delay1ms(10);
    
    stcPLLCfg.enInFreq    = SysctrlPllInFreq6_12MHz;    //XTH 8MHz
    stcPLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;  //PLL ??
    stcPLLCfg.enPllClkSrc = SysctrlPllXthXtal;          //???????XTH
    stcPLLCfg.enPllMul    = SysctrlPllMul6;             //8MHz x 6 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg); 
    

    Flash_WaitCycle(FlashWaitCycle1);    


    Sysctrl_ClkSourceEnable(SysctrlClkPLL, TRUE);    

    Sysctrl_SysClkSwitch(SysctrlClkPLL);

}


static void App_SysClkInit(void)
{
		Sysctrl_SetHCLKDiv(SysctrlHclkDiv1);
    Sysctrl_SetPCLKDiv(SysctrlPclkDiv1);
		App_SystemClkInit_PLL48M_byXTH();
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
		
		Gpio_ClrIO(GpioPortB, GpioPin14);
		Gpio_SetIO(GpioPortB, GpioPin13);
		Gpio_ClrIO(GpioPortB, GpioPin15);
	
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
				Gpio_SetIO(GpioPortB, GpioPin13);
				SET_SYS_TIME(CAT1_TM, 500);
				state = 1;
		break;
		case 1:
				if(CHECK_SYS_TIME(CAT1_TM) == 0){
						Gpio_ClrIO(GpioPortB, GpioPin13);
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
    stcCanInitCfg.enCanRxBufMode = CanRxBufNotStored;
    stcCanInitCfg.enCanSTBMode   = CanSTBFifoMode;

    CAN_Init(&stcCanInitCfg);
		
    stcFilter.enAcfFormat = CanAllFrames;
    stcFilter.enFilterSel = CanFilterSel1;
		stcFilter.u32CODE     = 0x00000000;
    stcFilter.u32MASK     = 0x1FFFFFFF;
    CAN_FilterConfig(&stcFilter, TRUE);
		CAN_IrqCmd(CanRxIrqEn, TRUE);
		EnableNvic(CAN_IRQn, IrqLevel0, TRUE);
}


uint8_t rx_buff[IOT_BUFF_SIZE];
uint8_t gps_rx_buff[GPS_BUFF_SIZE];
static stc_dma_cfg_t stcDmaCfg;
static void UART0_DMA_Config()
{	
		Sysctrl_SetPeripheralGate(SysctrlPeripheralDma,TRUE); 
		DDL_ZERO_STRUCT(stcDmaCfg);  
		stcDmaCfg.u32DstAddress = 0x40000000;
		stcDmaCfg.enSrcAddrReloadCtl = DmaMskSrcAddrReloadEnable; 
		stcDmaCfg.enSrcBcTcReloadCtl = DmaMskBcTcReloadEnable; 
		stcDmaCfg.enDestAddrReloadCtl = DmaMskDstAddrReloadEnable;
		stcDmaCfg.enTransferMode = DmaMskOneTransfer; 
		stcDmaCfg.enDstAddrMode = DmaMskDstAddrFix; 
		stcDmaCfg.enSrcAddrMode = DmaMskSrcAddrInc;
		stcDmaCfg.u16BlockSize = 1;
		stcDmaCfg.enMode = DmaMskBlock; 
		stcDmaCfg.enTransferWidth = DmaMsk8Bit;
		stcDmaCfg.enRequestNum = DmaUart0TxTrig;
		stcDmaCfg.enPriority = DmaMskPriorityFix; 
		Dma_ClrStat(DmaCh1);
		Dma_EnableChannelIrq(DmaCh1);
		EnableNvic(DMAC_IRQn,IrqLevel3,TRUE);
		Dma_DisableChannel(DmaCh1);
		Dma_Enable(); 
}
static uint8_t data[56];
void UART0_DMA_Send(uint8_t *buf, uint16_t len)
{
		memcpy(data, buf, len);
		stcDmaCfg.u32SrcAddress = (uint32_t)data;
		stcDmaCfg.u16TransferCnt = len; 
		Dma_InitChannel(DmaCh1, &stcDmaCfg); 
		Dma_EnableChannel(DmaCh1);	
		Dma_ClrStat(DmaCh1);
}

void Dmac_IRQHandler(void)
{    
    if(DmaTransferComplete == Dma_GetStat(DmaCh1))
		{
				Dma_DisableChannel(DmaCh1);
		}
    Dma_ClrStat(DmaCh1);
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
		stcCfg.enRunMode        = UartMskMode3;
		stcCfg.enStopBit        = UartMsk1bit;           
    stcCfg.enMmdorCk        = UartMskEven;           
    stcCfg.stcBaud.u32Baud  = baud;                  
    stcCfg.stcBaud.enClkDiv = UartMsk8Or16Div;       
    stcCfg.stcBaud.u32Pclk  = Sysctrl_GetPClkFreq(); 
    Uart_Init(M0P_UART0, &stcCfg);     
		Uart_ClrStatus(M0P_UART0,UartRC);
		Uart_EnableFunc(M0P_UART0,UartDmaTxFunc);
		Uart_EnableIrq(M0P_UART0,UartRxIrq);
		EnableNvic(UART0_2_IRQn, IrqLevel3, TRUE);
		FIFO_Init(IOT_UART, rx_buff, IOT_BUFF_SIZE);
		
		UART0_DMA_Config();
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
		CAN_SetFrame(&stcTxFrame);
		CAN_TransmitCmd(CanPTBTxCmd);
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
		
		DDL_ZERO_STRUCT(stcGpioCfg);
		
		stcGpioCfg.enDir = GpioDirIn;
		Gpio_Init(GpioPortA, GpioPin9, &stcGpioCfg);
		Gpio_Init(GpioPortD, GpioPin1, &stcGpioCfg);
		Gpio_SetIO(GpioPortD, GpioPin5);		
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
	App_Rtc_Init(6);
	Sys_Deinit();
	Exit_Interrupt_Init();
	Wdt_Feed();
		while(1){
				Wdt_Feed();
				Lpm_GotoDeepSleep(FALSE); 
				if(week_flag) {
					printf("wake...\r\n");
					week_flag = 0;
				} else {		
						break;
				}
		}	
		sys_rest();
		printf("sys week\r\n");
		SET_SYS_TIME(WEEK_TIME, 180000);
}
void Sys_Init()
{
		App_WdtInit(WdtT13s1);
		Wdt_Start();
		App_SysClkInit();
		Sysctrl_ClkSourceEnable(SysctrlClkXTL,TRUE);
//		SysTick_Config(SystemCoreClock/1000);
		cat1_init();
		print_gpio_init();
		UART1_GPS_Init(UART_GPS_BAUD);
		UART0_Iot_Init(UART_IOT_BAUD);
		g_cat1_state = CAT1_POWEROFF;
		GPS_init();
		App_Can_init(CAN_BAUD);
		Wdt_Feed();
		App_Timer3Cfg(3000);
		Tim3_M0_Run();
		SET_SYS_TIME(WEEK_TIME, 180000);
		printf("compile time:%s%s\r\n", __DATE__, __TIME__);
		printf("SOFTVISION:%02x%02x, HWVISION:%02x%02x\r\n", SOFT_VERSION_H, SOFT_VERSION_L, HW_VERSION_H, HW_VERSION_L);
		printf("SystemCoreClock:%d\r\n", SystemCoreClock);	
		printf("sys init\r\n");
}