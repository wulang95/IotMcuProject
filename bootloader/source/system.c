#include "gpio.h"
#include "sysctrl.h"
#include "wdt.h"
#include "flash.h"
#include "system.h"


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
		INTX_ENABLE();
		jump2app=(iapfun)*(volatile uint32_t*)(appxaddr+4);		//用户代码区第二个字为程序开始地址(复位地址)		
		MSR_MSP(*(volatile uint32_t*)appxaddr);					//初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
		for(int i = 0; i < 8; i++)
		{			
			NVIC->ICER[i] = 0xFFFFFFFF;	/* 关闭中断*/
			NVIC->ICPR[i] = 0xFFFFFFFF;	/* 清除中断标志位 */
		}
		jump2app();									//跳转到APP.
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

static void App_WdtInit(en_wdt_time_t W_Tm)
{
    Sysctrl_SetPeripheralGate(SysctrlPeripheralWdt,TRUE);
    Wdt_Init(WdtResetEn, W_Tm);
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

void Sys_Init()
{
		App_WdtInit(WdtT13s1);
		Wdt_Start();
		App_SysClkInit();
		Sysctrl_ClkSourceEnable(SysctrlClkXTL,TRUE);
		SysTick_Config(SystemCoreClock/1000);
		print_gpio_init();
		Wdt_Feed();
		if(Flash_Init(12, TRUE) != Ok){
				printf("Flash_Init is fail\r\n");
		}
		printf("compile time:%s%s\r\n", __DATE__, __TIME__);
		printf("SystemCoreClock:%d\r\n", SystemCoreClock);	
		printf("app adr:%08x\r\n", APP_ADR);
		printf("back_app adr:%08x\r\n", BACK_APP_ADR);
		printf("sys init\r\n");		
}