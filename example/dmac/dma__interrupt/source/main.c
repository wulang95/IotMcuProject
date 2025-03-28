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
 * @brief  Source file for DMAC example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
* Include files
******************************************************************************/
#include "ddl.h"
#include "dmac.h"
/******************************************************************************
* Local pre-processor symbols/macros ('#define')                            
******************************************************************************/
#define BUFFER_SIZE 32U
/******************************************************************************
* Global variable definitions (declared in header file with 'extern')
******************************************************************************/

/******************************************************************************
* Local type definitions ('typedef')                                         
******************************************************************************/

/******************************************************************************
* Local function prototypes ('static')
******************************************************************************/
/* Private variables ---------------------------------------------------------*/
// DMA传输源地址
static uint32_t aSRC_Const_Buffer[BUFFER_SIZE] =
{
  0x01020304, 0x05060708, 0x090A0B0C, 0x0D0E0F10,
  0x11121314, 0x15161718, 0x191A1B1C, 0x1D1E1F20,
  0x21222324, 0x25262728, 0x292A2B2C, 0x2D2E2F30,
  0x31323334, 0x35363738, 0x393A3B3C, 0x3D3E3F40,
  0x41424344, 0x45464748, 0x494A4B4C, 0x4D4E4F50,
  0x51525354, 0x55565758, 0x595A5B5C, 0x5D5E5F60,
  0x61626364, 0x65666768, 0x696A6B6C, 0x6D6E6F70,
  0x71727374, 0x75767778, 0x797A7B7C, 0x7D7E7F80
};
// DMA传输目标地址
static uint32_t aDST_Buffer[BUFFER_SIZE];

/******************************************************************************
* Local variable definitions ('static')                                      *
******************************************************************************/

/******************************************************************************
* Local pre-processor symbols/macros ('#define')                             
******************************************************************************/

/*****************************************************************************
* Function implementation - global ('extern') and local ('static')
******************************************************************************/
void App_Dma1Cfg(void);
static int32_t myMemcmp(uint8_t *mem1,uint8_t *mem2,uint32_t bytesize);
static void Error_Handle();
/**
******************************************************************************
** \brief  Main function of project
**
** \return uint32_t return value, if needed
**
******************************************************************************/

int32_t main(void)
{     
  // DMA1通道配置
  App_Dma1Cfg();
    
  //软件启动DmaCh1 
  Dma_SwStart(DmaCh1);          

  while(1)
  {
      ;
  }
}

// DMA通道配置，通过软件触发的方式，把aSRC_Const_Buffer数组的值传输到aDST_Buffer数组中
void App_Dma1Cfg(void)
{
    stc_dma_cfg_t stcDmaCfg;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralDma,TRUE);      //打开DMA时钟
  
    DDL_ZERO_STRUCT(stcDmaCfg);                                //结构体变量值清零
    DDL_ZERO_STRUCT(aDST_Buffer);                              //数组值清零
  
    stcDmaCfg.enMode =  DmaMskBlock;                           //选择块传输
    stcDmaCfg.u16BlockSize = 0x01;                             //块传输个数
    stcDmaCfg.u16TransferCnt = BUFFER_SIZE;                    //块传输次数，一次传输数据大小为 块传输个数*BUFFER_SIZE
    stcDmaCfg.enTransferWidth = DmaMsk32Bit;                   //传输数据的宽度，此处选择字(32Bit)宽度
    stcDmaCfg.enSrcAddrMode = DmaMskSrcAddrInc;                //源地址自增
    stcDmaCfg.enDstAddrMode = DmaMskDstAddrInc;                //目的地址自增
    stcDmaCfg.enDestAddrReloadCtl = DmaMskDstAddrReloadDisable;//禁止重新加载传输目的地址
    stcDmaCfg.enSrcAddrReloadCtl = DmaMskSrcAddrReloadDisable; //禁止重新加载传输源地址
    stcDmaCfg.enSrcBcTcReloadCtl = DmaMskBcTcReloadDisable;    //禁止重新加载BC/TC值
    stcDmaCfg.u32SrcAddress = (uint32_t)&aSRC_Const_Buffer[0]; //指定传输源地址
    stcDmaCfg.u32DstAddress = (uint32_t)&aDST_Buffer[0];       //指定传输目的地址
    stcDmaCfg.enRequestNum = DmaSWTrig;                        //设置为软件触发
    stcDmaCfg.enTransferMode = DmaMskOneTransfer;              //dma传输一次，DMAC传输完成时清除CONFA:ENS位
    stcDmaCfg.enPriority = DmaMskPriorityFix;                  //各通道固定优先级，CH0优先级 > CH1优先级
    Dma_InitChannel(DmaCh1,&stcDmaCfg);
  
    Dma_EnableChannelIrq(DmaCh1);             //传输成功完成时产生中断
    EnableNvic(DMAC_IRQn,IrqLevel3,TRUE);     //NVIC对应DMAC中断位使能   
  
    Dma_Enable();                             //使能DMA
    Dma_EnableChannel(DmaCh1);                //使能DmaCh1
}

//内存数据比较
static int32_t myMemcmp(uint8_t *mem1,uint8_t *mem2,uint32_t bytesize)
{
  int i = 0;
  uint8_t *p = mem1;
  uint8_t *q = mem2;
  if(p == NULL|| q == NULL)
    return -1;
  
  for(i = 0;i < bytesize;i++,p++,q++)
  {
    if(*p != *q)
    {
      return -1;
    }
  }
  return 0;
}

// 错误处理函数
static void Error_Handle()
{
  while(1);
}

// DMA中断
void Dmac_IRQHandler(void)
{    
    if(DmaTransferComplete == Dma_GetStat(DmaCh1))
  {
        //等待传输完成
        if(myMemcmp((uint8_t *)&aDST_Buffer[0],(uint8_t *)&aSRC_Const_Buffer[0],BUFFER_SIZE) == -1)
        {
            Error_Handle();
        }
  }
    Dma_ClrStat(DmaCh1);
}
/******************************************************************************
* EOF (not truncated)
******************************************************************************/


