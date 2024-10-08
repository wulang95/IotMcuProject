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
 * @brief  Source file for HDIV example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
* Include files
******************************************************************************/
#include "ddl.h"
#include "hdiv.h"
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
volatile uint32_t  u32Qutient = 0; 
volatile uint32_t  u32Remainder = 0;
  
/******************************************************************************
* Local variable definitions ('static')                                      *
******************************************************************************/

static void Error_Handle()
{
  while(1);
}

/******************************************************************************
* Local pre-processor symbols/macros ('#define')                             
******************************************************************************/

/*****************************************************************************
* Function implementation - global ('extern') and local ('static')
******************************************************************************/
/**
******************************************************************************
** \brief  Main function of project
**
** \return uint32_t return value, if needed
**
******************************************************************************/

int32_t main(void)
{    
  en_result_t enRc = Ok;
  uint32_t Dividend = 0x000F4240;
  uint16_t Divisor = 0x0000619B;
  stc_div_unsigned_result_t stcDivResult;
  
  DDL_ZERO_STRUCT(stcDivResult);
  
  Sysctrl_SetPeripheralGate(SysctrlPeripheralHdiv,TRUE);
  
  enRc = Hdiv_Unsigned(Dividend,Divisor,&stcDivResult);
  
  if(Ok == enRc)
  {
    u32Qutient = stcDivResult.Quotient;
    u32Remainder = stcDivResult.Remainder;
  }
  else
  {
    Error_Handle();
  }
  u32Qutient = u32Qutient;
  u32Remainder = u32Remainder;
  
  while(1);
}

/******************************************************************************
* EOF (not truncated)
******************************************************************************/


