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
volatile int32_t  i32Qutient = 0; 
volatile int32_t  i32Remainder = 0;

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
    stc_div_signed_result_t stcDivResult;
  int16_t Divisor = 0x0000619B;
  int32_t Dividend = -(0x000F4240);;


  DDL_ZERO_STRUCT(stcDivResult);
  
  Sysctrl_SetPeripheralGate(SysctrlPeripheralHdiv,TRUE);
  
  enRc = Hdiv_Signed(Dividend,Divisor,&stcDivResult);
  
  if(Ok == enRc)
  {
    i32Qutient = stcDivResult.Quotient;
    i32Remainder = stcDivResult.Remainder;
  }
  else
  {
    Error_Handle();
  }
  i32Qutient = i32Qutient;
  i32Remainder = i32Remainder;
  
  while(1);
}

/******************************************************************************
* EOF (not truncated)
******************************************************************************/


