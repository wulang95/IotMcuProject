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
 * @file   lpm.h
 *
 * @brief  Header file for LPM functions
 *
 * @author MADS Team 
 *
 ******************************************************************************/

#ifndef __LPM_H__
#define __LPM_H__

/*****************************************************************************
 * Include files
 *****************************************************************************/
#include "ddl.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *******************************************************************************
 ** \defgroup LpmGroup Low Power Management (LPM)
 **
 **
 ******************************************************************************/
//@{
    
/******************************************************************************
 ** Global pre-processor symbols/macros ('#define')                            
 ******************************************************************************/

/******************************************************************************
 * Global type definitions
 ******************************************************************************/

/******************************************************************************
 * Global variable declarations ('extern', definition in C source)
 *****************************************************************************/

/******************************************************************************
 * Global function prototypes (definition in C source)
 *****************************************************************************/
///<功能配置及操作函数
///<进入普通休眠模式
void Lpm_GotoSleep(boolean_t bOnExit);
///<进入深度休眠模式
void Lpm_GotoDeepSleep(boolean_t bOnExit);

//@} // LpmGroup

#ifdef __cplusplus
#endif


#endif /* __LPM_H__ */
/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/

