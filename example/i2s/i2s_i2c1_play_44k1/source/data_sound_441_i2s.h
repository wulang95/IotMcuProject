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
 * @file   data_sound_441_i2s.h
 *
 * @brief  Header file for I2S example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

#ifndef __DATA_SOUND_I2S_H__
#define __DATA_SOUND_I2S_H__

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "hc32l07x.h"
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************************/
/* Global pre-processor symbols/macros ('define')                            */
/*****************************************************************************/

/******************************************************************************
 * Global definitions
 ******************************************************************************/

  
/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/
extern const uint16_t au16PixieDustSoundI2s_441[];
extern uint32_t u32WavLen_44k;

  
/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* __DATA_SOUND_I2S_H__ */
/*****************************************************************************/
/* EOF (not truncated)                                                       */
/*****************************************************************************/
