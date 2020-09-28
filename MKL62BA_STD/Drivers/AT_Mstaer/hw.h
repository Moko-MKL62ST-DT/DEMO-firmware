/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: contains all hardware driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
/**
  ******************************************************************************
  * @file    hw.h
  * @author  MCD Application Team
  * @brief   contains all hardware driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_H__
#define __HW_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "usart.h"
#include "tiny_sscanf.h"

#include <stdarg.h>
#include "tiny_vsnprintf.h"
#include "gpio.h"
#include "SEGGER_RTT.h"
	
#define LOG_LUO(...) 	SEGGER_RTT_printf(0,__VA_ARGS__);//fprintf(stdout, ">>>>>" format "<<<<", ##__VA_ARGS__)  
	
typedef enum
{
  HW_UNLOCKED = 0x00U,
  HW_LOCKED   = 0x01U
} HW_LockTypeDef;

#define HW_LOCK(__HANDLE__)               \
  do {                                    \
    if ((__HANDLE__)->Lock == HW_LOCKED)  \
    {                                     \
      return;                             \
    }                                     \
    else                                  \
    {                                     \
      (__HANDLE__)->Lock = HW_LOCKED;     \
    }                                     \
  } while (0)

#define HW_UNLOCK(__HANDLE__)             \
  do {                                    \
    (__HANDLE__)->Lock = HW_UNLOCKED;     \
  } while (0)

#ifdef __cplusplus
}
#endif

#endif /* __HW_H__ */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

