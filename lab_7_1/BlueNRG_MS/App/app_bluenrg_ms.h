/**
  ******************************************************************************
  * @file    app_bluenrg_ms.h
  * @author  SRA Application Team
  * @brief   Header file for app_bluenrg_ms.c
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_BLUENRG_MS_H
#define APP_BLUENRG_MS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_types.h"

/* Exported Functions --------------------------------------------------------*/
void MX_BlueNRG_MS_Init(void);
void MX_BlueNRG_MS_Process(void);
void BLE_SetFilteredData(float x, float y, float z);

#ifdef __cplusplus
}
#endif
#endif /* APP_BLUENRG_MS_H */

