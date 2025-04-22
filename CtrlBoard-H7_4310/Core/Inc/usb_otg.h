/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usb_otg.h
  * @brief   This file contains all the function prototypes for
  *          the usb_otg.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_OTG_H__
#define __USB_OTG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USB_OTG_HS_PCD_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USB_OTG_H__ */

