/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart7;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_UART7_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* ---------------------------------------------------------------------------
 * UART7 引脚对（查过 CubeMX 芯片库：STM32H723VGT6 / LQFP100 上
 * UART7 只有这几种引脚组合，PF6/PF7 在这颗片子上根本不存在）：
 *   UART7_PAIR_PE : PE7 = UART7_RX, PE8 = UART7_TX   ← 板上丝印就是这一组，当前使用
 *   UART7_PAIR_PA : PA8 = UART7_RX, PB3 = UART7_TX   ← 备用（换板子才可能用上）
 * ------------------------------------------------------------------------- */
#define UART7_PAIR_PE   0u
#define UART7_PAIR_PA   1u

void UART7_BindPins(uint8_t pair);
uint8_t UART7_CurrentPins(void);
uint8_t UART7_InitError(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

