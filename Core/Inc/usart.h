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
 * UART7 引脚对（CubeMX 芯片库：STM32H723VGT6 / LQFP100 上 UART7 只有这三组）：
 *   UART7_PAIR_PE  : PE7 = RX, PE8  = TX   ← 板上丝印写的就是这一组
 *   UART7_PAIR_PB3 : PB3 = RX, PA15 = TX   ← 候选（怕丝印/走线不一致时扫出来）
 *   UART7_PAIR_PA8 : PA8 = RX, PB4  = TX   ← 候选
 * 自检模式会轮流切换这三组、每组发不同字符（U/V/W），
 * 用串口助手看排针上出现的是哪个字符，就知道它实际连到哪组脚。
 * ------------------------------------------------------------------------- */
#define UART7_PAIR_PE   0u
#define UART7_PAIR_PB3  1u
#define UART7_PAIR_PA8  2u

void UART7_BindPins(uint8_t pair);
uint8_t UART7_CurrentPins(void);
uint8_t UART7_InitError(void);

/* UART7 单独自检：上电后只从 UART7 发 ASCII 并回显收到的字节。
 * 由 main.c 里的 UART7_TX_TEST_LOOP 开关决定要不要进这个模式。 */
void UART7_TxTest(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

