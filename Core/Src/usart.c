/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart7;

/* ---------------------------------------------------------------------------
 * UART7 引脚对管理（K230 链路用）
 *
 * 为什么需要这个：板子上 UART7 接插件只标了 "UART7"，
 * 而 STM32H723VGT6(LQFP100) 上 UART7 有两组可用引脚（都是 AF11）：
 *     UART7_PAIR_PE : PE7 = RX, PE8 = TX   （CubeMX 默认，最可能）
 *     UART7_PAIR_PA : PA8 = RX, PB3 = TX   （备用；
 *         PF6/PF7 组合在这颗 LQFP100 上不存在，别再试了）
 * 不可能靠猜，索性两个都试：gimbal_link_init() 里每个脚等 400ms，
 * 哪个脚收到字节就用哪个；都没收到就回默认 PE7/PE8。
 *
 * 说明：这里只切 GPIO 的复用功能，不动 UART7 外设本身，
 * 所以切换时不会丢配置，也不会触发 HAL 重初始化。
 * RX 脚一律开内部上拉：悬空脚最怕被噪声拉出假的起始位。
 * ------------------------------------------------------------------------- */
static uint8_t s_uart7_pair = UART7_PAIR_PE;

void UART7_BindPins(uint8_t pair)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  s_uart7_pair = pair;

  /* 先把两组脚都还原成模拟态，避免两个输出同时驱动 */
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_7 | GPIO_PIN_8);
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3);

  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_UART7;

  if (pair == UART7_PAIR_PA)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_8;                   /* PA8 = UART7_RX */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_3;                   /* PB3 = UART7_TX */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  else
  {
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8;      /* PE7=RX, PE8=TX */
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
}

uint8_t UART7_CurrentPins(void)
{
  return s_uart7_pair;
}

/* UART7 init function */

void MX_UART7_UART_Init(void)
{
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16910CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==UART7)
  {
  /* USER CODE BEGIN UART7_MspInit 0 */

  /* USER CODE END UART7_MspInit 0 */
  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART7;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* UART7 clock enable */
    __HAL_RCC_UART7_CLK_ENABLE();

    /* 引脚对由 s_uart7_pair 决定（PE7/PE8 或 PA8/PB3） */
    UART7_BindPins(s_uart7_pair);

  /* USER CODE BEGIN UART7_MspInit 1 */

  /* USER CODE END UART7_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==UART7)
  {
  /* USER CODE BEGIN UART7_MspDeInit 0 */

  /* USER CODE END UART7_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART7_CLK_DISABLE();

    /**UART7 GPIO Configuration
    PE7/PE8 或 PA8/PB3 ------> UART7
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_7|GPIO_PIN_8);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3);

  /* USER CODE BEGIN UART7_MspDeInit 1 */

  /* USER CODE END UART7_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

