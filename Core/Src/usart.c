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
/* UART7 初始化是否出错（1 = 出错）。故意不死等 Error_Handler：
 * 现场如果卡死在 Error_Handler，整块板子一点反应都没有，太难查；
 * 这里只记标志，由链路层打一条 TEXT 日志出来。 */
static uint8_t s_uart7_err;

uint8_t UART7_InitError(void)
{
  return s_uart7_err;
}

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
    s_uart7_err = 1u;
    return;
  }
  /* FIFO 这三步做成"尽力而为"：万一某个型号不支持，也不要卡死整机 */
  (void)HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8);
  (void)HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8);
  (void)HAL_UARTEx_DisableFifoMode(&huart7);
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

#include <stdio.h>

/* ===========================================================================
 * 串口自检（由 main.c 的 UART7_TX_TEST_LOOP 开关调用）
 *
 * 目的：把"云台程序"完全排除在外，只验证串口本身通不通。
 *   同时从两个口发一模一样的内容，用来 A/B 对比是哪一侧的问题：
 *       UART7  TX = PE08      （板上 UART7 排针那根）
 *       USART1 TX = PA09      （原来接地瓜派的那根，应该也能接到）
 *
 * 发的内容（不可能看错）：
 *   开机先发 64 个 'U'（16 进制就是 55 55 55 ...），
 *   然后每 200ms 发一行 ASCII，带递增计数。
 *
 * 接法：USB-TTL 的 RX 接上面任意一个 TX 脚，**GND 必须共地**，
 *      串口助手 115200 / 8 / N / 1。测的时候把 K230 的数据线拔掉。
 * ========================================================================= */
static void uart_tx_both(const char *s, uint16_t n)
{
    HAL_UART_Transmit(&huart7, (uint8_t *)s, n, 50u);
    HAL_UART_Transmit(&huart1, (uint8_t *)s, n, 50u);
}

void UART7_TxTest(void)
{
    uint32_t n = 0u;
    uint32_t t;
    char     line[96];
    static const char burst[] =
        "UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU"
        "UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU";   /* 64 个 0x55 */
    static const char head[] =
        "\r\n"
        "==================================================\r\n"
        " TX SELF TEST     115200 8N1\r\n"
        " sending on BOTH:  UART7 TX=PE08   USART1 TX=PA09\r\n"
        " USB-TTL RX -> one of those TX pins, GND common\r\n"
        " EVERY 200ms: 128 x 'U' (HEX 55) + one count line\r\n"
        "==================================================\r\n";

    /* 按板上丝印固定引脚：RX=PE7, TX=PE8 */
    UART7_BindPins(UART7_PAIR_PE);

    uart_tx_both(head, (uint16_t)(sizeof(head) - 1u));

    while (1)
    {
        /* 每轮都连发两串 0x55（共 128 个）：
           ASCII 视图是 UUUU...，HEX 视图是 55 55 55 ...。
           持续发送的好处是：在串口助手里反复换波特率，
           哪个波特率能读出一整串 U，就说明实际波特率是那个。 */
        uart_tx_both(burst, (uint16_t)(sizeof(burst) - 1u));
        uart_tx_both(burst, (uint16_t)(sizeof(burst) - 1u));

        int len = snprintf(line, sizeof(line),
                           "SELFTEST count=%lu   U7=PE08  U1=PA09\r\n",
                           (unsigned long)n);
        if (len > 0)
        {
            uart_tx_both(line, (uint16_t)len);
        }

        /* 回显 UART7 收到的字节（十六进制），方便看对面有没有在发 */
        while (__HAL_UART_GET_FLAG(&huart7, UART_FLAG_RXNE) != 0u)
        {
            uint8_t b = (uint8_t)(huart7.Instance->RDR & 0xFFu);
            int m = snprintf(line, sizeof(line), "[U7 RX] %02X ", b);
            if (m > 0)
            {
                uart_tx_both(line, (uint16_t)m);
            }
        }

        n++;
        t = HAL_GetTick();
        while ((HAL_GetTick() - t) < 200u)
        {
            /* 空转 200ms */
        }
    }
}

/* USER CODE END 1 */

