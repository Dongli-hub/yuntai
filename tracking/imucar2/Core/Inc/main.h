/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* MPU6050 模块接在板上 H3 座(1=5V 2=GND 3=PB14 4=PB15 ...):
      PB14 = SCL, PB15 = SDA   (MPU6050 模块自带 4.7k 上拉)
   本工程用软件 I2C 驱动, 不使用中断。                              */
#define IMU_I2C_SCL_Pin       GPIO_PIN_14
#define IMU_I2C_SCL_GPIO_Port GPIOB
#define IMU_I2C_SDA_Pin       GPIO_PIN_15
#define IMU_I2C_SDA_GPIO_Port GPIOB

/* 电机使能开关 SW4 (BSI-10): 公共端 PC13
 *   一端经 R15 10k 上拉到 3V3, 另一端直接接 GND
 *   -> PC13 为输入, 内部上拉; 拨到哪边决定读高还是读低          */
#define MOTOR_ENABLE_Pin       GPIO_PIN_13
#define MOTOR_ENABLE_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
