/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "dma.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "line_sensor.h"
#include "imu.h"
#include "track.h"
#include "link.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STARTUP_WAIT_MS   1500u   /* 上电后等模块/IMU 就绪(校准期间小车必须静止) */

/*
 * 1 = 上电先做一次电机死区扫描: 左轮占空比从 0% 慢慢加到 100%, 每 200ms 打一行
 *     "[SWEEP] duty=xx% ENC=nnnn"。你盯着左轮什么时候开始转, 把那一行的 duty
 *     填到 Core/Inc/motor.h 的 MOTOR_DEADZONE_PERCENT 里。
 *     **扫描时轮子会转, 必须把车架空**; 测完把这里改回 0。
 */
#define MOTOR_DEADZONE_SWEEP   0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Chassis_Telemetry(void);
#if MOTOR_DEADZONE_SWEEP
static void Motor_DeadzoneSweep(void);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  Link_Init();

  /*
   * 第一行一定要最早打出来: 只要它出现, 就说明"程序在跑 + 时钟正常 +
   * USART1/PA9 没问题", 剩下所有异常就都是后面某一步的事。
   */
  Link_Printf("\r\n[BOOT] imucar2 line-follow  clk=%lu Hz\r\n",
              (unsigned long)SystemCoreClock);

  /* 电机 + 编码器(先让 PWM 输出为 0, 车不会动) */
  Motor_Init();
  Link_Printf("[1/4] motor + encoder ready\r\n");

#if MOTOR_DEADZONE_SWEEP
  Motor_DeadzoneSweep();
#endif

  /* 8 路巡线模块: 启动 DMA 接收并下发 "$0,0,1#" */
  LineSensor_Init();
  Track_Init();
  Link_Printf("[2/4] line sensor driver ready\r\n");

  /* 板载 IMU: 初始化 + 陀螺仪零偏校准(此时小车必须完全静止) */
  {
      uint8_t imu_ok = IMU_Init();

      if (imu_ok != 0u) {
          Link_Printf("[3/4] imu ok, addr=0x%02X WHO_AM_I=0x%02X %s\r\n",
                      (unsigned int)IMU_GetAddress(),
                      (unsigned int)IMU_GetWhoAmI(),
                      (IMU_IsExpected() != 0u) ? "(MPU6050)" : "(unexpected!)");
          IMU_CalibrateGyro(600u);
          IMU_ResetYaw();
          Link_Printf("[4/4] gyro calibrated, starting 5ms loop\r\n");
      } else {
          uint8_t bus;
          uint8_t found[8];
          uint8_t n;
          uint8_t k;

          Link_Printf("[3/4] imu FAIL: no answer at 0x68/0x69\r\n");

          /* 总线体检: 先看两根线能不能被释放成高电平 */
          bus = IMU_BusCheck();
          Link_Printf("[DIAG] i2c idle SCL=%u SDA=%u (want 1/1)\r\n",
                      (unsigned int)(((bus & 0x01u) != 0u) ? 1u : 0u),
                      (unsigned int)(((bus & 0x02u) != 0u) ? 1u : 0u));

          /* 再全总线扫一遍, 看看到底有没有器件应答 */
          n = IMU_ProbeBus(found, (uint8_t)sizeof(found));
          if (n == 0u) {
              Link_Printf("[DIAG] i2c scan 0x08..0x77: nobody ACKed\r\n");
          } else {
              Link_Printf("[DIAG] i2c scan: %u device(s) ->", (unsigned int)n);
              for (k = 0u; (k < n) && (k < (uint8_t)sizeof(found)); k++) {
                  Link_Printf(" 0x%02X", (unsigned int)found[k]);
              }
              Link_Printf("\r\n");
          }
          Link_Printf("[4/4] gyro NOT calibrated, 5ms loop starts (turn degraded)\r\n");
      }
  }

  /* 让传感器/IMU 再稳一会儿, 然后开始 5ms 控制节拍 */
  HAL_Delay(STARTUP_WAIT_MS);

  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 上电阶段如果模块还没回数据, 周期性重发使能命令 */
    LineSensor_Maintenance();

#if LINK_CHASSIS_TELEMETRY
    {
        static uint32_t tlm_tick = 0u;
        uint32_t now = HAL_GetTick();

        if ((now - tlm_tick) >= LINK_CHASSIS_TELEMETRY_PERIOD_MS) {
            tlm_tick += LINK_CHASSIS_TELEMETRY_PERIOD_MS;
            if ((now - tlm_tick) >= LINK_CHASSIS_TELEMETRY_PERIOD_MS) {
                tlm_tick = now;
            }
            Chassis_Telemetry();
        }
    }
#endif

#if LINK_DEBUG_TEXT_TELEMETRY
    /* 每 2 秒打一次底层寄存器: 用来确认 PWM 是真的发出去了, 以及编码器有没有计数 */
    {
        static uint32_t diag_tick = 0u;
        if ((HAL_GetTick() - diag_tick) >= 2000u) {
            diag_tick = HAL_GetTick();
            Link_Printf("[DIAG] TIM3 CR1=%04X CCER=%04X PSC=%u ARR=%u CCR=%u/%u/%u/%u "
                        "AF=%02X/%02X ENC=%u/%u\r\n",
                        (unsigned int)htim3.Instance->CR1,
                        (unsigned int)htim3.Instance->CCER,
                        (unsigned int)htim3.Instance->PSC,
                        (unsigned int)htim3.Instance->ARR,
                        (unsigned int)htim3.Instance->CCR1,
                        (unsigned int)htim3.Instance->CCR2,
                        (unsigned int)htim3.Instance->CCR3,
                        (unsigned int)htim3.Instance->CCR4,
                        /* PA6/PA7 的配置在 GPIOA_CRL 的 bit31:24 (不是 CRH!) */
                        (unsigned int)((GPIOA->CRL >> 24) & 0xFFu),
                        (unsigned int)(GPIOB->CRL & 0xFFu),
                        (unsigned int)htim4.Instance->CNT,
                        (unsigned int)htim8.Instance->CNT);
            /* 最近一次转弯的实测数据: 跑完接上串口也能读到 */
            Track_PrintLastTurn();
        }
    }
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* 5ms 定时器中断: 巡线数据 -> IMU 角度 -> 循迹状态机 -> 速度环 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        LineSensor_Poll();
        IMU_Update();
        Track_Update();
        Motor_ControlUpdate();
    }
}

#if MOTOR_DEADZONE_SWEEP
/*
 * 电机死区扫描(调试用): 左轮占空比 0 -> 100% 每 200ms 一档, 每档打一行。
 * 轮子刚开始转的那一档, 就是当前电池/摩擦下的实际死区, 填进 MOTOR_DEADZONE_PERCENT。
 * 同时看 ENC 有没有跟着变: 变了说明编码器也是好的。
 */
static void Motor_DeadzoneSweep(void)
{
    uint32_t d;

    Link_Printf("\r\n");
    Link_Printf("[SWEEP] !!! DEADZONE SWEEP - LIFT THE CAR OFF THE GROUND !!!\r\n");
    Link_Printf("[SWEEP] duty 0->100%%, 200ms per step, both wheels\r\n");

    for (d = 0u; d <= 100u; d += 2u) {
        uint16_t l0;
        uint16_t r0;
        uint32_t t0;
        int16_t  dl;
        int16_t  dr;

        Motor_SetDutyRaw(MOTOR_LEFT, (float)d);
        Motor_SetDutyRaw(MOTOR_RIGHT, (float)d);

        l0 = (uint16_t)htim4.Instance->CNT;
        r0 = (uint16_t)htim8.Instance->CNT;

        t0 = HAL_GetTick();
        while ((HAL_GetTick() - t0) < 200u) {
            /* 等轮子把这一档转出来 */
        }

        /* 这一档 200ms 里两个编码器各走了多少格: 0 = 没转起来 */
        dl = (int16_t)((uint16_t)htim4.Instance->CNT - l0);
        dr = (int16_t)((uint16_t)htim8.Instance->CNT - r0);

        Link_Printf("[SWEEP] %3u%%   L=%5d  R=%5d\r\n",
                    (unsigned int)d,
                    (int)dl,
                    (int)dr);
    }
    Motor_SetDutyRaw(MOTOR_LEFT, 0.0f);
    Motor_SetDutyRaw(MOTOR_RIGHT, 0.0f);
    Motor_Stop();
    Link_Printf("[SWEEP] done\r\n");
}
#endif

/* 50Hz 底盘状态帧: 16 字节固定长度, 小端字段, CRC16-CCITT-FALSE。 */
static void Chassis_Telemetry(void)
{
    static uint16_t seq = 0u;
    TrackState state = Track_GetState();
    int8_t     raw_turn_dir = Track_GetTurnDir();
    int8_t     turn_dir = 0;
    uint8_t    phase;
    uint8_t    flags = 0u;
    uint8_t    fault;
    uint8_t    direction_valid;

    switch (state) {
    case TRACK_LINE:
        phase = (Track_IsTurnExitActive() != 0u) ?
                LINK_PHASE_TURN_EXIT : LINK_PHASE_STRAIGHT;
        break;
    case TRACK_PRE_TURN:
        phase = LINK_PHASE_PRE_TURN;
        break;
    case TRACK_TURN:
        phase = LINK_PHASE_TURNING;
        break;
    case TRACK_LOST:
    case TRACK_STOP:
    default:
        phase = LINK_PHASE_FAULT;
        break;
    }

    direction_valid =
        ((phase == LINK_PHASE_PRE_TURN) ||
         (phase == LINK_PHASE_TURNING) ||
         (phase == LINK_PHASE_TURN_EXIT)) &&
        ((raw_turn_dir == 1) || (raw_turn_dir == -1));
    if (direction_valid != 0u) {
        turn_dir = raw_turn_dir;
        flags |= LINK_FLAG_TURN_DIR_VALID;
    }

    fault = (uint8_t)((phase == LINK_PHASE_FAULT) ||
                      (Motor_GetStallFlag() != 0u) ||
                      (IMU_IsOnline() == 0u) ||
                      (((phase == LINK_PHASE_PRE_TURN) ||
                        (phase == LINK_PHASE_TURNING) ||
                        (phase == LINK_PHASE_TURN_EXIT)) &&
                       (direction_valid == 0u)));
    if ((Motor_IsEnabled() != 0u) && (fault == 0u)) {
        flags |= LINK_FLAG_CONTROL_VALID;
    }
    if (state != TRACK_STOP) {
        flags |= LINK_FLAG_AUTO_TRACKING;
    }
    if (fault != 0u) {
        phase = LINK_PHASE_FAULT;
        flags |= LINK_FLAG_FAULT;
    }

    Link_SendChassisFrame(seq, phase, turn_dir, flags, HAL_GetTick());
    seq++;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  /* 先把死因打出来(裸寄存器, 不依赖 HAL): 时钟配不上时会停在这里 */
  Link_Panic("\r\n[PANIC] Error_Handler: clock or peripheral init failed\r\n");
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
