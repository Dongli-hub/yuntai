/**
 * 语法检查用的"前置头"（用 gcc -include 强制先包含）。
 *
 * 关键技巧：先把真实 main.h / usart.h / fdcan.h / spi.h 的 include guard
 * 定义掉，这样它们后面被 #include 时就成了空文件，
 * 全部 HAL 类型改由本文件的桩提供 —— 于是不需要 arm-none-eabi-gcc，
 * 也不需要真的把整个 HAL 拖进来，就能对业务代码做语法/类型检查。
 *
 * 本文件只在 PC 上被使用，绝不参与 STM32 固件编译。
 */
#ifndef __HOST_SYNTAX_STUB_PRE_H__
#define __HOST_SYNTAX_STUB_PRE_H__

/* 1) 屏蔽真实硬件头 */
#define __MAIN_H
#define __USART_H__
#define __FDCAN_H__
#define __SPI_H__

/* 2) 基本类型 */
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#define __packed __attribute__((packed))
#define __weak   __attribute__((weak))

/* CMSIS 里的 RESET/SET 是普通宏，业务代码里会直接用到 */
#ifndef RESET
#define RESET 0u
#endif
#ifndef SET
#define SET   1u
#endif

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;

typedef struct { uint32_t ISR; uint32_t RDR; uint32_t ICR; uint32_t dummy[4]; } USART_TypeDef;
typedef struct { uint32_t dummy[2]; } FDCAN_GlobalTypeDef;
typedef struct { uint32_t dummy[2]; } GPIO_TypeDef;
typedef struct { uint32_t dummy[2]; } SPI_TypeDef;

typedef struct { USART_TypeDef *Instance; uint32_t dummy[10]; } UART_HandleTypeDef;
typedef struct { SPI_TypeDef   *Instance; uint32_t dummy[6];  } SPI_HandleTypeDef;
typedef struct { FDCAN_GlobalTypeDef *Instance; uint32_t dummy[10]; } FDCAN_HandleTypeDef;

typedef struct
{
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
    uint32_t Alternate;
} GPIO_InitTypeDef;

typedef struct { uint32_t Identifier; uint32_t DataLength; uint32_t dummy[4]; } FDCAN_RxHeaderTypeDef;
typedef struct
{
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t TxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
    uint32_t TxEventFifoControl;
    uint32_t MessageMarker;
} FDCAN_TxHeaderTypeDef;
typedef struct
{
    uint32_t IdType;
    uint32_t FilterIndex;
    uint32_t FilterType;
    uint32_t FilterConfig;
    uint32_t FilterID1;
    uint32_t FilterID2;
} FDCAN_FilterTypeDef;

/* 3) GPIO / 中断 */
#define GPIO_PIN_0    0x0001u
#define GPIO_PIN_3    0x0008u
#define GPIO_PIN_4    0x0010u
#define GPIO_PIN_10   0x0400u
#define GPIO_PIN_12   0x1000u
#define GPIO_PIN_RESET 0u
#define GPIO_PIN_SET   1u
#define GPIO_MODE_OUTPUT_PP 1u
#define GPIO_NOPULL 0u
#define GPIO_SPEED_FREQ_LOW 0u
#define GPIOA ((GPIO_TypeDef *)0x1000)
#define GPIOC ((GPIO_TypeDef *)0x2000)
#define USART1_IRQn 37

/* 4) 用到的 HAL API（只声明） */
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, const uint8_t *d, uint16_t n);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *h);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t fifo,
                                         FDCAN_RxHeaderTypeDef *hdr, uint8_t *data);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h,
                                                FDCAN_TxHeaderTypeDef *hdr,
                                                uint8_t *data);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *h,
                                                 uint32_t its, uint32_t mask);
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *h,
                                         FDCAN_FilterTypeDef *f);
HAL_StatusTypeDef HAL_FDCAN_ConfigFifoWatermark(FDCAN_HandleTypeDef *h,
                                                uint32_t fifo, uint32_t wm);
void Error_Handler(void);
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *g);
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, uint32_t state);
void HAL_NVIC_SetPriority(int32_t irq, uint32_t pre, uint32_t sub);
void HAL_NVIC_EnableIRQ(int32_t irq);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h);

/* 5) 用到的 HAL 宏 */
#define __HAL_RCC_GPIOC_CLK_ENABLE()  do { } while (0)
#define __HAL_UART_ENABLE_IT(h, it)   do { (void)(h); } while (0)
#define __HAL_UART_CLEAR_FLAG(h, f)   do { (void)(h); } while (0)
#define __HAL_UART_GET_FLAG(h, f)     0u
#define UART_IT_RXNE          0u
#define UART_CLEAR_OREF       0u
#define UART_CLEAR_FEF        0u
#define UART_CLEAR_NEF        0u
#define UART_CLEAR_PEF        0u
#define USART_ISR_RXNE_RXFNE  0u
#define USART_ISR_ORE         0u
#define USART_ISR_FE          0u
#define USART_ISR_NE          0u
#define USART_ISR_PE          0u
#define FDCAN_RX_FIFO0        0u
#define FDCAN_STANDARD_ID     0u
#define FDCAN_DATA_FRAME      0u
#define FDCAN_ESI_ACTIVE      0u
#define FDCAN_BRS_OFF         0u
#define FDCAN_CLASSIC_CAN     0u
#define FDCAN_NO_TX_EVENTS    0u
#define FDCAN_FILTER_RANGE    0u
#define FDCAN_FILTER_TO_RXFIFO0 0u
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 0u
#define FDCAN_CFG_RX_FIFO0    0u

/* 6) 真实头文件里被屏蔽掉、但业务代码要用的 extern */
extern UART_HandleTypeDef  huart1;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern SPI_HandleTypeDef   hspi2;

#endif /* __HOST_SYNTAX_STUB_PRE_H__ */
