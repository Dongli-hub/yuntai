#ifndef __LINK_H__
#define __LINK_H__

#include "main.h"
#include <stdint.h>

/* ==========================================================================
 *  通信口 (USART1: PA9 = TX, PA10 = RX)
 * --------------------------------------------------------------------------
 *  这个口今后用来和 H723 主控通信。当前阶段把它当作调试 / 遥测口:
 *  默认每 LINK_TELEMETRY_PERIOD_MS 打一行文本, 便于看状态、调参数。
 *
 *  > 以后接上 H723 之后, 把 LINK_DEBUG_TELEMETRY 改成 0 关掉文本输出,
 *    再把二进制协议写在 Link_SendBytes() / Link_Poll() 里即可。
 * ========================================================================== */

#define LINK_BAUDRATE            115200u

/* H723底盘状态帧: 50Hz固定16字节二进制. */
#define LINK_BINARY_TELEMETRY    0
#define LINK_BINARY_PERIOD_MS    20u

/* 1 = 周期性输出CSV文本遥测(蓝牙调试用), 0 = 关闭.
 * 默认关闭, 避免和H723二进制协议混在同一串口. */
#define LINK_DEBUG_TELEMETRY     1
#define LINK_TELEMETRY_PERIOD_MS 50u

#define LINK_TX_BUF_SIZE         320u
#define LINK_TX_TIMEOUT_MS       20u

/* Chassis state protocol, type 0x31. */
#define LINK_PHASE_STRAIGHT      0u
#define LINK_PHASE_PRE_TURN      1u
#define LINK_PHASE_TURNING       2u
#define LINK_PHASE_TURN_EXIT     3u
#define LINK_PHASE_FAULT         4u

#define LINK_FLAG_CONTROL_VALID  (1u << 0)
#define LINK_FLAG_DIR_VALID      (1u << 1)
#define LINK_FLAG_AUTO_TRACKING  (1u << 2)
#define LINK_FLAG_FAULT          (1u << 3)

void Link_Init(void);

/* 直接发送一段字节 */
void Link_SendBytes(const uint8_t *data, uint16_t len);

/* 发送底盘状态帧 type=0x31, version=1. */
void Link_SendChassisState(uint8_t phase, int8_t turn_dir,
                           uint8_t flags, uint32_t chassis_ms);

/* 格式化输出(只用整数和字符串格式, 不依赖 printf 浮点支持) */
void Link_Printf(const char *fmt, ...);

/*
 * 把浮点转成字符串(避免 nano.specs 下 %f 不生效的问题)。
 * 例如 12.34 -> "12.34", -0.5 -> "-0.50"。返回写入长度。
 */
uint8_t Link_FloatToStr(char *out, float value, uint8_t decimals);

/*
 * 极端情况下的"遗言": 不依赖 HAL、不依赖中断, 直接用寄存器把 PA9 当
 * USART1_TX 打一行字。用于 Error_Handler / HardFault_Handler 里报死因。
 * 因为那时候主频可能是 8MHz(时钟没配上)也可能是 72MHz, 所以两种分频
 * 各打一遍, 哪遍能读懂就说明主频是哪一个。
 */
void Link_Panic(const char *msg);

#endif /* __LINK_H__ */
