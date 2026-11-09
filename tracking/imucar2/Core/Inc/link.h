#ifndef __LINK_H__
#define __LINK_H__

#include "main.h"
#include <stdint.h>

/* ==========================================================================
 *  通信口 (USART1: PA9 = TX, PA10 = RX)
 * --------------------------------------------------------------------------
 *  周期遥测使用 16 字节固定二进制帧, 50 Hz 发送给 H723。
 *  启动阶段仍保留少量文本诊断; 接收端按 AA 55 帧头重同步即可。
 * ========================================================================== */

#define LINK_BAUDRATE            115200u

/* 1 = 周期性发送底盘二进制状态帧, 0 = 关闭 */
#define LINK_CHASSIS_TELEMETRY          1
#define LINK_CHASSIS_TELEMETRY_PERIOD_MS 20u

/* 1 = 保留 2 秒一次的文本寄存器诊断, 正常联调时关闭 */
#define LINK_DEBUG_TEXT_TELEMETRY       0

#define LINK_CHASSIS_FRAME_SIZE   16u
#define LINK_CHASSIS_VERSION      1u
#define LINK_CHASSIS_TYPE         0x31u

/* phase 字段 */
#define LINK_PHASE_STRAIGHT       0u
#define LINK_PHASE_PRE_TURN       1u
#define LINK_PHASE_TURNING        2u
#define LINK_PHASE_TURN_EXIT      3u
#define LINK_PHASE_FAULT          4u

/* flags 字段 */
#define LINK_FLAG_CONTROL_VALID   0x01u
#define LINK_FLAG_TURN_DIR_VALID  0x02u
#define LINK_FLAG_AUTO_TRACKING   0x04u
#define LINK_FLAG_FAULT           0x08u

#define LINK_TX_BUF_SIZE         320u
#define LINK_TX_TIMEOUT_MS       20u

void Link_Init(void);

/* 直接发送一段字节 */
void Link_SendBytes(const uint8_t *data, uint16_t len);

/* CRC16-CCITT-FALSE: init=0xFFFF, poly=0x1021, refin=false, refout=false */
uint16_t Link_Crc16CcittFalse(const uint8_t *data, uint16_t len);

/* 组装并发送一帧 16 字节底盘状态 */
void Link_SendChassisFrame(uint16_t seq,
                           uint8_t phase,
                           int8_t turn_dir,
                           uint8_t flags,
                           uint32_t chassis_ms);

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
