#ifndef __DEBUG_UART_H__
#define __DEBUG_UART_H__

#include "main.h"
#include "usart.h"

/*
 * 调试输出的两条路子（排障时很有用）：
 *
 *   0（默认）= 打包成协议里的 TEXT 消息发给地瓜派，由上位机打印。
 *             这是正常使用方式，不会破坏二进制协议流。
 *
 *   1        = 直接以 ASCII 打到 USART1。
 *             好处：随便找个串口助手（115200）就能看到完整启动日志，
 *                   连地瓜派都不用接 —— 怀疑链路本身有问题时就切到这个。
 *             代价：地瓜派收不到有用数据（协议流被文本冲乱），
 *                   只能用来"看 H723 活着没有、卡在哪一步"。
 */
#define DEBUG_RAW_UART   0

void debug_uart_init(void);
void debug_print(const char *msg);
void debug_println(const char *msg);
void debug_print_imu(float yaw, float pitch, float roll, float gx, float gy, float gz, float ax, float ay, float az);

#endif
