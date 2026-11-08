#ifndef __LINE_SENSOR_H__
#define __LINE_SENSOR_H__

#include "main.h"
#include <stdint.h>

/* ==========================================================================
 *  8 路巡线模块 (串口方式) 驱动
 * --------------------------------------------------------------------------
 *  接线:
 *      STM32 PA2 (USART2_TX) ---> 模块 RX
 *      STM32 PA3 (USART2_RX) <--- 模块 TX
 *      5V / GND 与模块共地
 *
 *  模块协议(见 <8路巡线模块的使用方法.pdf>):
 *      主控 -> 模块: "$校准,模拟,数字#"    例如 "$0,0,1#" 表示只要数字量
 *      模块 -> 主控: "$D,x1:0,x2:1,...x8:0#"
 *                    其中 0 = 该探头压在黑线上, 1 = 白底
 *                    "$A,x1:4096,...x8:4096#" 为模拟量(本工程暂不使用)
 *
 *  模块上电后默认不发送数据, 必须先收到 "$0,0,1#" 才会持续输出。
 *
 *  接收方式: USART2 + DMA1_Channel6 循环接收; 每 5ms 在 TIM2 中断里调用
 *            LineSensor_Poll() 把新数据搬进解析缓冲, 按 '$'...'#' 分帧。
 *            上电阶段重复下发命令由主循环调用 LineSensor_Maintenance() 完成
 *            (不要在中断里做阻塞串口发送)。
 * ========================================================================== */

#define LINE_CH_NUM             8

/* ------------------------- 安装方向 / 极性适配 ------------------------- */
/* 1 = 模块左右装反了(第 1 路对应车体右侧), 硬件装反时改 1 */
#define LINE_REVERSE_ORDER      0
/* 1 = 模块上报的 1 表示压黑线(默认为 0 表示压黑线) */
#define LINE_ACTIVE_IS_ONE      0

/* 掉线保护: 超过该时间(ms)没有解析到有效帧就认为传感器掉线 -> 停车 */
#define LINE_ONLINE_TIMEOUT_MS  400u

/* ------------------------------- 接口 --------------------------------- */

/* 初始化: 启动 DMA 接收并下发一次使能命令 */
void LineSensor_Init(void);

/* 5ms 周期调用(放在 TIM2 中断里): 搬运 DMA 数据并解析 */
void LineSensor_Poll(void);

/* 主循环调用: 上电阶段如果还没收到数据, 周期性重发使能命令 */
void LineSensor_Maintenance(void);

/* 已归一化的 8 路状态: bit0..bit7 = 从左到右第 1..8 路, 1 = 压在黑线上 */
uint8_t LineSensor_GetMask(void);

/*
 * 模块报文里的原始值(没做极性/左右翻转), bit0..bit7 = x1..x8。
 * 用来判断极性: 把某个探头压到黑线上, 看这一位是从 1 变 0 还是从 0 变 1。
 */
uint8_t LineSensor_GetRaw(void);

/* 传感器是否在线(最近 LINE_ONLINE_TIMEOUT_MS 内收到过有效帧) */
uint8_t LineSensor_IsOnline(void);

/* 统计信息 / 调试 */
uint16_t LineSensor_GetFrameCount(void);
uint16_t LineSensor_GetBadFrameCount(void);
uint32_t LineSensor_GetLastOkTick(void);

/* 最近一帧原始数据(调试用), 返回实际长度(不含结尾 '\0') */
uint8_t LineSensor_GetLastFrame(char *out, uint8_t max);

/* 数一个字节里 1 的个数(8 位) */
uint8_t LineSensor_PopCount(uint8_t v);

#endif /* __LINE_SENSOR_H__ */
