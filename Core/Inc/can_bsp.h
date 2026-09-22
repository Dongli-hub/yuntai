#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__
#include "main.h"
#include "fdcan.h"

/* 一帧 CAN 接收报文（保留 ID，用于区分不同驱动器的回复）
 * JC 系列驱动器回复帧 ID = 0x580 + 驱动器ID */
typedef struct
{
	uint32_t id;
	uint8_t  len;
	uint8_t  data[8];
} CanRxFrame_t;

void can_bsp_init(void);
void can_filter_init(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint32_t *id, uint8_t *buf);

/* 读取指定总线最近一帧接收报文；返回 1 表示读到的数据是完整一致的 */
uint8_t can_bsp_get_frame(FDCAN_HandleTypeDef *hfdcan, CanRxFrame_t *out);

/* 已接收帧序号（每次成功存帧 +2，可用于判断是否有新帧） */
uint32_t can_bsp_get_rx_seq(FDCAN_HandleTypeDef *hfdcan);

void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);

/* FDCAN2(俯仰) 接收钩子：在 CAN 中断里直接调用，由上层实现。
 * 为什么要放到中断里：轮询方式下“读到的不一定是对的帧”，
 * 一旦判据不成立位置环就会静默退化成开环，而且很难察觉。
 * can_bsp.c 里有一个空的弱定义，上层重写即可。 */
void can_pitch_rx_hook(const uint8_t *data, uint8_t len);

#endif /* __CAN_BSP_H__ */
