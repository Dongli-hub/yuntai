#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__
#include "main.h"
#include "fdcan.h"

/* 一帧 CAN 数据（带 ID），从接收邮箱里取出来用 */
typedef struct
{
    uint32_t id;
    uint8_t  data[8];
    uint8_t  len;
} CanRxFrame_t;

void can_bsp_init(void);
void can_filter_init(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf);
uint8_t fdcanx_receive_frame(FDCAN_HandleTypeDef *hfdcan, uint32_t *id, uint8_t *buf);

/*
 * 接收邮箱：中断里把 FIFO 抽干塞进小队列，主循环里取。
 * 为什么要它：驱动器对每条速度指令都会回一帧（含位置），俯仰位置环就靠它。
 * 在中断里直接解析会把中断拖长，所以这里只搬数据、不解析。
 */
uint8_t can_bsp_get_frame(FDCAN_HandleTypeDef *hfdcan, CanRxFrame_t *out);
void    can_bsp_flush(FDCAN_HandleTypeDef *hfdcan);

void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);

#endif /* __CAN_BSP_H__ */
