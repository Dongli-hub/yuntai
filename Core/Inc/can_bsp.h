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

/* 周期维护：检测到 bus-off 就自动恢复。
 * ❗为什么必须有：总线上一旦没有节点应答我们的帧（驱动器没上电/波特率不对/
 *   接线断/CAN ID 不匹配），发送错误计数 TEC 每帧 +8，约 32 帧后就进入
 *   **bus-off**。进入 bus-off 后硬件【彻底不再发送】，而且自己不会出来 ——
 *   必须手动清 CCCR.INIT 才能重新上线。
 *   以 5ms 一帧算，不到 200ms 就 bus-off 且永不自救 ⟹ 之后无论固件怎么改、
 *   就算把驱动器修好了，这条总线也永远是死的。
 *   这同时也解释了历史上"上电偶尔完全失去控制"。                     */
void can_bsp_service(void);

/* bus-off 累计发生次数。可用来判断总线对面到底有没有节点在应答：
 * 一直为 0 = 有节点应答（正常）；不停增长 = 对面没东西/波特率不对/接线断 */
uint16_t can_bsp_get_busoff_count(FDCAN_HandleTypeDef *hfdcan);
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
