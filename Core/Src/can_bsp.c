#include "can_bsp.h"

/**
 * @brief:     CAN总线初始化
 * @param:     void
 * @retval:    void
 */
void can_bsp_init(void)
{
	can_filter_init();
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/**
 * @brief:     CAN过滤器初始化（接收所有ID）
 * @param:     void
 * @retval:    void
 */
void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;

	fdcan_filter.IdType = FDCAN_STANDARD_ID;
	fdcan_filter.FilterIndex = 0;
	fdcan_filter.FilterType = FDCAN_FILTER_RANGE;
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	fdcan_filter.FilterID1 = 0x0000;
	fdcan_filter.FilterID2 = 0x0000;

	if(HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);

	if(HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
}

/**
 * @brief:     发送CAN数据
 * @param:     hfdcan: FDCAN句柄
 * @param:     id: CAN设备ID
 * @param:     data: 发送的数据
 * @param:     len: 发送的数据长度
 * @retval:    0=成功, 1=失败
 */
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.Identifier = id;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = len;             // HAL内部已<<16, 此处不重复移位
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0x00;

	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) != HAL_OK)
		return 1;
	return 0;
}

/*
 * DLC 码值 -> 实际字节数
 * 经典 CAN 里 DLC 0~8 就是字节数；CAN-FD 里 9~15 对应 12/16/20/24/32/48/64。
 * ⚠ 必须过这张表：HAL_FDCAN_GetRxMessage() 填进 RxHeader.DataLength 的是
 *   【DLC 码值】(0~15)，不是"字节数<<16"。
 *   老代码写成 `DataLength >> 16` 得到恒为 0 —— 因为那个返回值没人用，
 *   所以这个 bug 一直藏着；一旦拿它当长度用（比如解析电机反馈），
 *   就会表现为"编码器反馈一条都收不到"。
 */
static const uint8_t s_dlc_to_len[16] = {
	0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 12u, 16u, 20u, 24u, 32u, 48u, 64u
};

/**
 * @brief:     接收CAN数据
 * @param:     hfdcan: FDCAN句柄
 * @param:     buf: 接收数据缓冲区
 * @retval:    接收到的数据长度（字节），0 表示没有数据
 */
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
{
	FDCAN_RxHeaderTypeDef fdcan_RxHeader;
	if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan_RxHeader, buf) != HAL_OK)
		return 0;
	return s_dlc_to_len[fdcan_RxHeader.DataLength & 0x0Fu];
}

/**
 * @brief:     接收CAN数据（同时给出 ID）
 * @retval:    接收到的数据长度，0 表示没有数据
 */
uint8_t fdcanx_receive_frame(FDCAN_HandleTypeDef *hfdcan, uint32_t *id, uint8_t *buf)
{
	FDCAN_RxHeaderTypeDef fdcan_RxHeader;
	if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan_RxHeader, buf) != HAL_OK)
		return 0;
	if(id != 0)
	{
		*id = fdcan_RxHeader.Identifier;
	}
	return s_dlc_to_len[fdcan_RxHeader.DataLength & 0x0Fu];
}

/* 接收数据缓冲区 */
uint8_t rx_data1[8] = {0};
uint8_t rx_data2[8] = {0};

/* ---------------- 接收邮箱（每路总线 4 深） ---------------- */
#define CAN_RX_Q_LEN 4u

static volatile CanRxFrame_t s_q1[CAN_RX_Q_LEN];
static volatile CanRxFrame_t s_q2[CAN_RX_Q_LEN];
static volatile uint8_t s_q1_head = 0u, s_q1_tail = 0u, s_q1_ovf = 0u;
static volatile uint8_t s_q2_head = 0u, s_q2_tail = 0u, s_q2_ovf = 0u;

static void can_q_push(volatile CanRxFrame_t *q, volatile uint8_t *head,
                       volatile uint8_t *tail, volatile uint8_t *ovf,
                       uint32_t id, const uint8_t *data, uint8_t len)
{
	uint8_t next = (uint8_t)((*head + 1u) % CAN_RX_Q_LEN);
	uint8_t i;
	if(next == *tail)
	{
		(*ovf)++;                     /* 队列满：丢新帧，保证已经排队的顺序不乱 */
		return;
	}
	q[*head].id  = id;
	q[*head].len = (len > 8u) ? 8u : len;
	for(i = 0u; i < 8u; i++)
	{
		q[*head].data[i] = (i < len) ? data[i] : 0u;
	}
	*head = next;                     /* 最后才动 head，消费者不会读到半帧 */
}

/**
 * @brief:     取一帧（非阻塞）
 * @retval:    1 = 取到，0 = 队列空
 */
uint8_t can_bsp_get_frame(FDCAN_HandleTypeDef *hfdcan, CanRxFrame_t *out)
{
	volatile CanRxFrame_t *q;
	volatile uint8_t *head;
	volatile uint8_t *tail;
	uint8_t i;

	if(hfdcan == &hfdcan1)      { q = s_q1; head = &s_q1_head; tail = &s_q1_tail; }
	else if(hfdcan == &hfdcan2) { q = s_q2; head = &s_q2_head; tail = &s_q2_tail; }
	else return 0;

	if(*tail == *head) return 0;
	out->id  = q[*tail].id;
	out->len = q[*tail].len;
	for(i = 0u; i < 8u; i++) out->data[i] = q[*tail].data[i];
	*tail = (uint8_t)((*tail + 1u) % CAN_RX_Q_LEN);
	return 1;
}

void can_bsp_flush(FDCAN_HandleTypeDef *hfdcan)
{
	if(hfdcan == &hfdcan1)      { s_q1_tail = s_q1_head; }
	else if(hfdcan == &hfdcan2) { s_q2_tail = s_q2_head; }
}

/**
 * @brief:     FDCAN1接收回调
 *            必须把硬件 FIFO 抽干：只取一帧的话，剩下的会一直卡在 FIFO 里，
 *            要等下一帧才再触发中断 —— 等于凭空丢帧。
 */
void fdcan1_rx_callback(void)
{
	uint32_t id;
	uint8_t  n;
	while((n = fdcanx_receive_frame(&hfdcan1, &id, rx_data1)) > 0u)
	{
		can_q_push(s_q1, &s_q1_head, &s_q1_tail, &s_q1_ovf, id, rx_data1, n);
	}
}

/**
 * @brief:     FDCAN2接收回调
 */
void fdcan2_rx_callback(void)
{
	uint32_t id;
	uint8_t  n;
	while((n = fdcanx_receive_frame(&hfdcan2, &id, rx_data2)) > 0u)
	{
		can_q_push(s_q2, &s_q2_head, &s_q2_tail, &s_q2_ovf, id, rx_data2, n);
	}
}

/**
 * @brief:     HAL库FDCAN中断回调函数
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		if(hfdcan == &hfdcan1)
		{
			fdcan1_rx_callback();
		}
		if(hfdcan == &hfdcan2)
		{
			fdcan2_rx_callback();
		}
	}
}
