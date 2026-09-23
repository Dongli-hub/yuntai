#include "can_bsp.h"
#include "debug_uart.h"

/**
 * @brief:     CAN总线初始化
 * @param:     void
 * @retval:    void
 */
void can_bsp_init(void)
{
	can_filter_init();

	/* ❗检查返回值。若 HAL_FDCAN_Init 没成功或没进初始化态，
	 *   HAL_FDCAN_Start 会静默失败 —— 这条总线就永远是死的，
	 *   而且跟控制代码毫无关系。 */
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) debug_println("FDCAN1 START FAIL");
	if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) debug_println("FDCAN2 START FAIL");

	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/* ===================== bus-off 自动恢复 =====================
 * 见 can_bsp.h 里的说明。每 50ms 查一次协议状态，
 * 发现 BusOff 就 Stop(置 INIT) + Start(清 INIT) 让它重新上线。
 * 滤波器在 message RAM 里，Stop/Start 不会动它，不用重配。 */
static uint16_t s_busoff_cnt[2];        /* [0]=FDCAN1, [1]=FDCAN2 */

static void can_bus_recover(FDCAN_HandleTypeDef *hfdcan, uint8_t idx)
{
	FDCAN_ProtocolStatusTypeDef st;

	if (HAL_FDCAN_GetProtocolStatus(hfdcan, &st) != HAL_OK) return;
	if (st.BusOff == 0u) return;

	HAL_FDCAN_Stop(hfdcan);
	HAL_FDCAN_Start(hfdcan);

	if (s_busoff_cnt[idx] < 0xFFFFu) s_busoff_cnt[idx]++;
}

void can_bsp_service(void)
{
	static uint32_t t_ms = 0;
	uint32_t now = HAL_GetTick();

	if ((now - t_ms) < 50) return;
	t_ms = now;

	can_bus_recover(&hfdcan1, 0);
	can_bus_recover(&hfdcan2, 1);
}

/* bus-off 发生次数（累计）。辅助判断总线对面到底有没有节点在应答 */
uint16_t can_bsp_get_busoff_count(FDCAN_HandleTypeDef *hfdcan)
{
	return s_busoff_cnt[(hfdcan == &hfdcan1) ? 0 : 1];
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
	/* 范围过滤：必须覆盖驱动器回复帧 ID(0x580+ID) 和发送用的 0x600+ID。
	 * 注意：范围模式是按 [FilterID1, FilterID2] 区间匹配，
	 *       写成 [0x0000,0x0000] 只会接收 ID=0，回复帧会被全部丢弃。 */
	fdcan_filter.FilterID1 = 0x000;                 // 范围起始
	fdcan_filter.FilterID2 = 0x7FF;                 // 范围结束 → 接收全部标准帧

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

/**
 * @brief:     接收CAN数据
 * @param:     hfdcan: FDCAN句柄
 * @param:     buf: 接收数据缓冲区
 * @retval:    接收到的数据长度
 */
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint32_t *id, uint8_t *buf)
{
	FDCAN_RxHeaderTypeDef fdcan_RxHeader;
	if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan_RxHeader, buf) != HAL_OK)
		return 0;
	if(id != NULL) *id = fdcan_RxHeader.Identifier;   /* 保留ID，用于区分回复来源 */
	return (uint8_t)(fdcan_RxHeader.DataLength >> 16);
}

/* ---------- 最近一帧接收报文的无锁缓存 ----------
 * 中断里写、主循环里读，用序号做一致性保护：奇数=写入中，偶数=写入完成 */
static CanRxFrame_t      s_frame[2];          /* [0]=FDCAN1, [1]=FDCAN2 */
static volatile uint32_t s_seq[2] = {0, 0};

static void rx_store(uint8_t idx, uint32_t id, uint8_t len, const uint8_t *buf)
{
	uint8_t i;
	s_seq[idx]++;
	s_frame[idx].id  = id;
	s_frame[idx].len = len;
	for(i = 0; i < 8; i++) s_frame[idx].data[i] = buf[i];
	s_seq[idx]++;
}

uint8_t can_bsp_get_frame(FDCAN_HandleTypeDef *hfdcan, CanRxFrame_t *out)
{
	uint8_t  idx = (hfdcan == &hfdcan1) ? 0 : 1;
	uint32_t s1, s2;
	uint8_t  retry;

	if(s_seq[idx] == 0) return 0;              /* 尚未收到任何报文 */

	for(retry = 0; retry < 4; retry++)
	{
		s1 = s_seq[idx];
		if(s1 & 1u) continue;                  /* 正在写入，重试 */
		*out = s_frame[idx];
		s2 = s_seq[idx];
		if(s1 == s2) return 1;                 /* 读取期间未被改写 */
	}
	return 0;
}

/**
 * @brief:     已接收帧序号（每次成功存帧 +2，可用于判断是否有新帧）
 */
uint32_t can_bsp_get_rx_seq(FDCAN_HandleTypeDef *hfdcan)
{
	return s_seq[(hfdcan == &hfdcan1) ? 0 : 1];
}

/**
 * @brief:     FDCAN1接收回调（偏航驱动器回复）
 */
void fdcan1_rx_callback(void)
{
	uint8_t  buf[8];
	uint32_t id = 0;
	uint8_t  len = fdcanx_receive(&hfdcan1, &id, buf);
	if(len) rx_store(0, id, len, buf);
}

/**
 * @brief:     FDCAN2接收回调（俯仰驱动器回复）
 */
void fdcan2_rx_callback(void)
{
	uint8_t  buf[8];
	uint32_t id = 0;
	uint8_t  len = fdcanx_receive(&hfdcan2, &id, buf);
	if(len)
	{
		rx_store(1, id, len, buf);
		can_pitch_rx_hook(buf, len);   /* 交给上层立即解析，见 can_bsp.h */
	}
}

/* 弱定义：上层（yuntai_task.c）会重写 */
__attribute__((weak)) void can_pitch_rx_hook(const uint8_t *data, uint8_t len)
{
	(void)data;
	(void)len;
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
