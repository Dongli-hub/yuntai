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

/**
 * @brief:     接收CAN数据
 * @param:     hfdcan: FDCAN句柄
 * @param:     buf: 接收数据缓冲区
 * @retval:    接收到的数据长度
 */
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
{
	FDCAN_RxHeaderTypeDef fdcan_RxHeader;
	if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan_RxHeader, buf) != HAL_OK)
		return 0;
	return fdcan_RxHeader.DataLength >> 16;
}

/* 接收数据缓冲区 */
uint8_t rx_data1[8] = {0};
uint8_t rx_data2[8] = {0};

/**
 * @brief:     FDCAN1接收回调
 */
void fdcan1_rx_callback(void)
{
	fdcanx_receive(&hfdcan1, rx_data1);
}

/**
 * @brief:     FDCAN2接收回调
 */
void fdcan2_rx_callback(void)
{
	fdcanx_receive(&hfdcan2, rx_data2);
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
