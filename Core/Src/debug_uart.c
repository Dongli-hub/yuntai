#include "debug_uart.h"
#include <string.h>

static char g_uart10_buf[256];
static volatile uint8_t g_uart10_tx_busy;

/* USART10 -> 蓝牙调试口：TX=PE03，115200 8N1。
 * 单缓冲中断发送；上一帧未发完时直接丢本帧，不能阻塞 1kHz 控制。 */
void debug_uart10_init(void)
{
    static const char header[] =
        "ms,mode,aim,age,seq,tu,tv,su,sv,ey,ep,wy,wp,ay,ap,"
        "yr,gyro,yiq,pit,ptgt,pcmd,piq,vfps,flt\r\n";
    HAL_NVIC_SetPriority(USART10_IRQn, 6u, 0u);
    HAL_NVIC_EnableIRQ(USART10_IRQn);
    (void)debug_uart10_send(header);
}

uint8_t debug_uart10_send(const char *msg)
{
    size_t length;

    if ((msg == NULL) || (g_uart10_tx_busy != 0u))
    {
        return 0u;
    }
    length = strlen(msg);
    if ((length == 0u) || (length >= sizeof(g_uart10_buf)))
    {
        return 0u;
    }
    memcpy(g_uart10_buf, msg, length);
    g_uart10_tx_busy = 1u;
    if (HAL_UART_Transmit_IT(&huart10, (uint8_t *)g_uart10_buf,
                             (uint16_t)length) != HAL_OK)
    {
        g_uart10_tx_busy = 0u;
        return 0u;
    }
    return 1u;
}

static void debug_uart10_tx_complete(void)
{
    g_uart10_tx_busy = 0u;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart10)
    {
        debug_uart10_tx_complete();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart10)
    {
        g_uart10_tx_busy = 0u;
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_FEF |
                                    UART_CLEAR_NEF | UART_CLEAR_PEF);
    }
}
