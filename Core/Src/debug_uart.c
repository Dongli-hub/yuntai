#include "debug_uart.h"
#include "gimbal_link.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static char g_tx_buf[256];
static char g_uart10_buf[256];
static volatile uint8_t g_uart10_tx_busy;

static void ftoa(char *buf, float val, int decimals)
{
    int sign = (val < 0) ? 1 : 0;
    if (sign) val = -val;
    int ipart = (int)val;
    float fpart = val - (float)ipart;
    int mult = 1;
    for (int i = 0; i < decimals; i++) mult *= 10;
    int fint = (int)(fpart * mult + 0.5f);
    if (fint >= mult) { ipart++; fint -= mult; }
    int pos = 0;
    if (sign) buf[pos++] = '-';
    pos += snprintf(buf + pos, 20, "%d", ipart);
    if (decimals > 0) {
        buf[pos++] = '.';
        char fmt[8];
        snprintf(fmt, sizeof(fmt), "%%0%dd", decimals);
        pos += snprintf(buf + pos, 20, fmt, fint);
    }
    buf[pos] = '\0';
}

void debug_uart_init(void) {}

/* USART10 专用蓝牙调试口：TX=PE03，115200 8N1。
 * 使用单缓冲中断发送；上一帧未发完时直接丢本帧，不能阻塞 1kHz 控制。 */
void debug_uart10_init(void)
{
    /* 精简调参表：原始靶心/激光像素 + 视觉 PID + 云台响应。
     * 链路计数、启动阶段和复位诊断仍由协议 TEXT 日志按需输出。 */
    static const char header[] =
        "ms,mode,aim,src,age,seq,tu,tv,su,sv,dist,ey,ep,wy,wp,ay,ap,"
        "iyy,iyp,yr,gyro,ycmd,yiq,ybase,pit,ptgt,pcmd,piq,vfps,flt\r\n";
    HAL_NVIC_SetPriority(USART10_IRQn, 6u, 0u);
    HAL_NVIC_EnableIRQ(USART10_IRQn);
    (void)debug_uart10_send(header);
}

uint8_t debug_uart10_send(const char *msg)
{
    size_t n;

    if ((msg == NULL) || (g_uart10_tx_busy != 0u))
    {
        return 0u;
    }
    n = strlen(msg);
    if ((n == 0u) || (n >= sizeof(g_uart10_buf)))
    {
        return 0u;
    }
    memcpy(g_uart10_buf, msg, n);
    g_uart10_tx_busy = 1u;
    if (HAL_UART_Transmit_IT(&huart10, (uint8_t *)g_uart10_buf,
                             (uint16_t)n) != HAL_OK)
    {
        g_uart10_tx_busy = 0u;
        return 0u;
    }
    return 1u;
}

void debug_uart10_tx_complete(void)
{
    g_uart10_tx_busy = 0u;
}

/*
 * 调试输出改走协议里的 TEXT 消息（0x93），由上位机打印。
 *
 * 为什么必须改：UART7 现在是二进制协议流，直接 printf 会把帧冲乱，
 * 现象是上位机 crc_err 暴涨、时好时坏 —— 很难查。
 * 包成 TEXT 帧以后，"每一步都看得见"的调试习惯完全保留。
 */
void debug_print(const char *msg)
{
#if DEBUG_RAW_UART
    /* 排障模式：直接打 ASCII 到串口，用串口助手就能看 */
    HAL_UART_Transmit(&huart7, (uint8_t *)msg, strlen(msg), 100);
#else
    gimbal_link_log(msg);
#endif
}

void debug_println(const char *msg)
{
    debug_print(msg);
    debug_print("\r\n");
}

void debug_print_imu(float yaw, float pitch, float roll, float gx, float gy, float gz, float ax, float ay, float az)
{
    char y[12], p[12], r[12], gxs[12], gys[12], gzs[12], axs[12], ays[12], azs[12];
    ftoa(y, yaw * 57.29578f, 2);
    ftoa(p, pitch * 57.29578f, 2);
    ftoa(r, roll * 57.29578f, 2);
    ftoa(gxs, gx, 2);
    ftoa(gys, gy, 2);
    ftoa(gzs, gz, 2);
    ftoa(axs, ax, 3);
    ftoa(ays, ay, 3);
    ftoa(azs, az, 3);
    int len = snprintf(g_tx_buf, sizeof(g_tx_buf),
        "Y:%s P:%s R:%s | G:%s %s %s | A:%s %s %s\r\n",
        y, p, r, gxs, gys, gzs, axs, ays, azs);
    if (len > 0) {
        gimbal_link_log(g_tx_buf);
    }
}
