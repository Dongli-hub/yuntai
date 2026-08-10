#include "debug_uart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static char g_tx_buf[256];

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

void debug_print(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
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
        HAL_UART_Transmit(&huart1, (uint8_t *)g_tx_buf, len, 100);
    }
}
