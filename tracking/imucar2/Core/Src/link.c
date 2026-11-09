#include "link.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

void Link_Init(void)
{
    /* USART1 的初始化在 MX_USART1_UART_Init() 里完成, 这里不用额外动作 */
}

void Link_SendBytes(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0u)) {
        return;
    }
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)data, len, LINK_TX_TIMEOUT_MS);
}

uint16_t Link_Crc16CcittFalse(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;

    if (data == NULL) {
        return crc;
    }

    while (len-- != 0u) {
        crc ^= (uint16_t)(*data++) << 8;
        for (i = 0u; i < 8u; i++) {
            if ((crc & 0x8000u) != 0u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static void link_store_le16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)(value >> 8);
}

static void link_store_le32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8) & 0xFFu);
    dst[2] = (uint8_t)((value >> 16) & 0xFFu);
    dst[3] = (uint8_t)(value >> 24);
}

void Link_SendChassisFrame(uint16_t seq,
                           uint8_t phase,
                           int8_t turn_dir,
                           uint8_t flags,
                           uint32_t chassis_ms)
{
    uint8_t  frame[LINK_CHASSIS_FRAME_SIZE];
    uint16_t crc;

    frame[0]  = 0xAAu;
    frame[1]  = 0x55u;
    frame[2]  = LINK_CHASSIS_VERSION;
    frame[3]  = LINK_CHASSIS_TYPE;
    link_store_le16(&frame[4], seq);
    frame[6]  = phase;
    frame[7]  = (uint8_t)turn_dir;
    frame[8]  = flags;
    frame[9]  = 0u;
    link_store_le32(&frame[10], chassis_ms);

    crc = Link_Crc16CcittFalse(frame, 14u);
    link_store_le16(&frame[14], crc);
    Link_SendBytes(frame, LINK_CHASSIS_FRAME_SIZE);
}

void Link_Printf(const char *fmt, ...)
{
    char    buf[LINK_TX_BUF_SIZE];
    va_list ap;
    int     n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        return;
    }
    if (n > (int)(sizeof(buf) - 1u)) {
        n = (int)(sizeof(buf) - 1u);
    }
    Link_SendBytes((const uint8_t *)buf, (uint16_t)n);
}

uint8_t Link_FloatToStr(char *out, float value, uint8_t decimals)
{
    uint8_t len = 0u;
    uint8_t i;
    uint32_t scale = 1u;
    uint32_t scaled;
    uint32_t int_part;
    uint32_t frac_part;
    char     tmp[12];
    uint8_t  tn = 0u;

    if (out == NULL) {
        return 0u;
    }
    if (decimals > 4u) {
        decimals = 4u;
    }
    for (i = 0u; i < decimals; i++) {
        scale *= 10u;
    }

    if (value < 0.0f) {
        out[len] = '-';
        len++;
        value = -value;
    }
    if (value > 9999.0f) {
        value = 9999.0f;
    }

    scaled    = (uint32_t)((value * (float)scale) + 0.5f);
    int_part  = scaled / scale;
    frac_part = scaled % scale;

    /* 整数部分 */
    do {
        tmp[tn] = (char)('0' + (int_part % 10u));
        tn++;
        int_part /= 10u;
    } while ((int_part != 0u) && (tn < sizeof(tmp)));
    while (tn > 0u) {
        tn--;
        out[len] = tmp[tn];
        len++;
    }

    /* 小数部分 */
    if (decimals != 0u) {
        out[len] = '.';
        len++;
        for (i = decimals; i > 0u; i--) {
            out[len] = (char)('0' + ((frac_part / (scale / 10u)) % 10u));
            len++;
            scale /= 10u;
        }
    }

    out[len] = '\0';
    return len;
}

/*
 * 裸寄存器版本的串口打印, 给 Error_Handler / HardFault_Handler 用。
 * 不调用 HAL、不开中断, 只要内核还在跑、PA9 还接在串口上就能看到。
 */
static void link_panic_at(uint32_t brr, const char *msg)
{
    uint32_t i;

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;

    /* PA9 -> 复用推挽输出, 50MHz (GPIOA_CRH 的 bit7:4) */
    GPIOA->CRH &= ~(0xFu << 4);
    GPIOA->CRH |=  (0xBu << 4);

    USART1->BRR = (uint16_t)brr;
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;

    for (i = 0u; msg[i] != '\0'; i++) {
        uint32_t t = 0u;
        while (((USART1->SR & USART_SR_TXE) == 0u) && (t < 200000u)) {
            t++;
        }
        USART1->DR = (uint16_t)msg[i];
    }
    i = 0u;
    while (((USART1->SR & USART_SR_TC) == 0u) && (i < 200000u)) {
        i++;
    }
}

void Link_Panic(const char *msg)
{
    if (msg == NULL) {
        return;
    }
    /* 8MHz / 115200 = 69 (时钟没配起来时的情况) */
    link_panic_at(69u, msg);
    /* 72MHz / 115200 = 625 (时钟正常时的情况) */
    link_panic_at(625u, msg);
}
