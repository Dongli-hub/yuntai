#include "link.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static uint16_t chassis_sequence;

void Link_Init(void)
{
    /* USART1 的初始化在 MX_USART1_UART_Init() 里完成, 这里不用额外动作 */
    chassis_sequence = 0u;
}

void Link_SendBytes(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0u)) {
        return;
    }
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)data, len, LINK_TX_TIMEOUT_MS);
}

void Link_Printf(const char *fmt, ...)
{
#if LINK_DEBUG_TELEMETRY
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
#else
    (void)fmt;
#endif
}

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, refin/refout false. */
static uint16_t link_crc16_ccitt_false(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    uint8_t bit;

    for (i = 0u; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0u; bit < 8u; bit++) {
            crc = (crc & 0x8000u) ?
                  (uint16_t)((crc << 1) ^ 0x1021u) :
                  (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void Link_SendChassisState(uint8_t phase, int8_t turn_dir,
                           uint8_t flags, uint32_t chassis_ms)
{
    uint8_t frame[16];
    uint16_t crc;

    frame[0] = 0xAAu;
    frame[1] = 0x55u;
    frame[2] = 0x01u;                  /* version */
    frame[3] = 0x31u;                  /* chassis state */
    frame[4] = (uint8_t)(chassis_sequence & 0xFFu);
    frame[5] = (uint8_t)(chassis_sequence >> 8);
    frame[6] = phase;
    frame[7] = (uint8_t)turn_dir;
    frame[8] = flags;
    frame[9] = 0u;
    frame[10] = (uint8_t)(chassis_ms & 0xFFu);
    frame[11] = (uint8_t)((chassis_ms >> 8) & 0xFFu);
    frame[12] = (uint8_t)((chassis_ms >> 16) & 0xFFu);
    frame[13] = (uint8_t)((chassis_ms >> 24) & 0xFFu);
    crc = link_crc16_ccitt_false(frame, 14u);
    frame[14] = (uint8_t)(crc & 0xFFu);
    frame[15] = (uint8_t)(crc >> 8);

    chassis_sequence++;
    Link_SendBytes(frame, (uint16_t)sizeof(frame));
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
