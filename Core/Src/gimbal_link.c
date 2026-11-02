/**
  ******************************************************************************
  * @file    gimbal_link.c
  * @brief   K230 -> H723 单向 ASCII 视觉链路
  *
  * K230 每帧发送：
  *   cx,cy,lx,ly,target_vx,target_vy,image_age
  *
  * 丢失帧：
  *   404,404,0,0,0,0,0
  *
  * 像素坐标在 H723 侧统一换算为角度，K230 不再重复计算角度。
  ******************************************************************************
  */

#include "gimbal_link.h"
#include "usart.h"
#include <string.h>

/* 当前硬件最终方案：K230 的数据从 USART1 RX(PA10) 输入。 */
#define GL_RX_HUART          huart1
#define GL_RX_IRQn           USART1_IRQn
#define GL_WATCHDOG_MS       250u
#define GL_TEXT_BUF_SIZE     128u
#define GL_RX_DRAIN_MAX      256u
#define GL_RX_PARSE_MAX      256u

/* 唯一视觉标定：真机 7.8 px/deg。 */
#define K230_ANGLE_PER_PX    0.128205128f

#ifndef GL_RX_BUF_SIZE
#define GL_RX_BUF_SIZE       512u
#endif

static uint8_t           s_rx_buf[GL_RX_BUF_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;

static GimbalCmd_t       s_cmd;
static uint32_t          s_last_aim_ms;
static uint8_t           s_alive;
static uint8_t           s_zero_req;
static char              s_text_line[GL_TEXT_BUF_SIZE];
static uint8_t           s_text_len;
static GimbalLinkStats_t s_stats;

static int32_t gl_parse_x100(const char **cursor, uint8_t *ok)
{
    const char *p = *cursor;
    int32_t sign = 1;
    int32_t whole = 0;
    int32_t frac = 0;
    uint8_t digits = 0u;
    uint8_t frac_digits = 0u;

    while (*p == ' ')
    {
        p++;
    }
    if (*p == '-')
    {
        sign = -1;
        p++;
    }
    else if (*p == '+')
    {
        p++;
    }
    while ((*p >= '0') && (*p <= '9'))
    {
        if (digits < 6u)
        {
            whole = whole * 10 + (int32_t)(*p - '0');
            digits++;
        }
        p++;
    }
    if (*p == '.')
    {
        p++;
        while ((*p >= '0') && (*p <= '9'))
        {
            if (frac_digits < 2u)
            {
                frac = frac * 10 + (int32_t)(*p - '0');
                frac_digits++;
            }
            p++;
        }
        if (frac_digits == 1u)
        {
            frac *= 10;
        }
    }
    *cursor = p;
    *ok = (uint8_t)(digits > 0u);
    return sign * (whole * 100 + frac);
}

static uint8_t gl_handle_text_line(uint32_t now_ms)
{
    const char *p = s_text_line;
    int32_t values[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t value_count = 0u;
    uint8_t offset;
    uint8_t velocity_offset;
    uint8_t ok;
    uint8_t lost;
    uint8_t was_aim;

    while (value_count < 10u)
    {
        values[value_count] = gl_parse_x100(&p, &ok);
        if (!ok)
        {
            return 0u;
        }
        value_count++;
        if (*p != ',')
        {
            break;
        }
        p++;
    }
    if (*p != '\0')
    {
        return 0u;
    }

    /*
     * 新 K230：7 列原始像素。
     * 旧 K230：2 列角度 + 8 列原始像素（其中包含已废弃的距离字段）。
     * 两版都在这一个解析器里转换为统一的控制量，避免分板烧录时断链。
     */
    if (value_count == 7u)
    {
        offset = 0u;
        velocity_offset = 4u;
    }
    else if (value_count == 9u)
    {
        offset = 2u;
        velocity_offset = 4u;
    }
    else if (value_count == 10u)
    {
        offset = 2u;
        velocity_offset = 5u;
    }
    else
    {
        return 0u;
    }

    lost = (uint8_t)(
        (values[offset] == 40400) && (values[offset + 1u] == 40400)
    );
    if (!lost)
    {
        if ((values[offset] < 0) || (values[offset] > 100000) ||
            (values[offset + 1u] < 0) ||
            (values[offset + 1u] > 100000) ||
            (values[offset + 2u] < 0) ||
            (values[offset + 2u] > 100000) ||
            (values[offset + 3u] < 0) ||
            (values[offset + 3u] > 100000))
        {
            return 0u;
        }
    }

    was_aim = (uint8_t)(s_cmd.mode == GP_MODE_AIM);
    s_cmd.mode = lost ? GP_MODE_STAB : GP_MODE_AIM;
    s_cmd.vision_seq++;
    s_cmd.px_ms = now_ms;
    s_cmd.rx_count++;
    s_cmd.aim_valid = (uint8_t)(lost ? 0u : 1u);

    s_cmd.px_ut = lost ? 0u :
        (uint16_t)(values[offset] / 100);
    s_cmd.px_vt = lost ? 0u :
        (uint16_t)(values[offset + 1u] / 100);
    s_cmd.px_us = lost ? 0u :
        (uint16_t)(values[offset + 2u] / 100);
    s_cmd.px_vs = lost ? 0u :
        (uint16_t)(values[offset + 3u] / 100);

    if (lost)
    {
        s_cmd.vis_yaw_deg = 0.0f;
        s_cmd.vis_pitch_deg = 0.0f;
        s_cmd.target_yaw_rate_dps = 0.0f;
        s_cmd.target_pitch_rate_dps = 0.0f;
        s_cmd.target_velocity_valid = 0u;
        s_cmd.vis_proc_age_ms = 0u;
    }
    else
    {
        int32_t dx_x100 =
            values[offset] - values[offset + 2u];
        int32_t dy_x100 =
            values[offset + 1u] - values[offset + 3u];

        s_cmd.vis_yaw_deg =
            ((float)dx_x100 / 100.0f) * K230_ANGLE_PER_PX;
        /* K230 相机 pitch 方向与 H723 控制方向相反。 */
        s_cmd.vis_pitch_deg =
            -((float)dy_x100 / 100.0f) * K230_ANGLE_PER_PX;

        s_cmd.target_yaw_rate_dps =
            ((float)values[offset + velocity_offset] / 100.0f) *
            K230_ANGLE_PER_PX;
        s_cmd.target_pitch_rate_dps =
            ((float)values[offset + velocity_offset + 1u] / 100.0f) *
            K230_ANGLE_PER_PX;
        s_cmd.target_yaw_rate_dps = (s_cmd.target_yaw_rate_dps > 640.0f) ?
                                    640.0f : s_cmd.target_yaw_rate_dps;
        s_cmd.target_yaw_rate_dps = (s_cmd.target_yaw_rate_dps < -640.0f) ?
                                    -640.0f : s_cmd.target_yaw_rate_dps;
        s_cmd.target_pitch_rate_dps =
            (s_cmd.target_pitch_rate_dps > 640.0f) ?
            640.0f : s_cmd.target_pitch_rate_dps;
        s_cmd.target_pitch_rate_dps =
            (s_cmd.target_pitch_rate_dps < -640.0f) ?
            -640.0f : s_cmd.target_pitch_rate_dps;
        s_cmd.target_velocity_valid = 1u;
        s_cmd.vis_proc_age_ms =
            (uint16_t)(
                ((values[value_count - 1u] >= 0) &&
                 (values[value_count - 1u] <= 1000)) ?
                (values[value_count - 1u] * 10) : 0
            );
    }

    s_last_aim_ms = now_ms;
    if (lost && was_aim)
    {
        s_zero_req = 1u;
    }
    s_alive = 1u;
    return 1u;
}

static void gl_text_feed_byte(uint8_t byte, uint32_t now_ms)
{
    if (byte == '\r')
    {
        return;
    }
    if (byte == '\n')
    {
        if (s_text_len > 0u)
        {
            s_text_line[s_text_len] = '\0';
            s_stats.lines++;
            if (gl_handle_text_line(now_ms) == 0u)
            {
                s_stats.rejected_lines++;
            }
        }
        s_text_len = 0u;
        return;
    }
    if ((byte < 32u) || (byte > 126u))
    {
        s_text_len = 0u;
        return;
    }
    if (s_text_len < (GL_TEXT_BUF_SIZE - 1u))
    {
        s_text_line[s_text_len++] = (char)byte;
    }
    else
    {
        s_text_len = 0u;
    }
}

static void gl_rx_push(uint8_t byte)
{
    uint16_t next = (uint16_t)(s_rx_head + 1u);
    if (next >= GL_RX_BUF_SIZE)
    {
        next = 0u;
    }
    if (next != s_rx_tail)
    {
        s_rx_buf[s_rx_head] = byte;
        s_rx_head = next;
        s_stats.rx_bytes++;
    }
}

void gimbal_link_init(void)
{
    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_last_aim_ms = 0u;
    s_alive = 0u;
    s_zero_req = 0u;
    s_text_len = 0u;
    memset(&s_stats, 0, sizeof(s_stats));
    memset(&s_cmd, 0, sizeof(s_cmd));
    s_cmd.mode = GP_MODE_STAB;

    __HAL_UART_CLEAR_FLAG(&GL_RX_HUART, UART_CLEAR_OREF | UART_CLEAR_FEF |
                                     UART_CLEAR_NEF | UART_CLEAR_PEF);
    __HAL_UART_ENABLE_IT(&GL_RX_HUART, UART_IT_RXNE);
    HAL_NVIC_SetPriority(GL_RX_IRQn, 6u, 0u);
    HAL_NVIC_EnableIRQ(GL_RX_IRQn);
}

void gimbal_link_rx_isr(void)
{
    uint32_t isr = GL_RX_HUART.Instance->ISR;

    if ((isr & USART_ISR_RXNE_RXFNE) != 0u)
    {
        gl_rx_push((uint8_t)(GL_RX_HUART.Instance->RDR & 0xFFu));
    }
    if ((isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE |
                USART_ISR_PE)) != 0u)
    {
        __HAL_UART_CLEAR_FLAG(&GL_RX_HUART, UART_CLEAR_OREF |
                              UART_CLEAR_FEF | UART_CLEAR_NEF |
                              UART_CLEAR_PEF);
    }
}

void gimbal_link_poll(uint32_t now_ms)
{
    uint16_t drain_count = 0u;
    uint16_t parse_count = 0u;

    /* RXNE 中断与轮询兜底并存，避免现场漏中断后整条链路永久静止。 */
    __HAL_UART_DISABLE_IT(&GL_RX_HUART, UART_IT_RXNE);
    while ((__HAL_UART_GET_FLAG(&GL_RX_HUART, UART_FLAG_RXNE) != 0u) &&
           (drain_count < GL_RX_DRAIN_MAX))
    {
        gl_rx_push((uint8_t)(GL_RX_HUART.Instance->RDR & 0xFFu));
        drain_count++;
    }
    __HAL_UART_CLEAR_FLAG(&GL_RX_HUART, UART_CLEAR_OREF |
                                      UART_CLEAR_FEF |
                                      UART_CLEAR_NEF |
                                      UART_CLEAR_PEF);
    __HAL_UART_ENABLE_IT(&GL_RX_HUART, UART_IT_RXNE);

    while ((s_rx_tail != s_rx_head) && (parse_count < GL_RX_PARSE_MAX))
    {
        uint8_t byte = s_rx_buf[s_rx_tail];
        parse_count++;
        s_rx_tail++;
        if (s_rx_tail >= GL_RX_BUF_SIZE)
        {
            s_rx_tail = 0u;
        }
        gl_text_feed_byte(byte, now_ms);
    }

    if ((s_alive != 0u) && ((now_ms - s_last_aim_ms) > GL_WATCHDOG_MS))
    {
        s_alive = 0u;
        s_cmd.mode = GP_MODE_STAB;
        s_cmd.aim_valid = 0u;
        s_zero_req = 1u;
    }
}

const GimbalCmd_t *gimbal_link_cmd(void)
{
    return &s_cmd;
}

GimbalLinkStats_t gimbal_link_stats(void)
{
    return s_stats;
}

uint8_t gimbal_link_take_zero_req(void)
{
    uint8_t request = s_zero_req;
    s_zero_req = 0u;
    return request;
}
