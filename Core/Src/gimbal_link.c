/**
  ******************************************************************************
  * @file    gimbal_link.c
  * @brief   H723(USART1) <-> 机载计算机链路实现
  *
  * 三个关键工程决定：
  *  1) 收：只用 RXNE 中断 + 软件环形缓冲，不碰 DMA、不改 .ioc。
  *     115200 下最高 11.5k 中断/s，对 550MHz 的 H723 完全不算负担，
  *     而且中断里只做"存一个字节"这一件事，绝不解析。
  *  2) 发：用 HAL_UART_Transmit_IT + 发送环形缓冲，绝不阻塞主循环。
  *     若用阻塞发送，一帧 28 字节要 2.4ms，会把 2ms 的 IMU 节拍拖歪。
  *  3) 看门狗：0.5s 收不到 AIM 就自动切 STAB + 关激光。
  *     这是"上位机崩了/线掉了"时的唯一保护，必须在 H723 侧做。
  ******************************************************************************
  */

#include "gimbal_link.h"
#include "gimbal_proto.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

/* ========================== 可调参数 ========================== */
#define GL_TELEM_PERIOD_MS   20u     /* 遥测周期：20ms = 50Hz */
#define GL_WATCHDOG_MS       500u    /* AIM 断流多久后自动降级到 STAB */
#define GL_LOG_PERIOD_MS     50u     /* 调试文本最快 20 条/s */
#define GL_TX_CHUNK          32u     /* 单次 IT 发送的字节数上限 */
#define GL_OFF_YAW_LIMIT     180.0f  /* 偏置安全限幅（防止上位机给飞了） */
#define GL_OFF_PITCH_LIMIT   80.0f

/* ========================== 缓冲区 ========================== */
#ifndef GL_RX_BUF_SIZE
#define GL_RX_BUF_SIZE       512u
#endif
#define GL_TX_BUF_SIZE       512u

static uint8_t           s_rx_buf[GL_RX_BUF_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile uint32_t s_rx_ovf;

static uint8_t           s_tx_buf[GL_TX_BUF_SIZE];
static volatile uint16_t s_tx_head;
static volatile uint16_t s_tx_tail;
static volatile uint16_t s_tx_busy_len;
static uint32_t          s_tx_busy_ms;
static volatile uint32_t s_tx_drop;

static gp_parser_t       s_parser;
static GimbalCmd_t       s_cmd;
static GimbalTelem_t     s_telem;

static uint32_t          s_last_aim_ms;
static uint32_t          s_last_telem_ms;
static uint32_t          s_last_log_ms;
static uint8_t           s_alive;
static uint8_t           s_zero_req;
static uint8_t           s_have_telem;
static uint8_t           s_seq;
static uint8_t           s_ready;
static uint32_t          s_log_dropped;

/* ========================== TX 环形缓冲 ========================== */

static uint16_t gl_tx_free(void)
{
    uint16_t used;
    if (s_tx_head >= s_tx_tail)
    {
        used = (uint16_t)(s_tx_head - s_tx_tail);
    }
    else
    {
        used = (uint16_t)(GL_TX_BUF_SIZE - s_tx_tail + s_tx_head);
    }
    return (uint16_t)(GL_TX_BUF_SIZE - 1u - used);
}

static void gl_tx_put(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    if (gl_tx_free() < (uint16_t)(len + 1u))
    {
        /* 空间不够就整帧丢掉（不能写一半，否则接收端会看到半帧）。
         * 丢弃不会有累积危害：接收端本来就有帧头重同步机制。 */
        s_tx_drop++;
        return;
    }
    for (i = 0; i < len; i++)
    {
        s_tx_buf[s_tx_head] = data[i];
        s_tx_head++;
        if (s_tx_head >= GL_TX_BUF_SIZE)
        {
            s_tx_head = 0u;
        }
    }
}

/* 把待发数据尽量交给硬件（中断方式），在主循环里调用 */
static void gl_tx_pump(void)
{
    uint16_t n;
    if (!s_ready)
    {
        /* init 之前绝不发送：那时 NVIC 还没开，HAL_UART_Transmit_IT 会一直
         * 等 TXE 中断，结果把发送通道永久占死。 */
        return;
    }
    if (s_tx_busy_len != 0u)
    {
        /* ---- 发送通道卡死保护 ----
         * 如果因为丢中断/错误标志导致 HAL 一直以为"还在发"，
         * 遥测会永久停发 —— 现象是"上位机收不到数据但 H723 还在跑"，
         * 这是最难查的一类故障。这里超过 100ms 就强制恢复：
         * 丢掉这段发了一半的数据（接收端有帧头重同步，不会乱），
         * 然后继续从当前队列头发新的完整帧。 */
        if ((HAL_GetTick() - s_tx_busy_ms) > 100u)
        {
            HAL_UART_AbortTransmit(&huart1);
            s_tx_tail     = s_tx_head;
            s_tx_busy_len = 0u;
            s_tx_drop++;
        }
        else
        {
            return;                   /* 上一笔还没发完 */
        }
    }
    if (s_tx_head == s_tx_tail)
    {
        return;                       /* 没东西可发 */
    }
    if (s_tx_head > s_tx_tail)
    {
        n = (uint16_t)(s_tx_head - s_tx_tail);
    }
    else
    {
        n = (uint16_t)(GL_TX_BUF_SIZE - s_tx_tail);
    }
    if (n > GL_TX_CHUNK)
    {
        n = GL_TX_CHUNK;
    }
    if (HAL_UART_Transmit_IT(&huart1, &s_tx_buf[s_tx_tail], n) == HAL_OK)
    {
        s_tx_busy_len = n;
        s_tx_busy_ms  = HAL_GetTick();
    }
}

/* HAL 发送完成回调（中断上下文）：推进 tail */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1)
    {
        return;
    }
    s_tx_tail = (uint16_t)(s_tx_tail + s_tx_busy_len);
    if (s_tx_tail >= GL_TX_BUF_SIZE)
    {
        s_tx_tail = (uint16_t)(s_tx_tail - GL_TX_BUF_SIZE);
    }
    s_tx_busy_len = 0u;
}

/* 出错时也要让发送通道恢复，否则一次错误就把遥测永久卡死 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1)
    {
        return;
    }
    s_tx_busy_len = 0u;
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_FEF |
                                    UART_CLEAR_NEF | UART_CLEAR_PEF);
}

/* ========================== 发送封装 ========================== */

static void gl_send(uint8_t msg_id, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[GP_HEAD_LEN + GP_MAX_PAYLOAD + GP_CRC_LEN];
    uint16_t n = gp_build(msg_id, s_seq++, payload, len, frame, (uint16_t)sizeof(frame));
    if (n != 0u)
    {
        gl_tx_put(frame, n);
    }
}

void gimbal_link_log(const char *text)
{
    uint32_t now = HAL_GetTick();
    size_t   len;

    if (text == NULL)
    {
        return;
    }
    /* 限速：日志再多也不能挤占遥测的带宽 */
    if ((now - s_last_log_ms) < GL_LOG_PERIOD_MS)
    {
        s_log_dropped++;
        return;
    }
    s_last_log_ms = now;

    len = strlen(text);
    if (len > 60u)
    {
        len = 60u;
    }
    gl_send(GP_MSG_TEXT, (const uint8_t *)text, (uint8_t)len);
}

void gimbal_link_log_force(const char *text)
{
    /* 调参回执不能被限速吃掉：临时把"上次发送时刻"清零，发完照常限速。
     * 只允许少量调用（一条回执一行），不然会挤占遥测带宽。 */
    s_last_log_ms = 0u;
    gimbal_link_log(text);
}

/* ========================== 收到一帧 ========================== */

static float gl_clampf(float v, float lim)
{
    if (v > lim)
    {
        return lim;
    }
    if (v < -lim)
    {
        return -lim;
    }
    return v;
}

static void gl_handle_frame(uint32_t now_ms)
{
    switch (s_parser.msg_id)
    {
    case GP_MSG_AIM:
        if (s_parser.len >= 6u)
        {
            s_cmd.yaw_offset_deg   = gl_clampf((float)gp_get_i16(&s_parser.payload[0]) / 100.0f,
                                               GL_OFF_YAW_LIMIT);
            s_cmd.pitch_offset_deg = gl_clampf((float)gp_get_i16(&s_parser.payload[2]) / 100.0f,
                                               GL_OFF_PITCH_LIMIT);
            s_cmd.flags      = s_parser.payload[4];
            s_cmd.quality    = s_parser.payload[5];
            s_cmd.laser      = (uint8_t)((s_cmd.flags & GP_AIM_LASER_ON) ? 1u : 0u);
            s_cmd.aim_valid  = (uint8_t)((s_cmd.flags & GP_AIM_VALID) ? 1u : 0u);
            s_cmd.boost      = (uint8_t)((s_cmd.flags & GP_AIM_BOOST) ? 1u : 0u);
            s_cmd.drawing    = (uint8_t)((s_cmd.flags & GP_AIM_DRAWING) ? 1u : 0u);
            s_cmd.locked     = (uint8_t)((s_cmd.flags & GP_AIM_LOCKED) ? 1u : 0u);
            s_cmd.rx_count++;
            s_last_aim_ms = now_ms;
            if (!s_alive)
            {
                s_alive = 1u;
                gimbal_link_log("[LINK] 上位机已连接");
            }
        }
        break;

    case GP_MSG_MODE:
        if (s_parser.len >= 1u)
        {
            uint8_t mode = s_parser.payload[0];
            if (mode <= GP_MODE_ESTOP)
            {
                if (mode != s_cmd.mode)
                {
                    static const char *names[] = {"IDLE", "STAB", "AIM", "UNWIND", "ESTOP"};
                    char buf[40];
                    snprintf(buf, sizeof(buf), "[LINK] MODE -> %s", names[mode]);
                    s_last_log_ms = 0u;              /* 模式切换必须打出来 */
                    gimbal_link_log(buf);
                }
                s_cmd.mode = mode;
            }
            else
            {
                s_cmd.mode = GP_MODE_STAB;           /* 非法模式一律走安全态 */
            }
            /* 非 AIM 模式下不允许点激光 */
            if (s_cmd.mode != GP_MODE_AIM)
            {
                s_cmd.laser = 0u;
            }
            gl_send(GP_MSG_ACK, (const uint8_t[]){GP_MSG_MODE, 0u}, 2u);
        }
        break;

    case GP_MSG_SET_ZERO:
        s_zero_req      = 1u;
        s_cmd.yaw_offset_deg   = 0.0f;
        s_cmd.pitch_offset_deg = 0.0f;
        gl_send(GP_MSG_ACK, (const uint8_t[]){GP_MSG_SET_ZERO, 0u}, 2u);
        break;

    case GP_MSG_UNWIND:
        if (s_parser.len >= 2u)
        {
            /* 协议里是 0.1°/s；这里换算成 °/s 存起来，
             * 具体动作由 yuntai_task 在 UNWIND 模式下执行 */
            uint32_t dps = gp_get_u16(s_parser.payload) / 10u;
            s_cmd.unwind_dps = (uint8_t)((dps > 90u) ? 90u : dps);
        }
        gl_send(GP_MSG_ACK, (const uint8_t[]){GP_MSG_UNWIND, 0u}, 2u);
        break;

    case GP_MSG_HEARTBEAT:
        s_last_aim_ms = now_ms;                  /* 心跳也算链路活着 */
        if (!s_alive)
        {
            s_alive = 1u;
        }
        break;

    case GP_MSG_PARAM:
        /* 运行时就地调参：value 一律是"真实值 × 100"（分辨率 0.01） */
        if (s_parser.len >= 3u)
        {
            uint8_t pid = s_parser.payload[0];
            gimbal_link_on_param(pid, gp_get_i16(&s_parser.payload[1]));
            /* 回 ACK 让上位机确认"这一条真的落地了"（code 里带上 param id） */
            gl_send(GP_MSG_ACK, (const uint8_t[]){GP_MSG_PARAM, pid}, 2u);
        }
        break;

    default:
        /* 未知消息直接忽略，不回 NACK（避免干扰对方） */
        break;
    }
}

/* ========================== 对外接口 ========================== */

void gimbal_link_init(void)
{
    uint16_t i;

    for (i = 0; i < GL_RX_BUF_SIZE; i++)
    {
        s_rx_buf[i] = 0u;
    }
    for (i = 0; i < GL_TX_BUF_SIZE; i++)
    {
        s_tx_buf[i] = 0u;
    }
    s_rx_head = s_rx_tail = 0u;
    s_tx_head = s_tx_tail = 0u;
    s_tx_busy_len = 0u;
    s_rx_ovf = s_tx_drop = 0u;
    s_last_aim_ms = s_last_telem_ms = s_last_log_ms = 0u;
    s_alive = 0u;
    s_zero_req = 0u;
    s_have_telem = 0u;
    s_seq = 0u;
    s_log_dropped = 0u;
    memset(&s_cmd, 0, sizeof(s_cmd));
    memset(&s_telem, 0, sizeof(s_telem));
    s_cmd.mode = GP_MODE_IDLE;
    gp_parser_init(&s_parser);

    /* 打开 USART1 的接收中断（接收只做"存字节"这一件事） */
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_FEF |
                                    UART_CLEAR_NEF | UART_CLEAR_PEF);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    HAL_NVIC_SetPriority(USART1_IRQn, 6u, 0u);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    s_ready = 1u;
}

void gimbal_link_rx_isr(void)
{
    uint32_t isr = huart1.Instance->ISR;

    if ((isr & USART_ISR_RXNE_RXFNE) != 0u)
    {
        uint8_t  b = (uint8_t)(huart1.Instance->RDR & 0xFFu);
        uint16_t next = (uint16_t)(s_rx_head + 1u);
        if (next >= GL_RX_BUF_SIZE)
        {
            next = 0u;
        }
        if (next != s_rx_tail)
        {
            s_rx_buf[s_rx_head] = b;
            s_rx_head = next;
        }
        else
        {
            s_rx_ovf++;                       /* 缓冲满：丢字节，靠帧头重同步 */
        }
    }

    /* 溢出/帧错/噪声错误必须清掉，否则 RXNE 中断会被永久卡住 */
    if ((isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) != 0u)
    {
        __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_FEF |
                                        UART_CLEAR_NEF | UART_CLEAR_PEF);
    }
}

void gimbal_link_poll(uint32_t now_ms)
{
    /* ---- 1. 把接收缓冲里的字节喂给解析器 ---- */
    while (s_rx_tail != s_rx_head)
    {
        uint8_t b = s_rx_buf[s_rx_tail];
        s_rx_tail++;
        if (s_rx_tail >= GL_RX_BUF_SIZE)
        {
            s_rx_tail = 0u;
        }
        if (gp_parser_feed(&s_parser, b))
        {
            gl_handle_frame(now_ms);
        }
    }

    /* ---- 2. 看门狗：断流 -> 安全模式 ---- */
    if ((s_alive != 0u) && ((now_ms - s_last_aim_ms) > GL_WATCHDOG_MS))
    {
        s_alive = 0u;
        s_cmd.mode             = GP_MODE_STAB;
        s_cmd.laser            = 0u;
        s_cmd.yaw_offset_deg   = 0.0f;
        s_cmd.pitch_offset_deg = 0.0f;
        s_last_log_ms = 0u;
        gimbal_link_log("[LINK] AIM 断流 -> 自动切 STAB + 关激光");
    }

    /* ---- 3. 周期上报遥测 ---- */
    if ((s_have_telem != 0u) && ((now_ms - s_last_telem_ms) >= GL_TELEM_PERIOD_MS))
    {
        uint8_t payload[GP_GIMBAL_STATE_LEN];
        s_last_telem_ms = now_ms;
        gp_make_gimbal_state(payload,
                             s_telem.state, s_telem.fault,
                             s_telem.yaw_deg, s_telem.pitch_deg, s_telem.roll_deg,
                             s_telem.yaw_motor_deg, s_telem.pitch_motor_deg,
                             s_telem.gyro_y_dps, s_telem.gyro_z_dps,
                             s_telem.flags, s_telem.uptime_ms);
        gl_send(GP_MSG_GIMBAL_STATE, payload, GP_GIMBAL_STATE_LEN);
    }

    /* ---- 4. 把待发数据推给硬件 ---- */
    gl_tx_pump();
}

void gimbal_link_set_telem(const GimbalTelem_t *t)
{
    if (t != NULL)
    {
        s_telem = *t;
        s_have_telem = 1u;
    }
}

const GimbalCmd_t *gimbal_link_cmd(void)
{
    return &s_cmd;
}

uint8_t gimbal_link_alive(void)
{
    return s_alive;
}

uint8_t gimbal_link_take_zero_req(void)
{
    uint8_t r = s_zero_req;
    s_zero_req = 0u;
    return r;
}

const char *gimbal_link_stats(void)
{
    /* 保持在一行 60 字节以内，才发得进一条 TEXT 消息 */
    static char s_buf[64];
    snprintf(s_buf, sizeof(s_buf),
             "[STAT] rx=%lu crc=%lu ovf=%lu rsv=%lu txdrop=%lu",
             (unsigned long)s_parser.frames,   /* rx：收到多少帧 */
             (unsigned long)s_parser.crc_err,  /* crc：CRC 错帧数 */
             (unsigned long)s_rx_ovf,          /* ovf：接收缓冲溢出 */
             (unsigned long)s_parser.resync,   /* rsv：重同步次数 */
             (unsigned long)s_tx_drop);        /* txdrop：发送丢帧 */
    return s_buf;
}
