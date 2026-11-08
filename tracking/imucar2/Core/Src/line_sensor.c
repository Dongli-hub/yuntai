#include "line_sensor.h"
#include "usart.h"
#include <stddef.h>
#include <string.h>

/* ================================ 参数 ================================ */
#define LS_RX_BUF_SIZE       128u      /* DMA 循环缓冲(必须是 2 的幂, 且 > 一帧长度) */
#define LS_FRAME_SIZE        64u       /* 单帧最大长度, 含 '$' 和 '#' */
#define LS_CMD_PERIOD_MS     500u      /* 上电阶段的命令重发间隔 */
#define LS_CMD_RETRY_MS      6000u     /* 超过这个时间还没数据就不再重发 */
#define LS_POWER_ON_WAIT_MS  1000u     /* 上电后等待模块稳定(手册建议久一些) */

/* 只要数字量: 校准关闭, 模拟量关闭, 数字量打开 */
#define LS_CMD_DIGITAL       "$0,0,1#"
#define LS_CMD_LEN           7u

/* ================================ 状态 ================================ */
static uint8_t  rx_buf[LS_RX_BUF_SIZE];      /* DMA 写入的环形缓冲 */
static uint16_t rx_tail;                     /* 已经交给解析器的位置 */

static char     frame[LS_FRAME_SIZE];        /* 正在拼接的一帧 */
static uint8_t  frame_len;
static uint8_t  in_frame;

static char     last_frame[LS_FRAME_SIZE];
static uint8_t  last_frame_len;

static volatile uint8_t  mask;               /* 归一化后的 8 路状态 */
static volatile uint8_t  raw_bits;           /* 报文里的原始 8 位(未做极性翻转) */
static volatile uint32_t last_ok_tick;
static volatile uint16_t ok_frames;
static volatile uint16_t bad_frames;

static uint32_t cmd_tick;
static uint8_t  cmd_done;

/* ================================ 工具 ================================ */

uint8_t LineSensor_PopCount(uint8_t v)
{
    uint8_t n = 0u;

    while (v != 0u) {
        n++;
        v = (uint8_t)(v & (uint8_t)(v - 1u));
    }
    return n;
}

#if LINE_REVERSE_ORDER
static uint8_t ls_reverse8(uint8_t v)
{
    v = (uint8_t)(((v & 0x55u) << 1) | ((v & 0xAAu) >> 1));
    v = (uint8_t)(((v & 0x33u) << 2) | ((v & 0xCCu) >> 2));
    v = (uint8_t)((v << 4) | (v >> 4));
    return v;
}
#endif

/* 当前 DMA 写指针在环形缓冲里的位置 */
static uint16_t ls_dma_pos(void)
{
    uint16_t ndtr;
    uint16_t pos;

    if (huart2.hdmarx == NULL) {
        return 0u;
    }
    ndtr = (uint16_t)__HAL_DMA_GET_COUNTER(huart2.hdmarx);
    if (ndtr > LS_RX_BUF_SIZE) {
        ndtr = LS_RX_BUF_SIZE;
    }
    pos = (uint16_t)(LS_RX_BUF_SIZE - ndtr);
    if (pos >= LS_RX_BUF_SIZE) {
        pos = 0u;
    }
    return pos;
}

/* ============================== 帧解析 ============================== */

/*
 * 数字帧: "$D,x1:0,x2:1,x3:0,...#"
 * 逐个找 ':' 后面的那个字符, 依次就是 x1..x8 的值。
 */
static void ls_parse_frame(void)
{
    uint8_t i = 0u;
    uint8_t m = 0u;
    uint8_t raw = 0u;
    const char *p;

    frame[frame_len] = '\0';

    memcpy(last_frame, frame, (size_t)frame_len + 1u);
    last_frame_len = frame_len;

    if ((frame_len < 3u) || (frame[0] != '$')) {
        bad_frames++;
        return;
    }

    if (frame[1] != 'D') {
        /* 不是数字帧(比如模块被设成模拟量输出), 本驱动不使用, 也不算在线 */
        return;
    }

    for (p = frame; *p != '\0'; p++) {
        if (*p == ':') {
            p++;
            if (i < LINE_CH_NUM) {
                uint8_t v = (uint8_t)((*p == '1') ? 1u : 0u);
                if (v != 0u) {
                    raw |= (uint8_t)(1u << i);
                }
#if LINE_ACTIVE_IS_ONE
                if (v != 0u) {
                    m |= (uint8_t)(1u << i);
                }
#else
                if (v == 0u) {
                    m |= (uint8_t)(1u << i);
                }
#endif
                i++;
            }
        }
    }

    if (i < LINE_CH_NUM) {
        bad_frames++;
        return;
    }

#if LINE_REVERSE_ORDER
    m = ls_reverse8(m);
    raw = ls_reverse8(raw);
#endif

    raw_bits = raw;
    mask = m;
    last_ok_tick = HAL_GetTick();
    if (ok_frames < 0xFFFFu) {
        ok_frames++;
    }
}

/* 逐字节喂进帧缓冲, 遇到 '$' 开始、'#' 结束 */
static void ls_feed(uint8_t b)
{
    if (b == (uint8_t)'$') {
        in_frame = 1u;
        frame_len = 0u;
    } else if (in_frame == 0u) {
        return;
    }

    if (frame_len >= (LS_FRAME_SIZE - 1u)) {
        /* 一帧太长, 说明丢了帧尾, 直接丢弃重新同步 */
        in_frame = 0u;
        frame_len = 0u;
        bad_frames++;
        return;
    }

    frame[frame_len] = (char)b;
    frame_len++;

    if (b == (uint8_t)'#') {
        in_frame = 0u;
        ls_parse_frame();
        frame_len = 0u;
    }
}

/* ============================== 对外接口 ============================== */

static void ls_send_cmd(void)
{
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)LS_CMD_DIGITAL, LS_CMD_LEN, 20u);
}

void LineSensor_Init(void)
{
    rx_tail = 0u;
    frame_len = 0u;
    in_frame = 0u;
    last_frame_len = 0u;
    mask = 0u;
    raw_bits = 0u;
    last_ok_tick = 0u;
    ok_frames = 0u;
    bad_frames = 0u;
    cmd_done = 0u;
    (void)memset(rx_buf, 0, sizeof(rx_buf));
    (void)memset(last_frame, 0, sizeof(last_frame));

    /* 上电后模块需要一点时间稳定(手册建议 20s, 这里只等一下让探头电流稳住) */
    HAL_Delay(LS_POWER_ON_WAIT_MS);

    /* 先挂上 DMA, 再发命令, 避免漏掉模块的第一帧 */
    (void)HAL_UART_Receive_DMA(&huart2, rx_buf, (uint16_t)LS_RX_BUF_SIZE);

    /* 不使用错误中断: 否则一次帧错误 HAL 会把接收 DMA 拆掉 */
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_ERR);
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_PE);

    ls_send_cmd();
    cmd_tick = HAL_GetTick();
}

void LineSensor_Poll(void)
{
    uint16_t pos = ls_dma_pos();

    while (rx_tail != pos) {
        ls_feed(rx_buf[rx_tail]);
        rx_tail = (uint16_t)((rx_tail + 1u) % LS_RX_BUF_SIZE);
    }
}

void LineSensor_Maintenance(void)
{
    uint32_t now = HAL_GetTick();

    if (cmd_done != 0u) {
        return;
    }

    if ((now - cmd_tick) < LS_CMD_PERIOD_MS) {
        return;
    }
    cmd_tick = now;

    if (ok_frames != 0u) {
        cmd_done = 1u;              /* 已经收到数据, 不用再喊了 */
        return;
    }
    if (now > LS_CMD_RETRY_MS) {
        cmd_done = 1u;              /* 一直没反应, 交给掉线保护处理 */
        return;
    }
    ls_send_cmd();
}

uint8_t LineSensor_GetMask(void)
{
    return mask;
}

uint8_t LineSensor_GetRaw(void)
{
    return raw_bits;
}

uint8_t LineSensor_IsOnline(void)
{
    if (last_ok_tick == 0u) {
        return 0u;
    }
    return ((HAL_GetTick() - last_ok_tick) < LINE_ONLINE_TIMEOUT_MS) ? 1u : 0u;
}

uint16_t LineSensor_GetFrameCount(void)
{
    return ok_frames;
}

uint16_t LineSensor_GetBadFrameCount(void)
{
    return bad_frames;
}

uint32_t LineSensor_GetLastOkTick(void)
{
    return last_ok_tick;
}

uint8_t LineSensor_GetLastFrame(char *out, uint8_t max)
{
    uint8_t n = last_frame_len;

    if ((out == NULL) || (max == 0u)) {
        return 0u;
    }
    if (n > (uint8_t)(max - 1u)) {
        n = (uint8_t)(max - 1u);
    }
    if (n != 0u) {
        (void)memcpy(out, last_frame, n);
    }
    out[n] = '\0';
    return n;
}
