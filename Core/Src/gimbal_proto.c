/**
  ******************************************************************************
  * @file    gimbal_proto.c
  * @brief   云台串口协议的纯逻辑实现（无 HAL 依赖，可在 PC 上单测）
  ******************************************************************************
  */

#include "gimbal_proto.h"

/* ==========================================================================
 * CRC16/Modbus
 *   初值 0xFFFF，多项式 0xA001（0x8005 的反射形式）
 *   自检：crc16("123456789") == 0x4B37
 * ========================================================================== */
static uint16_t s_crc_table[256];
static uint8_t  s_crc_ready = 0;

static void crc_table_build(void)
{
    uint16_t i;
    uint8_t  bit;
    for (i = 0; i < 256u; i++)
    {
        uint16_t crc = i;
        for (bit = 0; bit < 8u; bit++)
        {
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1);
        }
        s_crc_table[i] = crc;
    }
    s_crc_ready = 1;
}

uint16_t gp_crc16_update(uint16_t crc, uint8_t byte)
{
    if (!s_crc_ready)
    {
        crc_table_build();
    }
    return (uint16_t)((crc >> 8) ^ s_crc_table[(crc ^ byte) & 0xFFu]);
}

uint16_t gp_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        crc = gp_crc16_update(crc, data[i]);
    }
    return crc;
}

/* ==========================================================================
 * 小端读写
 *   全部手写字节操作，不依赖结构体对齐，避免不同编译器/平台差异
 * ========================================================================== */
int16_t gp_get_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint16_t gp_get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint32_t gp_get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

void gp_put_i16(uint8_t *p, int16_t v)
{
    gp_put_u16(p, (uint16_t)v);
}

void gp_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

void gp_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

int16_t gp_deg_to_x100(float deg)
{
    float v = deg * 100.0f;
    /* 四舍五入后再限幅；±327.67° 之外一律夹住 */
    if (v >= 0.0f)
    {
        v += 0.5f;
    }
    else
    {
        v -= 0.5f;
    }
    if (v > 32767.0f)
    {
        return 32767;
    }
    if (v < -32768.0f)
    {
        return -32768;
    }
    return (int16_t)v;
}

/* ==========================================================================
 * 流式解析状态机
 * ========================================================================== */
enum
{
    GP_S_SOF1 = 0,
    GP_S_SOF2,
    GP_S_MSG,
    GP_S_SEQ,
    GP_S_LEN,
    GP_S_PAYLOAD,
    GP_S_CRC_LO,
    GP_S_CRC_HI
};

void gp_parser_init(gp_parser_t *p)
{
    uint8_t *raw = (uint8_t *)p;
    uint16_t i;
    for (i = 0; i < sizeof(gp_parser_t); i++)
    {
        raw[i] = 0u;
    }
    p->state = GP_S_SOF1;
}

uint8_t gp_parser_feed(gp_parser_t *p, uint8_t byte)
{
    switch (p->state)
    {
    case GP_S_SOF1:
        if (byte == GP_SOF1)
        {
            p->state = GP_S_SOF2;
        }
        else
        {
            p->resync++;
        }
        break;

    case GP_S_SOF2:
        if (byte == GP_SOF2)
        {
            p->state = GP_S_MSG;
            p->crc_calc = 0xFFFFu;
        }
        else if (byte == GP_SOF1)
        {
            /* 可能是 AA AA 55 这种重叠帧头，留在本状态等下一个字节 */
            p->resync++;
        }
        else
        {
            p->resync++;
            p->state = GP_S_SOF1;
        }
        break;

    case GP_S_MSG:
        p->msg_id   = byte;
        p->crc_calc = gp_crc16_update(p->crc_calc, byte);
        p->state    = GP_S_SEQ;
        break;

    case GP_S_SEQ:
        p->seq      = byte;
        p->crc_calc = gp_crc16_update(p->crc_calc, byte);
        p->state    = GP_S_LEN;
        break;

    case GP_S_LEN:
        p->len      = byte;
        p->crc_calc = gp_crc16_update(p->crc_calc, byte);
        p->idx      = 0u;
        if (byte > GP_MAX_PAYLOAD)
        {
            /* 长度非法：这个"帧头"是假的，丢掉它继续找下一个 */
            p->bad_len++;
            p->state = GP_S_SOF1;
        }
        else if (byte == 0u)
        {
            p->state = GP_S_CRC_LO;
        }
        else
        {
            p->state = GP_S_PAYLOAD;
        }
        break;

    case GP_S_PAYLOAD:
        p->payload[p->idx] = byte;
        p->crc_calc        = gp_crc16_update(p->crc_calc, byte);
        p->idx++;
        if (p->idx >= p->len)
        {
            p->state = GP_S_CRC_LO;
        }
        break;

    case GP_S_CRC_LO:
        p->crc_rx = byte;
        p->state  = GP_S_CRC_HI;
        break;

    case GP_S_CRC_HI:
        p->crc_rx |= (uint16_t)((uint16_t)byte << 8);
        p->state   = GP_S_SOF1;
        if (p->crc_rx == p->crc_calc)
        {
            p->frames++;
            return 1u;
        }
        p->crc_err++;
        break;

    default:
        p->state = GP_S_SOF1;
        break;
    }
    return 0u;
}

/* ==========================================================================
 * 组帧
 * ========================================================================== */
uint16_t gp_build(uint8_t msg_id, uint8_t seq, const uint8_t *payload,
                  uint8_t len, uint8_t *out, uint16_t out_cap)
{
    uint16_t total;
    uint16_t crc;
    uint8_t  i;

    if (len > GP_MAX_PAYLOAD)
    {
        return 0u;
    }
    total = (uint16_t)(GP_HEAD_LEN + len + GP_CRC_LEN);
    if (out_cap < total)
    {
        return 0u;
    }

    out[0] = GP_SOF1;
    out[1] = GP_SOF2;
    out[2] = msg_id;
    out[3] = seq;
    out[4] = len;
    for (i = 0; i < len; i++)
    {
        out[GP_HEAD_LEN + i] = payload[i];
    }
    crc = gp_crc16(&out[2], (uint16_t)(3u + len));
    gp_put_u16(&out[GP_HEAD_LEN + len], crc);
    return total;
}

uint8_t gp_make_gimbal_state(
    uint8_t *out,
    uint8_t  state, uint8_t fault,
    float yaw_deg, float pitch_deg, float roll_deg,
    float yaw_motor_deg, float pitch_motor_deg,
    float gyro_y_dps, float gyro_z_dps,
    uint8_t flags, uint32_t uptime_ms)
{
    /* 顺序必须与上位机 struct.unpack("<BBhhhhhhhBI") 完全一致 */
    out[0] = state;
    out[1] = fault;
    gp_put_i16(&out[2],  gp_deg_to_x100(yaw_deg));
    gp_put_i16(&out[4],  gp_deg_to_x100(pitch_deg));
    gp_put_i16(&out[6],  gp_deg_to_x100(roll_deg));
    gp_put_i16(&out[8],  gp_deg_to_x100(yaw_motor_deg));
    gp_put_i16(&out[10], gp_deg_to_x100(pitch_motor_deg));
    gp_put_i16(&out[12], gp_deg_to_x100(gyro_y_dps));
    gp_put_i16(&out[14], gp_deg_to_x100(gyro_z_dps));
    out[16] = flags;
    gp_put_u32(&out[17], uptime_ms);
    return GP_GIMBAL_STATE_LEN;
}

uint8_t gp_make_ack(uint8_t *out, uint8_t ack_msg_id, uint8_t code)
{
    out[0] = ack_msg_id;
    out[1] = code;
    return 2u;
}

