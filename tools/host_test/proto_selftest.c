/**
  ******************************************************************************
  * @file    proto_selftest.c
  * @brief   gimbal_proto.c 的 PC 端自测程序（不进 STM32 固件）
  *
  * 用法（由 run_test.py 自动调用）：
  *   proto_selftest gen              -> 打印自己组装的帧的十六进制
  *   proto_selftest parse <hex>      -> 逐字节喂给解析器，打印解析结果
  *
  * 它只依赖 Core/Src/gimbal_proto.c，不碰任何 HAL，
  * 所以能用 PC 上的 gcc 直接编译。
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "gimbal_proto.h"

static int hex_to_bin(const char *hex, uint8_t *out, int cap)
{
    int n = 0;
    size_t i, len = strlen(hex);
    for (i = 0; i + 1 < len && n < cap; i += 2)
    {
        unsigned v = 0;
        if (sscanf(&hex[i], "%2x", &v) != 1)
        {
            break;
        }
        out[n++] = (uint8_t)v;
    }
    return n;
}

static void dump(const uint8_t *buf, uint16_t n)
{
    uint16_t i;
    for (i = 0; i < n; i++)
    {
        printf("%02X", buf[i]);
    }
    printf("\n");
}

static int do_gen(void)
{
    uint8_t payload[GP_MAX_PAYLOAD];
    uint8_t frame[160];
    uint16_t n;
    static const char text[] = "[H723] BMI088 OK -> Motor Boot Delay";

    /* 1) GIMBAL_STATE：全部字段都给非平凡值，方便上位机逐字段比对 */
    n  = gp_make_gimbal_state(payload, 6u, 0u,
                              12.34f, -5.67f, 1.50f,
                              -90.00f, 10.25f, 2.50f, -1.25f,
                              (uint8_t)(GP_ST_CLOSED_LOOP | GP_ST_SPEED_MODE |
                                        GP_ST_ENCODER_OK | GP_ST_LASER_ON |
                                        GP_ST_READY),
                              123456u);
    n = gp_build(GP_MSG_GIMBAL_STATE, 7u, payload, (uint8_t)n, frame, sizeof(frame));
    dump(frame, n);

    /* 2) ACK(MODE, 0) */
    n = gp_make_ack(payload, GP_MSG_MODE, 0u);
    n = gp_build(GP_MSG_ACK, 8u, payload, (uint8_t)n, frame, sizeof(frame));
    dump(frame, n);

    /* 3) TEXT：调试文本 */
    memcpy(payload, text, sizeof(text) - 1u);
    n = gp_build(GP_MSG_TEXT, 9u, payload, (uint8_t)(sizeof(text) - 1u),
                 frame, sizeof(frame));
    dump(frame, n);

    /* 4) CRC 自检值也要报一下（"123456789" -> 0x4B37） */
    printf("crc789=%04X\n",
           gp_crc16((const uint8_t *)"123456789", 9u));
    return 0;
}

static int do_parse(const char *hex)
{
    uint8_t buf[1024];
    int n = hex_to_bin(hex, buf, (int)sizeof(buf));
    gp_parser_t p;
    int i;

    gp_parser_init(&p);
    for (i = 0; i < n; i++)
    {
        if (gp_parser_feed(&p, buf[i]))
        {
            printf("frame msg=0x%02X seq=%u len=%u", p.msg_id, p.seq, p.len);
            if (p.msg_id == GP_MSG_AIM && p.len >= 6u)
            {
                printf(" yaw=%d pitch=%d flags=%u quality=%u",
                       gp_get_i16(&p.payload[0]), gp_get_i16(&p.payload[2]),
                       p.payload[4], p.payload[5]);
            }
            else if (p.msg_id == GP_MSG_MODE && p.len >= 2u)
            {
                printf(" mode=%u arg=%u", p.payload[0], p.payload[1]);
            }
            else if (p.msg_id == GP_MSG_SET_ZERO)
            {
                printf(" zero");
            }
            else if (p.msg_id == GP_MSG_UNWIND && p.len >= 2u)
            {
                printf(" unwind_dps10=%u", gp_get_u16(p.payload));
            }
            else if (p.msg_id == GP_MSG_HEARTBEAT)
            {
                printf(" hb");
            }
            printf("\n");
        }
    }
    printf("stats frames=%lu crc_err=%lu bad_len=%lu resync=%lu\n",
           (unsigned long)p.frames, (unsigned long)p.crc_err,
           (unsigned long)p.bad_len, (unsigned long)p.resync);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "gen") == 0)
    {
        return do_gen();
    }
    if (argc >= 3 && strcmp(argv[1], "parse") == 0)
    {
        return do_parse(argv[2]);
    }
    printf("usage: proto_selftest gen | parse <hex>\n");
    return 1;
}
