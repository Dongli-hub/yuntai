# -*- coding: utf-8 -*-
"""k230_tx_sweep.py —— K230 "发送脚"扫描（配合 USB-TTL + 串口助手用）

目的：确认 K230 到底在哪个引脚上把数据发出去了。

做法：每 4 秒换一种发送方式，并在终端打印"现在正在用哪个脚发"：
    [1/3] IO9  （亚博封装 YbUart，最常用）
    [2/3] IO32 （12Pin GPIO 第 5 脚，UART3_TXD）
    [3/3] IO10 （把 UART1 的收发反过来用：IO10 发、IO9 收）

用法：
  1) K230 用 Type-C 连电脑（CanMV IDE），运行本脚本；
  2) USB-TTL：GND 接 K230 的 GND，**RX 依次去碰上面三个脚**；
     串口助手 115200 / 8 / N / 1，HEX 显示；
  3) 看到哪一段有 `AA 55 12 00 00 D1 C5` 循环出现，
     就说明那一行对应的引脚是能发数据的 —— 把结果告诉我。

注意：测 [3/3] 那 4 秒时，原来的接收方向会暂时失效属正常现象。
"""
import os
import time

BAUD = 115200
SOF = b"\xAA\x55"
MSG_HEARTBEAT = 0x12
HOLD_MS = 4000          # 每种发送方式持续多久


def _crc_table():
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if (c & 1) else (c >> 1)
        t.append(c)
    return t


_CRC = _crc_table()


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ _CRC[(crc ^ b) & 0xFF]
    return crc & 0xFFFF


def heartbeat_frame():
    import ustruct
    body = ustruct.pack("<BBB", MSG_HEARTBEAT, 0, 0)
    return SOF + body + ustruct.pack("<H", crc16(body))


def open_yb():
    """[1/3] 亚博封装：IO9 = TXD, IO10 = RXD"""
    from ybUtils.YbUart import YbUart
    return YbUart(baudrate=BAUD), "IO9"


def open_uart3():
    """[2/3] 12Pin GPIO：IO32 = TXD, IO33 = RXD"""
    from machine import FPIOA, UART
    fp = FPIOA()
    fp.set_function(32, FPIOA.UART3_TXD, ie=0, oe=1, pu=1)
    fp.set_function(33, FPIOA.UART3_RXD, ie=1, oe=0, pu=1)
    return UART(3, baudrate=BAUD), "IO32"


def open_uart1_swapped():
    """[3/3] 把 UART1 反过来用：IO10 = TXD, IO9 = RXD"""
    from machine import FPIOA, UART
    fp = FPIOA()
    fp.set_function(10, FPIOA.UART1_TXD, ie=0, oe=1, pu=1)
    fp.set_function(9, FPIOA.UART1_RXD, ie=1, oe=0, pu=1)
    return UART(1, baudrate=BAUD), "IO10"


OPENERS = [
    ("IO9  (YbUart 封装)", open_yb),
    ("IO32 (12Pin 第5脚 UART3)", open_uart3),
    ("IO10 (UART1 反用)", open_uart1_swapped),
]


def main():
    frame = heartbeat_frame()
    print("=" * 58)
    print("K230 发送脚扫描：每 %d 秒换一个脚，终端会提示当前是哪个" % (HOLD_MS // 1000))
    print("USB-TTL: RX 去碰提示的那个脚, GND 共地, 助手 115200 HEX")
    print("应该看到: %s 循环出现" % " ".join("%02X" % x for x in frame))
    print("=" * 58)

    while True:
        for idx, (label, opener) in enumerate(OPENERS):
            try:
                dev, pin_name = opener()
            except Exception as e:
                print("[%d/%d] %s 打不开: %s" % (idx + 1, len(OPENERS), label, e))
                continue

            print("")
            print("---- [%d/%d] 现在用 %s 发送（%d 秒）----"
                  % (idx + 1, len(OPENERS), label, HOLD_MS // 1000))

            t_end = time.ticks_ms() + HOLD_MS
            t_hb = time.ticks_ms()
            n = 0
            while time.ticks_diff(t_end, time.ticks_ms()) > 0:
                os.exitpoint()
                now = time.ticks_ms()
                if time.ticks_diff(now, t_hb) >= 200:
                    t_hb = now
                    try:
                        dev.write(frame)
                        n += 1
                    except Exception as e:
                        print("  发送失败: %s" % e)
                        break
                time.sleep_ms(2)
            print("  已发出 %d 帧（引脚 %s）" % (n, pin_name))

            try:
                dev.deinit()
            except Exception:
                pass


main()
