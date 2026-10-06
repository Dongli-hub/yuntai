# -*- coding: utf-8 -*-
"""k230_link_test.py —— K230 <-> H723 通信自检（只测通信，不碰电机）

用途：K230 用 Type-C 接上位机、H723 只供 MCU 电（电机不供电）时，
      单独确认这条串口链路是不是通的。

程序只做三件事：
  1) 每 200ms 发一帧 HEARTBEAT(0x12)
  2) 收 H723 的遥测(0x90) / ACK(0x91) / 调试文本(0x93) 并统计
  3) 每秒打印一次结论式报告，同时把状态画到 IDE 的画面视图里

判断标准（不看电机、不看编码器、不看 READY）：
  * 收到 GIMBAL_STATE  -> H723 -> K230 方向通
  * 收到 ACK(0x12)     -> K230 -> H723 方向也通（H723 收到心跳会回 ACK）
  两个都有 = 双向通信 OK。

没有独立屏幕也没关系：显示初始化失败会自动跳过，只看串行终端输出即可。
"""
import os
import time

# ============================ 参数 ============================
# 引脚：K230 EXPORT 口 IO9 = TXD, IO10 = RXD（接 H723 的 UART7）
UART_BACKEND = "auto"      # auto | yb | machine
UART_UNIT = 1
UART_TX = 9
UART_RX = 10
UART_BAUD = 115200

# 备选：如果接的是 12Pin GPIO 上的 UART3，把上面三行换成
#   UART_UNIT = 3 / UART_TX = 32 / UART_RX = 33

DISPLAY_MODE = "VIRT"      # VIRT(只用 IDE 画面) | LCD(带屏) | OFF
DISPLAY_W = 640
DISPLAY_H = 480

HEARTBEAT_MS = 200
PRINT_MS = 1000
# ==============================================================

SOF = b"\xAA\x55"
MSG_AIM = 0x10
MSG_MODE = 0x11
MSG_HEARTBEAT = 0x12
MSG_SET_ZERO = 0x13
MSG_GIMBAL_STATE = 0x90
MSG_ACK = 0x91
MSG_VERSION = 0x92
MSG_TEXT = 0x93


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


def build_frame(msg_id, payload=b"", seq=0):
    import ustruct
    body = ustruct.pack("<BBB", msg_id & 0xFF, seq & 0xFF,
                        len(payload)) + payload
    return SOF + body + ustruct.pack("<H", crc16(body))


class FrameParser(object):
    def __init__(self):
        self.buf = bytearray()
        self.ok = 0
        self.crc_err = 0
        self.bad_len = 0
        self.resync = 0
        self.bytes_rx = 0

    def feed(self, data):
        out = []
        self.bytes_rx += len(data)
        self.buf.extend(data)
        while True:
            i = self.buf.find(SOF)
            if i < 0:
                if len(self.buf) > 1:
                    self.resync += 1
                    del self.buf[:-1]
                break
            if i > 0:
                self.resync += 1
                del self.buf[:i]
            if len(self.buf) < 5:
                break
            length = self.buf[4]
            if length > 240:
                self.bad_len += 1
                del self.buf[:2]
                continue
            total = 5 + length + 2
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:5 + length])
            crc_rx = self.buf[5 + length] | (self.buf[6 + length] << 8)
            if crc16(body) != crc_rx:
                self.crc_err += 1
                del self.buf[:2]
                continue
            self.ok += 1
            out.append((body[0], body[1], bytes(body[3:])))
            del self.buf[:total]
        return out


def open_uart():
    """优先用 machine.UART 显式指定引脚，不行再退回亚博封装的 YbUart。"""
    if UART_BACKEND in ("auto", "machine"):
        try:
            from machine import UART, Pin
            dev = UART(UART_UNIT, baudrate=UART_BAUD, tx=Pin(UART_TX),
                       rx=Pin(UART_RX), bits=8, parity=None, stop=0)
            return dev, "machine.UART(%d) TX=IO%d RX=IO%d" % (
                UART_UNIT, UART_TX, UART_RX)
        except Exception as e:
            if UART_BACKEND == "machine":
                raise
            print("machine.UART 打开失败(%s)，改用 YbUart" % e)
    from ybUtils.YbUart import YbUart
    return YbUart(baudrate=UART_BAUD), "YbUart(亚博封装)"


def uart_read(dev, n=256):
    try:
        if hasattr(dev, "any") and dev.any() <= 0:
            return b""
        return dev.read(n) or b""
    except Exception:
        return b""


def main():
    dev, uart_name = open_uart()
    print("=" * 56)
    print("K230 通信自检")
    print("串口: %s @ %d" % (uart_name, UART_BAUD))
    print("=" * 56)

    parser = FrameParser()

    # ---- 显示（没有屏也能跑，失败就跳过） ----
    canvas = None
    display_ok = False
    if DISPLAY_MODE != "OFF":
        try:
            from media.display import Display
            from media.media import MediaManager
            import image
            if DISPLAY_MODE == "LCD":
                Display.init(Display.ST7701, width=DISPLAY_W,
                             height=DISPLAY_H, to_ide=True)
            else:
                Display.init(Display.VIRT, width=DISPLAY_W,
                             height=DISPLAY_H, fps=30)
            MediaManager.init()
            canvas = image.Image(DISPLAY_W, DISPLAY_H, image.RGB565)
            display_ok = True
            print("显示已初始化（画到 IDE 画面视图）")
        except Exception as e:
            print("显示初始化失败(%s)，只输出文字" % e)

    n_state = 0
    n_ack = 0
    n_ack_hb = 0
    n_text = 0
    hb_sent = 0
    last_state = None
    t_hb = time.ticks_ms()
    t_print = time.ticks_ms()
    t0 = time.ticks_ms()

    print("开始：每 200ms 发一帧心跳，等待 H723 回应...")
    try:
        while True:
            os.exitpoint()
            now = time.ticks_ms()

            if time.ticks_diff(now, t_hb) >= HEARTBEAT_MS:
                t_hb = now
                try:
                    dev.write(build_frame(MSG_HEARTBEAT))
                    hb_sent += 1
                except Exception as e:
                    print("发送失败: %s" % e)

            data = uart_read(dev)
            if data:
                for msg_id, seq, payload in parser.feed(data):
                    if msg_id == MSG_GIMBAL_STATE:
                        n_state += 1
                        if len(payload) == 21:
                            import ustruct
                            last_state = ustruct.unpack("<BBhhhhhhhBI",
                                                        payload)
                    elif msg_id == MSG_ACK:
                        n_ack += 1
                        if len(payload) >= 1 and payload[0] == MSG_HEARTBEAT:
                            n_ack_hb += 1
                    elif msg_id == MSG_TEXT:
                        n_text += 1
                        try:
                            print("  H723: %s" % payload.decode("utf-8"))
                        except Exception:
                            print("  H723 TEXT %r" % payload)
                    elif msg_id == MSG_VERSION:
                        print("  H723 VERSION: %r" % payload)

            if time.ticks_diff(now, t_print) >= PRINT_MS:
                t_print = now
                print("-" * 56)
                print("心跳发出 %d | 收到: 遥测%d ACK%d(其中心跳ACK%d) 文本%d "
                      "| 字节%d ok%d crc_err%d bad_len%d"
                      % (hb_sent, n_state, n_ack, n_ack_hb, n_text,
                         parser.bytes_rx, parser.ok, parser.crc_err,
                         parser.bad_len))
                if last_state is not None:
                    print("  遥测: state=%d fault=%d yaw=%.2f pitch=%.2f "
                          "flags=0x%02X up=%dms"
                          % (last_state[0], last_state[1],
                             last_state[2] / 100.0, last_state[3] / 100.0,
                             last_state[9], last_state[10]))
                # ---- 结论 ----
                if n_state > 0 and n_ack_hb > 0:
                    print("  >>> 双向通信 OK（H723 收到心跳并回了 ACK）")
                elif n_state > 0:
                    print("  >>> H723->K230 通了；还没收到心跳 ACK："
                          "等 1~2 秒，若一直没有看 K230 TX / H723 RX 这根线")
                elif parser.bytes_rx > 0 and parser.ok == 0:
                    print("  >>> 收到字节但一帧都解不出来：波特率或线材问题"
                          "（两边都必须 115200 8N1，共地）")
                else:
                    print("  >>> 一个字节都没收到：检查接线 "
                          "(K230 IO9->H723 UART7 RX, IO10->UART7 TX, GND 共地)"
                          "、H723 是否已上电")

                if display_ok and canvas is not None:
                    canvas.clear()
                    y = 6
                    canvas.draw_string_advanced(6, y, 24,
                                                "K230 <-> H723 LINK TEST",
                                                color=(255, 255, 255))
                    y += 34
                    lines = [
                        "uart: %s" % uart_name,
                        "hb_sent=%d  state=%d  ack=%d(hb %d)  text=%d"
                        % (hb_sent, n_state, n_ack, n_ack_hb, n_text),
                        "rx_bytes=%d ok=%d crc_err=%d bad_len=%d"
                        % (parser.bytes_rx, parser.ok, parser.crc_err,
                           parser.bad_len),
                    ]
                    if last_state is not None:
                        lines.append("state=%d fault=%d up=%dms"
                                     % (last_state[0], last_state[1],
                                        last_state[10]))
                        lines.append("yaw=%.2f pitch=%.2f flags=0x%02X"
                                     % (last_state[2] / 100.0,
                                        last_state[3] / 100.0,
                                        last_state[9]))
                    if n_state > 0 and n_ack_hb > 0:
                        lines.append("RESULT: BOTH OK")
                        col = (0, 255, 0)
                    elif n_state > 0 or parser.bytes_rx > 0:
                        lines.append("RESULT: PARTIAL")
                        col = (255, 220, 0)
                    else:
                        lines.append("RESULT: NO DATA")
                        col = (255, 80, 80)
                    for ln in lines:
                        canvas.draw_string_advanced(6, y, 20, ln,
                                                    color=(200, 255, 200))
                        y += 28
                    canvas.draw_string_advanced(6, y + 10, 22, lines[-1],
                                                color=col)
                    Display.show_image(canvas)
            time.sleep_ms(2)
    except KeyboardInterrupt:
        print("用户停止")
    finally:
        try:
            dev.deinit()
        except Exception:
            pass
        if display_ok:
            try:
                Display.deinit()
                os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
                time.sleep_ms(100)
                MediaManager.deinit()
            except Exception:
                pass
        print("已退出。通信统计：ok=%d crc_err=%d 字节=%d"
              % (parser.ok, parser.crc_err, parser.bytes_rx))


main()
