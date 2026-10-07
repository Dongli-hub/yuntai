# -*- coding: utf-8 -*-
"""t1_link.py —— K230 与 H723 串口链路测试（第二步）

只做两件事：
  1) 每 200ms 给 H723 发一帧 HEARTBEAT_G（0x12）
  2) 收 H723 的遥测（0x90 GIMBAL_STATE，50Hz）并显示在屏幕上

注意：本脚本不会切换 H723 的模式、不会开激光，安全。
屏幕上能看到 state / flags / yaw / pitch 就是通了。
"""
import os
import time

# ============================ 参数 ============================
UART_BACKEND = "auto"      # auto | yb | machine
UART_UNIT = 1              # machine.UART 用
UART_TX = 9                # EXPORT: IO9(TXD)；12Pin UART3 是 32
UART_RX = 10               # EXPORT: IO10(RXD)；12Pin UART3 是 33
UART_BAUD = 115200

DISPLAY_MODE = "VIRT"      # VIRT(无屏,只看IDE) | LCD(ST7701屏+IDE)
DISPLAY_W = 640
DISPLAY_H = 480

HEARTBEAT_MS = 200         # 心跳周期
PRINT_MS = 1000            # 终端打印周期
# ==============================================================

SOF = b"\xAA\x55"
MSG_HEARTBEAT_G = 0x12
MSG_GIMBAL_STATE = 0x90
MSG_ACK = 0x91
MSG_TEXT = 0x93


def _crc_table():
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
        table.append(crc)
    return table


_CRC = _crc_table()


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ _CRC[(crc ^ b) & 0xFF]
    return crc & 0xFFFF


def build_frame(msg_id, payload=b"", seq=0):
    import ustruct
    body = ustruct.pack("<BBB", msg_id & 0xFF, seq & 0xFF, len(payload)) + payload
    return SOF + body + ustruct.pack("<H", crc16(body))


class FrameParser(object):
    def __init__(self):
        self.buf = bytearray()
        self.crc_err = 0
        self.bad_len = 0
        self.frames_ok = 0

    def feed(self, data):
        out = []
        self.buf.extend(data)
        while True:
            i = self.buf.find(SOF)
            if i < 0:
                if len(self.buf) > 1:
                    # MicroPython 的 bytearray 不支持切片删除，只能重新切片
                    self.buf = self.buf[-1:]
                break
            if i > 0:
                self.buf = self.buf[i:]
            if len(self.buf) < 5:
                break
            length = self.buf[4]
            if length > 240:
                self.bad_len += 1
                self.buf = self.buf[2:]
                continue
            total = 5 + length + 2
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:5 + length])
            crc_rx = self.buf[5 + length] | (self.buf[6 + length] << 8)
            if crc16(body) != crc_rx:
                self.crc_err += 1
                self.buf = self.buf[2:]
                continue
            self.frames_ok += 1
            out.append((body[0], body[1], bytes(body[3:])))
            self.buf = self.buf[total:]
        return out


class UartLink(object):
    def __init__(self):
        self.dev = None
        self.kind = ""
        if UART_BACKEND in ("auto", "yb"):
            try:
                from ybUtils.YbUart import YbUart
                self.dev = YbUart(baudrate=UART_BAUD)
                self.kind = "YbUart"
            except Exception as e:
                if UART_BACKEND == "yb":
                    raise
                print("YbUart 不可用(%s)，改用 machine.UART" % e)
        if self.dev is None:
            from machine import UART, Pin
            self.dev = UART(UART_UNIT, baudrate=UART_BAUD,
                            tx=Pin(UART_TX), rx=Pin(UART_RX),
                            bits=8, parity=None, stop=0)
            self.kind = "machine.UART(%d) tx=IO%d rx=IO%d" % (
                UART_UNIT, UART_TX, UART_RX)
        print("串口已打开: %s @%d" % (self.kind, UART_BAUD))

    def write(self, data):
        self.dev.write(data)

    def read(self, n=256):
        try:
            if hasattr(self.dev, "any"):
                if self.dev.any() <= 0:
                    return b""
            return self.dev.read(n) or b""
        except Exception:
            return b""

    def close(self):
        try:
            self.dev.deinit()
        except Exception:
            pass


def unpack_gimbal_state(p):
    import ustruct
    if len(p) != 21:
        return None
    v = ustruct.unpack("<BBhhhhhhhBI", p)
    return {
        "state": v[0], "fault": v[1],
        "yaw": v[2] / 100.0, "pitch": v[3] / 100.0, "roll": v[4] / 100.0,
        "ymotor": v[5] / 100.0, "pmotor": v[6] / 100.0,
        "gyro_y": v[7] / 100.0, "gyro_z": v[8] / 100.0,
        "flags": v[9], "up_ms": v[10],
    }


def main():
    from media.display import Display
    from media.media import MediaManager

    link = UartLink()
    parser = FrameParser()

    if DISPLAY_MODE == "LCD":
        Display.init(Display.ST7701, width=DISPLAY_W, height=DISPLAY_H,
                     to_ide=True)
    else:
        Display.init(Display.VIRT, width=DISPLAY_W, height=DISPLAY_H, fps=30)
    MediaManager.init()

    import image
    canvas = image.Image(DISPLAY_W, DISPLAY_H, image.RGB565)

    t_last_hb = time.ticks_ms()
    t_last_print = time.ticks_ms()
    st = None
    st_count = 0
    last_rx_ms = 0
    ack_text = ""

    print("开始监听 H723 遥测（每 1 秒打印一行）...")
    try:
        while True:
            os.exitpoint()
            now = time.ticks_ms()

            if time.ticks_diff(now, t_last_hb) >= HEARTBEAT_MS:
                t_last_hb = now
                link.write(build_frame(MSG_HEARTBEAT_G))

            data = link.read(256)
            if data:
                for msg_id, seq, payload in parser.feed(data):
                    if msg_id == MSG_GIMBAL_STATE:
                        s = unpack_gimbal_state(payload)
                        if s:
                            st = s
                            st_count += 1
                            last_rx_ms = now
                    elif msg_id == MSG_ACK and len(payload) >= 2:
                        ack_text = "ACK msg=0x%02X code=%d" % (
                            payload[0], payload[1])
                    elif msg_id == MSG_TEXT:
                        try:
                            print("H723: %s" % payload.decode("utf-8"))
                        except Exception:
                            print("H723 TEXT %r" % payload)

            canvas.clear()
            y = 4
            canvas.draw_string_advanced(
                4, y, 24, "T1 LINK  %s" % link.kind, color=(255, 255, 255))
            y += 30
            if st is None:
                canvas.draw_string_advanced(
                    4, y, 24, "no telemetry...", color=(255, 80, 80))
            else:
                age = time.ticks_diff(now, last_rx_ms)
                ready = "READY" if (st["flags"] & 0x10) else "not-ready"
                lines = [
                    "state=%d fault=%d %s" % (st["state"], st["fault"], ready),
                    "yaw=%.2f pitch=%.2f roll=%.2f" % (
                        st["yaw"], st["pitch"], st["roll"]),
                    "ymotor=%.1f pmotor=%.1f" % (st["ymotor"], st["pmotor"]),
                    "gyro_y=%.1f gyro_z=%.1f" % (st["gyro_y"], st["gyro_z"]),
                    "flags=0x%02X up=%dms" % (st["flags"], st["up_ms"]),
                    "rx age=%dms  frames=%d" % (age, st_count),
                ]
                for ln in lines:
                    canvas.draw_string_advanced(4, y, 20, ln,
                                                color=(120, 255, 120))
                    y += 26
            canvas.draw_string_advanced(
                4, DISPLAY_H - 40, 20,
                "ok=%d crc_err=%d bad_len=%d" % (
                    parser.frames_ok, parser.crc_err, parser.bad_len),
                color=(255, 220, 0))
            if ack_text:
                canvas.draw_string_advanced(4, DISPLAY_H - 20, 16, ack_text,
                                            color=(200, 200, 255))
            Display.show_image(canvas)

            if time.ticks_diff(now, t_last_print) >= PRINT_MS:
                t_last_print = now
                if st is None:
                    print("遥测: 无 | ok=%d crc_err=%d bad_len=%d"
                          % (parser.frames_ok, parser.crc_err, parser.bad_len))
                else:
                    print("遥测: state=%d fault=%d yaw=%.2f pitch=%.2f "
                          "flags=0x%02X up=%dms | ok=%d crc_err=%d bad_len=%d"
                          % (st["state"], st["fault"], st["yaw"], st["pitch"],
                             st["flags"], st["up_ms"], parser.frames_ok,
                             parser.crc_err, parser.bad_len))
            time.sleep_ms(2)
    except KeyboardInterrupt:
        print("用户停止")
    finally:
        link.close()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()
        print("已退出")


main()
