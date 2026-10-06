# -*- coding: utf-8 -*-
"""t3_spot.py —— 激光光斑检测 + 光轴点标定（第四步）

！开激光前必须确认：
    云台已固定、激光指向靶纸方向，绝不朝人眼；
    按下停止/拔线后 H723 会在 500ms 内自动切 STAB 关激光。

LIGHT_LASER = True 时脚本会：
    1) 等 H723 READY（遥测 flags bit4）
    2) 发 SET_ZERO(0x13) 记零位 -> MODE(0x11)=AIM
    3) 每 20ms 发 AIM(0x10)，偏置 0，flags = LASER_ON|AIM_VALID
   退出时自动发 MODE=STAB 关激光。

检测：在 ROI（光斑只可能出现在这一小块）里用 LAB 阈值找亮色块，
画面上白框 = ROI，绿框 = 命中的光斑，红十字 = 光斑中心。
终端打印 中心/面积/RGB/LAB，用于改阈值。
"""
import os
import time

# ============================ 参数 ============================
UART_BACKEND = "auto"      # auto | yb | machine
UART_UNIT = 1
UART_TX = 9
UART_RX = 10
UART_BAUD = 115200

IMG_W = 640
IMG_H = 480
DISPLAY_MODE = "VIRT"
DISPLAY_W = 640
DISPLAY_H = 480

LIGHT_LASER = True         # False = 不开激光（纯看画面）

# ROI：光斑可能出现的小窗口（同轴装配 -> 位置刚性固定）
ROI_CX = 320
ROI_CY = 240
ROI_HALF = 90

# LAB 阈值（可多组）：亮且偏暖（红激光过曝中心发白、边缘暖色）
THRESHOLDS = [
    (60, 100, 8, 60, -10, 60),    # 暖色亮斑
    (85, 100, -20, 40, -20, 60),  # 过曝白芯
]
BLOB_MIN_AREA = 3
BLOB_MAX_AREA = 4000

PRINT_MS = 300
# ==============================================================

SOF = b"\xAA\x55"
MSG_AIM = 0x10
MSG_MODE = 0x11
MSG_HEARTBEAT = 0x12
MSG_SET_ZERO = 0x13
MSG_GIMBAL_STATE = 0x90
MODE_STAB = 1
MODE_AIM = 2


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

    def feed(self, data):
        out = []
        self.buf.extend(data)
        while True:
            i = self.buf.find(SOF)
            if i < 0:
                if len(self.buf) > 1:
                    del self.buf[:-1]
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < 5:
                break
            length = self.buf[4]
            if length > 240:
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
    try:
        from ybUtils.YbUart import YbUart
        return YbUart(baudrate=UART_BAUD), "YbUart"
    except Exception as e:
        print("YbUart 不可用(%s)，改用 machine.UART" % e)
    from machine import UART, Pin
    dev = UART(UART_UNIT, baudrate=UART_BAUD, tx=Pin(UART_TX),
               rx=Pin(UART_RX), bits=8, parity=None, stop=0)
    return dev, "machine.UART(%d) IO%d/IO%d" % (UART_UNIT, UART_TX, UART_RX)


def main():
    from media.sensor import Sensor, CAM_CHN_ID_0
    from media.display import Display
    from media.media import MediaManager

    uart, uart_name = open_uart()
    print("串口: %s" % uart_name)
    parser = FrameParser()

    sensor = Sensor()
    sensor.reset()
    time.sleep_ms(100)
    sensor.set_framesize(width=IMG_W, height=IMG_H, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

    if DISPLAY_MODE == "LCD":
        Display.init(Display.ST7701, width=DISPLAY_W, height=DISPLAY_H,
                     to_ide=True)
    else:
        Display.init(Display.VIRT, width=DISPLAY_W, height=DISPLAY_H, fps=30)
    MediaManager.init()
    sensor.run()

    roi = (max(0, ROI_CX - ROI_HALF), max(0, ROI_CY - ROI_HALF),
           min(IMG_W, 2 * ROI_HALF), min(IMG_H, 2 * ROI_HALF))

    laser_on = False
    ready = False
    zero_sent = False
    t_hb = time.ticks_ms()
    t_aim = time.ticks_ms()
    t_print = time.ticks_ms()

    print("t3 启动。ROI=%s  LIGHT_LASER=%s" % (str(roi), LIGHT_LASER))
    try:
        while True:
            os.exitpoint()
            now = time.ticks_ms()

            # ---------- 串口：心跳 / 收遥测 ----------
            if time.ticks_diff(now, t_hb) >= 200:
                t_hb = now
                try:
                    uart.write(build_frame(MSG_HEARTBEAT))
                except Exception:
                    pass
            try:
                if uart.any() > 0:
                    for msg_id, seq, payload in parser.feed(uart.read(256)):
                        if msg_id == MSG_GIMBAL_STATE and len(payload) == 21:
                            import ustruct
                            v = ustruct.unpack("<BBhhhhhhhBI", payload)
                            ready = bool(v[9] & 0x10)
            except Exception:
                pass

            # ---------- 激光控制 ----------
            if LIGHT_LASER and ready and not zero_sent:
                try:
                    uart.write(build_frame(MSG_SET_ZERO))
                    time.sleep_ms(20)
                    uart.write(build_frame(MSG_MODE,
                                           bytes([MODE_AIM, 0])))
                    zero_sent = True
                    laser_on = True
                    print("H723 READY -> SET_ZERO + AIM（开激光）")
                except Exception as e:
                    print("发送失败: %s" % e)
            if LIGHT_LASER and laser_on and \
                    time.ticks_diff(now, t_aim) >= 20:
                t_aim = now
                try:
                    import ustruct
                    payload = ustruct.pack("<hhBB", 0, 0, 0x03, 200)
                    uart.write(build_frame(MSG_AIM, payload))
                except Exception:
                    pass

            # ---------- 视觉：ROI 内找光斑 ----------
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            best = None
            for th in THRESHOLDS:
                for b in img.find_blobs([th], roi=roi, merge=True,
                                        pixels_threshold=BLOB_MIN_AREA,
                                        area_threshold=BLOB_MIN_AREA):
                    try:
                        area = b.area()
                    except Exception:
                        area = b[4]
                    try:
                        cx, cy = b.cx(), b.cy()
                    except Exception:
                        cx, cy = b[5], b[6]
                    try:
                        rect = b.rect()
                    except Exception:
                        rect = (b[0], b[1], b[2], b[3])
                    if area < BLOB_MIN_AREA or area > BLOB_MAX_AREA:
                        continue
                    if best is None or area > best[0]:
                        best = (area, cx, cy, rect)

            img.draw_rectangle(roi, color=(255, 255, 255), thickness=1)
            txt = "ROI %d,%d" % (ROI_CX, ROI_CY)
            if best is not None:
                area, cx, cy, rect = best
                img.draw_rectangle(rect, color=(0, 255, 0), thickness=2)
                img.draw_cross(cx, cy, color=(255, 0, 0), size=10,
                               thickness=2)
                txt = "spot=(%d,%d) area=%d" % (cx, cy, area)

            if time.ticks_diff(now, t_print) >= PRINT_MS:
                t_print = now
                if best is None:
                    print("未检到光斑 | ready=%s laser=%s ok=%d crc=%d"
                          % (ready, laser_on, parser.ok, parser.crc_err))
                else:
                    area, cx, cy, rect = best
                    try:
                        rgb = str(img.get_pixel(cx, cy))
                    except Exception as e:
                        rgb = "?%s" % e
                    print("光斑 中心=(%d,%d) 面积=%d RGB=%s | "
                          "ready=%s laser=%s" % (cx, cy, area, rgb,
                                                 ready, laser_on))
            img.draw_string_advanced(4, IMG_H - 26, 20, txt,
                                     color=(255, 220, 0))
            Display.show_image(img)
    except KeyboardInterrupt:
        print("用户停止")
    finally:
        if LIGHT_LASER:
            try:
                uart.write(build_frame(MSG_MODE, bytes([MODE_STAB, 0])))
                time.sleep_ms(50)
                print("已发 STAB，激光应已关闭")
            except Exception:
                pass
        try:
            uart.deinit()
        except Exception:
            pass
        sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()
        print("已退出")


main()
