# -*- coding: utf-8 -*-
"""pc_sim_main.py —— 在电脑上把 main.py 完整跑一遍（不需要 K230）

原理：伪造 K230 的运行环境（machine / media.sensor / image / ustruct），
      再用真实 OpenCV 生成合成场景（黑胶带靶纸 + 激光光斑），
      同时伪造一个 H723：解析收到的帧、按一阶模型转动云台、回遥测。

这样就能在电脑上验证 main.py 的：
  * 状态机（WAIT_READY -> SET_ZERO -> TRACK）
  * 协议收发（心跳/模式/AIM/遥测）
  * 视觉检测 -> 误差 -> 控制 -> 收敛

用法：
    cd D:\\STM32\\STM32projects\\yuntai2
    python k230_aim\\tools\\pc_sim_main.py            # cv2 方案
    python k230_aim\\tools\\pc_sim_main.py rects      # 原生 find_rects 方案
"""
import os
import struct
import sys
import time as real_time
import types

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
MAIN_PY = os.path.join(ROOT, "k230_aim", "main.py")

IMG_W, IMG_H = 640, 480
PX_PER_DEG = 430.0 * 3.14159265 / 180.0     # fx=430px 时，1° ≈ 7.5px

# ----------------------------------------------------------------------
# 虚拟时间
# ----------------------------------------------------------------------
_vtime = [0]


def ticks_ms():
    return _vtime[0]


def ticks_diff(a, b):
    return a - b


def sleep_ms(ms):
    _vtime[0] += int(ms)


real_time.ticks_ms = ticks_ms
real_time.ticks_diff = ticks_diff
real_time.sleep_ms = sleep_ms

_iterations = [0]
MAX_ITERS = 6000


def exitpoint(*a, **kw):
    _iterations[0] += 1
    if _iterations[0] > MAX_ITERS:
        raise KeyboardInterrupt()


os.exitpoint = exitpoint
os.EXITPOINT_ENABLE_SLEEP = 0

sys.modules["ustruct"] = struct

# ----------------------------------------------------------------------
# 合成场景
# ----------------------------------------------------------------------
class Scene(object):
    def __init__(self):
        self.tgt_u = 300.0
        self.tgt_v = 240.0
        self.spot_u = 330.0
        self.spot_v = 250.0

    def render(self):
        img = np.full((IMG_H, IMG_W, 3), 190, np.uint8)
        w, h, t = 200, 330, 20
        x0 = int(self.tgt_u - w / 2)
        y0 = int(self.tgt_v - h / 2)
        cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (25, 25, 30), t)
        cv2.circle(img, (int(self.spot_u), int(self.spot_v)), 5,
                   (250, 190, 250), -1)
        return img


SCENE = Scene()


class FakeFrame(object):
    def __init__(self):
        self.np = SCENE.render()

    def to_numpy_ref(self):
        return self.np

    def draw_line(self, *a, **kw):
        pass

    def draw_circle(self, *a, **kw):
        pass

    def draw_cross(self, *a, **kw):
        pass

    def draw_rectangle(self, *a, **kw):
        pass

    def draw_string_advanced(self, *a, **kw):
        pass

    def find_rects(self, threshold=20000, x_gradient=8, y_gradient=8):
        gray = cv2.cvtColor(self.np, cv2.COLOR_BGR2GRAY)
        bin_img = cv2.adaptiveThreshold(gray, 255,
                                        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                        cv2.THRESH_BINARY_INV, 31, 7)
        contours, _ = cv2.findContours(bin_img, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in contours:
            if cv2.contourArea(c) < 3000:
                continue
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            if len(approx) != 4:
                continue

            class R(object):
                def __init__(self, pts):
                    self._p = pts

                def corners(self):
                    return self._p

                def rect(self):
                    xs = [p[0] for p in self._p]
                    ys = [p[1] for p in self._p]
                    return (min(xs), min(ys), max(xs) - min(xs),
                            max(ys) - min(ys))
            out.append(R([(int(p[0][0]), int(p[0][1])) for p in approx]))
        return out

    def find_blobs(self, thresholds, roi=None, merge=True,
                   pixels_threshold=3, area_threshold=3):
        x, y = SCENE.spot_u, SCENE.spot_v
        if roi is not None:
            rx, ry, rw, rh = roi
            if not (rx <= x <= rx + rw and ry <= y <= ry + rh):
                return []

        class B(object):
            def __init__(self, x, y):
                self.v = (int(x) - 4, int(y) - 4, 9, 9)

            def area(self):
                return 50

            def __getitem__(self, i):
                if i == 4:
                    return 50
                if i == 5:
                    return self.v[0] + 4
                if i == 6:
                    return self.v[1] + 4
                return self.v[i]
        return [B(x, y)]

    def get_pixel(self, x, y):
        p = self.np[int(y), int(x)]
        return (int(p[2]), int(p[1]), int(p[0]))


class FakeSensor(object):
    RGB565 = 0
    RGB888 = 1
    GRAYSCALE = 2

    def __init__(self, *a, **kw):
        self.frames = 0

    def reset(self):
        pass

    def set_framesize(self, **kw):
        pass

    def set_pixformat(self, *a, **kw):
        pass

    def snapshot(self, *a, **kw):
        sleep_ms(30)
        self.frames += 1
        return FakeFrame()

    def run(self):
        pass

    def stop(self):
        pass


class FakeDisplay(object):
    VIRT = 0
    ST7701 = 1

    @staticmethod
    def init(*a, **kw):
        pass

    @staticmethod
    def deinit():
        pass

    @staticmethod
    def show_image(*a, **kw):
        pass


class FakeMediaManager(object):
    @staticmethod
    def init():
        pass

    @staticmethod
    def deinit():
        pass


class FakeImage(object):
    RGB565 = 0
    RGB888 = 1

    def __init__(self, w, h, fmt):
        self.w, self.h, self.fmt = w, h, fmt

    def clear(self):
        pass

    def draw_string_advanced(self, *a, **kw):
        pass


def install_fake_modules():
    machine = types.ModuleType("machine")

    class Pin(object):
        def __init__(self, n):
            self.n = n

    class UART(object):
        def __init__(self, unit, **kw):
            self.unit = unit
            self.rx_buf = bytearray()
            FAKE_H723.bind(self)

        def write(self, data):
            FAKE_H723.feed(bytes(data))

        def any(self):
            return len(self.rx_buf)

        def read(self, n=256):
            if not self.rx_buf:
                return b""
            out = bytes(self.rx_buf[:n])
            del self.rx_buf[:n]
            return out

        def deinit(self):
            pass

    machine.Pin = Pin
    machine.UART = UART
    sys.modules["machine"] = machine

    sensor_mod = types.ModuleType("media.sensor")
    sensor_mod.Sensor = FakeSensor
    sensor_mod.CAM_CHN_ID_0 = 0
    display_mod = types.ModuleType("media.display")
    display_mod.Display = FakeDisplay
    media_mod = types.ModuleType("media.media")
    media_mod.MediaManager = FakeMediaManager
    media_pkg = types.ModuleType("media")
    media_pkg.sensor = sensor_mod
    media_pkg.display = display_mod
    media_pkg.media = media_mod
    sys.modules["media"] = media_pkg
    sys.modules["media.sensor"] = sensor_mod
    sys.modules["media.display"] = display_mod
    sys.modules["media.media"] = media_mod

    image_mod = types.ModuleType("image")
    image_mod.Image = FakeImage
    image_mod.RGB565 = FakeImage.RGB565
    image_mod.RGB888 = FakeImage.RGB888
    sys.modules["image"] = image_mod


# ----------------------------------------------------------------------
# 假 H723
# ----------------------------------------------------------------------
def _crc16(body):
    crc = 0xFFFF
    for byte in body:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


class FakeH723(object):
    def __init__(self):
        self.uart = None
        self.buf = bytearray()
        self.t = 0.0
        self.att_yaw = 0.0
        self.att_pitch = 0.0
        self.cmd_yaw = 0.0
        self.cmd_pitch = 0.0
        self.mode = 0
        self.laser = 0
        self.ready = False
        self.aim_count = 0
        self.ack_count = 0

    def bind(self, uart):
        self.uart = uart

    def feed(self, data):
        self.buf.extend(data)
        while True:
            i = self.buf.find(b"\xAA\x55")
            if i < 0 or len(self.buf) - i < 7:
                break
            if i > 0:
                del self.buf[:i]
            length = self.buf[4]
            total = 5 + length + 2
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:5 + length])
            msg_id = body[0]
            payload = body[3:]
            del self.buf[:total]
            self.on_frame(msg_id, payload)

    def on_frame(self, msg_id, payload):
        if msg_id == 0x10 and len(payload) == 6:
            y, p, flags, q = struct.unpack("<hhBB", payload)
            self.cmd_yaw = y / 100.0
            self.cmd_pitch = p / 100.0
            self.laser = 1 if (flags & 0x01) else 0
            self.aim_count += 1
        elif msg_id == 0x11 and len(payload) >= 1:
            self.mode = payload[0]
            self.send_ack(0x11, self.mode)
        elif msg_id == 0x12:
            self.send_ack(0x12, 1)
        elif msg_id == 0x13:
            self.cmd_yaw = self.cmd_pitch = 0.0
            self.att_yaw = self.att_pitch = 0.0

    def _send(self, msg_id, payload):
        body = struct.pack("<BBB", msg_id, 0, len(payload)) + payload
        self.uart.rx_buf.extend(b"\xAA\x55" + body +
                                struct.pack("<H", _crc16(body)))

    def send_ack(self, msg_id, code):
        self.ack_count += 1
        self._send(0x91, bytes([msg_id, code]))

    def send_telemetry(self):
        gz = struct.pack("<BBhhhhhhhBI", 6, 0,
                         int(self.att_yaw * 100), int(self.att_pitch * 100),
                         0, 0, 0, 0, 0,
                         0x10 if self.ready else 0x00, int(self.t * 1000))
        self._send(0x90, gz)

    def step(self):
        if not self.ready and self.t > 1.0:
            self.ready = True
        dt = 0.02
        tau = 0.12
        if self.mode == 2:
            self.att_yaw += (self.cmd_yaw - self.att_yaw) * dt / tau
            self.att_pitch += (self.cmd_pitch - self.att_pitch) * dt / tau
        SCENE.tgt_u = 300.0 - self.att_yaw * PX_PER_DEG
        SCENE.tgt_v = 240.0 - self.att_pitch * PX_PER_DEG
        self.t += dt
        self.send_telemetry()


FAKE_H723 = None


def run():
    global FAKE_H723
    method = sys.argv[1] if len(sys.argv) > 1 else "cv2"
    FAKE_H723 = FakeH723()
    install_fake_modules()

    src = open(MAIN_PY, "r", encoding="utf-8").read()
    src = src.replace('TARGET_METHOD = "rects"',
                      'TARGET_METHOD = "%s"' % method)
    src = src.replace('DISPLAY_MODE = "VIRT"', 'DISPLAY_MODE = "OFF"')

    fake_t = [0.0]

    def exitpoint2(*a, **kw):
        now = _vtime[0] / 1000.0
        while fake_t[0] < now:
            FAKE_H723.step()
            fake_t[0] += 0.02
        exitpoint(*a, **kw)

    os.exitpoint = exitpoint2

    print("=" * 66)
    print("PC 仿真：TARGET_METHOD=%s，最多 %d 个循环" % (method, MAX_ITERS))
    print("=" * 66)
    try:
        exec(compile(src, MAIN_PY, "exec"), {"__name__": "__main__"})
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass

    ex = SCENE.tgt_u - SCENE.spot_u
    ey = SCENE.tgt_v - SCENE.spot_v
    err = (ex * ex + ey * ey) ** 0.5
    print("-" * 66)
    print("仿真结束：虚拟时间 %.2fs" % (_vtime[0] / 1000.0))
    print("AIM 帧 %d 条，ACK %d 条" % (FAKE_H723.aim_count,
                                       FAKE_H723.ack_count))
    print("末端偏置 yaw=%.2f° pitch=%.2f° ，云台姿态 yaw=%.2f°"
          % (FAKE_H723.cmd_yaw, FAKE_H723.cmd_pitch, FAKE_H723.att_yaw))
    print("末端残差 %.2f px（%.4f°）" % (err, err / PX_PER_DEG))
    print("结果：%s" % ("闭环收敛 OK" if err < 12 else "未收敛，需要检查"))
    return 0 if err < 12 else 1


if __name__ == "__main__":
    sys.exit(run())
