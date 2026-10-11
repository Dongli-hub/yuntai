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
    python k230_aim\\tools\\pc_sim_main.py

（场景尺寸、死区、K 都从 main.py 里读/跟着算，所以改了 PROC_W/PROC_H 之后
  这里不用动 —— 仿真跑通了就说明那两个分辨率下 main.py 的链路是自洽的。）
"""
import io
import os
import re
import struct
import sys
import time as real_time
import types

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
MAIN_PY = os.path.join(ROOT, "k230_aim", "main.py")


def _read_proc_size():
    """从 main.py 里读 PROC_W/PROC_H —— 仿真场景必须和真实处理分辨率一致，
    否则 main.py 里那套"按 S_SCALE 缩放"的判据会全部对不上（手动同步迟早会忘）。"""
    txt = io.open(MAIN_PY, encoding="utf-8").read()
    w = int(re.search(r"^PROC_W\s*=\s*(\d+)", txt, re.M).group(1))
    h = int(re.search(r"^PROC_H\s*=\s*(\d+)", txt, re.M).group(1))
    return w, h


IMG_W, IMG_H = _read_proc_size()
S_SCALE = IMG_W / 640.0
# fx=430px(640 档) 时 1° ≈ 7.5px；分辨率一换，px/° 同比缩放（和 main.py 的
# VIS_K = 7.8*S 是同一个道理，只是这里用的焦距估值略小）
PX_PER_DEG = 430.0 * S_SCALE * 3.14159265 / 180.0

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
        # 基准点都按 S_SCALE 缩放：640x480 时就是 (300,240)/(312,252)
        self.tgt_u = 300.0 * S_SCALE
        self.tgt_v = 240.0 * S_SCALE
        # 与 main.py 的 USE_FIXED_LASER / LASER_UV 保持一致（激光在画面里的固定位置）
        self.spot_u = 312.0 * S_SCALE
        self.spot_v = 252.0 * S_SCALE

    def render(self):
        img = np.full((IMG_H, IMG_W, 3), 190, np.uint8)
        w = int(200 * S_SCALE)
        h = int(330 * S_SCALE)
        t = int(round(20 * S_SCALE))
        x0 = int(self.tgt_u - w / 2)
        y0 = int(self.tgt_v - h / 2)
        cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (25, 25, 30), t)
        # 激光点：红边要够红（a>=25、b>=-20）才能过 SPOT_THRESHOLDS 第二档。
        # 2026-10-11：原来的 (250,190,250) 算出来 b=-21，正好差 1 检不出来。
        cv2.circle(img, (int(self.spot_u), int(self.spot_v)),
                   int(round(5 * S_SCALE)),
                   (120, 120, 255), -1)
        return img


SCENE = Scene()


class FakeBlob(object):
    """复现 CanMV blob 的下标访问：b[0..3]=x,y,w,h  b[4]=面积  b[5/6]=中心"""

    def __init__(self, x, y, w, h, area):
        self._v = (x, y, w, h, area, x + w // 2, y + h // 2)

    def __getitem__(self, i):
        return self._v[i]


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

    def find_blobs(self, thresholds, roi=None, merge=True, margin=4,
                   pixels_threshold=3, area_threshold=3):
        """仿真版 find_blobs：在合成场景上做真实 LAB 阈值分割。
        2026-10-11：原来这里是个只认光斑的占位桩，导致仿真里"靶纸检测"
        永远命中 0；现在按 CanMV 语义实现（阈值 -> ROI -> margin 合并 ->
        连通域），PC 仿真才能真正跑通"视觉 -> 上报 -> 外环"整条链。"""
        lab = cv2.cvtColor(self.np, cv2.COLOR_BGR2LAB)
        L = lab[:, :, 0].astype(np.float32) / 2.55      # 0~100（同 CanMV）
        A = lab[:, :, 1].astype(np.int16) - 128
        B = lab[:, :, 2].astype(np.int16) - 128
        mask = np.zeros(L.shape, np.uint8)
        for (l0, l1, a0, a1, b0, b1) in thresholds:
            m = ((L >= l0) & (L <= l1) & (A >= a0) & (A <= a1) &
                 (B >= b0) & (B <= b1))
            mask[m] = 255
        if roi is not None:
            rx, ry, rw, rh = roi
            keep = np.zeros_like(mask)
            y1 = max(0, min(IMG_H, ry + rh))
            x1 = max(0, min(IMG_W, rx + rw))
            keep[max(0, ry):y1, max(0, rx):x1] = \
                mask[max(0, ry):y1, max(0, rx):x1]
            mask = keep
        if merge and (margin > 0):
            k = np.ones((2 * margin + 1, 2 * margin + 1), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        n, _labels, stats, _cents = cv2.connectedComponentsWithStats(mask, 4)
        out = []
        for i in range(1, n):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < max(int(pixels_threshold), int(area_threshold)):
                continue
            out.append(FakeBlob(int(stats[i, cv2.CC_STAT_LEFT]),
                                int(stats[i, cv2.CC_STAT_TOP]),
                                int(stats[i, cv2.CC_STAT_WIDTH]),
                                int(stats[i, cv2.CC_STAT_HEIGHT]), area))
        return out

    def get_histogram(self, *a, **kw):
        """仿真版直方图：用 OpenCV 的 Otsu 阈值冒充 CanMV 的
        img.get_histogram().get_threshold().value()，让 PC 仿真也能走
        main.py 新的 Otsu 阈值路径。"""
        gray = cv2.cvtColor(self.np, cv2.COLOR_BGR2GRAY)
        v, _ = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        class _P(object):
            def __init__(self, val):
                self._v = val

            def value(self):
                return self._v

        class _H(object):
            def __init__(self, val):
                self._v = val

            def get_threshold(self):
                return _P(self._v)

        return _H(int(v))

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
        # 视觉外环状态（镜像 H723 固件 vis_outer_step 的数学，2026-10-11 新架构）
        self.px_ut = 0
        self.px_vt = 0
        self.px_us = 0
        self.px_vs = 0
        self.aim_flags = 0
        self.vis_k_x100 = 0            # AIM 尾部带的 K×100（0 = 没带）
        self.px_ms = -1e9
        self.vis_eu = 0.0
        self.vis_ev = 0.0
        self.vis_seen = False
        self.vis_last_ms = -1e9

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
        if msg_id == 0x10 and len(payload) in (10, 12):
            # 新架构（2026-10-11）：ut vt us vs (u16×4 小端) + flags + quality
            # 12 字节版：尾部再带 kx100 = 像素/度 ×100（分辨率可切换）
            (ut, vt, us, vs, flags, _q) = struct.unpack("<HHHHBB", payload[:10])
            self.px_ut, self.px_vt = ut, vt
            self.px_us, self.px_vs = us, vs
            self.aim_flags = flags
            if len(payload) >= 12:
                self.vis_k_x100 = struct.unpack("<h", payload[10:12])[0]
            self.px_ms = self.t * 1000.0
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
            self.vis_eu = self.vis_ev = 0.0
            self.vis_seen = False

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
        # ---- 视觉外环（镜像 H723 固件 vis_outer_step）：像素误差 -> 偏置 ----
        vid = (self.aim_flags & 0x02) != 0          # AIM_VALID
        now_ms = self.t * 1000.0
        if vid and (self.px_ms - self.vis_last_ms) > 1.0:
            self.vis_last_ms = self.px_ms
            eu = self.px_ut - self.px_us
            ev = self.px_vt - self.px_vs
            db = 3.0 * S_SCALE      # 死区也随分辨率缩放（同 H723 的 ks 做法）
            if abs(eu) < db:
                eu = 0.0
            if abs(ev) < db:
                ev = 0.0
            if not self.vis_seen:
                self.vis_eu, self.vis_ev, self.vis_seen = eu, ev, True
            else:
                self.vis_eu += 0.7 * (eu - self.vis_eu)
                self.vis_ev += 0.7 * (ev - self.vis_ev)
        if (self.mode == 2) and vid and \
                ((now_ms - self.vis_last_ms) <= 150.0):
            # K：优先用 AIM 帧带过来的（4~16 之间才算数），否则默认 7.8
            k = self.vis_k_x100 / 100.0
            if (k < 4.0) or (k > 16.0):
                k = 7.8
            ru = -4.0 * (self.vis_eu / k)            # VIS_SIGN*VIS_KP/K
            rv = -3.0 * (self.vis_ev / k)
            ru = max(-120.0, min(120.0, ru))
            rv = max(-60.0, min(60.0, rv))
            self.cmd_yaw = max(-120.0, min(120.0, self.cmd_yaw + ru * dt))
            self.cmd_pitch = max(-20.0, min(20.0, self.cmd_pitch + rv * dt))
        tau = 0.12
        if self.mode == 2:
            self.att_yaw += (self.cmd_yaw - self.att_yaw) * dt / tau
            self.att_pitch += (self.cmd_pitch - self.att_pitch) * dt / tau
        # 符号约定：偏置增大（att 跟着增大）-> 目标在画面里向正方向移。
        # 只有这样 SIGN=-1 的外环才是负反馈（与现场收敛方向一致）。
        SCENE.tgt_u = 300.0 + self.att_yaw * PX_PER_DEG
        SCENE.tgt_v = 240.0 + self.att_pitch * PX_PER_DEG
        self.t += dt
        self.send_telemetry()


FAKE_H723 = None


def run():
    global FAKE_H723
    FAKE_H723 = FakeH723()
    install_fake_modules()

    src = open(MAIN_PY, "r", encoding="utf-8").read()
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
    print("PC 仿真：%dx%d，最多 %d 个循环" % (IMG_W, IMG_H, MAX_ITERS))
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
