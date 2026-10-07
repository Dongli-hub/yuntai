# -*- coding: utf-8 -*-
"""main.py —— K230 机载计算机瞄准程序（替代地瓜派上的 rdk_aim）

整体上电即自动运行：把本文件保存到 K230 SD 卡根目录、命名 main.py 即可。

它做的事（和地瓜派版一一对应）：
   相机取图 -> 找黑胶带框(靶纸) -> 找激光光斑
        -> 像素误差 -> 角度偏置 -> 串口发给 H723 -> 云台转过去

和 H723 的约定（协议不变）：
   帧: AA 55 | msg_id | seq | len | payload | crc16(小端)
   0x12 心跳 / 0x13 SET_ZERO / 0x11 MODE(0IDLE 1STAB 2AIM) / 0x10 AIM(偏置)
   0x90 遥测 / 0x91 ACK / 0x93 调试文本
   H723 端: 0.5s 收不到 AIM 自动切 STAB 并关激光（安全看门狗）

引脚（K230 EXPORT 排针 -> H723 UART7）：
   IO9  (TXD) -> H723 UART7 RX
   IO10 (RXD) -> H723 UART7 TX
   GND        -> GND        （必须共地）
   5V         -> 5V         （整机由 H723 侧供电时用）
   如果接的是 12Pin GPIO 的 UART3，把 UART_UNIT/TX/RX 改成 3/32/33。

调参都在下面「配置区」。改完直接重新运行即可。
"""
import gc
import math
import os
import time

# ============================================================================
#  配置区
# ============================================================================

# --- 串口引脚 ---
UART_BACKEND = "auto"      # auto(=先 machine.UART, 失败退 YbUart) | machine | yb
UART_UNIT = 1
UART_TX = 9
UART_RX = 10
UART_BAUD = 115200

# --- 显示 ---
DISPLAY_MODE = "VIRT"      # VIRT(只用 IDE 画面) | LCD(接了屏) | OFF
DISPLAY_W = 640
DISPLAY_H = 480

# --- 相机 ---
IMG_W = 640
IMG_H = 480
TARGET_METHOD = "rects"    # rects(原生 find_rects, RGB565) | cv2(OpenCV, RGB888)

# --- 靶纸（黑胶带框）检测 ---
RECT_THRESHOLD = 20000     # find_rects 阈值，检不到就调小
RECT_XGRAD = 8
RECT_YGRAD = 8
ADAPT_BLOCK = 31           # cv2 自适应阈值窗口（奇数）
ADAPT_C = 7
POLY_EPS = 0.02            # approxPolyDP 精度（周长比例）
MIN_AREA_RATIO = 0.02      # 框面积 / 画面面积 下限
ASPECT_MIN = 1.20          # 长边/短边（180x297 胶带框 = 1.65）
ASPECT_MAX = 2.30
PAPER_LONG_M = 0.297       # 胶带框长边实际长度（米），用于估距离

# --- 激光光斑检测 ---
SPOT_ROI_HALF = 130        # 只在画面中心这块 ±130px 里找光斑（同轴 = 位置固定）
SPOT_THRESHOLDS = [        # LAB 阈值，可多组
    (60, 100, 8, 60, -10, 60),     # 亮且偏暖（红激光边缘）
    (85, 100, -20, 40, -20, 60),   # 过曝白芯
]
SPOT_MIN_AREA = 3
SPOT_MAX_AREA = 3000
SPOT_MAX_ASPECT = 3.0
SPOT_ADAPT_GAIN = 0.06     # 光轴点慢速自适应（把误检拖跑的风险限制住）
SPOT_ADAPT_LIMIT = 6.0     # 单次最多修正多少像素

# --- 瞄准控制（积分型视觉伺服 + P 项挂在实测姿态上）---
FX_PX = 430.0              # 像素焦距（640 宽）。见文件末尾校准说明
SIGN_YAW = 1.0             # 偏置方向；若越转越远就把对应项改 -1
SIGN_PITCH = 1.0
KP_P_YAW = 0.75            # P 项：一次给出 75% 的偏差角度
KP_P_PITCH = 0.50
KI = 0.015                 # I 项：只抹静差
KI_BAND_PX = 40.0          # 误差小于这么多像素才允许积分
I_LIMIT_DEG = 3.0          # 积分限幅
DEADBAND_PX = 2.0          # 误差死区（640x480 下 1px≈3.5mm@1.5m；1280x720 用 6）
RATE_LIMIT_DPS = 8.0       # 偏置变化率限幅（防甩）
MAX_YAW_DEG = 170.0
MAX_PITCH_DEG = 60.0
LOST_COAST_S = 1.0         # 丢靶后还能沿用最后误差多久

# --- 时序 / 安全 ---
AIM_HZ = 50                # 给 H723 发 AIM 的频率
HEARTBEAT_MS = 200
MODE_REASSERT_MS = 2000    # 周期性重发 MODE（H723 可能被看门狗切回 STAB）
TELEM_TIMEOUT_MS = 1500    # 遥测断这么久 -> 主动发 STAB
LASER_ENABLE = True        # 是否让 H723 点激光（AIM 模式下才有效）
DEBUG_PRINT_MS = 1000      # 终端打印周期

# ============================================================================
#  协议
# ============================================================================
SOF = b"\xAA\x55"

MSG_AIM = 0x10
MSG_MODE = 0x11
MSG_HEARTBEAT = 0x12
MSG_SET_ZERO = 0x13
MSG_GIMBAL_STATE = 0x90
MSG_ACK = 0x91
MSG_TEXT = 0x93

MODE_IDLE = 0
MODE_STAB = 1
MODE_AIM = 2

FLAG_LASER_ON = 0x01
FLAG_AIM_VALID = 0x02
FLAG_LOCKED = 0x10

ST_READY = 0x10
ST_LASER_ON = 0x08


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


def pack_aim(yaw_deg, pitch_deg, flags, quality):
    import ustruct
    y = max(-32768, min(32767, int(round(yaw_deg * 100.0))))
    p = max(-32768, min(32767, int(round(pitch_deg * 100.0))))
    return ustruct.pack("<hhBB", y, p, flags & 0xFF,
                        max(0, min(255, int(quality))))


def pack_mode(mode, arg=0):
    import ustruct
    return ustruct.pack("<BB", mode & 0xFF, arg & 0xFF)


class FrameParser(object):
    def __init__(self):
        self.buf = bytearray()
        self.ok = 0
        self.crc_err = 0
        self.bad_len = 0
        self.resync = 0

    def feed(self, data):
        out = []
        self.buf.extend(data)
        while True:
            i = self.buf.find(SOF)
            if i < 0:
                if len(self.buf) > 1:
                    self.resync += 1
                    # MicroPython 的 bytearray 不支持切片删除（del buf[x:]），
                    # 必须用重新切片代替，否则报
                    # "TypeError: 'bytearray' object doesn't support item deletion"
                    self.buf = self.buf[-1:]
                break
            if i > 0:
                self.resync += 1
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
            self.ok += 1
            out.append((body[0], body[1], bytes(body[3:])))
            self.buf = self.buf[total:]
        return out


_GIMBAL_FMT = "<BBhhhhhhhBI"


def unpack_state(payload):
    """-> dict or None"""
    import ustruct
    if len(payload) != 21:
        return None
    v = ustruct.unpack(_GIMBAL_FMT, payload)
    return {
        "state": v[0], "fault": v[1],
        "yaw": v[2] / 100.0, "pitch": v[3] / 100.0, "roll": v[4] / 100.0,
        "ymotor": v[5] / 100.0, "pmotor": v[6] / 100.0,
        "gyro_y": v[7] / 100.0, "gyro_z": v[8] / 100.0,
        "flags": v[9], "up_ms": v[10],
    }


# ============================================================================
#  串口链路
# ============================================================================
class Link(object):
    """串口后端自动选择。

    实测（CanMV v1.4.3 / k230_canmv_yahboom 固件）：
      · machine.UART(1, tx=Pin(9), rx=Pin(10)) 打不开 ——
        固件启动时已经把 IO9/IO10 分配给了它自己的功能（报
        "pin(9) is not a GPIO pin"），所以这条要放在后面当备选。
      · ybUtils.YbUart 是可用的，它在固件里占的就是 IO9(TXD)/IO10(RXD)。
    按顺序试，哪个先打开用哪个。
    """

    @staticmethod
    def _candidates():
        def yb():
            from ybUtils.YbUart import YbUart
            return YbUart(baudrate=UART_BAUD)

        def m_nopin():
            from machine import UART
            return UART(UART_UNIT, baudrate=UART_BAUD, bits=8,
                        parity=None, stop=0)

        def m_pins19():
            from machine import UART, Pin
            return UART(1, baudrate=UART_BAUD, tx=Pin(9), rx=Pin(10),
                        bits=8, parity=None, stop=0)

        def m_pins3233():
            from machine import UART, Pin
            return UART(3, baudrate=UART_BAUD, tx=Pin(32), rx=Pin(33),
                        bits=8, parity=None, stop=0)

        def uart3_fpioa():
            # 备用线路：12Pin GPIO 的 IO32(TXD)/IO33(RXD) 手动配成 UART3。
            # 和 YbUart 配 IO9/IO10 是同一套做法（先 FPIOA 再建 UART）。
            from machine import FPIOA, UART
            fp = FPIOA()
            fp.set_function(32, FPIOA.UART3_TXD, ie=0, oe=1, pu=1)
            fp.set_function(33, FPIOA.UART3_RXD, ie=1, oe=0, pu=1)
            return UART(3, baudrate=UART_BAUD)

        def yb_swapped():
            # IO9/IO10 反着用（IO10=TXD, IO9=RXD）：
            # 两根数据线如果接反了，不用拆线也能通信。
            from machine import FPIOA, UART
            fp = FPIOA()
            fp.set_function(10, FPIOA.UART1_TXD, ie=0, oe=1, pu=1)
            fp.set_function(9, FPIOA.UART1_RXD, ie=1, oe=0, pu=1)
            return UART(1, baudrate=UART_BAUD)

        return [("YbUart(亚博封装, IO9/IO10)", yb),
                ("UART1 IO9/IO10 反接", yb_swapped),
                ("UART3 手配 IO32/IO33", uart3_fpioa),
                ("UART(%d) 不指定引脚" % UART_UNIT, m_nopin),
                ("UART(1) tx=IO9 rx=IO10", m_pins19),
                ("UART(3) tx=IO32 rx=IO33", m_pins3233)]

    def __init__(self):
        self.dev = None
        self.name = ""
        for label, opener in self._candidates():
            try:
                self.dev = opener()
                self.name = label
                break
            except Exception as e:
                print("串口后端 %s 打不开: %s" % (label, e))
        if self.dev is None:
            raise OSError("没有任何串口后端能打开")
        print("串口: %s @%d" % (self.name, UART_BAUD))

    def send(self, data):
        try:
            self.dev.write(data)
        except Exception as e:
            print("串口写失败: %s" % e)

    def read(self, n=256):
        try:
            if hasattr(self.dev, "any") and self.dev.any() <= 0:
                return b""
            return self.dev.read(n) or b""
        except Exception:
            return b""

    def close(self):
        try:
            self.dev.deinit()
        except Exception:
            pass


# ============================================================================
#  视觉：靶纸（黑胶带框）检测
# ============================================================================
def order_corners(pts):
    cx = sum(p[0] for p in pts) / 4.0
    cy = sum(p[1] for p in pts) / 4.0
    try:
        ang = [(math.atan2(p[1] - cy, p[0] - cx), p) for p in pts]
        ang.sort(key=lambda t: t[0])
        return [list(t[1]) for t in ang]
    except AttributeError:
        # MicroPython 的 math 可能是裁剪版（例如没有 atan2）：
        # 退化成"按 x 分左右、再按 y 分上下"。
        s = sorted(pts, key=lambda p: p[0])
        left = sorted(s[:2], key=lambda p: p[1])
        right = sorted(s[2:], key=lambda p: p[1])
        return [list(left[0]), list(right[0]), list(right[1]), list(left[1])]


def quad_center_area_aspect(q):
    (x0, y0), (x1, y1), (x2, y2), (x3, y3) = q
    d1x, d1y = x2 - x0, y2 - y0
    d2x, d2y = x3 - x1, y3 - y1
    den = d1x * d2y - d1y * d2x
    if abs(den) < 1e-6:
        cx, cy = (x0 + x2) / 2.0, (y0 + y2) / 2.0
    else:
        t = ((x1 - x0) * d2y - (y1 - y0) * d2x) / den
        cx, cy = x0 + t * d1x, y0 + t * d1y
    area = 0.5 * abs((x0 * y1 - x1 * y0) + (x1 * y2 - x2 * y1) +
                     (x2 * y3 - x3 * y2) + (x3 * y0 - x0 * y3))
    sides = [math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2),
             math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2),
             math.sqrt((x3 - x2) ** 2 + (y3 - y2) ** 2),
             math.sqrt((x0 - x3) ** 2 + (y0 - y3) ** 2)]
    a_len = (sides[0] + sides[2]) / 2.0
    b_len = (sides[1] + sides[3]) / 2.0
    long_side = max(a_len, b_len)
    short_side = max(1.0, min(a_len, b_len))
    return (cx, cy), area, long_side / short_side, long_side


class TargetDetector(object):
    """两种实现：rects(原生) / cv2(OpenCV 移植版)。"""

    def __init__(self, method):
        self.method = method
        self.kernel = None
        self.err = 0
        self.err_msg = ""
        self.last_n = 0
        if method == "cv2":
            import cv2
            self.cv2 = cv2
            self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    def detect(self, img, img_area):
        try:
            if self.method == "cv2":
                return self._detect_cv2(img, img_area)
            return self._detect_rects(img, img_area)
        except Exception as e:
            self.err += 1
            if self.err_msg != str(e):
                self.err_msg = str(e)
                print("检测异常: %s" % e)
            return None

    # ---------------------------------------------------------------
    def _detect_rects(self, img, img_area):
        best = None
        n = 0
        for r in img.find_rects(threshold=RECT_THRESHOLD,
                                x_gradient=RECT_XGRAD,
                                y_gradient=RECT_YGRAD):
            n += 1
            pts = [(int(p[0]), int(p[1])) for p in r.corners()]
            q = order_corners(pts)
            center, area, aspect, long_side = quad_center_area_aspect(q)
            if area < MIN_AREA_RATIO * img_area:
                continue
            if aspect < ASPECT_MIN or aspect > ASPECT_MAX:
                continue
            if best is None or area > best[1]:
                best = (q, area, center, aspect, long_side)
        self.last_n = n
        if best is None:
            return None
        q, area, center, aspect, long_side = best
        return {"quad": q, "center": center, "area": area, "aspect": aspect,
                "long_side": long_side, "n": n}

    # ---------------------------------------------------------------
    def _detect_cv2(self, img, img_area):
        cv2 = self.cv2
        img_np = img.to_numpy_ref()
        gray = cv2.cvtColor(img_np, cv2.COLOR_BGR2GRAY)
        bin_img = cv2.adaptiveThreshold(gray, 255,
                                        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                        cv2.THRESH_BINARY_INV,
                                        ADAPT_BLOCK, ADAPT_C)
        closed = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, self.kernel)
        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        best = None
        n = 0
        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_AREA_RATIO * img_area:
                continue
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, POLY_EPS * peri, True)
            if len(approx) != 4:
                continue
            n += 1
            pts = [(int(p[0][0]), int(p[0][1])) for p in approx]
            q = order_corners(pts)
            center, qarea, aspect, long_side = quad_center_area_aspect(q)
            if aspect < ASPECT_MIN or aspect > ASPECT_MAX:
                continue
            if best is None or qarea > best[1]:
                best = (q, qarea, center, aspect, long_side)
        self.last_n = n
        if best is None:
            return None
        q, area, center, aspect, long_side = best
        return {"quad": q, "center": center, "area": area, "aspect": aspect,
                "long_side": long_side, "n": n}


# ============================================================================
#  视觉：激光光斑
# ============================================================================
class SpotDetector(object):
    def __init__(self, method):
        self.method = method
        self.u = IMG_W / 2.0
        self.v = IMG_H / 2.0
        self.learned = False
        self.err = 0
        if method == "cv2":
            import cv2
            self.cv2 = cv2
            self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    def roi(self):
        x0 = int(max(0, self.u - SPOT_ROI_HALF))
        y0 = int(max(0, self.v - SPOT_ROI_HALF))
        x1 = int(min(IMG_W, self.u + SPOT_ROI_HALF))
        y1 = int(min(IMG_H, self.v + SPOT_ROI_HALF))
        if x1 - x0 < 20 or y1 - y0 < 20:
            x0, y0 = max(0, IMG_W // 2 - SPOT_ROI_HALF), \
                max(0, IMG_H // 2 - SPOT_ROI_HALF)
            x1, y1 = min(IMG_W, x0 + 2 * SPOT_ROI_HALF), \
                min(IMG_H, y0 + 2 * SPOT_ROI_HALF)
        return (x0, y0, x1 - x0, y1 - y0)

    def detect(self, img):
        try:
            if self.method == "cv2":
                return self._detect_cv2(img)
            return self._detect_blobs(img)
        except Exception as e:
            self.err += 1
            print("光斑检测异常: %s" % e)
            return None

    def _adapt(self, u, v):
        if not self.learned:
            self.u, self.v = u, v
            self.learned = True
            return
        du = max(-SPOT_ADAPT_LIMIT,
                 min(SPOT_ADAPT_LIMIT, (u - self.u) * SPOT_ADAPT_GAIN))
        dv = max(-SPOT_ADAPT_LIMIT,
                 min(SPOT_ADAPT_LIMIT, (v - self.v) * SPOT_ADAPT_GAIN))
        self.u += du
        self.v += dv

    # ---- 原生 find_blobs 版（RGB565）----
    def _detect_blobs(self, img):
        x0, y0, w, h = self.roi()
        best = None
        for th in SPOT_THRESHOLDS:
            for b in img.find_blobs([th], roi=(x0, y0, w, h), merge=True,
                                    pixels_threshold=SPOT_MIN_AREA,
                                    area_threshold=SPOT_MIN_AREA):
                try:
                    area = b.area()
                except Exception:
                    area = b[4]
                if area < SPOT_MIN_AREA or area > SPOT_MAX_AREA:
                    continue
                bw = b[2]
                bh = b[3]
                if bw < 1 or bh < 1:
                    continue
                if max(bw, bh) * 1.0 / min(bw, bh) > SPOT_MAX_ASPECT:
                    continue
                cx = b[5]
                cy = b[6]
                if best is None or area > best[0]:
                    best = (area, cx, cy, (b[0], b[1], bw, bh))
        if best is None:
            return None
        area, cx, cy, rect = best
        self._adapt(cx, cy)
        return {"uv": (cx, cy), "area": area, "rect": rect}

    # ---- OpenCV 版（RGB888）：暖色优势 + 亮度 ----
    def _detect_cv2(self, img):
        cv2 = self.cv2
        img_np = img.to_numpy_ref()
        x0, y0, w, h = self.roi()
        # 只取 ROI（ulab 切片）
        sub = img_np[y0:y0 + h, x0:x0 + w]
        r = sub[:, :, 0]
        g = sub[:, :, 1]
        b = sub[:, :, 2]
        # 暖色优势 = min(R-G, B-G) >= gap  （用两个饱和减法 + 与运算等价实现）
        rg = cv2.subtract(r, g)
        bg = cv2.subtract(b, g)
        m1 = cv2.threshold(rg, 6, 255, cv2.THRESH_BINARY)[1]
        m2 = cv2.threshold(bg, 6, 255, cv2.THRESH_BINARY)[1]
        # 亮度下限（三个通道都要够亮）
        m3 = cv2.threshold(g, 110, 255, cv2.THRESH_BINARY)[1]
        m4 = cv2.threshold(r, 110, 255, cv2.THRESH_BINARY)[1]
        m5 = cv2.threshold(b, 110, 255, cv2.THRESH_BINARY)[1]
        mask = cv2.bitwise_and(cv2.bitwise_and(cv2.bitwise_and(m1, m2), m3),
                               cv2.bitwise_and(m4, m5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for c in contours:
            area = cv2.contourArea(c)
            if area < SPOT_MIN_AREA or area > SPOT_MAX_AREA:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            if bw < 1 or bh < 1:
                continue
            if max(bw, bh) * 1.0 / min(bw, bh) > SPOT_MAX_ASPECT:
                continue
            if best is None or area > best[0]:
                best = (area, x, y, bw, bh)
        if best is None:
            return None
        area, x, y, bw, bh = best
        cx = x0 + x + bw / 2.0
        cy = y0 + y + bh / 2.0
        self._adapt(cx, cy)
        return {"uv": (cx, cy), "area": area, "rect": (x0 + x, y0 + y, bw, bh)}


# ============================================================================
#  瞄准控制
# ============================================================================
class AimCtrl(object):
    def __init__(self):
        self.yaw = 0.0
        self.pitch = 0.0
        self.i_u = 0.0
        self.i_v = 0.0
        self.err_px = 0.0
        self.valid = False
        self.locked = False

    def reset(self):
        self.i_u = 0.0
        self.i_v = 0.0

    def update(self, dt, err_u, err_v, att_rel):
        """err_u/err_v = 靶心 - 光斑（像素）；att_rel = 相对锁零基准的实测姿态"""
        self.err_px = math.sqrt(err_u * err_u + err_v * err_v)
        eu, ev = err_u, err_v
        if abs(eu) < DEADBAND_PX:
            eu = 0.0
        if abs(ev) < DEADBAND_PX:
            ev = 0.0

        # I 项用"没进死区前"的原始误差：死区只用来抑制 P 项抖动，
        # 不能让积分也停住，否则最后几像素的静差永远消不掉。
        e_deg_u_raw = err_u / FX_PX * 57.29578
        e_deg_v_raw = err_v / FX_PX * 57.29578
        e_deg_u = eu / FX_PX * 57.29578
        e_deg_v = ev / FX_PX * 57.29578

        # 条件积分：误差小才累加，避免"攒过头再冲出去"
        if self.err_px <= KI_BAND_PX and dt > 1e-4:
            self.i_u += KI * e_deg_u_raw
            self.i_v += KI * e_deg_v_raw
            self.i_u = max(-I_LIMIT_DEG, min(I_LIMIT_DEG, self.i_u))
            self.i_v = max(-I_LIMIT_DEG, min(I_LIMIT_DEG, self.i_v))

        if att_rel is not None:
            base_u = att_rel[0] + self.i_u
            base_v = att_rel[1] + self.i_v
        else:
            base_u = self.yaw + self.i_u
            base_v = self.pitch + self.i_v

        want_u = base_u + SIGN_YAW * KP_P_YAW * e_deg_u
        want_v = base_v + SIGN_PITCH * KP_P_PITCH * e_deg_v

        # 变化率限幅（防甩） + 绝对限幅
        step = RATE_LIMIT_DPS * max(dt, 1e-3)
        du = want_u - self.yaw
        dv = want_v - self.pitch
        if du > step:
            du = step
        elif du < -step:
            du = -step
        if dv > step:
            dv = step
        elif dv < -step:
            dv = -step
        self.yaw = max(-MAX_YAW_DEG, min(MAX_YAW_DEG, self.yaw + du))
        self.pitch = max(-MAX_PITCH_DEG, min(MAX_PITCH_DEG, self.pitch + dv))
        self.locked = self.err_px < DEADBAND_PX


# ============================================================================
#  显示 / 相机
# ============================================================================
def init_camera():
    from media.sensor import Sensor, CAM_CHN_ID_0
    sensor = Sensor()
    sensor.reset()
    time.sleep_ms(100)
    sensor.set_framesize(width=IMG_W, height=IMG_H, chn=CAM_CHN_ID_0)
    if TARGET_METHOD == "cv2":
        sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)
    else:
        sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
    return sensor


def print_fpioa_state():
    """启动时打印关键引脚功能，方便一眼确认串口引脚配对了。"""
    try:
        from machine import FPIOA
        fp = FPIOA()
        out = []
        for pin in (9, 10, 32, 33):
            try:
                out.append("IO%d=%s" % (pin, fp.get_pin_func(pin)))
            except Exception:
                out.append("IO%d=?" % pin)
        print("引脚功能: %s" % " ".join(out))
    except Exception:
        pass


_display = None
_media = None


def init_display():
    """返回画布 Image 或 None。没有屏幕时只用 IDE 画面视图（VIRT）。"""
    global _display, _media
    if DISPLAY_MODE == "OFF":
        return None
    try:
        from media.display import Display
        from media.media import MediaManager
        import image
        if DISPLAY_MODE == "LCD":
            Display.init(Display.ST7701, width=DISPLAY_W, height=DISPLAY_H,
                         to_ide=True)
        else:
            Display.init(Display.VIRT, width=DISPLAY_W, height=DISPLAY_H,
                         fps=30)
        _display = Display
        _media = MediaManager
        return image.Image(DISPLAY_W, DISPLAY_H, image.RGB565)
    except Exception as e:
        print("显示不可用(%s)，继续无显示运行" % e)
        _display = None
        _media = None
        return None


# ============================================================================
#  状态机
# ============================================================================
ST_WAIT_READY = 0
ST_SET_ZERO = 1
ST_TRACK = 2
ST_NAME = {ST_WAIT_READY: "WAIT_READY", ST_SET_ZERO: "SET_ZERO",
           ST_TRACK: "TRACK"}


def main():
    link = Link()
    parser = FrameParser()
    print_fpioa_state()
    sensor = init_camera()
    canvas = init_display()
    from media.media import MediaManager
    MediaManager.init()
    sensor.run()

    target_det = TargetDetector(TARGET_METHOD)
    spot_det = SpotDetector(TARGET_METHOD)
    ctrl = AimCtrl()

    state = ST_WAIT_READY
    state_t = time.ticks_ms()
    att_zero = None
    gz = None
    last_gz_t = 0
    t_hb = time.ticks_ms()
    t_aim = time.ticks_ms()
    t_mode = time.ticks_ms()
    t_print = time.ticks_ms()
    t_last_frame = time.ticks_ms()
    last_tgt = None
    last_tgt_t = 0
    n_frames = 0
    n_frames_prev = 0
    n_hit = 0
    n_spot = 0
    sent_mode = None

    print("=" * 60)
    print("K230 瞄准程序启动  %dx%d  检测=%s  显示=%s"
          % (IMG_W, IMG_H, TARGET_METHOD, DISPLAY_MODE))
    print("=" * 60)

    try:
        while True:
            os.exitpoint()
            now = time.ticks_ms()

            # ---------- 1. 收 H723 遥测 ----------
            data = link.read(256)
            if data:
                for msg_id, seq, payload in parser.feed(data):
                    if msg_id == MSG_GIMBAL_STATE:
                        s = unpack_state(payload)
                        if s is not None:
                            gz = s
                            last_gz_t = now
                    elif msg_id == MSG_TEXT:
                        try:
                            print("H723: %s" % payload.decode("utf-8"))
                        except Exception:
                            pass
                    elif msg_id == MSG_ACK:
                        pass

            # ---------- 2. 心跳 ----------
            if time.ticks_diff(now, t_hb) >= HEARTBEAT_MS:
                t_hb = now
                link.send(build_frame(MSG_HEARTBEAT))

            # ---------- 3. 安全：遥测断 -> STAB ----------
            if (state != ST_WAIT_READY) and \
                    (time.ticks_diff(now, last_gz_t) > TELEM_TIMEOUT_MS):
                link.send(build_frame(MSG_MODE, pack_mode(MODE_STAB)))
                sent_mode = MODE_STAB
                print("!! 遥测断流 -> 发 STAB，回到 WAIT_READY")
                state = ST_WAIT_READY
                state_t = now
                att_zero = None
                ctrl.reset()
                ctrl.yaw = 0.0
                ctrl.pitch = 0.0

            ready = bool(gz is not None and (gz["flags"] & ST_READY))

            # ---------- 4. 状态机 ----------
            if state == ST_WAIT_READY:
                if ready:
                    link.send(build_frame(MSG_MODE, pack_mode(MODE_STAB)))
                    sent_mode = MODE_STAB
                    link.send(build_frame(MSG_SET_ZERO))
                    att_zero = (gz["yaw"], gz["pitch"])
                    state = ST_SET_ZERO
                    state_t = now
                    print("H723 READY -> SET_ZERO，0.3s 后进 AIM")
            elif state == ST_SET_ZERO:
                if time.ticks_diff(now, state_t) >= 300:
                    link.send(build_frame(MSG_MODE, pack_mode(MODE_AIM)))
                    sent_mode = MODE_AIM
                    state = ST_TRACK
                    state_t = now
                    print("进入 AIM，开始闭环瞄准")
            elif state == ST_TRACK:
                # 周期重发 MODE：H723 被看门狗切回 STAB 后自动拉回来
                if time.ticks_diff(now, t_mode) >= MODE_REASSERT_MS:
                    t_mode = now
                    link.send(build_frame(MSG_MODE, pack_mode(MODE_AIM)))

                dt = time.ticks_diff(now, t_last_frame) / 1000.0
                t_last_frame = now
                img = sensor.snapshot()
                n_frames += 1

                res = target_det.detect(img, float(IMG_W * IMG_H))
                spot = spot_det.detect(img)

                # 丢靶滑行：短时间内沿用最后一次误差继续闭环
                if res is not None:
                    n_hit += 1
                    tgt_uv = res["center"]
                    last_tgt = tgt_uv
                    last_tgt_t = now
                else:
                    if (time.ticks_diff(now, last_tgt_t) / 1000.0) > \
                            LOST_COAST_S:
                        tgt_uv = None
                    else:
                        tgt_uv = last_tgt

                att_rel = None
                if att_zero is not None and ready and gz is not None:
                    att_rel = (gz["yaw"] - att_zero[0],
                               gz["pitch"] - att_zero[1])

                if spot is not None:
                    n_spot += 1
                    su, sv = spot["uv"]
                else:
                    su, sv = spot_det.u, spot_det.v

                if tgt_uv is not None and spot is not None:
                    ctrl.valid = True
                    ctrl.update(max(dt, 1e-3), tgt_uv[0] - su, tgt_uv[1] - sv,
                                att_rel)
                else:
                    ctrl.valid = False
                    ctrl.locked = False

                # ---- 画到 IDE 画面 ----
                if canvas is not None:
                    if res is not None:
                        q = res["quad"]
                        for i in range(4):
                            a, b = q[i], q[(i + 1) % 4]
                            img.draw_line(a[0], a[1], b[0], b[1],
                                          color=(0, 255, 0), thickness=2)
                        img.draw_cross(int(res["center"][0]),
                                       int(res["center"][1]),
                                       color=(255, 0, 0), size=12,
                                       thickness=2)
                    if spot is not None:
                        img.draw_circle(int(su), int(sv), 8,
                                        color=(255, 255, 0), thickness=2)
                    else:
                        img.draw_circle(int(spot_det.u), int(spot_det.v), 8,
                                        color=(128, 128, 128), thickness=1)
                    img.draw_string_advanced(
                        4, 4, 18,
                        "%s err=%.0fpx yaw=%.1f pit=%.1f"
                        % (ST_NAME[state], ctrl.err_px, ctrl.yaw, ctrl.pitch),
                        color=(255, 220, 0))
                    if _display is not None:
                        _display.show_image(img)

            # ---------- 5. 发 AIM（50Hz，保持链路活着）----------
            if state in (ST_SET_ZERO, ST_TRACK) and \
                    time.ticks_diff(now, t_aim) >= (1000 // AIM_HZ):
                t_aim = now
                flags = 0
                if state == ST_TRACK and ctrl.valid:
                    flags |= FLAG_AIM_VALID
                if ctrl.locked:
                    flags |= FLAG_LOCKED
                if state == ST_TRACK and LASER_ENABLE:
                    flags |= FLAG_LASER_ON
                quality = 200 if ctrl.valid else 0
                link.send(build_frame(MSG_AIM,
                                      pack_aim(ctrl.yaw, ctrl.pitch,
                                               flags, quality)))

            # ---------- 6. 终端打印 ----------
            if time.ticks_diff(now, t_print) >= DEBUG_PRINT_MS:
                fps = ((n_frames - n_frames_prev) * 1000.0 /
                       max(1, time.ticks_diff(now, t_print)))
                n_frames_prev = n_frames
                t_print = now
                if gz is not None:
                    print("[%s] %.1ffps gz(state=%d flags=0x%02X yaw=%.1f) "
                          "tgt %d/%d spot %d/%d err=%.0f yaw=%.1f pit=%.1f "
                          "ok=%d crc=%d"
                          % (ST_NAME[state], fps, gz["state"], gz["flags"],
                             gz["yaw"], n_hit, n_frames, n_spot, n_frames,
                             ctrl.err_px, ctrl.yaw, ctrl.pitch,
                             parser.ok, parser.crc_err))
                else:
                    print("[%s] 等 H723 遥测... ok=%d crc=%d"
                          % (ST_NAME[state], parser.ok, parser.crc_err))
            gc.collect()
            time.sleep_ms(1)
    except KeyboardInterrupt:
        print("用户停止")
    finally:
        # 退出时务必让云台回到安全态并关激光
        try:
            link.send(build_frame(MSG_MODE, pack_mode(MODE_STAB)))
            time.sleep_ms(50)
        except Exception:
            pass
        link.close()
        try:
            sensor.stop()
        except Exception:
            pass
        if canvas is not None:
            try:
                _display.deinit()
                os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
                time.sleep_ms(100)
                _media.deinit()
            except Exception:
                pass
        print("已退出")


main()
