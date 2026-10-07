# -*- coding: utf-8 -*-
"""vision_check.py —— K230 视觉验证 v17（改用 cv_lite 边缘四边形检测）

画面标注：绿四边形=靶纸四条边（贴着黑胶带）、红十=靶心(对角线交点)、黄圈=激光光斑
终端每 0.5s 一行：FPS / 靶心 / 长边 / 比 / 对比 / 距离 / 光斑 / 误差 / 分段耗时

—— v17 相对 v16 的改动 ——
1) **换检测器**：靶纸四边形不再用"亮块+扫描拟合"，改成资料里 cv_lite 的
   `rgb888_find_rectangles_with_corners`（Canny 边缘 + 多边形拟合，C 加速，
   直接给 4 个角点）。它是沿着**真实的边**（黑胶带内沿）走的，不像亮块那样
   会被阴影/圆环/背景亮边带偏——现场"边跳变"就是这么来的。
   找不到/装不了 cv_lite 时，退回"亮块外接框"这种最简兜底（只画正矩形）。
2) **画面倒置**：新加 CAM_VFLIP / CAM_HMIRROR 开关（第 40 行），
   画面倒着就把 CAM_VFLIP 设 True（sensor.set_vflip）。传感器翻转后
   检测和绘制都在同一个坐标系里，靶心/误差不受影响。
3) **删掉冗余**：v13~v16 试过的"行/列二分扫描、Theil-Sen 拟合、最小外接矩形、
   快速复测"那套代码全部删除，只留当前在用的判据。
4) 画面仍隔帧推送（to_ide 传输比处理慢，追太紧会撕裂/花屏）；
   光斑每 SPOT_EVERY 帧搜一次（光轴固定，位置不变）。

判据（都来自现场日志实测）：真靶纸 长边184~225px、斜视时更短；
误检是 36~38px 小亮块或 71~300px 的背景块。所以保留：
长边 60~480px、长宽比 0.70~3.0、内外对比≥40 且外侧 12 点≥70% 更暗、
以及跟踪尺寸闸门(0.55~1.45x)+连续确认(3次)+重新锁定尺寸闸门(0.4~2.5x)。
"""
import os
import time
import math

# ============================ 相机 / 显示 ============================
IMG_W = 640
IMG_H = 480
DISPLAY_QUALITY = 40       # IDE 画面 JPEG 质量（越小传输越快）
SHOW_EVERY = 2             # 每几帧推一次画面（识别仍每帧都算）
CAM_VFLIP = True           # 画面上下颠倒 -> True（你现在需要这个）
CAM_HMIRROR = False        # 画面左右镜像 -> True

# ============================ cv_lite 四边形检测 ============================
CANNY_LO = 50              # Canny 低阈值（检不到就调小，误检多就调大）
CANNY_HI = 150             # Canny 高阈值
APPROX_EPS = 0.02          # 多边形拟合精度（越小越贴边，越小越易受噪声影响）
AREA_MIN_RATIO = 0.002     # 最小面积比例（相对整幅）
MAX_ANGLE_COS = 0.35       # 角度余弦上限（越小越接近直角矩形）
BLUR_SIZE = 5              # 高斯模糊核（奇数）

# ============================ 靶纸判据 ============================
PAPER_MIN_LONG = 60        # 四边形长边下限 px（2.2m 处 A4 约 60px）
PAPER_MAX_LONG = 480       # 上限 px
PAPER_ASPECT_MIN = 0.70    # 长边/短边（A4=1.41，斜视透视会缩到接近 1）
PAPER_ASPECT_MAX = 3.00
PAPER_CONTRAST_MIN = 40    # 内亮度 - 外亮度（0~255 量程）
PAPER_DARK_MARGIN = 25     # 单个外侧采样点算“暗”的门槛
PAPER_DARK_FRAC_MIN = 0.70  # 外侧 12 个点里至少这么多比例比内部暗

# ============================ 跟踪 / 闸门 ============================
TRACK_K = 0.50             # 跟踪窗 = 四边形长边 x TRACK_K + TRACK_PAD
TRACK_PAD = 30
HOLD_FRAMES = 15           # 丢靶后还画多少帧
LOST_FULL = 18             # 丢这么多帧后放弃小窗，改全图搜索
FULL_EVERY = 2             # 每几帧做一次全图搜索
SMOOTH = 0.55              # 平滑系数
DEADBAND_PX = 1.5          # 平滑死区（小于它不动，画面不抖）
SIZE_GATE_LO = 0.55        # 跟踪时允许的长边变化范围
SIZE_GATE_HI = 1.45
CONFIRM_N = 3              # 全图候选连续确认几次才算重新锁定
CONFIRM_DXY = 35           # 确认时的位置一致范围 px
CONFIRM_DSIZE = 0.60       # 确认时的尺寸一致范围
RELOCK_LO = 0.40           # 重新锁定时相对上次的尺寸范围（纸不会半秒变 1/4 大）
RELOCK_HI = 2.50
RELOCK_FREE_LOST = 100     # 丢靶超过这么多帧就放开尺寸限制（靶纸可能被搬远）
PENDING_MISS = 3           # 确认过程中允许漏几次

# ============================ 激光光斑 ============================
SPOT_THRESHOLDS = [
    (88, 100, -40, 90, -50, 90),   # 过曝白芯（纸面 LAB-L≈78 到不了）
    (58, 100, 25, 90, -20, 70),    # 明显红边（a>=25）
]
SPOT_MIN_AREA = 2
SPOT_MAX_AREA = 2500
SPOT_MAX_ASPECT = 3.0
SPOT_ROI_HALF = 45         # 只在学习到的光轴点附近找
SPOT_LEARN_FRAMES = 10
SPOT_NEAR_PX = 12
SPOT_EVERY = 4             # 每几帧搜一次光斑（光轴固定，中间帧复用）

# ============================ 距离 ============================
FX_PX = 445.0              # 像素焦距（640 宽；0.6m 处 A4 长边≈222px 反推）
PAPER_LONG_M = 0.297       # A4 长边（米）

# ============================ 串口 / 协议 ============================
UART_BAUD = 115200
HEARTBEAT_MS = 200
AIM_MS = 20
PRINT_MS = 500
GC_EVERY = 30

SOF = b"\xAA\x55"
MSG_AIM = 0x10
MSG_MODE = 0x11
MSG_HEARTBEAT = 0x12
MSG_SET_ZERO = 0x13
MSG_GIMBAL_STATE = 0x90
MSG_TEXT = 0x93
ST_READY = 0x10

try:
    import cv_lite
    HAVE_CV = True
except Exception:
    HAVE_CV = False


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
    return ustruct.pack("<hhBB",
                        max(-32768, min(32767, int(round(yaw_deg * 100.0)))),
                        max(-32768, min(32767, int(round(pitch_deg * 100.0)))),
                        flags & 0xFF, max(0, min(255, int(quality))))


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
                    self.buf = self.buf[-1:]
                break
            if i > 0:
                self.buf = self.buf[i:]
            if len(self.buf) < 5:
                break
            length = self.buf[4]
            if length > 240:
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


# ============================ 视觉：靶纸四边形 ============================
def luma(r, g, b):
    return (r * 77 + g * 150 + b * 29) >> 8


def px_luma(img, x, y):
    """取某点亮度（0~255）。取不到返回 -1。"""
    if x < 0:
        x = 0
    elif x >= IMG_W:
        x = IMG_W - 1
    if y < 0:
        y = 0
    elif y >= IMG_H:
        y = IMG_H - 1
    try:
        p = img.get_pixel(int(x), int(y))
    except Exception:
        return -1
    if p is None:
        return -1
    return luma(p[0], p[1], p[2])


def paper_contrast(img, x, y, w, h):
    """内亮外暗校验：返回 (内部平均亮度, 外侧平均亮度, 外侧合格比例)。

    外侧取 12 个点（四边各 3 个），要求至少 PAPER_DARK_FRAC_MIN 的比例
    明显比内部暗——背景柜子那种“只有一边有暗边”的亮块因此过不了。
    """
    u = ((x + w * 0.30, y + h * 0.50), (x + w * 0.70, y + h * 0.50),
         (x + w * 0.50, y + h * 0.30), (x + w * 0.50, y + h * 0.70),
         (x + w * 0.50, y + h * 0.50))
    o = ((x - 5, y + h * 0.25), (x - 5, y + h * 0.50), (x - 5, y + h * 0.75),
         (x + w + 5, y + h * 0.25), (x + w + 5, y + h * 0.50),
         (x + w + 5, y + h * 0.75),
         (x + w * 0.25, y - 5), (x + w * 0.50, y - 5), (x + w * 0.75, y - 5),
         (x + w * 0.25, y + h + 5), (x + w * 0.50, y + h + 5),
         (x + w * 0.75, y + h + 5))
    si = 0
    so = 0
    ni = 0
    no = 0
    for p in u:
        v = px_luma(img, p[0], p[1])
        if v >= 0:
            si += v
            ni += 1
    for p in o:
        v = px_luma(img, p[0], p[1])
        if v >= 0:
            so += v
            no += 1
    if ni < 3 or no < 6:
        return -1, -1, 0.0
    ins = si / float(ni)
    ok = 0
    for p in o:
        v = px_luma(img, p[0], p[1])
        if v >= 0 and v < (ins - PAPER_DARK_MARGIN):
            ok += 1
    return ins, so / float(no), ok / float(no)


def _quad_metrics(c):
    """给 4 个角点算 (中心, 长边, 短边, 面积)。角点按轮廓顺序给出。"""
    a = 0.0
    for i in range(4):
        x1, y1 = c[i]
        x2, y2 = c[(i + 1) % 4]
        a += x1 * y2 - x2 * y1
    qa = abs(a) * 0.5
    if qa < 1.0:
        return None
    d = []
    for i in range(4):
        x1, y1 = c[i]
        x2, y2 = c[(i + 1) % 4]
        d.append(math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))
    dl1 = (d[0] + d[2]) / 2.0
    dl2 = (d[1] + d[3]) / 2.0
    if dl1 >= dl2:
        long_side, short_side = dl1, dl2
    else:
        long_side, short_side = dl2, dl1
    # 对角线交点 = 透视意义下的中心（角点顺序循环即可，c0-c2 / c1-c3 是对角线）
    d1x, d1y = c[2][0] - c[0][0], c[2][1] - c[0][1]
    d2x, d2y = c[3][0] - c[1][0], c[3][1] - c[1][1]
    den = d1x * d2y - d1y * d2x
    if abs(den) < 1e-6:
        ctr = ((c[0][0] + c[2][0]) / 2.0, (c[0][1] + c[2][1]) / 2.0)
    else:
        t = ((c[1][0] - c[0][0]) * d2y - (c[1][1] - c[0][1]) * d2x) / den
        ctr = (c[0][0] + t * d1x, c[0][1] + t * d1y)
    return ctr, long_side, max(1.0, short_side), qa


def find_paper(img, roi, dbg_on=False):
    """找靶纸四边形。返回 (best, 候选数, 诊断串)。

    best = (px, cu, cv, long_side, aspect, density, ins, outs, corners)
    · 有 cv_lite：用 Canny+四边形检测（沿真实边缘，C 加速），直接拿 4 角
    · 没有 cv_lite：退回亮块外接框（只画正矩形，功能受限但能跑）
    """
    if not HAVE_CV:
        return _find_paper_blob(img, roi, dbg_on)
    rx, ry, rw, rh = roi
    rx1 = rx + rw
    ry1 = ry + rh
    try:
        rects = cv_lite.rgb888_find_rectangles_with_corners(
            [IMG_H, IMG_W], img.to_numpy_ref(), CANNY_LO, CANNY_HI,
            APPROX_EPS, AREA_MIN_RATIO, MAX_ANGLE_COS, BLUR_SIZE)
    except Exception as e:
        return None, 0, "[cv_lite异常:%s] " % e
    best = None
    n = 0
    dbg = ""
    for r in rects:
        if len(r) < 12:
            continue
        c = ((r[4], r[5]), (r[6], r[7]), (r[8], r[9]), (r[10], r[11]))
        m = _quad_metrics(c)
        if m is None:
            continue
        ctr, long_side, short_side, qa = m
        if (ctr[0] < rx) or (ctr[0] > rx1) or (ctr[1] < ry) or (ctr[1] > ry1):
            continue                      # 不在跟踪窗里
        n += 1
        aspect = long_side / short_side
        xs = (c[0][0], c[1][0], c[2][0], c[3][0])
        ys = (c[0][1], c[1][1], c[2][1], c[3][1])
        bx = min(xs)
        by = min(ys)
        w = max(xs) - bx
        h = max(ys) - by
        if dbg_on and n <= 3:
            dbg += "[四边%.0fx%.0f 比%.2f 面%.0f] " % (
                long_side, short_side, aspect, qa)
        if (long_side < PAPER_MIN_LONG) or (long_side > PAPER_MAX_LONG):
            continue
        if (aspect < PAPER_ASPECT_MIN) or (aspect > PAPER_ASPECT_MAX):
            continue
        if (bx <= 1) or (by <= 1) or ((bx + w) >= IMG_W - 1) or \
                ((by + h) >= IMG_H - 1):
            continue
        ins, outs, frac = paper_contrast(img, bx, by, w, h)
        if dbg_on and n <= 3:
            dbg += "{内%d 外%d 暗边%.0f%%} " % (ins, outs, frac * 100.0)
        if (ins < 0) or ((ins - outs) < PAPER_CONTRAST_MIN) or \
                (frac < PAPER_DARK_FRAC_MIN):
            continue
        score = qa * (0.5 + min(ins - outs, 80) / 80.0)
        if best is None or score > best[0]:
            best = (score, (qa, ctr[0], ctr[1], long_side, aspect, 1.0,
                            ins, outs, c))
    if best is None:
        return None, n, dbg
    return best[1], n, dbg


def _find_paper_blob(img, roi, dbg_on=False):
    """兜底：没有 cv_lite 时用亮块外接框当四边形（只画正矩形）。"""
    best = None
    n = 0
    dbg = ""
    blobs = img.find_blobs([(58, 100, -32, 32, -32, 32)], roi=roi, merge=True,
                           margin=6, area_threshold=900, pixels_threshold=900)
    for b in blobs:
        x, y, w, h, px = b[0], b[1], b[2], b[3], b[4]
        if roi[0] or roi[1]:
            if (x + w / 2.0) < roi[0] or (y + h / 2.0) < roi[1]:
                x += roi[0]
                y += roi[1]
        n += 1
        long_side = w if w > h else h
        aspect = long_side / max(1.0, float(w if w < h else h))
        inset = px / float(w * h)
        ins, outs, frac = paper_contrast(img, x, y, w, h)
        if dbg_on and n <= 3:
            dbg += "[亮块%dx%d 密%.2f 内%d 外%d 暗边%.0f%%] " % (
                w, h, inset, ins, outs, frac * 100.0)
        if (w < 8) or (h < 8) or (inset < 0.70):
            continue
        if (long_side < PAPER_MIN_LONG) or (long_side > PAPER_MAX_LONG):
            continue
        if (aspect < PAPER_ASPECT_MIN) or (aspect > PAPER_ASPECT_MAX):
            continue
        if (x <= 1) or (y <= 1) or ((x + w) >= IMG_W - 1) or \
                ((y + h) >= IMG_H - 1):
            continue
        if (ins < 0) or ((ins - outs) < PAPER_CONTRAST_MIN) or \
                (frac < PAPER_DARK_FRAC_MIN):
            continue
        c = ((x, y), (x + w, y), (x + w, y + h), (x, y + h))
        score = px * (0.5 + min(ins - outs, 80) / 80.0)
        if best is None or score > best[0]:
            best = (score, (px, x + w / 2.0, y + h / 2.0, long_side, aspect,
                            inset, ins, outs, c))
    if best is None:
        return None, n, dbg
    return best[1], n, dbg


class PaperTracker(object):
    """候选 -> 尺寸闸门 -> 连续确认 -> 平滑（治“绿框乱飘”的那一套）。"""

    def __init__(self):
        self.u = 0.0
        self.v = 0.0
        self.w = 0.0
        self.h = 0.0
        self.have = False
        self.lost = 0
        self.meas = None
        self.corners = None
        self.pend = None       # [cu, cv, long, cnt, miss]
        self.frame = 0
        self.last_n = 0
        self.last_dbg = ""
        self.did_full = False
        self.state = "搜索"

    def roi(self):
        r = int(max(self.w, self.h) * TRACK_K) + TRACK_PAD
        x = int(self.u - r)
        y = int(self.v - r)
        if x < 0:
            x = 0
        if y < 0:
            y = 0
        w = 2 * r
        h = 2 * r
        if x + w > IMG_W:
            w = IMG_W - x
        if y + h > IMG_H:
            h = IMG_H - y
        return (x, y, w, h)

    def fresh(self):
        return self.have and (self.lost < HOLD_FRAMES)

    def box(self):
        return (int(self.u - self.w / 2.0), int(self.v - self.h / 2.0),
                int(self.w), int(self.h))

    def _size_ok(self, long_side):
        if not self.have:
            return True
        old = max(self.w, self.h)
        return (SIZE_GATE_LO * old) <= long_side <= (SIZE_GATE_HI * old)

    def _confirm(self, cand):
        """全图候选的连续确认。返回 True = 确认为目标。"""
        if cand is None:
            if self.pend is not None:
                self.pend[4] += 1
                if self.pend[4] > PENDING_MISS:
                    self.pend = None
            return False
        cu, cv, lo = cand[1], cand[2], cand[3]
        # 重新锁定也要过尺寸合理性（现场锁到 71px 背景块就是这么漏的）；
        # 丢靶太久说明靶纸可能被搬远，就放开这个限制
        if self.have and (self.lost < RELOCK_FREE_LOST):
            old = max(self.w, self.h)
            if (lo < RELOCK_LO * old) or (lo > RELOCK_HI * old):
                self.pend = None
                return False
        if self.pend is None:
            self.pend = [cu, cv, lo, 1, 0]
            return False
        if (abs(cu - self.pend[0]) <= CONFIRM_DXY) and \
                (abs(cv - self.pend[1]) <= CONFIRM_DXY) and \
                (abs(lo - self.pend[2]) <= CONFIRM_DSIZE * self.pend[2]):
            k = self.pend[3] + 1
            self.pend[0] += (cu - self.pend[0]) / float(k)
            self.pend[1] += (cv - self.pend[1]) / float(k)
            self.pend[2] += (lo - self.pend[2]) / float(k)
            self.pend[3] = k
            self.pend[4] = 0
            if k >= CONFIRM_N:
                self.pend = None
                return True
        else:
            self.pend = [cu, cv, lo, 1, 0]
        return False

    def _accept(self, cand):
        cu, cv = cand[1], cand[2]
        w = max(1.0, cand[3])
        h = max(1.0, w / max(1.0, cand[4]))
        if not self.have:
            self.u, self.v = cu, cv
            self.have = True
        else:
            if abs(cu - self.u) > DEADBAND_PX:
                self.u += SMOOTH * (cu - self.u)
            if abs(cv - self.v) > DEADBAND_PX:
                self.v += SMOOTH * (cv - self.v)
        self.w = w
        self.h = h
        self.meas = cand
        self.corners = cand[8]
        self.lost = 0
        self.state = "锁定"

    def step(self, img, finder):
        """跑一帧跟踪。返回本帧确认的候选或 None。"""
        self.frame += 1
        cand = None
        n = 0
        dbg = ""
        self.did_full = False
        if self.have and self.lost < LOST_FULL:
            cand, n, dbg = finder(img, self.roi())
            if (cand is not None) and (not self._size_ok(cand[3])):
                dbg = "[尺寸闸门%.0fpx] " % cand[3] + dbg
                cand = None
            if (cand is None) and ((self.frame % FULL_EVERY) == 0):
                self.did_full = True
                c2, n2, d2 = finder(img, (0, 0, IMG_W, IMG_H))
                n += n2
                dbg += d2
                if self._confirm(c2):
                    cand = c2
        else:
            if (self.frame % FULL_EVERY) == 0:
                self.did_full = True
                c2, n2, d2 = finder(img, (0, 0, IMG_W, IMG_H), True)
                n += n2
                dbg = d2
                if self._confirm(c2):
                    cand = c2
        if cand is not None:
            self._accept(cand)
        else:
            self.lost += 1
            if self.pend is not None:
                self.state = "确认%d/%d" % (self.pend[3], CONFIRM_N)
            else:
                self.state = "搜索"
        self.last_n = n
        self.last_dbg = dbg
        return cand


class SpotDetector(object):
    """找激光光斑：暖色/过曝亮块，学一个光轴点后锁定。"""

    def __init__(self):
        self.u = IMG_W / 2.0
        self.v = IMG_H / 2.0
        self.locked = False
        self.learn_u = self.u
        self.learn_v = self.v
        self.learn_n = 0
        self.meas = None

    def detect(self, img):
        x0 = int(self.u - SPOT_ROI_HALF)
        y0 = int(self.v - SPOT_ROI_HALF)
        if x0 < 0:
            x0 = 0
        if y0 < 0:
            y0 = 0
        w = min(IMG_W - x0, SPOT_ROI_HALF * 2)
        h = min(IMG_H - y0, SPOT_ROI_HALF * 2)
        roi = (x0, y0, w, h)
        best = None
        for b in img.find_blobs(SPOT_THRESHOLDS, roi=roi, merge=True,
                                pixels_threshold=SPOT_MIN_AREA,
                                area_threshold=SPOT_MIN_AREA):
            area = b[4]
            bw, bh = b[2], b[3]
            if area < SPOT_MIN_AREA or area > SPOT_MAX_AREA:
                continue
            if bw < 1 or bh < 1:
                continue
            if max(bw, bh) * 1.0 / min(bw, bh) > SPOT_MAX_ASPECT:
                continue
            bx, by = b[0], b[1]
            if roi[0] or roi[1]:
                if (bx + bw / 2.0) < roi[0] or (by + bh / 2.0) < roi[1]:
                    bx += roi[0]
                    by += roi[1]
            cu = bx + bw / 2.0
            cv = by + bh / 2.0
            d = abs(cu - self.u) + abs(cv - self.v)
            if best is None or d < best[0]:
                best = (d, area, cu, cv)
        if best is None:
            self.meas = None
            self.learn_n = 0
            return None
        self.meas = (best[1], best[2], best[3])
        if not self.locked:
            if self.learn_n == 0:
                self.learn_u, self.learn_v = best[2], best[3]
                self.learn_n = 1
            elif (abs(best[2] - self.learn_u) <= SPOT_NEAR_PX) and \
                    (abs(best[3] - self.learn_v) <= SPOT_NEAR_PX):
                self.learn_n += 1
                self.learn_u += (best[2] - self.learn_u) / float(self.learn_n)
                self.learn_v += (best[3] - self.learn_v) / float(self.learn_n)
            else:
                self.learn_u, self.learn_v = best[2], best[3]
                self.learn_n = 1
            if self.learn_n >= SPOT_LEARN_FRAMES:
                self.u, self.v = self.learn_u, self.learn_v
                self.locked = True
                print("光斑位置已锁定: (%.1f, %.1f)" % (self.u, self.v))
        return self.meas


def print_l_diag(img):
    """丢靶时的亮度诊断，用来判断阈值该往哪调。"""
    err = None
    try:
        st = img.get_statistics(roi=(0, 0, IMG_W, IMG_H))
        print("  [诊断] L均=%.0f 中=%.0f 大=%.0f"
              % (st.l_mean(), st.l_median(), st.l_max()))
        return
    except Exception as e:
        err = e
    try:
        h = img.get_histogram(roi=(0, 0, IMG_W, IMG_H))
        print("  [诊断] Otsu=%d" % h.get_threshold().l_value())
    except Exception:
        print("  [诊断] 统计不可用: %s" % err)


def main():
    from media.sensor import Sensor, CAM_CHN_ID_0
    from media.display import Display
    from media.media import MediaManager
    import gc

    from ybUtils.YbUart import YbUart
    uart = YbUart(baudrate=UART_BAUD)
    parser = FrameParser()
    print("串口: YbUart @%d" % UART_BAUD)

    sensor = Sensor(id=2, width=1280, height=960, fps=90)
    sensor.reset()
    time.sleep_ms(100)
    sensor.set_framesize(width=IMG_W, height=IMG_H, chn=CAM_CHN_ID_0)
    # cv_lite 需要 RGB888（to_numpy_ref 拿到的就是它的数据）
    sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)
    try:
        sensor.set_vflip(CAM_VFLIP)
        sensor.set_hmirror(CAM_HMIRROR)
    except Exception as e:
        print("翻转设置失败(可忽略): %s" % e)
    Display.init(Display.ST7701, width=IMG_W, height=IMG_H,
                 to_ide=True, quality=DISPLAY_QUALITY)
    MediaManager.init()
    sensor.run()
    clock = time.clock()

    tracker = PaperTracker()
    spot_det = SpotDetector()

    ready = False
    armed = False
    last_text = ""
    t_hb = time.ticks_ms()
    t_aim = time.ticks_ms()
    t_print = time.ticks_ms()
    n_frame = 0
    n_frame_prev = 0
    n_tgt = 0
    n_spot = 0
    sum_grab = 0
    sum_tgt = 0
    sum_spot = 0
    sum_show = 0
    sum_n = 0

    print("=" * 68)
    print("vision_check v17: 检测器=%s 画面VFLIP=%s HMIRROR=%s"
          % ("cv_lite四边形" if HAVE_CV else "亮块兜底(!)", CAM_VFLIP,
             CAM_HMIRROR))
    print("  Canny=%d/%d eps=%.3f 面积比>=%.4f 角度cos<=%.2f 模糊%d"
          % (CANNY_LO, CANNY_HI, APPROX_EPS, AREA_MIN_RATIO,
             MAX_ANGLE_COS, BLUR_SIZE))
    print("  长边%.0f~%.0f 比%.2f~%.2f 对比>=%d 暗边>=%.0f%%  跟踪窗=长边x%.2f+%d"
          % (PAPER_MIN_LONG, PAPER_MAX_LONG, PAPER_ASPECT_MIN,
             PAPER_ASPECT_MAX, PAPER_CONTRAST_MIN,
             PAPER_DARK_FRAC_MIN * 100, TRACK_K, TRACK_PAD))
    print("=" * 68)

    try:
        while True:
            os.exitpoint()
            now = time.ticks_ms()

            # ---------- 串口 ----------
            try:
                if uart.any() > 0:
                    for msg_id, seq, payload in parser.feed(uart.read(256)):
                        if msg_id == MSG_GIMBAL_STATE and len(payload) == 21:
                            import ustruct
                            v = ustruct.unpack("<BBhhhhhhhBI", payload)
                            ready = bool(v[9] & ST_READY)
                        elif msg_id == MSG_TEXT:
                            try:
                                txt = payload.decode("utf-8")
                            except Exception:
                                txt = ""
                            if txt != last_text:
                                last_text = txt
                                print("  H723: %s" % txt)
            except Exception:
                pass

            if time.ticks_diff(now, t_hb) >= HEARTBEAT_MS:
                t_hb = now
                try:
                    uart.write(build_frame(MSG_HEARTBEAT))
                except Exception:
                    pass

            if ready and not armed:
                try:
                    uart.write(build_frame(MSG_SET_ZERO))
                    time.sleep_ms(20)
                    uart.write(build_frame(MSG_MODE, bytes([2, 0])))
                    armed = True
                    print("H723 READY -> SET_ZERO + AIM（激光打开，偏置 0）")
                except Exception as e:
                    print("发 MODE 失败: %s" % e)

            if armed and time.ticks_diff(now, t_aim) >= AIM_MS:
                t_aim = now
                try:
                    uart.write(build_frame(MSG_AIM,
                                           pack_aim(0.0, 0.0, 0x03, 200)))
                except Exception:
                    pass

            # ---------- 取图 ----------
            t0 = time.ticks_ms()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            t_grab = time.ticks_diff(time.ticks_ms(), t0)
            n_frame += 1

            # ---------- 靶纸四边形 ----------
            t0 = time.ticks_ms()
            cand = tracker.step(img, find_paper)
            t_tgt = time.ticks_diff(time.ticks_ms(), t0)
            if cand is not None:
                n_tgt += 1

            # ---------- 光斑（每 SPOT_EVERY 帧搜一次）----------
            t0 = time.ticks_ms()
            if (n_frame % SPOT_EVERY) == 0:
                spot = spot_det.detect(img)
            else:
                spot = spot_det.meas
            t_spot = time.ticks_diff(time.ticks_ms(), t0)
            if spot is not None:
                n_spot += 1

            if tracker.lost == 8:
                print_l_diag(img)

            # ---------- 画 ----------
            if tracker.fresh():
                c = tracker.corners
                if c is not None:
                    for i in range(4):
                        a = c[i]
                        b = c[(i + 1) % 4]
                        img.draw_line(int(a[0]), int(a[1]),
                                      int(b[0]), int(b[1]),
                                      color=(0, 255, 0), thickness=2)
                else:
                    bx, by, bw, bh = tracker.box()
                    img.draw_rectangle(bx, by, bw, bh, color=(0, 255, 0),
                                       thickness=2)
                img.draw_cross(int(tracker.u), int(tracker.v),
                               color=(255, 0, 0), size=14, thickness=2)
            if spot is not None:
                img.draw_circle(int(spot[1]), int(spot[2]), 8,
                                color=(255, 255, 0), thickness=2)
                img.draw_cross(int(spot[1]), int(spot[2]),
                               color=(255, 255, 0), size=6, thickness=1)

            # 隔帧推画面（to_ide 传输比处理慢，追太紧会撕裂/花屏）
            t0 = time.ticks_ms()
            if (n_frame % SHOW_EVERY) == 0:
                Display.show_image(img)
            t_show = time.ticks_diff(time.ticks_ms(), t0)
            del img

            sum_grab += t_grab
            sum_tgt += t_tgt
            sum_spot += t_spot
            sum_show += t_show
            sum_n += 1

            # ---------- 打印 ----------
            if time.ticks_diff(now, t_print) >= PRINT_MS:
                fps = ((n_frame - n_frame_prev) * 1000.0 /
                       max(1, time.ticks_diff(now, t_print)))
                n_frame_prev = n_frame
                t_print = now
                line = "FPS=%.1f " % fps
                if tracker.meas is not None and tracker.lost == 0:
                    px, cu, cv, long_side, aspect, density, ins, outs, c = \
                        tracker.meas
                    dist_m = FX_PX * PAPER_LONG_M / max(1.0, long_side)
                    line += ("靶=纸(%s) 靶心=(%.0f,%.0f) 长边=%.0fpx 比%.2f "
                             "对比=%.0f 距离≈%.2fm"
                             % (tracker.state, tracker.u, tracker.v,
                                long_side, aspect, ins - outs, dist_m))
                else:
                    line += "靶=%s%s 候选=%d %s" % (
                        tracker.state,
                        ("(全图)" if tracker.did_full else "(跟踪窗)"),
                        tracker.last_n, tracker.last_dbg)
                if spot is not None:
                    if tracker.meas is not None and tracker.lost == 0:
                        eu = tracker.u - spot[1]
                        ev = tracker.v - spot[2]
                        err = math.sqrt(eu * eu + ev * ev)
                        line += (" | 光斑=(%.0f,%.0f) 误差=(%.0f,%.0f)px "
                                 "=%.1fpx(%.2f°)"
                                 % (spot[1], spot[2], eu, ev, err,
                                    err / FX_PX * 57.29578))
                    else:
                        line += " | 光斑=(%.0f,%.0f)" % (spot[1], spot[2])
                else:
                    line += " | 光斑: 未检出"
                if sum_n > 0:
                    line += " | ms 取图%.0f 目标%.0f 光斑%.0f 显示%.0f" % (
                        sum_grab / float(sum_n), sum_tgt / float(sum_n),
                        sum_spot / float(sum_n), sum_show / float(sum_n))
                line += " | 靶%d/%d 斑%d" % (n_tgt, n_frame, n_spot)
                print(line)
                sum_grab = 0
                sum_tgt = 0
                sum_spot = 0
                sum_show = 0
                sum_n = 0

            if (n_frame % GC_EVERY) == 0:
                gc.collect()
    except KeyboardInterrupt:
        print("用户停止")
    finally:
        try:
            uart.write(build_frame(MSG_MODE, bytes([1, 0])))
            time.sleep_ms(50)
        except Exception:
            pass
        try:
            uart.deinit()
        except Exception:
            pass
        try:
            sensor.stop()
        except Exception:
            pass
        try:
            Display.deinit()
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
            time.sleep_ms(100)
            MediaManager.deinit()
        except Exception:
            pass
        print("已退出（激光已关闭）")


main()
