# -*- coding: utf-8 -*-
"""vision_check.py —— K230 视觉验证 v11

在 CanMV IDE 里直接运行。画面显示在 IDE 的帧缓冲（不需要接屏幕），终端看数据。

画面标注：
    绿框  = 检出的 A4 靶纸
    红十  = 靶心（= 靶纸中心 = 同心圆圆心）
    黄圈  = 激光光斑

终端每 0.5 秒一行，例如：
    FPS=17.2 靶=纸 靶心=(330,268) 长边=222px 比1.15 密0.92 对比=41 距离≈0.60m
    光斑=(319,216) 误差=(11,52)px =53.2px(6.85°) | ms 目标9 光斑3 显示31

—— v11 的判据为什么这么定（依据截图实测像素，别再回退）——
1) 靶子 = A4 白纸亮块 + “四周更暗”校验。
   实测本场地 L(0~100)：白纸 60~70、木柜 25~35、墙 45、黑胶带 20。
   固定亮度阈值就能把纸从背景里分出来；纸四周的黑胶带正好提供
   “亮块四周必须更暗”的校验条件。白瓷砖地面虽然亮，但没有这圈暗框，
   所以当初担心的“白地面误检”不会发生。
   （v6~v9 用“暗块找胶带”失败：胶带 L≈20 与木柜 L≈25~35 几乎一样，阈值一高
     就把柜门阴影连成一片。v10 以为靶纸上的圈是红色，实测圈是深色印刷，也不对。）
2) 速度：find_blobs 只在跟踪窗里跑，不再每帧做全图直方图/全图搜索
   （这是 v6 只有 5.5fps 的主因）；丢靶才全图搜索，且最多每 3 帧一次。
3) 稳定：检出结果做指数平滑，丢靶后保持 15 帧再消失，画面不会忽有忽没。

真检不到时的调参顺序：PAPER_TH（58，先试 52 或 64）-> PAPER_CONTRAST_MIN。
"""
import os
import time
import math

# ============================ 画面 / 显示 ============================
IMG_W = 640
IMG_H = 480
DISPLAY_QUALITY = 50

# ============================ 靶纸检测 ============================
PAPER_TH = 58              # 亮度阈值（0~100）。纸偏暗就调小、背景也亮就调大
PAPER_TH_ALT = 72          # 兜底阈值：全图第一遍失败时再试一次
PAPER_A_MAX = 32           # |a| 上限（偏色背景如木柜/草地会被排除）
PAPER_B_MAX = 32           # |b| 上限
PAPER_MIN_AREA = 450       # 最小像素面积（2.2m 处 A4 约 55x39px ≈ 2100px）
PAPER_MAX_AREA_RATIO = 0.85
PAPER_ASPECT_MIN = 1.05    # 长边/短边（A4=1.414，斜视会更极端）
PAPER_ASPECT_MAX = 3.00
PAPER_DENSITY_MIN = 0.50   # 实际像素 / 外接框面积
PAPER_CONTRAST_MIN = 22    # 内亮度 - 外亮度（0~255 量程）

# 跟踪窗 / 保持
TRACK_PAD = 45             # 跟踪窗 = 上次方框 + 这个余量
HOLD_FRAMES = 15           # 丢靶后还画/还用多少帧
FULL_EVERY = 3             # 丢靶时每几帧做一次全图搜索
SMOOTH = 0.55              # 平滑系数（1.0=不平滑）

# ============================ 激光光斑 ============================
SPOT_THRESHOLDS = [
    (50, 100, 6, 80, -30, 70),     # 暖色亮块（红激光边缘）
    (80, 100, -25, 70, -40, 80),   # 过曝白芯
]
SPOT_MIN_AREA = 2
SPOT_MAX_AREA = 2500
SPOT_MAX_ASPECT = 3.0
SPOT_ROI_HALF = 55         # 光轴固定，只在学习到的点附近找
SPOT_LEARN_FRAMES = 12
SPOT_NEAR_PX = 30

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


# ============================ 视觉：靶纸 ============================
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
    """内亮外暗校验：返回 (内部平均亮度, 四周平均亮度)，取不到给 -1。"""
    u = ((x + w * 0.30, y + h * 0.50), (x + w * 0.70, y + h * 0.50),
         (x + w * 0.50, y + h * 0.30), (x + w * 0.50, y + h * 0.70),
         (x + w * 0.50, y + h * 0.50))
    o = ((x - 5, y + h * 0.50), (x + w + 5, y + h * 0.50),
         (x + w * 0.50, y - 5), (x + w * 0.50, y + h + 5),
         (x - 5, y - 5))
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
    if ni < 3 or no < 3:
        return -1, -1
    return si / float(ni), so / float(no)


def norm_bbox(bx, by, bw, bh, roi):
    """兼容 find_blobs 返回“相对 ROI”或“全图”两种坐标。"""
    if roi[0] or roi[1]:
        if (bx + bw / 2.0) < roi[0] or (by + bh / 2.0) < roi[1]:
            return bx + roi[0], by + roi[1]
    return bx, by


def find_paper(img, roi, th, dbg_on=False):
    """在 roi 里找 A4 靶纸。返回 (best, 亮块数, 诊断字符串)。"""
    best = None
    n = 0
    dbg = ""
    blobs = img.find_blobs(
        [(th, 100, -PAPER_A_MAX, PAPER_A_MAX, -PAPER_B_MAX, PAPER_B_MAX)],
        roi=roi, merge=True, margin=6,
        area_threshold=PAPER_MIN_AREA, pixels_threshold=PAPER_MIN_AREA)
    if not blobs:
        return None, 0, ""
    for b in blobs:
        n += 1
        x, y, w, h, px = b[0], b[1], b[2], b[3], b[4]
        x, y = norm_bbox(x, y, w, h, roi)
        if w < 8 or h < 8:
            continue
        long_side = w if w > h else h
        short_side = w if w < h else h
        aspect = long_side / float(short_side)
        density = px / float(w * h)
        ins, outs = paper_contrast(img, x, y, w, h)
        if dbg_on and n <= 3:
            dbg += "[%dpx 比%.2f 密%.2f 内%d 外%d] " % (
                px, aspect, density, ins, outs)
        if px < PAPER_MIN_AREA:
            continue
        if px > PAPER_MAX_AREA_RATIO * IMG_W * IMG_H:
            continue
        if aspect < PAPER_ASPECT_MIN or aspect > PAPER_ASPECT_MAX:
            continue
        if density < PAPER_DENSITY_MIN:
            continue
        if x <= 1 or y <= 1 or (x + w) >= IMG_W - 1 or (y + h) >= IMG_H - 1:
            continue
        if ins < 0 or (ins - outs) < PAPER_CONTRAST_MIN:
            continue
        score = px * (0.5 + min(ins - outs, 80) / 80.0)
        if best is None or score > best[0]:
            best = (score, (px, x, y, w, h, long_side, aspect, density,
                            ins, outs))
    if best is None:
        return None, n, dbg
    return best[1], n, dbg


class PaperTracker(object):
    """跟踪状态：平滑 + 丢靶保持 + 跟踪窗。"""

    def __init__(self):
        self.u = 0.0
        self.v = 0.0
        self.w = 0.0
        self.h = 0.0
        self.have = False
        self.lost = 0
        self.meas = None       # (px, x, y, w, h, long_side, aspect,
                               #  density, ins, outs)

    def roi(self):
        r = int(max(self.w, self.h) * 0.6) + TRACK_PAD
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

    def update(self, meas):
        if meas is None:
            self.lost += 1
            return
        x, y, w, h = meas[1], meas[2], meas[3], meas[4]
        cu = x + w / 2.0
        cv = y + h / 2.0
        if not self.have:
            self.u, self.v, self.w, self.h = cu, cv, w, h
            self.have = True
        else:
            self.u += SMOOTH * (cu - self.u)
            self.v += SMOOTH * (cv - self.v)
            self.w += SMOOTH * (w - self.w)
            self.h += SMOOTH * (h - self.h)
        self.meas = meas
        self.lost = 0

    def fresh(self):
        return self.have and (self.lost < HOLD_FRAMES)

    def box(self):
        return (int(self.u - self.w / 2.0), int(self.v - self.h / 2.0),
                int(self.w), int(self.h))


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
        for th in SPOT_THRESHOLDS:
            for b in img.find_blobs([th], roi=roi, merge=True,
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
                bx, by = norm_bbox(b[0], b[1], bw, bh, roi)
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
            # 未锁定时：光斑只要在搜索窗里、连续 SPOT_LEARN_FRAMES 帧都出现在
            # 同一个位置（±SPOT_NEAR_PX），就把它记成光轴点。位置突然跳变则重新数。
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


def print_l_diag(img, roi):
    """丢靶时的亮度诊断，用来判断阈值该往哪调。"""
    try:
        h = img.get_histogram(roi=roi)
        print("  [诊断] L均=%.0f 中=%.0f 大=%.0f Otsu=%d"
              % (h.l_mean(), h.l_median(), h.l_max(),
                 h.get_threshold().l_value()))
    except Exception as e:
        print("  [诊断] 直方图不可用: %s" % e)


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
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
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
    sum_tgt = 0
    sum_spot = 0
    sum_show = 0
    sum_n = 0

    print("=" * 68)
    print("vision_check v11: 靶纸=白纸亮块+四周暗框  亮度阈值=%d/%d"
          % (PAPER_TH, PAPER_TH_ALT))
    print("  跟踪窗=方框+%dpx  对比下限=%d  距离=%.0f x %.3fm / 长边px"
          % (TRACK_PAD, PAPER_CONTRAST_MIN, FX_PX, PAPER_LONG_M))
    print("=" * 68)

    try:
        while True:
            os.exitpoint()
            now = time.ticks_ms()

            # ---------- 串口（和 H723 保持心跳，让激光点亮）----------
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
            clock.tick()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            n_frame += 1

            # ---------- 靶纸 ----------
            t0 = time.ticks_ms()
            did_full = False
            cand = None
            n_blob = 0
            dbg = ""
            if tracker.have and tracker.lost < 25:
                cand, n_blob, dbg = find_paper(img, tracker.roi(), PAPER_TH)
                if cand is None and (n_frame % FULL_EVERY) == 0:
                    did_full = True
                    cand, n_blob, dbg = find_paper(
                        img, (0, 0, IMG_W, IMG_H), PAPER_TH)
            else:
                if (n_frame % FULL_EVERY) == 0:
                    did_full = True
                    cand, n_blob, dbg = find_paper(
                        img, (0, 0, IMG_W, IMG_H), PAPER_TH, True)
                    if cand is None:
                        cand, n2, dbg2 = find_paper(
                            img, (0, 0, IMG_W, IMG_H), PAPER_TH_ALT, True)
                        n_blob += n2
                        dbg = dbg + dbg2
            t_tgt = time.ticks_diff(time.ticks_ms(), t0)

            tracker.update(cand)
            if cand is not None:
                n_tgt += 1

            # ---------- 光斑 ----------
            t0 = time.ticks_ms()
            spot = spot_det.detect(img)
            t_spot = time.ticks_diff(time.ticks_ms(), t0)
            if spot is not None:
                n_spot += 1

            # 丢靶很久时打一次亮度诊断（帮我们判断阈值往哪调）
            if tracker.lost == 8 or (tracker.lost > 30 and
                                     tracker.lost % 60 == 0):
                print_l_diag(img, (0, 0, IMG_W, IMG_H))

            # ---------- 画 ----------
            if tracker.fresh():
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

            t0 = time.ticks_ms()
            Display.show_image(img)
            t_show = time.ticks_diff(time.ticks_ms(), t0)
            del img

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
                    px, x, y, w, h, long_side, aspect, density, ins, outs = \
                        tracker.meas
                    dist_m = FX_PX * PAPER_LONG_M / max(1.0, long_side)
                    line += ("靶=纸 靶心=(%.0f,%.0f) 长边=%.0fpx 比%.2f "
                             "密%.2f 对比=%.0f 距离≈%.2fm"
                             % (tracker.u, tracker.v, long_side, aspect,
                                density, ins - outs, dist_m))
                else:
                    line += "靶=未检出%s 亮块=%d %s" % (
                        ("(全图)" if did_full else "(跟踪窗)"), n_blob, dbg)
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
                    line += " | ms 目标%.0f 光斑%.0f 显示%.0f" % (
                        sum_tgt / float(sum_n), sum_spot / float(sum_n),
                        sum_show / float(sum_n))
                line += " | 靶%d/%d 斑%d" % (n_tgt, n_frame, n_spot)
                print(line)
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
