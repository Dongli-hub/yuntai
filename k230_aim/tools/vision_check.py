# -*- coding: utf-8 -*-
"""vision_check.py —— 视觉验证（靶框 + 激光光斑 + 误差 + 距离），只检测不追靶

v4 关键改动：**用 ROI 跟踪**。
  find_rects 支持 roi 参数 —— 只在"上一帧靶心周围的小窗口"里搜矩形，
  这样既保留原生 find_rects 的**真实四角点**（斜视时绿框贴着四边形走），
  又避免了全图搜索的卡顿（之前卡就是全图 find_rects 太慢）。
  丢靶时每隔几帧做一次全图搜索兜底。

画面里只画三样：
  绿线框    = 贴着黑胶带框的四边形
  红叉      = 靶心（四边形对角线交点 = 投影中心）
  黄圈+黄叉 = 激光光斑

终端每秒一行：FPS / 候选 / 靶心 / 长边px / 比例 / **距离估算** / 光斑 / 误差(px,°)

距离估算：靶纸长边实际 297mm，按小孔成像
    distance_m = FX_PX * 0.297 / 长边像素
（FX_PX 先用 440，等现场用卷尺量出准确距离后再校准一次即可）
"""
import os
import time
import math

# ============================ 参数 ============================
IMG_W = 640
IMG_H = 480
DISPLAY_QUALITY = 50       # IDE 画面质量（越小越流畅）

# --- 靶框（原生 find_rects，真实四角点）---
RECT_THRESHOLD = 12000     # 找不到就往小调；误检多就往大调
RECT_XGRAD = 8
RECT_YGRAD = 8
MIN_AREA_RATIO = 0.015
ASPECT_MIN = 1.15          # 靶纸 297x180 → 1.65（用真实边长算，斜视也准）
ASPECT_MAX = 2.60

TRACK_ROI_HALF = 120       # 跟踪窗口半径（找到靶后只在这块里搜）
FULL_SEARCH_EVERY = 6      # 丢靶期间：每隔几帧做一次全图搜索

# --- 激光光斑（原生 find_blobs，LAB 阈值）---
SPOT_THRESHOLDS = [
    (85, 100, -20, 40, -20, 60),   # 过曝白芯
    (60, 100, 8, 60, -10, 60),     # 暖色亮斑
]
SPOT_MIN_AREA = 3
SPOT_MAX_AREA = 3000
SPOT_MAX_ASPECT = 3.0
SPOT_CX = -1               # 想手动指定就填像素坐标（-1 = 自动学习并锁定）
SPOT_CY = -1
SPOT_ROI_HALF = 90
SPOT_LEARN_FRAMES = 15
SPOT_NEAR_PX = 25

# --- 距离估算 ---
FX_PX = 440.0              # 像素焦距（640 宽）：0.6m 处长边约 219px → fx≈442
PAPER_LONG_M = 0.297       # 靶纸长边实际长度（米）

UART_BAUD = 115200
HEARTBEAT_MS = 200
AIM_MS = 20
PRINT_MS = 1000
GC_EVERY = 10
# ==============================================================

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


def order_corners(pts):
    cx = sum(p[0] for p in pts) / 4.0
    cy = sum(p[1] for p in pts) / 4.0
    try:
        ang = [(math.atan2(p[1] - cy, p[0] - cx), p) for p in pts]
        ang.sort(key=lambda t: t[0])
        return [list(t[1]) for t in ang]
    except AttributeError:
        s = sorted(pts, key=lambda p: p[0])
        left = sorted(s[:2], key=lambda p: p[1])
        right = sorted(s[2:], key=lambda p: p[1])
        return [list(left[0]), list(right[0]), list(right[1]), list(left[1])]


def quad_metrics(q):
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

    def dist(ax, ay, bx, by):
        return math.sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by))

    sides = [dist(x1, y1, x0, y0), dist(x2, y2, x1, y1),
             dist(x3, y3, x2, y2), dist(x0, y0, x3, y3)]
    a_len = (sides[0] + sides[2]) / 2.0
    b_len = (sides[1] + sides[3]) / 2.0
    long_side = max(a_len, b_len)
    return (cx, cy), area, long_side / max(1.0, min(a_len, b_len)), long_side


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

    img_area = float(IMG_W * IMG_H)
    if (SPOT_CX >= 0) and (SPOT_CY >= 0):
        spot_u, spot_v = float(SPOT_CX), float(SPOT_CY)
        locked = True
        print("光斑 ROI 由参数指定: (%.0f, %.0f)" % (spot_u, spot_v))
    else:
        spot_u, spot_v = IMG_W / 2.0, IMG_H / 2.0
        locked = False
    learn_u, learn_v, learn_n = spot_u, spot_v, 0

    tgt_u, tgt_v = IMG_W / 2.0, IMG_H / 2.0
    have_tgt = False
    lost_n = 0

    ready = False
    armed = False
    t_hb = time.ticks_ms()
    t_aim = time.ticks_ms()
    t_print = time.ticks_ms()
    n_frame = 0
    n_tgt = 0
    n_spot = 0

    print("=" * 66)
    print("vision_check v4: ROI跟踪 + find_rects(四角点) + find_blobs(光斑)")
    print("  跟踪窗±%d  阈值=%d  比例=[%.2f,%.2f]  FX=%.0fpx"
          % (TRACK_ROI_HALF, RECT_THRESHOLD, ASPECT_MIN, ASPECT_MAX, FX_PX))
    print("=" * 66)

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
                                print("  H723: %s" % payload.decode("utf-8"))
                            except Exception:
                                pass
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

            # ---------- 靶框（ROI 跟踪 + 丢靶时全图兜底）----------
            clock.tick()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            n_frame += 1

            if have_tgt and (lost_n < FULL_SEARCH_EVERY * 2):
                x0 = int(max(0, tgt_u - TRACK_ROI_HALF))
                y0 = int(max(0, tgt_v - TRACK_ROI_HALF))
                w0 = int(min(IMG_W, tgt_u + TRACK_ROI_HALF)) - x0
                h0 = int(min(IMG_H, tgt_v + TRACK_ROI_HALF)) - y0
                search_roi = (x0, y0, w0, h0)
            else:
                x0, y0 = 0, 0
                search_roi = (0, 0, IMG_W, IMG_H)

            best = None
            n_rect = 0
            list_txt = ""
            for r in img.find_rects(roi=search_roi, threshold=RECT_THRESHOLD,
                                    x_gradient=RECT_XGRAD,
                                    y_gradient=RECT_YGRAD):
                n_rect += 1
                pts_rel = []
                pts_abs = []
                for p in r.corners():
                    px, py = int(p[0]), int(p[1])
                    pts_rel.append((px, py))
                    pts_abs.append((px + x0, py + y0))
                if len(pts_rel) != 4:
                    continue
                # 不同固件版本里 ROI 内的坐标可能是"相对 ROI"也可能是"全图"，
                # 用"靶心是否落在搜索窗内"来判断该用哪种解释
                pts = pts_rel
                if search_roi != (0, 0, IMG_W, IMG_H):
                    c_abs, _, _, _ = quad_metrics(order_corners(pts_abs))
                    if (x0 - 20) <= c_abs[0] <= (x0 + w0 + 20) and \
                            (y0 - 20) <= c_abs[1] <= (y0 + h0 + 20):
                        pts = pts_abs
                q = order_corners(pts)
                center, area, aspect, long_side = quad_metrics(q)
                if n_rect <= 3:
                    list_txt += " [%.1f%% 比=%.2f]" % (100.0 * area / img_area,
                                                       aspect)
                if area < MIN_AREA_RATIO * img_area:
                    continue
                if aspect < ASPECT_MIN or aspect > ASPECT_MAX:
                    continue
                if best is None or area > best[1]:
                    best = (q, area, center, aspect, long_side)

            if best is not None:
                tgt_u, tgt_v = best[2]
                have_tgt = True
                lost_n = 0
            else:
                lost_n += 1

            # ---------- 光斑 ----------
            spot = None
            roi = (int(spot_u - SPOT_ROI_HALF), int(spot_v - SPOT_ROI_HALF),
                   SPOT_ROI_HALF * 2, SPOT_ROI_HALF * 2)
            for th in SPOT_THRESHOLDS:
                for b in img.find_blobs([th], roi=roi, merge=True,
                                        pixels_threshold=SPOT_MIN_AREA,
                                        area_threshold=SPOT_MIN_AREA):
                    try:
                        area = b.area()
                    except Exception:
                        area = b[4]
                    if area < SPOT_MIN_AREA or area > SPOT_MAX_AREA:
                        continue
                    bw, bh = b[2], b[3]
                    if bw < 1 or bh < 1:
                        continue
                    if max(bw, bh) * 1.0 / min(bw, bh) > SPOT_MAX_ASPECT:
                        continue
                    if spot is None or area > spot[0]:
                        spot = (area, b[5], b[6], (b[0], b[1], bw, bh))

            if (spot is not None) and (not locked):
                far = (abs(spot[1] - spot_u) > SPOT_NEAR_PX) or \
                      (abs(spot[2] - spot_v) > SPOT_NEAR_PX)
                if far:
                    learn_n = 0
                else:
                    learn_n += 1
                    learn_u += (spot[1] - learn_u) / float(learn_n)
                    learn_v += (spot[2] - learn_v) / float(learn_n)
                    if learn_n >= SPOT_LEARN_FRAMES:
                        spot_u, spot_v = learn_u, learn_v
                        locked = True
                        print("光斑位置已锁定: (%.1f, %.1f)" % (spot_u, spot_v))
            elif spot is None:
                learn_n = 0

            # ---------- 画 ----------
            if best is not None:
                q = best[0]
                for i in range(4):
                    a, b = q[i], q[(i + 1) % 4]
                    img.draw_line(a[0], a[1], b[0], b[1],
                                  color=(0, 255, 0), thickness=2)
                img.draw_cross(int(best[2][0]), int(best[2][1]),
                               color=(255, 0, 0), size=14, thickness=2)
            if spot is not None:
                img.draw_circle(int(spot[1]), int(spot[2]), 8,
                                color=(255, 255, 0), thickness=2)
                img.draw_cross(int(spot[1]), int(spot[2]),
                               color=(255, 255, 0), size=6, thickness=1)

            # ---------- 打印 ----------
            if time.ticks_diff(now, t_print) >= PRINT_MS:
                t_print = now
                line = "FPS=%.1f 候选=%d%s" % (clock.fps(), n_rect, list_txt)
                if best is None:
                    line += "  靶框: 未检出"
                else:
                    n_tgt += 1
                    dist_m = FX_PX * PAPER_LONG_M / max(1.0, best[4])
                    line += "  靶心=(%.0f,%.0f) 长边=%.0fpx 比=%.2f 距离≈%.2fm" \
                        % (best[2][0], best[2][1], best[4], best[3], dist_m)
                    if spot is not None:
                        n_spot += 1
                        eu, ev = best[2][0] - spot[1], best[2][1] - spot[2]
                        err = math.sqrt(eu * eu + ev * ev)
                        line += ("  光斑=(%.0f,%.0f) 误差=(%.0f,%.0f)px "
                                 "=%.1fpx(%.2f°)" % (spot[1], spot[2], eu, ev,
                                                     err,
                                                     err / FX_PX * 57.29578))
                    else:
                        line += "  光斑: 未检出"
                line += "  [靶%d/%d 斑%d]" % (n_tgt, n_frame, n_spot)
                print(line)

            Display.show_image(img)
            del img
            if (n_frame % GC_EVERY) == 0:
                gc.collect()
            time.sleep_ms(2)
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
