# -*- coding: utf-8 -*-
"""vision_check.py —— 视觉验证（靶框 + 激光光斑 + 误差 + 距离）

v6：回到"黑胶带框"方案，但**不再用慢的 find_rects**，改用 C 加速的
     原生 find_blobs（连通域）——这是把国一方案的"固定阈值二值化 + 轮廓"
     在本固件上等价实现（本固件没有 cv2）：

  · 靶框 = **黑胶带这个暗色连通域**（环）
      用 LAB 暗阈值找块；黑胶带在纸/白瓷砖背景上都很暗，固定阈值很稳。
      环的质心 = 框中心 = 靶心（对称结构，质心就是几何中心）。
      再用国一那套几何校验：长宽比、对边比、内角（有 min_corners 时）。
  · 光斑 = ROI 内的"暖色亮块"（R、B 都高于 G；打印的红圈是暗红、白纸是中性，
            都被这个判据排除）。检不到就退回已锁定的光轴点。
  · 距离 = FX_PX × 0.297m(纸长边) ÷ 框长边像素（用卷尺标定一次 FX_PX 即可，
            与靶子远近无关，后面 1.2~2.2m 同样适用）。

画面里只画三样：绿框(靶框)、红叉(靶心)、黄圈/黄叉(光斑)。
"""
import os
import time
import math

# ============================ 参数 ============================
IMG_W = 640
IMG_H = 480
DISPLAY_QUALITY = 50       # IDE 画面质量（越小越流畅）

# --- 黑胶带：暗色连通域（LAB）---
TAPE_THRESHOLD = (0, 45, -60, 60, -60, 60)
TAPE_MIN_AREA = 400        # 像素面积下限（太远时框会变小，可往下调）
TAPE_MAX_AREA_RATIO = 0.60
TAPE_ASPECT_MIN = 1.15     # 胶带框 297x180 → 1.65
TAPE_ASPECT_MAX = 2.60
TAPE_SIDE_RATIO_TOL = 0.45 # 对边长度相对差上限（国一用的 0.4）
TAPE_ANGLE_TOL = 30.0      # 内角偏离 90° 的容差（国一用的 30°）

# --- 激光光斑：暖色亮块（LAB）---
SPOT_THRESHOLDS = [
    (55, 100, 8, 70, -20, 60),     # 暖色亮斑（主判据）
    (85, 100, -20, 40, -20, 60),   # 过曝白芯
]
SPOT_MIN_AREA = 3
SPOT_MAX_AREA = 3000
SPOT_MAX_ASPECT = 3.0
SPOT_CX = -1               # 想手动指定就填像素坐标（-1 = 自动学习并锁定）
SPOT_CY = -1
SPOT_ROI_HALF = 90
SPOT_LEARN_FRAMES = 15
SPOT_NEAR_PX = 25

# --- 距离（标定一次 FX_PX 即可，与远近无关）---
FX_PX = 440.0              # 像素焦距（640 宽）；用卷尺量一次距离校准
PAPER_LONG_M = 0.297       # A4 长边 297mm（胶带贴在纸边，外边长≈297mm）

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


def dist2(ax, ay, bx, by):
    return math.sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by))


def corners_geometry_ok(pts):
    """国一那套几何校验：内角接近 90°、对边长度接近。"""
    if len(pts) != 4:
        return False
    # 角度
    for i in range(4):
        p = pts[i]
        a = pts[(i - 1) % 4]
        b = pts[(i + 1) % 4]
        v1x, v1y = a[0] - p[0], a[1] - p[1]
        v2x, v2y = b[0] - p[0], b[1] - p[1]
        n1 = math.sqrt(v1x * v1x + v1y * v1y)
        n2 = math.sqrt(v2x * v2x + v2y * v2y)
        if n1 < 1e-6 or n2 < 1e-6:
            return False
        c = (v1x * v2x + v1y * v2y) / (n1 * n2)
        if c > 1.0:
            c = 1.0
        elif c < -1.0:
            c = -1.0
        ang = math.acos(c) * 57.29578
        if abs(ang - 90.0) > TAPE_ANGLE_TOL:
            return False
    # 对边长度
    s = [dist2(pts[0][0], pts[0][1], pts[1][0], pts[1][1]),
         dist2(pts[1][0], pts[1][1], pts[2][0], pts[2][1]),
         dist2(pts[2][0], pts[2][1], pts[3][0], pts[3][1]),
         dist2(pts[3][0], pts[3][1], pts[0][0], pts[0][1])]
    for (a, b) in ((0, 2), (1, 3)):
        m = max(s[a], s[b])
        if m < 1e-6:
            return False
        if abs(s[a] - s[b]) / m > TAPE_SIDE_RATIO_TOL:
            return False
    if min(s) < 15:
        return False
    return True


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

    ready = False
    armed = False
    t_hb = time.ticks_ms()
    t_aim = time.ticks_ms()
    t_print = time.ticks_ms()
    n_frame = 0
    n_tgt = 0
    n_spot = 0

    print("=" * 68)
    print("vision_check v6: 黑胶带暗块(find_blobs, C加速) + 暖色光斑")
    print("  胶带阈值=%s 面积≥%d 比例=[%.2f,%.2f]"
          % (str(TAPE_THRESHOLD), TAPE_MIN_AREA, TAPE_ASPECT_MIN,
             TAPE_ASPECT_MAX))
    print("  距离 = %.0f × 0.297 ÷ 框长边像素" % FX_PX)
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

            # ---------- 靶框：黑胶带暗块 ----------
            clock.tick()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            n_frame += 1

            best = None            # (area, cx, cy, long_side_px, aspect)
            n_blob = 0
            list_txt = ""
            for b in img.find_blobs([TAPE_THRESHOLD], merge=True,
                                    area_threshold=TAPE_MIN_AREA,
                                    pixels_threshold=TAPE_MIN_AREA):
                n_blob += 1
                try:
                    area = b.area()
                except Exception:
                    area = b[4]
                if area < TAPE_MIN_AREA:
                    continue
                if area > TAPE_MAX_AREA_RATIO * img_area:
                    continue
                x, y, w, h = b[0], b[1], b[2], b[3]
                aspect = max(w, h) / max(1.0, min(w, h))
                if n_blob <= 3:
                    list_txt += " [%dpx %.1f%% 比=%.2f]" % (area,
                                                            100.0 * area /
                                                            img_area, aspect)
                if aspect < TAPE_ASPECT_MIN or aspect > TAPE_ASPECT_MAX:
                    continue
                # 贴边的大块多半是背景/阴影，不要
                if (x <= 1) or (y <= 1) or (x + w >= IMG_W - 1) or \
                        (y + h >= IMG_H - 1):
                    continue
                # 有 min_corners 就做国一那套几何校验
                ok_geo = True
                corners = None
                try:
                    corners = b.min_corners()
                    ok_geo = corners_geometry_ok(corners)
                except Exception:
                    corners = None
                if not ok_geo:
                    continue
                if best is None or area > best[0]:
                    best = (area, b[5], b[6], max(w, h), aspect, corners)

            # ---------- 光斑：暖色亮块 ----------
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
                if best[5] is not None:
                    c = best[5]
                    for i in range(4):
                        a = c[i]
                        d = c[(i + 1) % 4]
                        img.draw_line(int(a[0]), int(a[1]), int(d[0]),
                                      int(d[1]), color=(0, 255, 0), thickness=2)
                else:
                    bx, by, bw, bh = (int(best[1] - best[3] / 2),
                                      int(best[2] - best[3] / 2),
                                      int(best[3]), int(best[3]))
                    img.draw_rectangle(bx, by, bw, bh, color=(0, 255, 0),
                                       thickness=2)
                img.draw_cross(int(best[1]), int(best[2]), color=(255, 0, 0),
                               size=14, thickness=2)
            if spot is not None:
                img.draw_circle(int(spot[1]), int(spot[2]), 8,
                                color=(255, 255, 0), thickness=2)
                img.draw_cross(int(spot[1]), int(spot[2]),
                               color=(255, 255, 0), size=6, thickness=1)

            # ---------- 打印 ----------
            if time.ticks_diff(now, t_print) >= PRINT_MS:
                t_print = now
                line = "FPS=%.1f 暗块=%d%s" % (clock.fps(), n_blob, list_txt)
                if best is None:
                    line += "  靶框: 未检出"
                else:
                    n_tgt += 1
                    dist_m = FX_PX * PAPER_LONG_M / max(1.0, best[3])
                    line += ("  靶心=(%.0f,%.0f) 长边=%.0fpx 比=%.2f "
                             "距离≈%.2fm 面积=%.0fpx"
                             % (best[1], best[2], best[3], best[4], dist_m,
                                best[0]))
                    if spot is not None:
                        n_spot += 1
                        eu, ev = best[1] - spot[1], best[2] - spot[2]
                        err = math.sqrt(eu * eu + ev * ev)
                        line += ("  光斑=(%.0f,%.0f) 误差=(%.0f,%.0f)px "
                                 "=%.1fpx(%.2f°)"
                                 % (spot[1], spot[2], eu, ev, err,
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
