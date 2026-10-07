# -*- coding: utf-8 -*-
"""vision_check.py —— 视觉验证（靶框 + 激光光斑 + 误差），只检测不追靶

这块板子的固件是**精简版**：没有 cv2、math 里连 hypot 都没有，
所以本脚本只用原生 image 模块：find_rects / find_blobs / draw_*。

画面里：
  蓝框       = find_rects 找到的所有候选（用于调阈值）
  绿框/红叉  = 通过筛选的靶框与靶心
  白框       = 光斑搜索 ROI（同轴激光位置固定，只在这一小块里找）
  黄圈/黄叉  = 检出的激光光斑

终端每秒一行：FPS / 候选数 / 靶心 / 尺寸比例 / 光斑 / 误差(px, °)

激光由脚本通过 H723 打开（AIM 模式 + LASER_ON，偏置恒 0 → 云台只保持稳定）。
"""
import os
import time
import math

# ============================ 参数 ============================
IMG_W = 640
IMG_H = 480
DISPLAY_MODE = "LCD"       # LCD(ST7701+to_ide，IDE 里有画面) | VIRT | OFF

RECT_THRESHOLD = 20000     # find_rects 阈值：找不到就把数往小调，误检多就往大调
RECT_XGRAD = 8
RECT_YGRAD = 8
MIN_AREA_RATIO = 0.015     # 框面积 / 画面面积 下限
ASPECT_MIN = 1.20          # 靶纸 297x180 的长短边比 ≈1.65
ASPECT_MAX = 2.40

SPOT_ROI_HALF = 130        # 光斑搜索半径
SPOT_THRESHOLDS = [        # LAB 阈值（可多组）：亮且偏暖
    (60, 100, 8, 60, -10, 60),
    (85, 100, -20, 40, -20, 60),
]
SPOT_MIN_AREA = 3
SPOT_MAX_AREA = 3000
SPOT_MAX_ASPECT = 3.0

FX_PX = 430.0              # 像素焦距（640 宽时的估计；标定后改实测值）

UART_BAUD = 115200
HEARTBEAT_MS = 200
AIM_MS = 20
PRINT_MS = 1000
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
                    self.buf = self.buf[-1:]        # 不能用 del 切片
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


def open_uart():
    from ybUtils.YbUart import YbUart
    return YbUart(baudrate=UART_BAUD)


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

    def dist(ax, ay, bx, by):                    # 没有 math.hypot
        return math.sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by))

    sides = [dist(x1, y1, x0, y0), dist(x2, y2, x1, y1),
             dist(x3, y3, x2, y2), dist(x0, y0, x3, y3)]
    a_len = (sides[0] + sides[2]) / 2.0
    b_len = (sides[1] + sides[3]) / 2.0
    long_side = max(a_len, b_len)
    return (cx, cy), area, \
        long_side / max(1.0, min(a_len, b_len)), long_side


def main():
    from media.sensor import Sensor, CAM_CHN_ID_0
    from media.display import Display
    from media.media import MediaManager

    uart = open_uart()
    parser = FrameParser()
    print("串口: YbUart @%d" % UART_BAUD)

    sensor = Sensor()
    sensor.reset()
    time.sleep_ms(100)
    sensor.set_framesize(width=IMG_W, height=IMG_H, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

    display_ok = True
    try:
        if DISPLAY_MODE == "LCD":
            Display.init(Display.ST7701, width=IMG_W, height=IMG_H,
                         to_ide=True)
        elif DISPLAY_MODE == "VIRT":
            Display.init(Display.VIRT, width=IMG_W, height=IMG_H, fps=30)
        else:
            display_ok = False
    except Exception as e:
        print("显示初始化失败(%s)，继续无显示运行" % e)
        display_ok = False
    MediaManager.init()
    sensor.run()
    clock = time.clock()

    img_area = float(IMG_W * IMG_H)
    spot_u = IMG_W / 2.0
    spot_v = IMG_H / 2.0
    learned = False
    ready = False
    armed = False
    t_hb = time.ticks_ms()
    t_aim = time.ticks_ms()
    t_print = time.ticks_ms()
    n_frame = 0
    n_tgt = 0
    n_spot = 0

    print("=" * 62)
    print("vision_check: 靶框+光斑检测（不追靶）  阈值=%d  比例=[%.2f,%.2f]"
          % (RECT_THRESHOLD, ASPECT_MIN, ASPECT_MAX))
    print("=" * 62)

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

            # ---------- 视觉 ----------
            clock.tick()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            n_frame += 1

            best = None
            n_rect = 0
            list_txt = ""
            for r in img.find_rects(threshold=RECT_THRESHOLD,
                                    x_gradient=RECT_XGRAD,
                                    y_gradient=RECT_YGRAD):
                n_rect += 1
                pts = []
                for p in r.corners():
                    pts.append((int(p[0]), int(p[1])))
                q = order_corners(pts)
                center, area, aspect, long_side = quad_metrics(q)
                # 所有候选都画出来（蓝色），方便调阈值
                img.draw_rectangle(r.rect(), color=(80, 80, 255), thickness=1)
                if n_rect <= 3:
                    list_txt += " [%.1f%% 比=%.2f]" % (100.0 * area / img_area,
                                                       aspect)
                if area < MIN_AREA_RATIO * img_area:
                    continue
                if aspect < ASPECT_MIN or aspect > ASPECT_MAX:
                    continue
                if best is None or area > best[1]:
                    best = (q, area, center, aspect, long_side)

            spot = None
            for th in SPOT_THRESHOLDS:
                for b in img.find_blobs([th],
                                        roi=(int(spot_u - SPOT_ROI_HALF),
                                             int(spot_v - SPOT_ROI_HALF),
                                             SPOT_ROI_HALF * 2,
                                             SPOT_ROI_HALF * 2),
                                        merge=True,
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
            if spot is not None:
                su, sv = spot[1], spot[2]
                if not learned:
                    spot_u, spot_v = su, sv
                    learned = True
                else:
                    spot_u += max(-4.0, min(4.0, (su - spot_u) * 0.05))
                    spot_v += max(-4.0, min(4.0, (sv - spot_v) * 0.05))

            # ---------- 画 ----------
            if best is not None:
                q = best[0]
                for i in range(4):
                    a, b = q[i], q[(i + 1) % 4]
                    img.draw_line(a[0], a[1], b[0], b[1],
                                  color=(0, 255, 0), thickness=2)
                img.draw_cross(int(best[2][0]), int(best[2][1]),
                               color=(255, 0, 0), size=14, thickness=2)
            img.draw_rectangle(int(spot_u - SPOT_ROI_HALF),
                               int(spot_v - SPOT_ROI_HALF),
                               SPOT_ROI_HALF * 2, SPOT_ROI_HALF * 2,
                               color=(255, 255, 255), thickness=1)
            if spot is not None:
                img.draw_rectangle(spot[3], color=(0, 255, 0), thickness=2)
                img.draw_circle(int(spot[1]), int(spot[2]), 8,
                                color=(255, 255, 0), thickness=2)

            # ---------- 打印 ----------
            if time.ticks_diff(now, t_print) >= PRINT_MS:
                t_print = now
                line = "FPS=%.1f 候选=%d%s" % (clock.fps(), n_rect, list_txt)
                if best is None:
                    line += "  靶框: 未检出"
                else:
                    n_tgt += 1
                    cu, cv = best[2]
                    line += "  靶心=(%.0f,%.0f) 长边=%.0fpx 比=%.2f" % (
                        cu, cv, best[4], best[3])
                    if spot is not None:
                        n_spot += 1
                        eu, ev = cu - spot[1], cv - spot[2]
                        err = math.sqrt(eu * eu + ev * ev)
                        line += ("  光斑=(%.0f,%.0f) 误差=(%.0f,%.0f)px "
                                 "=%.1fpx(%.2f°)" % (spot[1], spot[2], eu, ev,
                                                     err,
                                                     err / FX_PX * 57.29578))
                    else:
                        line += "  光斑: 未检出"
                line += "  [靶%d/%d 斑%d]" % (n_tgt, n_frame, n_spot)
                print(line)

            if display_ok:
                Display.show_image(img)
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
        if display_ok:
            try:
                Display.deinit()
                os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
                time.sleep_ms(100)
                MediaManager.deinit()
            except Exception:
                pass
        print("已退出（激光已关闭）")


main()
