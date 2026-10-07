# -*- coding: utf-8 -*-
"""t4_track.py —— 靶框 + 激光光斑 联合检测（IDE 里能看画面、终端出误差）

这是"瞄准闭环"的离线预演：只检测 + 显示 + 打印误差，**不追**（偏置恒为 0）。
激光通过 H723 打开（AIM 模式 + LASER_ON），云台只会保持稳定、不会去追靶。

画面里会画：
  绿框       = 黑胶带靶框
  红十字     = 靶框中心（靶心）
  白框       = 光斑搜索 ROI（同轴激光的位置是固定的，只在这一小块里找）
  黄圈+黄叉  = 检出的激光光斑

终端每秒打印一行：
  目标(中心) / 光斑 / 误差(px) / 误差(度) / FPS

注意（都是这块板子踩过的坑，这里已经绕开）：
  · MicroPython 的 math 没有 hypot → 全部用 sqrt
  · bytearray 不能切片删除 → 解析器用重新切片
  · 显示要用 ST7701 + to_ide=True，IDE 帧缓冲里才有画面
"""
import os
import time
import math

# ============================ 参数 ============================
IMG_W = 640
IMG_H = 480
DISPLAY_MODE = "LCD"       # LCD(ST7701+to_ide, IDE 里有画面) | VIRT | OFF

DET_SCALE = 2              # 检测在 1/2 分辨率(320x240)上做 → 快 4 倍
ADAPT_BLOCK = 31
ADAPT_C = 7
POLY_EPS = 0.02
MIN_AREA_RATIO = 0.02      # 框面积/画面面积
ASPECT_MIN = 1.15
ASPECT_MAX = 2.40

SPOT_ROI_HALF = 130        # 光斑搜索半径（以图像中心为初值）
SPOT_MIN_AREA = 4
SPOT_MAX_AREA = 4000

FX_PX = 430.0              # 像素焦距（640 宽时的估计值，标定后改成实测）

UART_BAUD = 115200
HEARTBEAT_MS = 200
AIM_MS = 20                # AIM 发送周期（50Hz）
PRINT_MS = 1000
# ==============================================================

SOF = b"\xAA\x55"
MSG_AIM = 0x10
MSG_MODE = 0x11
MSG_HEARTBEAT = 0x12
MSG_SET_ZERO = 0x13
MSG_GIMBAL_STATE = 0x90
MSG_ACK = 0x91
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
    y = max(-32768, min(32767, int(round(yaw_deg * 100.0))))
    p = max(-32768, min(32767, int(round(pitch_deg * 100.0))))
    return ustruct.pack("<hhBB", y, p, flags & 0xFF,
                        max(0, min(255, int(quality))))


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
                    self.buf = self.buf[-1:]      # 不能用 del 切片
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
    try:
        from ybUtils.YbUart import YbUart
        return YbUart(baudrate=UART_BAUD), "YbUart"
    except Exception as e:
        print("YbUart 打不开(%s)，改用 machine.UART" % e)
    from machine import UART
    return UART(1, baudrate=UART_BAUD), "UART1"


# ----------------------------------------------------------------------
# 视觉：靶框（黑胶带）
# ----------------------------------------------------------------------
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

    def dist(ax, ay, bx, by):                    # 没有 math.hypot
        return math.sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by))

    sides = [dist(x1, y1, x0, y0), dist(x2, y2, x1, y1),
             dist(x3, y3, x2, y2), dist(x0, y0, x3, y3)]
    a_len = (sides[0] + sides[2]) / 2.0
    b_len = (sides[1] + sides[3]) / 2.0
    long_side = max(a_len, b_len)
    short_side = max(1.0, min(a_len, b_len))
    return (cx, cy), area, long_side / short_side, long_side


def detect_target(img_np):
    """在 1/DET_SCALE 分辨率上找黑框，坐标乘回去。返回 dict 或 None。"""
    import cv2
    h, w = img_np.shape[0], img_np.shape[1]
    sw, sh = w // DET_SCALE, h // DET_SCALE
    small = cv2.resize(img_np, (sw, sh))
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    bin_img = cv2.adaptiveThreshold(gray, 255,
                                    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                    cv2.THRESH_BINARY_INV,
                                    ADAPT_BLOCK, ADAPT_C)
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    closed = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, k)
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    img_area = float(sw * sh)
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
        # 注意：K230 的 cv2 里 approxPolyDP 返回的点是 [x, y]（不是 OpenCV 的 (N,1,2)），
        # 这里两种形状都兼容一下
        pts = []
        for p in approx:
            try:
                px, py = int(p[0][0]), int(p[0][1])
            except TypeError:
                px, py = int(p[0]), int(p[1])
            pts.append((px * DET_SCALE, py * DET_SCALE))
        q = order_corners(pts)
        center, qarea, aspect, long_side = quad_center_area_aspect(q)
        if aspect < ASPECT_MIN or aspect > ASPECT_MAX:
            continue
        if best is None or qarea > best[1]:
            best = (q, qarea, center, aspect, long_side)
    if best is None:
        return None
    q, area, center, aspect, long_side = best
    return {"quad": q, "center": center, "area": area, "aspect": aspect,
            "long_side": long_side, "n": n}


# ----------------------------------------------------------------------
# 视觉：激光光斑（暖色优势：min(R,B) - G 足够大，且整体够亮）
# ----------------------------------------------------------------------
def detect_spot(img_np, cx, cy, half):
    import cv2
    h, w = img_np.shape[0], img_np.shape[1]
    x0 = max(0, int(cx) - half)
    y0 = max(0, int(cy) - half)
    x1 = min(w, int(cx) + half)
    y1 = min(h, int(cy) + half)
    # 整幅图做阈值（K230 上 numpy 切片不可靠），再用 ROI 只挑框内的连通域
    c0, g, c2 = cv2.split(img_np)
    gap = cv2.subtract(c0, g)
    gap2 = cv2.subtract(c2, g)
    m1 = cv2.threshold(gap, 6, 255, cv2.THRESH_BINARY)[1]
    m2 = cv2.threshold(gap2, 6, 255, cv2.THRESH_BINARY)[1]
    mg = cv2.threshold(g, 110, 255, cv2.THRESH_BINARY)[1]
    mask = cv2.bitwise_and(cv2.bitwise_and(m1, m2), mg)
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
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
        if max(bw, bh) * 1.0 / min(bw, bh) > 3.0:
            continue
        ccx = x + bw / 2.0
        ccy = y + bh / 2.0
        if (ccx < x0) or (ccx > x1) or (ccy < y0) or (ccy > y1):
            continue                      # ROI 门控
        if best is None or area > best[0]:
            best = (area, ccx, ccy, (x, y, bw, bh))
    if best is None:
        return None
    area, ccx, ccy, rect = best
    return {"uv": (ccx, ccy), "area": area, "rect": rect}


# ----------------------------------------------------------------------
def main():
    from media.sensor import Sensor, CAM_CHN_ID_0
    from media.display import Display
    from media.media import MediaManager

    uart, uart_name = open_uart()
    parser = FrameParser()
    print("串口: %s @%d" % (uart_name, UART_BAUD))

    sensor = Sensor()
    sensor.reset()
    time.sleep_ms(100)
    sensor.set_framesize(width=IMG_W, height=IMG_H, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)

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

    # 光斑搜索中心：先用图像中心，检出后自适应
    spot_u = IMG_W / 2.0
    spot_v = IMG_H / 2.0
    learned = False

    ready = False
    armed = False
    t_hb = time.ticks_ms()
    t_aim = time.ticks_ms()
    t_print = time.ticks_ms()
    n_frame = 0
    tgt_hit = 0
    spot_hit = 0

    print("=" * 60)
    print("t4_track: 靶框 + 光斑 联合检测（不追靶，只显示误差）")
    print("  显示=%s  检测分辨率=%dx%d" % (DISPLAY_MODE, IMG_W // DET_SCALE,
                                            IMG_H // DET_SCALE))
    print("  激光由 H723 打开（AIM + LASER_ON，偏置 0）")
    print("=" * 60)

    try:
        while True:
            os.exitpoint()
            now = time.ticks_ms()

            # ---------- 串口：收遥测 / 发心跳 / 发 AIM ----------
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
                    uart.write(build_frame(MSG_MODE, bytes([2, 0])))   # AIM
                    armed = True
                    print("H723 READY -> SET_ZERO + AIM（激光打开，偏置 0）")
                except Exception as e:
                    print("发 MODE 失败: %s" % e)

            if armed and time.ticks_diff(now, t_aim) >= AIM_MS:
                t_aim = now
                try:
                    # 偏置 0 + LASER_ON + AIM_VALID：只开激光、不乱动
                    uart.write(build_frame(MSG_AIM,
                                           pack_aim(0.0, 0.0, 0x03, 200)))
                except Exception:
                    pass

            # ---------- 视觉 ----------
            clock.tick()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            img_np = img.to_numpy_ref()
            n_frame += 1

            tgt = detect_target(img_np)
            spot = detect_spot(img_np, spot_u, spot_v, SPOT_ROI_HALF)
            if spot is not None:
                su, sv = spot["uv"]
                if not learned:
                    spot_u, spot_v = su, sv
                    learned = True
                else:                       # 慢速自适应（同轴光斑位置固定）
                    du = (su - spot_u) * 0.05
                    dv = (sv - spot_v) * 0.05
                    spot_u += max(-4.0, min(4.0, du))
                    spot_v += max(-4.0, min(4.0, dv))

            # ---------- 画 ----------
            if tgt is not None:
                q = tgt["quad"]
                for i in range(4):
                    a, b = q[i], q[(i + 1) % 4]
                    img.draw_line(a[0], a[1], b[0], b[1],
                                  color=(0, 255, 0), thickness=2)
                img.draw_cross(int(tgt["center"][0]), int(tgt["center"][1]),
                               color=(255, 0, 0), size=14, thickness=2)
            img.draw_rectangle(int(spot_u - SPOT_ROI_HALF),
                               int(spot_v - SPOT_ROI_HALF),
                               SPOT_ROI_HALF * 2, SPOT_ROI_HALF * 2,
                               color=(255, 255, 255), thickness=1)
            if spot is not None:
                su, sv = spot["uv"]
                img.draw_circle(int(su), int(sv), 10, color=(255, 255, 0),
                                thickness=2)
                img.draw_cross(int(su), int(sv), color=(255, 255, 0),
                               size=8, thickness=2)

            # ---------- 每秒打印 ----------
            if time.ticks_diff(now, t_print) >= PRINT_MS:
                t_print = now
                msg = "FPS=%.1f" % clock.fps()
                if tgt is None:
                    msg += "  靶框: 未检出(候选%d)" % (tgt["n"] if tgt else 0)
                else:
                    tgt_hit += 1
                    cu, cv = tgt["center"]
                    msg += "  靶心=(%.0f,%.0f) 长边=%.0fpx 比例=%.2f" % (
                        cu, cv, tgt["long_side"], tgt["aspect"])
                    if spot is not None:
                        spot_hit += 1
                        eu = cu - su
                        ev = cv - sv
                        err = math.sqrt(eu * eu + ev * ev)
                        deg = err / FX_PX * 57.29578
                        msg += ("  光斑=(%.0f,%.0f) 误差=(%.0f,%.0f)px"
                                " |%.0fpx|=%.2f°"
                                % (su, sv, eu, ev, err, deg))
                    else:
                        msg += "  光斑: 未检出"
                msg += "  [框%d/%d 斑%d]" % (tgt_hit, n_frame, spot_hit)
                print(msg)

            if display_ok:
                Display.show_image(img)
            time.sleep_ms(2)
    except KeyboardInterrupt:
        print("用户停止")
    finally:
        try:
            uart.write(build_frame(MSG_MODE, bytes([1, 0])))   # STAB: 关激光
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
