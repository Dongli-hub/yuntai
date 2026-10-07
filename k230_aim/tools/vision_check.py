# -*- coding: utf-8 -*-
"""vision_check.py —— 视觉验证（靶心 + 激光光斑 + 误差 + 距离）

v5 换了检测思路（为了速度和斜视稳定性）：

  · **靶心 = 白纸块的中心**（不再去找黑胶带框）
      白纸是个又大又亮的连通域，用 cv_lite 的 C 加速找块一次就出来，
      快、稳、斜视也照样是一个完整白块；纸面同心圆本来就是以纸中心为圆心，
      所以"纸块中心"就是靶心。
  · **光斑 = ROI 内的小亮点**（同一套找块，按面积区分：纸是大块、光斑是小块）
  · 黑胶带框只用来画个参考框（用矩形检测，可选，失败不影响瞄准）

画面里只画三样：
  绿框      = 白纸块（靶面）
  红叉      = 靶心（纸块中心）
  黄圈/黄叉 = 激光光斑

终端每秒一行：FPS / 纸块面积% / 靶心 / 长边px / 比例 / 距离估算 / 光斑 / 误差(px,°)

距离估算：白纸可见部分长边 ≈261mm（A4 297mm 减去上下各 18mm 胶带）
    distance_m = FX_PX * 0.261 / 长边像素
FX_PX 先用 440（0.6m 处胶带框长边≈219px 反推）。用卷尺核对后校准一次即可。
"""
import os
import time
import math

# ============================ 参数 ============================
IMG_W = 640
IMG_H = 480
DISPLAY_QUALITY = 50       # IDE 画面质量（越小越流畅）

# --- 白纸（靶面）：RGB 阈值 [Rmin,Rmax, Gmin,Gmax, Bmin,Bmax] ---
# 白纸是"三通道都亮"的块。太亮/背景也白就把下限往上调。
PAPER_THRESHOLD = [165, 255, 165, 255, 165, 255]
PAPER_MIN_AREA_RATIO = 0.02     # 纸块至少占画面 2%
PAPER_MAX_AREA_RATIO = 0.90     # 太大说明把背景也算进来了
PAPER_ASPECT_MIN = 1.10         # 白纸可见部分 ≈174x261mm → 1.5
PAPER_ASPECT_MAX = 2.20
PAPER_KERNEL = 1

# --- 激光光斑：更亮（过曝白芯）---
SPOT_THRESHOLD = [235, 255, 200, 255, 200, 255]
SPOT_MIN_AREA = 3
SPOT_MAX_AREA = 3000
SPOT_MAX_ASPECT = 3.0
SPOT_CX = -1               # 想手动指定就填像素坐标（-1 = 自动学习并锁定）
SPOT_CY = -1
SPOT_ROI_HALF = 90
SPOT_LEARN_FRAMES = 15
SPOT_NEAR_PX = 25

# --- 距离估算 ---
FX_PX = 440.0              # 像素焦距（640 宽）
PAPER_LONG_M = 0.261       # 白纸可见部分长边（米）

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


def blob_center(b):
    """优先用最小外接矩形的四角交点（就是纸中心），没有就用质心。"""
    try:
        pts = b.min_corners()
        (x0, y0), (x1, y1), (x2, y2), (x3, y3) = pts
        d1x, d1y = x2 - x0, y2 - y0
        d2x, d2y = x3 - x1, y3 - y1
        den = d1x * d2y - d1y * d2x
        if abs(den) > 1e-6:
            t = ((x1 - x0) * d2y - (y1 - y0) * d2x) / den
            return (x0 + t * d1x, y0 + t * d1y)
    except Exception:
        pass
    return (float(b[5]), float(b[6]))


def main():
    from media.sensor import Sensor, CAM_CHN_ID_0
    from media.display import Display
    from media.media import MediaManager
    import cv_lite
    import gc

    from ybUtils.YbUart import YbUart
    uart = YbUart(baudrate=UART_BAUD)
    parser = FrameParser()
    print("串口: YbUart @%d" % UART_BAUD)

    shape = [IMG_H, IMG_W]
    sensor = Sensor(id=2, width=1280, height=960, fps=90)
    sensor.reset()
    time.sleep_ms(100)
    sensor.set_framesize(width=IMG_W, height=IMG_H, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)
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

    print("=" * 66)
    print("vision_check v5: 靶心=白纸块中心(cv_lite找块) + 光斑=ROI内小亮点")
    print("  纸阈值=%s  面积比=[%.2f,%.2f] 比例=[%.2f,%.2f]  FX=%.0f"
          % (str(PAPER_THRESHOLD), PAPER_MIN_AREA_RATIO, PAPER_MAX_AREA_RATIO,
             PAPER_ASPECT_MIN, PAPER_ASPECT_MAX, FX_PX))
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

            # ---------- 一次找块，同时得到白纸和光斑 ----------
            clock.tick()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            img_np = img.to_numpy_ref()
            n_frame += 1

            paper = None
            spot = None

            # 白纸（大块）
            blobs = cv_lite.rgb888_find_blobs(shape, img_np, PAPER_THRESHOLD,
                                              int(PAPER_MIN_AREA_RATIO *
                                                  img_area), PAPER_KERNEL)
            for i in range(0, len(blobs), 4):
                x, y, w, h = blobs[i], blobs[i + 1], blobs[i + 2], blobs[i + 3]
                area = float(w * h)
                if area < PAPER_MIN_AREA_RATIO * img_area:
                    continue
                if area > PAPER_MAX_AREA_RATIO * img_area:
                    continue
                # 贴到画面边缘的大块多半是背景/墙，不要
                if (x <= 1) or (y <= 1) or (x + w >= IMG_W - 1) or \
                        (y + h >= IMG_H - 1):
                    continue
                aspect = max(w, h) / max(1.0, min(w, h))
                if aspect < PAPER_ASPECT_MIN or aspect > PAPER_ASPECT_MAX:
                    continue
                if paper is None or area > paper[0]:
                    paper = (area, x, y, w, h, aspect, max(w, h))

            # 光斑（很小、很亮的块，且必须落在锁定的 ROI 里）
            sblobs = cv_lite.rgb888_find_blobs(shape, img_np, SPOT_THRESHOLD,
                                               SPOT_MIN_AREA, 1)
            for i in range(0, len(sblobs), 4):
                x, y, w, h = (sblobs[i], sblobs[i + 1], sblobs[i + 2],
                              sblobs[i + 3])
                area = float(w * h)
                if area < SPOT_MIN_AREA or area > SPOT_MAX_AREA:
                    continue
                if max(w, h) * 1.0 / max(1.0, min(w, h)) > SPOT_MAX_ASPECT:
                    continue
                ccx, ccy = x + w / 2.0, y + h / 2.0
                if abs(ccx - spot_u) > SPOT_ROI_HALF or \
                        abs(ccy - spot_v) > SPOT_ROI_HALF:
                    continue
                if spot is None or area > spot[0]:
                    spot = (area, ccx, ccy, (x, y, w, h))

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
            if paper is not None:
                _, x, y, w, h, _, _ = paper
                img.draw_rectangle(x, y, w, h, color=(0, 255, 0), thickness=2)
                cu, cv_ = x + w / 2.0, y + h / 2.0
                img.draw_cross(int(cu), int(cv_), color=(255, 0, 0), size=14,
                               thickness=2)
            if spot is not None:
                img.draw_circle(int(spot[1]), int(spot[2]), 8,
                                color=(255, 255, 0), thickness=2)
                img.draw_cross(int(spot[1]), int(spot[2]),
                               color=(255, 255, 0), size=6, thickness=1)

            # ---------- 打印 ----------
            if time.ticks_diff(now, t_print) >= PRINT_MS:
                t_print = now
                line = "FPS=%.1f" % clock.fps()
                if paper is None:
                    line += "  白纸: 未检出"
                else:
                    n_tgt += 1
                    cu, cv_ = paper[1] + paper[3] / 2.0, paper[2] + paper[4] / 2.0
                    dist_m = FX_PX * PAPER_LONG_M / max(1.0, paper[6])
                    line += ("  纸块=%.1f%% 靶心=(%.0f,%.0f) 长边=%.0fpx "
                             "比=%.2f 距离≈%.2fm"
                             % (100.0 * paper[0] / img_area, cu, cv_,
                                paper[6], paper[5], dist_m))
                    if spot is not None:
                        n_spot += 1
                        eu, ev = cu - spot[1], cv_ - spot[2]
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
            del img_np
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
