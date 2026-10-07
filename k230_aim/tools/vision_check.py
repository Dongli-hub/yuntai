# -*- coding: utf-8 -*-
"""vision_check.py —— 视觉验证（靶框 + 激光光斑 + 误差），只检测不追靶

这一版是按亚博例程（cv_lite / 07.Face）的做法重写的：
  · cv_lite 的 C 加速算子：rgb888_find_rectangles / rgb888_find_blobs
    （比原生 find_rects 快得多，人脸检测那个例程之所以流畅就是靠这套）
  · Display.init(..., to_ide=True, quality=50)：IDE 画面流畅的关键参数
  · cv_lite.rgb888_adjust_exposure_fast(...)：软件调曝光，gain<1 变暗

画面里：
  蓝框       = 所有候选矩形
  绿框/红叉  = 通过筛选的靶框与靶心
  白框       = 光斑搜索 ROI
  黄圈/黄叉  = 检出的激光光斑

终端每秒一行：FPS / 候选数 / 靶心 / 长边px / 比例 / 光斑 / 误差(px, °)
激光由脚本通过 H723 打开（AIM + LASER_ON，偏置恒 0 → 云台只保持稳定）
"""
import os
import time
import math

# ============================ 参数 ============================
IMG_W = 640
IMG_H = 480
DISPLAY_QUALITY = 50       # IDE 画面质量：越小越流畅（例程用的就是 50）

# --- 曝光（软件增益）：画面太亮/光斑检不到就调小，比如 0.6 ---
EXPOSURE_GAIN = 1.0        # <1 变暗，>1 变亮

# --- 矩形检测（cv_lite，C 加速）---
CANNY1 = 50
CANNY2 = 150
APPROX_EPS = 0.03          # 多边形拟合精度
AREA_MIN_RATIO = 0.01      # 最小面积比例
MAX_ANGLE_COS = 0.5        # 越小越"像矩形"
GAUSS_BLUR = 5
ASPECT_MIN = 1.20          # 靶纸 297x180 → 长短边比 ≈1.65
ASPECT_MAX = 2.40

# --- 光斑检测（cv_lite 色块，阈值为 [Rmin,Rmax, Gmin,Gmax, Bmin,Bmax]）---
# 判据 = "很亮"（激光中心过曝成白芯，比白纸亮）。
# 如果画面整体偏亮导致连白纸都算进来，把 EXPOSURE_GAIN 调小（0.6~0.8）。
SPOT_THRESHOLD = [220, 255, 150, 255, 150, 255]
SPOT_MIN_AREA = 3
SPOT_MAX_AREA = 3000
SPOT_MAX_ASPECT = 3.0
SPOT_KERNEL = 1

# 光斑位置是**刚性固定**的（同轴激光），所以：
#   先自动学习一次 → 连续稳定 SPOT_LEARN_FRAMES 帧就"锁定"，
#   之后 ROI 再也不动（避免被误检拖走 —— 之前飘到角落里就是这么来的）。
SPOT_CX = -1               # 想手动指定就填像素坐标（-1 = 自动学习）
SPOT_CY = -1
SPOT_ROI_HALF = 90         # ROI 半径（锁死后就这么大，足够覆盖视差漂移）
SPOT_LEARN_FRAMES = 15     # 学习期：连续多少帧稳定就锁定
SPOT_NEAR_PX = 25          # 学习期只接受"离当前中心这么近"的检测

FX_PX = 430.0              # 像素焦距（640 宽时的估计；标定后改实测值）

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
                    self.buf = self.buf[-1:]        # bytearray 不能 del 切片
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
    import image as image_mod

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

    print("=" * 64)
    print("vision_check(cv_lite版): 靶框+光斑  曝光增益=%.2f  显示质量=%d"
          % (EXPOSURE_GAIN, DISPLAY_QUALITY))
    print("  靶框参数: canny=%d/%d eps=%.2f 比例=[%.2f,%.2f]"
          % (CANNY1, CANNY2, APPROX_EPS, ASPECT_MIN, ASPECT_MAX))
    print("  光斑阈值: %s" % str(SPOT_THRESHOLD))
    print("=" * 64)

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

            # ---------- 取图 + 曝光 ----------
            clock.tick()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            img_np = img.to_numpy_ref()
            n_frame += 1

            if EXPOSURE_GAIN != 1.0:
                img_np = cv_lite.rgb888_adjust_exposure_fast(
                    shape, img_np, EXPOSURE_GAIN)
                img = image_mod.Image(IMG_W, IMG_H, image_mod.RGB888,
                                      alloc=image_mod.ALLOC_REF,
                                      data=img_np)

            # ---------- 找黑胶带框 ----------
            best = None
            n_rect = 0
            list_txt = ""
            rects = cv_lite.rgb888_find_rectangles(shape, img_np, CANNY1,
                                                   CANNY2, APPROX_EPS,
                                                   AREA_MIN_RATIO,
                                                   MAX_ANGLE_COS, GAUSS_BLUR)
            for i in range(0, len(rects), 4):
                x, y, w, h = rects[i], rects[i + 1], rects[i + 2], rects[i + 3]
                n_rect += 1
                area = w * h
                aspect = max(w, h) / max(1.0, min(w, h))
                img.draw_rectangle(x, y, w, h, color=(80, 80, 255), thickness=1)
                if n_rect <= 3:
                    list_txt += " [%.1f%% 比=%.2f]" % (100.0 * area / img_area,
                                                       aspect)
                if area < AREA_MIN_RATIO * img_area:
                    continue
                if aspect < ASPECT_MIN or aspect > ASPECT_MAX:
                    continue
                if best is None or area > best[0]:
                    best = (area, x + w / 2.0, y + h / 2.0, aspect,
                            max(w, h), (x, y, w, h))

            # ---------- 找激光光斑（ROI 内最亮的块）----------
            spot = None
            raw_n = 0                      # 通过亮度阈值、面积/形状筛选的块数
            blobs = cv_lite.rgb888_find_blobs(shape, img_np, SPOT_THRESHOLD,
                                              SPOT_MIN_AREA, SPOT_KERNEL)
            for i in range(0, len(blobs), 4):
                x, y, w, h = blobs[i], blobs[i + 1], blobs[i + 2], blobs[i + 3]
                area = w * h
                if area < SPOT_MIN_AREA or area > SPOT_MAX_AREA:
                    continue
                if max(w, h) * 1.0 / max(1.0, min(w, h)) > SPOT_MAX_ASPECT:
                    continue
                # 要求整块都落在 ROI 内（只要中心在 ROI 里的话，
                # 整张白纸这种大块的"中心"也可能落进 ROI，会造成误检）
                if (x < spot_u - SPOT_ROI_HALF) or \
                        (x + w > spot_u + SPOT_ROI_HALF) or \
                        (y < spot_v - SPOT_ROI_HALF) or \
                        (y + h > spot_v + SPOT_ROI_HALF):
                    continue
                raw_n += 1
                if spot is None or area > spot[0]:
                    spot = (area, x + w / 2.0, y + h / 2.0, (x, y, w, h))

            # ---- 学习 / 锁定：锁定之后 ROI 绝不再动 ----
            if spot is not None and not locked:
                far = (abs(spot[1] - spot_u) > SPOT_NEAR_PX) or \
                      (abs(spot[2] - spot_v) > SPOT_NEAR_PX)
                if far:
                    learn_n = 0                      # 跳太远，重新数
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
                bx, by = best[1], best[2]
                img.draw_rectangle(best[5], color=(0, 255, 0), thickness=2)
                img.draw_cross(int(bx), int(by), color=(255, 0, 0), size=14,
                               thickness=2)
            img.draw_rectangle(int(spot_u - SPOT_ROI_HALF),
                               int(spot_v - SPOT_ROI_HALF),
                               SPOT_ROI_HALF * 2, SPOT_ROI_HALF * 2,
                               color=(255, 255, 255),
                               thickness=2 if locked else 1)
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
                    line += "  靶心=(%.0f,%.0f) 长边=%.0fpx 比=%.2f" % (
                        best[1], best[2], best[4], best[3])
                    if spot is not None:
                        n_spot += 1
                        eu, ev = best[1] - spot[1], best[2] - spot[2]
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
