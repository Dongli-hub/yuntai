# -*- coding: utf-8 -*-
"""cv_lite 闭运算最小测试：只做 取图 -> close -> 显示。

在 CanMV IDE 里直接运行。用来确认：
  1) 这版固件的 cv_lite.rgb888_close 能不能跑；
  2) 闭运算后的灰度图能不能显示到 IDE 帧缓冲区；
  3) 3x3 / 5x5 核能不能把靶纸上的黑色同心圆填掉。

不加载 PaperDetector、不做识别、不发串口。
"""
import os
import time
import gc

from media.sensor import *
from media.display import *
from media.media import *
import image
import cv_lite

W = 640
H = 480
KERNEL = 3
ITERS = 1
TH = 0          # 0 = cv_lite 内部 Otsu
ROI_MODE = False
ROI_W = 320
ROI_H = 240


def main():
    sensor = Sensor(id=2, width=1280, height=960, fps=90)
    sensor.reset()
    sensor.set_framesize(width=W, height=H, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)
    Display.init(Display.ST7701, width=W, height=H, to_ide=True)
    MediaManager.init()
    sensor.run()

    print("=" * 60)
    print("cv_lite close test: %dx%d kernel=%d iterations=%d th=%s"
          % (W, H, KERNEL, ITERS, "Otsu" if TH == 0 else str(TH)))
    if ROI_MODE:
        print("ROI close: %dx%d" % (ROI_W, ROI_H))
    print("=" * 60)

    n = 0
    sum_close = 0
    sum_show = 0
    sum_frames = 0
    t_print = time.ticks_ms()
    out = None
    full = None
    try:
        while True:
            os.exitpoint()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            t0 = time.ticks_ms()
            try:
                if ROI_MODE:
                    rx = (W - ROI_W) // 2
                    ry = (H - ROI_H) // 2
                    try:
                        src = img.crop((rx, ry, ROI_W, ROI_H))
                        sw, sh = ROI_W, ROI_H
                    except Exception as e:
                        print("ROI crop 失败(%s)，改全图" % e)
                        src = img
                        sw, sh = W, H
                        rx = ry = 0
                else:
                    src = img
                    sw, sh = W, H
                    rx = ry = 0
                arr = src.to_numpy_ref()
                closed = cv_lite.rgb888_close(
                    [sh, sw], arr, KERNEL, ITERS, TH)
                t1 = time.ticks_ms()
                close_ms = time.ticks_diff(t1, t0)
                out = image.Image(
                    sw, sh, image.GRAYSCALE,
                    alloc=image.ALLOC_REF, data=closed)
                if n < 3:
                    print("frame %d: close=%dms roi=%dx%d"
                          % (n, close_ms, sw, sh))
                if ROI_MODE and (sw < W or sh < H):
                    if full is None:
                        full = image.Image(W, H, image.GRAYSCALE)
                        try:
                            full.clear()
                        except Exception:
                            full.draw_rectangle(
                                0, 0, W, H, color=0, fill=True)
                    full.draw_image(out, rx, ry)
                    Display.show_image(full)
                else:
                    Display.show_image(out)
                show_ms = time.ticks_diff(time.ticks_ms(), t1)
                # show_image 是异步的：等这次传输完成再进入下一帧，
                # 且 out/closed/arr 保留到下一轮被替换，不能立刻 del。
                time.sleep_ms(30)
                if n < 3:
                    print("frame %d: show_call=%dms" % (n, show_ms))
            except Exception as e:
                print("close/display 失败: %s" % e)
                out = None
                closed = None
                arr = None
                src = None
                img = None
                gc.collect()
                time.sleep_ms(500)
                continue

            n += 1
            sum_close += close_ms
            sum_show += show_ms
            sum_frames += 1
            now = time.ticks_ms()
            if time.ticks_diff(now, t_print) >= 1000:
                print("close fps=%.1f  avg_close=%dms avg_show_call=%dms"
                      % (n * 1000.0 / time.ticks_diff(now, t_print),
                         sum_close // max(1, sum_frames),
                         sum_show // max(1, sum_frames)))
                n = 0
                sum_close = 0
                sum_show = 0
                sum_frames = 0
                t_print = now
            # 闭运算结果每帧约 300KB；必须在下一帧 close 之前释放旧缓冲，
            # 否则第 4 帧就会 ndarray malloc fail。
            out = None
            closed = None
            arr = None
            src = None
            img = None
            gc.collect()
    except KeyboardInterrupt:
        print("用户停止")
    finally:
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
        print("已退出")


main()
