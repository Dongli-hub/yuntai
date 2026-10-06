# -*- coding: utf-8 -*-
"""t2_camera.py —— 相机 + 靶纸黑框检测（第三步）

两种检测方案，改 METHOD 切换，哪套好用哪套：
  METHOD = "cv2"    用 K230 固件自带的 OpenCV 移植版
                    （自适应阈值 -> 闭运算 -> findContours -> approxPolyDP）
                    这就是地瓜派那套算法的简化版
  METHOD = "rects"  用原生 image 模块的 find_rects()
                    （相机原生跑，快；23 年电赛 K230 方案就是这么干的）

画面上：绿色四边形 = 黑胶带框，品红对角线交点 = 靶心，红十字 = 靶心。
终端每 0.5s 打印一次 FPS 和检出数据。
"""
import os
import time
import math

# ============================ 参数 ============================
METHOD = "cv2"             # cv2 | rects

IMG_W = 640
IMG_H = 480

DISPLAY_MODE = "VIRT"      # VIRT(无屏,只看IDE) | LCD(ST7701屏+IDE)
DISPLAY_W = 640
DISPLAY_H = 480

# --- cv2 方案参数 ---
ADAPT_BLOCK = 31           # 自适应阈值窗口（奇数）
ADAPT_C = 7                # 自适应阈值偏移
QUAD_EPS = 0.02            # approxPolyDP 精度（周长的比例）
MIN_AREA_RATIO = 0.03      # 框面积至少占画面 3%

# --- rects 方案参数 ---
THRESHOLD = 20000
X_GRAD = 8
Y_GRAD = 8

# --- 公用筛选 ---
ASPECT_MIN = 1.15          # 长边/短边（A4=1.41，180x297 胶带框=1.65）
ASPECT_MAX = 2.30
PRINT_MS = 500
# ==============================================================


def order_corners(pts):
    """把 4 个角按绕中心的角度排序。"""
    cx = sum(p[0] for p in pts) / 4.0
    cy = sum(p[1] for p in pts) / 4.0
    ang = [(math.atan2(p[1] - cy, p[0] - cx), p) for p in pts]
    ang.sort(key=lambda t: t[0])
    return [list(t[1]) for t in ang]


def quad_metrics(q):
    """返回 (对角线交点, 面积, 长宽比)。"""
    (x0, y0), (x1, y1), (x2, y2), (x3, y3) = q
    d1x, d1y = x2 - x0, y2 - y0
    d2x, d2y = x3 - x1, y3 - y1
    den = d1x * d2y - d1y * d2x
    if abs(den) < 1e-6:
        center = ((x0 + x2) / 2.0, (y0 + y2) / 2.0)
    else:
        t = ((x1 - x0) * d2y - (y1 - y0) * d2x) / den
        center = (x0 + t * d1x, y0 + t * d1y)
    area = 0.5 * abs((x0 * y1 - x1 * y0) + (x1 * y2 - x2 * y1) +
                     (x2 * y3 - x3 * y2) + (x3 * y0 - x0 * y3))
    sides = [math.hypot(x1 - x0, y1 - y0),
             math.hypot(x2 - x1, y2 - y1),
             math.hypot(x3 - x2, y3 - y2),
             math.hypot(x0 - x3, y0 - y3)]
    a_len = (sides[0] + sides[2]) / 2.0
    b_len = (sides[1] + sides[3]) / 2.0
    return center, area, max(a_len, b_len) / max(1.0, min(a_len, b_len))


def find_quad_cv2(img, img_area):
    """OpenCV 方案：返回 ((q, center, area, aspect) 或 None, 候选数)。"""
    import cv2

    img_np = img.to_numpy_ref()
    gray = cv2.cvtColor(img_np, cv2.COLOR_BGR2GRAY)
    bin_img = cv2.adaptiveThreshold(gray, 255,
                                    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                    cv2.THRESH_BINARY_INV,
                                    ADAPT_BLOCK, ADAPT_C)
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    closed = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, k)
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    best = None
    n_cont = 0
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA_RATIO * img_area:
            continue
        n_cont += 1
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, QUAD_EPS * peri, True)
        if len(approx) != 4:
            continue
        pts = []
        for p in approx:
            pts.append((int(p[0][0]), int(p[0][1])))
        q = order_corners(pts)
        center, q_area, aspect = quad_metrics(q)
        if aspect < ASPECT_MIN or aspect > ASPECT_MAX:
            continue
        if best is None or q_area > best[2]:
            best = (q, center, q_area, aspect)
    return best, n_cont


def find_quad_rects(img, img_area):
    """原生 find_rects 方案。"""
    best = None
    n_rect = 0
    for r in img.find_rects(threshold=THRESHOLD,
                            x_gradient=X_GRAD, y_gradient=Y_GRAD):
        n_rect += 1
        pts = []
        for p in r.corners():
            pts.append((int(p[0]), int(p[1])))
        q = order_corners(pts)
        center, area, aspect = quad_metrics(q)
        if area < MIN_AREA_RATIO * img_area:
            continue
        if aspect < ASPECT_MIN or aspect > ASPECT_MAX:
            continue
        if best is None or area > best[2]:
            best = (q, center, area, aspect)
    return best, n_rect


def main():
    from media.sensor import Sensor, CAM_CHN_ID_0
    from media.display import Display
    from media.media import MediaManager

    if METHOD == "cv2":
        import cv2  # noqa: F401  提前失败，好排查

    sensor = Sensor()
    sensor.reset()
    time.sleep_ms(100)
    sensor.set_framesize(width=IMG_W, height=IMG_H, chn=CAM_CHN_ID_0)
    if METHOD == "cv2":
        sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)
    else:
        sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

    if DISPLAY_MODE == "LCD":
        Display.init(Display.ST7701, width=DISPLAY_W, height=DISPLAY_H,
                     to_ide=True)
    else:
        Display.init(Display.VIRT, width=DISPLAY_W, height=DISPLAY_H, fps=30)
    MediaManager.init()
    sensor.run()
    clock = time.clock()

    img_area = float(IMG_W * IMG_H)
    t_print = time.ticks_ms()
    hit = 0
    total = 0

    print("t2 相机启动 %dx%d  方法=%s" % (IMG_W, IMG_H, METHOD))
    try:
        while True:
            os.exitpoint()
            clock.tick()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            total += 1

            if METHOD == "cv2":
                best, n_seen = find_quad_cv2(img, img_area)
            else:
                best, n_seen = find_quad_rects(img, img_area)

            if best is not None:
                hit += 1
                q, center, area, aspect = best
                for i in range(4):
                    a = q[i]
                    b = q[(i + 1) % 4]
                    img.draw_line(a[0], a[1], b[0], b[1],
                                  color=(0, 255, 0), thickness=2)
                    img.draw_circle(a[0], a[1], 5, color=(0, 255, 255),
                                    thickness=2)
                img.draw_line(q[0][0], q[0][1], q[2][0], q[2][1],
                              color=(255, 0, 255), thickness=1)
                img.draw_line(q[1][0], q[1][1], q[3][0], q[3][1],
                              color=(255, 0, 255), thickness=1)
                img.draw_cross(int(center[0]), int(center[1]),
                               color=(255, 0, 0), size=12, thickness=2)

            now = time.ticks_ms()
            if time.ticks_diff(now, t_print) >= PRINT_MS:
                t_print = now
                if best is None:
                    print("FPS=%.1f 候选=%d  未检出合格框"
                          % (clock.fps(), n_seen))
                else:
                    q, center, area, aspect = best
                    print("FPS=%.1f 候选=%d 中心=(%.0f,%.0f) 面积=%.1f%% "
                          "长宽比=%.2f 角点=%s"
                          % (clock.fps(), n_seen, center[0], center[1],
                             100.0 * area / img_area, aspect, q))

            img.draw_string_advanced(
                4, IMG_H - 26, 20,
                "FPS %.1f  cand %d  hit %d/%d" % (
                    clock.fps(), n_seen, hit, total),
                color=(255, 220, 0))
            Display.show_image(img)
    except KeyboardInterrupt:
        print("用户停止")
    finally:
        sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()
        print("已退出")


main()
