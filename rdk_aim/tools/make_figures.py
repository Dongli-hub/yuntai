#!/usr/bin/env python3
"""生成设计报告用的原理图（用仿真数据画，数字都是真的）。

    python tools/make_figures.py

输出到 docs/img/：
  fig1_boresight.jpg   —— 同轴激光原理：为什么"对准图像中心"是错的，
                          "对准激光光斑"才对
  fig2_error_curve.png —— 一圈行驶 + 画圆的误差曲线（报告中"测试结果分析"可用）

图里的每个数字都来自仿真实跑，不是画着好看的示意。
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim import protocol as proto  # noqa: E402
from eaim.config import find_default_config, load_config  # noqa: E402
from eaim.geometry import CameraModel  # noqa: E402
from eaim.laser import LaserSpotDetector  # noqa: E402
from eaim.sim import SimWorld  # noqa: E402
from eaim.target import TargetDetector  # noqa: E402

FONT_CANDIDATES = [r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf",
                   "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
                   "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"]


def load_font(size):
    try:
        from PIL import ImageFont
    except Exception:
        return None
    for p in FONT_CANDIDATES:
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                continue
    return None


def draw_text_cn(img, items):
    """items: [(x, y, text, size, color_bgr)]。有中文字体就用 PIL，否则退回 cv2。"""
    font_cache = {}
    ok = True
    for _x, _y, _t, size, _c in items:
        if size not in font_cache:
            font_cache[size] = load_font(size)
        if font_cache[size] is None:
            ok = False
            break
    if not ok:
        for x, y, text, size, colour in items:
            cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                        size / 32.0, colour, 2, cv2.LINE_AA)
        return img
    from PIL import Image, ImageDraw
    pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil)
    for x, y, text, size, colour in items:
        draw.text((x, y), text, font=font_cache[size],
                  fill=(colour[2], colour[1], colour[0]))
    return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)


def crosshair(img, uv, colour, size=26, thick=2, gap=6):
    u, v = int(round(uv[0])), int(round(uv[1]))
    cv2.line(img, (u - size, v), (u - gap, v), colour, thick, cv2.LINE_AA)
    cv2.line(img, (u + gap, v), (u + size, v), colour, thick, cv2.LINE_AA)
    cv2.line(img, (u, v - size), (u, v - gap), colour, thick, cv2.LINE_AA)
    cv2.line(img, (u, v + gap), (u, v + size), colour, thick, cv2.LINE_AA)
    cv2.circle(img, (u, v), gap - 1, colour, thick, cv2.LINE_AA)


def fig1(sim_cfg, out_dir):
    """用仿真初始帧画原理图：光斑在哪、图像中心在哪、差多少毫米。"""
    cfg = sim_cfg
    cfg.camera.source = "sim"
    cfg.sim.car_enable = False
    w = SimWorld(cfg)
    w.on_mode(proto.pack_mode(proto.AimMode.AIM))
    w.on_aim(proto.pack_aim(0.0, 0.0, proto.AimFlags.LASER_ON, 0))
    frame = w.render(time.monotonic())
    cam = CameraModel(cfg.calib.fx, cfg.calib.fy, cfg.calib.cx, cfg.calib.cy)
    det = TargetDetector(cfg.target, cam)
    spot_det = LaserSpotDetector(cfg.laser)
    tgt = det.detect(frame)
    spot = spot_det.detect(frame)
    assert tgt is not None and spot is not None, "仿真帧必须能同时检出靶心和光斑"

    centre = (cam.cx, cam.cy)
    dist_mm = tgt.distance_m * 1000.0
    off_px = float(np.hypot(spot.uv[0] - centre[0], spot.uv[1] - centre[1]))
    off_mm = off_px / cam.fx * dist_mm          # 光斑偏离图像中心 -> 靶面毫米

    img = frame.copy()
    # 图像中心
    crosshair(img, centre, (255, 255, 255), 30, 2)
    # 激光光斑
    cv2.circle(img, (int(spot.uv[0]), int(spot.uv[1])), 13, (255, 90, 255), 2,
               cv2.LINE_AA)
    # 靶心
    crosshair(img, tgt.uv, (0, 230, 0), 22, 2)
    # 中心 -> 光斑 的偏差标出来
    cv2.arrowedLine(img, (int(centre[0]), int(centre[1])),
                    (int(spot.uv[0]), int(spot.uv[1])), (0, 255, 255), 2,
                    cv2.LINE_AA, tipLength=0.25)
    # 光斑 -> 靶心
    cv2.arrowedLine(img, (int(spot.uv[0]), int(spot.uv[1])),
                    (int(tgt.uv[0]), int(tgt.uv[1])), (255, 130, 0), 2,
                    cv2.LINE_AA, tipLength=0.06)
    # 局部放大：把光斑+中心那块放大贴到右下角
    pad = 55
    x0 = int(max(0, min(spot.uv[0], centre[0]) - pad))
    y0 = int(max(0, min(spot.uv[1], centre[1]) - pad))
    x1 = int(min(img.shape[1], max(spot.uv[0], centre[0]) + pad))
    y1 = int(min(img.shape[0], max(spot.uv[1], centre[1]) + pad))
    if x1 > x0 and y1 > y0:
        crop = img[y0:y1, x0:x1].copy()
        scale = 3
        big = cv2.resize(crop, None, fx=scale, fy=scale,
                         interpolation=cv2.INTER_NEAREST)
        bh, bw = big.shape[:2]
        if bw < img.shape[1] - 40 and bh < img.shape[0] - 40:
            px, py = img.shape[1] - bw - 18, 18
            img[py:py + bh, px:px + bw] = big
            cv2.rectangle(img, (px - 2, py - 2), (px + bw + 1, py + bh + 1),
                          (255, 255, 255), 2)
            cv2.rectangle(img, (x0, y0), (x1, y1), (255, 255, 255), 1)
            cv2.line(img, (x1, y0), (px, py + bh), (255, 255, 255), 1, cv2.LINE_AA)
            cv2.line(img, (x0, y0), (px, py), (255, 255, 255), 1, cv2.LINE_AA)
    items = [
        (18, 14, "同轴激光：光斑位置刚性固定，但**不在图像中心**", 26, (255, 255, 255)),
        (18, 52, "白十字 = 图像中心", 22, (255, 255, 255)),
        (18, 82, "品红圈 = 激光光斑实测位置", 22, (255, 90, 255)),
        (18, 112, "绿十字 = 检测到的靶心", 22, (0, 230, 0)),
        (18, 150, "光斑偏离图像中心 %.0f px" % off_px, 24, (0, 255, 255)),
        (18, 182, "= 靶面上 %.0f mm（距离 %.2f m）" % (off_mm, dist_mm / 1000.0),
         24, (0, 255, 255)),
        (18, 218, "所以判据必须是「靶心 vs 光斑」", 24, (0, 255, 0)),
        (18, 250, "不能是「靶心 vs 图像中心」", 24, (0, 90, 255)),
    ]
    items = [(x, y, t.replace("**", ""), s, c) for x, y, t, s, c in items]
    img = draw_text_cn(img, items)

    # 底部结论条
    bar = np.zeros((96, img.shape[1], 3), np.uint8)
    bar[:] = (32, 32, 36)
    bar = draw_text_cn(bar, [
        (16, 10, "若改成对准「图像中心」：在 %.2fm 处就已经偏 %.0fmm，"
                 "直接超过 D1<=20mm 的要求；" % (dist_mm / 1000.0, off_mm),
         22, (120, 200, 255)),
        (16, 48, "对准「激光光斑」：机械装配、激光轴夹角、距离视差全部自动被消掉，"
                 "稳态误差 -> 0", 22, (120, 255, 150)),
    ])
    out = np.vstack([img, bar])
    # 这张图有真实图像内容，用 JPG 存（PNG 会到 1MB 以上，JPG 只要 1/6）
    path = os.path.join(out_dir, "fig1_boresight.jpg")
    cv2.imwrite(path, out, [int(cv2.IMWRITE_JPEG_QUALITY), 93])
    print("已生成 %s" % path)
    print("  光斑(%.1f,%.1f)  图像中心(%.0f,%.0f)  偏差 %.1fpx = %.0fmm"
          % (spot.uv[0], spot.uv[1], centre[0], centre[1], off_px, off_mm))
    return path


def fig2(csv_path, out_dir):
    """误差曲线图：直接用 recorder 的 CSV 画，报告里"测试结果与分析"能直接贴。"""
    import csv
    if not csv_path or not os.path.exists(csv_path):
        print("跳过 fig2：没有 CSV（先跑一次 main.py sim 并保留 out/*.csv）")
        return None
    rows = list(csv.DictReader(open(csv_path, encoding="utf-8")))

    def col(name, need_laser=False):
        out = []
        for r in rows:
            if need_laser and r.get("laser", "0") != "1":
                continue
            v = r.get(name, "")
            if v in ("", "None"):
                continue
            try:
                out.append((float(r["t"]), float(v)))
            except ValueError:
                continue
        return np.array(out, dtype=np.float64) if out else np.zeros((0, 2))

    t_err = col("err_norm")
    if len(t_err) == 0:
        print("跳过 fig2：CSV 里没有 err_norm")
        return None
    d1 = col("sim_miss_mm", need_laser=True)
    d2 = col("sim_d2_mm", need_laser=True)
    t0 = float(t_err[0, 0])
    t1 = float(t_err[-1, 0])
    W = 1180

    def panel(height, series, ylim, title, note=""):
        """series: [(数据Nx2, BGR, 线宽)]"""
        img = np.full((height, W, 3), 255, np.uint8)
        left, right, top, bottom = 78, W - 24, 46, height - 34
        cv2.line(img, (left, bottom), (right, bottom), (80, 80, 80), 1, cv2.LINE_AA)
        cv2.line(img, (left, top), (left, bottom), (80, 80, 80), 1, cv2.LINE_AA)
        for i in range(5):
            y = int(top + (bottom - top) * i / 4.0)
            val = ylim[1] - (ylim[1] - ylim[0]) * i / 4.0
            cv2.line(img, (left - 5, y), (left, y), (120, 120, 120), 1)
            cv2.putText(img, "%.0f" % val, (14, y + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 60, 60), 1, cv2.LINE_AA)
        for data, colour, thick in series:
            pts = []
            for t, v in data:
                x = int(left + (right - left) * (t - t0) / max(1e-6, t1 - t0))
                y = int(bottom - (bottom - top) *
                        (np.clip(v, ylim[0], ylim[1]) - ylim[0]) /
                        max(1e-6, ylim[1] - ylim[0]))
                pts.append((x, y))
            if len(pts) > 1:
                cv2.polylines(img, [np.array(pts, np.int32)], False, colour,
                              thick, cv2.LINE_AA)
        return draw_text_cn(img, [
            (left + 8, 10, title, 21, (30, 30, 30)),
            (left + 8, height - 30, "时间 t / s（共 %.1fs）" % (t1 - t0), 17,
             (60, 60, 60)),
        ] + ([(right - 470, height - 30, note, 17, (30, 30, 30))] if note else []))

    ymax_e = max(20.0, float(np.percentile(t_err[:, 1], 99.5)) + 2.0)
    p1 = panel(360, [(t_err, (200, 70, 0), 2)], (0.0, ymax_e),
               "像素误差 |e| = |靶心 - 光斑|（px，0 = 激光正好打在靶心）")
    ymax = max(25.0, float(np.percentile(np.vstack([d1, d2])[:, 1], 99.5)) + 3.0) \
        if len(d1) and len(d2) else 25.0
    series = []
    if len(d1):
        series.append((d1, (0, 130, 0), 2))
    if len(d2):
        series.append((d2, (190, 0, 190), 2))
    p2 = panel(300, series, (0.0, ymax),
               "靶面真值误差（mm）：绿 = D1 到靶心，紫 = D2 到 r=6cm 红圈",
               "红虚线 = 20mm 指标线")
    # 20mm 指标线
    y20 = int((300 - 34) - (300 - 34 - 46) * min(1.0, 20.0 / ymax))
    cv2.line(p2, (78, y20), (W - 24, y20), (0, 0, 230), 1, cv2.LINE_AA)
    out = np.vstack([p1, p2])
    path = os.path.join(out_dir, "fig2_error_curve.png")
    cv2.imwrite(path, out)
    print("已生成 %s" % path)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None)
    ap.add_argument("--csv", default=None, help="要画的 CSV（默认取 out 里最新一个）")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out_dir = args.out or os.path.join(base, "docs", "img")
    os.makedirs(out_dir, exist_ok=True)
    cfg = load_config(args.config or find_default_config(),
                      extra_path=os.path.join(base, "configs", "sim.yaml"))
    fig1(cfg, out_dir)
    csv_path = args.csv
    if not csv_path:
        import glob
        files = sorted(glob.glob(os.path.join(base, "out", "*.csv")),
                       key=os.path.getmtime)
        csv_path = files[-1] if files else None
    fig2(csv_path, out_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
