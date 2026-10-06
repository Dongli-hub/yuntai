"""靶纸检测诊断：把"靶纸在画面里却检不到"这件事拆开看。

用法（在地瓜派上，e_aim 目录里）：

    # 现场抓 40 帧、统计每一档判据拒掉了多少帧
    python3 tools/diag_target.py --frames 40

    # 对已经存下来的图反复调参（不用再摆靶子，调参速度最快）
    python3 tools/diag_target.py --image out/diag/frame_007.jpg
    python3 tools/diag_target.py --image 'out/diag/*.jpg'

输出两类信息：
  1) 每帧一行：轮廓数 / 面积过 / 四边形过 / 候选数 -> 结果或拒因；
  2) 汇总：检出率 + 拒因直方图 + 各判据的实测分布。

它只读配置，不下发任何串口指令，可以随时跑。
"""

import argparse
import glob
import math
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim.camera import Camera                                    # noqa: E402
from eaim.config import (find_calibration_overlays, load_config)  # noqa: E402
from eaim.geometry import CameraModel                             # noqa: E402
from eaim.laser import LaserSpotDetector                          # noqa: E402
from eaim.target import TargetDetector                            # noqa: E402


def build(cfg_path, extra, overrides):
    cfg = load_config(cfg_path or None, extra_path=extra or None,
                      overrides=overrides or None)
    cam = CameraModel(fx=cfg.calib.fx, fy=cfg.calib.fy,
                      cx=cfg.calib.cx, cy=cfg.calib.cy, dist=cfg.calib.dist)
    return cfg, cam


def draw(frame, res, diag):
    img = frame.copy()
    if res is not None:
        if res.quad is not None and len(res.quad) == 4:
            cv2.polylines(img, [np.asarray(res.quad, dtype=np.int32)], True,
                          (0, 255, 0), 2)
        cv2.drawMarker(img, (int(res.uv[0]), int(res.uv[1])), (0, 0, 255),
                       cv2.MARKER_CROSS, 24, 2)
        text = "conf=%.2f %s" % (res.confidence, res.describe()[:60])
    else:
        text = "NO TARGET"
    cv2.putText(img, text, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, text, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 255, 255), 1, cv2.LINE_AA)
    line = ("contours=%d area_rej=%d poly_rej=%d cands=%d"
            % (diag.get("n_contours", 0), diag.get("n_area_rej", 0),
               diag.get("n_poly_rej", 0), diag.get("n_cands", 0)))
    cv2.putText(img, line, (8, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, line, (8, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 255), 1, cv2.LINE_AA)
    return img


def frame_line(idx, res, diag):
    if res is not None:
        return "帧%4d: 检出 conf=%.2f %s" % (idx, res.confidence, res.describe())
    rej = diag.get("reject", [])
    if diag.get("n_cands", 0):
        if rej:
            first = rej[0]
            extra = " ".join("%s=%s" % (k, v) for k, v in first.items()
                             if k != "why")
            why = "%s [%s]" % (first.get("why", "?"), extra) if extra \
                else first.get("why", "?")
        else:
            why = "候选全崩(异常)"
    else:
        why = "没找到四边形候选（面积拒 %d / 近似拒 %d / 太小 %d）" % (
            diag.get("n_area_rej", 0), diag.get("n_poly_rej", 0),
            diag.get("n_small_rej", 0))
    return ("帧%4d: 未检出 <- %s (轮廓 %d, 洞比例 %s)"
            % (idx, why, diag.get("n_contours", 0),
               diag.get("holes", [])[:4]))


def main() -> int:
    ap = argparse.ArgumentParser(description="靶纸检测诊断")
    ap.add_argument("--config", default="", help="默认 configs/default.yaml")
    ap.add_argument("--frames", type=int, default=40, help="抓多少帧（相机模式）")
    ap.add_argument("--image", default="", help="改用已保存的图片/通配符，不占相机")
    ap.add_argument("--save-dir", default="out/diag", help="标注图保存目录")
    ap.add_argument("--save-n", type=int, default=6, help="最多保存几张标注图")
    ap.add_argument("--scale", type=float, default=-1.0,
                    help="覆盖 process_scale（默认用配置里的值）")
    ap.add_argument("--set", action="append", default=[],
                    help="覆盖配置，如 --set target.loose_fallback=true")
    args = ap.parse_args()

    overrides = {}
    for item in args.set:
        if "=" in item:
            k, v = item.split("=", 1)
            overrides[k.strip()] = v.strip()

    cfg, cam = build(args.config, find_calibration_overlays(), overrides)
    scale = cfg.camera.process_scale if args.scale < 0 else args.scale
    det = TargetDetector(cfg.target, cam)
    # 和主程序保持一致：先检光斑，再把光斑交给靶纸检测去"补"胶带上的亮洞
    spot_det = LaserSpotDetector(cfg.laser)
    if cfg.calib.boresight_uv:
        spot_det.set_boresight(cfg.calib.boresight_uv)
    spot_det.set_boresight_table(cfg.calib.boresight_table)
    os.makedirs(args.save_dir, exist_ok=True)

    frames = []          # (名字, 图像)
    if args.image:
        paths = sorted(glob.glob(args.image))
        if not paths:
            print("找不到图片: %s" % args.image)
            return 2
        for p in paths:
            img = cv2.imread(p)
            if img is not None:
                frames.append((os.path.basename(p).rsplit(".", 1)[0], img))
        print("读入 %d 张图片，scale=%.2f，配置 %s"
              % (len(frames), scale, cfg.source_path))
    else:
        camobj = Camera(cfg.camera)
        if not camobj.start():
            print("相机打开失败: %s" % camobj.error)
            return 2
        print("相机 %dx%d %s @%.0ffps 生效，采集 %d 帧，scale=%.2f"
              % (cfg.camera.width, cfg.camera.height, camobj.actual_fourcc,
                 camobj.actual_fps, args.frames, scale))
        t_end = time.time() + max(3.0, args.frames / 8.0 + 4.0)
        while len(frames) < args.frames and time.time() < t_end:
            fr, _ = camobj.read()
            if fr is None:
                time.sleep(0.01)
                continue
            frames.append(("frame_%03d" % len(frames), fr.image))
        camobj.stop()

    n_ok = 0
    why_hist = {}
    tape_vals, conf_vals, ring_vals, aspect_vals = [], [], [], []
    saved = 0
    for i, (name, img) in enumerate(frames):
        t0 = time.time()
        spot = spot_det.detect(img, scale)
        spot_arg = None
        if spot is not None and spot.area > 0:
            spot_arg = (spot.uv[0], spot.uv[1],
                        math.sqrt(spot.area / math.pi))
        res = det.detect(img, scale, spot=spot_arg)
        ms = (time.time() - t0) * 1000.0
        diag = dict(det.diag)
        print(frame_line(i, res, diag) + ("  (%.0fms)" % ms))
        if res is not None:
            n_ok += 1
            conf_vals.append(res.confidence)
            tape_vals.append(res.tape_mm)
            ring_vals.append(res.ring_score)
        else:
            rej = diag.get("reject", [])
            key = rej[0]["why"] if rej else ("无候选" if not diag.get("n_cands", 0)
                                             else "候选全崩(异常)")
            why_hist[key] = why_hist.get(key, 0) + 1
        for item in diag.get("pass", []):
            aspect_vals.append(item["aspect"])
        if saved < args.save_n and (res is not None or i < 3 or i % 10 == 0):
            out = os.path.join(args.save_dir, "%s_%s.jpg"
                               % (name, "ok" if res is not None else "no"))
            cv2.imwrite(out, draw(img, res, diag))
            # 同时存一份【没有画线】的原图：调参/离线复现都要用干净的原图，
            # 画了绿框的图会干扰胶带厚度的量测（实测差 2~3mm）。
            cv2.imwrite(os.path.join(args.save_dir, "%s_raw.jpg" % name), img)
            saved += 1

    print("\n================ 汇总 ================")
    if frames:
        print("检出 %d/%d = %.0f%%" % (n_ok, len(frames),
                                     100.0 * n_ok / len(frames)))
    if why_hist:
        print("拒因分布:")
        for k, v in sorted(why_hist.items(), key=lambda kv: -kv[1]):
            print("   %-26s %d" % (k, v))
    if tape_vals:
        print("检出帧的胶带厚度(mm): 最小 %.1f 中位 %.1f 最大 %.1f"
              % (min(tape_vals), float(np.median(tape_vals)), max(tape_vals)))
        print("检出帧的置信度:       最小 %.2f 中位 %.2f 最大 %.2f"
              % (min(conf_vals), float(np.median(conf_vals)), max(conf_vals)))
        print("检出帧的红圈分:       最小 %.2f 中位 %.2f 最大 %.2f"
              % (min(ring_vals), float(np.median(ring_vals)), max(ring_vals)))
    if aspect_vals:
        print("候选长宽比: 最小 %.2f 中位 %.2f 最大 %.2f （A4=1.414）"
              % (min(aspect_vals), float(np.median(aspect_vals)), max(aspect_vals)))
    print("标注图已存到 %s" % args.save_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
