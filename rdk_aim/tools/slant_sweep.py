"""斜视鲁棒性量化测试：把一张真实靶纸照片合成出各种"斜视角度 + 距离"，
然后统计检测器能检出多少 —— 用来回答"斜着看靶子就丢靶"这个问题。

原理：对画面做一次单应变换，把靶纸那块梯形化（模拟斜视）、缩小（模拟变远），
背景一起变，成像效果和真实斜视几乎一样。靶纸在图像里怎么变都行，
检测器该检出的还是要检出。

用法（本地 PC 或地瓜派都能跑）：

    python3 tools/slant_sweep.py --image out/diag/frame_000_ok.jpg
    python3 tools/slant_sweep.py --image out/diag/frame_000_ok.jpg --save out/slant

输出一张表：每个 (斜视角, 方位, 缩放) 下 检出/未检出 + 置信度 + 拒因。
"""

import argparse
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim.config import (find_calibration_overlays, load_config)  # noqa: E402
from eaim.geometry import CameraModel, order_quad                 # noqa: E402
from eaim.target import TargetDetector                            # noqa: E402


def tilt_quad(quad, tilt_deg, azimuth_deg, scale):
    """算出梯形化后的四个角点。

    以靶纸中心为原点，把"远离相机的那条边"按 cos(tilt) 压缩，
    再把整块按 scale 缩放（模拟距离变化）。azimuth 决定往哪一侧斜。
    """
    q = np.asarray(quad, dtype=np.float64).reshape(4, 2)
    c = q.mean(axis=0)
    p = (q - c) * scale
    k = math.cos(math.radians(max(0.0, min(80.0, tilt_deg))))
    a = math.radians(azimuth_deg)
    axis = np.array([math.cos(a), math.sin(a)])
    d = p @ axis                       # 每个角点在"远近方向"上的投影
    dmax = max(1e-6, float(np.max(np.abs(d))))
    factor = 1.0 - (1.0 - k) * (d / dmax + 1.0) * 0.5
    p = p * factor[:, None]
    return (p + c).astype(np.float32)


def warp_frame(frame, src_quad, dst_quad):
    H = cv2.getPerspectiveTransform(np.asarray(src_quad, dtype=np.float32),
                                    np.asarray(dst_quad, dtype=np.float32))
    h, w = frame.shape[:2]
    return cv2.warpPerspective(frame, H, (w, h), flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_REPLICATE)


def main() -> int:
    ap = argparse.ArgumentParser(description="斜视鲁棒性测试")
    ap.add_argument("--image", required=True, help="一张正对靶纸的清晰照片")
    ap.add_argument("--config", default="")
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--save", default="", help="保存合成图（可选）")
    ap.add_argument("--scales", default="1.0,0.7,0.5",
                    help="距离缩放（1/scale 约等于距离倍数）")
    ap.add_argument("--min-conf", type=float, default=-1.0,
                    help="低于这个置信度算未检出（默认用配置里的 min_confidence）")
    args = ap.parse_args()

    overrides = {}
    for item in args.set:
        if "=" in item:
            k, v = item.split("=", 1)
            overrides[k.strip()] = v.strip()
    cfg = load_config(args.config or None,
                      extra_path=find_calibration_overlays() or None,
                      overrides=overrides or None)
    cam = CameraModel(fx=cfg.calib.fx, fy=cfg.calib.fy,
                      cx=cfg.calib.cx, cy=cfg.calib.cy, dist=cfg.calib.dist)
    det = TargetDetector(cfg.target, cam)
    min_conf = cfg.target.min_confidence if args.min_conf < 0 else args.min_conf

    frame = cv2.imread(args.image)
    if frame is None:
        print("读不到图片: %s" % args.image)
        return 2
    base = det.detect(frame, 1.0)
    if base is None or base.quad is None:
        print("基准图就检不到靶纸，先换一张正对靶纸的照片")
        return 2
    quad0 = np.asarray(base.quad, dtype=np.float32)
    print("基准: conf=%.2f %s" % (base.confidence, base.describe()))
    if args.save:
        os.makedirs(args.save, exist_ok=True)

    scales = [float(x) for x in args.scales.split(",") if x.strip()]
    print("\n%-8s %-8s %-6s %-8s %-9s %s"
          % ("斜视°", "方位°", "缩放", "结果", "置信度", "拒因/说明"))
    print("-" * 78)
    ok = 0
    total = 0
    for scale in scales:
        for tilt in (0, 15, 30, 45, 60):
            for az in (0, 90):
                total += 1
                dst = tilt_quad(quad0, tilt, az, scale)
                warped = warp_frame(frame, quad0, dst)
                res = det.detect(warped, 1.0)
                diag = dict(det.diag)
                # 置信度门限：主程序里低于 min_confidence 的结果会被丢掉，
                # 这里要用同一条规则，免得把"低分误检"算成检出。
                if res is not None and res.confidence < min_conf:
                    diag.setdefault("reject", []).append(
                        {"why": "置信度低于门限", "conf": round(res.confidence, 2)})
                    res = None
                if res is not None:
                    ok += 1
                    note = "uv=(%.0f,%.0f) d=%.2fm tape=%.1fmm" % (
                        res.uv[0], res.uv[1], res.distance_m, res.tape_mm)
                    print("%-8d %-8d %-6.2f %-8s %-9.2f %s"
                          % (tilt, az, scale, "检出", res.confidence, note))
                    if args.save:
                        img = warped.copy()
                        cv2.polylines(img, [np.asarray(res.quad, dtype=np.int32)],
                                      True, (0, 255, 0), 2)
                        cv2.drawMarker(img, (int(res.uv[0]), int(res.uv[1])),
                                       (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
                        cv2.imwrite(os.path.join(
                            args.save,
                            "t%02d_a%03d_s%.2f_ok.jpg" % (tilt, az, scale)), img)
                else:
                    rej = diag.get("reject", [])
                    if rej:
                        first = rej[0]
                        note = first.get("why", "?") + " " + " ".join(
                            "%s=%s" % (k, v) for k, v in first.items()
                            if k != "why")
                    else:
                        note = "无四边形候选(轮廓%d/面积拒%d/近似拒%d)" % (
                            diag.get("n_contours", 0), diag.get("n_area_rej", 0),
                            diag.get("n_poly_rej", 0))
                    print("%-8d %-8d %-6.2f %-8s %-9s %s"
                          % (tilt, az, scale, "未检出", "-", note))
                    if args.save:
                        cv2.imwrite(os.path.join(
                            args.save,
                            "t%02d_a%03d_s%.2f_no.jpg" % (tilt, az, scale)), warped)
    print("-" * 78)
    print("斜视检出率: %d/%d = %.0f%%" % (ok, total, 100.0 * ok / max(1, total)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
