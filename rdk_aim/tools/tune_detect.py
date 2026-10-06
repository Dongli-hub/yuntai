"""检测参数网格搜索：在同一批"合成斜视图"上比较不同参数组合。

为什么要有它：改检测阈值很危险 —— 现场一改就可能"靶纸在画面里却检不到"。
把一张真实靶纸照片合成成 斜视角 × 距离缩放 的一批图（靶心像素位置已知、
不变），就能在 PC 上几秒钟比较几十组参数，而不是靠感觉猜。

用法：

    python3 tools/tune_detect.py --image out/diag/mv_0.jpg

打分口径：
  1) 检出率（置信度 >= min_confidence 才算检出）；
  2) 检出帧的靶心像素误差中位数（合成图里靶心没动过，所以这个能反映
     "斜视时解出来的中心准不准"）。
"""

import argparse
import itertools
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import slant_sweep as ss                                            # noqa: E402
from eaim.config import load_config                                  # noqa: E402
from eaim.geometry import CameraModel                                # noqa: E402
from eaim.target import TargetDetector                               # noqa: E402


def build_cases(frame, quad0, scales, tilts, azs):
    out = []
    for scale, tilt, az in itertools.product(scales, tilts, azs):
        warped = ss.warp_frame(frame, quad0, ss.tilt_quad(quad0, tilt, az, scale))
        out.append(("%.2f/%d/%d" % (scale, tilt, az), warped))
    return out


def score(cases, cfg, cam, truth_uv, min_conf):
    det = TargetDetector(cfg, cam)
    hits = 0
    errs = []
    misses = []
    for name, img in cases:
        r = det.detect(img, 1.0)
        if r is None or r.confidence < min_conf:
            misses.append(name)
            continue
        hits += 1
        errs.append(float(np.hypot(r.uv[0] - truth_uv[0], r.uv[1] - truth_uv[1])))
    med = float(np.median(errs)) if errs else float("nan")
    return hits, med, misses


def main() -> int:
    ap = argparse.ArgumentParser(description="检测参数网格搜索")
    ap.add_argument("--image", required=True)
    ap.add_argument("--config", default="")
    ap.add_argument("--scales", default="1.0,0.7,0.5")
    ap.add_argument("--tilts", default="0,30,60")
    ap.add_argument("--azs", default="0,90")
    args = ap.parse_args()

    cfg = load_config(args.config or None)
    cam = CameraModel(fx=cfg.calib.fx, fy=cfg.calib.fy,
                      cx=cfg.calib.cx, cy=cfg.calib.cy, dist=cfg.calib.dist)
    frame = cv2.imread(args.image)
    if frame is None:
        print("读不到图片: %s" % args.image)
        return 2
    base = TargetDetector(cfg.target, cam).detect(frame, 1.0)
    if base is None:
        print("基准图检不到靶纸")
        return 2
    truth = base.uv
    quad0 = np.asarray(base.quad, dtype=np.float32)
    scales = [float(x) for x in args.scales.split(",")]
    tilts = [int(x) for x in args.tilts.split(",")]
    azs = [int(x) for x in args.azs.split(",")]
    cases = build_cases(frame, quad0, scales, tilts, azs)
    print("基准 uv=(%.1f, %.1f)，共 %d 个合成场景"
          % (truth[0], truth[1], len(cases)))
    print("%-42s %-8s %-10s %s" % ("参数", "检出", "中心误差中位", "漏检场景"))
    print("-" * 100)

    grid = []
    for block, c, otsu, k in itertools.product(
            (31, 41, 61), (4, 8, 14), (False, True), (3, 5)):
        grid.append(dict(adaptive_block=block, adaptive_c=c,
                         use_otsu=otsu, morph_k=k))
    results = []
    for g in grid:
        t = cfg.target
        old = (t.adaptive_block, t.adaptive_c, t.use_otsu)
        t.adaptive_block, t.adaptive_c, t.use_otsu = g["adaptive_block"], \
            g["adaptive_c"], g["use_otsu"]
        # morph kernel 通过临时改写 _binarize 的核大小来试（核不影响其它逻辑）
        det = TargetDetector(t, cam)
        det._morph_k = g["morph_k"]
        hits = 0
        errs = []
        misses = []
        for name, img in cases:
            r = _detect_with_morph(det, img, g["morph_k"])
            if r is None or r.confidence < cfg.target.min_confidence:
                misses.append(name)
                continue
            hits += 1
            errs.append(float(np.hypot(r.uv[0] - truth[0], r.uv[1] - truth[1])))
        t.adaptive_block, t.adaptive_c, t.use_otsu = old
        med = float(np.median(errs)) if errs else float("nan")
        results.append((hits, med, g, misses))

    results.sort(key=lambda r: (-r[0], 0.0 if np.isnan(r[1]) else r[1]))
    for hits, med, g, misses in results:
        print("%-42s %-8s %-10s %s"
              % ("block=%d c=%d otsu=%s morph=%d"
                 % (g["adaptive_block"], g["adaptive_c"], g["use_otsu"],
                    g["morph_k"]),
                 "%d/%d" % (hits, len(cases)),
                 "%.2fpx" % med if not np.isnan(med) else "-",
                 ",".join(misses[:6]) + ("..." if len(misses) > 6 else "")))
    return 0


def _detect_with_morph(det, img, k):
    """临时把形态学核换掉再检测（只用于调参对比）。"""
    orig = det._binarize

    def patched(frame, scale=1.0):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        block = det.cfg.adaptive_block
        if block % 2 == 0:
            block += 1
        adapt = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                      cv2.THRESH_BINARY_INV, block,
                                      det.cfg.adaptive_c)
        if det.cfg.use_otsu:
            _, otsu = cv2.threshold(gray, 0, 255,
                                    cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
            adapt = cv2.bitwise_or(adapt, otsu)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
        return cv2.morphologyEx(adapt, cv2.MORPH_CLOSE, kernel, iterations=1)

    det._binarize = patched
    try:
        return det.detect(img, 1.0)
    finally:
        det._binarize = orig


if __name__ == "__main__":
    sys.exit(main())
