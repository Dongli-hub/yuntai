#!/usr/bin/env python3
"""清晰度实时显示 —— 调相机对焦/确认有没有遮挡时用。

用法：
    python3 tools/focus_check.py                  # 默认跑 30 秒
    python3 tools/focus_check.py --seconds 60

看什么：
    屏幕上每 0.5 秒刷一行【清晰度】数值，**越大越清晰**：

        > 300   很清晰，靶纸上的红线、黑胶带边缘都能看清
        100~300 可用
        50~100  偏糊，红圈可能检不到，黑框勉强
        < 50    基本糊了 / 镜头被挡住 / 画面里没有细节

怎么用：
    1) 先把镜头前的遮挡物拿开（保护膜、手、挡片）
    2) 把相机对准靶纸，距离按比赛实际（0.5~1.5m）
    3) 一边微调镜头（有些模组镜头筒是螺纹的，可以拧）或移动相机，
       一边看这个数值 —— 调到最大就是最佳焦点
    4) 同时会把当前画面存到 /root/focus.jpg，可以拷回电脑看

注意：清晰度用的是"拉普拉斯方差"，它对**画面里有没有细节**很敏感。
      如果整个画面都是白墙，数值天然就低 —— 所以请对着有内容的
      东西（靶纸、带字的纸）来调。
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim.config import load_config, find_default_config  # noqa: E402


def sharpness(gray: np.ndarray) -> float:
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--save", default="/root/focus.jpg")
    args = ap.parse_args()

    cfg = load_config(find_default_config())
    try:
        import cv2 as _cv
        cap = _cv.VideoCapture(int(cfg.camera.device), _cv.CAP_V4L2)
    except Exception as exc:                                   # noqa: BLE001
        print("打不开相机：%s" % exc)
        return 1
    if not cap.isOpened():
        print("相机没打开")
        return 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.camera.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.camera.height)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*cfg.camera.fourcc[:4]))
    cap.set(cv2.CAP_PROP_FPS, cfg.camera.fps)
    for _ in range(15):
        cap.read()

    print("=" * 62)
    print(" 清晰度实时显示（越大越清晰），跑 %.0f 秒" % args.seconds)
    print(" 参考: >300 很清晰 | 100~300 可用 | 50~100 偏糊 | <50 糊了/被挡")
    print("=" * 62)

    best = -1.0
    t_end = time.time() + args.seconds
    next_report = 0.0
    while time.time() < t_end:
        ok, frame = cap.read()
        if not ok or frame is None:
            continue
        now = time.time()
        if now < next_report:
            continue
        next_report = now + 0.5
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        s = sharpness(gray)
        if s > best:
            best = s
            cv2.imwrite(args.save, frame)
            mark = "  <- 目前最清晰，已存图"
        else:
            mark = ""
        bar = "#" * int(min(40, s / 15.0))
        print("  清晰度 %7.1f  %-40s%s" % (s, bar, mark), flush=True)
    cap.release()
    print("-" * 62)
    print("最好的一次: %.1f，图已存到 %s" % (best, args.save))
    if best < 50:
        print("→ 基本可以断定：镜头被挡住 / 有保护膜没撕 / 对着的东西没有细节")
    elif best < 150:
        print("→ 偏糊。靶纸上的红圈（线宽 1mm）很可能检不到，建议再调")
    else:
        print("→ 清晰度够用，可以进入标定流程")
    return 0


if __name__ == "__main__":
    sys.exit(main())

