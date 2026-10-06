#!/usr/bin/env python3
"""镜头畸变自标定（不用打印棋盘）：用靶纸的直边"弯了多少"反推 k1。

原理（plumb-line / 直线法）：
    理想的直线经过镜头后如果变弯了，就说明有径向畸变。靶纸的黑胶带外框
    是**已知的直线**，所以只要量出每条边"离弦的最大距离"（弓高 sagitta），
    再找一个 k1 让去畸变后这些边重新变直，就得到了畸变系数。

为什么不用棋盘：现场没打印机/懒得贴棋盘时，这条路用现成的靶纸就能做，
    而且比"只标 fx/fy"更贴近我们真正关心的东西（靶心位置精度）。

用法：
    python3 tools/calib_distortion.py --port /dev/ttyS1 --source 0          # 只看结果
    python3 tools/calib_distortion.py --port /dev/ttyS1 --source 0 --save   # 写进 configs/intrinsic.yaml

它会自动把云台转到 ±8°/±16° 采 5 个姿态（同一张靶纸在不同画面位置），
这样可以对 k1 拟合得更稳。
"""

import argparse
import math
import os
import sys
import threading
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim import protocol as proto  # noqa: E402
from eaim.app import AimApp, RunOptions  # noqa: E402
from eaim.config import find_default_config, load_config, save_yaml  # noqa: E402


def _edges_from_frame(det, frame, mode: str = "paper"):
    """返回每条边上的点（原图像素坐标）列表。

    mode:
      paper —— 二值化出**纸的白色内边缘**量弓高（推荐）。
               胶带是手工贴的，外边缘本身就不齐；而且深色木柜会和胶带
               粘成一块，量出来的"弯曲"根本不是畸变（实测踩过：
               胶带边 14~29px、拟合失败；纸边才是干净的直线）。
      tape  —— 用检测器自己那套暗色二值化（会带上背景干扰，仅作对照）。
    """
    if mode == "paper":
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(gray, 0, 255,
                                  cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
    else:
        binary = det._binarize(frame, 1.0)
    cnts, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:3]
    for c in cnts:
        if cv2.contourArea(c) < 5000:
            continue
        peri = cv2.arcLength(c, True)
        for eps in (0.01, 0.02, 0.04):
            approx = cv2.approxPolyDP(c, eps * peri, True)
            if len(approx) == 4 and cv2.isContourConvex(approx):
                break
        else:
            continue
        pts = c.reshape(-1, 2).astype(np.float64)
        corners = approx.reshape(4, 2).astype(np.float64)
        # 把 4 个角映射到轮廓序号，再沿轮廓取四条边
        idx = [int(np.argmin(np.linalg.norm(pts - q, axis=1))) for q in corners]
        idx = sorted(set(idx))
        if len(idx) != 4:
            continue
        edges = []
        n = len(pts)
        for k in range(4):
            i0, i1 = idx[k], idx[(k + 1) % 4]
            if i1 <= i0:
                i1 += n
            seg = [pts[i % n] for i in range(i0, i1 + 1)]
            if len(seg) >= 10:
                edges.append(np.array(seg))
        if len(edges) == 4:
            return edges
    return None


def _sagitta(points):
    """点到"首尾弦"的最大垂距（弓高）。"""
    if len(points) < 5:
        return 0.0
    a, b = points[0], points[-1]
    ab = b - a
    L = float(np.linalg.norm(ab))
    if L < 20.0:
        return 0.0
    d = np.abs(np.cross(np.tile(ab, (len(points), 1)), points - a)) / L
    return float(d.max())


def _undistort(pts, K, k1):
    out = cv2.undistortPoints(pts.reshape(-1, 1, 2).astype(np.float64), K,
                              np.array([k1, 0.0, 0.0, 0.0, 0.0]))
    return out.reshape(-1, 2) * np.array([K[0, 0], K[1, 1]]) + np.array([K[0, 2], K[1, 2]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyS1")
    ap.add_argument("--source", default="0")
    ap.add_argument("--offsets", default="0,8,-8,16,-16", help="采样时用的偏航偏置（度）")
    ap.add_argument("--settle", type=float, default=1.6)
    ap.add_argument("--save", action="store_true", help="写进 configs/intrinsic.yaml")
    ap.add_argument("--mode", default="paper", choices=("paper", "tape"),
                    help="量哪条边：paper=纸的白边（推荐），tape=胶带外沿")
    args = ap.parse_args()

    cfg = load_config(find_default_config())
    cfg.link_gimbal.port = args.port
    cfg.link_gimbal.enable = True
    cfg.link_car.enable = False
    app = AimApp(cfg, RunOptions(laser=True, quiet=True))
    if not app.setup():
        return 1

    st = {"yaw": 0.0, "pitch": 0.0}
    _stop = threading.Event()

    def _keep():
        while not _stop.is_set():
            app.gimbal_link.send(proto.MsgId.AIM,
                                 proto.pack_aim(st["yaw"], st["pitch"],
                                                proto.AimFlags.LASER_ON, 0))
            _stop.wait(0.1)

    threading.Thread(target=_keep, daemon=True).start()
    app.gimbal_link.send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.AIM))

    K = np.array([[cfg.calib.fx, 0.0, cfg.calib.cx],
                  [0.0, cfg.calib.fy, cfg.calib.cy],
                  [0.0, 0.0, 1.0]])
    all_edges = []
    print("=" * 70)
    print(" 畸变自标定：依次转到 %s 度，各采一帧" % args.offsets)
    print("=" * 70)
    for off in [float(v) for v in args.offsets.split(",")]:
        st["yaw"] = off
        time.sleep(args.settle)
        f, _ = app.camera.read()
        if f is None:
            print("  偏置 %+5.1f°：没拿到画面，跳过" % off)
            continue
        edges = _edges_from_frame(app.detector, f.image, args.mode)
        if edges is None:
            print("  偏置 %+5.1f°：没找到靶纸外框，跳过（先跑 prep.sh 确认检出）" % off)
            continue
        sag = [_sagitta(e) for e in edges]
        all_edges.extend(edges)
        print("  偏置 %+5.1f°：4 条边弓高 = %s px"
              % (off, " ".join("%.2f" % v for v in sag)))

    _stop.set()
    app.gimbal_link.send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.STAB))
    time.sleep(0.1)
    app.shutdown()

    if len(all_edges) < 8:
        print("\n有效边太少（%d 条），先确认靶纸能被检出再重跑。" % len(all_edges))
        return 1

    # 剔掉"本来就弯得离谱"的边：那多半是胶带把纸拉得不平 / 手工贴歪，
    # 不是镜头畸变。留着会把 k1 拉偏（实测那条 9px 的边一直不变，
    # 典型的物理不平，不是畸变）。
    sags = np.array([_sagitta(e) for e in all_edges])
    med = float(np.median(sags))
    keep = [e for e, s in zip(all_edges, sags) if s <= max(6.0, 2.0 * med)]
    if len(keep) >= 6:
        all_edges = keep
    before = float(np.mean([_sagitta(e) for e in all_edges]))
    print("（剔除离群边后参与拟合 %d 条，平均弓高 %.2f px）" % (len(all_edges), before))
    best_k1, best_cost = 0.0, None
    for k1 in np.arange(-0.60, 0.21, 0.01):
        cost = float(np.mean([_sagitta(_undistort(e, K, float(k1)))
                              for e in all_edges]))
        if best_cost is None or cost < best_cost:
            best_k1, best_cost = float(k1), cost
    after = float(np.mean([_sagitta(_undistort(e, K, best_k1)) for e in all_edges]))

    print("\n" + "=" * 70)
    print(" 去畸变前平均弓高 %.2f px  ->  k1=%.2f 去畸变后 %.2f px"
          % (before, best_k1, after))
    if before < 1.5:
        print(" 结论：畸变本来就很小（<1.5px），可以不折腾。")
    elif after > before * 0.8:
        print(" 结论：拟合没帮上忙（可能是边被挡住/靶纸太斜）。换个姿态重跑。")
    else:
        print(" 结论：k1≈%.2f，弓高从 %.2f 降到 %.2f px（降 %.0f%%）"
              % (best_k1, before, after, 100 * (1 - after / max(1e-6, before))))
    print("=" * 70)

    if args.save and after <= before:
        out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           "configs", "intrinsic.yaml")
        save_yaml(out, {"calib": {
            "fx": float(cfg.calib.fx), "fy": float(cfg.calib.fy),
            "cx": float(cfg.calib.cx), "cy": float(cfg.calib.cy),
            "dist": [float(best_k1), 0.0, 0.0, 0.0, 0.0]}})
        print("已写入 %s（程序启动自动叠加；记得重跑 calib_boresight.py 重新标光轴点）" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
