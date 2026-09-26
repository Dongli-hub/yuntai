#!/usr/bin/env python3
"""相机内参标定（棋盘格）。

用法：
    python tools/calib_intrinsic.py --source 0
    python tools/calib_intrinsic.py --source 0 --cols 9 --rows 6 --square 25
    python tools/calib_intrinsic.py --images boards/*.jpg

操作：
    把棋盘格放在靶面附近不同位置/角度，平整；窗口里出现彩色角点即识别成功，
    按 SPACE 采集一张，采到 >=12 张后按 C 计算并写入 configs/intrinsic.yaml，
    按 Q 退出。

为什么内参重要：
    瞄准环的增益直接等于 fx*pi/180（像素/度）。fx 差 10%，瞄准的收敛速度就
    差 10%（最终精度不受影响，因为环里有积分）。但 fx 差太多会拖慢收敛，
    对 2s 的指标不利。
"""

import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim.config import find_default_config, load_config, save_yaml  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--cols", type=int, default=9, help="棋盘格内角点数（列）")
    ap.add_argument("--rows", type=int, default=6, help="棋盘格内角点数（行）")
    ap.add_argument("--square", type=float, default=25.0, help="方格边长 mm")
    ap.add_argument("--images", nargs="*", default=None, help="离线用：已有图片")
    ap.add_argument("--out", default=None, help="输出 yaml（默认 configs/intrinsic.yaml）")
    ap.add_argument("--min-views", type=int, default=12)
    args = ap.parse_args()

    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out_path = args.out or os.path.join(base, "configs", "intrinsic.yaml")
    pattern = (args.cols, args.rows)
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    obj_points, img_points = [], []
    shape = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4)

    def find_corners(gray, preview=None):
        ok, corners = cv2.findChessboardCorners(
            gray, pattern,
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            return None
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        if preview is not None:
            cv2.drawChessboardCorners(preview, pattern, corners, True)
        return corners

    def add(corners):
        obj_points.append(objp.copy())
        img_points.append(corners)

    if args.images:
        files = []
        for pat in args.images:
            files.extend(sorted(glob.glob(pat)))
        for f in files:
            img = cv2.imread(f)
            if img is None:
                continue
            shape = img.shape[:2][::-1]
            corners = find_corners(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
            if corners is not None:
                add(corners)
                print("采集 %s（共 %d 张）" % (f, len(obj_points)))
        if len(obj_points) < 6:
            print("有效图太少：%d" % len(obj_points))
            return 1
    else:
        src = int(args.source) if str(args.source).isdigit() else args.source
        cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            print("打开相机失败")
            return 1
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        print("SPACE 采集 / C 计算并保存 / Q 退出")
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            shape = frame.shape[:2][::-1]
            vis = frame.copy()
            corners = find_corners(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), vis)
            found = corners is not None
            cv2.putText(vis, "found=%s  views=%d/%d" %
                        (found, len(obj_points), args.min_views),
                        (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0) if found else (0, 0, 255), 2)
            cv2.imshow("calib_intrinsic", vis)
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord("q"):
                break
            if key == ord(" ") and found:
                add(corners)
                print("已采集 %d 张" % len(obj_points))
            if key == ord("c"):
                break
        cap.release()
        cv2.destroyAllWindows()

    if len(obj_points) < 6:
        print("有效图太少（%d），标定不可靠" % len(obj_points))
        return 1
    rms, K, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, shape, None, None)
    print("\n标定完成：重投影 RMS = %.3f px" % rms)
    print("fx=%.2f fy=%.2f cx=%.2f cy=%.2f" % (K[0, 0], K[1, 1], K[0, 2], K[1, 2]))
    print("畸变 = %s" % np.round(dist.ravel(), 5).tolist())
    hfov = 2.0 * np.degrees(np.arctan(shape[0] / (2.0 * K[0, 0])))
    print("水平视场 ≈ %.1f°  -> 扫描步长建议 %.0f°"
          % (hfov, hfov * 2.0 / 3.0))
    if rms > 1.0:
        print("警告：RMS 偏大，棋盘可能不平整或图像有运动模糊")
    cfg = load_config(find_default_config())
    data = {"calib": {"fx": float(K[0, 0]), "fy": float(K[1, 1]),
                      "cx": float(K[0, 2]), "cy": float(K[1, 2]),
                      "dist": [float(x) for x in dist.ravel()]}}
    save_yaml(out_path, data)
    print("已写入 %s" % out_path)
    print("用法：python main.py run --extra-config configs/intrinsic.yaml")
    print("（也可以把这几行直接抄进 configs/default.yaml 的 calib 段）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
