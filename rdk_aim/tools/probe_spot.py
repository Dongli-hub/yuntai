#!/usr/bin/env python3
"""光斑颜色探针：把"蓝优势"最强的几个像素/连通域直接打出来。

什么时候用：
  * tools/check_laser.py 检出率低，怀疑"光斑太淡 / 阈值不对"
  * 怀疑检到的是**假光斑**（窗边色散、反光边缘），想知道真光斑的数值长什么样
  * 想定量决定 laser.b_min / laser.b_minus_others 设多少

它做什么（只读，不发任何运动指令）：
  1) 抓一帧，算出 advantage = B - max(G,R)（这就是代码用的判据）
  2) 打印 advantage 最大的 N 个像素：坐标 + BGR + 蓝优势
  3) 按当前阈值做一遍连通域，打印最大的几个块的面积/位置/平均B/最大优势
  4) 打印"真光斑应该在哪"的参考：画面里 B 通道最亮的区域

怎么读结果：
  * 如果 advantage 最大的地方**不是**你肉眼看到的光斑位置 -> 那是假光斑，
    要做的是提高 b_minus_others，或者用位置门控（calib.boresight_uv）把它排掉
  * 如果真光斑位置的 advantage 比最大值小很多（比如 20 对 70），
    说明真光斑过曝成白团了 -> 把相机曝光调短（camera.exposure），
    或者在 configs/default.yaml 里把 laser.b_minus_others 降到 15~20

用法：
    python3 tools/probe_spot.py --source 0 --frames 20
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim.camera import Camera  # noqa: E402
from eaim.config import find_default_config, load_config  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0")
    ap.add_argument("--frames", type=int, default=20, help="抓多少帧，取中位帧")
    ap.add_argument("--top", type=int, default=6, help="打印前几个最强像素")
    ap.add_argument("--scale", type=float, default=1.0, help="按这个缩放检测（默认原图）")
    ap.add_argument("--at", default=None, help="额外测量一个点，格式 x,y（原图坐标）")
    ap.add_argument("--exposure", type=int, default=None,
                    help="临时指定曝光（V4L2 绝对值，单位 100us；越小画面越暗）")
    ap.add_argument("--gain", type=float, default=None, help="临时指定增益")
    ap.add_argument("--save", default="out/probe.jpg", help="把这一帧存下来")
    args = ap.parse_args()

    cfg = load_config(find_default_config())
    if args.exposure is not None:
        cfg.camera.exposure = int(args.exposure)
        print("（临时曝光 = %d，约 %.1f ms）" % (args.exposure, args.exposure / 10.0))
    if args.gain is not None:
        cfg.camera.gain = float(args.gain)
    cfg.camera.source = "uvc" if args.source.isdigit() else args.source
    if args.source.isdigit():
        cfg.camera.device = int(args.source)
    cam = Camera(cfg.camera)
    if not cam.start():
        print("相机打不开：%s" % cam.error)
        return 1

    frames = []
    want = max(3, args.frames)
    t_end = time.monotonic() + 10.0
    while len(frames) < want and time.monotonic() < t_end:
        f, _ = cam.read()
        if f is not None:
            frames.append(f.image)
        else:
            time.sleep(0.02)     # read() 只在"有新帧"时返回 Frame，其余返回 None
    cam.stop()
    if not frames:
        print("一帧都没抓到")
        return 1

    img = frames[len(frames) // 2]
    scale = float(args.scale)
    if scale != 1.0:
        img = cv2.resize(img, None, fx=scale, fy=scale,
                         interpolation=cv2.INTER_AREA)
    b, g, r = cv2.split(img)
    adv = b.astype(np.int16) - np.maximum(g, r).astype(np.int16)

    print("=" * 70)
    print(" 画面 %dx%d，缩放到 %.2f 后用代码同款判据：advantage = B - max(G,R)"
          % (img.shape[1], img.shape[0], scale))
    print(" 当前阈值：b_min=%d  b_minus_others=%d"
          % (cfg.laser.b_min, cfg.laser.b_minus_others))
    print("=" * 70)

    flat = adv.ravel()
    order = np.argsort(flat)[-max(1, args.top):][::-1]
    print("蓝优势最强的 %d 个像素：" % len(order))
    for k in order:
        y, x = divmod(int(k), adv.shape[1])
        px = img[y, x]
        print("   adv=%3d  位置=(%4d,%4d)  BGR=(%3d,%3d,%3d)"
              % (adv[y, x], x, y, px[0], px[1], px[2]))

    mask = ((adv > max(1, cfg.laser.b_minus_others - 1)) &
            (b > max(1, cfg.laser.b_min - 1))).astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:
        print("按当前阈值没有任何连通域 —— 阈值太高了")
    else:
        order2 = sorted(range(1, n), key=lambda i: -st[i, 4])[:5]
        print("按当前阈值的最大的几个连通域（面积 / 中心 / 平均B / 最大优势）：")
        for i in order2:
            area = int(st[i, 4])
            cx = st[i, 0] + st[i, 2] / 2.0
            cy = st[i, 1] + st[i, 3] / 2.0
            sel = lab == i
            print("   面积=%4d  中心=(%6.1f,%6.1f)  平均B=%5.1f  最大优势=%3d  贴边=%s"
                  % (area, cx, cy, b[sel].mean(), int(adv[sel].max()),
                     "是" if (st[i, 0] <= 4 or st[i, 1] <= 4 or
                              st[i, 0] + st[i, 2] >= img.shape[1] - 4 or
                              st[i, 1] + st[i, 3] >= img.shape[0] - 4) else "否"))
    print()
    # ---- 第二种判据：紫/品红优势 min(B,R) - G ----
    # 405nm 蓝紫激光打在白纸上，中心常过曝成"白里透粉"：
    #   B 和 R 都高、G 明显低  ->  min(B,R) - G 很大
    # 而红色靶心是 R 高 B 低（min(B,R)=B 很小）-> 这个判据天然不会和靶心混；
    # 窗边/衣物的蓝色色散只有 B 高 -> 也会被排掉。
    vio = np.minimum(b, r).astype(np.int16) - g.astype(np.int16)
    flat2 = vio.ravel()
    order3 = np.argsort(flat2)[-max(1, args.top):][::-1]
    print("紫/品红优势 [min(B,R) - G] 最强的 %d 个像素：" % len(order3))
    for k in order3:
        y, x = divmod(int(k), vio.shape[1])
        px = img[y, x]
        print("   vio=%3d  位置=(%4d,%4d)  BGR=(%3d,%3d,%3d)"
              % (vio[y, x], x, y, px[0], px[1], px[2]))

    if args.at:
        try:
            xs, ys = (int(float(v)) for v in args.at.split(","))
            px = img[ys, xs]
            print()
            print("指定点 (%d,%d)：BGR=(%d,%d,%d)  蓝优势=%d  紫优势=%d"
                  % (xs, ys, px[0], px[1], px[2],
                     int(adv[ys, xs]), int(vio[ys, xs])))
        except Exception as exc:                               # noqa: BLE001
            print("--at 解析失败：%s" % exc)
    print()
    print("提示：光斑中心如果过曝成白色，它的 advantage 会很小（B≈G≈R≈255）；")
    print("      这时把 camera.exposure 调短、或把 b_minus_others 降到 15~20。")
    try:
        cv2.imwrite(args.save, img)
        print("这一帧已存到 %s" % args.save)
    except Exception as exc:                                   # noqa: BLE001
        print("存图失败：%s" % exc)
    return 0


if __name__ == "__main__":
    sys.exit(main())
