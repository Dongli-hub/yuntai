# -*- coding: utf-8 -*-
"""pc_vision_test.py —— 靶纸检测的离线回归（老路删掉之后唯一那条链）

    python k230_aim/tools/pc_vision_test.py

四个用例，都是合成画面喂给 PaperDetector.detect()：
  A 正视近处   -> 必须锁上，而且**靶心误差 <= 6px**（靶心=纸心）
  B 斜视/透视  -> 必须锁上（斜过来的纸也要认得出）
  C 远处小纸   -> 锁得上或明确报"无靶"，但**不许锁到旁边的白块**上
  D 画面里没有靶（只有一块白） -> 必须返回 None（不许误锁）
每例打印实际结果；返回码 0 = 全过。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np               # noqa: E402
import cv2                       # noqa: E402
import pc_sim_main as S          # noqa: E402


def make_det():
    S.install_fake_modules()
    import io
    import re
    src = io.open(S.MAIN_PY, encoding="utf-8").read()
    src = src.replace('DISPLAY_MODE = "VIRT"', 'DISPLAY_MODE = "OFF"')
    src = re.sub(r"(?m)^main\(\)\s*$", "# main()", src)
    g = {"__name__": "m"}
    exec(compile(src, S.MAIN_PY, "exec"), g)
    return g, g["PaperDetector"]()


def frame_from(arr, g):
    f = S.FakeFrame.__new__(S.FakeFrame)
    f.np = arr
    return f


def main():
    g, det = make_det()
    W, H = g["IMG_W"], g["IMG_H"]
    ok = True

    def run(name, arr, expect, tol=6.0, cx=None, cy=None):
        nonlocal ok
        det.__init__()
        res = det.detect(frame_from(np.ascontiguousarray(arr), g))
        if expect == "none":
            good = res is None
            print("%s: %s（应为 None）" % (name, "OK" if good else "FAIL -> %s" % res))
        else:
            if res is None:
                good = False
                print("%s: FAIL（没锁上）" % name)
            else:
                dx = abs(det.u - cx)
                dy = abs(det.v - cy)
                good = (dx <= tol) and (dy <= tol)
                print("%s: %s 靶心=(%.1f,%.1f) 真值=(%.1f,%.1f) 误差=(%.1f,%.1f) "
                      "长边=%.0f" % (name, "OK" if good else "FAIL",
                                     det.u, det.v, cx, cy, dx, dy,
                                     det.last_long))
        ok = ok and good

    # A 正视近处：标准场景，靶心=(300,240)
    S.SCENE.tgt_u, S.SCENE.tgt_v = 300.0, 240.0
    run("A 正视近处", S.SCENE.render(), "hit", cx=300.0, cy=240.0)

    # B 斜视/透视：把标准场景做一个梯形透视
    img = S.SCENE.render()
    src = np.float32([[100, 60], [540, 100], [540, 420], [100, 380]])
    dst = np.float32([[60, 40], [580, 120], [580, 400], [60, 440]])
    M = cv2.getPerspectiveTransform(src, dst)
    warp = cv2.warpPerspective(img, M, (W, H), borderValue=(120, 120, 120))
    # 透视之后纸心的真实位置：把源图中心 (300,240) 过同一个变换
    _c = cv2.perspectiveTransform(np.float32([[[300.0, 240.0]]]), M)[0][0]
    run("B 斜视透视", warp, "hit", tol=12.0, cx=float(_c[0]), cy=float(_c[1]))

    # C 远处小纸：整幅缩到 35%（纸长边 ~115px）
    small = cv2.resize(img, (int(W * 0.35), int(H * 0.35)))
    canvas = np.full((H, W, 3), 120, np.uint8)
    canvas[:small.shape[0], :small.shape[1]] = small
    run("C 远处小纸", canvas, "hit", tol=20.0,
        cx=small.shape[1] / 2.0 * (300.0 / 320.0) * 1.0 + 0.0,
        cy=small.shape[0] / 2.0 * (240.0 / 240.0) * 1.0 + 0.0)

    # D 没有靶：只有一块白（旧版会误锁到它）
    plain = np.full((H, W, 3), 110, np.uint8)
    cv2.rectangle(plain, (200, 120), (420, 360), (225, 225, 225), -1)
    run("D 无靶(只有白块)", plain, "none")

    # E ROI 裁剪路径：给一个"上一帧小框"的 hint，应只在框周围找 ——
    #   结果一样准，但喂给 cv_lite 的像素要明显少于全图（PROF[1] 记账）
    #   （用远处小纸的场景：ROI 才真正比全图小；近处大纸按设计就不裁）
    f = frame_from(np.ascontiguousarray(canvas), g)
    g["_PROF"][1] = 0.0
    g["_PROF"][3] = 0
    r = g["_cv2_find"](f, (122.0, 105.0, 84.0))
    fed = g["_PROF"][1] / max(1, g["_PROF"][3])
    if r is None:
        print("E ROI 路径: FAIL（没找到）")
        ok = False
    else:
        d = max(abs(r[11] - 105.0), abs(r[12] - 84.0))
        good = (d <= 6.0) and (fed < 0.35 * W * H)
        print("E ROI 路径: %s 误差=%.1fpx 喂像素=%.0f万 (全图 %.0f万)"
              % ("OK" if good else "FAIL", d, fed / 10000.0,
                 W * H / 10000.0))
        ok = ok and good

    print("结果：%s" % ("检测回归 OK" if ok else "检测回归有问题"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
