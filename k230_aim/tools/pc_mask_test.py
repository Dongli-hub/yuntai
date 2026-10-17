# -*- coding: utf-8 -*-
"""pc_mask_test.py —— 验证"反向二值化 ∪ Canny"那张融合图

    python k230_aim/tools/pc_mask_test.py

为什么要有它：检测链每帧都会建立融合图：
img.copy() -> 反向二值化 -> 闭运算 -> Canny -> 融合。
它一旦断了，识别会直接失效。所以留一个最小回归：跑几帧 detect()，检查
  · 不抛异常；
  · _GY_MASK_OK 置位；
  · mask_out 真的拿到了那张图。
  · 融合图之后**不会**再调用 cv_lite 的高层矩形函数（它内部会重跑 Canny）。
返回码：0 = OK，1 = 有问题。
"""
import io
import os
import re
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pc_sim_main as S          # noqa: E402  （复用它的假模块/假场景）
import pc_field_test as F        # noqa: E402


def main():
    S.install_fake_modules()
    src = io.open(S.MAIN_PY, encoding="utf-8").read()
    # 只加载模块、不跑整机主循环：main.py 末尾那行裸的 main() 注掉
    src, _n = re.subn(r"(?m)^main\(\)\s*$", "# main()  # 本测试自己驱动",
                      src)
    assert _n == 1, "main.py 末尾没找到那行 main()（改了？）"

    g = {"__name__": "k230_main_under_test"}
    exec(compile(src, S.MAIN_PY, "exec"), g)      # 只加载，不跑 main()

    from media.sensor import Sensor, CAM_CHN_ID_0
    import cv_lite
    sensor = Sensor(id=2, width=1280, height=960, fps=90)
    sensor.reset()
    sensor.set_framesize(width=g["IMG_W"], height=g["IMG_H"], chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_0)
    sensor.run()

    det = g["PaperDetector"]()
    n_hit = 0
    mask_out = [None]
    # 135r 架构约束：融合图之后**不得**再进 cv_lite 的高层矩形函数
    # （那个函数内部会重新 Canny）。把它设成炸弹，跑完整链验证没人调用。
    _orig_rr = cv_lite.grayscale_find_rectangles_with_corners

    def _must_not_recanny(*a, **kw):
        raise AssertionError("融合图之后又调用了会重跑 Canny 的矩形函数")

    cv_lite.grayscale_find_rectangles_with_corners = _must_not_recanny
    try:
        for i in range(6):
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            res = det.detect(img, mask_out)
            if res is not None:
                n_hit += 1
    finally:
        cv_lite.grayscale_find_rectangles_with_corners = _orig_rr

    ok = True
    if not g["_GY_MASK_OK"][0]:
        print("[FAIL] _GY_MASK_OK 没置位")
        ok = False
    if mask_out[0] is None:
        print("[FAIL] mask_out 是 None（没有取得融合图）")
        ok = False
    # 现场两张“白框明明可见但曾报 r0轮”的融合图：即使连通域丢失，
    # 中心 ROI 也必须在同一张融合图上拟合出内外四边形。
    shot_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "field_shots")
    for name in ("live_mask_a.png", "live_mask_b.png",
                 "live_mask_c.png", "live_mask_d.png"):
        p = os.path.join(shot_dir, name)
        a = cv2.imdecode(np.fromfile(p, dtype=np.uint8),
                         cv2.IMREAD_GRAYSCALE)
        if a is None:
            print("[FAIL] 读不到 %s" % p)
            ok = False
            continue
        a = cv2.resize(a, (g["IMG_W"], g["IMG_H"]),
                       interpolation=cv2.INTER_AREA)
        mf = F.GrayFrame(a)
        bands = g["_mask_band_quads"](mf, a, None)
        big = 0
        for qo, qi in bands:
            xs = [p[0] for p in qo]
            ys = [p[1] for p in qo]
            bw = max(xs) - min(xs)
            bh = max(ys) - min(ys)
            if (max(bw, bh) >= 200.0) and g["_quad_geom_ok"](qo):
                big += 1
        good = big > 0
        print("现场融合图 %s: %s（%d 组内外四边形，其中大且几何合格 %d 组）"
              % (name, "OK" if good else "FAIL", len(bands), big))
        ok = ok and good
    # 左侧背景线粘连的现场图：目标四边形应仍落在靶纸外框附近。
    p = os.path.join(shot_dir, "live_mask_f.png")
    if os.path.isfile(p):
        a = cv2.imdecode(np.fromfile(p, dtype=np.uint8),
                         cv2.IMREAD_GRAYSCALE)
        a = cv2.resize(a, (g["IMG_W"], g["IMG_H"]),
                       interpolation=cv2.INTER_AREA)
        a = np.where(a > 128, 255, 0).astype(np.uint8)
        bands = g["_mask_band_quads"](F.GrayFrame(a), a, None)
        hit = 0
        for qo, _qi in bands:
            c = g["_gy_centroid4"](qo)
            xs = [p[0] for p in qo]
            ys = [p[1] for p in qo]
            long_side = max(max(xs) - min(xs), max(ys) - min(ys))
            if c is not None and (120 <= long_side <= 160) and \
                    (abs(c[0] - 346.0) <= 12.0) and \
                    (abs(c[1] - 248.0) <= 12.0):
                hit += 1
        good = hit > 0
        print("现场融合图 live_mask_f.png: %s（背景粘连过滤）"
              % ("OK" if good else "FAIL"))
        ok = ok and good
    print("检测链认出靶 %d/6 帧；另加背景粘连图检查；_GY_MASK_OK=%s mask=%s"
          % (n_hit, g["_GY_MASK_OK"][0],
             "有" if mask_out[0] is not None else "无"))
    print("结果：%s" % ("融合图检测链 OK" if ok else "融合图检测链有问题"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
