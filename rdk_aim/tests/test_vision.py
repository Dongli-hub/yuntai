"""视觉层测试：用合成靶纸验证"黑框 + 红圈"检测的中心精度，以及光斑检测。

关键点：这里不依赖仿真模块，自己画一张透视后的靶纸，
真值由同一个单应矩阵给出，所以精度是**可量化**的。
"""

import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim.config import TargetConfig, LaserConfig  # noqa: E402
from eaim.geometry import CameraModel, apply_h  # noqa: E402
from eaim.laser import LaserSpotDetector  # noqa: E402
from eaim.target import TargetDetector  # noqa: E402

W, H = 1280, 720
FX, FY, CX, CY = 700.0, 700.0, 640.0, 360.0


def make_paper_texture(px_per_mm=3.0):
    w, h = int(210 * px_per_mm), int(297 * px_per_mm)
    tex = np.full((h, w, 3), 236, np.uint8)
    t = int(18 * px_per_mm)
    cv2.rectangle(tex, (0, 0), (w - 1, h - 1), (26, 26, 30), -1)
    cv2.rectangle(tex, (t, t), (w - 1 - t, h - 1 - t), (236, 236, 236), -1)
    c = (w // 2, h // 2)
    for r in (20, 40, 60, 80, 100):
        cv2.circle(tex, c, int(r * px_per_mm), (40, 40, 205),
                   max(1, int(px_per_mm)))
    cv2.circle(tex, c, max(1, int(0.6 * px_per_mm)), (40, 40, 205), -1)
    return tex


def render(yaw_deg, pitch_deg, distance_mm, offset_mm=(0.0, 0.0),
           px_per_mm=3.0):
    """把靶纸按给定的云台角度/距离投到图像上，返回 (图像, 真值靶心像素)。"""
    cam = CameraModel(FX, FY, CX, CY)
    a, b = math.radians(yaw_deg), math.radians(pitch_deg)
    Ry = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0],
                   [-math.sin(a), 0, math.cos(a)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(b), -math.sin(b)],
                   [0, math.sin(b), math.cos(b)]])
    R = Ry @ Rx
    C = np.array([105.0 + offset_mm[0], 248.5 + offset_mm[1], -distance_mm])
    Rt = R.T
    Hm = cam.K @ np.stack([Rt[:, 0], Rt[:, 1], -Rt @ C], axis=1)
    Hm = Hm / Hm[2, 2]
    Htex = Hm @ np.diag([1.0 / px_per_mm, 1.0 / px_per_mm, 1.0])
    frame = cv2.warpPerspective(make_paper_texture(px_per_mm), Htex, (W, H),
                               flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_CONSTANT,
                               borderValue=(150, 150, 150))
    truth = apply_h(Hm, np.array([[105.0, 148.5]]))[0]
    return frame, (float(truth[0]), float(truth[1]))


def detector():
    return TargetDetector(TargetConfig(), CameraModel(FX, FY, CX, CY))


def test_target_center_accuracy_across_distance():
    det = detector()
    for dist in (500.0, 800.0, 1200.0, 1600.0):
        # 500mm 时 A4 的角半高已经有 16.5°，再加上相机比靶心低 11°，
        # 只要偏轴超过 ~13° 靶纸的上边就会被画面切掉（这是真实约束：
        # 赛题里小车在 AB 段离靶 50cm，所以云台必须大致对准靶心）。
        # 因此近距用 ±10° 验证，远距才用 ±25°。
        yaws = (-10.0, 0.0, 10.0) if dist <= 500.0 else (-25.0, 0.0, 25.0)
        quad_hits = 0
        for yaw in yaws:
            frame, truth = render(yaw, 0.0, dist)
            r = det.detect(frame)
            assert r is not None, "距离 %.0fmm yaw %.0f 时未检出靶纸" % (dist, yaw)
            err = math.hypot(r.uv[0] - truth[0], r.uv[1] - truth[1])
            if r.source == "quad":
                assert r.ring_score > 0.6, "环覆盖分过低 %.2f" % r.ring_score
                assert err < 2.5, \
                    "距离 %.0fmm yaw %.0f 黑框路径中心误差 %.2fpx" % (dist, yaw, err)
                quad_hits += 1
            else:
                assert err < 20.0, \
                    "距离 %.0fmm yaw %.0f 红圈兜底中心误差 %.2fpx" % (dist, yaw, err)
        assert quad_hits >= 2, \
            "距离 %.0fmm 时黑框主路径命中只有 %d/3" % (dist, quad_hits)


def test_target_distance_estimate():
    det = detector()
    frame, _ = render(0.0, 0.0, 1100.0)
    r = det.detect(frame)
    assert r is not None
    # 位姿反解给出的是"靶纸原点"的距离，与靶心距离略有差别，放宽到 8%
    assert abs(r.distance_m - 1.10) / 1.10 < 0.08, \
        "距离估计 %.3fm 偏差过大" % r.distance_m


def test_target_rejects_black_square():
    """场地里那条 1m 黑色巡线方框不能被抓成靶纸。

    注意线宽要按【真实比例】画：赛题是 1m 边长 + 18mm 线宽 = 1.8%。
    画粗了（比如 3%）就不代表真实赛道，测不出问题（这个坑踩过）。
    """
    img = np.full((H, W, 3), 200, np.uint8)
    side = 600
    line = int(round(side * 0.018))            # 1.8% —— 和赛题一致
    cv2.rectangle(img, (300, 60), (300 + side, 60 + side), (25, 25, 25), line)
    det = detector()
    assert det.detect(img) is None, "把巡线黑方框误判成靶纸了"

    # 反过来：线宽画得比真实赛道粗很多时，确实可能骗过判据 ——
    # 所以现场如果发现误检，先确认场地里的线宽是不是真的 1.8cm
    img2 = np.full((H, W, 3), 200, np.uint8)
    cv2.rectangle(img2, (200, 150), (1000, 600), (25, 25, 25), 40)
    # 这一条只做记录，不强制断言


def test_laser_spot_accuracy():
    img = np.full((H, W, 3), 210, np.uint8)
    img[:, :, 0] = 200
    u0, v0 = 730.0, 402.0
    ys, xs = np.mgrid[0:H, 0:W]
    d = np.sqrt((xs - u0) ** 2 + (ys - v0) ** 2)
    gain = np.clip(1.0 - d / 4.0, 0.0, 1.0)[:, :, None]
    colour = np.array([255.0, 80.0, 140.0])
    img = np.clip(img * (1 - gain) + colour[None, None, :] * gain, 0,
                  255).astype(np.uint8)
    det = LaserSpotDetector(LaserConfig())
    det.set_boresight((730.0, 402.0))
    r = det.detect(img)
    assert r is not None, "光斑未检出"
    err = math.hypot(r.uv[0] - u0, r.uv[1] - v0)
    assert err < 0.5, "光斑质心误差 %.2fpx" % err


def test_laser_gate_rejects_far_spot():
    """位置门控：离已知光轴点太远的蓝色斑不能被当成激光。"""
    img = np.full((H, W, 3), 210, np.uint8)
    img[:, :, 0] = 200
    cv2.circle(img, (200, 150), 6, (255, 60, 120), -1)
    det = LaserSpotDetector(LaserConfig())
    det.set_boresight((640.0, 360.0))
    det.learned = True
    assert det.detect(img) is None, "门控失效：把远处蓝点当成光斑了"


def test_ring_geometry_visible_fraction():
    from eaim.target import _visible_fraction
    # r=100mm 的圈在竖放 A4 上会被左右两侧的 18mm 胶带盖掉约 1/3
    frac = _visible_fraction(100.0, 105.0 - 18.0, 148.5 - 18.0)
    assert 0.55 < frac < 0.80, "可见比例 %.3f 不合理" % frac
    assert _visible_fraction(60.0, 87.0, 130.5) == 1.0


def test_boresight_distance_table():
    """光斑位置随距离查表插值（光斑检不到时的兜底精度）。

    这是用户提的那个问题的答案：两轴不平行/有平移时，固定像素点只在
    标定距离上准，所以用 (距离, u, v) 的折线来描述它。
    """
    det = LaserSpotDetector(LaserConfig())
    det.set_boresight((600.0, 380.0))
    # 没表 -> 永远是单点（老行为，不能变差）
    assert det.boresight_for_distance(0.8) == (600.0, 380.0)
    assert det.boresight_for_distance(None) == (600.0, 380.0)

    det.set_boresight_table([[0.5, 620.0, 400.0], [1.0, 600.0, 380.0],
                             [1.5, 590.0, 370.0]])
    # 端点 / 中点
    assert det.boresight_for_distance(0.4) == (620.0, 400.0)
    assert det.boresight_for_distance(0.5) == (620.0, 400.0)
    assert det.boresight_for_distance(1.5) == (590.0, 370.0)
    assert det.boresight_for_distance(2.0) == (590.0, 370.0)
    u, v = det.boresight_for_distance(0.75)
    assert abs(u - 610.0) < 1e-9 and abs(v - 390.0) < 1e-9
    # 距离未知（靶纸没检到）时不能乱猜 -> 退回单点
    assert det.boresight_for_distance(None) == (600.0, 380.0)
