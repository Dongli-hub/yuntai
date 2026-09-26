"""控制层测试：
  1) 瞄准环在"纯转动 + 一帧延迟"的简化被控对象上必须无静差收敛
  2) kp 的稳定边界要与理论 kp < 2*sin(45°/d) 一致
  3) 超前补偿在匀速运动下能显著减小跟踪误差
  4) 画圆轨迹的相位能跟着拐角事件正确重同步
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim.aim import AimController  # noqa: E402
from eaim.config import ControlConfig, DrawConfig, TargetConfig  # noqa: E402
from eaim.geometry import CameraModel, build_rectifier  # noqa: E402
from eaim.trajectory import CircleTrajectory  # noqa: E402

FX = 700.0
PY = FX * math.pi / 180.0        # 每度对应像素


class Plant:
    """简化被控对象：偏置 -> 靶心像素（纯转动，一帧延迟）。

    物理依据见 docs/00 第 1.1 节：激光光斑在图像里固定不动，
    云台转 theta 度 => 靶心像素移动 PY*theta。
    """

    def __init__(self, delay_frames=1, target_u=760.0, spot_u=640.0):
        self.delay = delay_frames
        self.cmd = [0.0] * (delay_frames + 1)
        self.applied = 0.0
        self.spot = (spot_u, 360.0)
        self.base_u = target_u

    @property
    def target(self):
        return (self.base_u - PY * self.applied, 360.0)

    def step(self, yaw_deg):
        self.cmd.append(yaw_deg)
        self.applied = self.cmd.pop(0)


def run_loop(kp, delay_frames=1, frames=80, lead_s=0.0):
    cam = CameraModel(FX, FX, 640.0, 360.0)
    ctrl = AimController(ControlConfig(kp=kp, lead_s=lead_s, deadband_px=0.0,
                                       rate_limit_dps=1e6),
                         cam, sign_yaw=1.0, sign_pitch=-1.0)
    plant = Plant(delay_frames)
    errs = []
    for k in range(frames):
        target = plant.target
        out = ctrl.update(k * 0.02, 0.02, target, (0.0, 0.0), plant.spot)
        errs.append(out.err_norm)
        plant.step(ctrl.yaw_deg)
    return errs


def test_aim_converges_zero_error():
    errs = run_loop(kp=0.22, delay_frames=1)
    assert errs[-1] < 0.01, "稳态误差 %.4fpx 应为 0（积分型环节无静差）" % errs[-1]
    assert max(errs[:3]) > 50.0, "初始误差应该很大"
    assert errs[20] < 1.0, "20 帧内应收敛，实际 %.2fpx" % errs[20]


def test_kp_stability_boundary_matches_theory():
    """稳定边界 kp < 2*sin(45°/(d+0.5))，d = 环路延迟帧数。

    推导：开环 = kp/(z-1) 再延迟 d 帧。
    相位 = -90°（积分） - ωT/2（积分器的相位亏损） - d·ωT
    令 =-180° 得 ωT = 90°/(d+0.5)；此处幅值为 kp/(2·sin(ωT/2))，
    所以稳定条件 kp < 2·sin(45°/(d+0.5))。
      d=1 -> 1.000 ; d=2 -> 0.618 ; d=3 -> 0.445 ; d=4 -> 0.347 ; d=5 -> 0.285
    """
    for d in (1, 2, 3, 4):
        # 采样回路里"测量->计算->执行->再测量"本身就占 1 帧，
        # 所以总延迟 D = 被控对象延迟 d + 1
        limit = 2.0 * math.sin(math.radians(45.0) / (d + 1.0))
        stable = run_loop(kp=limit * 0.7, delay_frames=d, frames=250)
        assert stable[-1] < 0.5, "d=%d kp=%.2f 应稳定，实际末值 %.1f" % (
            d, limit * 0.7, stable[-1])
        unstable = run_loop(kp=limit * 1.6, delay_frames=d, frames=60)
        assert unstable[-1] > 1.0 or max(unstable[20:]) > 3.0, \
            "d=%d kp=%.2f 应发散，实际末值 %.2f" % (d, limit * 1.6, unstable[-1])


def test_lead_compensation_reduces_moving_error():
    """匀速运动 + 一帧延迟：加超前补偿后残差应明显减小。"""
    def track(lead_s):
        cam = CameraModel(FX, FX, 640.0, 360.0)
        ctrl = AimController(ControlConfig(kp=0.25, lead_s=lead_s, lead_gain=1.0,
                                           deadband_px=0.0, rate_limit_dps=1e6),
                             cam, sign_yaw=1.0, sign_pitch=-1.0)
        plant = Plant(1, target_u=700.0)
        t = 0.0
        dt = 0.02
        errs = []
        for k in range(200):
            t += dt
            # 靶心以 300px/s 匀速横向移动（相当于小车匀速平移），
            # 同时叠加云台自身动作造成的像移 —— 这才构成真正的闭环
            base_u = 700.0 + 300.0 * t
            tgt = (base_u - PY * plant.applied, 360.0)
            out = ctrl.update(t, dt, tgt, (300.0, 0.0), plant.spot)
            plant.step(ctrl.yaw_deg)
            if k > 100:
                errs.append(out.err_norm)
        return float(np.mean(errs))

    e_plain = track(0.0)
    e_lead = track(0.04)      # 总延迟约 2 帧 = 40ms
    assert e_lead < e_plain * 0.7, \
        "超前补偿无效：无超前 %.1fpx -> 有超前 %.1fpx" % (e_plain, e_lead)


def test_aim_rate_limit_and_clamp():
    cam = CameraModel(FX, FX, 640.0, 360.0)
    cfg = ControlConfig(kp=0.5, rate_limit_dps=100.0, max_yaw_deg=30.0,
                        max_pitch_deg=20.0, deadband_px=0.0)
    ctrl = AimController(cfg, cam)
    prev = 0.0
    for k in range(200):
        out = ctrl.update(k * 0.02, 0.02, (1200.0, 700.0), (0.0, 0.0), (640.0, 360.0))
        assert abs(out.yaw_deg - prev) <= 100.0 * 0.02 + 1e-6
        prev = out.yaw_deg
    assert abs(ctrl.yaw_deg) <= 30.0 + 1e-9
    assert abs(ctrl.pitch_deg) <= 20.0 + 1e-9


def make_rect():
    quad = np.array([[300.0, 200.0], [900.0, 260.0], [880.0, 640.0], [320.0, 600.0]])
    return build_rectifier((210.0, 297.0), quad, 3.0)


def test_circle_setpoint_is_on_the_ring():
    rect = make_rect()
    tr = CircleTrajectory(DrawConfig(radius_mm=60.0, phase_source="time",
                                     lap_time_s=20.0))
    tr.set_target(rect)
    for phase in (0.0, 0.13, 0.25, 0.5, 0.77, 0.99):
        uv = tr.setpoint(phase)
        assert uv is not None
        mm = rect.img_to_mm([uv])[0]
        r = math.hypot(mm[0] - 105.0, mm[1] - 148.5)
        assert abs(r - 60.0) < 0.5, "相位 %.2f 对应半径 %.2fmm" % (phase, r)


def test_circle_phase_resync_on_corner():
    rect = make_rect()
    tr = CircleTrajectory(DrawConfig(radius_mm=60.0, phase_source="corner",
                                     lap_time_s=20.0))
    tr.set_target(rect)
    tr.seed_corner(0)
    t = 0.0
    for _ in range(100):          # 自由推进 2s = 0.1 圈
        t += 0.02
        tr.update(t)
    assert abs(tr.phase - 0.1) < 0.02, "自由推进相位 %.3f 应约 0.1" % tr.phase
    tr.on_corner(t, 1)            # 第 1 个拐角 -> 应回到 0.25 圈
    tr.on_corner(t + 5.0, 2)
    tr.on_corner(t + 10.0, 3)
    for k in range(1, 60):
        tr.update(t + 10.0 + k * 0.02)
    assert abs(tr.phase - 0.75) < 0.05, "拐角重同步后相位 %.3f 应约 0.75" % tr.phase
    assert abs(tr.lap_time - 20.0) < 1.0, "单圈时间估计 %.1fs" % tr.lap_time


def test_circle_phase_never_jumps():
    """同步只允许改相位的"速度"，不允许让设定点瞬移（否则激光会画出台阶）。"""
    rect = make_rect()
    tr = CircleTrajectory(DrawConfig(radius_mm=60.0, phase_source="corner"))
    tr.set_target(rect)
    tr.seed_corner(0)
    t = 0.0
    prev = tr.setpoint(0.0)
    max_step = 0.0
    for k in range(600):
        t += 0.02
        if k == 100:
            tr.on_corner(t, 1)
        if k == 300:
            tr.on_corner(t, 2)
        tr.update(t)
        sp = tr.setpoint()
        if sp and prev:
            max_step = max(max_step, math.hypot(sp[0] - prev[0], sp[1] - prev[1]))
        prev = sp
    assert max_step < 12.0, "设定点单帧最大跳变 %.1fpx 太大" % max_step
