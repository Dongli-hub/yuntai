"""画圆轨迹（发挥部分 3）。

思路：不"凭感觉画圆"，而是直接用靶面上那条红圈的数学方程当设定点：

    靶面坐标(mm)  p(phi) = 中心 + r*(cos, sin)      <- 已知量，不用视觉
          | H（黑框四边形给出的单应矩阵）
          v
    图像像素      sp(phi) = H * p(phi)
          | 交给 aim.py 的同一个控制律
          v
    云台偏置

于是 D2（光斑痕迹与 r=6cm 红圈的最大距离）只取决于"跟踪误差"，
而不是"画得像不像圆"——这一点让指标天然容易满足。

同步：1/4 圈 = 5s（20s 单圈）的容差下，用 RCT6 的拐角事件每 1/4 圈硬同步
一次即可。**同步只改相位时钟的速度，不改设定点位置**，
否则激光会在靶面上瞬移。
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .config import DrawConfig
from .geometry import Rectifier

__all__ = ["CircleTrajectory", "DrawStatus"]


@dataclass
class DrawStatus:
    phase: float = 0.0             # 0~1 一圈内的相位
    phase_err: float = 0.0         # 与期望相位的偏差（圈）
    lap_time_s: float = 0.0        # 当前估计的单圈时间
    corners: int = 0               # 累计拐角数
    setpoint_uv: Optional[Tuple[float, float]] = None
    synced: bool = False


class CircleTrajectory:
    """在靶面坐标系里生成 r=6cm 的圆，并映射成图像像素设定点。"""

    def __init__(self, cfg: DrawConfig, cam=None):
        self.cfg = cfg
        self.cam = cam
        self.rect: Optional[Rectifier] = None
        self.phase = 0.0
        self.lap_time = max(1.0, cfg.lap_time_s)
        self.corner_count = 0
        self.last_corner_t: Optional[float] = None
        self.last_update_t: Optional[float] = None
        self._phase_target: Optional[float] = None
        self._target_t: Optional[float] = None
        self._car_phase: Optional[float] = None
        self._car_phase_t: Optional[float] = None
        self.synced = False
        # 相位纠偏的最大速度（圈/秒）。0.5 圈/秒 => 半圈误差 1s 内拉回，
        # 且设定点移动平滑（不会让激光在靶面上跳）
        self.resync_rate_turn_s = 0.5

    def set_target(self, rect: Optional[Rectifier]) -> None:
        self.rect = rect

    def enable(self, now: float) -> None:
        self.last_update_t = now
        self.phase = 0.0
        self.corner_count = 0
        self._phase_target = None
        self._target_t = None
        self.synced = False

    def reset(self) -> None:
        self.phase = 0.0
        self.corner_count = 0
        self.last_corner_t = None
        self.last_update_t = None
        self._phase_target = None
        self._target_t = None
        self._car_phase = None
        self._car_phase_t = None
        self.synced = False
        self.lap_time = max(1.0, self.cfg.lap_time_s)

    def on_corner(self, now: float, index: int = 0) -> None:
        """RCT6 报告"刚过一个拐角"。拐角 = 1/4 圈。"""
        self.corner_count += 1
        if self.last_corner_t is not None:
            dt = now - self.last_corner_t
            if 0.2 < dt < 60.0:
                measured = dt * 4.0        # 4 个拐角 = 1 圈
                self.lap_time = 0.7 * self.lap_time + 0.3 * measured
        self.last_corner_t = now
        self._phase_target = self.corner_count / 4.0 + self.cfg.phase_offset
        self._target_t = now
        self.synced = True

    def on_lap(self, now: float, lap_time_s: Optional[float] = None) -> None:
        """RCT6 报告"完成一圈"（比拐角更强的同步基准）。"""
        if lap_time_s and lap_time_s > 0.5:
            self.lap_time = 0.5 * self.lap_time + 0.5 * float(lap_time_s)
        self.corner_count = int(round(self.corner_count / 4.0) * 4)
        self._phase_target = self.corner_count / 4.0 + self.cfg.phase_offset
        self._target_t = now
        self.synced = True

    def seed_corner(self, index: int) -> None:
        """小车从第 index 段起点开始跑时，把相位直接对齐到 index/4，
        这样"车在 A 角"就对应"光斑在圆的 0 相位"。"""
        idx = max(0, min(3, int(index)))
        self.corner_count = idx
        self.last_corner_t = None
        self._phase_target = idx / 4.0 + self.cfg.phase_offset
        self._target_t = self.last_update_t
        self.phase = idx / 4.0 + self.cfg.phase_offset
        self.synced = True

    def set_car_progress(self, lap_index: int, segment: int, progress: int,
                         now: float) -> None:
        """RCT6 直接上报了段号 + 段内进度 -> 相位最准。"""
        frac = ((segment % 4) + max(0, min(100, progress)) / 100.0) / 4.0
        self._car_phase = max(0, lap_index - 1) + frac + self.cfg.phase_offset
        self._car_phase_t = now

    def set_lap_time(self, lap_time_s: float) -> None:
        """RCT6 上报了单圈时间 -> 用它当参考时钟的速度。"""
        if lap_time_s and 2.0 < lap_time_s < 120.0:
            self.lap_time = 0.7 * self.lap_time + 0.3 * float(lap_time_s)

    def _target_now(self, now: float) -> Optional[float]:
        """参考时钟：上一次同步点 + 按单圈时间自由推进。

        关键点：参考值必须**随时间自由推进**，否则刚 seed 完 phase=0、
        target=0，误差恒为 0，相位就"卡死"不动了（这是实测踩过的坑）。
        拐角事件只负责把参考时钟拉回 k/4，之后它自己按 lap_time 走。
        """
        base, t0 = None, None
        if self.cfg.phase_source == "car_phase" and self._car_phase is not None:
            base = self._car_phase
            t0 = getattr(self, "_car_phase_t", None)
        elif self._phase_target is not None:
            base = self._phase_target
            t0 = self._target_t
        if base is None:
            return None
        if t0 is None:
            # 懒初始化：seed_corner() 可能发生在第一次 update() 之前，
            # 这时还没有时间基准。若不补上，参考值就"不会随时间推进"，
            # 相位会被恒定拉回起点 —— 实测表现为"画圆卡在 1/4 圈不动"。
            if self.cfg.phase_source == "car_phase":
                self._car_phase_t = now
            else:
                self._target_t = now
            return base
        return base + max(0.0, now - t0) / max(0.5, self.lap_time)

    def update(self, now: float) -> DrawStatus:
        cfg = self.cfg
        if self.last_update_t is None:
            self.last_update_t = now
        dt = max(0.0, min(0.5, now - self.last_update_t))
        self.last_update_t = now

        source = cfg.phase_source
        if source == "off":
            self.phase = cfg.phase_offset
        else:
            target = self._target_now(now)
            if target is None:
                # 还没同步过（time 模式，或 corner 模式还没收到第一个拐角）
                self.phase += dt / max(0.5, self.lap_time)
            else:
                self.phase = self._advance(self.phase, target, dt)

        tgt = self._target_now(now)
        status = DrawStatus(
            phase=self.phase % 1.0,
            phase_err=(self.phase if tgt is None else tgt) - self.phase,
            lap_time_s=self.lap_time,
            corners=self.corner_count,
            synced=self.synced,
        )
        status.setpoint_uv = self.setpoint()
        return status

    def _advance(self, phase: float, target: float, dt: float) -> float:
        """按 1/lap_time 的速率推进相位，并以有限速度纠偏到 target。"""
        rate = 1.0 / max(0.5, self.lap_time)
        err = target - phase
        if abs(err) > 1e-5:
            rate += max(-self.resync_rate_turn_s,
                        min(self.resync_rate_turn_s, err * 2.0))
        return phase + rate * dt

    def setpoint_mm(self, phase: Optional[float] = None) -> Optional[np.ndarray]:
        """靶面上的设定点（mm）。"""
        if self.rect is None:
            return None
        ph = self.phase if phase is None else phase
        a = 2.0 * math.pi * ph
        s = -1.0 if self.cfg.direction == "ccw" else 1.0
        cx, cy = self.rect.center_mm
        r = self.cfg.radius_mm
        return np.array([cx + r * math.cos(a), cy + s * r * math.sin(a)])

    def setpoint(self, phase: Optional[float] = None) -> Optional[Tuple[float, float]]:
        """图像像素设定点。"""
        p = self.setpoint_mm(phase)
        if p is None or self.rect is None:
            return None
        uv = self.rect.mm_to_img([p])[0]
        return (float(uv[0]), float(uv[1]))

    def mm_error_at(self, uv) -> Optional[np.ndarray]:
        """把某个像素点换算成"相对靶面圆心"的 mm 偏移，用于估算 D1/D2。"""
        if self.rect is None:
            return None
        mm = self.rect.img_to_mm([uv])[0]
        return mm - np.array(self.rect.center_mm)
