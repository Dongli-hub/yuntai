"""靶心像素轨迹的 α-β 滤波 + 短时滑行。

为什么需要它：
  1) 视觉检测有噪声，直接拿原始像素做微分会得到抖动极大的速度
  2) 瞄准环要用"目标像素速度"做超前补偿（抵消曝光+处理+执行的总延迟）
  3) 目标短时丢失（打光、抖动、遮挡）时需要凭速度继续外推，
     云台才不会"愣住"——这对运动中的连续瞄准很重要

α-β 滤波比卡尔曼简单得多，参数也只有两个，且对"匀速运动"是最优的一类估计，
完全够用。
"""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

__all__ = ["AlphaBetaTracker", "TrackState"]


@dataclass
class TrackState:
    uv: Tuple[float, float] = (0.0, 0.0)
    vel: Tuple[float, float] = (0.0, 0.0)      # px/s
    valid: bool = False
    coasting: bool = False
    age_s: float = 0.0                          # 距上次真实测量的时间


class AlphaBetaTracker:
    """二维 α-β 跟踪器（位置 + 速度）。"""

    def __init__(self, alpha: float = 0.45, beta: float = 0.10,
                 max_speed_px_s: float = 6000.0, jump_reset_px: float = 220.0,
                 stale_reset_s: float = 0.8):
        self.alpha = float(alpha)
        self.beta = float(beta)
        self.max_speed = float(max_speed_px_s)
        self.jump_reset_px = float(jump_reset_px)
        self.stale_reset_s = float(stale_reset_s)
        self.pos = np.zeros(2)
        self.vel = np.zeros(2)
        self.last_t: Optional[float] = None
        self.initialized = False
        self.last_measure_t: Optional[float] = None

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self.pos[:] = 0.0
        self.vel[:] = 0.0
        self.last_t = None
        self.last_measure_t = None
        self.initialized = False

    def seed(self, uv: Tuple[float, float], t: float) -> None:
        self.pos[:] = np.asarray(uv, dtype=np.float64)
        self.vel[:] = 0.0
        self.last_t = t
        self.last_measure_t = t
        self.initialized = True

    # ------------------------------------------------------------------
    def predict(self, t: float) -> np.ndarray:
        """按当前速度外推到时刻 t（不吸收新测量）。"""
        if not self.initialized:
            return self.pos.copy()
        dt = max(0.0, float(t) - (self.last_t or t))
        return self.pos + self.vel * dt

    def update(self, uv: Optional[Tuple[float, float]], t: float,
               coast_s: float = 0.35) -> TrackState:
        """喂入一次测量（uv=None 表示本帧没测到）。

        coast_s: 允许"凭速度滑行"多久；超过则 valid=False。
        """
        t = float(t)
        if uv is None:
            if not self.initialized:
                return TrackState(valid=False)
            age = t - (self.last_measure_t or t)
            self.last_t = t
            return TrackState(uv=tuple(self.predict(t)), vel=tuple(self.vel),
                              valid=age <= coast_s, coasting=True, age_s=age)

        meas = np.asarray(uv, dtype=np.float64)
        if (not self.initialized) or self.last_t is None:
            self.seed(uv, t)
            return TrackState(uv=tuple(self.pos), vel=(0.0, 0.0), valid=True, age_s=0.0)

        dt = t - self.last_t
        if dt <= 1e-6:
            self.pos = meas.copy()
            self.last_t = t
            self.last_measure_t = t
            return TrackState(uv=tuple(self.pos), vel=tuple(self.vel), valid=True, age_s=0.0)
        if dt > self.stale_reset_s:
            # 停了太久（比如重捕获），重开滤波器
            self.seed(uv, t)
            return TrackState(uv=tuple(self.pos), vel=(0.0, 0.0), valid=True, age_s=0.0)

        pred = self.pos + self.vel * dt
        innov = meas - pred
        if float(np.linalg.norm(innov)) > self.jump_reset_px:
            # 突变（换了个目标 / 丢了又抓回来）-> 直接重开，别让速度被污染
            self.seed(uv, t)
            return TrackState(uv=tuple(self.pos), vel=(0.0, 0.0), valid=True, age_s=0.0)

        self.pos = pred + self.alpha * innov
        self.vel = self.vel + (self.beta / dt) * innov
        speed = float(np.linalg.norm(self.vel))
        if speed > self.max_speed:
            self.vel *= self.max_speed / speed
        self.last_t = t
        self.last_measure_t = t
        return TrackState(uv=tuple(self.pos), vel=tuple(self.vel), valid=True, age_s=0.0)

