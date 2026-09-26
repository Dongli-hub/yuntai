"""瞄准控制器：积分型视觉伺服 + 目标速度超前补偿。

控制律（每帧执行一次）::

    e      = 靶心像素(带超前) - 光斑像素
    dtheta = e / px_per_deg           # 把光斑挪 e 像素需要转的角度
    offset += kp * dtheta             # 累加进云台偏置

为什么"只累加"就够：
    offset 本身就是角度的积分，而 dtheta 是"这一帧还差多少角度"。
    把还差的角度乘系数不断累加，等价于一个 I 型（一阶无静差）系统：
    稳态时 e 必为 0，也就是激光严格落在靶心上。
    这个结论不依赖任何增益标定的准确性——标定有 20% 误差也只影响收敛速度。

为什么增益与距离无关：
    偏置转 theta 度 -> 靶心在图像里移动 fx*theta 像素，与靶的距离无关
    （纯转动不改变距离）。所以 px_per_deg = fx*pi/180 就是环路的
    "被控对象增益"，把它除掉以后剩下的 kp 是一个无量纲的、与距离无关的旋钮。

kp 怎么选：一帧纯延迟时特征方程 z^2 - z + kp = 0
    kp = 0.25 -> 实轴重根（临界阻尼，最优）
    kp = 0.35 -> 半径 0.59 的复根（略超调，收敛更快，推荐起点）
    kp >= 1.0 -> 单位圆上，等幅振荡
"""

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from .config import ControlConfig
from .geometry import CameraModel

__all__ = ["AimOutput", "AimController"]


@dataclass
class AimOutput:
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    err_u: float = 0.0                # 本帧实际用于控制的像素误差（已含超前）
    err_v: float = 0.0
    raw_err_u: float = 0.0            # 未加超前的原始误差（用于日志/判据）
    raw_err_v: float = 0.0
    err_norm: float = 0.0             # 误差模长，px
    valid: bool = False               # 本帧有真实目标测量
    coasting: bool = False            # 靠外推滑行中
    locked: bool = False
    boost: bool = False
    quality: int = 0                  # 0~255，给 H723/RCT6 看的粗略质量

    def describe(self) -> str:
        return ("off=(%.2f,%.2f)d e=(%.1f,%.1f)px |e|=%.1f valid=%d coast=%d "
                "lock=%d boost=%d q=%d"
                % (self.yaw_deg, self.pitch_deg, self.err_u, self.err_v,
                   self.err_norm, self.valid, self.coasting, self.locked,
                   self.boost, self.quality))


class AimController:
    """把"两个像素点"的误差变成云台角度偏置。"""

    def __init__(self, cfg: ControlConfig, cam: CameraModel,
                 sign_yaw: float = 1.0, sign_pitch: float = 1.0):
        self.cfg = cfg
        self.cam = cam
        self.sign_yaw = float(sign_yaw)
        self.sign_pitch = float(sign_pitch)
        self.px_per_deg_u, self.px_per_deg_v = cam.px_per_deg
        self.yaw_deg = 0.0
        self.pitch_deg = 0.0
        self._last_yaw = 0.0
        self._last_pitch = 0.0
        self._lock_count = 0
        self._boost_until = -1e9
        self._lead_damp_until = -1e9
        self._last_target: Optional[Tuple[float, float]] = None
        self._last_vel: Tuple[float, float] = (0.0, 0.0)
        self._last_target_t = -1e9

    # ------------------------------------------------------------------
    def reset(self, yaw_deg: float = 0.0, pitch_deg: float = 0.0) -> None:
        self.yaw_deg = float(yaw_deg)
        self.pitch_deg = float(pitch_deg)
        self._last_yaw = self.yaw_deg
        self._last_pitch = self.pitch_deg
        self._lock_count = 0
        self._last_target = None
        self._last_target_t = -1e9
        self._last_vel = (0.0, 0.0)

    def set_offset(self, yaw_deg: float, pitch_deg: float) -> None:
        """直接设定偏置（扫描捕获阶段用）。"""
        self.yaw_deg = float(self._clamp_yaw(yaw_deg))
        self.pitch_deg = float(self._clamp_pitch(pitch_deg))
        self._last_yaw = self.yaw_deg
        self._last_pitch = self.pitch_deg

    def note_corner(self, now: float) -> None:
        """RCT6 报告拐角事件。

        拐角的物理本质是**目标像素速度方向发生突变**（小车 90° 转向，
        载着靶的面视速度从"横向掠过"变成"远离"）。
        此时基于历史速度的超前补偿反而会把误差推大 —— 实测拐角处
        误差尖峰主要就是这个原因。所以：
          * 提高跟踪增益（加快把误差拉回来）
          * 同时**压低超前补偿**（速度估计此刻不可信）
        """
        self._boost_until = now + self.cfg.corner_boost_s
        self._lead_damp_until = now + self.cfg.corner_boost_s

    def clear_lock(self) -> None:
        self._lock_count = 0

    @property
    def locked(self) -> bool:
        return self._lock_count >= self.cfg.locked_frames

    # ------------------------------------------------------------------
    def update(self, now: float, dt: float,
               target_uv: Optional[Tuple[float, float]],
               target_vel: Optional[Tuple[float, float]],
               spot_uv: Optional[Tuple[float, float]],
               spot_ok: bool = True) -> AimOutput:
        """推进一次瞄准控制。

        target_uv / target_vel: 滤波后的靶心位置与像素速度（None = 本帧没测到）
        spot_uv: 光斑位置；为 None 时用标定好的光轴点
        """
        cfg = self.cfg
        out = AimOutput(yaw_deg=self.yaw_deg, pitch_deg=self.pitch_deg)
        boost = now < self._boost_until
        out.boost = boost

        if spot_uv is None:
            return out                       # 连光斑位置都不知道，什么都不做

        if target_uv is not None:
            self._last_target = (float(target_uv[0]), float(target_uv[1]))
            self._last_vel = (float(target_vel[0]), float(target_vel[1])) \
                if target_vel else (0.0, 0.0)
            self._last_target_t = now
            coasting = False
        else:
            if self._last_target is None:
                return out
            age = now - self._last_target_t
            if age > cfg.lost_coast_s:
                out.coasting = False
                out.valid = False
                return out
            # 短时滑行：用最后的速度外推目标位置，继续闭环
            lead = age
            self._last_target = (self._last_target[0] + self._last_vel[0] * lead,
                                 self._last_target[1] + self._last_vel[1] * lead)
            self._last_target_t = now
            coasting = True

        # ---- 超前补偿：抵消曝光+处理+串口+执行的总延迟 ----
        lead_s = cfg.lead_s * cfg.lead_gain
        if boost:
            lead_s *= cfg.corner_lead_scale
        tu = self._last_target[0] + self._last_vel[0] * lead_s
        tv = self._last_target[1] + self._last_vel[1] * lead_s
        raw_eu = self._last_target[0] - float(spot_uv[0])
        raw_ev = self._last_target[1] - float(spot_uv[1])
        eu = tu - float(spot_uv[0])
        ev = tv - float(spot_uv[1])

        out.raw_err_u, out.raw_err_v = raw_eu, raw_ev
        out.err_u, out.err_v = eu, ev
        out.err_norm = math.hypot(raw_eu, raw_ev)
        out.valid = spot_ok and not coasting
        out.coasting = coasting

        # ---- 死区：误差很小就别动，省得被像素噪声推得来回抖 ----
        db = cfg.deadband_px
        if abs(eu) < db:
            eu = 0.0
        if abs(ev) < db:
            ev = 0.0

        # ---- 积分型视觉伺服 ----
        kp = cfg.kp_corner if boost else cfg.kp
        dyaw = self.sign_yaw * eu / max(1e-6, self.px_per_deg_u)
        dpitch = self.sign_pitch * ev / max(1e-6, self.px_per_deg_v)
        new_yaw = self.yaw_deg + kp * dyaw
        new_pitch = self.pitch_deg + kp * dpitch

        # ---- 变化率限幅：重捕获瞬间不猛冲（保护机械与电缆） ----
        max_step = cfg.rate_limit_dps * max(1e-4, dt)
        new_yaw = self._last_yaw + _clamp(new_yaw - self._last_yaw, max_step)
        new_pitch = self._last_pitch + _clamp(new_pitch - self._last_pitch, max_step)

        self.yaw_deg = self._clamp_yaw(new_yaw)
        self.pitch_deg = self._clamp_pitch(new_pitch)
        self._last_yaw, self._last_pitch = self.yaw_deg, self.pitch_deg
        out.yaw_deg, out.pitch_deg = self.yaw_deg, self.pitch_deg

        # ---- 锁定判据 ----
        if out.err_norm <= cfg.locked_tol_px and not coasting:
            self._lock_count += 1
        else:
            self._lock_count = 0
        out.locked = self.locked

        # ---- 粗糙质量分（0~255），给 H723 与 RCT6 做联锁/降速 ----
        q = 255.0 * (1.0 - min(1.0, out.err_norm / 120.0))
        out.quality = int(max(0, min(255, q)))
        return out

    # ------------------------------------------------------------------
    def _clamp_yaw(self, value: float) -> float:
        lim = self.cfg.max_yaw_deg
        return _clamp(value, lim)

    def _clamp_pitch(self, value: float) -> float:
        lim = self.cfg.max_pitch_deg
        return _clamp(value, lim)

    def angle_error_deg(self, target_uv, spot_uv) -> Tuple[float, float]:
        """把像素误差换算成"还差多少度"，用于日志与诊断。"""
        return ((float(target_uv[0]) - float(spot_uv[0])) / max(1e-6, self.px_per_deg_u),
                (float(target_uv[1]) - float(spot_uv[1])) / max(1e-6, self.px_per_deg_v))


def _clamp(value: float, limit: float) -> float:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value
