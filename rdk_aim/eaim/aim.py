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
    stalled: bool = False            # 指令长时间无效（冻结积分中）
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
        # 偏航方向的"命令角度 -> 像素位移"额外缩放（见 config.yaw_gain_scale）：
        # H723 按昨天那版（半速率姿态积分）运行时，命令 1° 实际转 2°，
        # 所以这里乘 2 才能和真实被控对象对上。
        self.px_per_deg_u *= max(1e-3, float(getattr(cfg, "yaw_gain_scale", 1.0)))
        self.yaw_deg = 0.0
        self.pitch_deg = 0.0
        # 积分项状态（P+I 控制里的 I，见下面控制律说明）
        self._i_yaw = 0.0
        self._i_pitch = 0.0
        # D 项（误差变化率的低通滤波值 + 上一帧误差）
        self._d_u = 0.0
        self._d_v = 0.0
        self._last_e_u = 0.0
        self._last_e_v = 0.0
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
        self._i_yaw = 0.0
        self._i_pitch = 0.0
        self._d_u = 0.0
        self._d_v = 0.0
        self._last_e_u = 0.0
        self._last_e_v = 0.0
        self._last_yaw = self.yaw_deg
        self._last_pitch = self.pitch_deg
        self._lock_count = 0
        self._last_target = None
        self._last_target_t = -1e9
        self._last_vel = (0.0, 0.0)

    def set_offset(self, yaw_deg: float, pitch_deg: float,
                   yaw_limit_deg: Optional[float] = None) -> None:
        """直接设定偏置（扫描捕获阶段用）。

        注意：扫描是"直接设值"，所以顺带把积分量清零 —— 否则扫描期间攒下的
        积分会在下一次闭环时突然叠加上去，等于给一个莫名其妙的偏置跳变。

        yaw_limit_deg：扫描专用限幅。单向扫一圈时偏置会走到 360°，
        不能再用跟踪阶段的 ±170° 限幅，否则扫到 180° 以后就卡住不动了。
        """
        lim = self.cfg.max_yaw_deg if yaw_limit_deg is None else float(yaw_limit_deg)
        self.yaw_deg = float(max(-lim, min(lim, float(yaw_deg))))
        self.pitch_deg = float(self._clamp_pitch(pitch_deg))
        self._i_yaw = 0.0
        self._i_pitch = 0.0
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
        # 顺手把"指令无效冻结"也解开：重新捕获/重新进入跟踪时必须给一次机会，
        # 否则会死在"冻结 -> 不动 -> 误差不降 -> 一直冻结"里（现场就是这么丢的）。
        self.release_stall()

    def release_stall(self) -> None:
        """解开"指令无效"冻结，并清掉观察窗口。"""
        self._stall_t0 = None
        self._stall_frozen = False

    @property
    def locked(self) -> bool:
        return self._lock_count >= self.cfg.locked_frames

    # ------------------------------------------------------------------
    def update(self, now: float, dt: float,
               target_uv: Optional[Tuple[float, float]],
               target_vel: Optional[Tuple[float, float]],
               spot_uv: Optional[Tuple[float, float]],
               spot_ok: bool = True,
               att_rel: Optional[Tuple[float, float]] = None) -> AimOutput:
        """推进一次瞄准控制。

        target_uv / target_vel: 滤波后的靶心位置与像素速度（None = 本帧没测到）
        spot_uv: 光斑位置；为 None 时用标定好的光轴点
        att_rel: 云台【实际姿态】相对锁零基准的变化量（度），来自 H723 遥测。
                 用来做"防超前"限幅，见下面 lag_lead_deg 的说明。
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

        # ---- P + I 视觉伺服（2026-09-30 改，用户建议"增大 K、减小 I"）----
        # 原来只有 I（偏置一帧帧积分），现场反馈"很久才到靶心、到了马上又出去"：
        #   纯 I 环要把误差一点点"攒"过去，300px 得攒好几秒；而云台本身还有
        #   0.3~1s 滞后，攒的过程中平台还在动，于是又慢又摆。
        # 现在改成国一那种结构：**P 为主 + I 消静差**
        #     yaw_cmd = 实测姿态 + kp_p × (误差角度) + 积分量
        #   · P 项一次就给出"大部分该转的角度"（kp_p=0.55 → 一次给 55%），
        #     收敛速度立刻快一个量级；
        #   · 基准取【实测姿态】：平台还没转到位时命令也不会累积超前，
        #     天然不过冲（这正是之前"防超前"要解决的问题，现在从结构上解决了）；
        #   · I 项很小（ki=0.04，原 0.12）且限幅 ±8°，只抹最后的静差，
        #     避免像之前那样一路积分到 ±90° 跑飞。
        # 增益分档保留：误差很大时用 kp_coarse 把 I 放快一点。
        kp = cfg.kp_corner if boost else cfg.kp
        coarse = float(getattr(cfg, "kp_coarse", 0.0) or 0.0)
        if (not boost) and coarse > 0.0 and out.err_norm > cfg.coarse_err_px:
            kp = coarse
        dyaw = self.sign_yaw * eu / max(1e-6, self.px_per_deg_u)
        dpitch = self.sign_pitch * ev / max(1e-6, self.px_per_deg_v)
        e_deg_u, e_deg_v = dyaw, dpitch
        i_lim = float(getattr(cfg, "i_limit_deg", 3.0))
        # ⚠ 积分只在【误差已经很小】时才累加（条件积分 / 抗风阻）：
        #   现场"好不容易到靶心、马上又出去"就是积分风阻 —— 误差大时 I 一直
        #   在攒，等 P 把误差拉到 0，I 还顶在那儿放不掉，于是又朝反方向冲出去。
        #   现在误差 > i_band_px 时 I 保持不动，只有进到最后这一段才让它抹静差。
        i_band = float(getattr(cfg, "i_band_px", 40.0))
        if out.err_norm <= i_band:
            self._i_yaw = _clamp(self._i_yaw + kp * e_deg_u, i_lim)
            self._i_pitch = _clamp(self._i_pitch + kp * e_deg_v, i_lim)
        kp_p = float(getattr(cfg, "kp_p", 0.55))
        if att_rel is not None:
            # 有实测姿态：P 项挂在"实测姿态"这个锚点上（一次给 55% 的偏差），
            # 平台还没转到位时命令也不会累积 —— 快且不过冲。
            base_yaw = float(att_rel[0]) + self._i_yaw
            base_pitch = float(att_rel[1]) + self._i_pitch
            # ---- D 项（用户要求加阻尼）----
            # 误差变化率的低通滤波值：误差在快速变小（d<0）时把命令往回带一点，
            # 抑制"快到靶心时冲过头"；纯 P 在平台有滞后时正是这样过冲的。
            # 限幅 ±3°，防止单帧跳变（误检/丢帧）把命令甩出去。
            kp_d = float(getattr(cfg, "kp_d", 0.25))
            kp_d_v = float(getattr(cfg, "kp_d_pitch", 0.0))
            if kp_d > 0.0 and dt > 1e-4:
                d_u = (e_deg_u - self._last_e_u) / dt
                self._d_u = 0.7 * self._d_u + 0.3 * d_u
            else:
                self._d_u = 0.0
            # ⚠ 俯仰轴默认**不要 D**：俯仰环本来就是电机编码器闭环、响应快，
            #   而误差是按 10fps 采样的，差分会被噪声主导 —— 现场表现就是
            #   "打中之后俯仰电机一直上下跳"。要加也得从 0.05 这种小值起步。
            if kp_d_v > 0.0 and dt > 1e-4:
                d_v = (e_deg_v - self._last_e_v) / dt
                self._d_v = 0.7 * self._d_v + 0.3 * d_v
            else:
                self._d_v = 0.0
            d_lim = float(getattr(cfg, "d_limit_deg", 3.0))
            d_yaw = _clamp(kp_d * self._d_u, d_lim)
            d_pitch = _clamp(kp_d_v * self._d_v, d_lim)
            # 俯仰的比例增益单独给（可以比偏航小）：偏航要克服静摩擦与滞后，
            # 俯仰已经很跟手，P 给大了只会把像素噪声放大成"上下跳"。
            kp_p_v = float(getattr(cfg, "kp_p_pitch", kp_p))
            new_yaw = base_yaw + kp_p * e_deg_u + d_yaw
            new_pitch = base_pitch + kp_p_v * e_deg_v + d_pitch
        else:
            # ⚠ 没有姿态遥测时**绝对不能加 P**：没有锚点，P 项会一帧一帧叠加，
            #   几帧就把偏置推到限幅（单元测试直接抓到了：稳态误差 1346px）。
            #   这时退回纯 I（原行为），保证没有遥测也能用。
            new_yaw = self.yaw_deg + kp * e_deg_u
            new_pitch = self.pitch_deg + kp * e_deg_v
        self._last_e_u, self._last_e_v = e_deg_u, e_deg_v

        # ---- "指令无效"保护（防姿态漂移把偏置一路带跑）----
        # 现场实测（2026-09-30，CSV 逐帧）：光斑和靶心在画面里【都静止不动】
        # （画面完全没变），但 H723 上报的姿态角以 2.4°/s 匀速漂 —— 说明平台
        # 实际没转、姿态估计在漂（陀螺零偏残留）。视觉环却以为"还没到位"，
        # 于是偏置一路积分：-30° → -89°，25 秒里匀速跑飞，最后必然撞限幅。
        # 判据：误差长时间不下降、偏置却已经动了不少 —— 那就是"下了指令没效果"，
        # 继续积分毫无意义。冻结积分、记一条警告，等误差自己变小再恢复。
        # （这不是取代 H723 侧的零偏标定，只是让上位机别跟着漂。）
        cfg_stall = cfg
        stall_err = float(getattr(cfg_stall, "stall_err_px", 60.0))
        stall_s = float(getattr(cfg_stall, "stall_s", 2.5))
        stall_off = float(getattr(cfg_stall, "stall_delta_deg", 5.0))
        if out.err_norm > stall_err:
            if getattr(self, "_stall_t0", None) is None:
                self._stall_t0 = now
                self._stall_off0 = self.yaw_deg
                self._stall_err0 = out.err_norm
            elif (now - self._stall_t0) > stall_s:
                moved = abs(self.yaw_deg - getattr(self, "_stall_off0", self.yaw_deg))
                improved = out.err_norm < 0.8 * getattr(self, "_stall_err0",
                                                        out.err_norm)
                if moved > stall_off and not improved:
                    # 冻结最长只持续 stall_hold_s：它只是"别跟着漂"，
                    # 绝不能变成"永远不动"（否则丢靶重捕后永远收敛不了）。
                    # ⚠ 计时只在【刚进入冻结】那一刻设一次 —— 如果每帧都刷新，
                    #   截止时间会被一直往后推，等于永远不解冻（单元测试抓到了）。
                    if not getattr(self, "_stall_frozen", False):
                        self._stall_until = now + float(
                            getattr(cfg_stall, "stall_hold_s", 2.0))
                    self._stall_frozen = True
        else:
            self._stall_t0 = None
            self._stall_frozen = False
        if getattr(self, "_stall_frozen", False) and \
                now >= getattr(self, "_stall_until", 0.0):
            self.release_stall()
        if getattr(self, "_stall_frozen", False):
            # 冻结：偏置保持不动（既不积分也不回退），只更新日志用的量
            new_yaw = self.yaw_deg
            new_pitch = self.pitch_deg
            out.stalled = True

        # ---- 变化率限幅：重捕获瞬间不猛冲（保护机械与电缆） ----
        max_step = cfg.rate_limit_dps * max(1e-4, dt)
        new_yaw = self._last_yaw + _clamp(new_yaw - self._last_yaw, max_step)
        new_pitch = self._last_pitch + _clamp(new_pitch - self._last_pitch, max_step)

        self.yaw_deg = self._clamp_yaw(new_yaw)
        self.pitch_deg = self._clamp_pitch(new_pitch)
        # ---- 防超前限幅（只压偏航）----
        # 现场逐帧数据（2026-09-29）：误差已经收到 -0.3px 时，命令偏置停在
        # 2.93° 没动，可平台还在追更早的指令 —— gz_yaw 从 2.32 一路爬到 3.17，
        # 误差于是从 0 冲到 +28px（正好 0.74°×38px/°）。这就是"明明到靶心了
        # 又晃走"的直接原因。
        # 做法：命令偏置最多只允许比【实际姿态】超前 lag_lead_deg；
        # 落后不限（要能把偏差拉回来），扫描时的直接设值也不受这里管。
        if att_rel is not None:
            try:
                # 允许的超前量随误差放宽：
                #   误差小时（<40px）只允许超前 1.2° —— 防过冲；
                #   误差大时要敢于领先，否则偏航轴的静摩擦死区会把平台卡住
                #   （实测：命令 1.5° 时平台 0.7s 只走了 0.34°），
                #   所以按 0.35°/px 把超前量放到最多 lag_lead_max_deg。
                base_lead = float(getattr(cfg, "lag_lead_deg", 1.2))
                lead_max = float(getattr(cfg, "lag_lead_max_deg", 8.0))
                err_deg = abs(float(eu)) / max(1.0, self.px_per_deg_u)
                lead = min(lead_max, max(base_lead, 0.6 * err_deg))
                cap = float(att_rel[0]) + max(0.0, lead)
                if self.yaw_deg > cap:
                    self.yaw_deg = cap
                    self._last_yaw = min(self._last_yaw, cap)
            except Exception:                                  # noqa: BLE001
                pass
        self._last_yaw, self._last_pitch = self.yaw_deg, self.pitch_deg
        out.yaw_deg, out.pitch_deg = self.yaw_deg, self.pitch_deg

        # ---- 锁定判据 ----
        # ⚠ 相机只有 ~9fps、检测偶尔会漏一帧。原来"只要 coasting 就清零"，
        #   结果连续 6 帧几乎永远攒不够 —— 实测锁定时间被拖到 10~18 秒。
        #   这里允许最多 2 帧的短时滑行（tracker 的位置仍然有效、误差也是
        #   按外推算的），不打断计数；真正的偏差才清零。
        self._coast_run = getattr(self, "_coast_run", 0) + 1 if coasting else 0
        if out.err_norm <= cfg.locked_tol_px and self._coast_run <= 2:
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
