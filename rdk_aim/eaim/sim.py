"""PC 端全闭环仿真：合成靶纸 + 云台执行器模型 + 小车沿赛道运动。

为什么值得写这个模块：
  视觉伺服最难的部分不是算法而是"现场整定"。有了仿真，kp / lead_s / 检测阈值
  都可以先在 PC 上跑通、看到误差曲线，现场只需小幅微调。
  而且仿真是**走完整协议链路**的（SimLink 真的会打包/解包字节帧），
  所以协议本身的 bug 也能在这里暴露。

坐标系（纸面系 P）：
    x 向右，y 向下（重力方向），z 指向纸面内侧。
    靶纸 = z=0 平面上 [0,210]x[0,297] 的矩形，靶心 = (105, 148.5, 0)。
    地面在 y = 148.5 + 250 = 398.5（靶心离地 250mm）。
    赛道 = 靶面外 500mm 处的一条 1m x 1m 方形线（与赛题一致），
    A=(-500,500) B=(500,500) D=(500,1500) C=(-500,1500)（单位 mm，x-z 平面）。

激光模型与真实同轴装配一致：激光**平行于光轴**但存在横向偏置
（默认右 30mm、下 20mm）。于是光斑在图像里的位置会随距离轻微变化
（800mm 处偏 26px、1500mm 处偏 14px）——这正是真实系统会遇到的情况。
仿真里保留这个效应，程序就必须真的靠"光斑闭环"工作，
而不是依赖"假设光斑固定在画面中心"这种理想化前提。
"""

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import cv2
import numpy as np

from . import protocol as proto
from .config import AppConfig
from .geometry import CameraModel
from .protocol import (AimFlags, AimMode, CarCmd, CarEvent, CarState, Frame,
                       GimbalState)

__all__ = ["SimWorld", "SimLink", "SimStatus"]


def _rot_x(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def _rot_y(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


class _DelayLine:
    """纯延迟环节（模拟曝光 + 处理 + 串口 + 驱动器的总延迟）。"""

    def __init__(self, delay_s: float, initial: Tuple[float, float] = (0.0, 0.0)):
        self.delay = max(0.0, float(delay_s))
        self.buf: Deque[Tuple[float, float, float]] = deque()
        self.buf.append((0.0, initial[0], initial[1]))

    def push(self, t: float, yaw: float, pitch: float) -> None:
        self.buf.append((float(t), float(yaw), float(pitch)))
        while len(self.buf) > 2 and (t - self.buf[0][0]) > self.delay * 4.0 + 2.0:
            self.buf.popleft()

    def get(self, t: float) -> Tuple[float, float]:
        out = (self.buf[0][1], self.buf[0][2])
        while len(self.buf) > 1 and (t - self.buf[1][0]) >= self.delay:
            out = (self.buf[1][1], self.buf[1][2])
            self.buf.popleft()
        return out


@dataclass
class SimStatus:
    yaw_cmd: float = 0.0
    pitch_cmd: float = 0.0
    yaw_applied: float = 0.0
    pitch_applied: float = 0.0
    miss_mm: float = 0.0
    spot_on_paper: bool = False
    rel_yaw_deg: float = 0.0
    car_x: float = 0.0
    car_z: float = 0.0
    car_s: float = 0.0
    laps_done: int = 0
    laser_on: bool = False


class SimWorld:
    """仿真世界。长度单位 mm，角度单位度，时间单位秒。"""

    def __init__(self, cfg: AppConfig):
        self.cfg = cfg
        self.cam = CameraModel(fx=cfg.calib.fx, fy=cfg.calib.fy,
                               cx=cfg.calib.cx, cy=cfg.calib.cy,
                               dist=cfg.calib.dist)
        s = cfg.sim
        self.ground_y = 148.5 + 250.0
        self.cam_height = 150.0
        self.laser_offset_cam = np.array([30.0, 20.0, 0.0])
        self.track = [(-500.0, 500.0), (500.0, 500.0), (500.0, 1500.0),
                      (-500.0, 1500.0)]
        self.seg_len = [1000.0, 1000.0, 1000.0, 1000.0]
        self.total_len = float(sum(self.seg_len))
        self.car_heading_table = [0.0, 90.0, 180.0, 270.0]
        self._rng = np.random.RandomState(20250925)
        self._lock = threading.RLock()
        self.mode = AimMode.IDLE
        self.laser_on = False
        self.cmd_yaw = 0.0
        self.cmd_pitch = 0.0
        self.applied_yaw = 0.0
        self.applied_pitch = 0.0
        self.rel_yaw = 0.0
        self.fault = 0
        self._delay = _DelayLine(s.actuator_delay_s)
        self._last_t = 0.0
        self.t0 = time.monotonic()
        self.car_running = False
        self.car_s = 0.0
        self.car_speed_mm_s = self.total_len / max(1.0, s.car_lap_time_s)
        self.car_laps_done = 0
        self.car_laps_target = 0
        self.car_seg = 0
        self._next_corner_s = 1000.0
        self._heading_prev = 0.0
        self._last_car_telemetry = 0.0
        self.gimbal_tx: Deque[bytes] = deque()
        self.car_tx: Deque[bytes] = deque()
        self._seq_g = 0
        self._seq_c = 0
        self._paper_tex: Optional[np.ndarray] = None
        self.paper_px_per_mm = 3.0
        self.corner_events: List[int] = []
        self._noise_cache = {}

    def step(self, t: float) -> None:
        with self._lock:
            dt = min(0.2, max(0.0, t - self._last_t))
            self._last_t = t
            if dt <= 0.0:
                return
            self._advance_car(dt)
            self._advance_gimbal(t, dt)
            self._car_telemetry(t)

    def _advance_car(self, dt: float) -> None:
        if not self.cfg.sim.car_enable or not self.car_running:
            return
        self.car_s += self.car_speed_mm_s * dt
        while self.car_s >= self._next_corner_s - 1e-9 and \
                self._next_corner_s < self.total_len:
            idx = int(round(self._next_corner_s / 1000.0)) % 4
            self._push_car_event(CarEvent.CORNER, idx, 0)
            self.corner_events.append(idx)
            self._next_corner_s += 1000.0
        if self.car_s >= self.total_len:
            self.car_s -= self.total_len
            self._next_corner_s -= self.total_len
            self.car_laps_done += 1
            self._push_car_event(CarEvent.LAP_DONE, self.car_laps_done, 0)
            if self.car_laps_target and self.car_laps_done >= self.car_laps_target:
                self.car_running = False
                self._push_car_event(CarEvent.STOPPED, self.car_seg, 0)
        self.car_seg = int(self.car_s // 1000.0) % 4

    def _advance_gimbal(self, t: float, dt: float) -> None:
        s = self.cfg.sim
        self._delay.push(t, self.cmd_yaw, self.cmd_pitch)
        tgt_yaw, tgt_pitch = self._delay.get(t)
        if self.mode in (AimMode.IDLE, AimMode.ESTOP):
            tgt_yaw, tgt_pitch = self.applied_yaw, self.applied_pitch
        elif self.mode == AimMode.STAB:
            tgt_yaw, tgt_pitch = 0.0, 0.0
        max_step = max(1e-6, s.actuator_max_rate_dps * dt)
        alpha = 1.0 - math.exp(-dt / max(1e-3, s.actuator_tau_s))
        step_y = max(-max_step, min(max_step, tgt_yaw - self.applied_yaw))
        step_p = max(-max_step, min(max_step, tgt_pitch - self.applied_pitch))
        self.applied_yaw += step_y * alpha
        self.applied_pitch += step_p * alpha
        if self.car_running:
            heading = self.car_heading_table[self.car_seg % 4]
            dhead = ((heading - self._heading_prev + 540.0) % 360.0) - 180.0
            self.rel_yaw -= dhead
            self._heading_prev = heading
        if self.mode == AimMode.UNWIND:
            rate = 30.0 * dt
            self.rel_yaw += max(-rate, min(rate, -self.rel_yaw))
            self.laser_on = False

    def _car_telemetry(self, t: float) -> None:
        if t - self._last_car_telemetry < 0.05:
            return
        self._last_car_telemetry = t
        st = self.car_state()
        self._seq_c = (self._seq_c + 1) & 0xFF
        self.car_tx.append(proto.build_frame(
            proto.MsgId.CAR_STATE,
            proto.pack_car_state(st.state, st.lap_index, st.segment, st.progress,
                                 int(st.speed_mm_s), st.lap_time_ms,
                                 st.gray_bits, st.flags),
            self._seq_c))

    def _push_car_event(self, event: int, index: int, value: int = 0) -> None:
        self._seq_c = (self._seq_c + 1) & 0xFF
        self.car_tx.append(proto.build_frame(
            proto.MsgId.CAR_EVENT, proto.pack_car_event(event, index, value),
            self._seq_c))

    def _push_gimbal_state(self, t: float) -> None:
        st = self.gimbal_state(t)
        self._seq_g = (self._seq_g + 1) & 0xFF
        self.gimbal_tx.append(proto.build_frame(
            proto.MsgId.GIMBAL_STATE,
            proto.pack_gimbal_state(st.state, st.fault, st.yaw_deg, st.pitch_deg,
                                    st.roll_deg, st.yaw_motor_deg,
                                    st.pitch_motor_deg, st.gyro_y_dps,
                                    st.gyro_z_dps, st.flags, st.uptime_ms),
            self._seq_g))

    # ------------------------------------------------------------------
    # 下行处理（地瓜派 -> 世界）
    # ------------------------------------------------------------------
    def on_aim(self, payload: bytes) -> None:
        msg = proto.unpack_aim(payload)
        with self._lock:
            self.cmd_yaw = msg["yaw_deg"]
            self.cmd_pitch = msg["pitch_deg"]
            self.laser_on = bool(msg["flags"] & AimFlags.LASER_ON) and \
                self.mode == AimMode.AIM

    def on_mode(self, payload: bytes) -> None:
        msg = proto.unpack_mode(payload)
        with self._lock:
            self.mode = int(msg["mode"])
            if self.mode != AimMode.AIM:
                self.laser_on = False
            if self.mode == AimMode.STAB:
                self.cmd_yaw = 0.0
                self.cmd_pitch = 0.0

    def on_car_cmd(self, payload: bytes) -> None:
        msg = proto.unpack_car_cmd(payload)
        with self._lock:
            cmd = int(msg["cmd"])
            if cmd == CarCmd.START:
                self.car_running = True
                self.car_laps_done = 0
                seg = max(0, min(3, int(msg["arg"])))
                self.car_s = seg * 1000.0
                self.car_seg = seg
                self._heading_prev = self.car_heading_table[seg]
                self._next_corner_s = (seg + 1) * 1000.0
                self._push_car_event(CarEvent.STARTED, seg, 0)
            elif cmd == CarCmd.STOP:
                self.car_running = False
                self._push_car_event(CarEvent.STOPPED, self.car_seg, 0)
            elif cmd == CarCmd.SET_LAPS:
                self.car_laps_target = max(1, int(msg["arg"]))
            elif cmd == CarCmd.ZERO_TIMER:
                self.car_laps_done = 0

    def on_gimbal_poll(self, t: float) -> None:
        with self._lock:
            self._push_gimbal_state(t)

    # ------------------------------------------------------------------
    # 状态
    # ------------------------------------------------------------------
    def gimbal_state(self, t: float) -> GimbalState:
        flags = 0
        if self.mode in (AimMode.AIM, AimMode.STAB, AimMode.UNWIND):
            flags |= 0x01
            flags |= 0x02
        flags |= 0x04
        # READY 位：启动流程走完、可以接受偏置（上位机只认这一位）
        if self.mode != AimMode.IDLE:
            flags |= 0x10
        if self.laser_on:
            flags |= 0x08
        state = 2 if self.mode == AimMode.IDLE else 6
        return GimbalState(
            state=state, fault=self.fault,
            yaw_deg=self.applied_yaw, pitch_deg=self.applied_pitch, roll_deg=0.0,
            yaw_motor_deg=self.rel_yaw, pitch_motor_deg=self.applied_pitch,
            gyro_y_dps=0.0, gyro_z_dps=0.0, flags=flags,
            uptime_ms=int((t - self.t0) * 1000.0),
        )

    def car_state(self) -> CarState:
        s = self.car_s % self.total_len
        seg = int(s // 1000.0) % 4
        prog = int(round((s - seg * 1000.0) / 10.0))
        lap_ms = int(round(self.total_len / max(1.0, self.car_speed_mm_s) * 1000.0))
        return CarState(
            state=0x01 if self.car_running else 0x00,
            lap_index=max(1, self.car_laps_done + 1),
            segment=seg, progress=max(0, min(100, prog)),
            speed_mm_s=int(self.car_speed_mm_s if self.car_running else 0),
            lap_time_ms=lap_ms, gray_bits=0x18, flags=0,
        )

    def status(self) -> SimStatus:
        x, z = self.car_xz()
        miss, on_paper = self.truth_miss_mm()
        return SimStatus(yaw_cmd=self.cmd_yaw, pitch_cmd=self.cmd_pitch,
                         yaw_applied=self.applied_yaw,
                         pitch_applied=self.applied_pitch,
                         miss_mm=miss, spot_on_paper=on_paper,
                         rel_yaw_deg=self.rel_yaw, car_x=x, car_z=z,
                         car_s=self.car_s, laps_done=self.car_laps_done,
                         laser_on=self.laser_on)

    # ------------------------------------------------------------------
    # 几何
    # ------------------------------------------------------------------
    def car_xz(self) -> Tuple[float, float]:
        s = self.car_s % self.total_len
        acc = 0.0
        for i, L in enumerate(self.seg_len):
            if s < acc + L:
                f = (s - acc) / L
                p0 = np.array(self.track[i])
                p1 = np.array(self.track[(i + 1) % 4])
                p = p0 + (p1 - p0) * f
                return float(p[0]), float(p[1])
            acc += L
        return float(self.track[0][0]), float(self.track[0][1])

    def camera_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """返回 (相机在纸面系的位置 C, 相机姿态 R_pc)。"""
        x, z = self.car_xz()
        C = np.array([105.0 + x, self.ground_y - self.cam_height, -(500.0 + z)])
        d = -C
        dn = d / np.linalg.norm(d)
        base_pitch = -math.degrees(math.asin(max(-1.0, min(1.0, dn[1]))))
        base_yaw = math.degrees(math.atan2(dn[0], dn[2]))
        R = _rot_y(base_yaw + self.applied_yaw) @ \
            _rot_x(base_pitch + self.applied_pitch)
        return C, R

    def homography_mm_to_img(self) -> np.ndarray:
        """纸面 mm -> 图像像素 的单应矩阵（平面投影必然是同应变换）。"""
        C, R = self.camera_pose()
        Rt = R.T
        H = self.cam.K @ np.stack([Rt[:, 0], Rt[:, 1], -Rt @ C], axis=1)
        return H / H[2, 2]

    def laser_spot_point(self) -> Optional[np.ndarray]:
        """激光落点（射线与纸面 z=0 平面求交）。"""
        C, R = self.camera_pose()
        origin = C + R @ self.laser_offset_cam
        direction = R @ np.array([0.0, 0.0, 1.0])
        if abs(direction[2]) < 1e-9:
            return None
        t = (0.0 - origin[2]) / direction[2]
        if t <= 0:
            return None
        return origin + direction * t

    def truth_miss_mm(self) -> Tuple[float, bool]:
        """真值：激光落点到靶心的距离（mm），以及是否落在靶纸上。"""
        p = self.laser_spot_point()
        if p is None:
            return 1e9, False
        on_paper = (-1.0 <= p[0] <= 211.0) and (-1.0 <= p[1] <= 298.0)
        return math.hypot(p[0] - 105.0, p[1] - 148.5), on_paper

    # ------------------------------------------------------------------
    # 渲染
    # ------------------------------------------------------------------
    def paper_texture(self) -> np.ndarray:
        if self._paper_tex is not None:
            return self._paper_tex
        k = self.paper_px_per_mm
        W, H = int(round(210.0 * k)), int(round(297.0 * k))
        tex = np.full((H, W, 3), 236, np.uint8)
        t = int(round(18.0 * k))
        cv2.rectangle(tex, (0, 0), (W - 1, H - 1), (26, 26, 30), -1)
        cv2.rectangle(tex, (t, t), (W - 1 - t, H - 1 - t), (236, 236, 236), -1)
        center = (int(round(W / 2)), int(round(H / 2)))
        line = max(1, int(round(1.0 * k)))
        for r_mm in (20, 40, 60, 80, 100):
            cv2.circle(tex, center, int(round(r_mm * k)), (40, 40, 205), line)
        cv2.circle(tex, center, max(1, int(round(0.6 * k))), (40, 40, 205), -1)
        noise = self._rng.normal(0.0, 3.0, tex.shape)
        self._paper_tex = np.clip(tex.astype(np.float32) + noise, 0,
                                  255).astype(np.uint8)
        return self._paper_tex

    def render(self, t: float) -> Optional[np.ndarray]:
        with self._lock:
            self.step(t)
            cfg = self.cfg
            W, H = int(cfg.camera.width), int(cfg.camera.height)
            Hmm = self.homography_mm_to_img()
            k = self.paper_px_per_mm
            Htex = Hmm @ np.diag([1.0 / k, 1.0 / k, 1.0])
            frame = cv2.warpPerspective(self.paper_texture(), Htex, (W, H),
                                        flags=cv2.INTER_LINEAR,
                                        borderMode=cv2.BORDER_CONSTANT,
                                        borderValue=(158, 158, 158))
            if self.laser_on:
                frame = self._draw_spot(frame)
            if cfg.sim.noise > 0:
                noise = self._noise(frame.shape[:2], cfg.sim.noise)
                frame = cv2.add(frame, noise, dtype=cv2.CV_8U)
            return frame

    def _noise(self, shape, sigma: float) -> np.ndarray:
        """固定图样的噪声（相当于传感器的固定图样噪声），缓存后按帧复用。

        性能说明：每帧现场生成 (720,1280,3) 正态噪声要 30ms 级，仿真根本跑不动；
        而 cv2.add(..., dtype=CV_8U) 只要 1.5ms，差 8 倍。
        """
        key = (shape[0], shape[1], round(float(sigma), 3))
        cached = self._noise_cache.get(key)
        if cached is not None:
            return cached
        tile = np.clip(self._rng.normal(0.0, 6.0, (256, 256, 3)), -30.0, 30.0)
        reps = (shape[0] // 256 + 2, shape[1] // 256 + 2, 1)
        tiled = np.tile(tile, reps)[:shape[0], :shape[1]]
        if abs(sigma - 6.0) > 1e-6:
            tiled = tiled * (sigma / 6.0)
        arr = np.clip(tiled, -127, 127).astype(np.int16)
        if len(self._noise_cache) > 4:
            self._noise_cache.clear()
        self._noise_cache[key] = arr
        return arr

    def _draw_spot(self, frame: np.ndarray) -> np.ndarray:
        p = self.laser_spot_point()
        if p is None:
            return frame
        # 光斑点在"纸面系"里，必须先变到相机坐标系再投影
        # （纸面系里 p 的 z 恰好约等于 0，直接投影会除以 0 得到天文数字）
        C, R = self.camera_pose()
        p_cam = R.T @ (p - C)
        uv = self.cam.project(np.array([p_cam]))[0]
        u, v = float(uv[0]), float(uv[1])
        if not (-200.0 <= u <= frame.shape[1] + 200.0):
            return frame
        if not (-200.0 <= v <= frame.shape[0] + 200.0):
            return frame
        r_out, r_in = 9.0, 3.2
        x0 = int(max(0, u - r_out * 3))
        y0 = int(max(0, v - r_out * 3))
        x1 = int(min(frame.shape[1], u + r_out * 3))
        y1 = int(min(frame.shape[0], v + r_out * 3))
        if x1 <= x0 or y1 <= y0:
            return frame
        ys, xs = np.mgrid[y0:y1, x0:x1]
        d = np.sqrt((xs - u) ** 2 + (ys - v) ** 2)
        core = np.clip(1.0 - d / max(1e-6, r_in), 0.0, 1.0)
        halo = np.exp(-(d / r_out) ** 2) * 0.85
        gain = np.clip(core + halo, 0.0, 1.0)[:, :, None]
        # BGR：405nm 蓝紫在相机里的典型响应 —— B 饱和，R 中等，G 最低
        # （注意不能让 R 也饱和，否则"蓝优势"判据失效，这正是真实情况的写照）
        spot_color = np.array([255.0, 90.0, 150.0])
        region = frame[y0:y1, x0:x1].astype(np.float32)
        blended = region * (1.0 - gain) + spot_color[None, None, :] * gain
        frame[y0:y1, x0:x1] = np.clip(blended, 0, 255).astype(np.uint8)
        return frame


class SimLink:
    """仿真链路：走完整的协议打包/解包，但底层不是串口。

    接口与 link.SerialLink 一致（start/send/read_frames/healthy/close/stats），
    所以 app.py 里两者可以直接互换。
    """

    def __init__(self, world: SimWorld, kind: str = "gimbal"):
        self.world = world
        self.kind = kind
        self.name = "sim-" + kind
        self.tx_frames = 0
        self.parser = proto.FrameParser()
        self._seq = 0
        self._last_rx = time.monotonic()

    def start(self) -> bool:
        return True

    def send(self, msg_id: int, payload: bytes = b"", seq: Optional[int] = None) -> bool:
        if seq is None:
            seq = self._seq
            self._seq = (self._seq + 1) & 0xFF
        wire = proto.build_frame(msg_id, payload, seq)
        for frame in self.parser.feed(wire):       # 走一遍对端解码路径
            self._dispatch(frame)
        self.tx_frames += 1
        self._last_rx = time.monotonic()
        return True

    def _dispatch(self, frame: Frame) -> None:
        if self.kind == "gimbal":
            if frame.msg_id == proto.MsgId.AIM:
                self.world.on_aim(frame.payload)
            elif frame.msg_id == proto.MsgId.MODE:
                self.world.on_mode(frame.payload)
        elif frame.msg_id == proto.MsgId.CAR_CMD:
            self.world.on_car_cmd(frame.payload)

    def read_frames(self) -> List[Frame]:
        t = time.monotonic()
        if self.kind == "gimbal":
            self.world.on_gimbal_poll(t)
            src = self.world.gimbal_tx
        else:
            with self.world._lock:
                src = self.world.car_tx
        if not src:
            return []
        blob = b""
        while src:
            blob += src.popleft()
        frames = self.parser.feed(blob)
        if frames:
            self._last_rx = time.monotonic()
        return frames

    def healthy(self) -> bool:
        return True

    def rx_age(self) -> float:
        return time.monotonic() - self._last_rx

    def stats(self) -> str:
        return "sim-%s tx=%d rx=%d" % (self.kind, self.tx_frames,
                                       self.parser.frames_ok)

    def close(self) -> None:
        pass
