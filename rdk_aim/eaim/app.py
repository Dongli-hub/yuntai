"""应用装配：把视觉、控制、链路、状态机、记录、显示全部接起来。

主循环（一帧一次）：

    取最新帧
      -> target.detect()                       找靶心
      -> spot.detect()                          找激光光斑
      -> tracker.update()                       α-β 滤波 + 短时滑行
      -> 处理两条链路的上行帧                   云台状态 / 小车状态 / 拐角事件
      -> 状态机                                 决定本帧"设定点"是什么
      -> aim.update()                           算出 yaw/pitch 偏置
      -> 按 tx_hz 下发 AIM 帧
      -> 记录 / 显示

状态机的每个分支都显式写出进入动作、退出条件、超时与降级，
并且每次转移都打印一行日志 —— 现场"卡在哪一步"一眼可见。
"""

import math
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

from . import protocol as proto
from . import viz
from .aim import AimController, AimOutput
from .camera import Camera
from .config import AppConfig
from .estimator import AlphaBetaTracker, TrackState
from .geometry import CameraModel
from .laser import LaserSpotDetector, SpotResult
from .link import make_link
from .protocol import AimFlags, AimMode, CarCmd, CarEvent, CarState, Frame, GimbalState
from .recorder import Recorder
from .state_machine import State, SweepPlanner
from .target import TargetDetector, TargetResult
from .trajectory import CircleTrajectory

__all__ = ["AimApp", "RunOptions"]


@dataclass
class RunOptions:
    duration_s: float = 0.0          # 0 = 一直跑到 Ctrl-C
    draw: bool = False               # 画圆模式（发挥部分 3）
    laps: int = 1                    # 让小车跑几圈
    auto_start_car: bool = False     # 锁定后自动发"开始行驶"
    laser: bool = True               # 是否使用激光
    budget_s: float = 0.0            # 0 = 用配置里的
    start_corner: int = 0            # 小车起跑时所在的段（0=A..B）
    # 到点后不退出、继续把激光压在靶心上（观察/拍照用；Ctrl-C 才退出）。
    # 为什么需要：默认到点就切 STAB，偏置归零 -> 激光会离开靶心回到上电朝向，
    # 看起来像"打中了又跑掉"。
    keep_aim: bool = False
    stop_at_lap: bool = True         # 跑完设定圈数就结束
    quiet: bool = False


class AimApp:
    def __init__(self, cfg: AppConfig, options: Optional[RunOptions] = None,
                 sim_world=None):
        self.cfg = cfg
        self.opt = options or RunOptions()
        self.sim_world = sim_world
        self.cam_model = CameraModel(fx=cfg.calib.fx, fy=cfg.calib.fy,
                                     cx=cfg.calib.cx, cy=cfg.calib.cy,
                                     dist=cfg.calib.dist)
        self.camera: Optional[Camera] = None
        self.gimbal_link = None
        self.car_link = None
        self.detector = TargetDetector(cfg.target, self.cam_model)
        self.spot_detector = LaserSpotDetector(cfg.laser)
        self.aim = AimController(cfg.control, self.cam_model,
                                 cfg.calib.sign_yaw, cfg.calib.sign_pitch)
        # α-β 跟踪器：α 是"多信新测量"，越小越平滑但越滞后。
        # 实测（9fps + 检测本身有 ±20~40px 抖动）默认 0.45 太"跟"，环路会
        # 追着检测噪声来回摆（表现为"激光一直晃、停不在一个点上"）。
        # 这里降到 0.25；因为云台自身的运动已经被"命令偏置前馈"补偿掉了，
        # 平滑带来的那点滞后不会再拖累跟踪。
        # α 从 0.25 提到 0.8（见 ControlConfig.track_alpha 的说明）：
        # 检测已经很稳，滤波滞后才是环路的主要延迟来源。
        self.tracker = AlphaBetaTracker(
            alpha=float(getattr(cfg.control, "track_alpha", 0.8)),
            beta=float(getattr(cfg.control, "track_beta", 0.05)))
        self.trajectory = CircleTrajectory(cfg.draw, self.cam_model)
        self.sweep = SweepPlanner(cfg.acquire.mode, cfg.acquire.yaw_range_deg,
                                  cfg.acquire.pitch_range_deg,
                                  cfg.acquire.step_deg,
                                  center=(cfg.acquire.center_yaw_deg,
                                          cfg.acquire.center_pitch_deg),
                                  pitch_layers=getattr(cfg.acquire,
                                                       "pitch_layers", None))
        self.rec: Optional[Recorder] = None
        self.state = State.INIT
        self.state_since = 0.0
        self.t0 = 0.0
        self.frames = 0
        self.state_history: List[Tuple[float, str]] = []
        # 链路状态
        self.gimbal: Optional[GimbalState] = None
        self.car: Optional[CarState] = None
        self.corner_count = 0
        self.lap_done_count = 0
        self.car_started = False
        self._car_dispatched = False
        self.last_corner_t = -1e9
        self.link_stale_warned = False
        # 时序
        self._last_tx = 0.0
        self._last_car_tx = 0.0
        self._last_snapshot = 0.0
        self._lock_time: Optional[float] = None
        self._last_target_t = -1e9
        self._acquire_target_frames = 0
        self._fine_pts: list = []
        self._fine_i = 0
        self._last_rect = None
        self._lost_since: Optional[float] = None
        self._prev_state = State.INIT
        self.target: Optional[TargetResult] = None
        self.spot: Optional[SpotResult] = None
        self.track = TrackState()
        self.aim_out: Optional[AimOutput] = None
        self.draw_status = None
        self._last_aim_t = 0.0
        self._sweep_move_t = 0.0
        self._sweep_next_t = 0.0
        self._last_sp: Optional[Tuple[float, float]] = None
        self._sent_mode: Optional[int] = None
        self._laser_gate = False
        self._triggered = False
        self._last_trigger_warn = 0.0
        self._running = False
        self.fps = 0.0
        self._fps_ema = None
        self._last_loop_t = 0.0
        self.detect_count = 0
        self.detect_ok = 0
        self.spot_ok = 0
        self.spot_fallback = 0
        self.gate_reject = 0        # 被"跳变门控"拦掉的误检帧数
        self.detect_scale_fallback = 0   # 缩小图没检到、退回全图才检到的帧数
        # ---- 命令偏置前馈预测（见 _process_vision 里的说明）----
        self._ff_uv = None          # 最近一次实测靶心
        self._ff_off = (0.0, 0.0)   # 那次实测时的云台偏置
        self.pred_uv = None         # 由它外推出来的"本帧靶心应该在哪"
        self._bs_rows = []          # 自动学的"距离 -> 光斑像素"表

    # ------------------------------------------------------------------
    # 装配
    # ------------------------------------------------------------------
    def setup(self) -> bool:
        cfg = self.cfg
        self.t0 = time.monotonic()
        if cfg.camera.source == "sim" and self.sim_world is None:
            # 命令行把 camera.source 设成 sim 时自动建仿真世界，
            # 这样 check / replay / unwind 也都能在 PC 上无硬件跑
            from .sim import SimWorld

            self.sim_world = SimWorld(cfg)
            self.log("相机源为 sim -> 已自动启用仿真世界（无硬件模式）")
        if cfg.calib.boresight_uv:
            self.spot_detector.set_boresight(cfg.calib.boresight_uv)
            self.log("标定光轴点: (%.1f, %.1f)" % cfg.calib.boresight_uv)
        if cfg.calib.boresight_table:
            self.spot_detector.set_boresight_table(cfg.calib.boresight_table)
            self.log("光斑-距离标定表: %d 点（%s）"
                     % (len(cfg.calib.boresight_table),
                        " / ".join("%.2fm" % t[0] for t in cfg.calib.boresight_table)))
        if self.opt.draw and cfg.draw.laser_gate:
            if cfg.calib.boresight_uv:
                self._laser_gate = True
                self.log("画圆激光门控开启：起跑前关激光，避免靶心与径向拖痕超 D2")
            else:
                self.log("画圆需要激光门控，但没标定光轴点（calib.boresight_uv）——"
                         "已自动关闭门控。建议先跑 tools/calib_boresight.py",
                         level="WARN")
        frame_fn = None
        if self.sim_world is not None:
            frame_fn = self.sim_world.render
        self.camera = Camera(cfg.camera, frame_fn=frame_fn)
        if not self.camera.start():
            self.log("相机启动失败: %s" % self.camera.error, level="ERR")
            return False
        if self.sim_world is not None:
            self.gimbal_link = self._make_sim_link("gimbal")
            self.car_link = self._make_sim_link("car")
        else:
            self.gimbal_link = make_link(cfg.link_gimbal, "gimbal")
            self.car_link = make_link(cfg.link_car, "car")
            if not self.gimbal_link.start():
                self.log("云台链路未打开: %s"
                         % getattr(self.gimbal_link, "error", ""), level="WARN")
            if not self.car_link.start():
                self.log("小车链路未打开: %s"
                         % getattr(self.car_link, "error", ""), level="WARN")
            # ---- 启动时下发"在线安全参数" ----
            # 最要紧的是俯仰位置环增益：默认 KP=15/KI=45 会让俯仰轴自激
            # （实测峰峰摆 115），而 IMU 装在俯仰轴下方、遥测里俯仰一直是"稳的"，
            # 只有画面在跳 —— 表现就是"靶纸完整在画面里也检不到、误差收不敛"。
            # 固件默认值虽已改，但这里每次启动都下发一遍，没重烧固件也能用。
            for pid, val in getattr(cfg.link_gimbal, "startup_params", []) or []:
                self.gimbal_link.send(proto.MsgId.PARAM,
                                      proto.pack_param(pid, float(val)))
                self.log("启动参数: 0x%02X = %g" % (pid, val), level="DBG")
        run_dir = cfg.app.run_dir
        if not os.path.isabs(run_dir):
            run_dir = os.path.join(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))), run_dir)
        w, h = self.camera.resolution()
        # 把相机"实际生效"的格式与帧率打出来 —— 这一步能立刻发现
        # "配置写了 MJPG 却跑在 YUYV 上"这种静默降级（帧率会差 6 倍）
        self.log("相机实际生效: %s %dx%d 自报%.0ffps"
                 % (self.camera.actual_fourcc or "?", w, h,
                    self.camera.actual_fps))
        if self.camera.actual_fourcc and \
                cfg.camera.fourcc and \
                self.camera.actual_fourcc.upper() != cfg.camera.fourcc.upper():
            self.log("⚠ 想要的格式是 %s，实际却是 %s —— 帧率会差很多，"
                     "检查驱动是否支持该格式" % (cfg.camera.fourcc,
                                          self.camera.actual_fourcc),
                     level="WARN")
        self.rec = Recorder(run_dir, name="aim", video=cfg.app.record_video,
                            frame_size=(w, h), fps=cfg.camera.fps)
        self.log("配置: %s" % (cfg.source_path or "内置默认值"))
        for wmsg in cfg.warnings:
            self.log("配置告警: %s" % wmsg, level="WARN")
        self.log("相机: %s %dx%d @%dfps  记录: %s"
                 % (cfg.camera.source, w, h, cfg.camera.fps, self.rec.csv_path))
        self.log("扫描计划 %d 个点位，扫完一遍约 %.2fs（dwell=%dms, step=%.0f°, "
                 "过点 %.0f°/s）"
                 % (len(self.sweep),
                    self.sweep.estimate_duration(cfg.acquire.dwell_ms,
                                                 cfg.acquire.move_dps),
                    cfg.acquire.dwell_ms, cfg.acquire.step_deg,
                    cfg.acquire.move_dps))
        self._set_state(State.BOOT_WAIT, time.monotonic())
        # 云台保活线程：即使视觉卡住，AIM 也照发（否则 H723 的 0.5s 看门狗跳闸）
        self._start_tx_thread()
        return True

    def _make_sim_link(self, kind: str):
        from .sim import SimLink

        return SimLink(self.sim_world, kind)

    # ------------------------------------------------------------------
    def log(self, msg: str, level: str = "INFO") -> None:
        if self.opt.quiet and level == "INFO":
            return
        now = time.monotonic()
        rel = (now - self.t0) if self.t0 else 0.0
        print("[%7.3fs][%-4s][%-9s] %s" % (rel, level, self.state.label, msg),
              flush=True)

    def _set_state(self, new: State, now: float, reason: str = "") -> None:
        if new == self.state:
            return
        old = self.state
        self._prev_state = old
        self.state = new
        self.state_since = now
        self.state_history.append((now, new.label))
        extra = (" <- %s" % reason) if reason else ""
        self.log("状态 %s -> %s%s" % (old.label, new.label, extra))
        if new == State.ACQUIRE:
            # 丢靶后从【丢靶前一刻的云台位置】开始左右找，而不是回到上电零位
            # 重新扫一圈（用户实测要求：这样找回来快得多）。
            # 第一次捕获时 aim 的偏置就是 (0,0) = 上电朝向，行为不变。
            self.sweep.set_center(self.aim.yaw_deg, self.aim.pitch_deg)
            self._acquire_target_frames = 0
            # 上一轮扫描留下的"当前目标点"必须清掉：否则重新进入 ACQUIRE 时
            # 会直接沿用上次的点位（还带着 old 的到位时间戳），扫描节奏全乱。
            self._scan_tgt = None
            self._scan_arrived_t = 0.0
            # ---- 两档扫描：先"就近细扫"，再"大范围扫描" ----
            # 刚刚还在跟踪（fine_recent_s 秒内看到过靶纸）-> 说明靶纸就在附近，
            # 大概率只是漏了几帧，先以当前位置为中心 ±16° 慢慢找；
            # 细扫走完还是没有 -> 才交给 ±175° 的大范围扫描。
            # 现场踩过的坑：丢了就甩 60° 大格扫描，靶纸早飞出画面，
            # 表现为"光斑在靶纸宽度内来回匀速摆、永远停不下来"。
            acq = self.cfg.acquire
            recent = (now - self._last_target_t) < acq.fine_recent_s
            if recent and acq.fine_span_deg > 0.5:
                self._fine_pts = self.sweep.fine_points(
                    (self.aim.yaw_deg, self.aim.pitch_deg),
                    acq.fine_span_deg, acq.fine_step_deg, acq.fine_pitch_deg)
                self._fine_i = 0
                self.log("就近细扫：以当前位姿为中心 ±%.0f°，%d 个点位"
                         % (acq.fine_span_deg, len(self._fine_pts)), level="DBG")
            else:
                self._fine_pts = []
                self._fine_i = 0
        if new == State.LOST:
            self._lost_since = now
        if new == State.FAULT:
            self._safe_mode()

    def _safe_mode(self) -> None:
        """进入安全模式：云台只做稳定、关激光。"""
        if self.gimbal_link is not None:
            self.gimbal_link.send(proto.MsgId.MODE, proto.pack_mode(AimMode.STAB))
            self.gimbal_link.send(proto.MsgId.AIM,
                                  proto.pack_aim(0.0, 0.0, 0, 0))

    def shutdown(self) -> None:
        self._running = False
        # 先把保活线程停掉，免得它在关闭链路之后又去写串口
        self._tx_stop = getattr(self, "_tx_stop", None)
        if self._tx_stop is not None:
            self._tx_stop.set()
        th = getattr(self, "_tx_thread", None)
        if th is not None:
            try:
                th.join(timeout=0.5)
            except Exception:
                pass
        try:
            self.log("退出：切换到 STAB 并关激光（云台仍保持稳定）")
            self._safe_mode()
            if self.car_link is not None:
                self.car_link.send(proto.MsgId.CAR_CMD,
                                   proto.pack_car_cmd(CarCmd.STOP))
        except Exception:
            pass
        for link in (self.gimbal_link, self.car_link):
            try:
                if link is not None:
                    link.close()
            except Exception:
                pass
        if self.camera is not None:
            self.camera.stop()
        if self.rec is not None:
            self.rec.close()
        summary = self.rec.summary() if self.rec else ""
        self.log("视觉统计: 帧=%d 检出=%d(%.0f%%) 光斑=%d 光轴兜底=%d"
                 % (self.detect_count, self.detect_ok,
                    100.0 * self.detect_ok / max(1, self.detect_count),
                    self.spot_ok, self.spot_fallback))
        if summary:
            self.log("误差统计: %s" % summary)
        if self._lock_time is not None and self.t0:
            self.log("首次锁定用时: %.3fs（预算 %.1fs）"
                     % (self._lock_time - self.t0, self.opt.budget_s or
                        self.cfg.app.budget_s))
        if self.state_history:
            self.log("状态轨迹: " + " -> ".join(s for _, s in self.state_history))
        for link in (self.gimbal_link, self.car_link):
            if link is not None:
                self.log("链路统计: %s" % link.stats())
        if self.rec is not None:
            self.log("记录文件: %s" % self.rec.csv_path)

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------
    def run(self) -> int:
        if not self.setup():
            self.shutdown()
            return 1
        budget = self.opt.budget_s or self.cfg.app.budget_s
        self.opt.budget_s = budget
        self.log("计时开始（预算 %.1fs%s）"
                 % (budget, "，含 H723 启动时间" if self.cfg.app.timer_start ==
                    "poweron" else "，等待触发"))
        self._running = True
        loop_period = 1.0 / max(30.0, min(240.0, self.cfg.camera.fps * 2.0))
        deadline = (self.t0 + self.opt.duration_s) if self.opt.duration_s > 0 else None
        held = False
        # SIGTERM（kill / timeout 命令）默认直接杀进程、跳过所有收尾 —— 那样
        # H723 会一直保持在最后一个偏置上，下一次锁零就把零点记歪（实测踩过）。
        # 这里把它变成优雅退出：循环退出后的 finally 会切回 STAB。
        try:
            import signal

            def _on_term(_sig, _frm):
                self.log("收到终止信号，切回 STAB 后退出", level="WARN")
                self._running = False

            signal.signal(signal.SIGTERM, _on_term)
        except Exception:                                      # noqa: BLE001
            pass
        try:
            while self._running:
                t_loop = time.monotonic()
                if deadline is not None and t_loop >= deadline:
                    if not held:
                        held = True
                        self.log("到达设定运行时长（统计口径到此为止）——"
                                 "继续实时瞄准，不打完不撒手；Ctrl-C 才退出")
                frame, _ = self.camera.read()
                new_frame = frame is not None
                if frame is not None:
                    self._process_vision(frame.image, frame.t)
                    self.frames += 1
                    dt = frame.t - self._last_loop_t if self._last_loop_t else 0.0
                    self._last_loop_t = frame.t
                    if dt > 1e-6:
                        inst = 1.0 / dt
                        self._fps_ema = inst if self._fps_ema is None \
                            else 0.9 * self._fps_ema + 0.1 * inst
                        self.fps = self._fps_ema
                now = time.monotonic()
                self._tick_links(now)
                self._update_state(now, new_frame)
                self._send(now)
                if frame is not None:
                    self._record(frame.image, now)
                    if self.cfg.app.show_window or self._snapshot_due(now):
                        self._render(frame.image)
                slack = (t_loop + loop_period) - time.monotonic()
                if slack > 0:
                    time.sleep(slack)
        except KeyboardInterrupt:
            self.log("收到 Ctrl-C")
        finally:
            self.shutdown()
        return 0

    # ------------------------------------------------------------------
    def _process_vision(self, image: np.ndarray, t: float) -> None:
        cfg = self.cfg
        self.detect_count += 1
        # 检测在缩小图上做（地瓜派 CPU 是瓶颈：全分辨率 111ms/帧 = 9fps，
        # 缩到 0.5 后约 30ms/帧 = 30fps；瞄准精度不受影响，见 target.detect 注释）
        scale = cfg.camera.process_scale
        # ---- 光斑先检 ----
        # 两个原因：① 靶纸检测要用光斑位置去"补"黑胶带上被打亮的洞
        # （否则靶心落在 u≈500 这种位姿时，激光正好打在胶带上，
        #   胶带环断开 -> 连续丢靶 -> 云台锁一下丢一下地来回摆）；
        # ② 光斑本身走的是 ROI，很快，先检不会拖慢整体。
        self.spot = None
        if self._laser_on():
            self.spot = self.spot_detector.detect(image, scale)
        if self.spot is not None:
            self.spot_ok += 1
        spot_arg = None
        if self.spot is not None and self.spot.area > 0:
            spot_arg = (self.spot.uv[0], self.spot.uv[1],
                        math.sqrt(self.spot.area / math.pi))
        # ---- 跟踪快通道：锁定/跟踪时只在靶心附近找 ----
        # 全图 1280x720 的二值化+找轮廓是单帧耗时的大头（实测 110~210ms，
        # 也就是 5~9fps，视觉环的延时主要来自这里）。锁定之后靶心位置已知，
        # 裁一块以它为中心的正方形就够，实测能省 60% 以上时间。
        # ROI 里没检到会自动退回全图，所以不会因此丢靶。
        roi = None
        if (self.state in (State.LOCK, State.TRACK, State.DRAW)
                and getattr(self.tracker, "initialized", False)):
            try:
                tu, tv = float(self.tracker.pos[0]), float(self.tracker.pos[1])
                rr = float(getattr(cfg.target, "roi_track_px", 260.0))
                # rr <= 40 视为"关掉快通道"（方便现场 A/B 对比）
                if rr > 40.0 and 0.0 <= tu < image.shape[1] \
                        and 0.0 <= tv < image.shape[0]:
                    roi = (tu, tv, rr)
            except Exception:                                  # noqa: BLE001
                roi = None
        self.target = self.detector.detect(image, scale, spot=spot_arg, roi=roi)
        # ---- 降分辨率提速的保险 ----
        # 现场实测（2026-09-30）：process_scale=0.65 时单帧从 ~100ms 降到 ~60ms，
        # 近距离 8/8 全检出；但靶纸变远（>0.9m）时胶带只剩几个像素，缩小图可能
        # 检不到。所以"缩小图没检到 -> 全图再找一遍"，用一点点时间换不漏靶。
        if (self.target is None and scale < 0.999
                and self.state in (State.LOCK, State.TRACK, State.DRAW)):
            self.target = self.detector.detect(image, 1.0, spot=spot_arg)
            if self.target is not None:
                self.detect_scale_fallback += 1
        if self.target is not None and \
                self.target.confidence < cfg.target.min_confidence:
            self.target = None            # 置信度不够就当没看到
        if self.target is not None:
            # ---- 跳变门控（只在"已经在跟踪"时生效）----
            # 云台跟踪阶段不可能一瞬间把靶心挪几百像素，出现这种跳变一定是
            # 误检（别的四边形/反光被当成靶纸）——实测会把环路一把拽走，
            # 表现就是"俯仰突然上下摆动一下又回来"。这里直接丢掉这一帧。
            # 连续丢失超过 1s 后门控失效，避免真的换了靶纸后永远锁不上。
            #
            # ⚠⚠ 2026-09-27 晚 现场抓到的致命 bug：门控【必须扣掉云台自己的运动】。
            #   原来拿"上一次检出的像素位置"当基准，可云台在这中间是按我们自己的
            #   指令转的：靶纸在画面里移动 266px 是【指令造成的】，
            #   却被判成"跳变误检"丢掉 -> 连续 1 秒没结果 -> 判丢靶 -> 重新扫描。
            #   实测日志：60° 处锁定 uv=(417,384)，伺服把偏置往 +7° 拉，
            #   靶心跟着走到 ~683px（>120px 门限），于是整整 1.05s 全被门控吃掉。
            #   现在改成：先按"命令偏置变化 × px/°"预测靶心该在哪，再和预测比。
            last = getattr(self, "_gate_uv", None)
            last_t = getattr(self, "_gate_t", 0.0)
            off_now = (self.aim.yaw_deg, self.aim.pitch_deg)
            gate_off = getattr(self, "_gate_off", off_now)
            pred_u = last[0] + (off_now[0] - gate_off[0]) * self.aim.px_per_deg_u \
                if last is not None else 0.0
            pred_v = last[1] + (off_now[1] - gate_off[1]) * self.aim.px_per_deg_v \
                if last is not None else 0.0
            if (last is not None and self.state in (State.LOCK, State.TRACK,
                                                    State.DRAW)
                    and (t - last_t) < 1.0 and
                    math.hypot(self.target.uv[0] - pred_u,
                               self.target.uv[1] - pred_v) > cfg.target.jump_gate_px):
                self.gate_reject += 1
                self.target = None
            else:
                self._gate_uv = self.target.uv
                self._gate_t = t
                self._gate_off = off_now
                self.detect_ok += 1
                self._last_target_t = t
                if self.target.rect is not None:
                    self._last_rect = self.target.rect
        # ---- ① 命令偏置前馈预测：把 ~9fps 的观测"连续化" ----
        # 相机只有 9fps、云台还有延迟，靠"像素速度外推"很不准（实测表现就是
        # 飘忽）。但"云台转了多少"是我们自己下发的命令（已知、无噪声），所以：
        #     靶心预测位置 = 上次实测位置 + (当前偏置 - 那次实测时的偏置) × px/°
        # 掉帧时用这个预测顶上（_state_aiming 里用），比速度外推稳得多，
        # 而且它天然补偿了云台自身的运动（相当于把已知量从观测里扣掉）。
        # 用【实测姿态】而不是【命令偏置】推算靶心该在哪：
        #   命令偏置发下去之后，云台要过 0.3~1s 才真的转到位。拿命令去外推，
        #   等于提前把"还没发生的转动"算进去 —— 环路以为已经到了，
        #   于是继续积偏置，等平台真转过来就冲过头，形成左右摆。
        #   改用遥测里的实际姿态（gz_yaw/gz_pitch）后，外推的是真发生过的转动，
        #   环路就不会跟自己较劲。没有遥测时自动退回原来的命令外推。
        att_now = None
        gz = self.gimbal
        if gz is not None and bool(getattr(gz, "ready", lambda: False)()):
            att_now = (float(gz.yaw_deg), float(gz.pitch_deg))
        lead_src = att_now if att_now is not None else \
            (self.aim.yaw_deg, self.aim.pitch_deg)
        if self.target is not None:
            self._ff_uv = (float(self.target.uv[0]), float(self.target.uv[1]))
            self._ff_off = lead_src
        self.pred_uv = None
        if self._ff_uv is not None:
            self.pred_uv = (
                self._ff_uv[0] + (lead_src[0] - self._ff_off[0]) * self.aim.px_per_deg_u,
                self._ff_uv[1] + (lead_src[1] - self._ff_off[1]) * self.aim.px_per_deg_v)

        # ---- ② 自动学"距离 -> 光斑像素位置"表 ----
        # 用户要求：先用外框面积估距离，再用距离把激光点映射到画面里的位置。
        # 这里不需要手工标定：光斑检到时顺手把 (距离, 光斑像素) 记下来，
        # 按距离分档慢速更新；光斑检不到时 _spot_uv() 就用这张表插值兜底。
        if (self.spot is not None and self.target is not None
                and self.target.distance_m > 0.2):
            self._learn_boresight(self.target.distance_m, self.spot.uv)

        # 跟踪滤波
        uv = self.target.uv if self.target is not None else None
        self.track = self.tracker.update(uv, t,
                                         coast_s=cfg.control.lost_coast_s)
        # 画圆的设定点需要单应矩阵，重捕获后自动更新
        if self._last_rect is not None:
            self.trajectory.set_target(self._last_rect)

    def _spot_uv(self) -> Optional[Tuple[float, float]]:
        return self._spot_uv_impl()

    def _att_rel(self) -> Optional[Tuple[float, float]]:
        """云台【实测姿态】相对锁零基准的变化量（度）。

        锁零那一刻（SET_ZERO）的姿态就是"偏置 0"，把它记下来；
        之后 gz_yaw - 基准 = 平台真正转过去的角度 —— 用它才能知道
        "平台到底跟没跟上指令"。没遥测/没锁零时返回 None（调用方自动跳过）。
        """
        gz = self.gimbal
        if gz is None:
            return None
        try:
            if not gz.ready():
                return None
        except Exception:                                      # noqa: BLE001
            return None
        base = getattr(self, "_att_zero", None)
        if base is None:
            return None
        return (float(gz.yaw_deg) - base[0], float(gz.pitch_deg) - base[1])

    def _learn_boresight(self, dist_m: float, uv) -> None:
        """把 (距离, 光斑像素) 记进表里：光斑检不到时按距离插值兜底。

        为什么按距离分档：激光和相机不同轴，光斑在画面里的位置是距离的函数
        （角度 ≈ 两轴夹角 + 平移/距离），固定一个像素点只在标定距离上准。
        这里不手工标定——只要光斑能检到就顺手更新，2cm 内算同一档，
        每档慢速收敛（0.25 的步长，抗偶发误检）。
        """
        rows = self._bs_rows
        for r in rows:
            if abs(r[0] - dist_m) < 0.02:
                r[1] += 0.25 * (float(uv[0]) - r[1])
                r[2] += 0.25 * (float(uv[1]) - r[2])
                break
        else:
            rows.append([float(dist_m), float(uv[0]), float(uv[1])])
            rows.sort(key=lambda r: r[0])
            del rows[12:]                    # 最多留 12 档，够覆盖 0.4~2m
        try:
            self.spot_detector.set_boresight_table(rows)
        except Exception:                                      # noqa: BLE001
            pass

    def _spot_uv_impl(self) -> Optional[Tuple[float, float]]:
        if self.spot is not None:
            return self.spot.uv
        # 光斑没检到时的兜底顺序：
        #   1) 有"光斑位置-距离"标定表 + 本帧知道靶纸距离 -> 按距离插值
        #      （视差是随距离变的，用一条曲线描述它比用一个固定点准得多）
        #   2) 退到单一光轴点（老行为）
        dist = None
        if self.target is not None and self.target.distance_m > 0.05:
            dist = self.target.distance_m
        uv = self.spot_detector.boresight_for_distance(dist)
        if uv is not None:
            self.spot_fallback += 1
            return uv
        return None

    def _laser_on(self) -> bool:
        # 激光物理常亮（直接接电源）时：光斑一直在，永远用实测光斑，
        # 不退回"光轴点兜底"那条路径 —— 精度更好。
        if self.cfg.laser.always_on:
            return True
        if not self.opt.laser:
            return False
        if self._laser_gate and not self.car_started:
            return False
        # 画圆模式的两个"关激光"窗口，都是为了不在靶面上留下多余拖痕：
        #   1) 起跑前先把光斑挪到圆的起点 —— 此时若开着激光，
        #      会在靶面上画出"从圆心到圆周"的一条径向拖痕，直接超 D2
        #   2) 跑完最后一圈后 —— 继续照只是把起点反复加深
        if self.state == State.DONE and self.opt.draw:
            return False
        if self.state == State.DRAW and not self.car_started:
            return False
        return self.state >= State.ACQUIRE and self.state not in (
            State.FAULT, State.ESTOP, State.UNWIND)

    # ------------------------------------------------------------------
    def _tick_links(self, now: float) -> None:
        # 链路健康检查：H723 遥测断了要立刻看得见，而不是傻等
        if self.state >= State.ACQUIRE and not self.sim_world:
            age = self.gimbal_link.rx_age()
            if age > self.cfg.link_gimbal.telemetry_timeout_s:
                if not self.link_stale_warned:
                    self.link_stale_warned = True
                    self.log("云台遥测已断 %.2fs —— 查接线/波特率/共地。"
                             "H723 侧 0.5s 收不到 AIM 会自动切 STAB 并关激光"
                             % age, level="WARN")
            elif self.link_stale_warned:
                self.link_stale_warned = False
                self.log("云台遥测恢复")
        for frame in self.gimbal_link.read_frames():
            if frame.msg_id == proto.MsgId.GIMBAL_STATE:
                try:
                    self.gimbal = proto.unpack_gimbal_state(frame.payload)
                except Exception as exc:
                    self.log("GIMBAL_STATE 解析失败: %s" % exc, level="WARN")
            elif frame.msg_id == proto.MsgId.ACK:
                info = proto.unpack_ack(frame.payload)
                if info["code"] != 0:
                    self.log("云台 NACK %s code=%d"
                             % (proto.MsgId.name(info["msg_id"]), info["code"]),
                             level="WARN")
            elif frame.msg_id == proto.MsgId.VERSION:
                self.log("云台版本: %s" % frame.payload.decode("ascii", "replace"))
            elif frame.msg_id == proto.MsgId.TEXT:
                # H723 的调试文本（原来的串口 printf 现在走协议，不会冲乱帧）
                self.log("H723 %s" % frame.payload.decode("utf-8", "replace"))
        for frame in self.car_link.read_frames():
            self._on_car_frame(frame, now)

    def _on_car_frame(self, frame: Frame, now: float) -> None:
        if frame.msg_id == proto.MsgId.CAR_STATE:
            try:
                self.car = proto.unpack_car_state(frame.payload)
                if self.car.lap_time_ms > 2000:
                    self.trajectory.set_lap_time(self.car.lap_time_ms / 1000.0)
            except Exception as exc:
                self.log("CAR_STATE 解析失败: %s" % exc, level="WARN")
        elif frame.msg_id == proto.MsgId.CAR_EVENT:
            ev = proto.unpack_car_event(frame.payload)
            if ev.event == CarEvent.CORNER:
                self.corner_count += 1
                self.last_corner_t = now
                self.aim.note_corner(now)
                self.trajectory.on_corner(now, ev.index)
                self.log("拐角 %d（累计 %d）-> 增强瞄准 %.0fms"
                         % (ev.index, self.corner_count,
                            self.cfg.control.corner_boost_s * 1000), level="DBG")
                if self.opt.draw and self.state in (State.TRACK, State.LOCK):
                    self._set_state(State.DRAW, now, "拐角同步")
                if self.car is not None:
                    self.trajectory.set_car_progress(self.car.lap_index,
                                                     self.car.segment,
                                                     self.car.progress, now)
            elif ev.event == CarEvent.LAP_DONE:
                self.lap_done_count += 1
                lap_time = (self.car.lap_time_ms / 1000.0) if self.car else None
                self.trajectory.on_lap(now, lap_time)
                self.log("完成第 %d 圈（目标 %d 圈）"
                         % (self.lap_done_count, self.opt.laps))
                if self.opt.stop_at_lap and self.lap_done_count >= self.opt.laps:
                    self._set_state(State.DONE, now, "圈数达到")
            elif ev.event == CarEvent.STARTED:
                self.car_started = True
                self.log("小车开始行驶（段 %d）" % ev.index)
                self.trajectory.seed_corner(ev.index)
                if self.state in (State.LOCK,):
                    self._set_state(State.DRAW if self.opt.draw else State.TRACK,
                                    now, "小车启动")
            elif ev.event == CarEvent.STOPPED:
                self.car_started = False
                self.log("小车停止")

    # ------------------------------------------------------------------
    # 状态机
    # ------------------------------------------------------------------
    def _desired_mode(self) -> int:
        st = self.state
        if st in (State.INIT,):
            return AimMode.IDLE
        if st == State.UNWIND:
            return AimMode.UNWIND
        if st in (State.BOOT_WAIT, State.FAULT, State.ESTOP):
            return AimMode.STAB
        return AimMode.AIM

    def _return_state(self) -> State:
        if self.opt.draw:
            return State.DRAW if self.car_started else State.LOCK
        return State.TRACK if self.car_started else State.LOCK

    def _poll_trigger(self, now: float) -> bool:
        """等外部触发：有终端就等回车，否则等 trigger_file 出现。

        为什么要这个：H723 上电自检要 3.4s，而基本要求(2) 是"2s 内命中"。
        先把瞄准模块上电、等云台变硬（GIMBAL_STATE 就绪），再触发计时，
        才是符合题意的口径（题目只要求两个电源开关独立，没规定合闸即计时）。
        见 docs/03 风险 R1。
        """
        path = self.cfg.app.trigger_file
        if path and os.path.exists(path):
            try:
                os.remove(path)
            except OSError:
                pass
            return True
        if sys.stdin is not None and sys.stdin.isatty():
            try:
                import select

                ready, _, _ = select.select([sys.stdin], [], [], 0)
                if ready:
                    sys.stdin.readline()
                    return True
            except Exception:
                pass
        if now - self._last_trigger_warn > 3.0:
            self._last_trigger_warn = now
            self.log("云台已就绪，等待触发：按回车，或 touch %s" % path)
        return False

    def _update_state(self, now: float, new_frame: bool = True) -> None:
        st = self.state
        if st == State.BOOT_WAIT:
            if self.gimbal is None:
                if now - self.state_since > 10.0:
                    self._set_state(State.FAULT, now, "收不到云台状态帧")
                return
            if not self.gimbal.ready():
                return
            if self.cfg.app.timer_start == "trigger" and not self._triggered:
                if self._poll_trigger(now):
                    self._triggered = True
                    self.t0 = now
                    self.log("收到触发，开始计时（预算 %.1fs）" % self.opt.budget_s)
                return
            # ---- 重新锁零之前，先把云台送回它自己的基准位 ----
            # SET_ZERO 的含义是"把当前电机角记成新的零点"。如果上一次运行
            # 是被 Ctrl-C / kill 掉的（没走到收尾，H723 还保持在某个扫描偏置上），
            # 直接 SET_ZERO 就会把零点记到那个偏置上：之后"偏置 0"指向别处。
            # 实测踩过：俯仰基准被挪了 36.7°（正好一个扫描档位），云台一进
            # STAB 就对着天花板，而且怎么调都找不回来（只能重启 H723）。
            # 所以先发 STAB + 偏置 0，等它真的转回旧基准，再锁零。
            t_pre = getattr(self, "_pre_zero_t", 0.0)
            if t_pre == 0.0:
                self._pre_zero_t = now
                self.log("锁零前先回基准位（STAB + 偏置清零，等 1.2s）")
            self.gimbal_link.send(proto.MsgId.MODE, proto.pack_mode(AimMode.STAB))
            self.gimbal_link.send(proto.MsgId.AIM,
                                  proto.pack_aim(0.0, 0.0, 0, 0))
            # 等 1 秒让云台回到基准位即可 —— 陀螺零偏的标定现在由 H723
            # 自己在 LOCK 阶段（电机已使能、轴被按住）完成，比这里等更准，
            # 而且上位机本来就是等 READY 位才走到这一步的。
            if now - t_pre < 1.0:
                return
            self.aim.reset(0.0, 0.0)
            self.trajectory.reset()
            self.tracker.reset()
            # 锁零这一刻的姿态 = 偏置 0 的基准，记下来给"防超前"限幅用
            gz0 = self.gimbal
            if gz0 is not None:
                try:
                    self._att_zero = (float(gz0.yaw_deg), float(gz0.pitch_deg))
                except Exception:                              # noqa: BLE001
                    self._att_zero = None
            self.gimbal_link.send(proto.MsgId.SET_ZERO, b"")
            self._set_state(State.ACQUIRE, now,
                            "云台就绪 state=%d" % self.gimbal.state)
            return
        if st == State.ACQUIRE:
            self._state_acquire(now, new_frame)
            return
        if st == State.LOST:
            self._state_lost(now)
            return
        if st in (State.LOCK, State.TRACK, State.DRAW, State.DONE):
            self._state_aiming(now, new_frame)
            return
        if st == State.UNWIND:
            if now - self.state_since > 8.0:
                self._set_state(State.ACQUIRE, now, "解绕结束")
            return

    def _state_acquire(self, now: float, new_frame: bool = True) -> None:
        acq = self.cfg.acquire
        # timeout_s <= 0 表示"一直找，不放弃"（台架定点测试用）。
        # 默认 12s 是为了比赛：找不到就进 FAULT 停住，别让云台一直转。
        if acq.timeout_s > 0 and now - self.state_since > acq.timeout_s:
            self._set_state(State.FAULT, now,
                            "扫描 %.1fs 未找到靶纸" % acq.timeout_s)
            return
        if not new_frame:
            return

        # ---- 1) 朝当前扫描点【缓慢】移动 ----
        # 为什么限速：以前是直接把偏置一步设过去，轴以最快速度甩过去
        # （日志里 "P p=109 t=172 o=120"：差 63° 还在追），拍到的全是运动模糊帧，
        # 结果"扫一圈什么都找不到"。现在按 scan_dps 慢慢挪，挪到位再判。
        tgt = getattr(self, "_scan_tgt", None)
        if tgt is None:
            # ⚠ 扫描点位要夹到偏置限幅之内（yaw ±170 / pitch ±60）。
            #   SweepPlanner 会给出 ±175 这种点，而 AimController.set_offset()
            #   会把它夹回 170 —— 到位判定永远不满足，扫描就卡在那一格不动
            #   （现场表现："扫了两下就停住、再也不动了"）。实测踩过。
            sy, sp = self._next_scan_point()
            ctl = self.cfg.control
            # 单向扫一圈时偏置会走到 360°，扫描限幅要单独放宽（跟踪限幅不变）
            ylim = float(getattr(ctl, "max_yaw_scan_deg", 400.0)) \
                if getattr(self.sweep, "one_way", False) else ctl.max_yaw_deg
            tgt = (max(-ylim, min(ylim, sy)),
                   max(-ctl.max_pitch_deg, min(ctl.max_pitch_deg, sp)))
            self._scan_tgt = tgt
            self._scan_arrived_t = 0.0
            if getattr(self, "_scan_from_fine", False):
                pass                     # 细扫的点已经在 _next_scan_point 里打过日志
            elif self.sweep.index <= 3 or self.sweep.index % 5 == 1:
                self.log("扫描 %d/%d -> (%.0f°, %.0f°)"
                         % (self.sweep.index, len(self.sweep), tgt[0], tgt[1]),
                         level="DBG")
        dt = max(1e-3, now - getattr(self, "_scan_step_t", now))
        self._scan_step_t = now
        fine = bool(self._fine_pts) and self._fine_i <= len(self._fine_pts)
        dps = acq.fine_scan_dps if fine else acq.scan_dps
        step = max(5.0, dps) * dt
        cy, cp = self.aim.yaw_deg, self.aim.pitch_deg
        dy = max(-step, min(step, tgt[0] - cy))
        # ⚠ 俯仰单独限速（很慢）：用户实测反馈"俯仰电机寻找时上下摆动太快"。
        #   俯仰层只在换层时动一次，慢一点既省时间也不会把画面甩糊。
        dstep = max(4.0, float(getattr(acq, "pitch_scan_dps", 25.0))) * dt
        dp = max(-dstep, min(dstep, tgt[1] - cp))
        self.aim.set_offset(cy + dy, cp + dp,
                            yaw_limit_deg=float(getattr(
                                self.cfg.control, "max_yaw_scan_deg", 400.0)))
        if abs(tgt[0] - (cy + dy)) > 1e-3 or abs(tgt[1] - (cp + dp)) > 1e-3:
            # 还在路上：不看检测结果（模糊帧不算数）
            self._scan_arrived_t = 0.0
            self._acquire_target_frames = 0
            return
        if getattr(self, "_scan_arrived_t", 0.0) == 0.0:
            self._scan_arrived_t = now        # 刚刚到位
        # ---- 光"命令到位"还不够，必须等【平台真的停稳】再取帧 ----
        # 现场逐帧数据（2026-09-30）：扫描点时命令是瞬间到的（200°/s），
        # 可平台还在以几百°/s 追过来 —— 判定却在命令到位后立刻开始，
        # 拍到的帧全是运动模糊，靶纸一掠而过（日志里 44.4s 就是这样：
        # 检出时平台还在从 -120° 往回走，0.1s 后靶纸就飞出画面 -> 判丢靶）。
        # 做法：用遥测里的实际姿态算角速度，还在动就重新计时。
        gz = self.gimbal
        if gz is not None:
            try:
                att = (float(gz.yaw_deg), float(gz.pitch_deg))
                prev_att = getattr(self, "_scan_att", None)
                self._scan_att = att
                if prev_att is not None:
                    dt_a = max(1e-3, now - getattr(self, "_scan_att_t", now))
                    rate = max(abs(att[0] - prev_att[0]),
                               abs(att[1] - prev_att[1])) / dt_a
                    if rate > float(getattr(acq, "settle_rate_dps", 8.0)):
                        self._scan_arrived_t = 0.0     # 还在飞 -> 重新计时
                        return
                self._scan_att_t = now
            except Exception:                                  # noqa: BLE001
                pass
        settle_ms = acq.fine_settle_ms if fine else acq.settle_ms
        dwell_ms = acq.fine_dwell_ms if fine else acq.dwell_ms
        if (now - self._scan_arrived_t) * 1000.0 < settle_ms:
            return                            # 再等它停稳（避开伺服余振）

        # ---- 2) 停稳了：只要连续 confirm_frames 帧看到靶纸就开始打 ----
        if self.target is not None:
            self._acquire_target_frames += 1
        else:
            self._acquire_target_frames = 0
        if self._acquire_target_frames >= max(1, acq.confirm_frames):
            self.aim.clear_lock()
            self._set_state(State.LOCK, now,
                            "检出靶纸 conf=%.2f %s"
                            % (self.target.confidence, self.target.describe()))
            return

        # ---- 3) 这个点看够了，换下一个 ----
        # 注意：判定窗口必须 >= settle + confirm_frames 个帧周期，否则会出现
        # "还没攒够连续 2 帧就换点"-> 什么都检不到（提速后实测踩过：
        # dwell 120ms < 2 帧(9fps≈220ms)，第一点明明有靶纸却直接跳走了）。
        judge_ms = max(dwell_ms,
                       120.0 + 160.0 * max(1, acq.confirm_frames))
        if (now - self._scan_arrived_t) * 1000.0 >= settle_ms + judge_ms:
            self._scan_tgt = None

    def _next_scan_point(self):
        """扫描取点：先把"就近细扫"的点走完，再交给大范围扫描规划器。"""
        if self._fine_pts and self._fine_i < len(self._fine_pts):
            pt = self._fine_pts[self._fine_i]
            self._fine_i += 1
            self._scan_from_fine = True
            if self._fine_i == 1 or self._fine_i % 4 == 0:
                self.log("细扫 %d/%d -> (%.0f°, %.0f°)"
                         % (self._fine_i, len(self._fine_pts), pt[0], pt[1]),
                         level="DBG")
            return pt
        if self._fine_pts:
            # 细扫走完还没看到靶纸 -> 放开大范围扫描（只提示一次）
            self._fine_pts = []
            self._fine_i = 0
            self.log("就近细扫未找到，转入大范围扫描", level="DBG")
        self._scan_from_fine = False
        return self.sweep.next_point()

    def _state_lost(self, now: float) -> None:
        age = now - self._last_target_t
        if self.target is not None and self.track.valid:
            self._set_state(self._return_state(), now, "重新捕获")
            return
        if age > self.cfg.control.lost_research_s:
            self._set_state(State.ACQUIRE, now, "丢失 %.2fs，重新扫描" % age)

    def _state_aiming(self, now: float, new_frame: bool = True) -> None:
        cfg = self.cfg
        if self.target is None and (now - self._last_target_t) > cfg.control.lost_coast_s:
            self._set_state(State.LOST, now,
                            "目标丢失 %.2fs" % (now - self._last_target_t))
            return
        # 关键：一帧只允许积分一次。控制环跑得比相机快时若不判新帧，
        # 同一份误差会被重复累加，等效增益翻几倍 -> 必然振荡
        if not new_frame:
            return
        dt = now - getattr(self, "_last_aim_t", now)
        self._last_aim_t = now
        setpoint = None
        velocity = None
        if self.track.valid:
            setpoint = self.track.uv
            velocity = self.track.vel
        elif self.pred_uv is not None:
            # 本帧没测到（或刚过滑行期）：用"命令偏置前馈"的预测顶上。
            # 比 tracker 的像素速度外推稳得多 —— 命令是我们自己下的、无噪声。
            setpoint = self.pred_uv
        if self.state == State.DRAW:
            status = self.trajectory.update(now)
            self.draw_status = status
            if not self.car_started and getattr(self, "_car_start_at", None) is not None:
                out_now = getattr(self, "aim_out", None)
                near = bool(out_now is not None and out_now.err_norm <=
                            self.cfg.draw.start_tol_px)
                if near or now >= self._car_start_at:
                    self.log("画圆起点就位（误差 %.1fpx，等待 %.2fs）"
                             % (out_now.err_norm if out_now else -1.0,
                                now - (self._car_start_at - self.cfg.draw.prestart_s)))
                    self._send_car_start(now)
            if status.setpoint_uv is not None:
                if getattr(self, "_last_sp", None) is not None and dt > 1e-4:
                    velocity = ((status.setpoint_uv[0] - self._last_sp[0]) / dt,
                                (status.setpoint_uv[1] - self._last_sp[1]) / dt)
                setpoint = status.setpoint_uv
                self._last_sp = status.setpoint_uv
            elif not self.track.valid:
                setpoint = None
        out = self.aim.update(now, dt, setpoint if self.track.valid or
                              self.state == State.DRAW else None,
                              velocity, self._spot_uv(),
                              att_rel=self._att_rel())
        self.aim_out = out
        if out.locked and self._lock_time is None:
            self._lock_time = now
            self.log("★ 锁定靶心：用时 %.3fs，误差 %.2fpx"
                     % (now - self.t0, out.err_norm))
        if self.opt.auto_start_car and out.locked and not self._car_dispatched \
                and self.state == State.LOCK:
            self._start_car(now)

    # ------------------------------------------------------------------
    def _start_car(self, now: float) -> None:
        self._car_dispatched = True
        if self.opt.draw:
            # 画圆模式：先把设定点放到圆的起点并等光斑就位，再发车。
            # 否则起步瞬间光斑还在圆心（D2 会先冲出去一段）。
            self.trajectory.seed_corner(self.opt.start_corner)
            self._set_state(State.DRAW, now, "启动画圆（等光斑就位）")
            self._car_start_at = now + max(0.0, self.cfg.draw.prestart_s)
            self.log("画圆准备：%.2fs 后发车" % self.cfg.draw.prestart_s)
            return
        self._send_car_start(now)

    def _send_car_start(self, now: float) -> None:
        self.car_link.send(proto.MsgId.CAR_CMD,
                           proto.pack_car_cmd(CarCmd.SET_LAPS, self.opt.laps))
        self.car_link.send(proto.MsgId.CAR_CMD,
                           proto.pack_car_cmd(CarCmd.START, self.opt.start_corner))
        self.trajectory.seed_corner(self.opt.start_corner)
        self.car_started = True
        self.log("已下发启动：%d 圈，起点段 %d" % (self.opt.laps, self.opt.start_corner))
        self._set_state(State.DRAW if self.opt.draw else State.TRACK, now, "启动小车")

    def start_car_now(self) -> None:
        """供外部（按键/定时）调用。"""
        self._start_car(time.monotonic())

    # ------------------------------------------------------------------
    def _send(self, now: float) -> None:
        """主循环里的发送：只负责小车链路。

        云台（AIM/MODE）改由独立线程按固定频率发 —— 见 _start_tx_thread()。
        """
        if self.car_link is None:
            return
        out = getattr(self, "aim_out", None)
        car_hz = max(1.0, self.cfg.link_car.tx_hz)
        if now - self._last_car_tx < 1.0 / car_hz:
            return
        self._last_car_tx = now
        err_u = out.err_u if out is not None else 0.0
        err_v = out.err_v if out is not None else 0.0
        flags = 0
        if self._laser_on():
            flags |= AimFlags.LASER_ON
        if out is not None:
            if out.valid:
                flags |= AimFlags.AIM_VALID
            if out.boost:
                flags |= AimFlags.BOOST
            if out.locked:
                flags |= AimFlags.LOCKED
        if self.state == State.DRAW:
            flags |= AimFlags.DRAWING
        quality = out.quality if out is not None else 0
        self.car_link.send(proto.MsgId.AIM_STATE,
                           proto.pack_aim_state(bool(out and out.locked),
                                                int(self.state), quality,
                                                flags, err_u, err_v))

    # ------------------------------------------------------------------
    def _start_tx_thread(self) -> None:
        """独立线程：按固定频率给 H723 发 MODE + AIM。

        为什么必须单独开线程（2026-09-27 晚 现场事故）：H723 侧有一个 0.5s 的
        AIM 看门狗，超时就自动降级到 STAB 并清零偏置。而上位机是单线程模型，
        检测偶尔会卡 0.4~0.55 秒（全分辨率二值化 + 多个候选透视矫正，
        实测帧间隔分布里 0.4~0.55s 出现了 74 次）—— 这段时间发不出 AIM，
        看门狗就跳闸。跳闸之后 H723 不会自己回来，而上位机还以为在 AIM，
        于是偏置被一路积分到限幅、画面却不动（这就是现场"云台没反应"的真凶）。
        把发送搬到独立线程之后，哪怕检测卡住一秒，链路也一直是活的。
        """
        if self.gimbal_link is None:
            return
        self._tx_stop = threading.Event()

        def _loop():
            period = 1.0 / max(1.0, min(200.0, self.cfg.link_gimbal.tx_hz))
            while not self._tx_stop.is_set():
                t0 = time.monotonic()
                try:
                    self._send_gimbal(t0)
                except Exception:                              # noqa: BLE001
                    pass
                slack = (t0 + period) - time.monotonic()
                if slack > 0:
                    time.sleep(slack)

        self._tx_thread = threading.Thread(target=_loop, name="aim_tx", daemon=True)
        self._tx_thread.start()

    def _send_gimbal(self, now: float) -> None:
        """给 H723 发 MODE（周期性重发）+ AIM（当前偏置与标志）。"""
        if self.gimbal_link is None:
            return
        mode = self._desired_mode()
        # ⚠ 2026-09-27 晚 现场事故：H723 侧有"0.5s 收不到 AIM 就自动降级到 STAB"
        #   的安全看门狗。一旦触发（串口卡顿、上位机被调度走），H723 会自己
        #   回到 STAB 并且**不会再回到 AIM** —— 而上位机这边只在"模式变化时"
        #   才下发 MODE，于是它一直以为还在 AIM：偏置被积分到 ±170°/-60° 的
        #   限幅上，画面却一动不动（实测记录的 yaw 就是从 0 一路爬到 170）。
        #   现在：① 每 mode_reassert_s 秒无条件下发一次 MODE（同值重发在固件里
        #   是空操作，不刷日志）；② 偏置被夹住时冻结积分，避免风阻式累积。
        last_mode_t = getattr(self, "_last_mode_t", 0.0)
        if (mode != getattr(self, "_sent_mode", None)
                or (now - last_mode_t) >= self.cfg.link_gimbal.mode_reassert_s):
            self.gimbal_link.send(proto.MsgId.MODE, proto.pack_mode(mode))
            changed = mode != getattr(self, "_sent_mode", None)
            self._sent_mode = mode
            self._last_mode_t = now
            if changed:
                self.log("下发模式 %s" % AimMode.name_of(mode), level="DBG")
        tx_hz = max(1.0, self.cfg.link_gimbal.tx_hz)
        if now - self._last_tx < 1.0 / tx_hz:
            return
        self._last_tx = now
        flags = 0
        if self._laser_on():
            flags |= AimFlags.LASER_ON
        out = getattr(self, "aim_out", None)
        if out is not None:
            if out.valid:
                flags |= AimFlags.AIM_VALID
            if out.boost:
                flags |= AimFlags.BOOST
            if out.locked:
                flags |= AimFlags.LOCKED
        if self.state == State.DRAW:
            flags |= AimFlags.DRAWING
        quality = out.quality if out is not None else 0
        self.gimbal_link.send(proto.MsgId.AIM,
                              proto.pack_aim(self.aim.yaw_deg, self.aim.pitch_deg,
                                             flags, quality))
        # 偏置顶到限幅 = 环路被打开（云台没在 AIM 模式、或者机构卡住）。
        # 现场实测就是靠这个现象才定位到"H723 偷偷降级回 STAB"。
        ctl = self.cfg.control
        if (abs(self.aim.yaw_deg) > ctl.max_yaw_deg - 1.0
                or abs(self.aim.pitch_deg) > ctl.max_pitch_deg - 1.0):
            if now - getattr(self, "_sat_warn_t", -1e9) > 3.0:
                self._sat_warn_t = now
                self.log("偏置顶到限幅 (%.0f°, %.0f°) —— 云台没跟上指令，"
                         "检查是否被切回 STAB / 机构卡住"
                         % (self.aim.yaw_deg, self.aim.pitch_deg), level="WARN")

    # ------------------------------------------------------------------
    # 记录与显示
    # ------------------------------------------------------------------
    def _record(self, image: np.ndarray, now: float) -> None:
        if self.rec is None:
            return
        target = self.target
        spot = self.spot
        out = getattr(self, "aim_out", None)
        track = getattr(self, "track", None)
        car = self.car
        gz = self.gimbal
        sim_miss: object = ""
        sim_on: object = ""
        sim_x: object = ""
        sim_y: object = ""
        sim_d2: object = ""
        if self.sim_world is not None:
            m, on = self.sim_world.truth_miss_mm()
            sim_miss = round(m, 3)
            sim_on = int(on)
            pt = self.sim_world.laser_spot_point()
            if pt is not None:
                sim_x = round(float(pt[0]), 2)
                sim_y = round(float(pt[1]), 2)
                r = ((pt[0] - 105.0) ** 2 + (pt[1] - 148.5) ** 2) ** 0.5
                sim_d2 = round(abs(r - self.cfg.draw.radius_mm), 3)
        draw = getattr(self, "draw_status", None)
        self.rec.log(
            t=round(now - self.t0, 4),
            state=self.state.label,
            target_ok=int(target is not None),
            src=target.source if target is not None else "",
            conf=round(target.confidence, 3) if target is not None else "",
            dist_m=round(target.distance_m, 3) if target is not None else "",
            tu=round(target.uv[0], 2) if target is not None else "",
            tv=round(target.uv[1], 2) if target is not None else "",
            su=round(self._spot_uv()[0], 2) if self._spot_uv() else "",
            sv=round(self._spot_uv()[1], 2) if self._spot_uv() else "",
            err_u=round(out.err_u, 2) if out is not None else "",
            err_v=round(out.err_v, 2) if out is not None else "",
            err_norm=round(out.err_norm, 2) if out is not None else 0.0,
            yaw=round(self.aim.yaw_deg, 3),
            pitch=round(self.aim.pitch_deg, 3),
            valid=int(bool(out and out.valid)),
            coast=int(bool(out and out.coasting)),
            locked=int(bool(out and out.locked)),
            boost=int(bool(out and out.boost)),
            quality=out.quality if out is not None else "",
            laser=int(self._laser_on()),
            spot_area=spot.area if spot is not None else "",
            ring_score=round(target.ring_score, 3) if target is not None else "",
            gz_state=gz.state if gz is not None else "",
            gz_ready=int(bool(gz and gz.ready())),
            gz_yaw=round(gz.yaw_deg, 3) if gz is not None else "",
            gz_pitch=round(gz.pitch_deg, 3) if gz is not None else "",
            gz_motor=round(gz.yaw_motor_deg, 2) if gz is not None else "",
            car_lap=car.lap_index if car is not None else "",
            car_seg=car.segment if car is not None else "",
            car_prog=car.progress if car is not None else "",
            car_run=int(bool(car and car.running)),
            draw_phase=round(draw.phase, 4) if draw is not None else "",
            draw_err=round(draw.phase_err, 4) if draw is not None else "",
            sim_miss_mm=sim_miss,
            sim_on_paper=sim_on,
            sim_spot_x=sim_x,
            sim_spot_y=sim_y,
            sim_d2_mm=sim_d2,
        )
        if self.cfg.app.record_video or self.cfg.app.show_window or \
                self.cfg.app.snapshot_every_s > 0:
            annotated = self._annotate(image)
            self.rec.write_frame(annotated)
            if self._snapshot_due(now):
                self._save_snapshot(annotated, now)

    def _snapshot_due(self, now: float) -> bool:
        every = self.cfg.app.snapshot_every_s
        if every <= 0:
            return False
        if now - self._last_snapshot >= every:
            self._last_snapshot = now
            return True
        return False

    def _save_snapshot(self, frame: np.ndarray, now: float) -> None:
        if self.rec is None:
            return
        path = self.rec.base + "_%05d.jpg" % int(now - self.t0)
        try:
            cv2.imwrite(path, frame)
        except Exception:
            pass

    def _annotate(self, image: np.ndarray) -> np.ndarray:
        frame = image.copy()
        lines: List[str] = []
        if self.car is not None:
            lines.append("car " + self.car.describe())
        if self.gimbal is not None:
            lines.append("gimbal st=%d fl=0x%02X yaw=%.1f pit=%.1f motor=%.1f"
                         % (self.gimbal.state, self.gimbal.flags,
                            self.gimbal.yaw_deg, self.gimbal.pitch_deg,
                            self.gimbal.yaw_motor_deg))
            lines.append("绕线监视: 相对偏航角=%.0f° (每圈 -360°，>180° 就该解绕)"
                         % self.gimbal.yaw_motor_deg)
        if self.target is not None:
            lines.append("target " + self.target.describe())
        draw = getattr(self, "draw_status", None)
        if draw is not None:
            lines.append("draw phase=%.3f err=%.3f lap=%.1fs"
                         % (draw.phase, draw.phase_err, draw.lap_time_s))
        viz.draw_overlay(frame, target=self.target, spot=self.spot,
                         aim=getattr(self, "aim_out", None), lines=lines,
                         state="%s%s" % (self.state.label,
                                         " LOCKED" if self._lock_time else ""),
                         fps=self.fps)
        if self._snapshot_due_peek() and self._last_rect is not None and self.target is not None:
            rect_img = cv2.warpPerspective(
                image, self._last_rect.h_img2rect, self._last_rect.size,
                flags=cv2.INTER_LINEAR)
            frame = viz.draw_rectified_inset(frame, rect_img, max_w=160)
        return frame

    def _snapshot_due_peek(self) -> bool:
        every = self.cfg.app.snapshot_every_s
        if every <= 0:
            return False
        return (time.monotonic() - self._last_snapshot) < every

    # ------------------------------------------------------------------
    # 诊断工具
    # ------------------------------------------------------------------
    def check(self, duration: float = 6.0, laser: bool = False) -> int:
        """只做检测，不跑状态机。用于确认"相机能不能看清靶纸/光斑"。"""
        cfg = self.cfg
        if not self.setup():
            return 1
        self.log("自检 %.1fs：激光%s" % (duration, "开" if laser else "关"))
        if laser and self.gimbal_link is not None:
            # 真实固件里激光只在 AIM 模式下点亮，所以先切模式
            self.gimbal_link.send(proto.MsgId.MODE, proto.pack_mode(AimMode.AIM))
        t_end = time.monotonic() + duration
        next_report = time.monotonic() + 1.0
        n = ok = spots = 0
        conf_sum = 0.0
        dist_sum = 0.0
        spot_u = spot_v = 0.0
        ring_sum = 0.0
        while time.monotonic() < t_end:
            frame, _ = self.camera.read()
            if frame is not None:
                n += 1
                target = self.detector.detect(frame.image, self.cfg.camera.process_scale)
                if target is not None:
                    ok += 1
                    conf_sum += target.confidence
                    dist_sum += target.distance_m
                    ring_sum += target.ring_score
                spot = self.spot_detector.detect(frame.image, self.cfg.camera.process_scale)
                if spot is not None:
                    spots += 1
                    spot_u, spot_v = spot.uv
            now = time.monotonic()
            if now >= next_report:
                next_report = now + 1.0
                print("  帧=%d 靶纸=%d(%.0f%%) 光斑=%d 平均置信=%.2f 平均环分=%.2f "
                      "距离=%.2fm 光斑=(%.0f,%.0f)"
                      % (n, ok, 100.0 * ok / max(1, n), spots,
                         conf_sum / max(1, ok), ring_sum / max(1, ok),
                         dist_sum / max(1, ok), spot_u, spot_v), flush=True)
            if laser and self.gimbal_link is not None:
                self.gimbal_link.send(
                    proto.MsgId.AIM,
                    proto.pack_aim(self.aim.yaw_deg, self.aim.pitch_deg,
                                   AimFlags.LASER_ON, 0))
            time.sleep(0.002)
        ok_rate = 100.0 * ok / max(1, n)
        self.log("自检结果: 靶纸检出率 %.0f%%（%d/%d），光斑检出率 %.0f%%"
                 % (ok_rate, ok, n, 100.0 * spots / max(1, n)))
        if ok:
            self.log("平均距离 %.3fm，平均置信 %.2f，平均环覆盖分 %.2f"
                     % (dist_sum / ok, conf_sum / ok, ring_sum / ok))
        if spots:
            self.log("光斑平均位置 (%.1f, %.1f) —— 这个值可以直接写进 "
                     "configs/default.yaml 的 calib.boresight_uv" % (spot_u, spot_v))
        if ok_rate < 60.0:
            self.log("检出率偏低：先调 target.adaptive_block / require_red，"
                     "或降低相机分辨率换帧率", level="WARN")
        self.shutdown()
        return 0 if ok_rate >= 60.0 else 2

    def unwind(self, duration: float = 8.0) -> int:
        """解绕：把偏航相对角转回 0（激光关闭），跑完一轮后必须做。"""
        if not self.setup():
            return 1
        self.log("开始解绕（相对偏航角应在 %.1fs 内回到 0）" % duration)
        self.gimbal_link.send(proto.MsgId.MODE, proto.pack_mode(AimMode.UNWIND))
        # 120°/s = 20rpm：低于电机的静摩擦脱困转速(约10rpm)就完全推不动，
        # 解绕会卡在原地不动。固件侧还会把这个值夹到 [15,60]rpm 兜底。
        self.gimbal_link.send(proto.MsgId.UNWIND, proto.pack_unwind(120.0))
        self.gimbal_link.send(proto.MsgId.AIM, proto.pack_aim(0.0, 0.0, 0, 0))
        t_end = time.monotonic() + duration
        last_report = 0.0
        while time.monotonic() < t_end:
            for frame in self.gimbal_link.read_frames():
                if frame.msg_id == proto.MsgId.GIMBAL_STATE:
                    self.gimbal = proto.unpack_gimbal_state(frame.payload)
                elif frame.msg_id == proto.MsgId.TEXT:
                    self.log("H723 %s" % frame.payload.decode("utf-8", "replace"))
                elif frame.msg_id == proto.MsgId.ACK:
                    pass
            now = time.monotonic()
            if now - last_report > 0.5:
                last_report = now
                if self.gimbal is not None:
                    self.log("相对偏航角 = %.1f°" % self.gimbal.yaw_motor_deg)
            self.gimbal_link.send(proto.MsgId.AIM, proto.pack_aim(0.0, 0.0, 0, 0))
            time.sleep(0.02)
        self.gimbal_link.send(proto.MsgId.MODE, proto.pack_mode(AimMode.STAB))
        self.log("解绕结束，已切回 STAB")
        self.shutdown()
        return 0
