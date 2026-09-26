"""配置加载。

优先级：命令行 --config > configs/default.yaml > 本文件里的 dataclass 默认值。
有 PyYAML 就用 PyYAML，没有就用 simple_yaml 回退，两者结果一致。
所有参数都可以用 --set section.key=value 在命令行覆盖。
"""

import copy
import os
from dataclasses import dataclass, field, fields
from typing import Any, Dict, List, Optional, Tuple

from . import simple_yaml

try:
    import yaml as _yaml
except Exception:
    _yaml = None

__all__ = ["AppConfig", "load_config", "save_yaml", "find_default_config",
           "read_yaml", "merged_for_record"]

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _flt(data, key, default: float) -> float:
    try:
        return float(data.get(key, default))
    except (TypeError, ValueError, AttributeError):
        return default


def _int(data, key, default: int) -> int:
    try:
        return int(data.get(key, default))
    except (TypeError, ValueError, AttributeError):
        return default


def _bool(data, key, default: bool) -> bool:
    v = data.get(key, default)
    if isinstance(v, str):
        return v.strip().lower() in ("1", "true", "yes", "on")
    return bool(v)


def _str(data, key, default: str) -> str:
    v = data.get(key, default)
    return default if v is None else str(v)


def _tuple_f(data, key, default: Tuple[float, ...]) -> Tuple[float, ...]:
    v = data.get(key, None)
    if v is None:
        return default
    if isinstance(v, (int, float)):
        return (float(v),) * len(default)
    try:
        return tuple(float(x) for x in v)
    except (TypeError, ValueError):
        return default


def _opt_tuple_f(data, key, default):
    v = data.get(key, None)
    if v is None:
        return default
    if isinstance(v, (int, float)):
        return None
    try:
        return (float(v[0]), float(v[1]))
    except (TypeError, ValueError, IndexError):
        return default


def _table_f(data, key) -> List[Tuple[float, float, float]]:
    """解析 [[距离m, u, v], ...] 形式的"光斑位置-距离"标定表。

    脏数据直接丢掉、不抛异常：现场手改 yaml 很容易写错一格，
    宁可少用一条标定点，也不能让程序起不来。
    """
    raw = data.get(key, None)
    out: List[Tuple[float, float, float]] = []
    if isinstance(raw, (list, tuple)):
        for item in raw:
            try:
                vals = [float(x) for x in item]
            except (TypeError, ValueError):
                continue
            if len(vals) >= 3 and vals[0] > 0.05:
                out.append((vals[0], vals[1], vals[2]))
    out.sort(key=lambda t: t[0])
    return out


@dataclass
class CameraConfig:
    source: str = "uvc"
    device: Any = 0
    width: int = 1280
    height: int = 720
    fps: int = 60
    fourcc: str = "MJPG"
    exposure: int = 0
    autofocus: int = 1
    # 0 = 不要动驱动的缓冲数（推荐）。实测：设成 1 时 V4L2 无法流水线采集，
    # 帧率从 50fps 直接掉到 17fps（3 倍损失）。
    # 我们本来就靠"取帧线程持续丢旧帧"来保证不积压，不需要靠减少缓冲来降延迟。
    buffer_size: int = 0
    warmup_frames: int = 5
    # 检测用的缩放比例。实测：地瓜派 X3 上 1280x720 全分辨率检测要 111ms/帧
    # （=9fps，瓶颈是 CPU 而不是相机），缩到 0.5 后只要 ~30ms（=30fps）。
    # 精度几乎不损失 —— 黑框定位只需要粗略，真正的精度来自后面在矫正图上
    # 做的红圈亚像素质心（那一步不受这里影响）。
    process_scale: float = 0.5

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "CameraConfig":
        d = d or {}
        dev = d.get("device", 0)
        if isinstance(dev, str) and dev.strip().isdigit():
            dev = int(dev.strip())
        return CameraConfig(
            source=_str(d, "source", "uvc"),
            device=dev,
            width=_int(d, "width", 1280),
            height=_int(d, "height", 720),
            fps=_int(d, "fps", 60),
            fourcc=_str(d, "fourcc", "MJPG"),
            exposure=_int(d, "exposure", 0),
            autofocus=_int(d, "autofocus", 1),
            buffer_size=_int(d, "buffer_size", 0),
            warmup_frames=_int(d, "warmup_frames", 5),
            process_scale=_flt(d, "process_scale", 0.5),
        )


@dataclass
class CalibConfig:
    fx: float = 640.0
    fy: float = 640.0
    cx: float = 640.0
    cy: float = 360.0
    dist: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0)
    boresight_uv: Optional[Tuple[float, float]] = None
    # 光斑位置随距离漂移（视差）的标定表：[[距离m, u, v], ...]
    # 为什么需要它：激光轴和相机轴不重合时，光斑在画面里的位置是距离的函数
    # （详见 laser.py 里 boresight_for_distance 的推导）。光斑能检到时不用它；
    # 一旦检不到（反光、过曝、IR-cut 滤掉），它是唯一还能保住精度的兜底。
    boresight_table: List[Tuple[float, float, float]] = field(default_factory=list)
    sign_yaw: float = 1.0
    sign_pitch: float = 1.0

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "CalibConfig":
        d = d or {}
        return CalibConfig(
            fx=_flt(d, "fx", 640.0),
            fy=_flt(d, "fy", 640.0),
            cx=_flt(d, "cx", 640.0),
            cy=_flt(d, "cy", 360.0),
            dist=_tuple_f(d, "dist", (0.0, 0.0, 0.0, 0.0, 0.0)),
            boresight_uv=_opt_tuple_f(d, "boresight_uv", None),
            boresight_table=_table_f(d, "boresight_table"),
            sign_yaw=_flt(d, "sign_yaw", 1.0),
            sign_pitch=_flt(d, "sign_pitch", 1.0),
        )


@dataclass
class TargetConfig:
    paper_mm: Tuple[float, float] = (210.0, 297.0)
    tape_mm: float = 18.0
    warp_scale: float = 4.0
    min_area_ratio: float = 0.0015
    max_area_ratio: float = 0.85
    poly_eps_ratio: float = 0.02
    aspect_tol: float = 0.22
    red_enable: bool = True
    red_r_minus_g: int = 40
    red_r_minus_b: int = 40
    red_v_min: int = 80
    red_min_px: int = 80
    red_center_tol_px: float = 25.0
    fallback_red: bool = True
    max_candidates: int = 4
    ring_tol_mm: float = 2.5
    adaptive_block: int = 61
    adaptive_c: int = 8
    use_otsu: bool = False
    # 红圈是否作为"硬门槛"。
    # 实际相机下【建议关掉】：红圈线宽 1mm，0.8m 处成像不到 1 个像素，
    # 稍一失焦就检不到，当硬门槛会把真靶纸拒掉。
    # 关掉后：红圈只用来"加分 + 在检到时做亚像素质心精修"，
    # 硬门槛改由黑胶带厚度校验承担（那个抗模糊得多）。
    require_red: bool = False
    min_ring_score: float = 0.45
    min_confidence: float = 0.50
    # 黑胶带厚度校验（抗模糊的主力判据）。
    # 为什么要它：红圈线宽只有 1mm，在 0.8m 处成像不到 1 个像素，
    # 稍微失焦就检不到 —— 拿它当硬门槛会把真靶纸拒掉。
    # 而黑胶带有 18mm 宽（0.8m 处约 14 像素），糊了也能量出厚度。
    #
    # 判据：把四边形矫正成 A4 后，量四周黑边厚度，应该接近 18mm。
    # 场地里那条 1m 黑色巡线方框：18mm 线宽 / 1000mm 边长 = 1.8%，
    # 被强行矫正成 A4 后量出来只有约 3.8mm -> 比值 0.21 -> 直接排除。
    tape_min_ratio: float = 0.40
    tape_max_ratio: float = 1.80
    dark_thresh: int = 100

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "TargetConfig":
        d = d or {}
        paper = _tuple_f(d, "paper_mm", (210.0, 297.0))
        if len(paper) < 2:
            paper = (210.0, 297.0)
        return TargetConfig(
            paper_mm=(paper[0], paper[1]),
            tape_mm=_flt(d, "tape_mm", 18.0),
            warp_scale=_flt(d, "warp_scale", 4.0),
            min_area_ratio=_flt(d, "min_area_ratio", 0.0015),
            max_area_ratio=_flt(d, "max_area_ratio", 0.85),
            poly_eps_ratio=_flt(d, "poly_eps_ratio", 0.02),
            aspect_tol=_flt(d, "aspect_tol", 0.22),
            red_enable=_bool(d, "red_enable", True),
            red_r_minus_g=_int(d, "red_r_minus_g", 40),
            red_r_minus_b=_int(d, "red_r_minus_b", 40),
            red_v_min=_int(d, "red_v_min", 80),
            red_min_px=_int(d, "red_min_px", 80),
            red_center_tol_px=_flt(d, "red_center_tol_px", 25.0),
            fallback_red=_bool(d, "fallback_red", True),
            max_candidates=_int(d, "max_candidates", 4),
            ring_tol_mm=_flt(d, "ring_tol_mm", 2.5),
            adaptive_block=_int(d, "adaptive_block", 61),
            adaptive_c=_int(d, "adaptive_c", 8),
            use_otsu=_bool(d, "use_otsu", False),
            require_red=_bool(d, "require_red", False),
            min_ring_score=_flt(d, "min_ring_score", 0.45),
            min_confidence=_flt(d, "min_confidence", 0.50),
            tape_min_ratio=_flt(d, "tape_min_ratio", 0.40),
            tape_max_ratio=_flt(d, "tape_max_ratio", 1.80),
            dark_thresh=_int(d, "dark_thresh", 100),
        )


@dataclass
class LaserConfig:
    b_min: int = 150
    b_minus_others: int = 35
    min_area: int = 8
    max_area_ratio: float = 0.02
    # 激光是否【物理常亮】（直接接电源、不由 GPIO 控制）。
    # 设 true 时：光斑始终在画面里，于是永远用"实测光斑"做闭环，
    # 不会退回用标定的光轴点兜底 —— 精度更好，也省掉一次标定。
    always_on: bool = False
    # 下面三条用来挡"假光斑"（实测踩过：图像边缘的色度伪影被当成激光，
    # 而且因为"学习光轴点"机制，一个假光斑会把门控中心永久带偏）
    max_aspect: float = 3.0        # 光斑近似圆形；细长条不是激光
    border_margin: int = 4         # 贴画面边缘的连通域不要（光斑不会被裁掉一半）
    gate_enable: bool = True
    gate_radius_px: float = 90.0
    gate_relax_px: float = 260.0
    subpixel: bool = True

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "LaserConfig":
        d = d or {}
        return LaserConfig(
            b_min=_int(d, "b_min", 150),
            b_minus_others=_int(d, "b_minus_others", 35),
            min_area=_int(d, "min_area", 8),
            max_area_ratio=_flt(d, "max_area_ratio", 0.02),
            always_on=_bool(d, "always_on", False),
            max_aspect=_flt(d, "max_aspect", 3.0),
            border_margin=_int(d, "border_margin", 4),
            gate_enable=_bool(d, "gate_enable", True),
            gate_radius_px=_flt(d, "gate_radius_px", 90.0),
            gate_relax_px=_flt(d, "gate_relax_px", 260.0),
            subpixel=_bool(d, "subpixel", True),
        )


@dataclass
class ControlConfig:
    kp: float = 0.35
    rate_limit_dps: float = 220.0
    max_yaw_deg: float = 170.0
    max_pitch_deg: float = 60.0
    deadband_px: float = 3.0
    lead_s: float = 0.045
    lead_gain: float = 1.0
    kp_corner: float = 0.55
    corner_boost_s: float = 0.45
    corner_lead_scale: float = 0.35
    lost_coast_s: float = 0.35
    lost_research_s: float = 1.20
    locked_tol_px: float = 6.0
    locked_frames: int = 6

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "ControlConfig":
        d = d or {}
        return ControlConfig(
            kp=_flt(d, "kp", 0.35),
            rate_limit_dps=_flt(d, "rate_limit_dps", 220.0),
            max_yaw_deg=_flt(d, "max_yaw_deg", 170.0),
            max_pitch_deg=_flt(d, "max_pitch_deg", 60.0),
            deadband_px=_flt(d, "deadband_px", 3.0),
            lead_s=_flt(d, "lead_s", 0.045),
            lead_gain=_flt(d, "lead_gain", 1.0),
            kp_corner=_flt(d, "kp_corner", 0.55),
            corner_boost_s=_flt(d, "corner_boost_s", 0.45),
            corner_lead_scale=_flt(d, "corner_lead_scale", 0.35),
            lost_coast_s=_flt(d, "lost_coast_s", 0.35),
            lost_research_s=_flt(d, "lost_research_s", 1.20),
            locked_tol_px=_flt(d, "locked_tol_px", 6.0),
            locked_frames=_int(d, "locked_frames", 6),
        )


@dataclass
class AcquireConfig:
    mode: str = "spiral"
    yaw_range_deg: float = 175.0
    pitch_range_deg: float = 35.0
    step_deg: float = 28.0
    dwell_ms: int = 90
    settle_ms: int = 60
    timeout_s: float = 12.0
    coarse_max_step_deg: float = 45.0
    move_dps: float = 260.0

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "AcquireConfig":
        d = d or {}
        return AcquireConfig(
            mode=_str(d, "mode", "spiral"),
            yaw_range_deg=_flt(d, "yaw_range_deg", 175.0),
            pitch_range_deg=_flt(d, "pitch_range_deg", 35.0),
            step_deg=_flt(d, "step_deg", 28.0),
            dwell_ms=_int(d, "dwell_ms", 90),
            settle_ms=_int(d, "settle_ms", 60),
            timeout_s=_flt(d, "timeout_s", 12.0),
            coarse_max_step_deg=_flt(d, "coarse_max_step_deg", 45.0),
            move_dps=_flt(d, "move_dps", 260.0),
        )


@dataclass
class DrawConfig:
    radius_mm: float = 60.0
    direction: str = "ccw"
    phase_offset: float = 0.0
    phase_source: str = "corner"
    lap_time_s: float = 20.0
    max_speed_dps: float = 90.0
    prestart_s: float = 0.35
    start_tol_px: float = 10.0
    laser_gate: bool = True

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "DrawConfig":
        d = d or {}
        return DrawConfig(
            radius_mm=_flt(d, "radius_mm", 60.0),
            direction=_str(d, "direction", "ccw"),
            phase_offset=_flt(d, "phase_offset", 0.0),
            phase_source=_str(d, "phase_source", "corner"),
            lap_time_s=_flt(d, "lap_time_s", 20.0),
            max_speed_dps=_flt(d, "max_speed_dps", 90.0),
            prestart_s=_flt(d, "prestart_s", 0.35),
            start_tol_px=_flt(d, "start_tol_px", 10.0),
            laser_gate=_bool(d, "laser_gate", True),
        )


@dataclass
class LinkConfig:
    enable: bool = True
    port: str = "/dev/ttyUSB0"
    baud: int = 921600
    tx_hz: float = 100.0
    telemetry_timeout_s: float = 0.5

    @staticmethod
    def from_dict(d: Dict[str, Any], default_port: str, default_baud: int) -> "LinkConfig":
        d = d or {}
        return LinkConfig(
            enable=_bool(d, "enable", True),
            port=_str(d, "port", default_port),
            baud=_int(d, "baud", default_baud),
            tx_hz=_flt(d, "tx_hz", 100.0),
            telemetry_timeout_s=_flt(d, "telemetry_timeout_s", 0.5),
        )


@dataclass
class AppSection:
    run_dir: str = "out"
    log_level: str = "INFO"
    show_window: bool = False
    snapshot_every_s: float = 0.0
    record_video: bool = False
    timer_start: str = "poweron"
    trigger_file: str = "out/trigger"
    budget_s: float = 4.0
    max_fps: float = 0.0

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "AppSection":
        d = d or {}
        return AppSection(
            run_dir=_str(d, "run_dir", "out"),
            log_level=_str(d, "log_level", "INFO"),
            show_window=_bool(d, "show_window", False),
            snapshot_every_s=_flt(d, "snapshot_every_s", 0.0),
            record_video=_bool(d, "record_video", False),
            timer_start=_str(d, "timer_start", "poweron"),
            trigger_file=_str(d, "trigger_file", "out/trigger"),
            budget_s=_flt(d, "budget_s", 4.0),
            max_fps=_flt(d, "max_fps", 0.0),
        )


@dataclass
class SimConfig:
    target_center_mm: Tuple[float, float] = (0.0, 0.0)
    distance_m: float = 0.80
    camera_offset_mm: Tuple[float, float] = (0.0, -60.0)
    laser_on: bool = True
    noise: float = 2.0
    motion_blur: float = 0.0
    car_enable: bool = True
    car_lap_time_s: float = 20.0
    car_start_corner: int = 0
    actuator_tau_s: float = 0.03
    actuator_max_rate_dps: float = 300.0
    actuator_delay_s: float = 0.02

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "SimConfig":
        d = d or {}
        return SimConfig(
            target_center_mm=_tuple_f(d, "target_center_mm", (0.0, 0.0)),
            distance_m=_flt(d, "distance_m", 0.80),
            camera_offset_mm=_tuple_f(d, "camera_offset_mm", (0.0, -60.0)),
            laser_on=_bool(d, "laser_on", True),
            noise=_flt(d, "noise", 2.0),
            motion_blur=_flt(d, "motion_blur", 0.0),
            car_enable=_bool(d, "car_enable", True),
            car_lap_time_s=_flt(d, "car_lap_time_s", 20.0),
            car_start_corner=_int(d, "car_start_corner", 0),
            actuator_tau_s=_flt(d, "actuator_tau_s", 0.03),
            actuator_max_rate_dps=_flt(d, "actuator_max_rate_dps", 300.0),
            actuator_delay_s=_flt(d, "actuator_delay_s", 0.02),
        )


@dataclass
class AppConfig:
    camera: CameraConfig = field(default_factory=CameraConfig)
    calib: CalibConfig = field(default_factory=CalibConfig)
    target: TargetConfig = field(default_factory=TargetConfig)
    laser: LaserConfig = field(default_factory=LaserConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    acquire: AcquireConfig = field(default_factory=AcquireConfig)
    draw: DrawConfig = field(default_factory=DrawConfig)
    link_gimbal: LinkConfig = field(
        default_factory=lambda: LinkConfig("/dev/ttyUSB0", 921600))
    link_car: LinkConfig = field(
        default_factory=lambda: LinkConfig("/dev/ttyUSB1", 115200))
    app: AppSection = field(default_factory=AppSection)
    sim: SimConfig = field(default_factory=SimConfig)
    source_path: str = ""
    warnings: List[str] = field(default_factory=list)

    def as_dict(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {}
        for f in fields(self):
            if f.name in ("source_path", "warnings"):
                continue
            value = getattr(self, f.name)
            if hasattr(value, "__dataclass_fields__"):
                out[f.name] = {g.name: getattr(value, g.name) for g in fields(value)}
            else:
                out[f.name] = value
        return out

    def set_value(self, dotted: str, value: Any) -> bool:
        """支持 'control.kp=0.4' 形式的覆盖，返回是否成功。"""
        parts = dotted.split(".")
        if len(parts) != 2:
            self.warnings.append("忽略无法识别的覆盖项: %s" % dotted)
            return False
        section, key = parts
        if not hasattr(self, section):
            self.warnings.append("忽略未知配置段: %s" % section)
            return False
        obj = getattr(self, section)
        if not hasattr(obj, key):
            self.warnings.append("忽略未知配置项: %s" % dotted)
            return False
        current = getattr(obj, key)
        try:
            if isinstance(current, bool):
                setattr(obj, key, str(value).lower() in ("1", "true", "yes", "on"))
            elif isinstance(current, int) and not isinstance(current, bool):
                setattr(obj, key, int(float(value)))
            elif isinstance(current, float):
                setattr(obj, key, float(value))
            elif isinstance(current, tuple):
                cast = float if isinstance(current[0], float) else int
                setattr(obj, key, tuple(cast(x) for x in str(value).split(",")))
            else:
                setattr(obj, key, value)
            return True
        except (TypeError, ValueError) as exc:
            self.warnings.append("覆盖 %s 失败(%s)，保持原值" % (dotted, exc))
            return False


def find_default_config() -> str:
    for name in ("default.yaml", "default.yml"):
        p = os.path.join(PROJECT_ROOT, "configs", name)
        if os.path.exists(p):
            return p
    return ""


def find_config(name: str) -> str:
    """按名字在 configs/ 下找配置文件（sim / default / intrinsic ...）。"""
    for ext in ("", ".yaml", ".yml"):
        p = os.path.join(PROJECT_ROOT, "configs", name + ext)
        if os.path.exists(p):
            return p
    return ""


def find_calibration_overlays() -> List[str]:
    """自动收集标定文件（存在才用，按 intrinsic -> boresight 的顺序叠加）。

    这样日常启动只需要 `python main.py run` 一条命令 ——
    标定结果（相机内参、光轴点、偏置符号）会自动叠加到默认配置上，
    不用每次手动敲 --extra-config。
    """
    out: List[str] = []
    base = os.path.abspath(find_default_config()) if find_default_config() else ""
    for name in ("intrinsic", "boresight"):
        p = find_config(name)
        if p and os.path.abspath(p) != base:
            out.append(p)
    return out


def read_yaml(path: str) -> Dict[str, Any]:
    if _yaml is not None:
        with open(path, "r", encoding="utf-8") as fh:
            return _yaml.safe_load(fh) or {}
    return simple_yaml.load(path) or {}


def save_yaml(path: str, data: Dict[str, Any]) -> None:
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    if _yaml is not None:
        with open(path, "w", encoding="utf-8") as fh:
            _yaml.safe_dump(data, fh, allow_unicode=True, sort_keys=False,
                            default_flow_style=False)
    else:
        simple_yaml.dump(data, path)


def _apply(cfg: AppConfig, raw: Dict[str, Any]) -> None:
    known = ("camera", "calib", "target", "laser", "control", "acquire",
             "draw", "link_gimbal", "link_car", "app", "sim")
    for key in raw:
        if key not in known:
            cfg.warnings.append("未知配置段，已忽略: %s" % key)
    cfg.camera = CameraConfig.from_dict(raw.get("camera"))
    cfg.calib = CalibConfig.from_dict(raw.get("calib"))
    cfg.target = TargetConfig.from_dict(raw.get("target"))
    cfg.laser = LaserConfig.from_dict(raw.get("laser"))
    cfg.control = ControlConfig.from_dict(raw.get("control"))
    cfg.acquire = AcquireConfig.from_dict(raw.get("acquire"))
    cfg.draw = DrawConfig.from_dict(raw.get("draw"))
    cfg.link_gimbal = LinkConfig.from_dict(raw.get("link_gimbal"), "/dev/ttyUSB0", 921600)
    cfg.link_car = LinkConfig.from_dict(raw.get("link_car"), "/dev/ttyUSB1", 115200)
    cfg.app = AppSection.from_dict(raw.get("app"))
    cfg.sim = SimConfig.from_dict(raw.get("sim"))


def load_config(path: Optional[str] = None, extra_path: Optional[str] = None,
                overrides: Optional[Dict[str, Any]] = None) -> AppConfig:
    """加载配置。

    extra_path 里的段会覆盖 path 里的同名段（用于叠加标定文件，
    例如 default.yaml + intrinsic.yaml + boresight.yaml）。
    extra_path 可以是单个路径，也可以是路径列表（按顺序叠加，后面的覆盖前面的）。
    """
    cfg = AppConfig()
    base = path or find_default_config()
    if extra_path is None:
        extras: List[str] = []
    elif isinstance(extra_path, str):
        extras = [extra_path]
    else:
        extras = [p for p in extra_path if p]
    raw: Dict[str, Any] = {}
    for p in [base] + extras:
        if not p:
            continue
        if not os.path.exists(p):
            cfg.warnings.append("配置文件不存在，已忽略: %s" % p)
            continue
        part = read_yaml(p)
        if not isinstance(part, dict):
            cfg.warnings.append("配置文件顶层不是映射，已忽略: %s" % p)
            continue
        for k, v in part.items():
            if isinstance(v, dict) and isinstance(raw.get(k), dict):
                merged = dict(raw[k])
                merged.update(v)
                raw[k] = merged
            else:
                raw[k] = v
    cfg.source_path = base
    _apply(cfg, raw)
    for dotted, value in (overrides or {}).items():
        cfg.set_value(dotted, value)
    return cfg


def merged_for_record(cfg: AppConfig) -> Dict[str, Any]:
    """给日志用：把当前生效的配置导出成可打印的 dict。"""
    return copy.deepcopy(cfg.as_dict())
