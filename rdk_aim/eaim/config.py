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


def _param_list(data, key, default) -> List[Tuple[int, float]]:
    """解析 [[参数号, 值], ...] 形式的"启动时下发的在线参数"。

    参数号支持 0x05 这样的十六进制写法（YAML 里写 0x05 会被解析成整数 5，
    写字符串 "0x05" 也能认）。脏数据直接跳过，不让它把程序弄崩。
    """
    raw = data.get(key, None)
    if not isinstance(raw, (list, tuple)) or not raw:
        return list(default)
    out: List[Tuple[int, float]] = []
    for item in raw:
        try:
            pid = int(str(item[0]), 0) if not isinstance(item[0], int) else int(item[0])
            val = float(item[1])
        except (TypeError, ValueError, IndexError, KeyError):
            continue
        if 0 <= pid <= 0xFF:
            out.append((pid, val))
    return out or list(default)


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
    # 参与"打分"的候选四边形个数上限。4 -> 8：远距离/小目标时画面里
    # 大的黑块（柜门、阴影）会把真靶纸挤出候选表（实测就是这么丢靶的）。
    # 每个候选要多做一次透视矫正 + 量测，约 3ms，只在"什么都没检出"时才跑满。
    max_candidates: int = 8
    ring_tol_mm: float = 2.5
    # "回"字结构判据（用户建议）：外框里必须有一个面积占比合理的四边形内窗。
    # 理论占比 = (210-2*18)(297-2*18)/(210*297) ≈ 0.73
    # 默认关（库级默认保持宽松，合成测试不受影响）；
    # 现场配置 configs/default.yaml 里打开，用来挡误检。
    ring_structure_required: bool = False
    ring_ratio_min: float = 0.45
    ring_ratio_max: float = 0.95
    # 严格档全被拒时，是否退到"宽松档"再判一遍（斜视/远距/胶带贴歪时很关键）。
    # 库默认关（保证单测里"严格模式必须拒绝黑方块"）；现场配置里打开。
    loose_fallback: bool = False
    adaptive_block: int = 61
    adaptive_c: int = 8
    use_otsu: bool = False
    # 第二档二值化：严格档一张候选都不给时，用小窗口 + 小偏移再试一遍。
    # 大窗口的局部均值会被亮靶纸拉高，远距离时细黑胶带的对比度被吃掉、环断掉
    # （实测 1.5m 处 block=61 只检出 7/18 个斜视场景，block=31 能检出 14/18）。
    binarize_fallback: bool = True
    fallback_block: int = 31
    fallback_c: float = 4.0
    # ==== 国一那套预处理（2026-09-29 全面借鉴）====
    # 二值化方式：
    #   fixed    = 固定阈值反二值化（国一用的是 35）+ 大核闭运算 + Canny 融合
    #              —— 对"激光把黑框烧出洞""反光把边打断"最有效，因为
    #              Canny 的边缘会把断口接上、大核闭运算会把洞填掉
    #   adaptive = 原来的自适应阈值（光照极不均匀时更稳）
    # ⚠ 现场实测（2026-09-29，11 张现场原图对照）：
    #   固定阈值(国一 35)       -> 检出 0/11   ← 我们这台相机画面偏暗
    #      （纸面灰度只有 77、木柜背景 60~90，固定阈值会把木纹暗块全当黑框）
    #   自适应阈值(旧)          -> 检出 8/11
    #   自适应 + 闭运算 + Canny  -> 检出 8/11（另 3 张是靶纸被画面边缘裁掉）
    #   自适应 + 15x15 大闭运算   -> 检出 1/11（大核把胶带和暗背景粘成一块）
    # 所以：二值化沿用自适应（这是为我们的暗场标定的），
    # 但把国一那两样"治断口"的东西加上 —— 适度闭运算 + Canny 边缘融合。
    binarize_mode: str = "adaptive"
    fixed_thresh: int = 45        # 固定阈值（越低越"只认很黑的东西"）
    close_k: int = 5              # 闭运算核（全分辨率下的像素；国一在 640x480 用 9）
    # 开运算核：先抹掉比它细的东西（靶纸上的红圈、Canny 细边缘），
    # 免得白纸被切成碎片导致"内窗"配对配错（实测会让中心偏 5~6px）。
    # 18mm 胶带在 0.7m 处约 20px 宽，远粗于 5，所以不受影响。
    open_k: int = 1   # 1 = 关掉开运算（现场实测：我们的画面里胶带只有十几像素宽，开运算会把细处抹断）
    gauss_k: int = 5              # 高斯模糊核，必须奇数
    # 是否把 Canny 边缘并进前景。国一用它桥接断口；**我们实测默认关**：
    #   现场 11 张原图：闭5+无Canny = 8/11，闭9+Canny = 8/11（一样）
    #   合成靶纸精度：闭5+无Canny 最大误差 2.4px；加 Canny 之后 800/1200/1600mm
    #   三个距离直接检不到（边缘把白纸切碎，配不到内窗）。
    # 什么时候打开它：现场遇到"激光把胶带烧出洞导致丢靶"时再开（那正是它的用途）。
    use_canny: bool = False
    canny_lo: int = 50
    canny_hi: int = 150
    # 四边形几何校验（国一的 check_rectangle_geometry）
    angle_tol_deg: float = 30.0   # 内角与 90° 的最大偏差
    side_ratio_tol: float = 0.45  # 对边长度相对差上限
    min_area_abs: int = 400       # 绝对面积下限（像素²，全图坐标）
    min_perimeter: int = 60       # 绝对周长下限（像素）
    # "回"字结构：内窗/外框面积比下限。国一用 0.7；我们把外框当黑胶带外沿、
    # 内窗当纸面，实测比例约 0.74，所以 0.55 已经足够严。
    nested_ratio_min: float = 0.55
    # 内窗/外框面积比的**上限**：超过它就说明"外框只是细细一圈"，
    # 那是巡线黑方框（1.8% 线宽 → 面积比 0.93）之类的干扰，不是靶纸。
    # 真靶纸是"18mm 胶带压在 A4 上"，面积比实测 0.74 左右。
    # 这条判据是尺度无关的（不依赖胶带一定是 18mm），比量胶带厚度稳。
    nested_ratio_max: float = 0.92
    # "理想"内窗/外框面积比：18mm 胶带压在 A4 上 ≈ (174×261)/(210×297) = 0.73。
    # 置信度按 |ratio - 该值| 打分 —— 这样能自动挑出真靶纸，而不是被
    # 红圈碎片、柜门内框这些"也像回字"的候选抢走（合成图实测：不打分时
    # 中心会被带偏 5~7px，打分后回到 0.2px 以内）。
    nested_ratio_expect: float = 0.74
    nested_ratio_tol: float = 0.22
    # 找不到内窗（只有一个四边形）时的置信度上限：低于 min_confidence，
    # 也就是"没有回字结构就不认"—— 这条挡住了巡线黑方框、实心黑块。
    single_rect_max_conf: float = 0.44
    # 红圈是否作为"硬门槛"。
    # 实际相机下【建议关掉】：红圈线宽 1mm，0.8m 处成像不到 1 个像素，
    # 稍一失焦就检不到，当硬门槛会把真靶纸拒掉。
    # 关掉后：红圈只用来"加分 + 在检到时做亚像素质心精修"，
    # 硬门槛改由黑胶带厚度校验承担（那个抗模糊得多）。
    require_red: bool = False
    min_ring_score: float = 0.45
    min_confidence: float = 0.50
    # 跳变门控：跟踪阶段里，新检出的靶心离上一次超过这么多像素就判为误检
    # （云台不可能一瞬间把靶心挪几百像素；实测偶发误检会把环路拽走，
    #  表现就是"俯仰突然上下摆动一下"）。丢失超过 1 秒后自动失效。
    jump_gate_px: float = 120.0
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
    # 横向量出来的胶带厚度 / 纵向量出来的，不该差得离谱。
    # 场地里那条 1m 巡线黑方框被强行矫正成 A4 后会量出 6mm / 15mm（比值 2.5），
    # 真靶纸横竖基本一致（≈1.04）。这条判据不依赖"胶带一定是 18mm"的假设。
    tape_aniso_max: float = 2.0
    # "黑边围着亮纸"判据：矫正图里 中心区灰度 - 黑边灰度 不能低于这个值。
    # ⚠ 一定要写成【负】的宽松门槛。现场实测（2026-09-27 晚）：激光正好打中
    #   靶心时，中心会多出一圈自适应阈值造成的"假黑环"，这个差值掉到 9 ——
    #   如果门槛写成 +12，就会"打中反而检不到"，云台于是锁一下丢一下地摆。
    #   误检（柜门/阴影块）的差值是 -19~-27，所以 -10 既挡得住误检，
    #   又不会在打中时把真靶纸拒掉。
    # ⚠ 必须是【正】的小门槛：中心（白纸）必须比四周黑边亮这么多个灰度级。
    #   为什么：自适应阈值会把"大面积纯黑块"的**内部**也判成亮，于是实心黑块
    #   在二值图里看起来也是个"回字结构"（这个坑单元测试一直抓得到）。
    #   实测：实心黑块 中心-黑边 ≈ 0（拒），真靶纸 ≈ 60（收），
    #   激光打中靶心时因为光斑周围有一圈假黑环，差值掉到 ≈9 —— 所以门槛取 5：
    #   既挡住实心块，又不会在打中的那一刻把真靶纸拒掉。
    band_gap_min: float = 5.0
    # 靶纸到相机的合理距离范围（米）：E 题就在 0.5~1.6m，留点余量。
    # 现场实测有把远处柜门（解出 2.67m）锁成靶纸的情况，这条能直接挡掉。
    dist_min_m: float = 0.30
    dist_max_m: float = 2.00
    # 跟踪态"快通道"的 ROI 半径（像素）。**默认 0 = 关闭**，原因见下。
    # ROI 确实能把 LOCK 段帧率从 5.8fps 提到 14.9fps（实测），但它有一个
    # 致命副作用：靶心一旦漂到裁剪框边上，ROI 里检出的是"被裁掉一半的靶纸"，
    # 中心会偏（实测偏 70px 以上，而自检不一定拦得住），环路就按错的误差走、
    # 越走越偏，最后被判丢靶。
    # 现场 A/B（2026-09-29）：ROI 开 -> 反复 LOCK/LOST；ROI 关 -> 0 次丢靶、
    # 误差中位 0.00px。所以默认关掉，等把"裁剪自检"做扎实再打开。
    roi_track_px: float = 0.0
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
            max_candidates=_int(d, "max_candidates", 8),
            ring_tol_mm=_flt(d, "ring_tol_mm", 2.5),
            ring_structure_required=_bool(d, "ring_structure_required", True),
            ring_ratio_min=_flt(d, "ring_ratio_min", 0.45),
            ring_ratio_max=_flt(d, "ring_ratio_max", 0.95),
            loose_fallback=_bool(d, "loose_fallback", False),
            adaptive_block=_int(d, "adaptive_block", 61),
            adaptive_c=_int(d, "adaptive_c", 8),
            use_otsu=_bool(d, "use_otsu", False),
            binarize_fallback=_bool(d, "binarize_fallback", True),
            fallback_block=_int(d, "fallback_block", 31),
            fallback_c=_flt(d, "fallback_c", 4.0),
            binarize_mode=_str(d, "binarize_mode", "adaptive"),
            fixed_thresh=_int(d, "fixed_thresh", 45),
            close_k=_int(d, "close_k", 5),
            open_k=_int(d, "open_k", 1),
            gauss_k=_int(d, "gauss_k", 5),
            use_canny=_bool(d, "use_canny", False),
            canny_lo=_int(d, "canny_lo", 50),
            canny_hi=_int(d, "canny_hi", 150),
            angle_tol_deg=_flt(d, "angle_tol_deg", 30.0),
            side_ratio_tol=_flt(d, "side_ratio_tol", 0.45),
            min_area_abs=_int(d, "min_area_abs", 400),
            min_perimeter=_int(d, "min_perimeter", 60),
            nested_ratio_min=_flt(d, "nested_ratio_min", 0.55),
            nested_ratio_max=_flt(d, "nested_ratio_max", 0.92),
            nested_ratio_expect=_flt(d, "nested_ratio_expect", 0.74),
            nested_ratio_tol=_flt(d, "nested_ratio_tol", 0.22),
            single_rect_max_conf=_flt(d, "single_rect_max_conf", 0.44),
            require_red=_bool(d, "require_red", False),
            min_ring_score=_flt(d, "min_ring_score", 0.45),
            min_confidence=_flt(d, "min_confidence", 0.50),
            jump_gate_px=_flt(d, "jump_gate_px", 120.0),
            tape_min_ratio=_flt(d, "tape_min_ratio", 0.40),
            tape_max_ratio=_flt(d, "tape_max_ratio", 1.80),
            tape_aniso_max=_flt(d, "tape_aniso_max", 2.0),
            band_gap_min=_flt(d, "band_gap_min", 5.0),
            dist_min_m=_flt(d, "dist_min_m", 0.30),
            dist_max_m=_flt(d, "dist_max_m", 2.00),
            roi_track_px=_flt(d, "roi_track_px", 0.0),
            dark_thresh=_int(d, "dark_thresh", 100),
        )


@dataclass
class LaserConfig:
    # 判据模式：
    #   blue   —— advantage = B - max(G,R)，"蓝色占优"
    #             （适合光斑没被打亮成白芯、颜色还是深蓝紫的情况）
    #   violet —— advantage = min(B,R) - G，配合"足够亮"条件，
    #             "白芯 + 淡紫/品红边"
    # 实测（405nm 打白纸 + 这颗模组）：光斑是 BGR≈(89,43,150) 的品红，
    # 蓝优势是**负的**，所以必须用 violet，否则永远检不到真光斑、
    # 只会检到窗边/衣物的蓝色色散伪影。
    mode: str = "blue"
    b_min: int = 150
    b_minus_others: int = 35
    # violet 模式的两个阈值
    violet_gap: int = 12      # min(B,R) - G 至少这么大才算"偏紫"
    bright_min: int = 120     # min(B,G,R) 至少这么亮（排掉暗的色散伪影）
    min_area: int = 8
    max_area_ratio: float = 0.02
    # 光斑 ROI：>0 且已知光轴点时，只在这一小块里做**全分辨率**搜索。
    # 为什么需要：光斑只有几个像素，process_scale=0.5 会把它糊掉/碎掉；
    # 而全图全分辨率又太慢。ROI 同时解决"精度"和"速度"。
    roi_px: int = 0
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
            mode=_str(d, "mode", "blue"),
            b_min=_int(d, "b_min", 150),
            b_minus_others=_int(d, "b_minus_others", 35),
            violet_gap=_int(d, "violet_gap", 12),
            bright_min=_int(d, "bright_min", 120),
            min_area=_int(d, "min_area", 8),
            max_area_ratio=_flt(d, "max_area_ratio", 0.02),
            roi_px=_int(d, "roi_px", 0),
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
    # kp = **积分**增益（I）。2026-09-30 起主控制量是 kp_p 的比例项，
    # 这里只负责消掉最后的静差，所以从 0.12 降到 0.03（用户建议"减小 I"）。
    kp: float = 0.02
    # 大误差档的增益：误差 > coarse_err_px 时用 kp_coarse，靠近靶心后自动
    # 退回 kp。目的是"从画面边缘一路拉回中间"这一段快一点（现场实测：
    # 一直用很小的 kp，从 370px 拉回中心要 8~9 秒，中途一漏检就被判丢靶）。
    kp_coarse: float = 0.06  # 误差很大时 I 放快一点（仍远小于 P）
    coarse_err_px: float = 150.0
    # ---- P + I 视觉伺服的两个旋钮（2026-09-30 新增）----
    # kp_p：**比例**增益 —— 一次把误差的多少比例直接转成偏置角度。
    #   0.55 表示"一次给 55%"，剩余靠下一帧补齐。想更快就加到 0.7~0.9；
    #   开始来回摆就说明加过头了（云台滞后吃掉了相位裕度）。
    #   （这就是用户说的"增大 K"。）
    kp_p: float = 0.75
    # 俯仰的比例增益（单独给）：俯仰环是电机编码器闭环、很跟手，P 给大了
    # 会把 10fps 采样下的像素噪声放大成"俯仰上下跳"。默认取偏航的 ~2/3。
    kp_p_pitch: float = 0.50
    # kp_d：**微分**增益（秒）。误差在快速缩小时给一点阻尼，抑制快到靶心时过冲。
    # 用户要求"增大 K、减小 I、D 也适当增大"，2026-09-30 从 0 开到 0.25。
    # 给太大（>0.6）会被检测噪声放大，表现为靶心附近抖。
    kp_d: float = 0.35
    # 俯仰的微分增益：**默认 0 = 不加**（见 aim.py 里的说明，加了会抖）。
    kp_d_pitch: float = 0.0
    # D 项限幅（度）：单帧误检/丢帧会造成误差跳变，限一下防止命令被甩出去。
    d_limit_deg: float = 3.0
    # i_limit_deg：积分量限幅。I 只负责消静差，限到 ±8° 就够，
    #   绝不能像以前那样让它一路积到 ±90°（那会把云台带跑）。
    # I 的限幅故意很小：P 已经给了 55% 的偏差，I 只需要补最后那点静差。
    # 限到 ±3° 就能保证"即使 I 顶满，也不会把偏置带偏"（实测把 8° 时
    # P+I 会在靶心附近多冲 20%）。
    i_limit_deg: float = 3.0
    # 积分"只在误差进到这个范围内才累加"（条件积分/抗风阻）。单位像素。
    i_band_px: float = 40.0
    # α-β 跟踪滤波的两条系数。α 越大越"信测量"（滞后小、噪声大）。
    # 2026-09-27 晚 现场实测：α=0.25 意味着滤波后位置要 4 帧才追上真实位置，
    # 在 5~8fps 下相当于给环路额外加了 ~1 秒纯滞后 —— 这是"越调越摆"的
    # 主要来源之一。检测本身已经很稳（±3px），所以把 α 提到 0.8、β 保持小值。
    track_alpha: float = 0.8
    track_beta: float = 0.05
    # "防超前"限幅：命令偏置最多只允许比【实测姿态】超前这么多度。
    # 现场逐帧数据：误差已到 -0.3px、命令停在 2.93° 不动，平台却还在追更早的
    # 指令（gz_yaw 2.32->3.17），误差从 0 冲到 +28px —— 这就是"到靶心又晃走"。
    lag_lead_deg: float = 1.2
    # 误差越大，允许的超前越多（偏航轴有静摩擦死区，指令太小平台根本不动）。
    # 现场实测：误差 100px 时命令只超前 1.2°，平台 0.7s 才走 0.34°，
    # 环路等于被卡死；放到 8° 上限后平台才肯动。
    lag_lead_max_deg: float = 8.0
    # "指令无效"保护：误差长时间不下降、偏置却已经动了不少 —— 说明下了指令
    # 但画面没反应（姿态漂移 / 机构卡住 / 没跟上）。此时冻结积分，别让它跑飞。
    stall_err_px: float = 60.0      # 误差大于它就进入观察
    stall_s: float = 2.5            # 观察多久
    stall_delta_deg: float = 5.0    # 这期间偏置动了这么多、误差却没改善 -> 冻结
    # 冻结最长持续多久（秒）。必须有这个"自动解冻"，否则会死在
    # "冻结 -> 不动 -> 误差不降 -> 一直冻结"里 —— 现场表现就是
    # 丢靶重新捕获以后再也收敛不了（这个坑就是用户报的现象）。
    stall_hold_s: float = 2.0
    rate_limit_dps: float = 220.0
    max_yaw_deg: float = 170.0
    # 扫描专用偏航限幅：单向扫一圈时偏置要能走到 360°（跟踪仍用 max_yaw_deg）。
    max_yaw_scan_deg: float = 400.0
    max_pitch_deg: float = 60.0
    deadband_px: float = 3.0
    # 偏航"命令 1° -> 画面里多少像素"的额外缩放。H723 目前按【昨天那版】的
    # 姿态积分（半小时率）运行：命令 1° 实际转 2°，所以这里要乘 2，
    # 视觉环的模型才和真实被控对象一致。等 H723 改回 1:1 时把它调回 1.0。
    yaw_gain_scale: float = 1.0
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
            kp=_flt(d, "kp", 0.02),
            kp_coarse=_flt(d, "kp_coarse", 0.06),
            coarse_err_px=_flt(d, "coarse_err_px", 150.0),
            kp_p=_flt(d, "kp_p", 0.75),
            kp_p_pitch=_flt(d, "kp_p_pitch", 0.50),
            kp_d=_flt(d, "kp_d", 0.35),
            kp_d_pitch=_flt(d, "kp_d_pitch", 0.0),
            d_limit_deg=_flt(d, "d_limit_deg", 3.0),
            i_limit_deg=_flt(d, "i_limit_deg", 3.0),
            i_band_px=_flt(d, "i_band_px", 40.0),
            track_alpha=_flt(d, "track_alpha", 0.8),
            track_beta=_flt(d, "track_beta", 0.05),
            lag_lead_deg=_flt(d, "lag_lead_deg", 1.2),
            lag_lead_max_deg=_flt(d, "lag_lead_max_deg", 8.0),
            stall_err_px=_flt(d, "stall_err_px", 60.0),
            stall_s=_flt(d, "stall_s", 2.5),
            stall_delta_deg=_flt(d, "stall_delta_deg", 5.0),
            stall_hold_s=_flt(d, "stall_hold_s", 2.0),
            rate_limit_dps=_flt(d, "rate_limit_dps", 220.0),
            max_yaw_deg=_flt(d, "max_yaw_deg", 170.0),
            max_yaw_scan_deg=_flt(d, "max_yaw_scan_deg", 400.0),
            max_pitch_deg=_flt(d, "max_pitch_deg", 60.0),
            deadband_px=_flt(d, "deadband_px", 3.0),
            yaw_gain_scale=_flt(d, "yaw_gain_scale", 1.0),
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
    # 扫描网格的中心（度）：云台基线看不到靶纸时，把整个扫描网格挪过去。
    # 例：--set acquire.center_yaw_deg=9 --set acquire.center_pitch_deg=13
    center_yaw_deg: float = 0.0
    center_pitch_deg: float = 0.0
    # 扫描时偏置的移动速度（°/s）：慢一点，别让画面糊着就"路过"了
    # 扫描过点速度（°/s）。2026-09-30 从 120 提到 200：单点 60° 只要 0.3s，
    # 每点"停稳+判定"合计约 0.5s，一层 7 个点只需 ~3.5s（原来 ~7s）。
    # 运动模糊不担心：到位后还要等 settle_ms 再取帧。
    # 用户要求"再减小速度"：200 -> 80°/s。单点 60° 约 0.75s，
    # 加上停稳判定每点约 1.2s，一圈 6 个点约 7s（慢一点但每点都拍得清楚）。
    scan_dps: float = 80.0
    # 扫描时的俯仰层（度，相对上电基准），**按从上到下的顺序扫**。
    # 为什么是这么一组：E 题里靶纸挂在小车【上方】（靶心离地 1.1~1.3m，
    # 相机在车上 0.4~0.6m），俯仰需求基本都在"往上抬"这一侧；
    # 先扫 +25/+10 命中率最高，0 和 -10 只是兜底。
    # 用户实测反馈"俯仰电机寻找时上下摆动太快"：现在俯仰只在换层时动一次，
    # 换层速度由 pitch_scan_dps 单独限速（很慢）。
    # ⚠ 2026-09-30 再改：原来第一层就是 +25°（抬头），实测首次锁定被拖到 35 秒
    #   —— 靶子基本就在水平线附近，先抬头 25° 白扫 7 个点。现在**从水平开始**，
    #   上下各留一点余量（用户要求"俯仰寻找范围继续缩小"）。
    #   正 = 抬头，负 = 低头。如果你的机构正方向相反，把这三个数整体取反即可。
    pitch_layers: List[float] = field(
        default_factory=lambda: [0.0, 10.0, -10.0])
    pitch_scan_dps: float = 35.0
    # 扫描点取帧前，要求【实际姿态】的角速度低于这个值（°/s）。
    # 只看命令到位是不够的：命令是瞬间到的，平台还在追（实测几百°/s），
    # 这时拍的帧全是运动模糊，靶纸一掠而过 -> 明明扫到了却判成没看到。
    settle_rate_dps: float = 8.0
    # 丢靶后的"就近细扫"（见 SweepPlanner.fine_points）：
    # 先以丢靶前的位置为中心 ±fine_span_deg、每格 fine_step_deg 慢慢找，
    # 找不到再退到大范围扫描。设 fine_span_deg=0 就退回"直接大范围扫描"。
    fine_span_deg: float = 16.0
    fine_step_deg: float = 8.0
    fine_pitch_deg: float = 8.0
    fine_scan_dps: float = 80.0
    fine_settle_ms: int = 60
    fine_dwell_ms: int = 90
    # 最近多久之内看到过靶纸，才认为"就近细扫有意义"（秒）
    fine_recent_s: float = 6.0
    # 连续几帧看到靶纸才算"找到"（1 = 看到就开打，最快但可能被误检带偏）
    confirm_frames: int = 2

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
            center_yaw_deg=_flt(d, "center_yaw_deg", 0.0),
            center_pitch_deg=_flt(d, "center_pitch_deg", 0.0),
            scan_dps=_flt(d, "scan_dps", 80.0),
            pitch_layers=[float(x) for x in (
                d.get("pitch_layers") or [0.0, 10.0, -10.0])],
            pitch_scan_dps=_flt(d, "pitch_scan_dps", 35.0),
            settle_rate_dps=_flt(d, "settle_rate_dps", 8.0),
            fine_span_deg=_flt(d, "fine_span_deg", 16.0),
            fine_step_deg=_flt(d, "fine_step_deg", 8.0),
            fine_pitch_deg=_flt(d, "fine_pitch_deg", 8.0),
            fine_scan_dps=_flt(d, "fine_scan_dps", 80.0),
            fine_settle_ms=_int(d, "fine_settle_ms", 60),
            fine_dwell_ms=_int(d, "fine_dwell_ms", 90),
            fine_recent_s=_flt(d, "fine_recent_s", 6.0),
            confirm_frames=_int(d, "confirm_frames", 2),
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
    # 周期性重发 MODE（秒）。H723 侧的 AIM 看门狗是 0.5s：一旦因任何原因
    # 降级到 STAB，它不会自己回来，必须由上位机重发。同值重发在固件里是空操作。
    mode_reassert_s: float = 0.5
    # 启动时给 H723 下发的"在线参数"（param_id, value）。
    # 为什么放在这里：俯仰位置环默认 KP=15/KI=45 会自激（实测俯仰电机角峰峰
    # 摆 115、逐次跳变 73，而 IMU 装在俯仰轴下方、遥测里俯仰一直"纹波不动"）。
    # 俯仰一直摆 -> 画面一直跳/糊 -> 靶纸完整在画面里也检不到、误差永远收不敛。
    # 固件默认值已经改成 8/20，但**没重烧固件的板子**靠这一条兜底：
    # 每次启动自动把俯仰增益压到安全预设。
    startup_params: List[Tuple[int, float]] = field(
        default_factory=lambda: [(0x05, 8.0), (0x06, 20.0)])

    @staticmethod
    def from_dict(d: Dict[str, Any], default_port: str, default_baud: int) -> "LinkConfig":
        d = d or {}
        return LinkConfig(
            enable=_bool(d, "enable", True),
            port=_str(d, "port", default_port),
            baud=_int(d, "baud", default_baud),
            tx_hz=_flt(d, "tx_hz", 100.0),
            telemetry_timeout_s=_flt(d, "telemetry_timeout_s", 0.5),
            mode_reassert_s=_flt(d, "mode_reassert_s", 0.5),
            startup_params=_param_list(d, "startup_params",
                                       [(0x05, 8.0), (0x06, 20.0)]),
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
