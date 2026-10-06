"""靶纸检测：黑胶带外框四边形 -> 透视矫正 -> 红圈质心精修 -> 靶心。

为什么主检测目标是黑框而不是靶心红点：
    靶心直径 <=1mm，在 1.5m 处只有约半个像素，根本不可能稳定检出；
    而 18mm 宽的黑胶带外框在同样距离下面积占比 15% 以上、对比度极高。
    两者中心重合，所以"找黑框 -> 求中心"是最稳的路。

为什么还要红圈精修：
    黑框中心受边缘量化误差限制；而 5 个同心红圈的几何分布关于靶心四重对称，
    即使最外圈（r=10cm）被黑胶带遮住一部分（左右对称地遮），
    红色像素的质心仍严格落在靶心上，且亚像素精度。

最后的"环覆盖度打分"是一个极强的校验器：只有在矫正视图里
真的存在半径 2/4/6/8/10cm 的五条红圈，才认为这是 E 题的靶纸。
场地里那条 1m x 1m 的黑色巡线方框会被这一步干净地排除掉。
"""

import copy
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .config import TargetConfig
from .geometry import (CameraModel, Rectifier, build_rectifier, homography_pose,
                       is_convex_quad, order_quad, quad_area, quad_mean_size)

__all__ = ["TargetResult", "TargetDetector"]

# 赛题规定：靶心为中心，红圈半径 2/4/6/8/10 cm
RING_RADII_MM = (20.0, 40.0, 60.0, 80.0, 100.0)


@dataclass
class TargetResult:
    """一次靶纸检测的结果。"""

    uv: Tuple[float, float]          # 靶心像素坐标（已经过去畸变）
    uv_raw: Tuple[float, float]      # 靶心像素坐标（原始，未去畸变）
    quad: np.ndarray                 # 4x2 黑框外角点（原始图像素）
    rect: Optional[Rectifier]        # 靶面mm <-> 图像像素（兜底检测时为 None）
    distance_m: float = 0.0
    confidence: float = 0.0
    source: str = "quad"             # quad | red
    red_px: int = 0
    ring_score: float = 0.0
    refine_px: float = 0.0           # 红圈质心相对黑框中心的偏移（矫正图像素）
    tape_mm: float = 0.0             # 量出来的黑胶带厚度（mm）
    tape_score: float = 0.0          # 0~1，1 表示厚度刚好等于标称值

    def describe(self) -> str:
        return ("%s uv=(%.1f,%.1f) d=%.2fm conf=%.2f red=%d ring=%.2f refine=%.1fpx"
                " tape=%.1fmm/%.2f"
                % (self.source, self.uv[0], self.uv[1], self.distance_m,
                   self.confidence, self.red_px, self.ring_score, self.refine_px,
                   self.tape_mm, self.tape_score))


class _RingGeometry:
    """按矫正图尺寸缓存的"半径分箱表"，用于一次性算出径向直方图。"""

    __slots__ = ("size", "bins", "nbins", "mm_per_px", "expected_area", "tol_px")

    def __init__(self, size: Tuple[int, int], mm_per_px: float, tol_mm: float,
                 paper_mm: Tuple[float, float], tape_mm: float, line_mm: float = 1.0):
        self.size = size
        self.mm_per_px = mm_per_px
        tol_px = max(1.0, tol_mm / mm_per_px)
        h, w = size[1], size[0]
        cx, cy = (w - 1) * 0.5, (h - 1) * 0.5
        ys, xs = np.mgrid[0:h, 0:w]
        r_px = np.sqrt((xs - cx) ** 2 + (ys - cy) ** 2)
        self.nbins = int(r_px.max() / tol_px) + 2
        self.bins = np.clip((r_px / tol_px).astype(np.int32), 0, self.nbins - 1)
        # 每个环"应该"有多少红色像素：2*pi*r_px*line_px（扣除被胶带遮住的部分）
        hw = paper_mm[0] * 0.5 - tape_mm
        hh = paper_mm[1] * 0.5 - tape_mm
        line_px = max(1.0, line_mm / mm_per_px)
        self.expected_area = []
        for r_mm in RING_RADII_MM:
            r_px = r_mm / mm_per_px
            vis = _visible_fraction(r_mm, hw, hh)
            self.expected_area.append(max(1.0, 2.0 * math.pi * r_px * line_px * vis))
        self.tol_px = tol_px

    def ring_coverage(self, red_mask: np.ndarray) -> Tuple[float, List[float]]:
        """返回 (总环得分, 每个环的覆盖度)。"""
        if red_mask.dtype != bool:
            red_bool = red_mask > 0
        else:
            red_bool = red_mask
        count = np.count_nonzero(red_bool)
        if count == 0:
            return 0.0, [0.0] * len(RING_RADII_MM)
        hist = np.bincount(self.bins[red_bool], minlength=self.nbins).astype(np.float64)
        covers = []
        for r_mm, expected in zip(RING_RADII_MM, self.expected_area):
            center_bin = int(round(r_mm / self.mm_per_px / self.tol_px))
            lo = max(0, center_bin - 1)
            hi = min(self.nbins, center_bin + 2)
            measured = float(hist[lo:hi].sum())
            covers.append(min(1.0, measured / expected))
        inner = covers[:-1]
        score = 0.7 * (sum(inner) / max(1, len(inner))) + 0.3 * covers[-1]
        return score, covers


def _visible_fraction(r_mm: float, hw: float, hh: float) -> float:
    """半径 r 的红圈有多少比例没被黑胶带盖住（对称覆盖，不影响质心）。"""
    if r_mm <= min(hw, hh):
        return 1.0
    theta = np.linspace(0.0, 2.0 * math.pi, 720, endpoint=False)
    x = np.abs(r_mm * np.cos(theta))
    y = np.abs(r_mm * np.sin(theta))
    return float(np.count_nonzero((x <= hw) & (y <= hh))) / len(theta)


_RING_CACHE = {}


def _ring_geometry(size, mm_per_px, tol_mm, paper_mm, tape_mm) -> _RingGeometry:
    key = (size, round(mm_per_px, 6), round(tol_mm, 3), paper_mm, tape_mm)
    geo = _RING_CACHE.get(key)
    if geo is None:
        geo = _RingGeometry(size, mm_per_px, tol_mm, paper_mm, tape_mm)
        if len(_RING_CACHE) > 8:
            _RING_CACHE.clear()
        _RING_CACHE[key] = geo
    return geo


_QUAD_EPS_LIST = (0.010, 0.015, 0.020, 0.030, 0.040, 0.055)


def best_quad_from_contour(cnt, eps_list=_QUAD_EPS_LIST, min_fill: float = 0.90):
    """把一条轮廓变成四边形，返回 (四点有序数组 or None, 是否用了外接矩形兜底)。

    为什么要逐档试 eps（借鉴国一那版的 optimize_quadrilateral）：
      斜视、运动模糊、失焦时，黑胶带的边缘是"圆角 + 锯齿"，单一 eps=0.02
      经常近似出 5~6 个点，候选于是被整条丢掉 —— 实测表现就是"靶纸明明在
      画面里却检不到"。改成从紧到松试一遍，取"面积和原轮廓最接近"的那个
      四边形，能救回大量边缘不干净的候选。

    全都近似不出 4 点时，再看这个轮廓是不是"一块矩形"（它填满了自己的
    最小外接旋转矩形 ≥ 90%），是的话就用 minAreaRect 的四角兜底。
    ⚠ 门槛为什么是 0.90 而不是 0.78：圆（比如靶纸上印的那些红圈、圆洞）
    只填满外接正方形的 π/4 = 78.5%，0.78 会把**圆**兜成正方形，
    于是"红圈—圆洞"也被当成一组"回字结构"，把真靶纸挤掉。
    实测（合成靶纸）0.78 时中心误差 5.6px，0.90 时回到 0.2px 以内。
    兜底出来的框一定是矩形，所以调用方要给它打折（见 _evaluate 里的
    box_penalty）—— 它只用来防"该看见却看不见"，不用来提精度。
    """
    if cnt is None or len(cnt) < 4:
        return None, False
    hull = cv2.convexHull(cnt)
    peri = cv2.arcLength(hull, True)
    if peri <= 1.0:
        return None, False
    area = float(cv2.contourArea(cnt))
    best, best_score = None, -1e9
    for eps in eps_list:
        ap = cv2.approxPolyDP(hull, float(eps) * peri, True)
        if len(ap) != 4 or not is_convex_quad(ap):
            continue
        ratio = float(cv2.contourArea(ap)) / max(1.0, area)
        score = -abs(1.0 - ratio)          # 越接近 1 越贴合原轮廓
        if score > best_score:
            best_score, best = score, ap
    if best is not None:
        return order_quad(best.reshape(4, 2).astype(np.float64)), False
    rect = cv2.minAreaRect(cnt)
    bw, bh = float(rect[1][0]), float(rect[1][1])
    box_area = bw * bh
    if box_area > 1.0 and area / box_area >= float(min_fill):
        box = cv2.boxPoints(rect)
        if is_convex_quad(box):
            return order_quad(box.astype(np.float64)), True
    return None, False


def measure_tape_mm(warp: np.ndarray, rect: Rectifier,
                    dark_thresh: int = 100):
    """量"黑胶带"在矫正图里的厚度，**横竖分别量**，返回 (tx_mm, ty_mm)。

    做法：沿水平中心线从左右两边往中间走、沿垂直中心线从上下两边往中间走，
    数连续为"暗"的像素个数；横向取左右平均，纵向取上下平均。

    为什么用这个当主判据（而不是红圈）：
      红圈线宽只有 1mm，0.8m 处成像不到 1 个像素，稍一失焦就检不到；
      而黑胶带 18mm 宽（0.8m 处约 14 像素），糊了也能量出厚度。

    为什么必须【横竖分开量】：
      四边形被强行矫正成 A4 时，如果它的长宽比本来就不是 1.414，
      横竖两个方向的缩放比例就会不同 —— 同一个 18mm 的带子会被量成
      两个差很多的数。场地里那条 1m 巡线黑方框正是这种情况：
      横向量出约 6mm、纵向量出约 15mm，横竖一对比就露馅了。
      真靶纸是标准 A4 比例，横竖量出来基本相等。

    某一边量不出来（被画面边缘切掉 / 被激光光斑打断 / 反光过曝）时**不用它**，
    只要同一方向还剩一边就能给个估计；某一方向两边都量不出来就返回 0。
    返回 (0.0, 0.0) 表示两个方向都量不出来。

    为什么要放宽这一点：实测现场靶纸漂到画面边缘时，外侧那一条胶带被裁掉，
    原来"任何一边量不出来就整体放弃"的写法让整张靶纸直接检不到 ——
    而这时候恰恰最需要它把靶心拉回画面中间。
    """
    gray = cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY) if warp.ndim == 3 else warp
    h, w = gray.shape[:2]
    if h < 16 or w < 16:
        return 0.0, 0.0
    g = gray.astype(np.float32)
    cy, cx = h // 2, w // 2
    row = g[cy]                      # 水平中心线
    col = g[:, cx]                   # 垂直中心线
    xs = [_edge_crossing(row), _edge_crossing(row[::-1])]
    ys = [_edge_crossing(col), _edge_crossing(col[::-1])]
    vx = [v for v in xs if v > 0.0]
    vy = [v for v in ys if v > 0.0]
    tx = (sum(vx) / len(vx)) * rect.mm_per_px if vx else 0.0
    ty = (sum(vy) / len(vy)) * rect.mm_per_px if vy else 0.0
    return float(tx), float(ty)


def _band_contrast(warp: np.ndarray, rect: Rectifier, tape_mm_ref: float = 0.0,
                   min_gap: float = -10.0):
    """检查"黑边围着的一张亮纸"：返回 (是否通过, 中心与黑边的灰度差)。

    靶纸的物理样子就是"一圈黑胶带 + 中间一张白纸"。
    现场实测（2026-09-27 晚）：真靶纸中心区灰度约 77、黑胶带约 15 —— 差 60 多。

    ⚠ 判据只写成"中心不能比黑边【暗】太多"（min_gap = -10），不写"必须亮多少"。
      为什么（这一版是被现场打脸改出来的）：激光正好打中靶心时，画面中心会出现
      一个很亮的斑，自适应阈值会在光斑周围造出一圈"假黑环"，把这条判据的
      差值压到只有 9 —— 于是**打中的时候反而检不到靶纸**，云台就开始
      "锁上→丢靶→重扫→又锁上"地来回摆。而这恰恰是用户看到的现象。
      反过来，柜门、阴影块这些误检的差值是 -19~-27（中心比边暗），
      所以门槛取 -10 既挡得住误检，又不会在打中时把真靶纸拒掉。
    """
    gray = cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY) if warp.ndim == 3 else warp
    h, w = gray.shape[:2]
    if h < 24 or w < 24:
        return True, 0.0
    band = 3
    if tape_mm_ref and tape_mm_ref > 0.0:
        band = int(round(tape_mm_ref / max(1e-6, rect.mm_per_px)))
    band = int(max(3, min(band, round(0.18 * min(h, w)))))
    mask = np.zeros((h, w), dtype=bool)
    mask[:band, :] = True
    mask[h - band:, :] = True
    mask[:, :band] = True
    mask[:, w - band:] = True
    band_med = float(np.median(gray[mask]))
    core = gray[int(h * 0.28):int(h * 0.72), int(w * 0.28):int(w * 0.72)]
    if core.size < 16:
        return True, 0.0
    core_med = float(np.median(core))
    gap = core_med - band_med
    return bool(gap >= min_gap), float(gap)


def _edge_crossing(profile: np.ndarray) -> float:
    """从 profile 的起始端往中间走，找出"由暗转亮"的交叉位置（带亚像素插值）。

    为什么不用固定阈值/全局 Otsu：
      目标远或失焦时，18mm 的黑胶带会被模糊成"灰带"，灰度值飘忽不定，
      任何固定阈值都会时灵时不灵。而"从暗到亮的那个交叉点"是几何位置，
      受模糊影响小得多（模糊只是让过渡变缓，交叉点仍在中间）。

    做法：取起始段的暗电平 lo、取中段的亮电平 hi，
    在 (lo+hi)/2 处找第一个上穿点，线性插值到亚像素。
    返回 0 表示对比度太低、量不出来。
    """
    n = int(len(profile))
    if n < 8:
        return 0.0
    head = profile[: max(2, n // 4)]
    mid = profile[n // 4: max(n // 4 + 2, n // 2)]
    lo = float(np.min(head))
    hi = float(np.max(mid))
    if (hi - lo) < 15.0:            # 对比度不足（比如整张都糊了）→ 量不出来
        return 0.0
    thr = 0.5 * (lo + hi)
    for i in range(1, n):
        if (profile[i - 1] < thr) and (profile[i] >= thr):
            d = float(profile[i] - profile[i - 1])
            return float(i - 1) + (thr - float(profile[i - 1])) / max(1e-6, d)
    return 0.0


class TargetDetector:
    """靶纸检测器（无状态，可多帧复用）。"""

    def __init__(self, cfg: TargetConfig, cam: CameraModel):
        self.cfg = cfg
        self.cam = cam
        # ---- "宽松档"判据（严格档全被拒时用，借鉴国一那版的多级兜底）----
        # 斜视/远距/胶带贴歪时，"长宽比""胶带厚度"这些**透视敏感**的量都会明显
        # 偏离标称值，严格档会把整张靶纸拒掉 —— 实测表现就是"靶子在画面里却
        # 连续 1 秒检不出 -> 状态机判丢靶 -> 扫描大范围甩动"。
        self.cfg_loose = copy.copy(cfg)
        self.cfg_loose.aspect_tol = max(0.60, float(cfg.aspect_tol))
        # ⚠ 胶带厚度这一条即使在宽松档也不能放太松：它是把"靶纸"和
        # "实心黑方块（柜门/纸箱）"分开的主力判据（实测放宽到 0.12 时，
        # 实心黑方块会被误判成靶纸 —— 单元测试直接抓到了）。
        # 下界放宽（斜视/远距量出来的胶带会偏薄），上界反而收紧：
        # 实心黑方块量出来的"胶带"特别厚（≥1.8），上界放宽会把它放进来。
        self.cfg_loose.tape_min_ratio = min(0.15, float(cfg.tape_min_ratio))
        self.cfg_loose.tape_max_ratio = min(1.60, float(cfg.tape_max_ratio))
        self.cfg_loose.tape_aniso_max = max(2.5, float(
            getattr(cfg, "tape_aniso_max", 2.0)))
        # "回"字结构这一条要【继承严格档】：它是把靶纸和实心黑方块分开的关键，
        # 宽松档只是把"内窗面积比"的容差放宽（手工胶带/斜视时比例会偏）。
        self.cfg_loose.ring_structure_required = bool(cfg.ring_structure_required)
        self.cfg_loose.ring_ratio_min = min(0.30, float(cfg.ring_ratio_min))
        self.cfg_loose.ring_ratio_max = max(0.98, float(cfg.ring_ratio_max))
        self.cfg_loose.require_red = False
        # 面积法测距的参考对 [距离m, 面积px]：位姿解有效时自动更新（见 _evaluate）
        self._dist_ref = None
        self._rings = {}
        self.last_candidates = 0
        # 诊断信息：每帧 detect() 重置。tools/diag_target.py 会把它打出来，
        # 用来回答"靶纸明明在画面里，到底是哪一条判据把它拒掉的"。
        self.diag = {}

    # ------------------------------------------------------------------
    def _diag(self, key: str, value=None) -> None:
        """往诊断字典里记一笔（value 为 None 时表示计数 +1）。"""
        if value is None:
            self.diag[key] = self.diag.get(key, 0) + 1
        else:
            self.diag[key] = value

    def _diag_reject(self, reason: str, **kw) -> None:
        item = {"why": reason}
        item.update(kw)
        self.diag.setdefault("reject", []).append(item)
        if len(self.diag["reject"]) < 4:
            return                       # 只留前面几条，避免爆内存
        self.diag["reject"] = self.diag["reject"][:4]

    # ------------------------------------------------------------------
    def detect(self, frame: np.ndarray, scale: float = 1.0,
               spot: Optional[Tuple[float, float, float]] = None,
               roi: Optional[Tuple[float, float, float]] = None
               ) -> Optional[TargetResult]:
        """找靶纸。

        scale < 1 时：先在缩小图上做"二值化 + 找四边形"（这一步最吃 CPU），
        找到的四边形再乘回原图坐标；后面的透视矫正、红圈亚像素质心
        仍然在原图上做。这样省时间，而且【不会损失瞄准精度】——
        因为精度来自矫正图上的质心，跟这一步的缩放无关。

        spot: 激光光斑 (u, v, 半径) —— 用来在二值化后把光斑"补"回去。
        为什么需要它：激光打在黑胶带上会在带子上烧出一个亮洞，胶带环于是断开，
        整张靶纸检测失败（国一那版是靠 30x30 的大闭运算硬桥接，我们改成
        精确修补：只有光斑周围确实是暗的时候才把它当胶带补上）。
        现场实测：靶心在 u≈500 时，光斑正好落在右侧胶带上，检测会连续丢 1 秒，
        云台于是"锁一下、丢一下、来回摆"。

        roi: (中心u, 中心v, 半径) —— **跟踪时的快通道**。锁定之后靶心就在
        已知位置附近，全图 1280x720 的二值化+找轮廓纯属浪费：裁一块
        正方形去跑同样的流程，单帧耗时能从 ~130ms 降到 ~40ms（现场实测
        帧率 5fps -> 15fps 量级），视觉环的延时也就跟着降下来了。
        ROI 里什么都没检到时**自动退回全图**再找一遍，所以不会因为
        "靶纸跑出 ROI"而丢靶。
        """
        h, w = frame.shape[:2]
        img_area = float(h * w)
        self.diag = {"scale": float(scale), "reject": []}
        # ---- 跟踪快通道：先裁 ROI，坐标最后再挪回全图 ----
        # 注意 img_area 始终用【全图】面积：面积阈值（min/max_area_ratio）
        # 是按全图定义的，用裁剪图面积会让阈值悄悄变严。
        x0 = y0 = 0
        proc = frame
        if roi is not None:
            rcx, rcy, rr = float(roi[0]), float(roi[1]), float(roi[2])
            margin = 40
            bx0 = max(0, int(rcx - rr) - margin)
            by0 = max(0, int(rcy - rr) - margin)
            bx1 = min(w, int(rcx + rr) + margin)
            by1 = min(h, int(rcy + rr) + margin)
            if (bx1 - bx0) >= 200 and (by1 - by0) >= 200:
                proc = frame[by0:by1, bx0:bx1]
                x0, y0 = bx0, by0
        # ⚠ 存"裁剪图坐标系"下的光斑：后面的 rect / 单应矩阵都是按裁剪图建的，
        #   两边坐标系必须一致（否则光斑补洞/抹除会画到别的地方去）。
        self._spot_uv = None
        if spot is not None:
            self._spot_uv = (float(spot[0]) - x0, float(spot[1]) - y0,
                             float(spot[2]))
        ph, pw = proc.shape[:2]
        if scale and abs(scale - 1.0) > 1e-6 and scale > 0.05:
            sw = max(16, int(round(pw * scale)))
            sh = max(16, int(round(ph * scale)))
            small = cv2.resize(proc, (sw, sh), interpolation=cv2.INTER_AREA)
            inv = 1.0 / scale
        else:
            small = proc
            inv = 1.0
        # ⚠ 这里必须用【缩小图】的面积算阈值！
        #   之前误用了原图面积：缩小图的轮廓面积小 4 倍，而面积阈值没变，
        #   结果候选四边形全被滤掉 —— 表现为"缩放检测一个都找不到"。
        small_area = float(small.shape[0] * small.shape[1])
        # ---- 两级判决：先按"现场配置"判，全被拒就用"宽松配置"再判一遍 ----
        # 借鉴国一那版的多级兜底思路：他们的检测失败后会退到更松的策略，
        # 而不是直接返回 None。我们实测最痛的就是"靶纸在画面里、却连续 1 秒
        # 什么都检不出 -> 状态机判丢靶 -> 扫描大范围甩动"。
        saved_cfg = self.cfg
        try_cfgs = (saved_cfg, self.cfg_loose) if getattr(
            saved_cfg, "loose_fallback", False) else (saved_cfg,)
        # ---- 两级二值化：严格档 + "远距离/小目标"档 ----
        # 实测（tools/tune_detect.py 网格搜索）：同一张现场照片合成到 1.5m 大小后，
        # block=61/c=8 只能检出 7/18 个斜视场景，而 block=31/c=4 能检出 14/18 ——
        # 因为大窗口的局部均值会被亮靶纸拉高，细黑胶带的对比度被吃掉，环断掉。
        # 但小窗口在近处（胶带占 20~40px 宽）又会把带子内部判成亮，所以不能
        # 直接换掉。做法：严格档先判，全被拒再用小窗口档重来一遍。
        # 每一档 = (模式, 自适应窗口, 自适应偏移)。
        # ⚠ 这里必须显式带模式：`_binarize` 只要收到 block 参数就会走自适应分支，
        #   所以"固定阈值档"必须传 block=None，否则国一那套固定阈值根本用不上
        #   （实测就是这样：检测一直在用自适应阈值，把真靶纸的内窗判丢了）。
        if str(getattr(saved_cfg, "binarize_mode", "fixed")).lower() == "adaptive":
            variants = [("adaptive", int(saved_cfg.adaptive_block),
                         float(saved_cfg.adaptive_c))]
        else:
            variants = [("fixed", None, None)]
        if getattr(saved_cfg, "binarize_fallback", True):
            variants.append(("adaptive",
                             int(getattr(saved_cfg, "fallback_block", 31)),
                             float(getattr(saved_cfg, "fallback_c", 4.0))))
        best: Optional[TargetResult] = None
        n_cands = 0
        for vi, (bmode, block, c_off) in enumerate(variants):
            spot_local = None
            if spot is not None:
                spot_local = (float(spot[0]) - x0, float(spot[1]) - y0,
                              float(spot[2]))
            if bmode == "adaptive":
                binary = self._binarize(small, scale, block=block, c_off=c_off,
                                        spot=spot_local)
            else:
                binary = self._binarize(small, scale, spot=spot_local)
            # 第二档（兜底）只取前几个候选：它本来就是"严格档什么都没捞到"时
            # 才跑，候选给太多会把单帧耗时从 60ms 拉到 180ms（实测），
            # 而相机只有 9fps，宁可少看几个候选。
            cap = self.cfg.max_candidates if vi == 0 else min(
                4, int(self.cfg.max_candidates))
            quads = self._find_quads(binary, small_area, cap=cap)
            if inv != 1.0:
                quads = [(q * inv, (qi * inv if qi is not None else None),
                          a * inv * inv, cl, bx, ib, rt)
                         for q, qi, a, cl, bx, ib, rt in quads]
            n_cands += len(quads)
            for cfg_try in try_cfgs:
                self.cfg = cfg_try
                # 注意传 proc（裁剪图）：quads / 单应矩阵全都在这个坐标系里
                best = self._pick_best(proc, quads, img_area)
                if best is not None:
                    break
            if best is not None:
                break
        self.last_candidates = n_cands
        self.cfg = saved_cfg
        # ---- 最终硬门限：置信度不够就当没看到 ----
        # 放在整条"多档二值化 + 多档判据"流程的最后，而不是每一档里面。
        # 原因：如果在一档里面就按门限丢掉，程序会以为"这一档什么都没找到"
        # 而去试下一档，结果自适应档又把巡线黑方框之类的干扰配成"回字结构"
        # 报了出来（单元测试抓到的就是这个）。现在只要某一档给出了候选，
        # 就以它为准，低于门限就是不认。
        if best is not None and best.confidence < float(self.cfg.min_confidence):
            best = None
        if best is not None:
            if x0 or y0:
                moved = _shift_result(best, x0, y0)
                # ROI 结果自检：靶心必须落在 ROI 中心附近。如果偏出去太多，
                # 说明真正的靶纸已经跑出裁剪范围、ROI 里这个是"被裁掉一半的
                # 靶纸/别的四边形"（实测会把中心带偏 70px 以上）—— 这种结果
                # 直接不要，退回全图重找。
                bad = roi is not None and math.hypot(
                    moved.uv[0] - float(roi[0]),
                    moved.uv[1] - float(roi[1])) > 0.75 * float(roi[2])
                # 还要查"四边形是不是贴到裁剪框边上"：贴边 = 靶纸被裁了一半，
                # 这种时候中心一定是偏的（实测偏 71px），必须丢。
                if not bad and moved.quad is not None:
                    qx = np.asarray(moved.quad, dtype=float)
                    if (qx[:, 0].min() <= x0 + 4 or qx[:, 0].max() >= x0 + pw - 4
                            or qx[:, 1].min() <= y0 + 4
                            or qx[:, 1].max() >= y0 + ph - 4):
                        bad = True
                if bad:
                    best = None
                else:
                    return moved
            else:
                return best
        # ROI 快通道没检到 -> 退回全图再找一遍（保证不漏）
        if ((x0 or y0) or (best is None and roi is not None)) and roi is not None:
            return self.detect(frame, scale, spot=spot, roi=None)
        if self.cfg.fallback_red and inv == 1.0:
            return self._detect_red_only(frame)      # 兜底不做缩放，直接用原图
        return None

    def _pick_best(self, frame, quads, img_area) -> Optional[TargetResult]:
        """在候选四边形里挑最好的一个（用当前 self.cfg 的判据）。"""
        best: Optional[TargetResult] = None
        for item in quads:
            quad, inner, area, clipped = item[0], item[1], item[2], item[3]
            box_used = bool(item[4]) if len(item) > 4 else False
            inner_box = bool(item[5]) if len(item) > 5 else False
            ratio = float(item[6]) if len(item) > 6 else 0.0
            # ---- "回"字结构：只做【加权】，不做硬门槛 ----
            # 踩过的坑：一开始我把它做成硬要求（没有内窗就不认），结果只要
            # 胶带环在二值图里断一点（距离变远/光线变化就会），**完整的靶纸
            # 也会被拒掉** —— 现场表现就是"靶子明明在画面里却识别不到"。
            # 现在改成：有"回"字结构的候选给一个强加权（下面 confidence 里乘
            # 系数），没有的仍然参与竞争。这样既保留"柜门/纸箱抢不过真靶纸"
            # 的好处，又不会漏掉真靶纸。
            # 单个候选算崩了（退化单应、数值异常）不能把整个程序带走 ——
            # 现场实测就是这么"自己中断"的。这里兜住，跳过这个候选继续找。
            try:
                res = self._evaluate(frame, quad, area, img_area, inner=inner,
                                     clipped=clipped, box_used=box_used,
                                     inner_box=inner_box, spot=self._spot_uv,
                                     ratio=ratio)
            except Exception as exc:                           # noqa: BLE001
                # 记下原因再跳过：以前这里是"静默吞掉"，现场只看到
                # "候选全崩(异常)"却不知道是什么异常，等于没法查。
                self._diag_reject("候选异常", exc=type(exc).__name__,
                                  msg=str(exc)[:48])
                res = None
            if res is None:
                continue
            if best is None or res.confidence > best.confidence:
                best = res
        return best

    # ------------------------------------------------------------------
    def _binarize(self, frame: np.ndarray, scale: float = 1.0,
                  block: Optional[int] = None,
                  c_off: Optional[float] = None,
                  spot: Optional[Tuple[float, float, float]] = None) -> np.ndarray:
        """预处理：把"黑胶带 + 纸边"变成干净的前景。

        全面借鉴国一那版（2026-09-29）：
          灰度 → 高斯模糊 → **固定阈值反二值化** → **大核闭运算** → **并上 Canny 边缘**
        这三步各有明确用途，正是我们之前缺的：
          * 固定阈值：只要"很黑"的像素（激光烧出来的白洞、反光、纸上的印花
            都不会被当成前景），比自适应阈值稳得多；
          * 大核闭运算：国一文档里写得很清楚 —— "激光照亮黑色边框造成的断裂 /
            桥接激光造成的间隙"。我们现场丢靶的最大原因就是光斑把胶带环烧断，
            这一条直接对症；
          * Canny 融合：边缘通道把"胶带—纸"的边界接起来，即使带子被切掉一段，
            轮廓仍然是闭合的。
        自适应阈值保留成第二档（binarize_mode=adaptive 或兜底档用）。
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gk = int(getattr(self.cfg, "gauss_k", 5)) | 1
        gray = cv2.GaussianBlur(gray, (max(3, gk), max(3, gk)), 0)

        mode = str(getattr(self.cfg, "binarize_mode", "fixed") or "fixed").lower()
        if mode == "adaptive" or block is not None:
            # ---- 自适应阈值档 ----
            if block is None:
                block = self.cfg.adaptive_block
            if c_off is None:
                c_off = self.cfg.adaptive_c
            # 窗口必须跟着缩放一起缩（缩小图上 blockSize=61 会比半个靶纸还大）
            if scale and abs(scale - 1.0) > 1e-6:
                block = int(round(block * scale))
            if block % 2 == 0:
                block += 1
            block = max(9, block)
            binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                           cv2.THRESH_BINARY_INV, block,
                                           float(c_off))
        else:
            # ---- 固定阈值档（国一：35）----
            _, binary = cv2.threshold(gray, int(getattr(self.cfg, "fixed_thresh", 45)),
                                      255, cv2.THRESH_BINARY_INV)

        # ---- 大核闭运算：桥接被激光/反光打断的边框 ----
        # ---- 开运算：先抹掉"细线"（靶纸上的红圈、Canny 的细边缘）----
        # 为什么必须先开运算再闭运算：
        #   靶纸上的 5 条红圈只有 1mm 宽，在二值图里会把中间那张白纸**切成好几块**，
        #   于是"外框套住的矩形"变成一堆碎片，配对配到碎片上，中心就偏几像素
        #   （合成图实测偏 5.6px）。开运算（腐蚀→膨胀）把比核更细的东西抹掉，
        #   而 18mm 的胶带远粗于核，完好无损。
        ok_k = int(getattr(self.cfg, "open_k", 5))
        if scale and abs(scale - 1.0) > 1e-6:
            ok_k = max(3, int(round(ok_k * scale)))
        if ok_k % 2 == 0:
            ok_k += 1
        if ok_k >= 3:
            okernel = cv2.getStructuringElement(cv2.MORPH_RECT, (ok_k, ok_k))
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, okernel, iterations=1)
        ck = int(getattr(self.cfg, "close_k", 9))
        if scale and abs(scale - 1.0) > 1e-6:
            ck = max(3, int(round(ck * scale)))
        if ck % 2 == 0:
            ck += 1
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (ck, ck))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)

        # ---- 并上 Canny 边缘（国一：combined = binary | edges）----
        if bool(getattr(self.cfg, "use_canny", True)):
            edges = cv2.Canny(gray, int(getattr(self.cfg, "canny_lo", 50)),
                              int(getattr(self.cfg, "canny_hi", 150)), apertureSize=3)
            if scale and abs(scale - 1.0) > 1e-6:
                # 边缘在缩小图上会变细，稍微膨一下，保证连通性
                edges = cv2.dilate(edges, np.ones((2, 2), np.uint8), iterations=1)
            binary = cv2.bitwise_or(binary, edges)

        if self.cfg.use_otsu:
            _, otsu = cv2.threshold(gray, 0, 255,
                                    cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
            binary = cv2.bitwise_or(binary, otsu)
        # ---- 激光光斑"补洞"：把打在黑胶带上的亮斑按胶带补回去 ----
        if spot is not None:
            su, sv, sr = float(spot[0]), float(spot[1]), float(spot[2])
            h, w = binary.shape[:2]
            x, y = int(round(su * scale)), int(round(sv * scale))
            if 0 <= x < w and 0 <= y < h:
                r = int(max(4.0, min(60.0, (sr * 1.3 + 3.0) * scale)))
                # 只在"光斑正落在暗东西上"时才补：取紧贴光斑外圈那一环的灰度
                # 中位数当判据。落在白纸上（中位数 ≥95）就不动它 —— 那时候
                # 补上去反而会在纸中间凭空造出一个黑块。
                y0, y1 = max(0, y - r - 6), min(h, y + r + 7)
                x0, x1 = max(0, x - r - 6), min(w, x + r + 7)
                if (y1 - y0) > 8 and (x1 - x0) > 8:
                    yy, xx = np.mgrid[y0:y1, x0:x1]
                    d2 = (xx - x) ** 2 + (yy - y) ** 2
                    ring = (d2 > (r + 1.5) ** 2) & (d2 < (r + 6.0) ** 2)
                    if ring.any():
                        med = float(np.median(gray[y0:y1, x0:x1][ring]))
                        if med < 95.0:
                            cv2.circle(binary, (x, y), r, 255, -1)
                            self._diag("spot_patch", 1)
        return binary

    def _find_quads(self, binary: np.ndarray, img_area: float, cap=None):
        """找候选四边形，并按国一的思路做"父子配对"。

        国一那版的关键结构（我们全面照搬）：
          1. `findContours(..., RETR_TREE, ...)` 保留完整层级；
          2. 面积/周长粗筛 → `approxPolyDP(0.02·周长)` 必须是四点
             （近似不出四点时用外接旋转矩形兜底）；
          3. `check_rectangle_geometry`：内角偏离 90° 不超过 30°、
             对边长度相对差不超过 0.45；
          4. 按面积从小到大排，找"父-子"配对，且 子/父 面积比 ≥ 0.7
             （我们放宽到 0.55：外框是胶带外沿、内窗是纸面，实测约 0.74）；
          5. 优先取"面积最小的那个配对" —— 那就是最里面的那张纸。

        靶纸 = 黑胶带外框 + 里面的白纸，天然就是一个"回"字。用这个结构
        判据挡掉柜门/纸箱/巡线黑方框，比任何颜色判据都硬。
        返回 (外框四边形, 内窗四边形或 None, 面积, 是否贴边, 外接矩形兜底?, 内窗兜底?)
        """
        contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)
        self.diag["n_contours"] = len(contours)
        cands = []
        if hierarchy is None or not contours:
            return cands
        hier = hierarchy[0]
        cfg = self.cfg
        lo = float(cfg.min_area_ratio) * img_area
        hi = float(cfg.max_area_ratio) * img_area
        lo_abs = float(getattr(cfg, "min_area_abs", 400))
        min_peri = float(getattr(cfg, "min_perimeter", 60))
        n_area_rej = n_poly_rej = n_inside_rej = n_geom_rej = 0
        holes = []
        rects = []                      # 每个元素: dict
        for idx, cnt in enumerate(contours):
            area = float(cv2.contourArea(cnt))
            if area < max(lo, lo_abs) or area > hi:
                n_area_rej += 1
                continue
            peri = float(cv2.arcLength(cnt, True))
            if peri < min_peri:
                n_inside_rej += 1
                continue
            quad, box_used = best_quad_from_contour(cnt, eps_list=self._eps_list())
            if quad is None:
                n_poly_rej += 1
                continue
            wq, hq = quad_mean_size(quad)
            if min(wq, hq) < 12:
                n_inside_rej += 1
                continue
            if not self._quad_geometry_ok(quad):
                n_geom_rej += 1
                continue
            rects.append({"idx": idx, "quad": quad, "area": area,
                          "parent": int(hier[idx][3]),
                          "child": int(hier[idx][2]), "box": box_used})
        # ---- 父子配对（国一：面积比 ≥ MIN_AREA_RATIO，取面积最小的那个配对）----
        by_idx = {r["idx"]: r for r in rects}
        ratio_min = float(getattr(cfg, "nested_ratio_min", 0.55))
        ratio_max = float(getattr(cfg, "nested_ratio_max", 0.92))
        holes = []
        pairs = []
        # ⚠ 配对用【几何包含】而不是只看轮廓层级。
        #   为什么：把 Canny 边缘并进前景之后，轮廓层级会多出一两层
        #   （边缘线自己就是一个闭合环），"直接父轮廓"经常不是我们要的那个；
        #   而"谁被谁套住"是纯几何事实，稳得多。
        #   对每个外框，取"被它套住的最大矩形"当内窗 —— 那就是中间那张纸。
        for a in rects:
            best_inner = None
            for bb in rects:
                if bb is a or bb["area"] >= a["area"]:
                    continue
                ratio = bb["area"] / max(1.0, a["area"])
                if not (ratio_min <= ratio <= ratio_max):
                    continue
                if not _quad_inside(bb["quad"], a["quad"]):
                    continue
                if best_inner is None or bb["area"] > best_inner["area"]:
                    best_inner = bb
            if best_inner is not None:
                ratio = best_inner["area"] / max(1.0, a["area"])
                holes.append(round(float(ratio), 3))
                pairs.append((a, best_inner, ratio))
        used = set()
        # ⚠ 排序按【外框面积从大到小】，不是按内窗。
        #   实测踩过：靶纸上印的红圈本身也是"环带+洞"，会产出一堆小配对；
        #   按内窗面积升序排时，真靶纸（外框最大、内窗也大）被排到最后，
        #   被 max_candidates 截掉 —— 于是检出来的是红圈那种小框，中心偏几像素。
        for p, ch, ratio in sorted(pairs, key=lambda t: -t[0]["area"]):
            used.add(p["idx"])
            used.add(ch["idx"])
            cands.append((p["quad"], ch["quad"], float(p["area"]),
                          self._is_clipped(p["quad"], binary.shape), p["box"],
                          ch["box"], float(ratio)))
        # 剩下的单个四边形也进候选表（置信度会被压到 single_rect_max_conf 以下）
        for r in sorted(rects, key=lambda t: -t["area"]):
            if r["idx"] in used:
                continue
            cands.append((r["quad"], None, float(r["area"]),
                          self._is_clipped(r["quad"], binary.shape), r["box"],
                          False, 0.0))
        self.diag["n_area_rej"] = n_area_rej
        self.diag["n_poly_rej"] = n_poly_rej
        self.diag["n_small_rej"] = n_inside_rej
        self.diag["n_geom_rej"] = n_geom_rej
        self.diag["holes"] = holes
        self.diag["n_cands"] = len(cands)
        limit = int(self.cfg.max_candidates) if cap is None else int(cap)
        return cands[: max(1, limit)]

    def _quad_geometry_ok(self, quad) -> bool:
        """国一的 check_rectangle_geometry：内角接近 90°、对边长度接近。"""
        q = np.asarray(quad, dtype=np.float64).reshape(4, 2)
        tol_a = float(getattr(self.cfg, "angle_tol_deg", 30.0))
        tol_s = float(getattr(self.cfg, "side_ratio_tol", 0.45))
        for i in range(4):
            a, b, c = q[i - 1], q[i], q[(i + 1) % 4]
            v1, v2 = a - b, c - b
            n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
            if n1 < 1e-6 or n2 < 1e-6:
                return False
            cosang = float(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0))
            if abs(math.degrees(math.acos(cosang)) - 90.0) > tol_a:
                return False
        for i in range(2):
            s1 = float(np.linalg.norm(q[i] - q[(i + 1) % 4]))
            s2 = float(np.linalg.norm(q[i + 2] - q[(i + 3) % 4]))
            if abs(s1 - s2) / max(1.0, max(s1, s2)) > tol_s:
                return False
        return True

    @staticmethod
    def _is_clipped(quad, shape) -> bool:
        h, w = shape[:2]
        xs = np.asarray(quad)[:, 0]
        ys = np.asarray(quad)[:, 1]
        return bool(xs.min() <= 1.5 or ys.min() <= 1.5 or
                    xs.max() >= (w - 2.5) or ys.max() >= (h - 2.5))

    def _eps_list(self):
        """轮廓近似的 eps 档位：以配置值为起点，再补几档更松的。"""
        base = float(self.cfg.poly_eps_ratio)
        out = [base]
        for eps in _QUAD_EPS_LIST:
            if eps > base + 1e-6:
                out.append(float(eps))
        return tuple(out)

    # ------------------------------------------------------------------
    def _evaluate(self, frame, quad, area, img_area,
                  inner=None, clipped=False, box_used=False,
                  inner_box=False, spot=None,
                  ratio: float = 0.0) -> Optional[TargetResult]:
        cfg = self.cfg
        # ---- 先做"零成本"的形状预筛，再花钱做透视矫正 ----
        # 每个候选要花一次 warpPerspective + 量测（约 4ms）。实测单帧 180ms 里
        # 大头就是这个循环，而绝大多数候选（细长条、怪形状）根本不用看内容就能
        # 排除掉。这里的门限故意开得很宽（0.55~2.6），斜视到 60° 也过得去，
        # 只用来挡明显不是矩形的垃圾。
        wq0, hq0 = quad_mean_size(quad)
        pre_aspect = max(wq0, hq0) / max(1e-6, min(wq0, hq0))
        if pre_aspect < 0.55 or pre_aspect > 2.6:
            self._diag_reject("形状预筛(长宽比 %.2f)" % pre_aspect)
            return None
        # ---- 去畸变（镜头有桶形畸变时，"直边"在画面里是弯的）----
        # 现象：绿框和黑胶带对不齐、靶心跟着偏。实测用户反馈"画面有点球形、
        # 矩形边不是直线"。做法：先把四个角点用相机畸变参数纠正，再建单应矩阵
        # （角点精确对齐，画面内部误差很小，靶心在中间因此更准）。
        # dist 全 0 时是空操作；要真正生效得跑 tools/calib_intrinsic.py 做棋盘标定。
        if self.cam.has_distortion():
            quad = self.cam.undistort_points(quad)
        rect = build_rectifier(cfg.paper_mm, quad, cfg.warp_scale)
        if rect is None:
            self._diag_reject("单应退化(四边形自交/太小)", quad_wh=quad_mean_size(quad))
            return None
        warp = cv2.warpPerspective(frame, rect.h_img2rect, rect.size,
                                   flags=cv2.INTER_LINEAR,
                                   borderMode=cv2.BORDER_REPLICATE)
        # ---- 在矫正图里把激光光斑"抹掉" ----
        # 光斑最亮的地方是过曝白芯、周围还有一圈自适应阈值造成的"假黑环"。
        # 它正好落在靶心上时（= 我们已经打中的状态），会把"黑胶带厚度"的
        # 中心线测量带偏 -> 靶纸被判不合格 -> 云台重新扫描 -> 又跑开，
        # 现场现象就是"打中的那一刻开始来回摆"。
        # 做法：把光斑那块用【周围纸张的中位颜色】盖掉，后面的所有量测
        # （胶带厚度、红圈、质心）都在"没有激光"的画面上做。
        if spot is not None:
            warp = self._erase_spot(warp, rect, spot)
        # ---- 黑胶带厚度校验（抗模糊主力判据，见 measure_tape_mm 说明）----
        # 横竖分别量、分别判：四边形长宽比不对时两个方向会差很多，
        # 这正是"巡线黑方框"和"标准 A4 靶纸"最关键的区别。
        tx_mm, ty_mm = measure_tape_mm(warp, rect, cfg.dark_thresh)
        # ---- "黑边里包着一张亮纸"：靶纸的物理特征，几乎零成本 ----
        # 现场实测：真靶纸中心区灰度 ~77、黑胶带 ~15（差 60 多）；
        # 而柜门、阴影块、深色箱子这类误检，中间和黑边差不了多少。
        bright_ok, contrast = _band_contrast(
            warp, rect, tape_mm_ref=tx_mm or ty_mm,
            min_gap=float(getattr(cfg, "band_gap_min", 12.0)))
        if not bright_ok:
            self._diag_reject("内窗不比黑边亮(差 %.0f)" % contrast)
            return None
        if cfg.tape_mm <= 0.0:
            tx_mm = ty_mm = 0.0
        rx = tx_mm / cfg.tape_mm if tx_mm > 0.0 else 0.0
        ry = ty_mm / cfg.tape_mm if ty_mm > 0.0 else 0.0
        # 量不出来的方向（rx/ry = 0）不当成"超标"：那多半是被画面边缘切掉、
        # 被光斑打断、或者反光过曝。只要还有一个方向能证明"这里有一圈黑边"
        # 就够了 —— 否则靶纸一漂到画面边缘就会整块检不到。
        # ⚠ 2026-09-29：胶带厚度**不再当硬门槛**（改成只做加分）。
        #   原因：它是"拿外框当 A4 边界"量出来的，本身带 0.8 左右的系统性缩放，
        #   加上手贴胶带左右不等宽、斜视、模糊，量出来的值能把**真靶纸**拒掉。
        #   现场就是这个：靶纸完整在画面里、结构也配上了，却被"胶带厚度超标"
        #   拒掉 -> 云台来回找。国一那版压根不量胶带厚度，靠的是"回字结构"，
        #   我们改成同一路线。
        # 横竖一致性也不再是硬门槛（同一个道理），只参与打分。
        aniso = 0.0
        if rx > 0.0 and ry > 0.0:
            aniso = max(rx, ry) / max(1e-6, min(rx, ry))
        vals = [v for v in (tx_mm, ty_mm) if v > 0.0]
        tape_mm = sum(vals) / len(vals) if vals else 0.0
        rs = [r for r in (rx, ry) if r > 0.0]
        # 胶带分只做软加分：量出来"像一圈带子"就给分，量不出来也不扣死。
        # 另外把"横竖差太多"轻微罚一点（巡线方框就是这种形状）。
        tape_score = max(0.0, 1.0 - sum(abs(r - 1.0) for r in rs) / len(rs) / 0.6) \
            if rs else 0.0
        if aniso > 2.0:
            tape_score *= 0.5

        ring_score, red_px, centroid, covers = self._red_analysis(warp, rect,
                                                                 spot=spot)
        if cfg.require_red and (red_px < cfg.red_min_px or ring_score < cfg.min_ring_score):
            self._diag_reject("红圈不达标", red=red_px, ring=round(ring_score, 2))
            return None

        # ---- 靶心：国一的做法 = 校正图里"内层矩形"的质心 ----
        # 他们先把外框矫正成标准正视图，再在矫正图里重新找一次矩形，
        # 取最里面那个的质心当靶心；找不到就用矫正图中心（=外框中心）。
        # ⚠ 为什么不直接拿"内窗四边形"的对角线交点：靶纸上印的红圈会把
        #   白色纸面切成好几块，内窗轮廓常常只是其中一块（实测中心偏 24px）。
        #   在矫正图里重新找 + 用质心，对这个不敏感。
        rect_center = np.array([(rect.size[0] - 1) * 0.5, (rect.size[1] - 1) * 0.5])
        center_px = rect_center
        refine_px = 0.0
        use_centroid = False
        # 第一优先级：红圈质心（已在 _red_analysis 里屏蔽掉激光光斑）。
        # 五个同心圆关于靶心四重对称，质心精度远高于四边形角点；
        # 只有它离黑框中心不太远时才采信（防止被别处的红斑带跑）。
        if centroid is not None:
            delta = centroid - rect_center
            refine_px = float(np.linalg.norm(delta))
            if refine_px <= cfg.red_center_tol_px:
                center_px = centroid
                use_centroid = True
        c_img = cr2img_point(rect, center_px)
        uv_raw = (float(c_img[0]), float(c_img[1]))
        # 去畸变（有畸变参数时才有实际作用）
        uv = self.cam.undistort_points([uv_raw])[0]

        # 长宽比校验
        wq, hq = quad_mean_size(quad)
        obs = max(wq, hq) / max(1e-6, min(wq, hq))
        target_ratio = max(cfg.paper_mm) / min(cfg.paper_mm)
        aspect_err = abs(obs - target_ratio) / target_ratio
        aspect_score = float(max(0.0, 1.0 - aspect_err / max(1e-6, cfg.aspect_tol)))

        area_score = float(min(1.0, area / (0.05 * img_area)))
        center_score = float(max(0.0, 1.0 - refine_px / max(1e-6, cfg.red_center_tol_px)))
        if not use_centroid:
            center_score *= 0.5

        # ---- 置信度：以"回字结构"为主（国一的判据核心）----
        # 结构分是主角：能找到"外框 + 内窗且面积比合理"的配对，基本就是靶纸。
        # 其余（胶带厚度、面积、红圈）只做加分，不再当作硬门槛 —— 现场实测
        # 正是那些"硬门槛"（胶带厚度/内窗亮度）把真靶纸拒掉的。
        # 结构分：不只是"有没有内窗"，还要看"内窗比例像不像靶纸"。
        #   18mm 胶带贴在 A4 上，内窗/外框面积比 ≈ (174×261)/(210×297) = 0.73；
        #   而靶纸上的红圈被切成的小块、柜门内框之类，比例都在 0.3~0.5 或 >0.92。
        #   所以用 |ratio-0.74| 打分，能自动把真靶纸从一堆"看起来也像回字"的
        #   候选里挑出来（国一的 MIN_AREA_RATIO=0.7 就是这个意思，我们把它
        #   从"硬门槛"改成"打分"，对贴歪的胶带更宽容）。
        if inner is not None:
            ratio_exp = float(getattr(cfg, "nested_ratio_expect", 0.74))
            ratio_tol = float(getattr(cfg, "nested_ratio_tol", 0.22))
            structure = float(max(0.0, 1.0 - abs(float(ratio) - ratio_exp) / ratio_tol))
        else:
            structure = 0.0
        confidence = (0.30 + 0.34 * structure + 0.12 * area_score
                      + 0.12 * tape_score + 0.12 * ring_score)
        if inner is None:
            # 没有回字结构：置信度封顶到 min_confidence 以下（默认就不认）。
            # 这一条挡住巡线黑方框、实心黑块、普通柜门四边形。
            confidence = min(confidence,
                             float(getattr(cfg, "single_rect_max_conf", 0.44)))
        else:
            # 内窗也是"外接矩形兜底"出来的、或者外框被画面切了：不是干净证据
            if inner_box:
                confidence *= 0.93
            if box_used:
                confidence *= 0.90
            if clipped:
                confidence *= 0.85
        confidence = min(1.0, confidence)

        self.diag.setdefault("pass", []).append(dict(
            conf=round(confidence, 3), tape=round(tape_mm, 1),
            rx=round(rx, 2), ry=round(ry, 2), aspect=round(obs, 3),
            aspect_err=round(aspect_err, 3), inner=(inner is not None),
            ring=round(ring_score, 2), red=int(red_px), area=int(area)))

        # 注意：h_mm2px 的目标是"矫正图像素"，要再经 h_rect2img 才是原图，
        # 位姿分解必须用"靶面mm -> 原图像素"这一整条链
        pose = homography_pose(rect.h_rect2img @ rect.h_mm2px, self.cam)
        distance = pose["distance_m"] if pose else 0.0
        # ---- 面积法测距兜底（借鉴国一那版）----
        # 位姿解有时会失败（靶纸太斜/太远/边缘被切），这时 distance=0，
        # 下游"按距离查光斑位置"就断了。面积法的原理很简单：
        #     距离 ∝ 1/√(成像面积)  ->  L = L_ref × √(A_ref / A)
        # 参考对 (L_ref, A_ref) 不用手工标定：每次位姿解有效时顺手记下来，
        # 失败时用最近一次的参考对反推（慢速更新，抗误检）。
        if distance > 0.08:
            if self._dist_ref is None:
                self._dist_ref = [float(distance), float(area)]
            else:
                self._dist_ref[0] += 0.15 * (float(distance) - self._dist_ref[0])
                self._dist_ref[1] += 0.15 * (float(area) - self._dist_ref[1])
        elif self._dist_ref is not None and area > 1.0:
            distance = self._dist_ref[0] * math.sqrt(self._dist_ref[1] / float(area))
        # ---- 距离合理性（物理约束，很便宜的一条硬门限）----
        # E 题靶纸就在 0.5~1.6m 的范围内。现场实测有几帧把远处的柜门当成靶纸
        # （解出来 2.67m），一锁上去云台就冲着那边去。这里直接按物理范围挡掉。
        if distance > 0.05:
            dmin = float(getattr(cfg, "dist_min_m", 0.30))
            dmax = float(getattr(cfg, "dist_max_m", 2.00))
            if not (dmin <= distance <= dmax):
                self._diag_reject("距离不合理", d=round(distance, 2))
                return None
        return TargetResult(
            uv=(float(uv[0]), float(uv[1])),
            uv_raw=(float(uv_raw[0]), float(uv_raw[1])),
            quad=quad,
            rect=rect,
            distance_m=distance,
            confidence=confidence,
            source="quad",
            red_px=int(red_px),
            ring_score=ring_score,
            refine_px=refine_px if use_centroid else 0.0,
            tape_mm=tape_mm,
            tape_score=tape_score,
        )

    # ------------------------------------------------------------------
    def _erase_spot(self, warp, rect: Rectifier, spot):
        """把矫正图里激光光斑那一小块用周围纸色盖掉（只改这一小块像素）。"""
        try:
            px = apply_h(rect.h_img2rect,
                         np.array([[float(spot[0]), float(spot[1])]]))[0]
        except Exception:                                      # noqa: BLE001
            return warp
        h, w = warp.shape[:2]
        x, y = int(round(float(px[0]))), int(round(float(px[1])))
        if not (0 <= x < w and 0 <= y < h):
            return warp
        # 半径：按矫正比例换算，另外留 6px 余量盖住光斑外的"假黑环"
        r = int(max(6.0, min(0.25 * min(h, w),
                             float(spot[2]) / max(1e-6, rect.mm_per_px) * 1.6 + 6.0)))
        y0, y1 = max(0, y - r - 8), min(h, y + r + 9)
        x0, x1 = max(0, x - r - 8), min(w, x + r + 9)
        if (y1 - y0) < 8 or (x1 - x0) < 8:
            return warp
        patch = warp[y0:y1, x0:x1]
        yy, xx = np.mgrid[y0:y1, x0:x1]
        d2 = (xx - x) ** 2 + (yy - y) ** 2
        ann = (d2 > (r + 2.0) ** 2) & (d2 < (r + 8.0) ** 2)
        if not ann.any():
            return warp
        fill = np.median(patch[ann].reshape(-1, 3), axis=0)
        patch[d2 <= (r + 2.0) ** 2] = fill
        self._diag("spot_erased", 1)
        return warp

    # ------------------------------------------------------------------
    def _red_analysis(self, warp: np.ndarray, rect: Rectifier, spot=None):
        """在矫正视图里做红色分析：环覆盖度 + 加权质心。

        ⚠ spot 必须传：激光是【红色】的，打在纸上就是一个又大又亮的红斑。
        不把它挖掉的话，红圈质心会被它拽偏 —— 实测偏 15~21px（现场抓到的
        `refine=15.7~21.5px`）。而"红圈质心"是我们用来精修靶心位置的，
        偏 20px 就等于每发都偏 20px，而且这个偏差随光斑位置变，看着就像
        "怎么都收敛不到靶心"。
        """
        cfg = self.cfg
        if not cfg.red_enable:
            return 0.0, 0, None, []
        b, g, r = cv2.split(warp)
        rg = cv2.subtract(r, g)
        rb = cv2.subtract(r, b)
        _, m1 = cv2.threshold(rg, cfg.red_r_minus_g, 255, cv2.THRESH_BINARY)
        _, m2 = cv2.threshold(rb, cfg.red_r_minus_b, 255, cv2.THRESH_BINARY)
        mask = cv2.bitwise_and(m1, m2)
        if cfg.red_v_min > 0:
            _, m3 = cv2.threshold(r, cfg.red_v_min, 255, cv2.THRESH_BINARY)
            mask = cv2.bitwise_and(mask, m3)
        if spot is not None:
            # 在矫正图里把光斑那一块从"红色掩码"里挖掉（含周围一圈眩光）
            try:
                px = apply_h(rect.h_img2rect,
                             np.array([[float(spot[0]), float(spot[1])]]))[0]
                h, w = mask.shape[:2]
                x, y = int(round(float(px[0]))), int(round(float(px[1])))
                if 0 <= x < w and 0 <= y < h:
                    rr = int(max(4.0, min(0.3 * min(h, w),
                                          float(spot[2]) / max(1e-6, rect.mm_per_px)
                                          * 1.8 + 8.0)))
                    cv2.circle(mask, (x, y), rr, 0, -1)
            except Exception:                                  # noqa: BLE001
                pass
        red_px = int(cv2.countNonZero(mask))
        if red_px < max(4, cfg.red_min_px // 4):
            return 0.0, red_px, None, []
        geo = _ring_geometry(rect.size, rect.mm_per_px, cfg.ring_tol_mm,
                             rect.paper_mm, cfg.tape_mm)
        ring_score, covers = geo.ring_coverage(mask)
        # 加权质心：权重取"红优势"，弱化抗锯齿像素带来的偏差
        weight = cv2.min(rg, rb).astype(np.float32)
        weight = cv2.bitwise_and(weight, weight, mask=mask)
        m = cv2.moments(weight, binaryImage=False)
        if m["m00"] <= 1e-6:
            return ring_score, red_px, None, covers
        centroid = np.array([m["m10"] / m["m00"], m["m01"] / m["m00"]])
        return ring_score, red_px, centroid, covers

    # ------------------------------------------------------------------
    def _detect_red_only(self, frame: np.ndarray) -> Optional[TargetResult]:
        """兜底：黑框检不到时，直接找红圈并拟合圆心。

        用途：靶纸被画面边缘切掉一块、或者黑框被强反光破坏时，
        至少还能给出一个可用的靶心（精度差一些，也没有单应矩阵，
        所以画圆不可用）。

        中心用**椭圆最小二乘拟合**而不是 minEnclosingCircle：
        红圈是同心圆，外圈即使只露出一段圆弧，拟合出的椭圆中心
        仍然指向靶心；而 minEnclosingCircle 只取"包住已有像素的圆"，
        缺一块时圆心会被明显拉偏。
        """
        cfg = self.cfg
        b, g, r = cv2.split(frame)
        rg = cv2.subtract(r, g)
        rb = cv2.subtract(r, b)
        _, m1 = cv2.threshold(rg, cfg.red_r_minus_g, 255, cv2.THRESH_BINARY)
        _, m2 = cv2.threshold(rb, cfg.red_r_minus_b, 255, cv2.THRESH_BINARY)
        mask = cv2.bitwise_and(m1, m2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        cnt = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        if area < 120:
            return None
        radius = 0.0
        if len(cnt) >= 5:
            (cx, cy), (d1, d2), _ = cv2.fitEllipse(cnt)
            radius = 0.25 * (d1 + d2)
        else:
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
        if radius < 8:
            return None
        uv_raw = (float(cx), float(cy))
        uv = self.cam.undistort_points([uv_raw])[0]
        # 置信度刻意低于黑框路径（0.6~1.0），但高于 min_confidence 0.50，
        # 这样"红圈很清楚、只是黑框没检出"的情况下仍然可用
        return TargetResult(
            uv=(float(uv[0]), float(uv[1])),
            uv_raw=uv_raw,
            quad=None,
            rect=None,
            distance_m=0.0,
            confidence=0.60,
            source="red",
            red_px=int(cv2.countNonZero(mask)),
            ring_score=0.0,
        )


def _quad_center(quad) -> Tuple[float, float]:
    """四边形对角线的交点 = 该平面矩形中心在原图里的投影。

    透视变换保持"直线"与"交点"，所以矩形两条对角线的交点必然映射成四边形
    两条对角线的交点。用它当靶心比"四角点平均"更准：角点平均会被透视拉偏。
    """
    q = np.asarray(quad, dtype=np.float64).reshape(4, 2)
    a, b, c, d = q[0], q[1], q[2], q[3]
    p = _line_intersect(a, c, b, d)
    if p is None:                      # 退化四边形时退回四角平均
        return (float(q[:, 0].mean()), float(q[:, 1].mean()))
    return (float(p[0]), float(p[1]))


def _line_intersect(p1, p2, p3, p4):
    return _line_intersect_impl(p1, p2, p3, p4)


def _quad_inside(inner, outer, margin: float = 2.0) -> bool:
    """inner 这个四边形是否完全落在 outer 里面（允许 margin 像素的误差）。

    用 cv2.pointPolygonTest 的"带符号距离"判断，比比较外接矩形严格：
    外接矩形可能相交但形状并不真正嵌套（斜视时很常见）。
    """
    o = np.asarray(outer, dtype=np.float32).reshape(-1, 1, 2)
    for p in np.asarray(inner, dtype=np.float64).reshape(4, 2):
        d = cv2.pointPolygonTest(o, (float(p[0]), float(p[1])), True)
        if d < -float(margin):
            return False
    return True


def _line_intersect_impl(p1, p2, p3, p4):
    x1, y1 = float(p1[0]), float(p1[1])
    x2, y2 = float(p2[0]), float(p2[1])
    x3, y3 = float(p3[0]), float(p3[1])
    x4, y4 = float(p4[0]), float(p4[1])
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < 1e-9:
        return None
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / den
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / den
    return (px, py)


def cr2img_point(rect: Rectifier, pt_rect) -> Tuple[float, float]:
    """矫正图像素 -> 原图像素。"""
    from .geometry import apply_h

    p = apply_h(rect.h_rect2img, np.asarray(pt_rect, dtype=np.float64).reshape(-1, 2))
    return (float(p[0][0]), float(p[0][1]))


def _shift_result(res: TargetResult, dx: float, dy: float) -> TargetResult:
    """把"在 ROI 裁剪图里算出来"的结果平移到全图坐标系。

    quad / uv / uv_raw 直接加偏移；rect 的两个单应矩阵也要跟着平移
    （h_rect2img 输出的是图像像素，把平移量加到第三列即可，
    h_img2rect 再用逆矩阵重算，保证"靶面 mm <-> 原图像素"整条链一致）。
    """
    if res is None:
        return res
    res.uv = (float(res.uv[0]) + dx, float(res.uv[1]) + dy)
    res.uv_raw = (float(res.uv_raw[0]) + dx, float(res.uv_raw[1]) + dy)
    if res.quad is not None:
        res.quad = np.asarray(res.quad, dtype=np.float64) + np.array([dx, dy],
                                                                    dtype=np.float64)
    if res.rect is not None:
        h = np.array(res.rect.h_rect2img, dtype=np.float64, copy=True)
        h[0, 2] += float(dx)
        h[1, 2] += float(dy)
        res.rect.h_rect2img = h
        try:
            res.rect.h_img2rect = np.linalg.inv(h)
        except np.linalg.LinAlgError:
            pass
    return res
