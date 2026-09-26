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

    返回 (0.0, 0.0) 表示量不出来。
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
    # 任何一边量不出来就整体放弃（避免用一半数据得出错误结论）
    if min(xs + ys) <= 0.0:
        return 0.0, 0.0
    tx = 0.5 * (xs[0] + xs[1]) * rect.mm_per_px
    ty = 0.5 * (ys[0] + ys[1]) * rect.mm_per_px
    return float(tx), float(ty)


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
        self._rings = {}
        self.last_candidates = 0

    # ------------------------------------------------------------------
    def detect(self, frame: np.ndarray, scale: float = 1.0) -> Optional[TargetResult]:
        """找靶纸。

        scale < 1 时：先在缩小图上做"二值化 + 找四边形"（这一步最吃 CPU），
        找到的四边形再乘回原图坐标；后面的透视矫正、红圈亚像素质心
        仍然在原图上做。这样省时间，而且【不会损失瞄准精度】——
        因为精度来自矫正图上的质心，跟这一步的缩放无关。
        """
        h, w = frame.shape[:2]
        img_area = float(h * w)
        if scale and abs(scale - 1.0) > 1e-6 and scale > 0.05:
            sw = max(16, int(round(w * scale)))
            sh = max(16, int(round(h * scale)))
            small = cv2.resize(frame, (sw, sh), interpolation=cv2.INTER_AREA)
            inv = 1.0 / scale
        else:
            small = frame
            inv = 1.0
        # ⚠ 这里必须用【缩小图】的面积算阈值！
        #   之前误用了原图面积：缩小图的轮廓面积小 4 倍，而面积阈值没变，
        #   结果候选四边形全被滤掉 —— 表现为"缩放检测一个都找不到"。
        small_area = float(small.shape[0] * small.shape[1])
        binary = self._binarize(small, scale)
        quads = self._find_quads(binary, small_area)
        if inv != 1.0:
            quads = [(q * inv, a * inv * inv) for q, a in quads]
        self.last_candidates = len(quads)
        best: Optional[TargetResult] = None
        for quad, area in quads:
            res = self._evaluate(frame, quad, area, img_area)
            if res is None:
                continue
            if best is None or res.confidence > best.confidence:
                best = res
        if best is not None:
            return best
        if self.cfg.fallback_red and inv == 1.0:
            return self._detect_red_only(frame)      # 兜底不做缩放，直接用原图
        return None

    # ------------------------------------------------------------------
    def _binarize(self, frame: np.ndarray, scale: float = 1.0) -> np.ndarray:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        block = self.cfg.adaptive_block
        # 自适应阈值的窗口必须跟着缩放一起缩 —— 否则在缩小图上
        # blockSize=61 会比半个靶纸还大，局部阈值直接失效（实测踩过）
        if scale and abs(scale - 1.0) > 1e-6:
            block = int(round(block * scale))
        if block % 2 == 0:
            block += 1
        if block < 9:
            block = 9
        adaptive = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                         cv2.THRESH_BINARY_INV, block,
                                         self.cfg.adaptive_c)
        if self.cfg.use_otsu:
            # 自适应阈值对"大面积纯黑区"会失效（块内均值也是黑），
            # 用 Otsu 的全局结果补上这一块，两者取并集。
            _, otsu = cv2.threshold(gray, 0, 255,
                                    cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
            adaptive = cv2.bitwise_or(adaptive, otsu)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        return cv2.morphologyEx(adaptive, cv2.MORPH_CLOSE, kernel, iterations=1)

    def _find_quads(self, binary: np.ndarray, img_area: float):
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        lo = self.cfg.min_area_ratio * img_area
        hi = self.cfg.max_area_ratio * img_area
        cands = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < lo or area > hi:
                continue
            peri = cv2.arcLength(cnt, True)
            if peri < 4 * 20:
                continue
            approx = cv2.approxPolyDP(cnt, self.cfg.poly_eps_ratio * peri, True)
            if len(approx) != 4 or not is_convex_quad(approx):
                continue
            quad = order_quad(approx.reshape(4, 2))
            wq, hq = quad_mean_size(quad)
            if min(wq, hq) < 20:
                continue
            cands.append((quad, float(area)))
        cands.sort(key=lambda t: -t[1])
        return cands[: max(1, self.cfg.max_candidates)]

    # ------------------------------------------------------------------
    def _evaluate(self, frame, quad, area, img_area) -> Optional[TargetResult]:
        cfg = self.cfg
        rect = build_rectifier(cfg.paper_mm, quad, cfg.warp_scale)
        warp = cv2.warpPerspective(frame, rect.h_img2rect, rect.size,
                                   flags=cv2.INTER_LINEAR,
                                   borderMode=cv2.BORDER_REPLICATE)
        # ---- 黑胶带厚度校验（抗模糊主力判据，见 measure_tape_mm 说明）----
        # 横竖分别量、分别判：四边形长宽比不对时两个方向会差很多，
        # 这正是"巡线黑方框"和"标准 A4 靶纸"最关键的区别。
        tx_mm, ty_mm = measure_tape_mm(warp, rect, cfg.dark_thresh)
        if cfg.tape_mm <= 0.0:
            tx_mm = ty_mm = 0.0
        rx = tx_mm / cfg.tape_mm if tx_mm > 0.0 else 0.0
        ry = ty_mm / cfg.tape_mm if ty_mm > 0.0 else 0.0
        if not (cfg.tape_min_ratio <= rx <= cfg.tape_max_ratio):
            return None
        if not (cfg.tape_min_ratio <= ry <= cfg.tape_max_ratio):
            return None
        tape_mm = 0.5 * (tx_mm + ty_mm)
        tape_score = max(0.0, 1.0 - (abs(rx - 1.0) + abs(ry - 1.0)) * 0.5 / 0.6)

        ring_score, red_px, centroid, covers = self._red_analysis(warp, rect)
        if cfg.require_red and (red_px < cfg.red_min_px or ring_score < cfg.min_ring_score):
            return None

        rect_center = np.array([(rect.size[0] - 1) * 0.5, (rect.size[1] - 1) * 0.5])
        refine_px = 0.0
        use_centroid = False
        if centroid is not None:
            delta = centroid - rect_center
            refine_px = float(np.linalg.norm(delta))
            use_centroid = refine_px <= cfg.red_center_tol_px

        center_px = centroid if use_centroid else rect_center
        uv_raw = cr2img_point(rect, center_px)
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

        # 置信度里"黑带厚度"占大头（抗模糊），红圈分只在检到时加分
        confidence = (0.30 * tape_score + 0.25 * aspect_score
                      + 0.15 * area_score + 0.15 * center_score
                      + 0.15 * ring_score)

        # 注意：h_mm2px 的目标是"矫正图像素"，要再经 h_rect2img 才是原图，
        # 位姿分解必须用"靶面mm -> 原图像素"这一整条链
        pose = homography_pose(rect.h_rect2img @ rect.h_mm2px, self.cam)
        distance = pose["distance_m"] if pose else 0.0
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
    def _red_analysis(self, warp: np.ndarray, rect: Rectifier):
        """在矫正视图里做红色分析：环覆盖度 + 加权质心。"""
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


def cr2img_point(rect: Rectifier, pt_rect) -> Tuple[float, float]:
    """矫正图像素 -> 原图像素。"""
    from .geometry import apply_h

    p = apply_h(rect.h_rect2img, np.asarray(pt_rect, dtype=np.float64).reshape(-1, 2))
    return (float(p[0][0]), float(p[0][1]))
