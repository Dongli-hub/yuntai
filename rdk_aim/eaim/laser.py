"""激光光斑检测。

判据有两种模式（配置 laser.mode），必须按**实测**选，不能想当然：

  blue : advantage = B - max(G,R)          "蓝通道占优"
         适合蓝紫激光（405nm）没被打亮、颜色还是深蓝紫的情况
  warm : advantage = min(B,R) - G  + 亮度  "够亮 + 暖色（红/品红）占优"
         本项目的激光是**红色**的，打在白纸上中心过曝成白芯、边上一圈暖色，
         实测 BGR≈(89,43,150)（R>B>G），蓝优势是**负的** —— 用 blue 会
         一直检不到真光斑，只检到窗边/衣物的蓝色色散伪影（实测踩过）。

红色激光 + 红色靶心：**光靠颜色是分不开的**，真正把它们分开的是
  (1) 亮度：打印的红圈是"暗红"，红激光光斑是过曝的亮斑；
  (2) 位置：光斑相对图像是刚性固定的（同轴装配），只在标定的那一小块里找；
  (3) 尺寸/形状：光斑是团，红圈是细环。

不管哪种模式，"亮度"这一条都要一起满足：它能排掉那些又暗又蓝的
色散伪影（实测伪影 BGR≈(133,115,132)，亮度只有 115）。

更关键的是同轴装配带来的强先验：
    光斑在图像里的位置是"刚性固定"的（见 docs/00 第一节）。
所以默认只在已知光斑位置附近搜（门控 / ROI），其它地方一律不认，
白色反光、顶灯高光、窗边色散这类误检基本被一网打尽。
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from .config import LaserConfig

__all__ = ["SpotResult", "LaserSpotDetector"]


@dataclass
class SpotResult:
    uv: Tuple[float, float]          # 光斑亚像素坐标
    area: int                        # 光斑像素数
    score: float                     # 蓝优势均值（越大越像激光）
    brightness: float = 0.0
    gated: bool = False              # 是否被位置门控约束在已知点附近

    def describe(self) -> str:
        return ("spot=(%.1f,%.1f) area=%d score=%.0f bright=%.0f%s"
                % (self.uv[0], self.uv[1], self.area, self.score,
                   self.brightness, " gated" if self.gated else ""))


class LaserSpotDetector:
    """蓝紫激光光斑检测器。

    learn_boresight() 会在首次稳定检出后收紧门控半径 ——
    因为光斑位置是常数，一旦测到就不该再"跑到别处去"。
    """

    def __init__(self, cfg: LaserConfig):
        self.cfg = cfg
        self.boresight: Optional[Tuple[float, float]] = None
        self.table: list = []          # [[距离m, u, v], ...]，见 set_boresight_table
        self.learned = False
        self.gate_center: Optional[Tuple[float, float]] = None
        self.hits = 0
        self.misses = 0
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

    # ------------------------------------------------------------------
    def set_boresight(self, uv: Tuple[float, float]) -> None:
        self.boresight = (float(uv[0]), float(uv[1]))
        self.gate_center = self.boresight
        # 注意：这里**不**把 learned 置 1。配置里的光轴点只是"先验/门控中心"，
        # 真正的位置必须由实测来确认（见 detect() 里的慢速自适应）。

    def set_boresight_table(self, table) -> None:
        """装入"光斑位置-距离"标定表 [[距离m, u, v], ...]（按距离升序）。"""
        self.table = [(float(t[0]), float(t[1]), float(t[2]))
                      for t in (table or []) if len(t) >= 3]
        self.table.sort(key=lambda t: t[0])

    def boresight_for_distance(self, dist_m: Optional[float]) -> Optional[Tuple[float, float]]:
        """光斑没检到时，按靶纸距离估计"光斑此刻应该在哪个像素"。

        为什么光斑位置会随距离变（这是"标定一个固定点够不够"这个问题的核心）：

            相机看到的是"光斑相对相机的**方向角**"，不是它在靶面上的位置。
            设激光轴相对相机轴有平移 d(m)、夹角 alpha(rad)，靶面距离 L(m)：

                方向角 ≈ alpha + d / L

            · L 变大 -> d/L 变小 -> 光斑向"两轴平行时的方向"靠
            · 只有 alpha = 0 且 d = 0（真正的同轴）时才与距离无关

            所以"固定一个像素点"只在标定距离上准：0.8m 标定的点，
            到 1.5m 可能偏几十像素。做法不是猜，而是把 (L,u,v) 标 2~3 个点，
            运行时按实测距离线性插值 —— 距离由靶纸单应矩阵给出（A4 已知尺寸），
            不需要额外测距传感器。

        表是空的或距离未知时，退回单点 boresight（老行为，不会变差）。
        """
        if self.table and dist_m is not None and float(dist_m) > 0.05:
            L = float(dist_m)
            # 超出标定范围就夹在端点上：外推（尤其是近距离方向）很容易飞
            if L <= self.table[0][0]:
                return (self.table[0][1], self.table[0][2])
            if L >= self.table[-1][0]:
                return (self.table[-1][1], self.table[-1][2])
            for i in range(1, len(self.table)):
                l0, u0, v0 = self.table[i - 1]
                l1, u1, v1 = self.table[i]
                if L <= l1:
                    t = (L - l0) / (l1 - l0)
                    return (u0 + t * (u1 - u0), v0 + t * (v1 - v0))
        return self.boresight

    def _adapt(self, uv: Tuple[float, float]) -> None:
        """用实测光斑位置慢速修正光轴点。

        光斑位置是常数，但会随距离有轻微视差漂移；慢速自适应让它始终准。
        限幅 ±6px/次 是防止某个误检把门控中心"拖跑"。
        """
        if self.boresight is None:
            self.boresight = (float(uv[0]), float(uv[1]))
            self.gate_center = self.boresight
            self.learned = True
            return
        bx, by = self.boresight
        dx = max(-6.0, min(6.0, (float(uv[0]) - bx) * 0.06))
        dy = max(-6.0, min(6.0, (float(uv[1]) - by) * 0.06))
        self.boresight = (bx + dx, by + dy)
        self.gate_center = self.boresight

    def fail_streak(self) -> int:
        return self.misses

    # ------------------------------------------------------------------
    def detect(self, frame: np.ndarray, scale: float = 1.0) -> Optional[SpotResult]:
        """在整幅图中找激光光斑。返回 None 表示没找到。

        scale < 1 时在缩小图上找（光斑是个大团，缩小后照样能找到），
        找到的坐标再乘回去。这样在 CPU 弱的地瓜派上能省好几倍时间。

        若配置了 roi_px 且已知光轴点（标定过），则改为"在光轴点周围
        一小块里做全分辨率搜索"：光斑只有几个像素，缩放到 0.5 会被糊掉，
        而全图全分辨率又太慢 —— ROI 让两者兼得，而且 ROI 本身就起门控作用。
        """
        cfg = self.cfg
        ox = oy = 0
        roi_active = False
        if cfg.roi_px > 0:
            center = self.boresight if self.boresight is not None else self.gate_center
            R = int(cfg.roi_px)
            if center is not None and frame.shape[0] > 2 * R and frame.shape[1] > 2 * R:
                cx = int(round(center[0]))
                cy = int(round(center[1]))
                x0 = max(0, min(frame.shape[1] - 1, cx - R))
                y0 = max(0, min(frame.shape[0] - 1, cy - R))
                x1 = min(frame.shape[1], x0 + 2 * R)
                y1 = min(frame.shape[0], y0 + 2 * R)
                frame = frame[y0:y1, x0:x1]
                ox, oy = x0, y0
                scale = 1.0            # ROI 内用全分辨率（这正是要它的原因）
                roi_active = True
        if scale and abs(scale - 1.0) > 1e-6 and scale > 0.05:
            sw = max(16, int(round(frame.shape[1] * scale)))
            sh = max(16, int(round(frame.shape[0] * scale)))
            small = cv2.resize(frame, (sw, sh), interpolation=cv2.INTER_AREA)
            inv = 1.0 / scale
        else:
            small = frame
            inv = 1.0

        b, g, r = cv2.split(small)
        # 用 cv2.subtract（饱和减，SIMD 优化过）比 numpy 的 int16 转换快好几倍
        if str(cfg.mode).lower() in ("warm", "violet"):
            # "够亮 + 暖色（红/品红）占优"：R（和 B）都比 G 明显高。
            # 打印的红圈虽然也是红色，但它是**暗红**（亮度条件排掉）；
            # 窗边/衣物的蓝色色散是 B 高、R/G 都低（亮度条件也排掉）。
            advantage = cv2.subtract(cv2.min(b, r), g)
            bright = cv2.min(cv2.min(b, g), r)
            gap_min = max(1, cfg.violet_gap - 1)
            br_min = max(1, cfg.bright_min - 1)
        else:
            advantage = cv2.subtract(b, cv2.max(g, r))
            bright = b
            gap_min = max(1, cfg.b_minus_others - 1)
            br_min = max(1, cfg.b_min - 1)
        # 颜色优势 + 亮度双条件
        _, m1 = cv2.threshold(advantage, gap_min, 255,
                              cv2.THRESH_BINARY)
        _, m2 = cv2.threshold(bright, br_min, 255, cv2.THRESH_BINARY)
        mask = cv2.bitwise_and(m1, m2)
        # ROI 生效时它本身就是门控，不用再加一层（坐标也不是全图的了）
        gate = None if roi_active else self._gate_mask(small.shape, scale)
        if gate is not None:
            mask = cv2.bitwise_and(mask, gate)
        if cv2.countNonZero(mask) == 0:
            self.misses += 1
            return None
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if n_labels <= 1:
            self.misses += 1
            return None
        img_area = float(small.shape[0] * small.shape[1])
        img_h, img_w = small.shape[:2]
        max_area = cfg.max_area_ratio * img_area
        m = max(0, int(round(cfg.border_margin * scale)))
        best = None
        best_key = -1.0
        # 面积门槛要跟着缩放走（面积按 scale^2 缩）
        min_area = max(2, int(round(cfg.min_area * scale * scale)))
        for i in range(1, n_labels):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < min_area or area > max_area:
                continue
            x0 = int(stats[i, cv2.CC_STAT_LEFT])
            y0 = int(stats[i, cv2.CC_STAT_TOP])
            w = int(stats[i, cv2.CC_STAT_WIDTH])
            h = int(stats[i, cv2.CC_STAT_HEIGHT])
            # ---- 假光斑过滤 1：贴边的不要 ----
            # 真实激光光斑是个完整的亮团，不会正好贴在画面最边缘 1~2 像素上。
            # 实测踩过：图像边缘的色度伪影（一条偏蓝的边）被当成激光，
            # 而且因为"学习光轴点"机制，会把门控中心永久带偏。
            if m > 0 and (x0 <= m or y0 <= m or
                          (x0 + w) >= (img_w - m) or (y0 + h) >= (img_h - m)):
                continue
            # ---- 假光斑过滤 2：细长条不是激光（光斑近似圆形）----
            if cfg.max_aspect > 0.0:
                aspect = float(max(w, h)) / float(max(1, min(w, h)))
                if aspect > cfg.max_aspect:
                    continue
            sub = labels[y0:y0 + h, x0:x0 + w] == i
            adv_sub = advantage[y0:y0 + h, x0:x0 + w]
            b_sub = bright[y0:y0 + h, x0:x0 + w]
            score = float(adv_sub[sub].mean()) if area else 0.0
            brightness = float(b_sub[sub].mean()) if area else 0.0
            # 打分：蓝优势为主，面积作为次要因素（大一点更可能是激光而不是噪点）
            key = score * (1.0 + 0.05 * math.log10(max(1.0, area)))
            if key > best_key:
                best_key = key
                best = (i, area, score, brightness, sub, adv_sub, b_sub, x0, y0, w, h)
        if best is None:
            self.misses += 1
            return None
        i, area, score, brightness, sub, adv_sub, b_sub, x0, y0, w, h = best
        uv = self._centroid(sub, adv_sub, b_sub, x0, y0, w, h) if cfg.subpixel else \
            self._binary_centroid(sub, x0, y0)
        if inv != 1.0:
            uv = (uv[0] * inv, uv[1] * inv)      # 缩放图坐标 -> 原图坐标
        if roi_active:
            uv = (uv[0] + ox, uv[1] + oy)        # ROI 坐标 -> 原图坐标
        self.hits += 1
        self.misses = 0
        gated = self.learned
        self._adapt(uv)
        self.learned = True
        return SpotResult(uv=uv, area=int(area), score=score,
                          brightness=brightness, gated=gated)

    # ------------------------------------------------------------------
    def _gate_mask(self, shape, scale: float = 1.0) -> Optional[np.ndarray]:
        cfg = self.cfg
        if not cfg.gate_enable:
            return None
        center = self.boresight if self.boresight is not None else self.gate_center
        if center is None:
            return None
        radius = cfg.gate_radius_px if self.learned else cfg.gate_relax_px
        mask = np.zeros(shape[:2], dtype=np.uint8)
        # 门控中心/半径是"原图坐标"，缩放到当前检测尺度
        cv2.circle(mask, (int(round(center[0] * scale)), int(round(center[1] * scale))),
                   max(2, int(round(radius * scale))), 255, -1)
        return mask

    def _centroid(self, sub, adv_sub, b_sub, x0, y0, w, h) -> Tuple[float, float]:
        """用"蓝优势 + 亮度"加权求亚像素质心。"""
        weight = adv_sub.astype(np.float32) * (0.5 + 0.5 * b_sub.astype(np.float32) / 255.0)
        weight[~sub] = 0.0
        total = float(weight.sum())
        if total <= 1e-6:
            return self._binary_centroid(sub, x0, y0)
        ys, xs = np.mgrid[0:h, 0:w]
        cx = float((weight * xs).sum() / total) + x0
        cy = float((weight * ys).sum() / total) + y0
        return (cx, cy)

    @staticmethod
    def _binary_centroid(sub, x0, y0) -> Tuple[float, float]:
        ys, xs = np.nonzero(sub)
        if len(xs) == 0:
            return (float(x0), float(y0))
        return (float(xs.mean()) + x0, float(ys.mean()) + y0)
