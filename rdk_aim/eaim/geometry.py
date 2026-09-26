"""相机几何：小孔模型、角度-像素换算、单应矩阵、四边形规范化。

本文件是整个瞄准原理的数学底座，核心只有一条：

    相机转过 theta  =>  图像里的点移动 fx*tan(theta) ~= fx*theta 像素

所以"我要把光斑挪 e 个像素，需要转多少度"的答案是 theta = e/fx，
它就是瞄准环的开环增益，而且与靶的距离无关。
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np

__all__ = [
    "CameraModel",
    "Rectifier",
    "order_quad",
    "quad_mean_size",
    "homography_pose",
    "angle_between_rays_deg",
    "apply_h",
]


@dataclass
class CameraModel:
    """针孔相机模型（可选畸变校正）。"""

    fx: float = 640.0
    fy: float = 640.0
    cx: float = 640.0
    cy: float = 360.0
    dist: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0)

    @property
    def K(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    @property
    def center(self) -> Tuple[float, float]:
        return (self.cx, self.cy)

    @property
    def px_per_deg(self) -> Tuple[float, float]:
        """每度对应多少像素。这就是瞄准环要用的增益，与距离无关。"""
        k = math.pi / 180.0
        return self.fx * k, self.fy * k

    def has_distortion(self) -> bool:
        return any(abs(float(v)) > 1e-9 for v in self.dist)

    def undistort_points(self, pts) -> np.ndarray:
        arr = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
        if not self.has_distortion():
            return arr
        out = cv2.undistortPoints(arr.reshape(1, -1, 2), self.K,
                                  np.asarray(self.dist, dtype=np.float64))
        return out.reshape(-1, 2) * np.array([self.fx, self.fy]) + np.array([self.cx, self.cy])

    def pixel_to_ray(self, uv: Sequence[float]) -> np.ndarray:
        """像素坐标 -> 相机坐标系下的单位方向向量（+z 朝前）。"""
        u, v = float(uv[0]), float(uv[1])
        ray = np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        return ray / np.linalg.norm(ray)

    def pixel_to_angle_deg(self, uv: Sequence[float]) -> Tuple[float, float]:
        """像素坐标 -> 相对光轴的（水平角, 垂直角），单位度。"""
        u, v = float(uv[0]), float(uv[1])
        return (math.degrees(math.atan2(u - self.cx, self.fx)),
                math.degrees(math.atan2(v - self.cy, self.fy)))

    def project(self, pts_cam) -> np.ndarray:
        pts = np.asarray(pts_cam, dtype=np.float64).reshape(-1, 3)
        z = np.clip(pts[:, 2], 1e-9, None)
        return np.stack([self.fx * pts[:, 0] / z + self.cx,
                         self.fy * pts[:, 1] / z + self.cy], axis=1)


def apply_h(H: np.ndarray, pts) -> np.ndarray:
    """把 Nx2 的点用 3x3 单应矩阵映射（自动去齐次）。"""
    pts = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
    homo = np.concatenate([pts, np.ones((len(pts), 1))], axis=1) @ H.T
    w = homo[:, 2:3]
    w = np.where(np.abs(w) < 1e-12, 1e-12, w)
    return homo[:, :2] / w


def order_quad(pts) -> np.ndarray:
    """把 4 个点排成 左上 -> 右上 -> 右下 -> 左下。"""
    pts = np.asarray(pts, dtype=np.float64).reshape(4, 2)
    center = pts.mean(axis=0)
    ang = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    ordered = pts[np.argsort(ang)]
    start = int(np.argmin(ordered.sum(axis=1)))
    ordered = np.roll(ordered, -start, axis=0)
    v1 = ordered[1] - ordered[0]
    v2 = ordered[2] - ordered[1]
    if v1[0] * v2[1] - v1[1] * v2[0] < 0:      # 保证同一绕向
        ordered = ordered[[0, 3, 2, 1]]
    return ordered


def quad_mean_size(quad) -> Tuple[float, float]:
    """四边形的 (平均宽, 平均高)，像素。"""
    q = np.asarray(quad, dtype=np.float64).reshape(4, 2)
    w = (np.linalg.norm(q[1] - q[0]) + np.linalg.norm(q[2] - q[3])) * 0.5
    h = (np.linalg.norm(q[3] - q[0]) + np.linalg.norm(q[2] - q[1])) * 0.5
    return float(w), float(h)


def quad_area(quad) -> float:
    return float(abs(cv2.contourArea(np.asarray(quad, dtype=np.float32).reshape(-1, 1, 2))))


def is_convex_quad(pts) -> bool:
    return bool(cv2.isContourConvex(np.asarray(pts, dtype=np.float32).reshape(-1, 1, 2)))


@dataclass
class Rectifier:
    """靶纸正视图的坐标互转工具。

    约定：靶面毫米坐标原点在靶纸左上角，x 向右、y 向下，单位 mm。
    于是靶心 = (paper_w/2, paper_h/2)。
    """

    h_mm2px: np.ndarray          # 靶面mm -> 矫正图像素
    h_px2mm: np.ndarray          # 矫正图像素 -> 靶面mm
    h_rect2img: np.ndarray       # 矫正图像素 -> 原图像素
    h_img2rect: np.ndarray       # 原图像素 -> 矫正图像素
    size: Tuple[int, int]        # 矫正图 (宽, 高) 像素
    mm_per_px: float
    paper_mm: Tuple[float, float]

    # ---- 靶面 mm <-> 原图像素（画圆与瞄准都要用） ----
    def mm_to_img(self, pts_mm) -> np.ndarray:
        return apply_h(self.h_rect2img, apply_h(self.h_mm2px, pts_mm))

    def img_to_mm(self, pts_px) -> np.ndarray:
        return apply_h(self.h_px2mm, apply_h(self.h_img2rect, pts_px))

    # ---- 靶面 mm <-> 矫正图像素 ----
    def mm_to_rect(self, pts_mm) -> np.ndarray:
        return apply_h(self.h_mm2px, pts_mm)

    def rect_to_mm(self, pts_px) -> np.ndarray:
        return apply_h(self.h_px2mm, pts_px)

    @property
    def center_mm(self) -> Tuple[float, float]:
        return (self.paper_mm[0] * 0.5, self.paper_mm[1] * 0.5)

    def center_uv(self) -> Tuple[float, float]:
        """靶心在原图中的像素坐标。"""
        p = self.mm_to_img([self.center_mm])[0]
        return (float(p[0]), float(p[1]))

    def px_per_mm_at_center(self) -> float:
        """靶心处 1mm 对应多少像素（用于把像素容差换算成毫米）。"""
        c = np.array(self.center_mm)
        probe = c + np.array([10.0, 0.0])
        a = self.mm_to_img([c])[0]
        b = self.mm_to_img([probe])[0]
        return float(np.linalg.norm(b - a) / 10.0)


def build_rectifier(paper_mm: Sequence[float], quad, scale: float) -> Rectifier:
    """由黑框四边形构造 Rectifier。

    quad 必须是 order_quad() 之后的 左上/右上/右下/左下 顺序。
    通过比较四边形的长短边决定靶纸是横放还是竖放。
    """
    pw, ph = float(paper_mm[0]), float(paper_mm[1])
    w_obs, h_obs = quad_mean_size(quad)
    if h_obs >= w_obs:
        size = (max(8, int(round(pw * scale))), max(8, int(round(ph * scale))))
    else:
        size = (max(8, int(round(ph * scale))), max(8, int(round(pw * scale))))
    sw, sh = float(size[0] - 1), float(size[1] - 1)
    # 矫正图像素 <-> 靶面mm：线性缩放（x 与 y 的 mm/px 相同，保持图形不被拉伸）
    mm_per_px = (pw / sw + ph / sh) * 0.5
    h_px2mm = np.array(
        [[mm_per_px, 0.0, 0.0], [0.0, mm_per_px, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64
    )
    h_mm2px = np.linalg.inv(h_px2mm)
    dst = np.array([[0.0, 0.0], [sw, 0.0], [sw, sh], [0.0, sh]], dtype=np.float32)
    quad32 = np.asarray(quad, dtype=np.float64).reshape(4, 2).astype(np.float32)
    h_img2rect = cv2.getPerspectiveTransform(quad32, dst)
    h_rect2img = np.linalg.inv(h_img2rect)
    return Rectifier(
        h_mm2px=h_mm2px,
        h_px2mm=h_px2mm,
        h_rect2img=h_rect2img,
        h_img2rect=h_img2rect,
        size=size,
        mm_per_px=mm_per_px,
        paper_mm=(pw, ph),
    )


def homography_pose(h_mm2px: np.ndarray, cam: "CameraModel") -> Optional[dict]:
    """从"靶面mm -> 图像像素"的单应矩阵反解靶面相对相机的位姿。

    原理：H = K [r1 r2 t] / s，把 K^-1 H 的前两列单位化即可求出 s。
    返回 dict(distance_m, normal_cam, R_cam_from_paper, t_cam_mm) 或 None。
    """
    try:
        A = np.linalg.inv(cam.K) @ np.asarray(h_mm2px, dtype=np.float64)
    except np.linalg.LinAlgError:
        return None
    a1, a2, a3 = A[:, 0], A[:, 1], A[:, 2]
    n1, n2 = np.linalg.norm(a1), np.linalg.norm(a2)
    if n1 < 1e-12 or n2 < 1e-12:
        return None
    inv_s = 2.0 / (n1 + n2)
    r1, r2 = a1 * inv_s, a2 * inv_s
    r3 = np.cross(r1, r2)
    n3 = np.linalg.norm(r3)
    if n3 < 1e-12:
        return None
    r3 = r3 / n3
    r1 = r1 / max(np.linalg.norm(r1), 1e-12)
    r2 = np.cross(r3, r1)
    t = a3 * inv_s
    return {
        "distance_m": float(np.linalg.norm(t)) / 1000.0,   # t 的单位是 mm
        "t_cam_mm": t,
        "normal_cam": r3,
        "R_cam_from_paper": np.stack([r1, r2, r3], axis=1),
    }


def angle_between_rays_deg(cam: CameraModel, uv_a, uv_b) -> float:
    """两个像素点对应的视线夹角（度）。"""
    ra = cam.pixel_to_ray(uv_a)
    rb = cam.pixel_to_ray(uv_b)
    return math.degrees(math.acos(float(np.clip(np.dot(ra, rb), -1.0, 1.0))))

