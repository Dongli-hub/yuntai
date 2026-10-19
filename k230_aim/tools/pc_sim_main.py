# -*- coding: utf-8 -*-
"""pc_sim_main.py —— 在电脑上把 main.py 完整跑一遍（不需要 K230）

原理：伪造 K230 的运行环境（machine / media.sensor / image / ustruct），
      再用真实 OpenCV 生成合成场景（黑胶带靶纸 + 激光光斑），
      同时伪造一个 H723：解析收到的帧、按一阶模型转动云台、回遥测。

这样就能在电脑上验证 main.py 的：
  * 状态机（WAIT_READY -> SET_ZERO -> TRACK）
  * 协议收发（心跳/模式/AIM/遥测）
  * 视觉检测 -> 误差 -> 控制 -> 收敛

用法：
    cd D:\\STM32\\STM32projects\\yuntai2
    python k230_aim\\tools\\pc_sim_main.py

（场景尺寸、死区、K 都从 main.py 里读/跟着算，所以改了 PROC_W/PROC_H 之后
  这里不用动 —— 仿真跑通了就说明那两个分辨率下 main.py 的链路是自洽的。）
"""
import io
import os
import re
import struct
import sys
import time as real_time
import types

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
# main.py 的位置**多找几个地方**（2026-10-17）：
#   仓库里      = <repo>/k230_aim/main.py
#   桌面副本    = Desktop/K230代码/main.py（PC 工具和 main.py 摆在一起时）
# 以前只认第一种，桌面那份 pc_* 脚本一跑就说找不到 —— 这里按顺序找第一个存在的。
_HERE = os.path.dirname(os.path.abspath(__file__))
_MAIN_CANDIDATES = [
    os.path.join(ROOT, "k230_aim", "main.py"),
    os.path.join(os.path.dirname(_HERE), "main.py"),
    os.path.join(_HERE, "main.py"),
]
MAIN_PY = None
for _c in _MAIN_CANDIDATES:
    if os.path.isfile(_c):
        MAIN_PY = _c
        break
if MAIN_PY is None:
    raise SystemExit("找不到 main.py，找过这些地方：%s"
                     % "; ".join(_MAIN_CANDIDATES))


def _read_proc_size():
    """从 main.py 里读 PROC_W/PROC_H —— 仿真场景必须和真实处理分辨率一致，
    否则 main.py 里那套"按 S_SCALE 缩放"的判据会全部对不上（手动同步迟早会忘）。"""
    txt = io.open(MAIN_PY, encoding="utf-8").read()
    w = int(re.search(r"^PROC_W\s*=\s*(\d+)", txt, re.M).group(1))
    h = int(re.search(r"^PROC_H\s*=\s*(\d+)", txt, re.M).group(1))
    return w, h


IMG_W, IMG_H = _read_proc_size()
S_SCALE = IMG_W / 640.0
# fx=430px(640 档) 时 1° ≈ 7.5px；分辨率一换，px/° 同比缩放（和 main.py 的
# VIS_K = 7.8*S 是同一个道理，只是这里用的焦距估值略小）
PX_PER_DEG = 430.0 * S_SCALE * 3.14159265 / 180.0

# ----------------------------------------------------------------------
# 虚拟时间
# ----------------------------------------------------------------------
_vtime = [0]


def ticks_ms():
    return _vtime[0]


def ticks_diff(a, b):
    return a - b


def sleep_ms(ms):
    _vtime[0] += int(ms)


real_time.ticks_ms = ticks_ms
real_time.ticks_diff = ticks_diff
real_time.sleep_ms = sleep_ms

_iterations = [0]
MAX_ITERS = 6000


def exitpoint(*a, **kw):
    _iterations[0] += 1
    if _iterations[0] > MAX_ITERS:
        raise KeyboardInterrupt()


os.exitpoint = exitpoint
os.EXITPOINT_ENABLE_SLEEP = 0

sys.modules["ustruct"] = struct

# ----------------------------------------------------------------------
# 合成场景
# ----------------------------------------------------------------------
class Scene(object):
    def __init__(self):
        # 基准点都按 S_SCALE 缩放：640x480 时就是 (300,240)/(312,252)
        self.tgt_u = 300.0 * S_SCALE
        self.tgt_v = 240.0 * S_SCALE
        # 与 main.py 的 USE_FIXED_LASER / LASER_UV 保持一致（激光在画面里的固定位置）
        # ⚠ 2026-10-14：LASER_UV 已冻结为 71 号日志的自标定实测值 (315,246)，
        #   这里的合成光斑要跟着改，否则"末端残差"会把 (3,-6) 这个口径差算进去、
        #   看起来像仿真变差了（实际落点一点没变）。
        self.spot_u = 315.0 * S_SCALE
        self.spot_v = 246.0 * S_SCALE
        # 2026-10-16：用户换成**印刷靶**（黑框直接印在纸上，纸中间还有几圈
        #   印刷的同心环）—— 和"黑胶带贴在白纸上"是两种画面。置 True 就渲染
        #   印刷靶（pc_vision_test 的 H 例）。
        self.printed = False
        # 2026-10-16 晚：用户现场那张图 —— **墙是暗的**（反向二值化后墙也是白的！），
        #   靶纸边缘有一圈很窄的黑缝把它和墙分开。J 例专门用它压这个场景。
        self.deadwall = False
        # B 例（斜视）只考几何：把右侧那块"假白块"关掉，免得它在透视变换后
        #   和靶框粘成一个分量（那是另一个用例的题目）。
        self.no_clutter = False

    def render(self):
        if self.deadwall:
            return self.render_deadwall()
        if self.printed:
            return self.render_printed()
        return self.render_tape()

    def render_deadwall(self):
        """用户现场那张图：**暗墙**（反向二值化后墙也是白的）+ 靶纸。

        要点：靶纸边缘留一圈很窄的**亮缝**（现实里是纸和框之间那条白边），
        在反向二值化里它是黑的 —— 正好把"靶框"和"暗墙"分开。
        它们一旦被粘成一个分量，四点拟合必然失败（122r 现场就是 b37 个分量、
        33 个栽在"拟"这一关）。
        """
        img = np.full((IMG_H, IMG_W, 3), 70, np.uint8)      # 暗墙 -> 二值化后是白的
        w = int(220 * S_SCALE)
        h = int(340 * S_SCALE)
        t = int(round(26 * S_SCALE))                        # 印刷框宽度
        x0 = int(self.tgt_u - w / 2)
        y0 = int(self.tgt_v - h / 2)
        # 纸外的亮缝（= 二值化里的黑缝）：把纸和暗墙隔开
        cv2.rectangle(img, (x0 - 6, y0 - 6), (x0 + w + 6, y0 + h + 6),
                      (150, 150, 150), -1)
        # ⚠ 框要**画在纸里面**（先整张填框色、再把纸填回去）—— cv2.rectangle 的
        #   描边是以边线为中心画的，直接画会往外扩半个线宽、把"亮缝"盖掉，
        #   结果墙和框在二值图里粘成一个分量（那就不是要考的场景了）。
        cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (60, 60, 60), -1)
        cv2.rectangle(img, (x0 + t, y0 + t), (x0 + w - t, y0 + h - t),
                      (235, 235, 235), -1)
        _cu = int(self.tgt_u)
        _cv = int(self.tgt_v)
        _r0 = int(0.40 * min(w, h))
        for _i in range(4):
            _rr = int(_r0 * (1.0 - 0.22 * _i))
            cv2.circle(img, (_cu, _cv), _rr, (70, 70, 70),
                       max(2, int(4 * S_SCALE)))
        cv2.circle(img, (_cu, _cv), max(2, int(2 * S_SCALE)),
                   (70, 70, 70), -1)
        cv2.circle(img, (int(self.spot_u), int(self.spot_v)),
                   int(round(5 * S_SCALE)), (200, 200, 255), -1)
        cv2.circle(img, (int(self.spot_u), int(self.spot_v)),
                   int(round(2 * S_SCALE)), (245, 245, 255), -1)
        return img

    def render_printed(self):
        """印刷靶：白纸 + **印上去的**黑框 + 纸中间几圈印刷同心环。

        2026-10-16 现场：用户把靶纸换成了整张打印的版本 ——
          · 框和纸的对比度比黑胶带低（打印黑 ~60，不是胶带的 25）；
          · 纸中间还有 4 圈印刷同心环（比纸暗得多，反向二值化后是白圈）；
          · 纸边到框之间只有很窄的白边。
        这正是"二值化图看着挺好、矩形却认不出来"的那张画面，H 例专门用它。
        """
        img = np.full((IMG_H, IMG_W, 3), 150, np.uint8)     # 背景：白墙/柜面
        w = int(200 * S_SCALE)
        h = int(330 * S_SCALE)
        t = int(round(10 * S_SCALE))                        # 印刷框宽度 px
        x0 = int(self.tgt_u - w / 2)
        y0 = int(self.tgt_v - h / 2)
        cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (235, 235, 235), -1)
        cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (60, 60, 60), t)
        _cu = int(self.tgt_u)
        _cv = int(self.tgt_v)
        _r0 = int(0.40 * min(w, h))
        for _i in range(4):
            _rr = int(_r0 * (1.0 - 0.22 * _i))
            cv2.circle(img, (_cu, _cv), _rr, (70, 70, 70),
                       max(2, int(3 * S_SCALE)))
        cv2.circle(img, (_cu, _cv), max(2, int(2 * S_SCALE)),
                   (70, 70, 70), -1)
        cv2.circle(img, (int(self.spot_u), int(self.spot_v)),
                   int(round(5 * S_SCALE)), (200, 200, 255), -1)
        cv2.circle(img, (int(self.spot_u), int(self.spot_v)),
                   int(round(2 * S_SCALE)), (245, 245, 255), -1)
        return img

    def render_tape(self):
        # ⚠ 2026-10-15：场景**现实化**（原来背景和"纸面"一样亮 190，
        #   二值化时"纸"和背景根本分不开，Otsu 只能靠兜底值；而且激光点画成
        #   暗红实心盘，落在"纸"里面就成了块内的一坨厚黑 -> 新加的内轮廓校验
        #   必拒收）。现在照现场的样子铺三层：
        #     背景(白墙/木柜) 150、纸面 235、黑胶带 25 —— 阈值有真正的落脚处；
        #     激光点 = 白芯 + 淡红光晕（真实激光打在白纸上就是过曝的白点，
        #     不是黑点）。这样仿真才验得了"Otsu + 开/腐 + Canny"这条新链。
        # ⚠ 2026-10-16：背景 70 -> **120**，对齐"国一那套"的物理前提 ——
        #   反向二值化(CV2_TH=89)只把**比阈值暗**的东西变白：胶带(25~)变白、
        #   背景(120)与纸(235)保持黑。这样胶带圈才是**孤立的一条白带**，
        #   内外双轮廓(内框=纸的洞)才配得上对；背景若比阈值还暗，它会和胶带
        #   连成一片、把圈吞掉（那是仿真场景失真，不是算法问题）。
        # 2026-10-17：背景 120 -> **140**。自适应阈值是 0.515×纸面(P90)，
        #   现场木柜/纸 ≥0.56；140/235=0.60 才符合现场（120 只有 0.51，
        #   会被自适应阈值切成白，属于仿真场景失真）。
        img = np.full((IMG_H, IMG_W, 3), 140, np.uint8)
        w = int(200 * S_SCALE)
        h = int(330 * S_SCALE)
        t = int(round(20 * S_SCALE))
        x0 = int(self.tgt_u - w / 2)
        y0 = int(self.tgt_v - h / 2)
        cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (25, 25, 30), t)
        # 圈内 = 白纸（比背景亮一大截，这就是"内亮外黑"的来源）
        cv2.rectangle(img, (x0 + t, y0 + t), (x0 + w - t, y0 + h - t),
                      (235, 235, 235), -1)
        # 2026-10-17：**靶纸内部加同心环** —— 现场目标（印刷+胶带）内侧就有
        #   4 圈同心圆，检测链的 INTERIOR_TRANS_MIN 判据就是靠它区分
        #   "靶纸"和"灯/窗这种均匀亮块"。仿真场景必须一致，否则 A/B/C/G
        #   这些用例会被误判成"没有内部结构"。
        _cu = int(self.tgt_u)
        _cv = int(self.tgt_v)
        _r0 = int(0.40 * min(w, h))
        for _i in range(4):
            _rr = int(_r0 * (1.0 - 0.22 * _i))
            cv2.circle(img, (_cu, _cv), _rr, (70, 70, 70),
                       max(2, int(3 * S_SCALE)))
        cv2.circle(img, (_cu, _cv), max(2, int(2 * S_SCALE)),
                   (70, 70, 70), -1)
        # 激光点：白芯 + 淡红光晕（BGR；光晕 luma≈216，仍在纸面之上，
        # 不会在块里形成"厚黑"）。红边保持 a 分量够红，万一开光斑检测也能用。
        _su = int(self.spot_u)
        _sv = int(self.spot_v)
        cv2.circle(img, (_su, _sv), int(round(5 * S_SCALE)),
                   (200, 200, 255), -1)
        cv2.circle(img, (int(self.spot_u), int(self.spot_v)),
                   int(round(2 * S_SCALE)), (245, 245, 255), -1)
        # 低对比"假白块"（回归用）：一块亮底里挖一圈暗槽，槽里的白块也是候选，
        # 但它"外侧"采样点落在亮底上（几乎一样亮）-> 必然在"对比"这道便宜
        # 闸门上被拒。⚠ 2026-10-15 那版"闸门前置"的 UnboundLocalError 就是
        # 因为仿真场景里从来没有候选走到这条早退路（真机上一跑就 候选0）。
        _px0 = x0 + w + 30
        _py0 = max(0, y0 + 20)
        if (not self.no_clutter) and (_px0 + 150) < IMG_W and \
                (_py0 + 110) < IMG_H:
            cv2.rectangle(img, (_px0, _py0), (_px0 + 150, _py0 + 110),
                          (200, 200, 200), -1)
            cv2.rectangle(img, (_px0 + 6, _py0 + 6), (_px0 + 144, _py0 + 104),
                          (60, 60, 60), 4)
        return img


SCENE = Scene()


class FakeBlob(object):
    """复现 CanMV blob 的下标访问 + 质心接口。

    b[0..3]=x,y,w,h  b[4]=面积  b[5/6]=**外接框中心**（和 OpenMV 一致）；
    另外 cx()/cy()/cxf()/cyf() 是**质心**（2026-10-17 加：纸块兜底用的是质心，
    外接框中心在透视下会偏 10px 量级）。
    """

    def __init__(self, x, y, w, h, area, cx=None, cy=None, pts=None):
        self._v = (x, y, w, h, area, x + w // 2, y + h // 2)
        self._cx = float(x + w / 2.0) if cx is None else float(cx)
        self._cy = float(y + h / 2.0) if cy is None else float(cy)
        self._pts = pts

    def __getitem__(self, i):
        return self._v[i]

    def cx(self):
        return int(self._cx)

    def cy(self):
        return int(self._cy)

    def cxf(self):
        return self._cx

    def cyf(self):
        return self._cy

    def min_corners(self):
        """对应 blob.min_corners()：最小外接矩形的四个角点。"""
        if self._pts is None or len(self._pts) < 5:
            x, y, w, h = self._v[0], self._v[1], self._v[2], self._v[3]
            return ((x, y), (x + w, y), (x + w, y + h), (x, y + h))
        rect = cv2.minAreaRect(self._pts.astype(np.float32))
        box = cv2.boxPoints(rect)
        c = box.mean(axis=0)
        order = np.argsort(np.arctan2(box[:, 1] - c[1], box[:, 0] - c[0]))
        box = box[order]
        return tuple((float(p[0]), float(p[1])) for p in box)


class FakeFrame(object):
    def __init__(self):
        self.np = SCENE.render()

    def to_numpy_ref(self):
        return self.np

    def draw_line(self, *a, **kw):
        pass

    def draw_circle(self, *a, **kw):
        pass

    def draw_cross(self, *a, **kw):
        pass

    def draw_rectangle(self, *a, **kw):
        pass

    def draw_string_advanced(self, *a, **kw):
        pass

    def find_rects(self, threshold=20000, x_gradient=8, y_gradient=8):
        gray = cv2.cvtColor(self.np, cv2.COLOR_BGR2GRAY)
        bin_img = cv2.adaptiveThreshold(gray, 255,
                                        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                        cv2.THRESH_BINARY_INV, 31, 7)
        contours, _ = cv2.findContours(bin_img, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in contours:
            if cv2.contourArea(c) < 3000:
                continue
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            if len(approx) != 4:
                continue

            class R(object):
                def __init__(self, pts):
                    self._p = pts

                def corners(self):
                    return self._p

                def rect(self):
                    xs = [p[0] for p in self._p]
                    ys = [p[1] for p in self._p]
                    return (min(xs), min(ys), max(xs) - min(xs),
                            max(ys) - min(ys))
            out.append(R([(int(p[0][0]), int(p[0][1])) for p in approx]))
        return out

    def find_blobs(self, thresholds, roi=None, merge=True, margin=4,
                   pixels_threshold=3, area_threshold=3,
                   x_stride=2, y_stride=1):
        """仿真版 find_blobs：在合成场景上做真实 LAB 阈值分割。
        2026-10-11：原来这里是个只认光斑的占位桩，导致仿真里"靶纸检测"
        永远命中 0；现在按 CanMV 语义实现（阈值 -> ROI -> margin 合并 ->
        连通域），PC 仿真才能真正跑通"视觉 -> 上报 -> 外环"整条链。"""
        lab = cv2.cvtColor(self.np, cv2.COLOR_BGR2LAB)
        L = lab[:, :, 0].astype(np.float32) / 2.55      # 0~100（同 CanMV）
        A = lab[:, :, 1].astype(np.int16) - 128
        B = lab[:, :, 2].astype(np.int16) - 128
        # 2026-10-15：**两种阈值元组都接** —— 6 元组 = LAB（彩色图）；
        # 2 元组 = (lo,hi) 亮度（灰度图；CVLITE_ON 时 sensor 出灰度图走这条）。
        gray = cv2.cvtColor(self.np, cv2.COLOR_BGR2GRAY).astype(np.float32)
        mask = np.zeros(L.shape, np.uint8)
        for _t in thresholds:
            if len(_t) == 2:
                m = (gray >= _t[0]) & (gray <= _t[1])
            else:
                l0, l1, a0, a1, b0, b1 = _t
                m = ((L >= l0) & (L <= l1) & (A >= a0) & (A <= a1) &
                     (B >= b0) & (B <= b1))
            mask[m] = 255
        if roi is not None:
            rx, ry, rw, rh = roi
            keep = np.zeros_like(mask)
            y1 = max(0, min(IMG_H, ry + rh))
            x1 = max(0, min(IMG_W, rx + rw))
            keep[max(0, ry):y1, max(0, rx):x1] = \
                mask[max(0, ry):y1, max(0, rx):x1]
            mask = keep
        if merge and (margin > 0):
            k = np.ones((2 * margin + 1, 2 * margin + 1), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        n, _labels, stats, _cents = cv2.connectedComponentsWithStats(mask, 4)
        out = []
        for i in range(1, n):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < max(int(pixels_threshold), int(area_threshold)):
                continue
            ys, xs = np.where(_labels == i)
            out.append(FakeBlob(int(stats[i, cv2.CC_STAT_LEFT]),
                                int(stats[i, cv2.CC_STAT_TOP]),
                                int(stats[i, cv2.CC_STAT_WIDTH]),
                                int(stats[i, cv2.CC_STAT_HEIGHT]), area,
                                float(_cents[i][0]), float(_cents[i][1]),
                                np.column_stack([xs, ys])))
        return out

    def get_histogram(self, *a, **kw):
        """仿真版直方图：用 OpenCV 的 Otsu 阈值冒充 CanMV 的
        img.get_histogram().get_threshold().value()，让 PC 仿真也能走
        main.py 新的 Otsu 阈值路径。"""
        gray = cv2.cvtColor(self.np, cv2.COLOR_BGR2GRAY)
        v, _ = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        class _P(object):
            def __init__(self, val):
                self._v = val

            def value(self):
                return self._v

        class _H(object):
            def __init__(self, val):
                self._v = val

            def get_threshold(self):
                return _P(self._v)

        return _H(int(v))

    def get_pixel(self, x, y):
        p = self.np[int(y), int(x)]
        return (int(p[2]), int(p[1]), int(p[0]))

    def to_grayscale(self):
        """对应 CanMV 的 img.to_grayscale()：**返回新图**，原图不动。"""
        return FakeGrayFrame(self)

    def to_numpy_ref(self):
        """对应 CanMV 的 img.to_numpy_ref()：返回 ndarray 引用（仿真里就是 np）。"""
        return self.np


class FakeGrayFrame(object):
    """仿真版灰度图（形态学预处理路径用）：
    支持 close(size) 闭运算、灰度语义 find_blobs([(lo, hi)])、get_pixel 返回单值。"""

    def __init__(self, src):
        self.np = cv2.cvtColor(src.np, cv2.COLOR_BGR2GRAY)

    def close(self, size):
        k = 2 * int(size) + 1
        kern = np.ones((k, k), np.uint8)
        self.np = cv2.morphologyEx(self.np, cv2.MORPH_CLOSE, kern)
        return self

    def open(self, size, **kw):
        k = 2 * int(size) + 1
        kern = np.ones((k, k), np.uint8)
        self.np = cv2.morphologyEx(self.np, cv2.MORPH_OPEN, kern)
        return self

    def gaussian(self, size, **kw):
        k = 2 * int(size) + 1
        self.np = cv2.GaussianBlur(self.np, (k, k), 0)
        return self

    def erode(self, size, **kw):
        """对应 CanMV img.erode(size)：核 (2*size+1)²。"""
        k = 2 * int(size) + 1
        kern = np.ones((k, k), np.uint8)
        self.np = cv2.erode(self.np, kern)
        return self

    def copy(self, roi=None, **kw):
        """对应 CanMV img.copy([roi])：返回一份独立副本（不改原图）。"""
        n = FakeGrayFrame.__new__(FakeGrayFrame)
        if roi is None:
            n.np = self.np.copy()
        else:
            x, y, w, h = [int(v) for v in roi]
            n.np = np.ascontiguousarray(self.np[y:y + h, x:x + w]).copy()
        return n

    def dilate(self, size, **kw):
        """对应 CanMV img.dilate(size)：核 (2*size+1)²。"""
        k = 2 * int(size) + 1
        self.np = cv2.dilate(self.np, np.ones((k, k), np.uint8))
        return self

    def draw_image(self, src, x, y, x_scale=1.0, y_scale=1.0, **kw):
        """对应 CanMV img.draw_image(小图, x, y, x_scale=, y_scale=)：贴回原图。

        2026-10-16 加：预览图的大核闭运算走"降采样 -> close -> 放大贴回"
        （见 main.py _cv2_find ③），这一步要用它。"""
        s = src.np
        if (x_scale != 1.0) or (y_scale != 1.0):
            h = max(1, int(s.shape[0] * y_scale))
            w = max(1, int(s.shape[1] * x_scale))
            s = cv2.resize(s, (w, h), interpolation=cv2.INTER_NEAREST)
        h2 = min(s.shape[0], self.np.shape[0] - int(y))
        w2 = min(s.shape[1], self.np.shape[1] - int(x))
        if (h2 > 0) and (w2 > 0):
            self.np[int(y):int(y) + h2, int(x):int(x) + w2] = s[:h2, :w2]
        return self

    def find_edges(self, edge_type, threshold=None, **kw):
        """对应 CanMV img.find_edges(image.EDGE_CANNY, (lo, hi))：
        把图像变成"黑底白线"的边缘图（原图就地改）。"""
        if threshold is None:
            threshold = kw.get("threshold", (100, 200))
        lo, hi = int(threshold[0]), int(threshold[1])
        self.np = cv2.Canny(self.np, lo, hi)
        return self

    def b_or(self, other):
        """对应 CanMV img.b_or(other)：逐像素按位或（就地改）。"""
        self.np = cv2.bitwise_or(self.np, other.np)
        return self

    def binary(self, thresholds, **kw):
        lo = thresholds[0][0]
        hi = thresholds[0][1]
        m = (self.np >= lo) & (self.np <= hi)
        self.np = np.where(m, 255, 0).astype(np.uint8)
        return self

    def to_grayscale(self):
        # 已经是灰度格式：仿真里直接共用（对应板上"再转一次灰度"那步）
        return self

    def to_numpy_ref(self):
        """对应 CanMV 灰度图的 img.to_numpy_ref()：**单通道** ndarray 引用。
        （板上 sensor 配的是 GRAYSCALE，所以整条链拿到的就是 1 通道；
          轮廓扫描那套是按 1 通道写的，仿真里必须一致。）"""
        return self.np

    def get_pixel(self, x, y):
        return int(self.np[int(y), int(x)])

    def find_blobs(self, thresholds, roi=None, merge=True, margin=4,
                   pixels_threshold=3, area_threshold=3,
                   x_stride=2, y_stride=1):
        lo = thresholds[0][0]
        hi = thresholds[0][1]
        mask = np.zeros(self.np.shape, np.uint8)
        mask[(self.np >= lo) & (self.np <= hi)] = 255
        if roi is not None:
            rx, ry, rw, rh = roi
            keep = np.zeros_like(mask)
            y1 = max(0, min(IMG_H, ry + rh))
            x1 = max(0, min(IMG_W, rx + rw))
            keep[max(0, ry):y1, max(0, rx):x1] = \
                mask[max(0, ry):y1, max(0, rx):x1]
            mask = keep
        if merge and (margin > 0):
            k = np.ones((2 * margin + 1, 2 * margin + 1), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        n, _labels, stats, _cents = cv2.connectedComponentsWithStats(mask, 4)
        out = []
        for i in range(1, n):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < max(int(pixels_threshold), int(area_threshold)):
                continue
            ys, xs = np.where(_labels == i)
            out.append(FakeBlob(int(stats[i, cv2.CC_STAT_LEFT]),
                                int(stats[i, cv2.CC_STAT_TOP]),
                                int(stats[i, cv2.CC_STAT_WIDTH]),
                                int(stats[i, cv2.CC_STAT_HEIGHT]), area,
                                float(_cents[i][0]), float(_cents[i][1]),
                                np.column_stack([xs, ys])))
        return out


class FakeSensor(object):
    RGB565 = 0
    RGB888 = 1
    GRAYSCALE = 2

    def __init__(self, *a, **kw):
        self.frames = 0

    def reset(self):
        pass

    def set_framesize(self, **kw):
        pass

    def set_pixformat(self, *a, **kw):
        pass

    def snapshot(self, *a, **kw):
        sleep_ms(30)
        self.frames += 1
        return FakeFrame()

    def run(self):
        pass

    def stop(self):
        pass


class FakeDisplay(object):
    VIRT = 0
    ST7701 = 1

    @staticmethod
    def init(*a, **kw):
        pass

    @staticmethod
    def deinit():
        pass

    @staticmethod
    def show_image(*a, **kw):
        pass


class FakeMediaManager(object):
    @staticmethod
    def init():
        pass

    @staticmethod
    def deinit():
        pass


class FakeImage(object):
    RGB565 = 0
    RGB888 = 1
    GRAYSCALE = 2
    ALLOC_REF = 3              # 对应 image.ALLOC_REF：引用外部 buffer

    def __init__(self, w, h, fmt, alloc=None, data=None, **kw):
        self.w, self.h, self.fmt = w, h, fmt
        self.alloc = alloc
        # ALLOC_REF 语义：直接引用传进来的 ndarray（cv_lite 返回的边缘图就这么用）
        self.np = data

    def clear(self):
        pass

    def to_grayscale(self):
        """对应 image.Image(...).to_grayscale()（RGB -> 灰度）。"""
        f = FakeGrayFrame.__new__(FakeGrayFrame)
        if self.np is None:
            f.np = np.zeros((self.h, self.w), np.uint8)
        elif self.np.ndim == 3:
            f.np = cv2.cvtColor(self.np, cv2.COLOR_BGR2GRAY)
        else:
            f.np = self.np
        return f

    def dilate(self, size, **kw):
        """对应 image.Image(ALLOC_REF).dilate(size)：就地改 cv_lite 返回的 ndarray。"""
        k = 2 * int(size) + 1
        self.np = cv2.dilate(self.np, np.ones((k, k), np.uint8))
        return self

    def to_numpy_ref(self):
        return self.np

    def find_blobs(self, *a, **kw):
        f = FakeGrayFrame.__new__(FakeGrayFrame)
        f.np = self.np
        return f.find_blobs(*a, **kw)

    def get_pixel(self, x, y):
        p = self.np[int(y), int(x)]
        try:
            return (int(p[2]), int(p[1]), int(p[0]))
        except Exception:
            return int(p)

    def draw_string_advanced(self, *a, **kw):
        pass


def install_fake_modules():
    machine = types.ModuleType("machine")

    # ---- cv_lite 的假实现（资料《06.1 CV_lite图像处理专题》里的函数）----
    # 只实现 main.py 用到的那一个：grayscale_find_rectangles_with_corners，
    # 返回格式照抄例程 06.1/5 的注释：
    #   [[x,y,w,h, c1x,c1y, c2x,c2y, c3x,c3y, c4x,c4y], ...]
    def _fake_find_rects_corners(shape, np_img, c1, c2, eps, area_ratio,
                                 max_cos, blur):
        g = np_img
        if g.ndim == 3:
            g = cv2.cvtColor(g, cv2.COLOR_BGR2GRAY)
        g = np.ascontiguousarray(g.astype(np.uint8))
        k = int(blur) | 1
        if k >= 3:
            g = cv2.GaussianBlur(g, (k, k), 0)
        e = cv2.Canny(g, int(c1), int(c2))
        # 与 v2.9.0 的 C 实现一致：RETR_EXTERNAL。新链只取最外层框，
        # 靶心由框内亮纸块质心给出，不再依赖内轮廓配对。
        cnts, _ = cv2.findContours(e, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        H = int(shape[0])
        W = int(shape[1])
        amin = max(1.0, float(area_ratio) * H * W)
        out = []
        for c in cnts:
            if cv2.contourArea(c) < amin:
                continue
            peri = cv2.arcLength(c, True)
            ap = cv2.approxPolyDP(c, float(eps) * peri, True)
            if (len(ap) != 4) or (not cv2.isContourConvex(ap)):
                continue
            # ⚠ 2026-10-17：**max_angle_cos 这一关必须在仿真里也做掉** ——
            #   板上 C 函数有这个角度闸门（"越小越接近矩形"），以前的假实现
            #   把它漏了，所以"板上认不出、仿真全绿"。语义：四个内角里
            #   任何一个和 90° 的 |cos| 超过 max_cos 就丢
            #   （0.3 = 只放 72.5°~107.5°；0.5 = 放 ±30°，国一的口径）。
            pts4 = [np.array(q[0], np.float32) for q in ap]
            _bad = False
            for _i in range(4):
                _p0 = pts4[(_i - 1) % 4]
                _p1 = pts4[_i]
                _p2 = pts4[(_i + 1) % 4]
                _v1 = _p0 - _p1
                _v2 = _p2 - _p1
                _n1 = float(np.linalg.norm(_v1))
                _n2 = float(np.linalg.norm(_v2))
                if (_n1 < 1e-9) or (_n2 < 1e-9):
                    _bad = True
                    break
                if abs(float(np.dot(_v1, _v2)) / (_n1 * _n2)) > float(max_cos):
                    _bad = True
                    break
            if _bad:
                continue
            x, y, w, h = cv2.boundingRect(ap)
            p = [tuple(int(v) for v in q[0]) for q in ap]
            out.append([x, y, w, h,
                        p[0][0], p[0][1], p[1][0], p[1][1],
                        p[2][0], p[2][1], p[3][0], p[3][1]])
        return out

    cvl = types.ModuleType("cv_lite")
    cvl.grayscale_find_rectangles_with_corners = _fake_find_rects_corners

    def _fake_grayscale_find_blobs(shape, np_img, lo, hi, min_area, kernel):
        """对应 cv_lite.grayscale_find_blobs：inRange -> findContours(EXTERNAL)."""
        g = np.asarray(np_img)
        if g.ndim == 3:
            g = cv2.cvtColor(g, cv2.COLOR_BGR2GRAY)
        binary = np.where((g >= int(lo)) & (g <= int(hi)), 255, 0).astype(np.uint8)
        if int(kernel) > 1:
            k = np.ones((int(kernel), int(kernel)), np.uint8)
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, k)
        cnts, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in cnts:
            if cv2.contourArea(c) < float(min_area):
                continue
            x, y, w, h = cv2.boundingRect(c)
            out.extend([int(x), int(y), int(w), int(h)])
        return out

    cvl.grayscale_find_blobs = _fake_grayscale_find_blobs

    def _fake_find_edges(shape, np_img, t1, t2):
        """对应 cv_lite.grayscale_find_edges（资料 06.1/7）：Canny 边缘图。"""
        g = np_img
        if g.ndim == 3:
            g = cv2.cvtColor(g, cv2.COLOR_BGR2GRAY)
        g = np.ascontiguousarray(g.astype(np.uint8))
        e = cv2.Canny(g, int(t1), int(t2))
        return np.dstack([e, e, e]) if g.ndim == 3 else e

    cvl.grayscale_find_edges = _fake_find_edges
    sys.modules["cv_lite"] = cvl

    class Pin(object):
        def __init__(self, n):
            self.n = n

    class UART(object):
        def __init__(self, unit, **kw):
            self.unit = unit
            self.rx_buf = bytearray()
            FAKE_H723.bind(self)

        def write(self, data):
            FAKE_H723.feed(bytes(data))

        def any(self):
            return len(self.rx_buf)

        def read(self, n=256):
            if not self.rx_buf:
                return b""
            out = bytes(self.rx_buf[:n])
            del self.rx_buf[:n]
            return out

        def deinit(self):
            pass

    machine.Pin = Pin
    machine.UART = UART
    sys.modules["machine"] = machine

    sensor_mod = types.ModuleType("media.sensor")
    sensor_mod.Sensor = FakeSensor
    sensor_mod.CAM_CHN_ID_0 = 0
    display_mod = types.ModuleType("media.display")
    display_mod.Display = FakeDisplay
    media_mod = types.ModuleType("media.media")
    media_mod.MediaManager = FakeMediaManager
    media_pkg = types.ModuleType("media")
    media_pkg.sensor = sensor_mod
    media_pkg.display = display_mod
    media_pkg.media = media_mod
    sys.modules["media"] = media_pkg
    sys.modules["media.sensor"] = sensor_mod
    sys.modules["media.display"] = display_mod
    sys.modules["media.media"] = media_mod

    image_mod = types.ModuleType("image")
    image_mod.Image = FakeImage
    image_mod.RGB565 = FakeImage.RGB565
    image_mod.RGB888 = FakeImage.RGB888
    image_mod.GRAYSCALE = FakeImage.GRAYSCALE
    image_mod.ALLOC_REF = FakeImage.ALLOC_REF
    image_mod.EDGE_CANNY = 1
    image_mod.EDGE_SIMPLE = 2
    sys.modules["image"] = image_mod

    # ---- ulab.numpy：板上是内置的微型 numpy；PC 上直接用真 numpy 顶上 ----
    #   轮廓扫描那套（切片 / 比较 / argmax）用的是 ulab 支持的那一小撮接口，
    #   numpy 全部兼容 —— 所以离线回归跑的算法和板上是同一份代码。
    ulab_mod = types.ModuleType("ulab")
    ulab_mod.numpy = np
    sys.modules["ulab"] = ulab_mod
    sys.modules["ulab.numpy"] = np


# ----------------------------------------------------------------------
# 假 H723
# ----------------------------------------------------------------------
def _crc16(body):
    crc = 0xFFFF
    for byte in body:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


class FakeH723(object):
    def __init__(self):
        self.uart = None
        self.buf = bytearray()
        self.t = 0.0
        self.att_yaw = 0.0
        self.att_pitch = 0.0
        self.cmd_yaw = 0.0
        self.cmd_pitch = 0.0
        self.mode = 0
        self.laser = 0
        self.ready = False
        self.aim_count = 0
        self.ack_count = 0
        # 视觉外环状态（镜像 H723 固件 vis_outer_step 的数学，2026-10-11 新架构）
        self.px_ut = 0
        self.px_vt = 0
        self.px_us = 0
        self.px_vs = 0
        self.aim_flags = 0
        self.vis_k_x100 = 0            # AIM 尾部带的 K×100（0 = 没带）
        self.px_ms = -1e9
        self.vis_eu = 0.0
        self.vis_ev = 0.0
        self.vis_seen = False
        self.vis_last_ms = -1e9

    def bind(self, uart):
        self.uart = uart

    def feed(self, data):
        self.buf.extend(data)
        while True:
            i = self.buf.find(b"\xAA\x55")
            if i < 0 or len(self.buf) - i < 7:
                break
            if i > 0:
                del self.buf[:i]
            length = self.buf[4]
            total = 5 + length + 2
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:5 + length])
            msg_id = body[0]
            payload = body[3:]
            del self.buf[:total]
            self.on_frame(msg_id, payload)

    def on_frame(self, msg_id, payload):
        if msg_id == 0x10 and len(payload) in (10, 12):
            # 新架构（2026-10-11）：ut vt us vs (u16×4 小端) + flags + quality
            # 12 字节版：尾部再带 kx100 = 像素/度 ×100（分辨率可切换）
            (ut, vt, us, vs, flags, _q) = struct.unpack("<HHHHBB", payload[:10])
            self.px_ut, self.px_vt = ut, vt
            self.px_us, self.px_vs = us, vs
            self.aim_flags = flags
            if len(payload) >= 12:
                self.vis_k_x100 = struct.unpack("<h", payload[10:12])[0]
            self.px_ms = self.t * 1000.0
            self.laser = 1 if (flags & 0x01) else 0
            self.aim_count += 1
        elif msg_id == 0x11 and len(payload) >= 1:
            self.mode = payload[0]
            self.send_ack(0x11, self.mode)
        elif msg_id == 0x12:
            self.send_ack(0x12, 1)
        elif msg_id == 0x13:
            self.cmd_yaw = self.cmd_pitch = 0.0
            self.att_yaw = self.att_pitch = 0.0
            self.vis_eu = self.vis_ev = 0.0
            self.vis_seen = False

    def _send(self, msg_id, payload):
        body = struct.pack("<BBB", msg_id, 0, len(payload)) + payload
        self.uart.rx_buf.extend(b"\xAA\x55" + body +
                                struct.pack("<H", _crc16(body)))

    def send_ack(self, msg_id, code):
        self.ack_count += 1
        self._send(0x91, bytes([msg_id, code]))

    def send_telemetry(self):
        gz = struct.pack("<BBhhhhhhhBI", 6, 0,
                         int(self.att_yaw * 100), int(self.att_pitch * 100),
                         0, 0, 0, 0, 0,
                         0x10 if self.ready else 0x00, int(self.t * 1000))
        self._send(0x90, gz)

    def step(self):
        if not self.ready and self.t > 1.0:
            self.ready = True
        dt = 0.02
        # ---- 视觉外环（镜像 H723 固件 vis_outer_step）：像素误差 -> 偏置 ----
        vid = (self.aim_flags & 0x02) != 0          # AIM_VALID
        now_ms = self.t * 1000.0
        if vid and (self.px_ms - self.vis_last_ms) > 1.0:
            self.vis_last_ms = self.px_ms
            eu = self.px_ut - self.px_us
            ev = self.px_vt - self.px_vs
            db = 3.0 * S_SCALE      # 死区也随分辨率缩放（同 H723 的 ks 做法）
            if abs(eu) < db:
                eu = 0.0
            if abs(ev) < db:
                ev = 0.0
            if not self.vis_seen:
                self.vis_eu, self.vis_ev, self.vis_seen = eu, ev, True
            else:
                self.vis_eu += 0.7 * (eu - self.vis_eu)
                self.vis_ev += 0.7 * (ev - self.vis_ev)
        if (self.mode == 2) and vid and \
                ((now_ms - self.vis_last_ms) <= 150.0):
            # K：优先用 AIM 帧带过来的（4~16 之间才算数），否则默认 7.8
            k = self.vis_k_x100 / 100.0
            if (k < 4.0) or (k > 16.0):
                k = 7.8
            ru = -4.0 * (self.vis_eu / k)            # VIS_SIGN*VIS_KP/K
            rv = -3.0 * (self.vis_ev / k)
            ru = max(-120.0, min(120.0, ru))
            rv = max(-60.0, min(60.0, rv))
            self.cmd_yaw = max(-120.0, min(120.0, self.cmd_yaw + ru * dt))
            self.cmd_pitch = max(-20.0, min(20.0, self.cmd_pitch + rv * dt))
        tau = 0.12
        if self.mode == 2:
            self.att_yaw += (self.cmd_yaw - self.att_yaw) * dt / tau
            self.att_pitch += (self.cmd_pitch - self.att_pitch) * dt / tau
        # 符号约定：偏置增大（att 跟着增大）-> 目标在画面里向正方向移。
        # 只有这样 SIGN=-1 的外环才是负反馈（与现场收敛方向一致）。
        SCENE.tgt_u = 300.0 + self.att_yaw * PX_PER_DEG
        SCENE.tgt_v = 240.0 + self.att_pitch * PX_PER_DEG
        self.t += dt
        self.send_telemetry()


FAKE_H723 = None


def run():
    global FAKE_H723
    FAKE_H723 = FakeH723()
    install_fake_modules()

    src = open(MAIN_PY, "r", encoding="utf-8").read()
    # 仿真永远测"整机闭环"：即便现场把 main.py 切成了 LOG_ONLY（只监听，
    # 不发 SET_ZERO/AIM），这里也强制改回整机模式 —— 否则仿真的
    # "闭环收敛 OK/未收敛"判据会永远落空（AIM 帧 0 条）。
    src = re.sub(r"(?m)^LOG_ONLY\s*=\s*\w+", "LOG_ONLY = False", src)

    fake_t = [0.0]

    def exitpoint2(*a, **kw):
        now = _vtime[0] / 1000.0
        while fake_t[0] < now:
            FAKE_H723.step()
            fake_t[0] += 0.02
        exitpoint(*a, **kw)

    os.exitpoint = exitpoint2

    print("=" * 66)
    print("PC 仿真：%dx%d，最多 %d 个循环" % (IMG_W, IMG_H, MAX_ITERS))
    print("=" * 66)
    try:
        exec(compile(src, MAIN_PY, "exec"), {"__name__": "__main__"})
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass

    ex = SCENE.tgt_u - SCENE.spot_u
    ey = SCENE.tgt_v - SCENE.spot_v
    # ⚠ 2026-10-13：扣掉"激光点固定微调 LASER_TRIM"。
    #   整机把"靶心"驱动到 (假定光斑 = LASER_UV+trim) 上，所以控制的残差
    #   应相对这个参考点来算；仿真里的 SCENE.spot 是理想光斑，不扣会把
    #   现场标定值误判成"未收敛"。
    try:
        _m = re.search(r"(?m)^LASER_TRIM\s*=\s*\(\s*(-?\d+)\s*,\s*(-?\d+)\s*\)",
                       open(MAIN_PY, "r", encoding="utf-8").read())
        if _m:
            ex -= int(_m.group(1))
            ey -= int(_m.group(2))
    except Exception:
        pass
    err = (ex * ex + ey * ey) ** 0.5
    print("-" * 66)
    print("仿真结束：虚拟时间 %.2fs" % (_vtime[0] / 1000.0))
    print("AIM 帧 %d 条，ACK %d 条" % (FAKE_H723.aim_count,
                                       FAKE_H723.ack_count))
    print("末端偏置 yaw=%.2f° pitch=%.2f° ，云台姿态 yaw=%.2f°"
          % (FAKE_H723.cmd_yaw, FAKE_H723.cmd_pitch, FAKE_H723.att_yaw))
    print("末端残差 %.2f px（%.4f°）" % (err, err / PX_PER_DEG))
    print("结果：%s" % ("闭环收敛 OK" if err < 12 else "未收敛，需要检查"))
    return 0 if err < 12 else 1


if __name__ == "__main__":
    sys.exit(run())
