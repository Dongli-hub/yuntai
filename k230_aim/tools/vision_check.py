# -*- coding: utf-8 -*-
"""vision_check.py —— K230 视觉调试工具

在 CanMV IDE 中运行，只做图像显示和识别验证，不与 H723 通信。
视觉处理与 main.py 保持同步，显示和预览逻辑只存在于本文件。

画面：融合图预览，绿框为外框四边形，红十为靶心，黄圈为激光位置。
终端：帧率、目标、光斑、各阶段耗时及无靶原因。
"""
import os
import time
import math

# ============================ 画面 / 显示 ============================
# --- 相机 / 处理分辨率 ---
# 像素常数按 S_SCALE 自动缩放；PROC_W 需要按 ISP 要求对齐。
PROC_W = 640
PROC_H = 480
IMG_W = PROC_W
IMG_H = PROC_H
S_SCALE = PROC_W / 640.0    # 像素常数缩放系数（640 档 = 1.0，800 档 = 1.25）
VIS_K = 7.8 * S_SCALE       # px/°（现场标定 7.8 @640x480，随分辨率同比缩放）
# --- 显示画布 ---
# 板载显示固定 640x480；处理图按比例缩放后绘制。
DISPLAY_W = 640
DISPLAY_H = 480
DISPLAY_QUALITY = 40
# True 显示反向二值化与 Canny 的融合图，False 显示原始画面。
BIN_PREVIEW = True
# True 显示未二值化的灰度图，用于检查各区域亮度。
GRAY_PREVIEW = False
# 短暂丢检时继续绘制上一帧框，仅影响预览。
DRAW_HOLD = 3
# --- 曝光 ---
# AE 先运行到稳定，再通过 VICAP dev attr 冻结；冻结后明显变暗则恢复 AE。
# 软件曝光使用 cv_lite，启动时标定一次固定 gain。
EXPO_FIX = True
EXPO_SETTLE_MS = 1200      # run() 后先让 AE 自适应多久再冻结
EXPO_DARK_RATIO = 0.55     # 冻结后亮度 < 冻结前×这个比例 -> 判定"变暗了"，回滚
EXPO_SW_GAIN = 1.0          # 运行时固定使用的软件曝光增益（自动标定会改写它）
EXPO_AUTO_GAIN = True       # 启动时按 EXPO_TARGET_MEAN 自动算 EXPO_SW_GAIN
EXPO_SW_MIN = 0.50          # 自动增益下限（再暗细节就没了）
EXPO_SW_MAX = 1.60          # 自动增益上限（别把噪声放大太多）
EXPO_HW_GAIN = 0            # 模拟增益（厂家示例 20；0 = 保持冻结值）
EXPO_TARGET_MEAN = 92.0     # 冻结后把 10x10 网格均亮度闭环到这个值
# 每隔几帧向 IDE 推送一次预览。
PREVIEW_EVERY = 1
CAM_VFLIP = True           # 画面上下颠倒 -> True
CAM_HMIRROR = False        # 画面左右镜像 -> True
# --- 检测频率 ---
CV2_EVERY = 1              # 每隔几帧运行一次目标检测

# --- 融合图与四边形参数 ---
GY_TH = 45
GY_TH_FAR = 45
GY_CLOSE_SIZE = 3
GY_CLOSE_RETRY = 5
GY_CANNY1 = 50
GY_CANNY2 = 150
PAPER_MIN_SIDE = int(round(20 * S_SCALE))
QUAD_ANGLE_COS_MAX = 0.50       # 相邻角 90°±30°
QUAD_OPPOSITE_DIFF_MAX = 0.40   # 对边长度差比例
QUAD_SIDE_RATIO_MAX = 10.0      # 最长边/最短边
GY_AREA_RATIO = 0.70            # 内框/外框面积比
# --- 激光（固定坐标）---
USE_FIXED_LASER = True
LASER_UV = (int(round(315 * S_SCALE)), int(round(246 * S_SCALE)))
PAPER_MIN_LONG = int(round(28 * S_SCALE))
PAPER_MAX_LONG = int(round(460 * S_SCALE))
PAPER_ASPECT_MIN = 0.70
PAPER_ASPECT_MAX = 2.00
PAPER_CONTRAST_MIN = 20         # 内侧亮度 - 外侧亮度
PAPER_DARK_MARGIN = 25          # 外侧点相对内部至少暗多少
PAPER_DARK_FRAC_MIN = 0.20
PAPER_INNER_MAX = 235           # 饱和亮块检查阈值
INTERIOR_TRANS_MIN = 4
PROBE_OFF_PX = int(round(5 * S_SCALE))
LOST_FULL = 25
SMOOTH = 0.90              # 平滑系数（越大越跟手、越小越稳）
DEADBAND_PX = 1.5 * S_SCALE   # 平滑死区（小于它不动，画面不抖）
# 连续拒收后允许重新锁定。
ACCEPT_RELOCK_N = 4
JUMP_REJECT_BASE = int(round(35 * S_SCALE))
SMOOTH_FAR = 0.60
FAR_LONG_PX = 120.0 * S_SCALE
PAPER_LONG_M = 0.275
SPOT_ROI_HALF = int(round(45 * S_SCALE))
SPOT_THRESHOLDS = [
    (88, 100, -40, 90, -50, 90),   # 过曝白芯（纸面 LAB-L≈78 到不了）
    (58, 100, 25, 90, -20, 70),    # 红色光晕
]
SPOT_MIN_AREA = 2
SPOT_MAX_AREA = 6000
SPOT_MAX_ASPECT = 3.0
SPOT_EVERY = 4
SPOT_ADAPT_GAIN = 0.06
SPOT_ADAPT_LIMIT = 6.0
SPOT_UV = [IMG_W / 2.0, IMG_H / 2.0]   # 最近一次找到的激光点，主循环每帧更新
# 可选阈值扫描：数值越大，反向二值图里的白色区域越多。
GY_TH_SWEEP = False
GY_TH_SWEEP_SEC = 5
GY_TH_SWEEP_LIST = [30, 35, 40, 45, 50, 56, 64]
# 激光位置默认使用 LASER_UV。开启标定后显示检测到的光斑并打印中位数；
# 标定值需要同步写回 main.py。
CALIB_LASER = False


# --- 距离估计 ---
FX_PX = 445.0 * S_SCALE

# ============================ 时序 ============================
PRINT_MS = 500
GC_EVERY = 30

def luma(r, g, b):
    return (r * 77 + g * 150 + b * 29) >> 8

def px_luma(img, x, y):
    """取指定像素亮度；优先读零拷贝数组，失败时退回 get_pixel。"""
    if x < 0:
        x = 0
    elif x >= IMG_W:
        x = IMG_W - 1
    if y < 0:
        y = 0
    elif y >= IMG_H:
        y = IMG_H - 1
    x = int(x)
    y = int(y)
    # ① 数组快路径（零拷贝）
    _arr = None
    try:
        _arr = img.to_numpy_ref()
    except Exception:
        _arr = None
    if _arr is not None:
        try:
            p = _arr[y, x]
            try:
                n = len(p)
            except TypeError:
                return int(p)          # 灰度图：标量
            if n >= 3:
                return luma(int(p[0]), int(p[1]), int(p[2]))
            if n == 1:
                return int(p[0])
        except Exception:
            pass
    # 兼容不支持数组访问的帧对象。
    try:
        p = img.get_pixel(x, y)
    except Exception:
        return -1
    if p is None:
        return -1
    try:
        n = len(p)
    except TypeError:
        try:
            return int(p)
        except Exception:
            return -1
    if n >= 3:
        return luma(int(p[0]), int(p[1]), int(p[2]))
    if n == 1:
        return int(p[0])
    return -1
# 分段耗时探针：建图、轮廓拟合和整条检测链。
_PROF = [0.0, 0, 0.0, 0, 0.0, 0]

def prof_line():
    """分段探针拼成一行（只读，不清零）。"""
    n2 = _PROF[5] if _PROF[5] > 0 else 1
    return ("轮廓%.0fms/次x%d[建图%.0f 拟合%.0f 扫描%.0f万px]"
            % (_PROF[4] / float(n2), _PROF[5],
               _PROF[0] / float(n2), _PROF[2] / float(n2),
               _PROF[1] / float(n2) / 10000.0))


def prof_reset():
    for _i in range(len(_PROF)):
        _PROF[_i] = 0
# ============================================================================
#  融合图矩形检测
# ============================================================================
_GY_ERR = [0]
# 本帧实际使用的反向二值化阈值。
_GY_TH_NOW = [0]
_SW_SCALED = [False]
_SW_FRAME = [0]

_SW_EXPO_ERR = [0]
_GY_SRC_DBG = [""]
# 融合失败计数（只报第一次）。
_GY_FUSE_ERR = [0]
# 融合图是否已成功建立。
_GY_MASK_OK = [False]
# 最近一次融合图，供预览和离线测试复用。
_GY_MASK_CACHE = [None]
# 最近一次检测的拒收原因。
_GY_DBG = [""]
# 闭运算异常计数（只报第一次）。
_CLOSE_ERR = [0]
# 输出护栏的拒收原因和连续拒收次数。
_ACC_DBG = [""]

def _pt_in_quad(p, c):
    """射线法判断点是否在四边形内。"""
    x, y = p[0], p[1]
    inside = False
    j = 3
    for i in range(4):
        xi, yi = c[i]
        xj, yj = c[j]
        if (yi > y) != (yj > y):
            _t = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < _t:
                inside = not inside
        j = i
    return inside

def _gy_sort_cyclic(c):
    """按绕质心的方位角排序，保证四个角点构成环序。"""
    cx = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4.0
    cy = (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4.0
    return sorted(c, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))

def _gy_quad_area(c):
    """四边形面积（鞋带公式）。"""
    _s = 0.0
    for i in range(4):
        x1, y1 = c[i]
        x2, y2 = c[(i + 1) % 4]
        _s += x1 * y2 - x2 * y1
    return abs(_s) * 0.5

def _gy_centroid4(c):
    """用鞋带公式计算四边形面积加权重心。"""
    a2 = 0.0
    cx = 0.0
    cy = 0.0
    for i in range(4):
        x1, y1 = c[i]
        x2, y2 = c[(i + 1) % 4]
        _cr = x1 * y2 - x2 * y1
        a2 += _cr
        cx += (x1 + x2) * _cr
        cy += (y1 + y2) * _cr
    if abs(a2) < 1e-9:
        return None
    return (cx / (3.0 * a2), cy / (3.0 * a2))

def _frame_gray(img):
    """应用软件曝光并转换为灰度图；不支持时退回原始帧转灰度。"""
    _g = None
    if (EXPO_SW_GAIN > 0) and (abs(EXPO_SW_GAIN - 1.0) > 1e-6):
        try:
            import cv_lite
            import image
            _rgb = img.to_numpy_ref()
            _ok = False
            try:
                _ok = (len(_rgb.shape) == 3) and \
                    (int(_rgb.shape[0]) == IMG_H) and \
                    (int(_rgb.shape[1]) == IMG_W)
            except Exception:
                _ok = False
            if _ok:
                _out = cv_lite.rgb888_adjust_exposure_fast(
                    [IMG_H, IMG_W], _rgb, float(EXPO_SW_GAIN))
                if _out is not None:
                    _eimg = image.Image(IMG_W, IMG_H, image.RGB888,
                                        alloc=image.ALLOC_REF, data=_out)
                    try:
                        _g = _eimg.to_grayscale()
                    except Exception:
                        _g = None
                    if _g is None:
                        # 部分固件不支持 ALLOC_REF 图直接转灰度。
                        try:
                            _gs = image.Image(IMG_W, IMG_H, image.GRAYSCALE)
                            _eimg.copy_to(_gs)
                            _g = _gs
                        except Exception:
                            _g = None
                    try:
                        del _eimg
                        del _out
                    except Exception:
                        pass
                    # 限制频繁回收造成的帧延迟。
                    _SW_FRAME[0] += 1
                    if (_SW_FRAME[0] & 3) == 0:
                        try:
                            import gc as _gc
                            _gc.collect()
                        except Exception:
                            pass
        except Exception as e:
            _SW_EXPO_ERR[0] += 1
            if _SW_EXPO_ERR[0] == 1:
                log("[EXPO] 软件曝光不可用(%s) -> 用原始帧转灰度" % e)
    if _g is None:
        try:
            _g = img.to_grayscale()
        except Exception:
            _g = None
    return _g

def _interior_trans(img, x, y, w, h):
    """统计内框水平/垂直中线的明暗翻转次数，用于排除均匀灯窗。"""
    _tot = 0
    for _h in (1, 0):
        _vals = []
        for _i in range(32):
            _f = _i / 31.0
            if _h:
                _px = int(x + w * _f)
                _py = int(y + h * 0.5)
            else:
                _px = int(x + w * 0.5)
                _py = int(y + h * _f)
            _vals.append(px_luma(img, _px, _py))
        _signs = []
        for _i in range(1, 32):
            _d = (_vals[_i] if _vals[_i] >= 0 else 0) - \
                (_vals[_i - 1] if _vals[_i - 1] >= 0 else 0)
            if abs(_d) >= 8:
                _signs.append(1 if _d > 0 else -1)
        for _i in range(1, len(_signs)):
            if _signs[_i] != _signs[_i - 1]:
                _tot += 1
    return _tot

def _mask_runs(np_img, fixed, row_mode, lo, hi, min_run):
    """在融合图的某一行/列上找连续白段（只认足厚的白带）。

    输入已经是「反向二值化 ∪ Canny」后的 0/255 图，不再回灰度。
    Canny 本身留下的细线只有 1~2px，而黑胶带在掩码里是一条厚白带；
    `min_run` 用来把 Canny 细线和真正胶带区分开。
    """
    runs = []
    start = -1
    i = lo
    while i < hi:
        if row_mode:
            v = int(np_img[fixed, i])
        else:
            v = int(np_img[i, fixed])
        if v >= 128:
            if start < 0:
                start = i
        elif start >= 0:
            if (i - start) >= min_run:
                runs.append((start, i - 1))
            start = -1
        i += 1
    if (start >= 0) and ((hi - start) >= min_run):
        runs.append((start, hi - 1))
    return runs

def _fit_line_pts(pts):
    """最小二乘/PCA 直线 A*x+B*y+C=0（对竖线、斜线都不退化）。"""
    n = len(pts)
    if n < 4:
        return None
    mx = 0.0
    my = 0.0
    for p in pts:
        mx += p[0]
        my += p[1]
    mx /= n
    my /= n
    sxx = 0.0
    sxy = 0.0
    syy = 0.0
    for p in pts:
        dx = p[0] - mx
        dy = p[1] - my
        sxx += dx * dx
        sxy += dx * dy
        syy += dy * dy
    if (sxx + syy) < 1e-6:
        return None
    ang = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    dx = math.cos(ang)
    dy = math.sin(ang)
    a = -dy
    b = dx
    c = -(a * mx + b * my)
    return (a, b, c)

def _fit_line_pts_robust(pts):
    """外轮廓边线专用：去掉约 1/3 最远点后再拟合。

    行/列扫描在靶框旋转或背景有竖直白边时会在个别扫描线摸到另一条边；
    外轮廓必须由多数点决定，内轮廓仍用普通最小二乘。
    """
    if len(pts) < 5:
        return None
    line = _fit_line_pts(pts)
    if line is None:
        return None
    a, b, c = line
    norm = math.sqrt(a * a + b * b) + 1e-9
    scored = []
    for p in pts:
        d = abs(a * p[0] + b * p[1] + c) / norm
        scored.append((d, p))
    scored.sort(key=lambda q: q[0])
    keep = max(5, (len(scored) * 2) // 3)
    return _fit_line_pts([q[1] for q in scored[:keep]])

def _line_isect(l1, l2):
    if (l1 is None) or (l2 is None):
        return None
    a1, b1, c1 = l1
    a2, b2, c2 = l2
    det = a1 * b2 - a2 * b1
    if abs(det) < 1e-6:
        return None
    x = (-c1 * b2 + b1 * c2) / det
    y = (-a1 * c2 + c1 * a2) / det
    return (x, y)

def _quad_convex(c):
    """四个点按环序排列时是否始终同向转弯。"""
    if c is None:
        return False
    pos = 0
    neg = 0
    for i in range(4):
        ax, ay = c[i]
        bx, by = c[(i + 1) % 4]
        cx, cy = c[(i + 2) % 4]
        z = (bx - ax) * (cy - by) - (by - ay) * (cx - bx)
        if z > 0.5:
            pos += 1
        elif z < -0.5:
            neg += 1
    return (pos == 4) or (neg == 4)

def _quad_sides(c):
    out = []
    for i in range(4):
        ax, ay = c[i]
        bx, by = c[(i + 1) % 4]
        dx = bx - ax
        dy = by - ay
        out.append(math.sqrt(dx * dx + dy * dy))
    return out

def _quad_geom_ok(c):
    """旋转不变的四边形检查：边长、邻角和对边比例。"""
    if (c is None) or (not _quad_convex(c)):
        return False
    s = _quad_sides(c)
    smin = min(s)
    smax = max(s)
    if (smin < PAPER_MIN_SIDE) or (smax > QUAD_SIDE_RATIO_MAX * smin):
        return False
    # 对边：s0/s2、s1/s3
    if abs(s[0] - s[2]) > QUAD_OPPOSITE_DIFF_MAX * max(s[0], s[2]):
        return False
    if abs(s[1] - s[3]) > QUAD_OPPOSITE_DIFF_MAX * max(s[1], s[3]):
        return False
    # 相邻边夹角余弦（内角 90°±30°）
    for i in range(4):
        ax, ay = c[i]
        bx, by = c[(i + 1) % 4]
        px, py = c[(i - 1) % 4]
        v1x = bx - ax
        v1y = by - ay
        v2x = px - ax
        v2y = py - ay
        n1 = math.sqrt(v1x * v1x + v1y * v1y)
        n2 = math.sqrt(v2x * v2x + v2y * v2y)
        if (n1 < 1e-6) or (n2 < 1e-6):
            return False
        if abs((v1x * v2x + v1y * v2y) / (n1 * n2)) > QUAD_ANGLE_COS_MAX:
            return False
    return True

def _boundary_points(np_img, bx, by, bw, bh, hi=True):
    """在连通域框内取外边界采样点（行左右沿 + 列上下沿）。"""
    pts = []
    x0 = max(1, int(bx))
    y0 = max(1, int(by))
    x1 = min(IMG_W - 2, int(bx + bw) - 1)
    y1 = min(IMG_H - 2, int(by + bh) - 1)
    if (x1 - x0 < 8) or (y1 - y0 < 8):
        return pts
    step = max(1, int(max(bw, bh) / 40.0))
    y = y0
    while y <= y1:
        left = -1
        right = -1
        x = x0
        while x <= x1:
            v = int(np_img[y, x])
            on = (v >= 128) if hi else (v < 128)
            if on:
                if left < 0:
                    left = x
                right = x
            x += 1
        if left >= 0:
            pts.append((left, y))
            pts.append((right, y))
        y += step
    x = x0
    while x <= x1:
        top = -1
        bot = -1
        y = y0
        while y <= y1:
            v = int(np_img[y, x])
            on = (v >= 128) if hi else (v < 128)
            if on:
                if top < 0:
                    top = y
                bot = y
            y += 1
        if top >= 0:
            pts.append((x, top))
            pts.append((x, bot))
        x += step
    if len(pts) < 8:
        return pts
    # 用中位数中心做一次径向离群过滤，避免粘连背景把凸包拉偏。
    _xs = sorted(p[0] for p in pts)
    _ys = sorted(p[1] for p in pts)
    _mx = _xs[len(_xs) // 2]
    _my = _ys[len(_ys) // 2]
    _scored = []
    for p in pts:
        _dx = p[0] - _mx
        _dy = p[1] - _my
        _scored.append((math.sqrt(_dx * _dx + _dy * _dy), p))
    _scored.sort(key=lambda q: q[0])
    _r = _scored[int((len(_scored) - 1) * 0.85)][0] * 1.15
    return [p for d, p in _scored if d <= _r]

def _cross3(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - \
           (a[1] - o[1]) * (b[0] - o[0])

def _convex_hull(points):
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts
    lower = []
    for p in pts:
        while (len(lower) >= 2) and \
                (_cross3(lower[-2], lower[-1], p) <= 0):
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while (len(upper) >= 2) and \
                (_cross3(upper[-2], upper[-1], p) <= 0):
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]

def _pt_seg_dist(p, a, b):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    den = dx * dx + dy * dy
    if den <= 1e-9:
        return math.sqrt((p[0] - a[0]) ** 2 + (p[1] - a[1]) ** 2)
    t = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / den
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    qx = a[0] + t * dx
    qy = a[1] + t * dy
    return math.sqrt((p[0] - qx) ** 2 + (p[1] - qy) ** 2)

def _rdp_open(pts, eps):
    if len(pts) < 3:
        return list(pts)
    dmax = -1.0
    idx = 0
    a = pts[0]
    b = pts[-1]
    for i in range(1, len(pts) - 1):
        d = _pt_seg_dist(pts[i], a, b)
        if d > dmax:
            dmax = d
            idx = i
    if dmax > eps:
        left = _rdp_open(pts[:idx + 1], eps)
        right = _rdp_open(pts[idx:], eps)
        return left[:-1] + right
    return [a, b]

def _approx_quad(points):
    hull = _convex_hull(points)
    if len(hull) < 4:
        return None
    peri = 0.0
    for i in range(len(hull)):
        a = hull[i]
        b = hull[(i + 1) % len(hull)]
        peri += math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
    # 从最远点把闭合轮廓拆成两条开放链，分别 RDP（等价 approxPolyDP 闭轮廓）。
    i0 = hull[0]
    far = 1
    fd = -1.0
    for i in range(1, len(hull)):
        d = (hull[i][0] - i0[0]) ** 2 + (hull[i][1] - i0[1]) ** 2
        if d > fd:
            fd = d
            far = i
    for ratio in (0.005, 0.008, 0.01, 0.015, 0.02, 0.025, 0.03,
                  0.04, 0.05):
        eps = peri * ratio
        c1 = _rdp_open(hull[:far + 1], eps)
        c2 = _rdp_open(hull[far:] + [hull[0]], eps)
        ap = c1[:-1] + c2[:-1]
        if len(ap) == 4:
            q = _gy_sort_cyclic(ap)
            if _quad_convex(q):
                return q
        if len(ap) < 4:
            continue
    return None

def _hull_quad_from_bbox(np_img, bx, by, bw, bh, hi=True):
    pts = _boundary_points(np_img, bx, by, bw, bh, hi)
    if len(pts) < 12:
        return None
    return _approx_quad(pts)

def _mask_hull_pair(np_img, bx, by, bw, bh, inner_blob):
    """由外边界拟合四边形，再用内孔中心和面积比生成内四边形。"""
    qo = _hull_quad_from_bbox(np_img, bx, by, bw, bh, True)
    if (qo is None) or (not _quad_geom_ok(qo)):
        return None
    if inner_blob is None:
        return None
    ix, iy, iw, ih = inner_blob[0], inner_blob[1], inner_blob[2], inner_blob[3]
    if (iw < 8) or (ih < 8):
        return None
    co = _gy_centroid4(qo)
    if co is None:
        return None
    cx = ix + iw * 0.5
    cy = iy + ih * 0.5
    if not _pt_in_quad((cx, cy), qo):
        return None
    _br = (iw * ih) / float(max(1, bw * bh))
    _scl = math.sqrt(max(0.10, min(0.95, _br)))
    qi = _gy_sort_cyclic([
        (cx + (p[0] - co[0]) * _scl,
         cy + (p[1] - co[1]) * _scl) for p in qo])
    if (qi is None) or (not _quad_convex(qi)):
        return None
    ao = _gy_quad_area(qo)
    ai = _gy_quad_area(qi)
    if (ao < 1.0) or (ai < 1.0):
        return None
    ratio = ai / ao
    if (ratio < GY_AREA_RATIO) or (ratio > 0.97):
        return None
    ci = _gy_centroid4(qi)
    if (ci is None) or (not _pt_in_quad(ci, qo)):
        return None
    return (qo, qi)

def _pick_inner_blob(inv_blobs, bx, by, bw, bh):
    """在外框 bbox 内选最大的、未触边界的黑色内孔连通域。"""
    best = None
    if not inv_blobs:
        return None
    for b in inv_blobs:
        ix = int(b[0])
        iy = int(b[1])
        iw = int(b[2])
        ih = int(b[3])
        ia = int(b[4])
        if (ix <= bx + 2) or (iy <= by + 2) or \
                (ix + iw >= bx + bw - 2) or \
                (iy + ih >= by + bh - 2):
            continue
        asp = max(iw, ih) / float(max(1, min(iw, ih)))
        if (asp < 0.30) or (asp > 3.2):
            continue
        if (best is None) or (ia > best[4]):
            best = (ix, iy, iw, ih, ia)
    return best

def _mask_band_quad(np_img, bx, by, bw, bh):
    """细框断口兜底：扫描行/列白带并拟合内外四边形。

    正常路径使用 `_mask_hull_pair`；只有内孔没有形成独立连通域时才调用。
    """
    x0 = max(2, int(bx))
    y0 = max(2, int(by))
    x1 = min(IMG_W - 3, int(bx + bw) - 1)
    y1 = min(IMG_H - 3, int(by + bh) - 1)
    if (x1 - x0 < 12) or (y1 - y0 < 12):
        return None
    # 收集白行程，并把相邻 3px 内的 Canny 线并入同一条边带。
    min_run = 1
    ns = 14
    left_o = []
    left_i = []
    right_i = []
    right_o = []
    top_o = []
    top_i = []
    bot_i = []
    bot_o = []
    for k in range(ns):
        f = 0.12 + 0.76 * k / float(ns - 1)
        y = int(y0 + (y1 - y0) * f)
        runs = _mask_runs(np_img, y, True, x0, x1 + 1, min_run)
        if len(runs) >= 2:
            # 从外侧向内合并相邻白段。
            li_ix = 0
            l_end = runs[0][1]
            while (li_ix + 1 < len(runs)) and \
                    (runs[li_ix + 1][0] - l_end <= 3):
                li_ix += 1
                l_end = runs[li_ix][1]
            # 从右侧向内合并相邻白段。
            ri_ix = len(runs) - 1
            r_start = runs[ri_ix][0]
            while (ri_ix > 0) and \
                    (r_start - runs[ri_ix - 1][1] <= 3):
                ri_ix -= 1
                r_start = runs[ri_ix][0]
            l_out = runs[0][0]
            r_out = runs[-1][1]
            l_in = l_end
            r_in = r_start
            # 细框的内外边缘可能是两条独立的 Canny 线。
            _span = max(8, int(round(30 * S_SCALE)))
            if (l_end - l_out <= 2) and (li_ix + 1 < len(runs)) and \
                    (runs[li_ix + 1][0] - l_end <= _span):
                l_in = runs[li_ix + 1][1]
            if (r_out - r_start <= 2) and (ri_ix > 0) and \
                    (r_start - runs[ri_ix - 1][1] <= _span):
                r_in = runs[ri_ix - 1][0]
            if (l_in + 2) < r_in:
                left_o.append((l_out, y))
                left_i.append((l_in, y))
                right_i.append((r_in, y))
                right_o.append((r_out, y))
        x = int(x0 + (x1 - x0) * f)
        runs = _mask_runs(np_img, x, False, y0, y1 + 1, min_run)
        if len(runs) >= 2:
            ti_ix = 0
            t_end = runs[0][1]
            while (ti_ix + 1 < len(runs)) and \
                    (runs[ti_ix + 1][0] - t_end <= 3):
                ti_ix += 1
                t_end = runs[ti_ix][1]
            bi_ix = len(runs) - 1
            b_start = runs[bi_ix][0]
            while (bi_ix > 0) and \
                    (b_start - runs[bi_ix - 1][1] <= 3):
                bi_ix -= 1
                b_start = runs[bi_ix][0]
            t_out = runs[0][0]
            b_out = runs[-1][1]
            t_in = t_end
            b_in = b_start
            _span = max(8, int(round(30 * S_SCALE)))
            if (t_end - t_out <= 2) and (ti_ix + 1 < len(runs)) and \
                    (runs[ti_ix + 1][0] - t_end <= _span):
                t_in = runs[ti_ix + 1][1]
            if (b_out - b_start <= 2) and (bi_ix > 0) and \
                    (b_start - runs[bi_ix - 1][1] <= _span):
                b_in = runs[bi_ix - 1][0]
            if (t_in + 2) < b_in:
                top_o.append((x, t_out))
                top_i.append((x, t_in))
                bot_i.append((x, b_in))
                bot_o.append((x, b_out))
    if (len(left_o) < 5) or (len(right_o) < 5) or \
            (len(top_o) < 5) or (len(bot_o) < 5):
        return None
    lo = _fit_line_pts_robust(left_o)
    li = _fit_line_pts(left_i)
    ri = _fit_line_pts(right_i)
    ro = _fit_line_pts_robust(right_o)
    to = _fit_line_pts_robust(top_o)
    ti = _fit_line_pts(top_i)
    bi = _fit_line_pts(bot_i)
    bo = _fit_line_pts_robust(bot_o)
    if None in (lo, li, ri, ro, to, ti, bi, bo):
        return None

    def _mkq(lt, lb, ll, lr):
        p1 = _line_isect(lt, ll)
        p2 = _line_isect(lt, lr)
        p3 = _line_isect(lb, lr)
        p4 = _line_isect(lb, ll)
        if None in (p1, p2, p3, p4):
            return None
        return _gy_sort_cyclic([p1, p2, p3, p4])

    qo = _mkq(to, bo, lo, ro)
    qi = _mkq(ti, bi, li, ri)
    if (qo is None) or (qi is None):
        return None
    # 外框做完整几何检查；内框来自面积比缩放，只检查凸性和包含关系。
    if not _quad_geom_ok(qo):
        return None
    if not _quad_convex(qi):
        return None
    ao = _gy_quad_area(qo)
    ai = _gy_quad_area(qi)
    if (ao < 1.0) or (ai < 1.0):
        return None
    ratio = ai / ao
    if (ratio < 0.45) or (ratio > 0.96):
        return None
    co = _gy_centroid4(qo)
    ci = _gy_centroid4(qi)
    if (co is None) or (ci is None):
        return None
    if not _pt_in_quad(ci, qo):
        return None
    # 透视下允许少量偏移，但内外框仍应基本同心。
    if abs(co[0] - ci[0]) + abs(co[1] - ci[1]) > 0.35 * max(bw, bh):
        return None
    return (qo, qi)

def _mask_band_quads(fi, np_img, hint=None):
    """从融合图提取独立连通域，并拟合内外四边形。"""
    raw = None
    blobs = []
    seen = {}
    try:
        # 连通域来自低阈值二值区与 Canny 边缘的融合图。
        import cv_lite
        raw = cv_lite.grayscale_find_blobs(
            [IMG_H, IMG_W], np_img, 128, 255, 80, 1)
    except Exception:
        raw = None
    if raw is not None:
        for _i in range(0, len(raw) - 3, 4):
            _b4 = (int(raw[_i]), int(raw[_i + 1]),
                   int(raw[_i + 2]), int(raw[_i + 3]))
            if _b4 not in seen:
                seen[_b4] = 1
                blobs.append(_b4)
    # 两套连通域实现的面积定义不同，并联可补回细轮廓。
    try:
        _ib = fi.find_blobs([(128, 255)], roi=None, merge=False,
                            margin=0, pixels_threshold=80,
                            area_threshold=80)
    except Exception:
        _ib = None
    if _ib:
        for b in _ib:
            _b4 = (int(b[0]), int(b[1]), int(b[2]), int(b[3]))
            if _b4 not in seen:
                seen[_b4] = 1
                blobs.append(_b4)
    if not blobs:
        # 两块中心 ROI 仍会在下面执行，不在这里提前返回。
        blobs = []
    # 反相后的内孔白块用于定位内框。
    inv_blobs = []
    try:
        _inv = fi.copy()
        _inv.binary([(0, 127)])
        inv_blobs = _inv.find_blobs([(128, 255)], roi=None, merge=False,
                                    margin=0, pixels_threshold=60,
                                    area_threshold=60)
    except Exception:
        inv_blobs = []
    cand = []
    pref = hint[0] if hint is not None else 0.0
    hu = hint[1] if hint is not None else SPOT_UV[0]
    hv = hint[2] if hint is not None else SPOT_UV[1]
    for b in blobs:
        bx, by, bw, bh = b
        _long = max(bw, bh)
        _short = min(bw, bh)
        if (_short < 12) or (_long < PAPER_MIN_LONG) or \
                (_long > PAPER_MAX_LONG):
            continue
        _asp = _long / float(_short)
        if (_asp < 0.55) or (_asp > 2.4):
            continue
        _cx = bx + bw * 0.5
        _cy = by + bh * 0.5
        _sim = -abs(_long - pref) if (pref > 1.0) else 0.0
        _dc = abs(_cx - hu) + abs(_cy - hv)
        cand.append(((_sim, bw * bh, -_dc), bx, by, bw, bh))
    cand.sort(reverse=True)
    out = []
    # 锁定态只拟合前 4 个；搜索态保留 8 个覆盖同心环/杂波。
    _limit = 4 if hint is not None else 8
    for _key, bx, by, bw, bh in cand[:_limit]:
        _inner_blob = _pick_inner_blob(inv_blobs, bx, by, bw, bh)
        q = _mask_hull_pair(np_img, bx, by, bw, bh, _inner_blob) \
            if (_inner_blob is not None) else None
        if q is None:
            # 没有闭合内孔时才使用边带扫描。
            q = _mask_band_quad(np_img, bx, by, bw, bh)
        if q is not None:
            out.append(q)
            if hint is not None:
                _qxs = [p[0] for p in q[0]]
                _qys = [p[1] for p in q[0]]
                _ql = max(max(_qxs) - min(_qxs), max(_qys) - min(_qys))
                if abs(_ql - hint[0]) <= 0.06 * max(1.0, hint[0]):
                    break
    # 白框与背景粘连时，围绕激光点/上一帧中心补两块 ROI。
    if not out:
        for _rw, _rh in ((int(IMG_W * 0.82), int(IMG_H * 0.84)),
                         (int(IMG_W * 0.68), int(IMG_H * 0.70))):
            _bx = int(max(2, min(IMG_W - _rw - 2, hu - _rw * 0.5)))
            _by = int(max(2, min(IMG_H - _rh - 2, hv - _rh * 0.5)))
            _inner_blob = _pick_inner_blob(inv_blobs, _bx, _by, _rw, _rh)
            q = _mask_hull_pair(np_img, _bx, _by, _rw, _rh, _inner_blob) \
                if (_inner_blob is not None) \
                else None
            if q is None:
                q = _mask_band_quad(np_img, _bx, _by, _rw, _rh)
            if q is not None:
                out.append(q)
    return out

def _cv2_find(img, hint=None, th=None, mask_out=None):
    """在融合图上检测靶纸，返回外框、靶心和形状指标。

    流程：灰度 -> 反向二值化 -> 闭运算 -> Canny -> 融合图
          -> 白色连通域 -> 外边界凸包/RDP 四边 -> 内孔与几何过滤。

    hint 仅用于候选排序；mask_out 非空时返回融合图供调试工具使用。
    识别失败返回 None。
    """
    try:
        import cv_lite
        import image
    except Exception as e:
        _GY_ERR[0] += 1
        if _GY_ERR[0] == 1:
            log("[CV2] cv_lite/image 不可用(%s) -> 退回色块/矩形候选路" % e)
        return None
    _t = time.ticks_ms()
    try:
        # 当前帧已经是软件曝光后的灰度图。
        gray_np = img.to_numpy_ref()
        gi = None
        try:
            gi = img.copy()          # 在副本上处理，不修改相机帧
        except Exception:
            gi = None
        if gi is None:
            gi = img.to_grayscale()  # 兜底：至少换一张图
        if gi is None:
            gi = img
        edges_np = None
        try:
            edges_np = cv_lite.grayscale_find_edges(
                [IMG_H, IMG_W], gray_np, GY_CANNY1, GY_CANNY2)
        except Exception:
            edges_np = None
        # 反向二值化：暗部变为白色前景。
        _th = GY_TH if th is None else int(th)
        _GY_TH_NOW[0] = _th
        if _th <= 1:
            gi.binary([(0, 0)])
        else:
            gi.binary([(0, _th - 1)])
        # 闭运算补断口。
        if GY_CLOSE_SIZE > 0:
            try:
                gi.close(GY_CLOSE_SIZE)
            except Exception as e:
                if _CLOSE_ERR[0] == 0:
                    _CLOSE_ERR[0] = 1
                    log("[CV2] close(%d) 失败(%s) -> 跳过闭运算"
                        % (GY_CLOSE_SIZE, e))
        # 解包为 8bit 0/255 灰度，供连通域和融合使用。
        gn = gi
        try:
            _g2 = gi.to_grayscale()
            if _g2 is not None:
                gn = _g2
        except Exception:
            pass
        fi = gn
        if edges_np is not None:
            try:
                _fc = gn.copy()
                if _fc is not None:
                    fi = _fc
            except Exception:
                fi = gn
            _ferr = None
            try:
                ei = image.Image(IMG_W, IMG_H, image.GRAYSCALE,
                                 alloc=image.ALLOC_REF, data=edges_np)
                fi.b_or(ei)                   # 首选：按位或（就地改）
            except Exception as e:
                _ferr = e
                try:
                    _r = fi.add(ei)           # 兜底：饱和加（白+白=白）
                    if _r is not None:
                        fi = _r
                    _ferr = None
                except Exception:
                    pass
            if _ferr is not None:
                _GY_FUSE_ERR[0] += 1
                if _GY_FUSE_ERR[0] == 1:
                    log("[CV2] 融合(b_or/add)失败(%s) -> 只用二值图" % _ferr)
        _PROF[0] += time.ticks_diff(time.ticks_ms(), _t)
        try:
            if (mask_out is not None) and (fi is not None):
                mask_out[0] = fi
            if not _GY_MASK_OK[0]:
                if fi is not None:
                    log("[CV2] 融合图就绪 th=%d" % _th)
                else:
                    log("[CV2] 融合图没建起来（异常），本帧不检测")
            _GY_MASK_OK[0] = True
        except Exception:
            pass
        # 后续检测只读取这张融合图，不再回灰度重跑 Canny。
        dnp = None
        try:
            _dc = fi.copy()
            if _dc is not None:
                dnp = _dc.to_numpy_ref()
        except Exception:
            dnp = None
        if dnp is not None:
            # 只接受与处理分辨率一致的整幅灰度数组。
            try:
                if (int(dnp.shape[0]) != IMG_H) or \
                        (int(dnp.shape[1]) != IMG_W):
                    dnp = None
            except Exception:
                dnp = None
        _GY_SRC_DBG[0] = "融"
        if dnp is None:
            _GY_DBG[0] = "r0掩"
            return None
        _t2 = time.ticks_ms()
        bands = _mask_band_quads(fi, dnp, hint)
        _PROF[3] += 1
        _PROF[1] += IMG_W * IMG_H
        # 主核未检出时，用备用闭运算核重建一次融合图。
        if (not bands) and \
                (GY_CLOSE_RETRY > 0) and \
                (GY_CLOSE_RETRY != GY_CLOSE_SIZE):
            _g2i = None
            try:
                _g2i = img.copy()
            except Exception:
                _g2i = None
            if _g2i is not None:
                try:
                    _g2i.binary([(0, _th - 1)])
                    _g2i.close(GY_CLOSE_RETRY)
                    _gn2 = _g2i.to_grayscale()
                    if _gn2 is None:
                        _gn2 = _g2i
                    _f2 = _gn2
                    try:
                        _f2 = _gn2.copy()
                    except Exception:
                        _f2 = _gn2
                    if edges_np is not None:
                        try:
                            _e2 = image.Image(IMG_W, IMG_H, image.GRAYSCALE,
                                              alloc=image.ALLOC_REF,
                                              data=edges_np)
                            _f2.b_or(_e2)
                        except Exception:
                            pass
                    _n2 = _f2.to_numpy_ref()
                    bands = _mask_band_quads(_f2, _n2, hint)
                    _PROF[3] += 1
                    _PROF[1] += IMG_W * IMG_H
                    if bands:
                        _GY_SRC_DBG[0] = "融5"
                except Exception:
                    pass
                try:
                    del _g2i
                except Exception:
                    pass
        _PROF[2] += time.ticks_diff(time.ticks_ms(), _t2)
        try:
            del _dc
        except Exception:
            pass
        if not bands:
            _GY_DBG[0] = "r0%s" % _GY_SRC_DBG[0]
            return None

        # 每个候选已经包含同组内外四边形。
        _pairs = []
        for _qo, _qi in bands:
            _pairs.append((_gy_quad_area(_qi), _qi, _qo))
        if not _pairs:
            _GY_DBG[0] = "p0"
            return None
        # 多个候选按对比度、暗边、尺度和中心位置筛选。
        _best = None
        for _pj in _pairs:
            _ci = _pj[1]
            _xs = (_ci[0][0], _ci[1][0], _ci[2][0], _ci[3][0])
            _ys = (_ci[0][1], _ci[1][1], _ci[2][1], _ci[3][1])
            _bx = int(min(_xs))
            _by = int(min(_ys))
            _bw = int(max(_xs)) - _bx
            _bh = int(max(_ys)) - _by
            if (_bw < 8) or (_bh < 8):
                continue
            _i2, _o2, _f2 = paper_contrast(img, _bx, _by, _bw, _bh)
            if (_i2 < 0) or ((_i2 - _o2) < PAPER_CONTRAST_MIN):
                _GY_DBG[0] = "c"
                continue
            if _f2 < PAPER_DARK_FRAC_MIN:
                _GY_DBG[0] = "d"
                continue
            # 饱和且内部完全均匀的亮块通常是灯或窗。
            if (_i2 >= PAPER_INNER_MAX) and \
                    (_interior_trans(img, _bx, _by, _bw, _bh) <
                     INTERIOR_TRANS_MIN):
                _GY_DBG[0] = "t"
                continue
            _cx = _bx + _bw / 2.0
            _cy = _by + _bh / 2.0
            _dc = abs(_cx - SPOT_UV[0]) + abs(_cy - SPOT_UV[1])
            _oc = _pj[2]
            _oxs = (_oc[0][0], _oc[1][0], _oc[2][0], _oc[3][0])
            _oys = (_oc[0][1], _oc[1][1], _oc[2][1], _oc[3][1])
            _ol = max(max(_oxs) - min(_oxs), max(_oys) - min(_oys))
            prefer_long = hint[0] if hint is not None else 0.0
            if prefer_long > 1.0:
                _sim = -abs(_ol - prefer_long)
            else:
                _sim = 0.0
            _dp = 0.0
            if (hint is not None) and (hint[0] > 1.0):
                _dp = abs(_cx - hint[1]) + abs(_cy - hint[2])
            # 优先同尺度、嵌套关系和大面积。
            _key = (_sim, 1 if _pj[1] is not _pj[2] else 0,
                    _gy_quad_area(_pj[1]), -_dc, -_dp)
            if (_best is None) or (_key > _best[0]):
                _best = (_key, _pj)
        if _best is None:
            return None          # _GY_DBG 上一轮循环里已经写好了
        _inner = _best[1][1]          # 内框（纸的边界）
        _outer = _best[1][2]          # 对外输出的外框
        _ci = _gy_centroid4(_inner)   # 靶心 = 内框重心
        if _ci is None:
            return None
        _xs = (_outer[0][0], _outer[1][0], _outer[2][0], _outer[3][0])
        _ys = (_outer[0][1], _outer[1][1], _outer[2][1], _outer[3][1])
        _bx = int(min(_xs))
        _by = int(min(_ys))
        _bw = int(max(_xs)) - _bx
        _bh = int(max(_ys)) - _by
        if (_bw < 8) or (_bh < 8):
            _GY_DBG[0] = "s1"
            return None
        _long = max(_bw, _bh)
        _short = max(1, min(_bw, _bh))
        _aspect = _long / float(_short)
        if (_long < PAPER_MIN_LONG) or (_long > PAPER_MAX_LONG):
            _GY_DBG[0] = "s%d" % _long
            return None
        if (_aspect < PAPER_ASPECT_MIN) or (_aspect > PAPER_ASPECT_MAX):
            _GY_DBG[0] = "a%.2f" % _aspect
            return None
        _pxa = _gy_quad_area(_outer)
        # _dens 只用于日志，不再作为筛选闸门。
        _ss = _quad_sides(_outer)
        _dens = min(_ss) / max(_ss) if max(_ss) > 0 else 1.0
        _ins, _outs, _ = paper_contrast(img, _bx, _by, _bw, _bh)
        return (_pxa, _bx, _by, _bw, _bh, _long, _aspect, _dens,
                        _ins, _outs, tuple(_outer), _ci[0], _ci[1])
    except Exception as e:
        _GY_ERR[0] += 1
        if _GY_ERR[0] == 1:
            log("[CV2] 融合图检测异常(%s)" % e)
        return None
    finally:
        _PROF[4] += time.ticks_diff(time.ticks_ms(), _t)
        _PROF[5] += 1

def log_scene(img):
    """采样画面亮度分布，用于判断无靶是曝光、视野还是判据问题。"""
    _vals = []
    for gy in range(12):
        y = int(IMG_H * (gy + 0.5) / 12)
        for gx in range(10):
            x = int(IMG_W * (gx + 0.5) / 10)
            v = px_luma(img, x, y)
            if v >= 0:
                _vals.append(v)
    if _vals:
        _vals.sort()
        k = len(_vals)
        s = 0
        for v in _vals:
            s += v
        lo = _vals[0]
        hi = _vals[k - 1]
        p25 = _vals[k * 25 // 100]
        p50 = _vals[k * 50 // 100]
        p75 = _vals[k * 75 // 100]
        log("[SCENE] 画面亮度 均=%.0f 小=%d 大=%d P25=%d P50=%d P75=%d (0~255) "
            "（当前阈值 th=%d：反向二值化里白的=比它暗，胶带落在 P25 那档、"
            "木柜在 P50 那档）  靶=无靶时看这行"
            % (s / float(k), lo, hi, p25, p50, p75,
               (_GY_TH_NOW[0] if _GY_TH_NOW[0] > 0 else GY_TH)))

def paper_contrast(img, x, y, w, h):
    """返回内部均值、外侧均值和外侧暗点比例。"""
    u = ((x + w * 0.30, y + h * 0.50), (x + w * 0.70, y + h * 0.50),
         (x + w * 0.50, y + h * 0.30), (x + w * 0.50, y + h * 0.70),
         (x + w * 0.50, y + h * 0.50))
    o = ((x - PROBE_OFF_PX, y + h * 0.25), (x - PROBE_OFF_PX, y + h * 0.50),
         (x - PROBE_OFF_PX, y + h * 0.75),
         (x + w + PROBE_OFF_PX, y + h * 0.25),
         (x + w + PROBE_OFF_PX, y + h * 0.50),
         (x + w + PROBE_OFF_PX, y + h * 0.75),
         (x + w * 0.25, y - PROBE_OFF_PX), (x + w * 0.50, y - PROBE_OFF_PX),
         (x + w * 0.75, y - PROBE_OFF_PX),
         (x + w * 0.25, y + h + PROBE_OFF_PX),
         (x + w * 0.50, y + h + PROBE_OFF_PX),
         (x + w * 0.75, y + h + PROBE_OFF_PX))
    si = 0
    so = 0
    ni = 0
    no = 0
    for p in u:
        v = px_luma(img, p[0], p[1])
        if v >= 0:
            si += v
            ni += 1
    for p in o:
        v = px_luma(img, p[0], p[1])
        if v >= 0:
            so += v
            no += 1
    if ni < 3 or no < 6:
        return -1, -1, 0.0
    ins = si / float(ni)
    ok = 0
    for p in o:
        v = px_luma(img, p[0], p[1])
        if v >= 0 and v < (ins - PAPER_DARK_MARGIN):
            ok += 1
    return ins, so / float(no), ok / float(no)

class PaperDetector(object):
    """靶纸检测、时间一致性过滤和平滑。"""

    def __init__(self):
        self.u = IMG_W / 2.0
        self.v = IMG_H / 2.0
        self.w = 0.0
        self.h = 0.0
        self.have = False
        self.lost = 0
        self.meas = None
        self.corners = None
        self.frame = 0
        self.last_n = 0
        self.last_dbg = ""
        self.state = "搜索"
        self.err = 0
        self.err_msg = ""
        self.last_res = None
        # 平滑中心每帧位移。
        self.mot_px = 0.0
        # 原始候选位移，被拒帧也更新，避免快速运动时护栏锁死。
        self.raw_u = None
        self.raw_v = None
        self.cand_mot = 0.0
        self.rej_n = 0             # 连续拒收帧数
        self.last_long = 0.0       # 上一帧采纳的外框长边
        self.pend_long = 0.0
        self.pend_n = 0

    def fresh(self):
        return self.have and (self.lost == 0)

    def _accept(self, cand):
        """过滤中心跳变和尺度突变；连续拒收后允许重新锁定。"""
        w, h = cand[3], cand[4]
        c = cand[10]
        cu, cv = cand[11], cand[12]      # 四边形对角线交点 = 透视中心
        if self.raw_u is not None:
            _d = abs(cu - self.raw_u)
            _d2 = abs(cv - self.raw_v)
            if _d2 > _d:
                _d = _d2
            self.cand_mot = (0.6 * self.cand_mot + 0.4 * _d) \
                if self.cand_mot > 0 else _d
        self.raw_u, self.raw_v = cu, cv
        if self.have and (self.lost < LOST_FULL):
            _mv = self.mot_px if self.mot_px > self.cand_mot else self.cand_mot
            if _mv > 120.0:
                _mv = 120.0
            _jl = JUMP_REJECT_BASE + int(2.0 * _mv)
            if self.rej_n >= 2:
                _jl *= 2
            if self.rej_n >= 5:
                _jl *= 4
            _bad = None
            if abs(cu - self.u) > _jl:
                _bad = "跳"
            elif abs(cv - self.v) > _jl:
                _bad = "跳"
            else:
                old_l = self.last_long if self.last_long > 1.0 \
                    else max(self.w, self.h)
                if old_l > 1.0:
                    _dl = abs(cand[5] - old_l) / old_l
                    # 静止时尺度容差 8%，快速运动时放宽到 20%。
                    _scale_tol = 0.20 if _mv > 10.0 else 0.08
                    if _dl > _scale_tol:
                        # 连续 3 帧接近新尺度才确认换框。
                        if (self.pend_long > 1.0) and \
                                (abs(cand[5] - self.pend_long) <=
                                 0.05 * self.pend_long):
                            self.pend_n += 1
                        else:
                            self.pend_long = cand[5]
                            self.pend_n = 1
                        if self.pend_n >= 3:
                            self.pend_long = 0.0
                            self.pend_n = 0
                            old_l = cand[5]      # 确认切换 -> 放行尺寸闸门
                        else:
                            _bad = "换"
                    else:
                        self.pend_long = 0.0
                        self.pend_n = 0
                    _lo_l = 0.65 if _mv > 10.0 else 0.85
                    _hi_l = 1.45 if _mv > 10.0 else 1.18
                    if (_bad is None) and \
                            ((cand[5] < _lo_l * old_l) or
                             (cand[5] > _hi_l * old_l)):
                        _bad = "尺"
            if _bad is not None:
                self.rej_n += 1
                _ACC_DBG[0] = "%s%d" % (_bad, self.rej_n)
                if self.rej_n < ACCEPT_RELOCK_N:
                    return False
                # 连续拒收后清掉旧参考，重新锁定。
                self.rej_n = 0
                self.have = False
                self.lost = LOST_FULL
            else:
                self.rej_n = 0
                _ACC_DBG[0] = ""
        if (not self.have) or (self.lost >= LOST_FULL):
            # 首次锁定或丢失后重新锁定，直接吸附到当前测量。
            self.u, self.v, self.w, self.h = cu, cv, w, h
            self.have = True
            self.corners = c
        else:
            # 远处目标较小，使用更强的平滑。
            k = SMOOTH_FAR if (0.0 < cand[5] < FAR_LONG_PX) else SMOOTH
            if abs(cu - self.u) > DEADBAND_PX:
                self.u += k * (cu - self.u)
            if abs(cv - self.v) > DEADBAND_PX:
                self.v += k * (cv - self.v)
            if abs(w - self.w) > 2 * DEADBAND_PX:
                self.w += k * (w - self.w)
            if abs(h - self.h) > 2 * DEADBAND_PX:
                self.h += k * (h - self.h)
            # 四角同样做时间平滑。
            if c is not None:
                if self.corners is None:
                    self.corners = c
                else:
                    sc = []
                    for i in range(4):
                        ax = self.corners[i][0]
                        ay = self.corners[i][1]
                        sc.append((ax + k * (c[i][0] - ax),
                                   ay + k * (c[i][1] - ay)))
                    self.corners = tuple(sc)
        self.meas = cand
        self.lost = 0
        self.state = "锁定"
        self.last_long = cand[5]
        return True

    def detect(self, img, mask_out=None):
        """返回 dict（center/box/...）或 None（这一帧没有可用测量）。"""
        self.frame += 1
        # 非检测帧沿用上一帧结果。
        if self.have and (self.last_res is not None) and \
                ((self.frame % CV2_EVERY) != 0):
            self.last_n = 0
            return self.last_res
        cand = None
        # 软件曝光在灰度化之前完成。
        gimg = _frame_gray(img)
        if gimg is None:
            gimg = img
        _use_th = GY_TH_FAR if (self.have and
                                0.0 < self.last_long < FAR_LONG_PX) else GY_TH
        try:
            # hint 用于同尺度候选优先，不改变检测区域。
            cand = _cv2_find(gimg, (self.last_long, self.u, self.v)
                             if self.have else None, _use_th, mask_out)
            if (cand is None) and (_use_th != GY_TH_FAR):
                cand = _cv2_find(gimg, (self.last_long, self.u, self.v)
                                 if self.have else None, GY_TH_FAR, mask_out)
                if cand is not None:
                    _GY_DBG[0] = "低"
        except Exception as e:
            self.err += 1
            if self.err_msg != str(e):
                self.err_msg = str(e)
                # 异常写入日志文件。
                log("[ERROR] 靶纸检测异常: %s" % e)
            cand = None
        if cand is not None:
            if not self._accept(cand):
                cand = None
        self.last_n = 1 if (cand is not None) else 0
        if cand is None:
            self.lost += 1
            if self.lost >= LOST_FULL:
                # 丢失后清掉旧尺度和旧中心，允许重新吸附。
                self.have = False
                self.last_long = 0.0
                self.meas = None
            self.state = "锁定" if (self.have and self.lost < LOST_FULL) \
                else "搜索"
            # 同时记录检测拒收和输出护栏拒收原因。
            self.last_dbg = "[丢 %d 帧 %s%s]" % (
                self.lost, _GY_DBG[0],
                (" 护栏拒:%s" % _ACC_DBG[0]) if _ACC_DBG[0] else "")
            return None
        _du = self.u - self.meas[11]
        _dv = self.v - self.meas[12]
        _dd = _du * _du + _dv * _dv
        self.mot_px = _dd ** 0.5 if _dd > 0.0 else 0.0
        self.last_dbg = "[靶 ✓ %s 长边%.0f 形%.2f 对比%.0f]" % (
            _GY_DBG[0], cand[5], cand[7], cand[8] - cand[9])
        self.last_res = {"center": (self.u, self.v),
                         "box": (cand[1], cand[2], cand[3], cand[4]),
                         "area": cand[0], "aspect": cand[6],
                         "density": cand[7], "long_side": cand[5],
                         "contrast": cand[8] - cand[9], "n": self.last_n,
                         "state": self.state, "corners": self.corners}
        return self.last_res

class SpotDetector(object):
    def __init__(self):
        self.u = IMG_W / 2.0
        self.v = IMG_H / 2.0
        self.learned = False
        self.err = 0

    def roi(self):
        x0 = int(max(0, self.u - SPOT_ROI_HALF))
        y0 = int(max(0, self.v - SPOT_ROI_HALF))
        x1 = int(min(IMG_W, self.u + SPOT_ROI_HALF))
        y1 = int(min(IMG_H, self.v + SPOT_ROI_HALF))
        if x1 - x0 < 20 or y1 - y0 < 20:
            x0, y0 = max(0, IMG_W // 2 - SPOT_ROI_HALF), \
                max(0, IMG_H // 2 - SPOT_ROI_HALF)
            x1, y1 = min(IMG_W, x0 + 2 * SPOT_ROI_HALF), \
                min(IMG_H, y0 + 2 * SPOT_ROI_HALF)
        return (x0, y0, x1 - x0, y1 - y0)

    def detect(self, img):
        try:
            return self._detect_blobs(img)
        except Exception as e:
            self.err += 1
            print("光斑检测异常: %s" % e)
            return None

    def _adapt(self, u, v):
        if not self.learned:
            self.u, self.v = u, v
            self.learned = True
            return
        du = max(-SPOT_ADAPT_LIMIT,
                 min(SPOT_ADAPT_LIMIT, (u - self.u) * SPOT_ADAPT_GAIN))
        dv = max(-SPOT_ADAPT_LIMIT,
                 min(SPOT_ADAPT_LIMIT, (v - self.v) * SPOT_ADAPT_GAIN))
        self.u += du
        self.v += dv

    # 激光光斑色块检测。
    def _detect_blobs(self, img):
        x0, y0, w, h = self.roi()
        best_core = None
        best_halo = None
        for ti in range(len(SPOT_THRESHOLDS)):
            for b in img.find_blobs([SPOT_THRESHOLDS[ti]], roi=(x0, y0, w, h),
                                    merge=True,
                                    pixels_threshold=SPOT_MIN_AREA,
                                    area_threshold=SPOT_MIN_AREA):
                area = b[4]
                if area < SPOT_MIN_AREA or area > SPOT_MAX_AREA:
                    continue
                bw = b[2]
                bh = b[3]
                if bw < 1 or bh < 1:
                    continue
                if max(bw, bh) * 1.0 / min(bw, bh) > SPOT_MAX_ASPECT:
                    continue
                # 把 ROI 坐标转换回全图坐标。
                bx, by = b[0] + x0, b[1] + y0
                cx = bx + bw / 2.0
                cy = by + bh / 2.0
                if ti == 0:
                    if best_core is None or area > best_core[0]:
                        best_core = (area, cx, cy, (bx, by, bw, bh))
                else:
                    if best_halo is None or area > best_halo[0]:
                        best_halo = (area, cx, cy, (bx, by, bw, bh))
        best = best_core if best_core is not None else best_halo
        if best is None:
            return None
        area, cx, cy, rect = best
        self._adapt(cx, cy)
        return {"uv": (cx, cy), "area": area, "rect": rect}




def apply_flip(sensor):
    """按固件支持的形式设置上下/左右翻转。"""
    for kw in ({}, {"chn": 0}, {"chn": 1}):
        try:
            sensor.set_vflip(CAM_VFLIP, **kw)
        except Exception:
            pass
        try:
            sensor.set_hmirror(CAM_HMIRROR, **kw)
        except Exception:
            pass


# ============================ 画面亮度诊断 ============================
def log(msg):
    print(msg)


def einfo(e):
    """返回带异常类型的错误文本。"""
    return "%s: %s" % (type(e).__name__, e)

def apply_exposure(sensor):
    """先让 AE 稳定，再冻结硬件曝光并标定软件曝光增益。"""
    if not EXPO_FIX:
        return "关（EXPO_FIX=False）" + _apply_sw_scale()

    def _snap_mean():
        try:
            _im = sensor.snapshot(chn=0)
            _v = _grid_mean(_im)
            del _im
            return _v
        except Exception:
            return -1.0

    # 1) 等待 AE 适应。
    time.sleep_ms(EXPO_SETTLE_MS)
    _m1 = _snap_mean()
    # 2) 清除 ae_enable 并立即下发。
    _how = ""
    _why = ""
    try:
        sensor._dev_attr.pipe_ctrl.bits.ae_enable = 0
        _how = "ae_enable=0"
    except Exception as e:
        _how = ""
        _why = e
    if _how:
        try:
            import mpp
            mpp.kd_mpi_vicap_set_dev_attr(sensor._dev_id, sensor._dev_attr)
            _how = _how + " + set_dev_attr()"
            _froze = True
        except Exception as e:
            _how = _how + "（下发失败:%s）" % e
            _froze = False
    else:
        _how = "拿不到 dev attr(%s)" % _why
        _froze = False
    # 3) 明显变暗时恢复 AE。
    time.sleep_ms(400)
    _m2 = _snap_mean()
    _tail = ""
    # 只有冻结成功时才继续调模拟增益。
    _ae_off = _froze
    if (_m1 > 0) and (_m2 > 0) and (_m2 < _m1 * EXPO_DARK_RATIO):
        try:
            sensor._dev_attr.pipe_ctrl.bits.ae_enable = 1
            import mpp
            mpp.kd_mpi_vicap_set_dev_attr(sensor._dev_id, sensor._dev_attr)
            _tail = "；冻完变暗(%.0f->%.0f) 已把 AE 开回来" % (_m1, _m2)
            _ae_off = False
        except Exception as e:
            _tail = "；冻完变暗(%.0f->%.0f) 且回滚失败(%s)" % (_m1, _m2, e)
    # 4) 按启动亮度计算一次软件曝光增益，并验证输出亮度。
    global EXPO_SW_GAIN
    _m = _m2
    _sw_ok = False
    if (EXPO_TARGET_MEAN > 0) and (_m > 0):
        _g_new = EXPO_SW_GAIN
        if EXPO_AUTO_GAIN:
            _g_new = EXPO_TARGET_MEAN / _m
            if _g_new < EXPO_SW_MIN:
                _g_new = EXPO_SW_MIN
            elif _g_new > EXPO_SW_MAX:
                _g_new = EXPO_SW_MAX
        try:
            import cv_lite
            import image as _image
            _im = sensor.snapshot(chn=0)
            _rgb = _im.to_numpy_ref()
            _out = cv_lite.rgb888_adjust_exposure_fast(
                [IMG_H, IMG_W], _rgb, float(_g_new))
            if _out is not None:
                _eim = _image.Image(IMG_W, IMG_H, _image.RGB888,
                                    alloc=_image.ALLOC_REF, data=_out)
                _m3 = _grid_mean(_eim)
                try:
                    del _eim
                    del _out
                except Exception:
                    pass
                _sw_ok = True
                _tail += ("；软件曝光 gain=%.2f 亮度 %.0f->%.0f%s"
                          % (_g_new, _m, _m3,
                             " ✓" if abs(_m3 - _m) >= 2.0 else "（变化<%d）"
                             % 2))
                _m = _m3
                EXPO_SW_GAIN = float(_g_new)
            try:
                del _im
            except Exception:
                pass
        except Exception as e:
            _tail += "；软件曝光失败(%s: %s)" % (type(e).__name__, e)
        if not _sw_ok:
            EXPO_SW_GAIN = 1.0
            _tail += "；软件曝光不可用，退回 gain=1.0（原始帧）"

    # 5) 可选模拟增益；0 表示保持冻结时的值。
    if _ae_off and (EXPO_HW_GAIN > 0):
        try:
            from media.sensor import k_sensor_gain
            _g1 = sensor.again()
            _g0 = float(_g1.gain[0]) if _g1 is not None else -1.0
            _gh = k_sensor_gain()
            _gh.gain[0] = EXPO_HW_GAIN
            sensor.again(_gh)
            time.sleep_ms(300)
            _g2 = sensor.again()
            _g_rd = float(_g2.gain[0]) if _g2 is not None else -1.0
            _m4 = _snap_mean()
            _tail += "；模拟增益 %.2f->%d(读回%.2f) 亮度 %.0f" % (
                _g0, EXPO_HW_GAIN, _g_rd, _m4)
            if _m4 > 0:
                _m = _m4
        except Exception as e:
            _tail += "；模拟增益写入失败(%s: %s)" % (type(e).__name__, e)
    if (_m > 0) and (EXPO_TARGET_MEAN > 0):
        _tail += "；最终 均=%.0f（目标%.0f，软件曝光 gain=%.2f）" % (
            _m, EXPO_TARGET_MEAN, EXPO_SW_GAIN)
    _tail += _apply_sw_scale()
    return "冻曝光: %s；亮度 %.0f -> %.0f%s" % (_how, _m1, _m2, _tail)

def _grid_mean(img):
    """用 10×10 网格估算画面平均亮度。"""
    _arr = None
    try:
        _arr = img.to_numpy_ref()
    except Exception:
        _arr = None
    s = 0.0
    k = 0
    for gy in range(10):
        y = int(IMG_H * (gy + 0.5) / 10)
        for gx in range(10):
            x = int(IMG_W * (gx + 0.5) / 10)
            v = None
            if _arr is not None:
                try:
                    p = _arr[y, x]
                    try:
                        n = len(p)
                    except TypeError:
                        v = p
                    else:
                        if n >= 3:
                            # 先转 int，避免 uint8 相加溢出。
                            v = (int(p[0]) + int(p[1]) + int(p[2])) / 3.0
                        elif n == 1:
                            v = int(p[0])
                except Exception:
                    v = None
            if v is None:
                try:
                    p = img.get_pixel(x, y)
                except Exception:
                    continue
                if p is None:
                    continue
                try:
                    v = ((p[0] + p[1] + p[2]) / 3.0) \
                        if isinstance(p, (tuple, list)) else p
                except Exception:
                    continue
            s += v
            k += 1
    return (s / k) if k else -1.0

def _apply_sw_scale():
    """软件曝光改变亮度时，同步缩放绝对亮度判据。

    GY_TH 是软件曝光归一化后的固定阈值，不再乘 gain。
    """
    global PAPER_INNER_MAX, PAPER_CONTRAST_MIN
    global PAPER_DARK_MARGIN, GY_CANNY1, GY_CANNY2
    if _SW_SCALED[0]:
        return ""
    _SW_SCALED[0] = True
    _s = float(EXPO_SW_GAIN)
    if abs(_s - 1.0) < 1e-3:
        return "（软件曝光 gain=1.0，阈值不缩放）"
    _o = (PAPER_INNER_MAX, PAPER_CONTRAST_MIN,
          PAPER_DARK_MARGIN, GY_CANNY1, GY_CANNY2)
    PAPER_INNER_MAX = max(2, int(round(PAPER_INNER_MAX * _s)))
    PAPER_CONTRAST_MIN = max(1, int(round(PAPER_CONTRAST_MIN * _s)))
    PAPER_DARK_MARGIN = max(1, int(round(PAPER_DARK_MARGIN * _s)))
    GY_CANNY1 = max(1, int(round(GY_CANNY1 * _s)))
    GY_CANNY2 = max(2, int(round(GY_CANNY2 * _s)))
    return ("；曝光判据随 gain ×%.2f：饱和档 %d->%d "
            "对比 %d->%d 暗边 %d->%d Canny %d/%d->%d/%d；"
            "GY_TH 固定=%d（不乘 gain）"
            % (_s, _o[0], PAPER_INNER_MAX,
               _o[1], PAPER_CONTRAST_MIN, _o[2], PAPER_DARK_MARGIN,
               _o[3], _o[4], GY_CANNY1, GY_CANNY2, GY_TH))


def main():
    from media.sensor import Sensor, CAM_CHN_ID_0
    from media.display import Display
    from media.media import MediaManager
    import gc
    global GY_TH                   # GY_TH 轮换（GY_TH_SWEEP）要在运行时改它

    sensor = Sensor(id=2, width=1280, height=960, fps=90)
    sensor.reset()
    time.sleep_ms(100)
    apply_flip(sensor)
    # ISP 输出请求尺寸。
    sensor.set_framesize(width=IMG_W, height=IMG_H, chn=CAM_CHN_ID_0)
    # 软件曝光要求相机输出 RGB888，检测前再转灰度。
    sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)
    apply_flip(sensor)
    if EXPO_FIX:
        # run() 前保持 AE，运行后再冻结。
        log("曝光: 先让自动曝光(AE)自适应 %dms，run 后再冻住（见 apply_exposure）"
            % EXPO_SETTLE_MS)
    # 板载显示固定 640x480，处理图缩放后绘制。
    try:
        Display.init(Display.ST7701, width=DISPLAY_W, height=DISPLAY_H,
                     to_ide=True, quality=DISPLAY_QUALITY)
        print("显示: ST7701 %dx%d -> IDE 帧缓冲区（图像 %dx%d 缩着显示）"
              % (DISPLAY_W, DISPLAY_H, IMG_W, IMG_H))
    except Exception as e:
        print("ST7701 %dx%d 打不开(%s) -> 试 VIRT" % (DISPLAY_W, DISPLAY_H, e))
        Display.init(Display.VIRT, width=DISPLAY_W, height=DISPLAY_H, fps=30)
        print("显示: VIRT %dx%d -> IDE 帧缓冲区" % (DISPLAY_W, DISPLAY_H))
    MediaManager.init()
    sensor.run()
    # run() 后冻结 AE 并标定软件曝光。
    print("曝光: %s" % apply_exposure(sensor))

    # ---------- 预览缩放（把处理图像摆到 640x480 画布上）----------
    # 不放大（放大只会糊）；比画布小时居中摆放。
    import image
    _canvas = image.Image(DISPLAY_W, DISPLAY_H, image.RGB565)
    _sx = DISPLAY_W / float(IMG_W)
    _sy = DISPLAY_H / float(IMG_H)
    if (_sx > 1.0) or (_sy > 1.0):
        _sx = _sy = 1.0
    _ox = int((DISPLAY_W - IMG_W * _sx) * 0.5)
    _oy = int((DISPLAY_H - IMG_H * _sy) * 0.5)
    _preview_scaled = True        # draw_image 的缩放万一不支持 -> 退回推原图
    _bin_mode = ""                # 二值预览走哪条路（下面探测；"" = 用不了）
    _cv_shape = False             # 画布上能不能画图形（下面探测）
    _cv_text = False              # 画布上能不能写字
    _ov_bad = []                  # 哪些绘制调用这版固件不支持（每个只报一次）

    def _ov(name, fn, *a, **kw):
        """调一次绘制。某个 API 在这版固件上不支持时，只把它自己跳过 +
        打一行日志，避免单个 API 失败影响整层叠加。"""
        if name in _ov_bad:
            return
        try:
            fn(*a, **kw)
        except Exception as e:
            _ov_bad.append(name)
            print("叠加层 '%s' 不支持(%s) -> 以后跳过它" % (name, e))

    def _overlay(target, det, spot_uv, s, ox, oy, ink, txt):
        """绘制候选框、四边形、靶心、激光和状态文字。"""
        if spot_uv is not None and (type(spot_uv) is dict):
            spot_uv = spot_uv["uv"]
        if ink is not None:
            _v = int(ink)
            c_ok = c_cross = c_spot = _v
        else:
            c_ok = (0, 255, 0)
            c_cross = (255, 0, 0)
            c_spot = (255, 255, 0)
        if det.fresh():
            c4 = det.corners
            if c4 is not None:
                for i in range(4):
                    a = c4[i]
                    b = c4[(i + 1) % 4]
                    _ov("line", target.draw_line,
                        int(a[0] * s + ox), int(a[1] * s + oy),
                        int(b[0] * s + ox), int(b[1] * s + oy),
                        color=c_ok, thickness=2)
            elif det.meas is not None:
                bx, by, bw, bh = det.meas[1:5]
                _ov("rect", target.draw_rectangle,
                    int(bx * s + ox), int(by * s + oy),
                    int(bw * s), int(bh * s),
                    color=c_ok, thickness=2)
            _ov("cross", target.draw_cross,
                int(det.u * s + ox), int(det.v * s + oy),
                color=c_cross, size=10, thickness=2)
        if spot_uv is not None:
            _ov("circle", target.draw_circle,
                int(spot_uv[0] * s + ox), int(spot_uv[1] * s + oy), 6,
                color=c_spot, thickness=2)
        if txt and (ink is None):
            _ov("text", target.draw_string_advanced,
                4, 4, 16, txt, color=(255, 255, 0))

    def _gray_comp(im):
        """中心像素的分量数：灰度=1~2 个，彩色=3 个。查不了返回 -1。"""
        try:
            p = im.get_pixel(im.width() // 2, im.height() // 2)
            if p is None:
                return -1
            return len(p) if isinstance(p, tuple) else 1
        except Exception:
            return -1

    def _to_gray(img):
        """转换为灰度图，并返回 (图像, 使用的方式)。"""
        g = None
        try:
            g = img.to_grayscale()
        except Exception as e:
            print("to_grayscale 异常(%s)" % einfo(e))
            g = None
        if g is not None:
            n = _gray_comp(g)
            if n != 3:                 # 1~2 = 灰度；-1 = 查不了也先信它
                return g, "rv"
        elif _gray_comp(img) in (1, 2):     # 返回 None 且原地确实转灰了
            return img, "inplace"
        try:
            gb = image.Image(img.width(), img.height(), image.GRAYSCALE)
            img.copy_to(gb)
            if _gray_comp(gb) != 3:
                return gb, "copy"
        except Exception as e:
            print("灰度备用路(copy_to)也不行(%s)" % einfo(e))
        return None, ""

    # ---------- 相机会话与显示初始化 ----------
    # 纯取图测速用于区分采集上限和处理耗时。
    _t0 = time.ticks_ms()
    _n = 0
    _probe_img = None
    while time.ticks_diff(time.ticks_ms(), _t0) < 1500:
        _probe_img = sensor.snapshot(chn=CAM_CHN_ID_0)
        _n += 1
    _ms = max(1, time.ticks_diff(time.ticks_ms(), _t0))
    _w = _h = 0
    try:
        _w, _h = _probe_img.width(), _probe_img.height()
    except Exception:
        pass
    del _probe_img
    print("[PROBE] 纯取图 %d 帧 / %dms = %.1f fps（相机+ISP 的实际上限；"
          "实际输出 %dx%d，请求 %dx%d）"
          % (_n, _ms, _n * 1000.0 / _ms, _w, _h, IMG_W, IMG_H))

    # ---------- 预览路径探测 ----------
    _pv_what = "灰度预览" if GRAY_PREVIEW else "二值化预览"
    if BIN_PREVIEW or GRAY_PREVIEW:
        try:
            _pi = sensor.snapshot(chn=CAM_CHN_ID_0)
            if GRAY_PREVIEW:
                # 灰度预览不执行二值化。
                _gt, _gm = _to_gray(_pi)
                print("灰度探测: 转换路径=%s (rv=用返回值/inplace=原地/copy=预分配)"
                      % (_gm or "失败"))
                if _gt is not None:
                    _pi = _gt
            # 二值预览直接使用检测链的融合图。
            try:
                # 优先直接缩放，避免每帧分配。
                _canvas.draw_image(_pi, _ox, _oy,
                                   x_scale=_sx, y_scale=_sy)
                _bin_mode = "bin"
            except Exception as e:
                print("预览B(直画缩放)不行: %s" % e)
            if not _bin_mode:
                try:
                    # 其次使用预分配 RGB565 缓冲。
                    _bin_rgb = image.Image(IMG_W, IMG_H, image.RGB565)
                    _pi.copy_to(_bin_rgb)
                    _canvas.draw_image(_bin_rgb, _ox, _oy,
                                       x_scale=_sx, y_scale=_sy)
                    _bin_mode = "copy"
                except Exception as e:
                    print("预览C(预分配+copy_to)不行: %s" % e)
                    try:
                        del _bin_rgb
                    except Exception:
                        pass
            if not _bin_mode:
                try:
                    # 最后尝试每帧转 RGB565。
                    _prgb = _pi.to_rgb565()
                    _canvas.draw_image(_prgb, _ox, _oy,
                                       x_scale=_sx, y_scale=_sy)
                    _bin_mode = "rgb"
                    del _prgb
                except Exception as e:
                    print("预览A(转RGB565)不行: %s" % e)
            if _bin_mode:
                Display.show_image(_canvas)
            del _pi
        except Exception as e:
            print("预览变换(灰度化/binary)本身不行: %s" % e)
        # 逐项探测画布是否支持图形和文字。
        try:
            _canvas.draw_rectangle(0, 0, 8, 8, color=(255, 0, 0), thickness=1)
            _cv_shape = True
        except Exception as e:
            print("画布画矩形不行(%s)，叠加层改画在二值图上" % einfo(e))
        for _nm, _fn, _a in (("直线", _canvas.draw_line, (0, 0, 8, 8)),
                             ("十字", _canvas.draw_cross, (4, 4)),
                             ("圆", _canvas.draw_circle, (4, 4, 3))):
            try:
                _fn(*(_a + ((255, 0, 0),)))
            except Exception as e:
                print("画布画%s不行(%s)" % (_nm, e))
        try:
            _canvas.draw_string_advanced(2, 2, 16, "probe",
                                         color=(255, 255, 0))
            _cv_text = True
        except Exception as e:
            print("画布写字不行(%s)，就不显示 th= 那行字了" % einfo(e))
        if _bin_mode == "bin":
            print("%s: 做法B（图直接缩放，零分配）" % _pv_what)
        elif _bin_mode == "copy":
            print("%s: 做法C（预分配 RGB565 + copy_to）" % _pv_what)
        elif _bin_mode == "rgb":
            print("%s: 做法A（每帧转 RGB565）" % _pv_what)
        else:
            print("%s: 用不了 -> 本次跑彩色预览" % _pv_what)
        print("预览叠加: 画布图形=%s 画布文字=%s"
              % ("可以" if _cv_shape else "不行(改画二值图上)",
                 "可以" if _cv_text else "不行"))

    target_det = PaperDetector()
    spot_det = SpotDetector()

    last_spot = None
    cal_u = []            # 标定用：光斑 u 采样（中位数抗误检）
    cal_v = []
    t_print = time.ticks_ms()
    t_mask = time.ticks_ms()   # [MASK] 读数节拍（见 _dark_frac）
    _gy_sw = -1           # 当前阈值扫描档位
    _sw_n = 0             # 本档已打印的 FPS 行数（统计用）
    _sw_lock = 0          # 本档"靶=锁定"的行数
    n_frame = 0
    n_frame_prev = 0
    n_tgt = 0
    n_spot = 0
    sum_grab = 0
    sum_tgt = 0
    sum_spot = 0
    sum_show = 0
    sum_n = 0

    # 阈值扫描会修改模块级 GY_TH。
    global GY_TH

    print("=" * 68)
    print("vision_check: 纯视觉（不发串口）; 视觉处理 = k230_aim/main.py 原文")
    print("  分辨率 %dx%d (S=%.2f, K=%.2fpx/°)  靶心=四边形对角线交点"
          % (IMG_W, IMG_H, S_SCALE, VIS_K))
    def _ksize(n):
        return "关" if n <= 0 else ("%dx%d" % (2 * n + 1, 2 * n + 1))

    def _dark_frac(im, th):
        """估算画面中比阈值暗的像素比例。"""
        n = 0
        dk = 0
        for _gy in range(16):
            y = int(IMG_H * (_gy + 0.5) / 16)
            for _gx in range(20):
                x = int(IMG_W * (_gx + 0.5) / 20)
                v = px_luma(im, x, y)
                if v >= 0:
                    n += 1
                    if v < th:
                        dk += 1
        if n <= 0:
            return -1.0
        return dk * 100.0 / n

    print("  检测=融合图每%d帧一次 -> 白色连通域 -> 凸包/RDP 四边"
          " + 内孔；外框=外沿/内框重心=靶心" % CV2_EVERY)
    if GY_CLOSE_SIZE <= 0:
        _cl = "闭运算关"
    elif GY_CLOSE_SIZE <= 2:
        _cl = "闭运算%d×%d(全分辨率直接做)" % (2 * GY_CLOSE_SIZE + 1,
                                              2 * GY_CLOSE_SIZE + 1)
    else:
        _cl = "闭运算%d×%d(金字塔等效)" % (2 * (GY_CLOSE_SIZE // 2) + 1,
                                          2 * (GY_CLOSE_SIZE // 2) + 1)
    print("  预览=反向二值化(暗=白 th=%d) + %s ∪ "
          "Canny(%d,%d) -> 外框绿、靶心红"
          % (GY_TH, _cl, GY_CANNY1, GY_CANNY2))
    print("  曝光=%s  光斑每%d帧搜一次"
          % ("冻结 AE + 软件曝光标定(目标亮度%.0f，gain 启动时算一次)"
             % EXPO_TARGET_MEAN
             if EXPO_FIX else "自动", SPOT_EVERY))
    print("  过滤=%s"
          % ("对比>=%d/暗边>=%.0f%%/[仅饱和灯检查内部结构] -> "
             "长边%d~%dpx 比例[%.2f,%.2f] 四边几何(cos<=%.2f/最短边%d)"
             % (PAPER_CONTRAST_MIN, PAPER_DARK_FRAC_MIN * 100,
                PAPER_MIN_LONG, PAPER_MAX_LONG,
                PAPER_ASPECT_MIN, PAPER_ASPECT_MAX,
                QUAD_ANGLE_COS_MAX, PAPER_MIN_SIDE)))
    print("  丢检保持：绿框最多再留 3 帧（%s）" % "显示用，不影响上报")
    print("=" * 68)

    try:
        while True:
            os.exitpoint()
            now = time.ticks_ms()

            # ---------- 可选阈值扫描 ----------
            if GY_TH_SWEEP and GY_TH_SWEEP_LIST:
                _sw2 = ((time.ticks_ms() // (GY_TH_SWEEP_SEC * 1000)) %
                        len(GY_TH_SWEEP_LIST))
                if _sw2 != _gy_sw:
                    _gy_sw = _sw2
                    GY_TH = GY_TH_SWEEP_LIST[_sw2]
                    print(">>> 阈值扫描: GY_TH=%d（看预览：只有胶带是白的"
                          "那一档最好）" % GY_TH)

            # ---------- 取图 ----------
            t0 = time.ticks_ms()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            t_grab = time.ticks_diff(time.ticks_ms(), t0)
            n_frame += 1

            # ---------- 靶纸（与 main.py 同一套 PaperDetector）----------
            t0 = time.ticks_ms()
            res = target_det.detect(img, _GY_MASK_CACHE)
            t_tgt = time.ticks_diff(time.ticks_ms(), t0)
            if res is not None:
                n_tgt += 1

            # ---------- 光斑（每 SPOT_EVERY 帧搜一次，其它帧复用）----------
            t0 = time.ticks_ms()
            if CALIB_LASER:
                # 标定模式逐帧检测光斑。
                last_spot = spot_det.detect(img)
                spot = last_spot
                if last_spot is not None and len(cal_u) < 400:
                    cal_u.append(last_spot["uv"][0])
                    cal_v.append(last_spot["uv"][1])
            else:
                if (not USE_FIXED_LASER) and ((n_frame % SPOT_EVERY) == 0):
                    last_spot = spot_det.detect(img)
                spot = last_spot
            t_spot = time.ticks_diff(time.ticks_ms(), t0)
            if USE_FIXED_LASER and (not CALIB_LASER):
                # 使用固定激光坐标。
                spot = {"uv": LASER_UV}
            if spot is not None:
                n_spot += 1
                SPOT_UV[0], SPOT_UV[1] = spot["uv"]

            # 丢靶时打一行画面亮度：判断"没看到靶"还是"判据没过"
            if target_det.lost == 8 or (target_det.lost > 30 and
                                        target_det.lost % 90 == 0):
                log_scene(img)

            # ---------- 反向二值图暗像素比例 ----------
            if time.ticks_diff(now, t_mask) >= 2000:
                t_mask = now
                _th_now = _GY_TH_NOW[0] if _GY_TH_NOW[0] > 0 else GY_TH
                _df = _dark_frac(img, _th_now)
                if _df >= 0:
                    print("[MASK] th=%d 暗于此的像素=%.1f%% (只有胶带约 3~10%%；"
                          ">15%% 说明木柜也切进来了)" % (_th_now, _df))

            # ---------- 绘制叠加层 ----------
            if (not _bin_mode) and target_det.have and \
                    (target_det.lost <= DRAW_HOLD):
                c = target_det.corners
                if c is not None:
                    for i in range(4):
                        a = c[i]
                        b = c[(i + 1) % 4]
                        img.draw_line(int(a[0]), int(a[1]),
                                      int(b[0]), int(b[1]),
                                      color=(0, 255, 0), thickness=2)
                elif target_det.meas is not None:
                    bx, by, bw, bh = target_det.meas[1:5]
                    img.draw_rectangle(bx, by, bw, bh, color=(0, 255, 0),
                                       thickness=2)
                img.draw_cross(int(target_det.u), int(target_det.v),
                               color=(255, 0, 0), size=14, thickness=2)
            if (not _bin_mode) and (spot is not None):
                img.draw_circle(int(spot["uv"][0]), int(spot["uv"][1]), 8,
                                color=(255, 255, 0), thickness=2)
                img.draw_cross(int(spot["uv"][0]), int(spot["uv"][1]),
                               color=(255, 255, 0), size=6, thickness=1)

            # 按 PREVIEW_EVERY 推送预览。
            t0 = time.ticks_ms()
            if (n_frame % PREVIEW_EVERY) == 0:
                _ok = False
                _pending = False
                if _bin_mode:
                    # 灰度预览或融合图预览；各步骤独立捕获异常。
                    if GRAY_PREVIEW:
                        try:
                            # 与检测链使用同一灰度化流程。
                            _g2 = _frame_gray(img)
                            if _g2 is None:
                                _bin_mode = ""
                                print("灰度化不可用 -> 改彩色预览")
                            else:
                                img = _g2
                        except Exception as e:
                            _bin_mode = ""
                            print("灰度化失败(%s) -> 改彩色预览" % einfo(e))
                    else:
                        # 显示检测链生成的融合图。
                        try:
                            _mg = None
                            _mg = _GY_MASK_CACHE[0]
                            if _mg is None:
                                # 融合图暂未生成时跳过本帧绘制。
                                _pending = True
                            else:
                                img = _mg
                        except Exception as e:
                            _bin_mode = ""
                            print("检测图预览失败(%s) -> 改彩色预览" % einfo(e))
                if _pending:
                    pass                    # 等待下一帧融合图。
                elif _bin_mode:
                    try:
                        if _bin_mode == "bin":
                            _canvas.draw_image(img, _ox, _oy,
                                               x_scale=_sx, y_scale=_sy)
                        elif _bin_mode == "copy":
                            img.copy_to(_bin_rgb)
                            _canvas.draw_image(_bin_rgb, _ox, _oy,
                                               x_scale=_sx, y_scale=_sy)
                        else:                       # "rgb"
                            _rgb = img.to_rgb565()
                            _canvas.draw_image(_rgb, _ox, _oy,
                                               x_scale=_sx, y_scale=_sy)
                            del _rgb
                        _txt = ("th=%d(固定) %s 长边%.0f"
                                % ((_GY_TH_NOW[0] if _GY_TH_NOW[0] > 0
                                    else GY_TH), target_det.state,
                                   target_det.last_long))
                        _sp = None
                        if spot is not None:
                            _sp = spot["uv"]
                        if not _cv_shape:
                            # 画布不支持图形时，在融合图副本上绘制。
                            _ink = 255 if _GY_MASK_OK[0] else 0
                            try:
                                img = img.copy()
                            except Exception:
                                pass
                            _overlay(img, target_det, _sp,
                                     1.0, 0, 0, _ink, None)
                        if _cv_shape:
                            _overlay(_canvas, target_det, _sp,
                                     _sx, _ox, _oy, None,
                                     _txt if _cv_text else None)
                        _ok = True
                    except Exception as e:
                        _bin_mode = ""
                        print("二值图缩放/叠加失败(%s) -> 改彩色预览" % einfo(e))
                if _ok:
                    try:
                        Display.show_image(_canvas)
                    except Exception as e:
                        print("推画面失败(%s)" % einfo(e))
                else:
                    # 彩色预览：优先缩放，失败时直接推送原图。
                    if _preview_scaled:
                        try:
                            _canvas.draw_image(img, _ox, _oy,
                                               x_scale=_sx, y_scale=_sy)
                            Display.show_image(_canvas)
                        except Exception as e:
                            _preview_scaled = False
                            print("缩放预览不支持(%s) -> 直接推原图"
                                  "（画面会被裁掉一部分）" % e)
                            Display.show_image(img)
                    else:
                        Display.show_image(img)
            t_show = time.ticks_diff(time.ticks_ms(), t0)
            del img

            sum_grab += t_grab
            sum_tgt += t_tgt
            sum_spot += t_spot
            sum_show += t_show
            sum_n += 1

            # ---------- 打印 ----------
            if time.ticks_diff(now, t_print) >= PRINT_MS:
                fps = ((n_frame - n_frame_prev) * 1000.0 /
                       max(1, time.ticks_diff(now, t_print)))
                n_frame_prev = n_frame
                t_print = now
                line = "FPS=%.1f " % fps
                if res is not None:
                    m = target_det.meas
                    line += ("靶=锁定 距离≈%.2fm 靶心=(%.0f,%.0f) "
                             "长边=%.0fpx 比%.2f 形%.2f 对比=%.0f"
                             % (FX_PX * PAPER_LONG_M / max(1.0, m[5]),
                                target_det.u, target_det.v,
                                m[5], m[6], m[7], m[8] - m[9]))
                else:
                    line += "靶=%s 候选%d th=%d %s" % (
                        target_det.state, target_det.last_n,
                        (_GY_TH_NOW[0] if _GY_TH_NOW[0] > 0 else GY_TH),
                        target_det.last_dbg)
                if spot is not None:
                    if res is not None:
                        eu = target_det.u - spot["uv"][0]
                        ev = target_det.v - spot["uv"][1]
                        err = math.sqrt(eu * eu + ev * ev)
                        line += (" | 光斑=(%.0f,%.0f) 误差=(%.0f,%.0f)px"
                                 "=%.1fpx(%.2f°)"
                                 % (spot["uv"][0], spot["uv"][1], eu, ev,
                                    err, err / FX_PX * 57.29578))
                    else:
                        line += " | 光斑=(%.0f,%.0f)" % spot["uv"]
                else:
                    line += " | 光斑: 未检出"
                if CALIB_LASER:
                    # 用中位数抑制单帧误检。
                    if cal_u:
                        su = sorted(cal_u)
                        sv = sorted(cal_v)
                        k = len(su) // 2
                        line += (" | [标定] 光斑 n=%d 中位=(%.1f,%.1f) "
                                 "=> LASER_UV=(%d,%d)（现在填的是 %d,%d）"
                                 % (len(su), su[k], sv[k],
                                    int(su[k] + 0.5), int(sv[k] + 0.5),
                                    LASER_UV[0], LASER_UV[1]))
                    else:
                        line += " | [标定] 没找到光斑（激光开了吗？）"
                if sum_n > 0:
                    line += " | ms 取图%.0f 目标%.0f 光斑%.0f 显示%.0f" % (
                        sum_grab / float(sum_n), sum_tgt / float(sum_n),
                        sum_spot / float(sum_n), sum_show / float(sum_n))
                line += " | 靶%d/%d 斑%d" % (n_tgt, n_frame, n_spot)
                print(line)
                # 分段耗时。
                print("[PERF] 目标%.0fms/帧 | %s"
                      % (sum_tgt / float(max(1, sum_n)), prof_line()))
                prof_reset()
                sum_grab = 0
                sum_tgt = 0
                sum_spot = 0
                sum_show = 0
                sum_n = 0

            if (n_frame % GC_EVERY) == 0:
                gc.collect()
    except KeyboardInterrupt:
        print("已停止")
    except Exception as e:
        # 输出完整 traceback，便于在 IDE 中定位。
        print("[CRASH] %s: %s" % (type(e).__name__, e))
        try:
            import sys as _sys
            _sys.print_exception(e)
        except Exception:
            pass
    finally:
        try:
            sensor.stop()
        except Exception:
            pass
        try:
            Display.deinit()
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
            time.sleep_ms(100)
            MediaManager.deinit()
        except Exception:
            pass
        print("已退出")


main()
