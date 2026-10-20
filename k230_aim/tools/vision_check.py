# -*- coding: utf-8 -*-
"""vision_check.py —— K230 视觉调试工具

流程:
  RGB888 640x480 -> 固定130二值化 -> image.close(1) 3x3
  -> find_blobs 白块 -> 激光点附近目标筛选
  -> 二值 mask 四边扫描/最小二乘 -> 对角线交点
  -> 尺寸/比例/密度/对比度/黑框闸门

画面: 闭运算后的二值图; 橙框=白块候选; 绿框=拟合/锁定四边形; 红十字=靶心; 黄圈=激光.
纯视觉: 不与 H723 通信; 整机联调请用 main.py.
"""
import os
import time
import math
import image

# 全帧闭运算：二值化阈值 130，close(1) 对应 3x3。
USE_CLOSE_ROI = True
FULL_FRAME_CLOSE = True
GRAY_DIRECT = True
CLOSE_FRAME_COS = 0.65
CLOSE_ROI_KERNEL = 3
CLOSE_ROI_ITERS = 1
CLOSE_ROI_TH = 0
CLOSE_ROI_K = 0.52
CLOSE_FULL_PREVIEW = True
CLOSE_PREVIEW_ANY = True
try:
    import cv_lite as _cv_lite
except Exception as _e:
    _cv_lite = None
    USE_CLOSE_ROI = False
    print("cv_lite 不可用(%s) -> 关闭 ROI 闭运算" % _e)

PROC_W = 640
PROC_H = 480
IMG_W = PROC_W
IMG_H = PROC_H
CLOSE_BIN_TH = 130
CLOSE_MORPH_SIZE = 1
S_SCALE = PROC_W / 640.0
CLOSE_ROI_PAD = int(round(18 * S_SCALE))
CLOSE_ROI_MIN_LONG = int(round(100 * S_SCALE))
VIS_K = 7.8 * S_SCALE
DISPLAY_W = 640
DISPLAY_H = 480
DISPLAY_QUALITY = 40
BIN_PREVIEW = not FULL_FRAME_CLOSE
ALLOW_COLOR_FALLBACK = False
EXPO_FIX = False
EXPO_US = 8000
SHOW_EVERY = 2
CAM_VFLIP = True
CAM_HMIRROR = False

PAPER_TH = 50
PAPER_TH_FIXED = 0
PAPER_TH_NEAR_FIXED = 0
PAPER_A_MAX = 32
PAPER_B_MAX = 32
PAPER_MIN_AREA = int(350 * S_SCALE * S_SCALE)
PAPER_MAX_AREA_RATIO = 0.50
PAPER_MIN_LONG = int(round(36 * S_SCALE))
PAPER_MAX_LONG = int(round(480 * S_SCALE))
PAPER_ASPECT_MIN = 0.70
PAPER_ASPECT_MAX = 2.00
PAPER_DENSITY_MIN = 0.56
PAPER_DENSITY_MIN_STRONG = 0.45
PAPER_CONTRAST_STRONG = 70
PAPER_DENSITY_QUAD_MIN = 0.65
PAPER_DENSITY_NEAR_MIN = 0.72
PAPER_CONTRAST_MIN = 30
PAPER_DARK_MARGIN = 25
PAPER_DARK_FRAC_MIN = 0.80
QUAD_SCAN_N = 3
QUAD_PERP_PX = 1
QUAD_AREA_LO = 0.50
QUAD_AREA_HI = 1.60
QUAD_NEAR_LONG = int(round(190 * S_SCALE))
QUAD_NEAR_SIDE_K = 0.70
QUAD_NEAR_FILL_K = 0.50
QUAD_NEAR_EDGE_K = 0.30
QUAD_NEAR_DENSITY_HI = 1.30
QUAD_NEAR_COS_LIM = 0.42
SCAN_TH_K = 0.68
SCAN_TH_LO = 65
SCAN_TH_HI = 125
QUAD_LIM_PAD = int(round(18 * S_SCALE))
PROBE_OFF_PX = int(round(5 * S_SCALE))
TRACK_K = 0.50
TRACK_PAD = int(round(35 * S_SCALE))
TRACK_PAD_FAR = int(round(90 * S_SCALE))
MERGE_RING_DENSITY_MAX = 0.72
MERGE_CENTER_K = 0.15
MERGE_INNER_AREA_MIN = 0.03
SEED_EVERY = 16
RE_CHECK_EVERY = 40
HOLD_FRAMES = 25
LOST_FULL = 25
SMOOTH = 0.90
SMOOTH_NEAR = 0.90
DEADBAND_PX = 1.5 * S_SCALE
SIZE_GATE_LO = 0.50
SIZE_GATE_HI = 1.35
SIZE_RESCUE_DENSITY = 0.88
SIZE_RESCUE_CONTRAST = 30
SIZE_RESCUE_MAX = 2.50
CONFIRM_N = 1
CONFIRM_DXY = int(round(60 * S_SCALE))
CONFIRM_DSIZE = 1.00
PENDING_MISS = 8
JUMP_REJECT_PX = int(round(100 * S_SCALE))
SMOOTH_FAR = 0.60
FAR_LONG_PX = 120.0 * S_SCALE
PAPER_LONG_M = 0.275

SPOT_ROI_HALF = int(round(45 * S_SCALE))
SPOT_THRESHOLDS = [
    (88, 100, -40, 90, -50, 90),
    (58, 100, 25, 90, -20, 70),
]
SPOT_MIN_AREA = 2
SPOT_MAX_AREA = 6000
SPOT_MAX_ASPECT = 3.0
SPOT_EVERY = 4
SPOT_ADAPT_GAIN = 0.06
SPOT_ADAPT_LIMIT = 6.0
SPOT_PREF_PX = int(round(70 * S_SCALE))
SPOT_UV = [IMG_W / 2.0, IMG_H / 2.0]
USE_FIXED_LASER = True
LASER_UV = (int(round(323 * S_SCALE)), int(round(246 * S_SCALE)))
CALIB_LASER = False


FX_PX = 445.0 * S_SCALE

PRINT_MS = 500
GC_EVERY = 2 if FULL_FRAME_CLOSE else 10

def luma(r, g, b):
    return (r * 77 + g * 150 + b * 29) >> 8


_PIXBUF = [0, 0, 0]
_PIX_FAST = False
_ARR_MODE = 0


def px_probe_fast(img):
    """启动时探一次：get_pixel 支不支持"传一个 buf 就地填"。

    为什么值得做：px_luma 是四边形复测的**热路径**（每帧几百次调用），而
    `img.get_pixel(x, y)` 每次都要新建一个 (r,g,b) 元组 —— MicroPython 里
    一次分配就是几微秒，几百次就是 1~3ms/帧（占检测耗时的 10~20%）。
    OpenMV/CanMV 的 get_pixel 支持 `get_pixel(x, y, rgbtuple)` 就地写，
    我们探一下：能用就用（省掉分配 -> 帧率上去 -> 反应更快）。
    """
    global _PIX_FAST
    try:
        img.get_pixel(0, 0, _PIXBUF)
        _PIX_FAST = True
    except Exception:
        _PIX_FAST = False
    return _PIX_FAST


def px_luma(img, x, y):
    """取某点亮度（0~255）。取不到返回 -1。"""
    if x < 0:
        x = 0
    elif x >= IMG_W:
        x = IMG_W - 1
    if y < 0:
        y = 0
    elif y >= IMG_H:
        y = IMG_H - 1
    try:
        p = None
        if _PIX_FAST:
            try:
                p = img.get_pixel(int(x), int(y), _PIXBUF)
            except Exception:
                p = None
        if p is None:
            p = img.get_pixel(int(x), int(y))
    except Exception:
        return -1
    if p is None:
        return -1
    if type(p) is int:
        return p
    try:
        if len(p) == 1:
            return p[0]
    except Exception:
        pass
    return luma(p[0], p[1], p[2])


def px_luma_arr(arr, x, y):
    """直接从 RGB888 ndarray 取亮度，避免 RGB888 get_pixel 失效。"""
    global _ARR_MODE
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
    if (_ARR_MODE == 0) or (_ARR_MODE == 1):
        try:
            p = arr[y][x]
            if len(p) >= 3:
                _ARR_MODE = 1
                return luma(int(p[0]), int(p[1]), int(p[2]))
        except Exception:
            pass
    if (_ARR_MODE == 0) or (_ARR_MODE == 3):
        try:
            p = arr[y, x]
            if len(p) >= 3:
                _ARR_MODE = 3
                return luma(int(p[0]), int(p[1]), int(p[2]))
        except Exception:
            pass
    try:
        row = arr[y]
        i = x * 3
        _ARR_MODE = 2
        return luma(int(row[i]), int(row[i + 1]), int(row[i + 2]))
    except Exception:
        return -1


_scene_cache = [0, 0.0, 0.0]


def scene_th(img, n_frame=0):
    """按直方图用大津法决定白纸的 LAB-L 阈值（0~100）。

    优先设备 Otsu；结果不可用或不在 [38,62] 时用采样 Otsu；
    仍不可用则回退“画面均值 + 8”并夹到 [PAPER_TH,72]。
    每 10 帧重算一次，避免逐帧抖动。
    """
    if PAPER_TH_FIXED > 0:
        _scene_cache[1] = float(PAPER_TH_FIXED)
        _scene_cache[2] = 4.0
        return int(PAPER_TH_FIXED)
    if (n_frame % 10) != 0 and _scene_cache[1] > 0:
        return int(_scene_cache[1])
    t = -1.0
    src = 3.0
    try:
        thr = img.get_histogram().get_threshold().value()
        t = float(thr) * 100.0 / 255.0
        src = 1.0
    except Exception:
        t = -1.0
    if (t < 38.0) or (t > 62.0):
        t = _otsu_from_samples(img)
        if (t >= 38.0) and (t <= 62.0):
            src = 2.0
    if (t < 38.0) or (t > 62.0):
        s = 0
        k = 0
        for gy in range(5):
            y = int(IMG_H * (gy + 0.5) / 5)
            for gx in range(6):
                x = int(IMG_W * (gx + 0.5) / 6)
                v = px_luma(img, x, y)
                if v >= 0:
                    s += v
                    k += 1
        if k <= 0:
            return PAPER_TH
        t = (s / float(k)) * 100.0 / 255.0 + 8.0
        if t < PAPER_TH:
            t = PAPER_TH
        elif t > 72.0:
            t = 72.0
        src = 3.0
    _scene_cache[1] = t
    _scene_cache[2] = src
    return int(t)


def _otsu_from_samples(img):
    """自实现大津法（设备直方图 API 不可用时的备胎）：
    40x30 网格采样 -> 32 桶直方图 -> 找双峰分界 -> 折回 0~100。
    QGimbal 的"确定性二值化"就靠这个保证：每 10 帧一定有一个"算出来的"阈值，
    不依赖任何单一 API。返回 -1 表示样本不足。"""
    hist = [0] * 32
    n = 0
    for gy in range(30):
        y = int(IMG_H * (gy + 0.5) / 30)
        for gx in range(40):
            x = int(IMG_W * (gx + 0.5) / 40)
            v = px_luma(img, x, y)
            if v >= 0:
                hist[v >> 3] += 1
                n += 1
    if n < 200:
        return -1.0
    total = 0.0
    for i in range(32):
        total += hist[i] * i
    sum_b = 0.0
    wb = 0
    best = -1.0
    th_lo = 0
    th_hi = 0
    for i in range(32):
        wb += hist[i]
        if wb == 0:
            continue
        wf = n - wb
        if wf == 0:
            break
        sum_b += hist[i] * i
        mb = sum_b / wb
        mf = (total - sum_b) / wf
        var = wb * wf * (mb - mf) * (mb - mf)
        if var > best * 1.002:
            best = var
            th_lo = i
            th_hi = i
        elif var >= best * 0.998:
            th_hi = i
    th = (th_lo + th_hi) * 0.5
    return (th + 0.5) * 8.0 * 100.0 / 255.0


def log_scene(img):
    """每 5 秒打一行"画面亮度"（6x8 网格自己采样），用来判断为什么没锁到靶：
      · 均/大 都很小（均<60 且 大<90）-> 画面太暗 / 镜头被挡 / 对着暗墙
      · 有大亮块（大>=200）但没靶     -> 靶纸不在视野里，或太远太小
      · 均 100+ 且有大亮块            -> 视野正常，那是靶纸判据/阈值的问题
    以前没有这行时，"候选0"（一个亮块都没有）只能靠猜。"""
    lo = 255
    hi = 0
    s = 0
    k = 0
    for gy in range(6):
        y = int(IMG_H * (gy + 0.5) / 6)
        for gx in range(8):
            x = int(IMG_W * (gx + 0.5) / 8)
            v = px_luma(img, x, y)
            if v < 0:
                continue
            if v < lo:
                lo = v
            if v > hi:
                hi = v
            s += v
            k += 1
    if k:
        _src = _scene_cache[2]
        _tag = "(Otsu)" if _src == 1.0 else \
               ("(自算Otsu)" if _src == 2.0 else
                ("(固定)" if _src == 4.0 else "(回退)"))
        log("[SCENE] 画面亮度 均=%.0f 小=%d 大=%d (0~255) 阈值=%d%s  靶=无靶时看这行"
            % (s / float(k), lo, hi, int(_scene_cache[1]), _tag))


def paper_contrast(img, x, y, w, h, use_arr=False):
    """内亮外暗校验：返回 (内部平均亮度, 外侧平均亮度, 外侧合格比例)。

    外侧取 12 个点（四边各 3 个），要求其中至少 PAPER_DARK_FRAC_MIN 的比例
    明显比内部暗 —— 背景柜子那种“只有一边有暗边”的亮块因此过不了。
    """
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
    outer_vals = []
    if use_arr:
        get_luma = px_luma_arr
    else:
        get_luma = px_luma
    for p in u:
        v = get_luma(img, p[0], p[1])
        if v >= 0:
            si += v
            ni += 1
    for p in o:
        v = get_luma(img, p[0], p[1])
        outer_vals.append(v)
        if v >= 0:
            so += v
            no += 1
    if ni < 3 or no < 6:
        return -1, -1, 0.0
    ins = si / float(ni)
    ok = 0
    for v in outer_vals:
        if v >= 0 and v < (ins - PAPER_DARK_MARGIN):
            ok += 1
    return ins, so / float(no), ok / float(no)


def paper_frame_sides(img, x, y, w, h, ins):
    """近处黑胶带框校验：四边各 3 点，至少 2 点比纸面暗才算该边有框。

    反光可能破坏一边，所以允许 3/4 边；破坏两边以上就拒绝。
    """
    lim = ins - PAPER_DARK_MARGIN
    sides = 0
    for sx in (x - PROBE_OFF_PX, x + w + PROBE_OFF_PX):
        k = 0
        for fy in (0.25, 0.50, 0.75):
            v = px_luma(img, sx, y + h * fy)
            if v >= 0 and v < lim:
                k += 1
        if k >= 2:
            sides += 1
    for sy in (y - PROBE_OFF_PX, y + h + PROBE_OFF_PX):
        k = 0
        for fx in (0.25, 0.50, 0.75):
            v = px_luma(img, x + w * fx, sy)
            if v >= 0 and v < lim:
                k += 1
        if k >= 2:
            sides += 1
    return sides


def norm_bbox(bx, by, bw, bh, roi):
    """兼容 find_blobs 返回“相对 ROI”或“全图”两种坐标。"""
    if roi[0] or roi[1]:
        if (bx + bw / 2.0) < roi[0] or (by + bh / 2.0) < roi[1]:
            return bx + roi[0], by + roi[1]
    return bx, by


def merge_fragments(blobs, roi):
    """把"被印刷圈/阴影切碎的靶纸"重新拼成一块，**但绝不吸收背景大块**。

    为什么不用 find_blobs(merge=True)：那个 merge 是按**外接框**做的 ——
    两块的外接框一靠近（margin 之内）就并成一块。于是只要背景里有个大亮块
    （白墙/亮桌面），它的外接框罩住靶纸，靶纸就被吞进去，最后只剩一个
    "26 万像素 框800 ✗框长"的大块：**真靶纸永远不作为候选出现**。
    这里的规则：只拼“嵌套”的碎片 —— 印刷圈把纸切成同心环时，
    内圈的外接框在外圈里面，这种情况必须拼；相邻但不嵌套的亮块
    （反光、背景）一律不拼。
      · 嵌套但尺寸差很多时，只有“外环密度低 + 内盘同心 + 内盘不是噪点”
        才拼：这是近处印刷圈产生的中心白盘，必须并回外环；而背景大块
        罩住靶纸时外块是实心的（密度高），不会被这条规则吞掉；
      · 两块各自的长度 <= PAPER_MAX_LONG*1.3（"整幅背景大块"在这一步就被挡住）；
      · 并起来之后：长边 <= PAPER_MAX_LONG、长宽比 <= PAPER_ASPECT_MAX、
        面积 <= PAPER_MAX_AREA_RATIO × 画面（拼完还得像一张靶纸）。
    输入/输出都是 [(x, y, w, h, px), ...]（已归一化到全图坐标），
    输出按面积从大到小排。
    """
    items = []
    for b in blobs:
        x, y, w, h, px = b[0], b[1], b[2], b[3], b[4]
        x, y = norm_bbox(x, y, w, h, roi)
        items.append([x, y, w, h, px])
    lim = PAPER_MAX_LONG * 1.3
    amax = PAPER_MAX_AREA_RATIO * IMG_W * IMG_H
    again = True
    while again:
        again = False
        for i in range(len(items)):
            for j in range(i + 1, len(items)):
                a = items[i]
                c = items[j]
                la = a[2] if a[2] > a[3] else a[3]
                lc = c[2] if c[2] > c[3] else c[3]
                if (la > lim) or (lc > lim):
                    continue
                nested = ((a[0] >= c[0]) and (a[1] >= c[1]) and
                          (a[0] + a[2] <= c[0] + c[2]) and
                          (a[1] + a[3] <= c[1] + c[3])) or \
                         ((c[0] >= a[0]) and (c[1] >= a[1]) and
                          (c[0] + c[2] <= a[0] + a[2]) and
                          (c[1] + c[3] <= a[1] + a[3]))
                if not nested:
                    continue
                if nested and ((la < lc * 0.75) or (lc < la * 0.75)):
                    if la >= lc:
                        outer, inner, ol = a, c, la
                    else:
                        outer, inner, ol = c, a, lc
                    oa = outer[2] * outer[3]
                    ia = inner[2] * inner[3]
                    ocx = outer[0] + outer[2] * 0.5
                    ocy = outer[1] + outer[3] * 0.5
                    icx = inner[0] + inner[2] * 0.5
                    icy = inner[1] + inner[3] * 0.5
                    od = outer[4] / float(oa) if oa > 0 else 0.0
                    if (od > MERGE_RING_DENSITY_MAX) or \
                            (abs(icx - ocx) > MERGE_CENTER_K * ol) or \
                            (abs(icy - ocy) > MERGE_CENTER_K * ol) or \
                            (ia < MERGE_INNER_AREA_MIN * oa):
                        continue
                ux = min(a[0], c[0])
                uy = min(a[1], c[1])
                uw = max(a[0] + a[2], c[0] + c[2]) - ux
                uh = max(a[1] + a[3], c[1] + c[3]) - uy
                ulong = uw if uw > uh else uh
                ushort = uw if uw < uh else uh
                if (ulong > PAPER_MAX_LONG) or \
                        (ulong / max(1.0, ushort) > PAPER_ASPECT_MAX) or \
                        (uw * uh > amax):
                    continue
                items[i] = [ux, uy, uw, uh, a[4] + c[4]]
                del items[j]
                again = True
                break
            if again:
                break
    items.sort(key=lambda it: -it[4])
    return items


def _scan_th(ref):
    """按纸面自身亮度定的扫描门槛（纸面有阴影时也能摸到真边）。"""
    t = ref * SCAN_TH_K
    if t < SCAN_TH_LO:
        t = SCAN_TH_LO
    elif t > SCAN_TH_HI:
        t = SCAN_TH_HI
    return t


def _requad(img, seed):
    """快速复测：拿上次的四边形当种子只重扫四条边（省一次 find_blobs 的固定开销）。

    seed/main 的测量元组: (px,x,y,w,h,long,aspect,den,ins,outs,corners,cx,cy)
    返回同样格式的 (best, 亮块数=0, 诊断串)。
    """
    px = seed[0]
    c = seed[10]
    if c is None:
        return None, 0, ""
    xs = (c[0][0], c[1][0], c[2][0], c[3][0])
    ys = (c[0][1], c[1][1], c[2][1], c[3][1])
    bx = min(xs)
    by = min(ys)
    w = max(xs) - bx
    h = max(ys) - by
    if w < 8 or h < 8:
        return None, 0, ""
    ins, outs, frac = paper_contrast(img, bx, by, w, h)
    if (ins < 0) or ((ins - outs) < PAPER_CONTRAST_MIN) or \
            (frac < PAPER_DARK_FRAC_MIN):
        return None, 0, "[复测:亮度] "
    q = quad_from_blob(img, bx, by, w, h, _scan_th(ins))
    if q is None:
        return None, 0, "[复测:拟合] "
    corners, ctr, long_side, short_side, qa = q
    if (qa < QUAD_AREA_LO * px) or (qa > QUAD_AREA_HI * px):
        return None, 0, "[复测:面积] "
    if seed[5] >= QUAD_NEAR_LONG:
        box_long = w if w > h else h
        if (long_side < QUAD_NEAR_SIDE_K * box_long) or \
                (qa < QUAD_NEAR_FILL_K * w * h) or \
                _quad_inside_bad(corners, bx, by, w, h, QUAD_NEAR_EDGE_K):
            return None, 0, "[复测:内圈] "
    aspect = long_side / max(1.0, short_side)
    if (long_side < PAPER_MIN_LONG) or (long_side > PAPER_MAX_LONG) or \
            (aspect < PAPER_ASPECT_MIN) or (aspect > PAPER_ASPECT_MAX):
        return None, 0, "[复测:尺寸] "
    return (px, bx, by, w, h, long_side, aspect, px / max(1.0, qa),
            ins, outs, corners, ctr[0], ctr[1]), 0, ""


def _probe(img, x, y, dx, dy, th):
    """沿射线前后各 1px 取样，多数（>=2/3）亮才算“纸”。

    印刷圆圈的细线在“圆的正左/正右”几乎与横向射线垂直，只做垂直方向
    取最大跨不过去（扫描会停在第一圈圆环，四边形缩到圆环范围）；
    沿射线取多数则与细线角度无关。
    复测里每帧要上千次 _probe，3 个采样是速度和抗印刷圈之间的折中。
    """
    if dx:
        offs = ((x - 1, y), (x, y), (x + 1, y))
    else:
        offs = ((x, y - 1), (x, y), (x, y + 1))
    k = 0
    for p in offs:
        v = px_luma(img, p[0], p[1])
        if v >= th:
            k += 1
    return k >= 2


def _edge(img, xc, yc, dx, dy, limit, th):
    """从 (xc,yc) 沿 (dx,dy) 二分找最后一个“纸”像素的距离；找不到给 -1。"""
    if not _probe(img, xc, yc, dx, dy, th):
        return -1
    lo = 0
    hi = limit
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if _probe(img, xc + dx * mid, yc + dy * mid, dx, dy, th):
            lo = mid
        else:
            hi = mid - 1
    return lo


def _lsq(pts):
    """最小二乘拟合 v = a*u + b（点数<2 返回 None）。"""
    n = len(pts)
    if n < 2:
        return None
    su = sv = suu = suv = 0.0
    for u, v in pts:
        su += u
        sv += v
        suu += u * u
        suv += u * v
    den = n * suu - su * su
    if abs(den) < 1e-6:
        return None
    a = (n * suv - su * sv) / den
    return a, (sv - a * su) / float(n)


def _lsq_robust(pts):
    """Theil-Sen：点对斜率取中位数。斜视时部分扫描点会打到相邻边，
    普通最小二乘会被带偏几十像素，中位数法最多容忍一半坏点。"""
    n = len(pts)
    if n < 2:
        return None
    if n == 2:
        (u1, v1), (u2, v2) = pts[0], pts[1]
        if abs(u2 - u1) < 1e-6:
            return None
        a = (v2 - v1) / (u2 - u1)
        return a, v1 - a * u1
    sl = []
    for i in range(n - 1):
        for j in range(i + 1, n):
            du = pts[j][0] - pts[i][0]
            if (du > 1.5) or (du < -1.5):
                sl.append((pts[j][1] - pts[i][1]) / du)
    if len(sl) < 2:
        return _lsq(pts)
    sl.sort()
    m = len(sl)
    a = sl[m // 2] if (m % 2) else 0.5 * (sl[m // 2 - 1] + sl[m // 2])
    bs = []
    for u, v in pts:
        bs.append(v - a * u)
    bs.sort()
    nb = len(bs)
    b = bs[nb // 2] if (nb % 2) else 0.5 * (bs[nb // 2 - 1] + bs[nb // 2])
    return a, b


def _cross(e1, e2):
    """竖边 x = a*y+b 与横边 y = a*x+b 的交点。"""
    al, bl = e1
    at, bt = e2
    den = 1.0 - al * at
    if abs(den) < 1e-3:
        return None
    x = (al * bt + bl) / den
    return (x, at * x + bt)


def _quad_angle_bad(corners, cos_lim=0.574):
    """QGimbal-Vision 同款判据：矩形四个内角都应接近 90°（±35° -> |cos|≤0.574）。
    返回 True = 不合格（弃用四边形、退回外接框）。斜视透视下角度会偏离直角，
    容差放到 ±35°；超过这个限度的一定不是我们的靶纸（是乱接边的糊块）。"""
    for i in range(4):
        p0 = corners[i]
        p1 = corners[(i + 1) % 4]
        p2 = corners[(i + 2) % 4]
        v1x = p0[0] - p1[0]
        v1y = p0[1] - p1[1]
        v2x = p2[0] - p1[0]
        v2y = p2[1] - p1[1]
        n1 = (v1x * v1x + v1y * v1y) ** 0.5
        n2 = (v2x * v2x + v2y * v2y) ** 0.5
        if (n1 < 1e-3) or (n2 < 1e-3):
            return True
        c = (v1x * v2x + v1y * v2y) / (n1 * n2)
        if c > 1.0:
            c = 1.0
        elif c < -1.0:
            c = -1.0
        if (c > cos_lim) or (c < -cos_lim):
            return True
    return False


def _quad_inside_bad(corners, x, y, w, h, k):
    """近处四边形必须贴着亮块外框。

    如果四个角都缩在框内一大截，说明边扫描停在印刷圈上，不是靶纸外沿。
    返回 True = 不合格（弃用四边形，退回这个亮块的框中心）。
    """
    for p in corners:
        dx = min(abs(p[0] - x), abs(p[0] - (x + w)))
        dy = min(abs(p[1] - y), abs(p[1] - (y + h)))
        if (dx > w * k) and (dy > h * k):
            return True
    return False


def quad_from_blob(img, bx, by, bw, bh, th, pad=None):
    """把亮块拟合成四边形（斜视=梯形）。
    返回 (corners, center, long_side, short_side, quad_area) 或 None；
    center = 两条对角线交点 = 透视意义下的靶心。"""
    if pad is None:
        pad = QUAD_LIM_PAD
    icx = int(bx + bw / 2.0)
    icy = int(by + bh / 2.0)
    lim_l = icx - int(bx) + pad
    lim_r = int(bx + bw) - icx + pad
    lim_u = icy - int(by) + pad
    lim_d = int(by + bh) - icy + pad
    left = []
    right = []
    top = []
    bot = []
    for k in range(1, QUAD_SCAN_N + 1):
        yy = int(by + bh * k / float(QUAD_SCAN_N + 1))
        dl = _edge(img, icx, yy, -1, 0, lim_l, th)
        dr = _edge(img, icx, yy, 1, 0, lim_r, th)
        if dl >= 0:
            left.append((yy, icx - dl))
        if dr >= 0:
            right.append((yy, icx + dr))
        xx = int(bx + bw * k / float(QUAD_SCAN_N + 1))
        du = _edge(img, xx, icy, 0, -1, lim_u, th)
        dd = _edge(img, xx, icy, 0, 1, lim_d, th)
        if du >= 0:
            top.append((xx, icy - du))
        if dd >= 0:
            bot.append((xx, icy + dd))
    if (len(left) < 3) or (len(right) < 3) or \
            (len(top) < 3) or (len(bot) < 3):
        return None
    e_l = _lsq_robust(left)
    e_r = _lsq_robust(right)
    e_t = _lsq_robust(top)
    e_b = _lsq_robust(bot)
    if (e_l is None) or (e_r is None) or (e_t is None) or (e_b is None):
        return None
    tl = _cross(e_l, e_t)
    tr = _cross(e_r, e_t)
    br = _cross(e_r, e_b)
    bl = _cross(e_l, e_b)
    if (tl is None) or (tr is None) or (br is None) or (bl is None):
        return None
    c = [tl, tr, br, bl]
    bad = []
    for i in range(4):
        p = c[i]
        if (p[0] < bx - 5) or (p[0] > bx + bw + 5) or \
                (p[1] < by - 5) or (p[1] > by + bh + 5):
            bad.append(i)
    if len(bad) == 1:
        i = bad[0]
        a = c[(i - 1) % 4]
        b = c[(i + 1) % 4]
        d = c[(i + 2) % 4]
        c[i] = (a[0] + b[0] - d[0], a[1] + b[1] - d[1])
    elif len(bad) > 1:
        return None
    for p in c:
        if (p[0] < 1) or (p[0] > IMG_W - 2) or \
                (p[1] < 1) or (p[1] > IMG_H - 2):
            return None
    c = (c[0], c[1], c[2], c[3])
    qa = 0.0
    for i in range(4):
        x1, y1 = c[i]
        x2, y2 = c[(i + 1) % 4]
        qa += x1 * y2 - x2 * y1
    qa = abs(qa) * 0.5
    if qa < 1.0:
        return None
    d = []
    for i in range(4):
        x1, y1 = c[i]
        x2, y2 = c[(i + 1) % 4]
        d.append(math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))
    dl1 = (d[0] + d[2]) / 2.0
    dl2 = (d[1] + d[3]) / 2.0
    if dl1 >= dl2:
        long_side, short_side = dl1, dl2
    else:
        long_side, short_side = dl2, dl1
    d1x, d1y = br[0] - tl[0], br[1] - tl[1]
    d2x, d2y = bl[0] - tr[0], bl[1] - tr[1]
    den = d1x * d2y - d1y * d2x
    if abs(den) < 1e-6:
        ctr = ((tl[0] + br[0]) / 2.0, (tl[1] + br[1]) / 2.0)
    else:
        t = ((tr[0] - tl[0]) * d2y - (tr[1] - tl[1]) * d2x) / den
        ctr = (tl[0] + t * d1x, tl[1] + t * d1y)
    if (ctr[0] < 0) or (ctr[0] >= IMG_W) or \
            (ctr[1] < 0) or (ctr[1] >= IMG_H):
        return None
    return (c, ctr, long_side, max(1.0, short_side), qa)


class PaperDetector(object):
    """A4 靶纸检测（C 加速 find_blobs）+ 三道闸门 + 连续确认 + 平滑。

    返回 dict: center/box/area/aspect/density/long_side/contrast/n/state
    state: 锁定 / 确认x/3 / 搜索
    详细判据与依据见 tools/vision_check.py 顶部注释。
    """

    def __init__(self):
        self.u = IMG_W / 2.0
        self.v = IMG_H / 2.0
        self.w = 0.0
        self.h = 0.0
        self.have = False
        self.lost = 0
        self.meas = None
        self.corners = None
        self.pend = None
        self.frame = 0
        self.last_n = 0
        self.last_dbg = ""
        self.cands = []
        self.did_full = False
        self.state = "搜索"
        self.err = 0
        self.err_msg = ""
        self.last_long = 0.0
        self.vel_u = 0.0
        self.vel_v = 0.0
        self.prev_cu = None
        self.prev_cv = None

    def roi(self, expand=1.0):
        L = self.last_long if self.last_long > 1.0 else max(self.w, self.h)
        pu = self.u + self.vel_u * 0.70
        pv = self.v + self.vel_v * 0.70
        if pu < 0:
            pu = 0
        elif pu >= IMG_W:
            pu = IMG_W - 1
        if pv < 0:
            pv = 0
        elif pv >= IMG_H:
            pv = IMG_H - 1
        _v = abs(self.vel_u)
        if abs(self.vel_v) > _v:
            _v = abs(self.vel_v)
        r = int(L * 0.60 + 18 + min(36.0, _v * 2.0))
        if self.lost > 0:
            r += min(TRACK_PAD_FAR, self.lost * 8)
        r = int(r * expand)
        x = int(pu - r)
        y = int(pv - r)
        if x < 0:
            x = 0
        if y < 0:
            y = 0
        w = 2 * r
        h = 2 * r
        if x + w > IMG_W:
            w = IMG_W - x
        if y + h > IMG_H:
            h = IMG_H - y
        return (x, y, w, h)

    def fresh(self):
        return self.have and (self.lost < HOLD_FRAMES)

    def _size_ok(self, long_side, density=0.0, contrast=0.0):
        if not self.have:
            return True
        old = self.last_long if self.last_long > 1.0 else max(self.w, self.h)
        lo = SIZE_GATE_LO
        hi = SIZE_GATE_HI
        if self.lost > 0:
            k = 1.0 - 0.02 * self.lost
            if k < 0.6:
                k = 0.6
            lo = SIZE_GATE_LO * k
            hi = SIZE_GATE_HI / k
        if (lo * old) <= long_side <= (hi * old):
            return True
        return (density >= SIZE_RESCUE_DENSITY) and \
               (contrast >= SIZE_RESCUE_CONTRAST) and \
               (0.35 * old <= long_side <= SIZE_RESCUE_MAX * old)

    def _confirm(self, cand):
        """全图候选的连续确认。返回 True = 确认为目标。"""
        if cand is None:
            if self.pend is not None:
                self.pend[4] += 1
                if self.pend[4] > PENDING_MISS:
                    self.pend = None
            return False
        cu, cv, lo = cand[11], cand[12], cand[5]
        if self.have and (self.lost < 30):
            old = max(self.w, self.h)
            if (lo < 0.30 * old) or (lo > 3.0 * old):
                self.pend = None
                return False
        if self.pend is None:
            self.pend = [cu, cv, lo, 1, 0]
            if CONFIRM_N <= 1:
                self.pend = None
                return True
            return False
        if (abs(cu - self.pend[0]) <= CONFIRM_DXY) and \
                (abs(cv - self.pend[1]) <= CONFIRM_DXY) and \
                (abs(lo - self.pend[2]) <= CONFIRM_DSIZE * self.pend[2]):
            n = self.pend[3] + 1
            self.pend[0] += (cu - self.pend[0]) / float(n)
            self.pend[1] += (cv - self.pend[1]) / float(n)
            self.pend[2] += (lo - self.pend[2]) / float(n)
            self.pend[3] = n
            self.pend[4] = 0
            if n >= CONFIRM_N:
                self.pend = None
                return True
        else:
            self.pend = [cu, cv, lo, 1, 0]
        return False

    def _accept(self, cand):
        x, y, w, h = cand[1], cand[2], cand[3], cand[4]
        c = cand[10]
        cu, cv = cand[11], cand[12]
        old_l = self.last_long if self.last_long > 1.0 \
            else max(self.w, self.h)
        if self.have and (self.lost < LOST_FULL):
            _jump = max(JUMP_REJECT_PX * 0.55, old_l * 0.45) + 20.0
            _jump += self.lost * 8.0
            if abs(cu - self.u) > _jump:
                return False
            if abs(cv - self.v) > _jump:
                return False
            if old_l > 1.0:
                if (cand[5] < 0.40 * old_l) or (cand[5] > 2.50 * old_l):
                    return False
        if (not self.have) or (self.lost >= LOST_FULL):
            self.u, self.v, self.w, self.h = cu, cv, w, h
            self.have = True
            self.corners = c
            self.prev_cu, self.prev_cv = cu, cv
        else:
            if (self.lost == 0) and (self.prev_cu is not None):
                self.vel_u = 0.5 * self.vel_u + 0.5 * (cu - self.prev_cu)
                self.vel_v = 0.5 * self.vel_v + 0.5 * (cv - self.prev_cv)
            self.prev_cu, self.prev_cv = cu, cv
            if 0.0 < cand[5] < FAR_LONG_PX:
                k = SMOOTH_FAR
            elif cand[5] >= QUAD_NEAR_LONG:
                k = SMOOTH_NEAR
            else:
                k = SMOOTH
            if abs(cu - self.u) > DEADBAND_PX:
                self.u += k * (cu - self.u)
            if abs(cv - self.v) > DEADBAND_PX:
                self.v += k * (cv - self.v)
            if abs(w - self.w) > 2 * DEADBAND_PX:
                self.w += k * (w - self.w)
            if abs(h - self.h) > 2 * DEADBAND_PX:
                self.h += k * (h - self.h)
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
            else:
                self.corners = None
        self.meas = cand
        self.lost = 0
        self.state = "锁定"
        self.last_long = cand[5]
        return True

    def detect(self, img):
        self.frame += 1
        th_ad = scene_th(img, self.frame)
        if (PAPER_TH_NEAR_FIXED > 0) and self.have and \
                (self.lost < LOST_FULL) and \
                (self.last_long >= QUAD_NEAR_LONG):
            th_ad = PAPER_TH_NEAR_FIXED
            _scene_cache[1] = float(th_ad)
            _scene_cache[2] = 4.0
        if (self.frame % 8) == 0:
            th_alt = th_ad + 8
            if th_alt > 84:
                th_alt = 84
        else:
            th_alt = th_ad - 6
            if th_alt < 38:
                th_alt = 38
        cand = None
        n = 0
        dbg = ""
        self.did_full = False
        _stable = (self.lost == 0)
        _seed_every = 24 if _stable else SEED_EVERY
        _recheck_every = 60 if _stable else RE_CHECK_EVERY
        try:
            if self.have and self.lost < LOST_FULL:
                if ((self.frame % _seed_every) == 0) or (self.meas is None):
                    cand, n, dbg = self._find(img, self.roi(), th_ad)
                else:
                    cand, n, dbg = self._find(img, self.roi(), th_ad,
                                              self.meas)
                    if cand is None:
                        cand, n, dbg = self._find(img, self.roi(), th_ad)
                    if cand is None:
                        _roi2 = self.roi(1.6)
                        cand, n, dbg = self._find(img, _roi2, th_ad)
                if (cand is not None) and \
                        (not self._size_ok(cand[5], cand[7],
                                           cand[8] - cand[9])):
                    dbg = "[尺寸闸门%.0fpx] " % cand[5] + dbg
                    cand = None
                if (self.frame % _recheck_every) == 0:
                    cb, n2, d2 = self._find(img, (0, 0, IMG_W, IMG_H),
                                            th_ad, prefer=False)
                    self.did_full = True
                    n += n2
                    if cb is not None:
                        old_s = 0.0
                        if self.meas is not None:
                            old_s = self.meas[0] * (0.5 + min(max(
                                self.meas[8] - self.meas[9], 0), 80) / 80.0)
                        new_s = cb[0] * (0.5 + min(max(
                            cb[8] - cb[9], 0), 80) / 80.0)
                        cur_l = self.last_long if self.last_long > 1.0 \
                            else max(self.w, self.h)
                        cbn = (abs(cb[11] - SPOT_UV[0]) +
                               abs(cb[12] - SPOT_UV[1])) <= SPOT_PREF_PX
                        cun = (abs(self.u - SPOT_UV[0]) +
                               abs(self.v - SPOT_UV[1])) <= SPOT_PREF_PX
                        old_con = 0.0
                        if self.meas is not None:
                            old_con = max(self.meas[8] - self.meas[9], 0.0)
                        new_con = max(cb[8] - cb[9], 0.0)
                        if ((cbn and not cun) or (new_s > 1.6 * old_s) or
                                (new_con >= 60 and new_con >= 1.5 * old_con)) \
                                and \
                                (abs(cb[11] - self.u) +
                                 abs(cb[12] - self.v) > 40) and \
                                (0.35 * cur_l <= cb[5] <= 3.0 * cur_l):
                            self.u, self.v = cb[11], cb[12]
                            self.w, self.h = cb[3], cb[4]
                            self.corners = cb[10]
                            self.meas = cb
                            self.last_long = cb[5]
                            self.lost = 0
                            self.state = "锁定"
                            self.last_n = n
                            self.last_dbg = ("[全图复查换目标 %.0f->%.0f] "
                                             % (old_s, new_s)) + d2
                            return {"center": (self.u, self.v),
                                    "box": (cb[1], cb[2], cb[3], cb[4]),
                                    "area": cb[0], "aspect": cb[6],
                                    "density": cb[7], "long_side": cb[5],
                                    "contrast": cb[8] - cb[9], "n": n,
                                    "state": self.state,
                                    "corners": self.corners}
            else:
                self.did_full = True
                c2, n2, d2 = self._find(img, (0, 0, IMG_W, IMG_H), th_ad)
                if c2 is None and ((self.frame % 4) == 0):
                    c3, n3, d3 = self._find(img, (0, 0, IMG_W, IMG_H),
                                            th_alt)
                    c2, n2, d2 = c3, n2 + n3, d2 + d3
                n += n2
                dbg = d2
                if self._confirm(c2):
                    cand = c2
        except Exception as e:
            self.err += 1
            if self.err_msg != str(e):
                self.err_msg = str(e)
                print("靶纸检测异常: %s" % e)
            return None
        self.last_n = n
        self.last_dbg = dbg
        if cand is not None:
            if not self._accept(cand):
                cand = None
        if cand is None:
            self.lost += 1
            self.vel_u *= 0.5
            self.vel_v *= 0.5
            if self.pend is not None:
                self.state = "确认%d/%d" % (self.pend[3], CONFIRM_N)
            elif (not self.have) or (self.lost >= LOST_FULL):
                self.state = "搜索"
            else:
                self.state = "搜索"
        if cand is None:
            return None
        px, x, y, w, h, long_side, aspect, density, ins, outs = self.meas[:10]
        return {"center": (self.u, self.v), "box": (x, y, w, h),
                "area": px, "aspect": aspect, "density": density,
                "long_side": long_side, "contrast": ins - outs, "n": n,
                "state": self.state, "corners": self.corners}

    def _find(self, img, roi, th, seed=None, prefer=None):
        """在 roi 里找 A4 靶纸，返回 (best, 亮块数, 诊断字符串)。

        seed=上次测量元组时走快速复测（只重扫四边形，不找亮块）。"""
        if seed is not None:
            return _requad(img, seed)
        if prefer is None:
            prefer = self.have and (self.lost == 0)
        best = None
        n = 0
        dbg = ""
        self.cands = []
        if GRAY_DIRECT:
            _gth = int(max(0, min(255, th * 255 / 100.0)))
            _thresholds = [(_gth, 255)]
        else:
            _thresholds = [(th, 100, -PAPER_A_MAX, PAPER_A_MAX,
                            -PAPER_B_MAX, PAPER_B_MAX)]
        blobs = img.find_blobs(
            _thresholds,
            roi=roi, merge=False,
            area_threshold=PAPER_MIN_AREA,
            pixels_threshold=PAPER_MIN_AREA)
        if not blobs:
            return None, 0, ""
        blobs = merge_fragments(blobs, roi)
        if not blobs:
            return None, 0, ""
        for b in blobs:
            n += 1
            x, y, w, h, px = b[0], b[1], b[2], b[3], b[4]
            x, y = norm_bbox(x, y, w, h, roi)
            if w < 8 or h < 8:
                continue
            box_long = w if w > h else h
            why = ""
            corners = None
            qa = 0.0
            ins = 0
            outs = 0
            frac = 0.0
            near_sides = 4
            ctr = (x + w / 2.0, y + h / 2.0)
            if box_long < PAPER_MIN_LONG * 0.8 or \
                    box_long > PAPER_MAX_LONG * 1.3:
                why = "框长"
                long_side = box_long
                short_side = w if w < h else h
                density = px / float(w * h)
            elif box_long > (PAPER_ASPECT_MAX * 1.3) * min(w, h):
                why = "比例"
                long_side = box_long
                short_side = w if w < h else h
                density = px / float(w * h)
            else:
                ins, outs, frac = paper_contrast(img, x, y, w, h)
                q = quad_from_blob(img, x, y, w, h, _scan_th(ins))
                if q is not None:
                    corners, ctr, long_side, short_side, qa = q
                    if (qa < QUAD_AREA_LO * px) or (qa > QUAD_AREA_HI * px):
                        corners = None
                    elif (box_long >= QUAD_NEAR_LONG) and \
                            ((long_side < QUAD_NEAR_SIDE_K * box_long) or
                             (qa < QUAD_NEAR_FILL_K * w * h) or
                             _quad_inside_bad(corners, x, y, w, h,
                                              QUAD_NEAR_EDGE_K)):
                        corners = None
                    elif long_side < 0.70 * box_long:
                        corners = None
                    elif _quad_angle_bad(
                            corners,
                            QUAD_NEAR_COS_LIM
                            if box_long >= QUAD_NEAR_LONG else 0.574):
                        corners = None
                    else:
                        lim = QUAD_LIM_PAD + 8
                        for p in corners:
                            if (p[0] < x - lim) or (p[0] > x + w + lim) or \
                                    (p[1] < y - lim) or (p[1] > y + h + lim):
                                corners = None
                                break
                if corners is None:
                    long_side = box_long
                    short_side = w if w < h else h
                    density = px / float(w * h)
                    dens_min = PAPER_DENSITY_MIN
                    if (ins - outs) >= PAPER_CONTRAST_STRONG:
                        dens_min = PAPER_DENSITY_MIN_STRONG
                    if (box_long >= QUAD_NEAR_LONG) and \
                            (dens_min < PAPER_DENSITY_NEAR_MIN):
                        dens_min = PAPER_DENSITY_NEAR_MIN
                    ctr = (x + w / 2.0, y + h / 2.0)
                    if box_long >= QUAD_NEAR_LONG:
                        near_sides = paper_frame_sides(
                            img, x, y, w, h, ins)
                else:
                    if box_long >= QUAD_NEAR_LONG:
                        qx = qx2 = corners[0][0]
                        qy = qy2 = corners[0][1]
                        for p in corners:
                            if p[0] < qx:
                                qx = p[0]
                            elif p[0] > qx2:
                                qx2 = p[0]
                            if p[1] < qy:
                                qy = p[1]
                            elif p[1] > qy2:
                                qy2 = p[1]
                        qins, qouts, qfrac = paper_contrast(
                            img, qx, qy, qx2 - qx, qy2 - qy)
                        if qins >= 0:
                            ins, outs, frac = qins, qouts, qfrac
                        near_sides = paper_frame_sides(
                            img, qx, qy, qx2 - qx, qy2 - qy, ins)
                    density = px / max(1.0, qa)
                    dens_min = PAPER_DENSITY_QUAD_MIN
                    if (box_long >= QUAD_NEAR_LONG) and \
                            (dens_min < PAPER_DENSITY_NEAR_MIN):
                        dens_min = PAPER_DENSITY_NEAR_MIN
                aspect = long_side / max(1.0, short_side)
                if px < PAPER_MIN_AREA:
                    why = "面积"
                elif px > PAPER_MAX_AREA_RATIO * IMG_W * IMG_H:
                    why = "太大"
                elif long_side < PAPER_MIN_LONG or \
                        long_side > PAPER_MAX_LONG:
                    why = "长边"
                elif aspect < PAPER_ASPECT_MIN or \
                        aspect > PAPER_ASPECT_MAX:
                    why = "比例"
                elif (density < dens_min) or \
                        (density > (QUAD_NEAR_DENSITY_HI
                                    if (corners is not None and
                                        box_long >= QUAD_NEAR_LONG)
                                    else 1.08)):
                    why = "密度"
                elif (box_long >= QUAD_NEAR_LONG) and (near_sides < 3):
                    why = "黑框"
                elif ((x <= 1) or (y <= 1) or
                      ((x + w) >= IMG_W - 1) or ((y + h) >= IMG_H - 1)) and \
                        (corners is None) and (box_long < 90):
                    why = "贴边"
                elif ins < 0 or (ins - outs) < PAPER_CONTRAST_MIN:
                    why = "对比"
                elif frac < PAPER_DARK_FRAC_MIN:
                    why = "暗边"
            if n <= 6:
                dbg += "[%dpx 框%.0f 内%d 外%d 暗边%.0f%% 密%.2f%s%s] " % (
                    px, box_long, ins, outs, frac * 100.0, density,
                    ("四" if corners is not None else "框"),
                    (" ✗" + why) if why else " ✓")
            if len(self.cands) < 8:
                self.cands.append((x, y, w, h, 0 if why else 1))
            if why:
                continue
            score = px * (0.5 + min(ins - outs, 80) / 80.0)
            score *= 0.5 + 0.5 * (density if density < 1.0 else 1.0)
            if prefer and self.have and self.last_long > 1.0:
                d = abs(ctr[0] - self.u) + abs(ctr[1] - self.v)
                ds = abs(long_side - self.last_long) / self.last_long
                score *= 1.0 / (1.0 + d / 80.0 + ds * 2.0)
            dc = abs(ctr[0] - SPOT_UV[0]) + abs(ctr[1] - SPOT_UV[1])
            key = (1 if dc <= SPOT_PREF_PX else 0, score)
            if best is None or key > best[0]:
                best = (key, (px, x, y, w, h, long_side, aspect, density,
                              ins, outs, corners, ctr[0], ctr[1]))
        if best is None:
            return None, n, dbg
        bx, by, bw, bh = best[1][1], best[1][2], best[1][3], best[1][4]
        keep = []
        for cb in self.cands:
            ccx = cb[0] + cb[2] * 0.5
            ccy = cb[1] + cb[3] * 0.5
            is_best = (cb[0] == bx and cb[1] == by and
                       cb[2] == bw and cb[3] == bh)
            if (not is_best) and (bx <= ccx <= bx + bw) and \
                    (by <= ccy <= by + bh):
                continue
            keep.append((cb[0], cb[1], cb[2], cb[3],
                         1 if is_best else 0))
        self.cands = keep
        return best[1], n, dbg


class ClosePaperDetector(PaperDetector):
    """全帧二值闭运算检测器。

    二值化/闭运算负责生成白纸 mask；白块在 mask 上扫描四边，
    原始 RGB888 图像只做内亮外暗和黑框校验。
    """

    def __init__(self):
        PaperDetector.__init__(self)
        self.close_hit = False
        self.close_dbg = ""
        self.err_printed = False
        self.close_ms = 0
        self.quad_ms = 0
        self.close_view = None
        self.close_data = None
        self.close_roi = None
        self.close_miss_n = 0
        self.rgb_buf = None
        self.rgb_shape = None
        self.conv_logged = False
        self.cand_why = ""
        self.close_scale = 1
        self.mask_logged = False
        self.scan_dbg = ""
        self.debug_quad = None
        self.debug_score = 0.0

    def _close_roi(self):
        bw = bh = 0
        if self.meas is not None:
            bw = int(self.meas[3])
            bh = int(self.meas[4])
        if (bw < 20) or (bh < 20):
            bw = bh = int(self.last_long)
        rw = int(bw * CLOSE_ROI_K) + CLOSE_ROI_PAD
        rh = int(bh * CLOSE_ROI_K) + CLOSE_ROI_PAD
        x0 = int(self.u - rw)
        y0 = int(self.v - rh)
        if x0 < 0:
            x0 = 0
        if y0 < 0:
            y0 = 0
        x1 = int(self.u + rw)
        y1 = int(self.v + rh)
        if x1 > IMG_W:
            x1 = IMG_W
        if y1 > IMG_H:
            y1 = IMG_H
        return (x0, y0, x1 - x0, y1 - y0)

    @staticmethod
    def _order4(q):
        """把四个角按左上 -> 右上 -> 右下 -> 左下排好。"""
        cx = (q[0][0] + q[1][0] + q[2][0] + q[3][0]) * 0.25
        cy = (q[0][1] + q[1][1] + q[2][1] + q[3][1]) * 0.25
        s = sorted(q, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
        i0 = min(range(4), key=lambda i: s[i][0] + s[i][1])
        return tuple(s[i0:] + s[:i0])

    @staticmethod
    def _mask_white(view, x, y):
        """灰度闭运算图取点：只走单通道 get_pixel，不用 RGB 的 _PIXBUF。"""
        try:
            p = view.get_pixel(int(x), int(y))
        except Exception:
            return False
        if p is None:
            return False
        try:
            v = int(p)
            return (v >= 128) or (v == 1)
        except Exception:
            pass
        try:
            v = int(p.item())
            return (v >= 128) or (v == 1)
        except Exception:
            pass
        try:
            if len(p) == 1:
                v = int(p[0])
            else:
                v = (int(p[0]) * 77 + int(p[1]) * 150 +
                     int(p[2]) * 29) >> 8
            return (v >= 128) or (v == 1)
        except Exception:
            return False

    def _quad_metrics(self, q, strict=True):
        """返回 (四角, 对角线交点, 长边, 短边, 四边形面积)。"""
        c = self._order4(q)
        if strict:
            for p in c:
                if (p[0] < 1) or (p[0] > IMG_W - 2) or \
                        (p[1] < 1) or (p[1] > IMG_H - 2):
                    self.scan_dbg = "metrics:角点越界 %r" % (c,)
                    return None
        qa = 0.0
        for i in range(4):
            x1, y1 = c[i]
            x2, y2 = c[(i + 1) % 4]
            qa += x1 * y2 - x2 * y1
        qa = abs(qa) * 0.5
        if strict and (qa < 1.0):
            self.scan_dbg = "metrics:面积"
            return None
        d = []
        for i in range(4):
            x1, y1 = c[i]
            x2, y2 = c[(i + 1) % 4]
            d.append(math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))
        dl1 = (d[0] + d[2]) * 0.5
        dl2 = (d[1] + d[3]) * 0.5
        if dl1 >= dl2:
            long_side, short_side = dl1, dl2
        else:
            long_side, short_side = dl2, dl1
        tl, tr, br, bl = c
        d1x, d1y = br[0] - tl[0], br[1] - tl[1]
        d2x, d2y = bl[0] - tr[0], bl[1] - tr[1]
        den = d1x * d2y - d1y * d2x
        if abs(den) < 1e-6:
            ctr = ((tl[0] + br[0]) * 0.5, (tl[1] + br[1]) * 0.5)
        else:
            t = ((tr[0] - tl[0]) * d2y -
                 (tr[1] - tl[1]) * d2x) / den
            ctr = (tl[0] + t * d1x, tl[1] + t * d1y)
        return c, ctr, long_side, max(1.0, short_side), qa

    def _make_cand(self, img, bx, by, bw, bh, corners, ctr,
                   long_side, short_side, qa, ins=None, outs=None, frac=None,
                   sides_min=3):
        """把一套四角过完整的近处闸门，变成 _find 的候选元组。"""
        box_long = bw if bw > bh else bh
        aspect = long_side / max(1.0, short_side)
        if box_long >= QUAD_NEAR_LONG:
            if (long_side < QUAD_NEAR_SIDE_K * box_long) or \
                    (qa < QUAD_NEAR_FILL_K * bw * bh) or \
                    _quad_inside_bad(corners, bx, by, bw, bh,
                                     QUAD_NEAR_EDGE_K):
                self.cand_why = "近边"
                return None
            cos_lim = CLOSE_FRAME_COS if FULL_FRAME_CLOSE \
                else QUAD_NEAR_COS_LIM
        else:
            cos_lim = CLOSE_FRAME_COS if FULL_FRAME_CLOSE else 0.574
        if _quad_angle_bad(corners, cos_lim):
            self.cand_why = "角"
            return None
        if (long_side < PAPER_MIN_LONG) or \
                (long_side > PAPER_MAX_LONG) or \
                (aspect < PAPER_ASPECT_MIN) or \
                (aspect > PAPER_ASPECT_MAX):
            self.cand_why = "尺寸"
            return None
        if ins is None:
            ins, outs, frac = paper_contrast(img, bx, by, bw, bh)
        if (ins < 0) or ((ins - outs) < PAPER_CONTRAST_MIN) or \
                (frac < PAPER_DARK_FRAC_MIN):
            self.cand_why = "对比"
            return None
        sides = paper_frame_sides(img, bx, by, bw, bh, ins)
        if sides < sides_min:
            self.cand_why = "黑%d" % sides
            return None
        self.cand_why = ""
        return (int(qa), bx, by, bw, bh, long_side, aspect,
                1.0, ins, outs, corners, ctr[0], ctr[1])

    def _scan_quad(self, view, bx, by, bw, bh, ox, oy):
        """在闭运算白块 bbox 内扫四条外边，再拟合四边形。

        bx/by/bw/bh 是闭运算 ROI 内的局部 bbox；ox/oy 是 ROI 在全图的偏移。
        从左/右/上/下边界向里扫最外白像素，所以中心那个小黑点不会
        像原来的“从中心出发”扫描那样把边扫描带偏。
        """
        self.scan_dbg = ""
        self.debug_quad = None
        left = []
        right = []
        top = []
        bot = []
        for k in range(1, QUAD_SCAN_N + 1):
            yy = by + int(bh * k / float(QUAD_SCAN_N + 1))
            xl = -1
            xr = -1
            for x in range(bx, bx + bw):
                if self._mask_white(view, x, yy):
                    xl = x
                    break
            for x in range(bx + bw - 1, bx - 1, -1):
                if self._mask_white(view, x, yy):
                    xr = x
                    break
            if (xl >= 0) and (xr > xl):
                left.append((yy + oy, xl + ox))
                right.append((yy + oy, xr + ox))
            xx = bx + int(bw * k / float(QUAD_SCAN_N + 1))
            yt = -1
            yb = -1
            for y in range(by, by + bh):
                if self._mask_white(view, xx, y):
                    yt = y
                    break
            for y in range(by + bh - 1, by - 1, -1):
                if self._mask_white(view, xx, y):
                    yb = y
                    break
            if (yt >= 0) and (yb > yt):
                top.append((xx + ox, yt + oy))
                bot.append((xx + ox, yb + oy))
        if (len(left) < 3) or (len(right) < 3) or \
                (len(top) < 3) or (len(bot) < 3):
            self.scan_dbg = ("L%dR%dT%dB%d bbox=%d,%d,%dx%d"
                             % (len(left), len(right), len(top), len(bot),
                                bx, by, bw, bh))
            return None
        e_l = _lsq_robust(left)
        e_r = _lsq_robust(right)
        e_t = _lsq_robust(top)
        e_b = _lsq_robust(bot)
        if (e_l is None) or (e_r is None) or \
                (e_t is None) or (e_b is None):
            self.scan_dbg = "line"
            return None
        tl = _cross(e_l, e_t)
        tr = _cross(e_r, e_t)
        br = _cross(e_r, e_b)
        bl = _cross(e_l, e_b)
        if (tl is None) or (tr is None) or (br is None) or (bl is None):
            self.scan_dbg = "cross"
            return None
        self.debug_quad = (tl, tr, br, bl)
        m = self._quad_metrics((tl, tr, br, bl), strict=False)
        if (m is None) and (not self.scan_dbg):
            self.scan_dbg = "metrics"
        return m

    def _full_candidate(self, img):
        """二值 mask 上按 10.13 的 _find 逻辑检测靶纸。"""
        self.close_view = None
        self.close_data = None
        self.close_roi = None
        self.close_ms = 0
        self.quad_ms = 0
        arr = None
        try:
            t0 = time.ticks_ms()
            binary_np = None
            try:
                arr = img.to_numpy_ref()
                if GRAY_DIRECT:
                    binary_np = _cv_lite.grayscale_threshold_binary(
                        [IMG_H, IMG_W], arr, CLOSE_BIN_TH, 255)
                else:
                    binary_np = _cv_lite.rgb888_threshold_binary(
                        [IMG_H, IMG_W], arr, CLOSE_BIN_TH, 255)
                view = image.Image(
                    IMG_W, IMG_H, image.GRAYSCALE,
                    alloc=image.ALLOC_REF, data=binary_np)
            except Exception:
                arr = None
                try:
                    view = img.to_grayscale(copy=True)
                except Exception:
                    view = img.to_grayscale()
                view.binary([(CLOSE_BIN_TH, 255)])
                try:
                    view = view.to_grayscale()
                except Exception:
                    pass
            view.close(CLOSE_MORPH_SIZE)
            self.close_ms = time.ticks_diff(time.ticks_ms(), t0)
            if (arr is not None) and (not self.mask_logged):
                self.mask_logged = True
                if GRAY_DIRECT:
                    _v0 = px_luma(img, IMG_W // 2, IMG_H // 2)
                    print("[CLOSE] 灰度直出 中心=%d" % _v0)
                else:
                    _v0 = px_luma_arr(arr, IMG_W // 2, IMG_H // 2)
                    print("[CLOSE] ndarray取亮度 mode=%d 中心=%d"
                          % (_ARR_MODE, _v0))
            self.close_view = view
            self.close_data = binary_np
            self.close_roi = (0, 0, IMG_W, IMG_H)
            self.close_scale = 1
            t0 = time.ticks_ms()
            blobs = view.find_blobs(
                [(1, 255)], merge=False,
                area_threshold=PAPER_MIN_AREA,
                pixels_threshold=PAPER_MIN_AREA)
        except Exception as e:
            if not self.err_printed:
                self.err_printed = True
                print("全帧闭运算异常(%s)" % einfo(e))
            return None, 0, ""
        if not blobs:
            self.quad_ms = time.ticks_diff(time.ticks_ms(), t0)
            return None, 0, "[闭运算:无白块] "
        self.cands = []
        self.debug_quad = None
        self.debug_score = 0.0
        best = None
        n = 0
        dbg = ""
        for b in blobs:
            n += 1
            x, y, w, h, px = b[0], b[1], b[2], b[3], b[4]
            if w < 8 or h < 8:
                continue
            box_long = w if w > h else h
            use_arr = (not GRAY_DIRECT) and (arr is not None)
            ins, outs, frac = paper_contrast(
                arr if use_arr else img, x, y, w, h, use_arr)
            if (ins < 0) and use_arr:
                ins, outs, frac = paper_contrast(img, x, y, w, h, False)
            why = ""
            corners = None
            qa = 0.0
            ctr = (x + w / 2.0, y + h / 2.0)
            if self.have and (self.lost < 5):
                _max_d = max(45.0, self.last_long * 0.55) + self.lost * 10.0
                if abs(ctr[0] - self.u) + abs(ctr[1] - self.v) > _max_d:
                    continue
            if box_long < PAPER_MIN_LONG * 0.8 or \
                    box_long > PAPER_MAX_LONG * 1.3:
                why = "框长"
                long_side = box_long
                short_side = w if w < h else h
                density = px / float(w * h)
            else:
                # 只扫本白块 bbox，不跨到旁边的亮块
                q = quad_from_blob(view, x, y, w, h, 1, 0)
                if q is not None:
                    corners, ctr, long_side, short_side, qa = q
                    if (qa < QUAD_AREA_LO * px) or (qa > QUAD_AREA_HI * px):
                        corners = None
                    elif long_side < 0.70 * box_long:
                        corners = None
                    elif _quad_angle_bad(corners):
                        corners = None
                    else:
                        lim = 8
                        for p in corners:
                            if (p[0] < x - lim) or (p[0] > x + w + lim) or \
                                    (p[1] < y - lim) or (p[1] > y + h + lim):
                                corners = None
                                break
                if corners is None:
                    long_side = box_long
                    short_side = w if w < h else h
                    density = px / float(w * h)
                    dens_min = PAPER_DENSITY_MIN
                    if (ins - outs) >= PAPER_CONTRAST_STRONG:
                        dens_min = PAPER_DENSITY_MIN_STRONG
                    ctr = (x + w / 2.0, y + h / 2.0)
                else:
                    density = px / max(1.0, qa)
                    dens_min = PAPER_DENSITY_QUAD_MIN
                aspect = long_side / max(1.0, short_side)
                if px < PAPER_MIN_AREA:
                    why = "面积"
                elif px > PAPER_MAX_AREA_RATIO * IMG_W * IMG_H:
                    why = "太大"
                elif long_side < PAPER_MIN_LONG or \
                        long_side > PAPER_MAX_LONG:
                    why = "长边"
                elif aspect < PAPER_ASPECT_MIN or \
                        aspect > PAPER_ASPECT_MAX:
                    why = "比例"
                elif (density < dens_min) or (density > 1.08):
                    why = "密度"
                elif ((x <= 1) or (y <= 1) or
                      ((x + w) >= IMG_W - 1) or ((y + h) >= IMG_H - 1)) and \
                        (corners is None) and (box_long < 90):
                    why = "贴边"
                elif ins < 0 or (ins - outs) < PAPER_CONTRAST_MIN:
                    why = "对比"
                elif frac < PAPER_DARK_FRAC_MIN:
                    why = "暗边"
            if n <= 6:
                dbg += "[%dpx 框%.0f 内%d 外%d 暗边%.0f%% 密%.2f%s%s] " % (
                    px, box_long, ins, outs, frac * 100.0, density,
                    ("四" if corners is not None else "框"),
                    (" ✗" + why) if why else " ✓")
            if len(self.cands) < 8:
                self.cands.append((x, y, w, h, 0 if why else 1))
            _dc = abs(ctr[0] - SPOT_UV[0]) + abs(ctr[1] - SPOT_UV[1])
            dbg_score = px / (1.0 + _dc / 40.0)
            if dbg_score > self.debug_score:
                self.debug_score = dbg_score
                if corners is not None:
                    self.debug_quad = corners
                else:
                    self.debug_quad = ((x, y), (x + w, y),
                                       (x + w, y + h), (x, y + h))
            if why:
                continue
            score = px * (0.5 + min(ins - outs, 80) / 80.0)
            score *= 0.5 + 0.5 * (density if density < 1.0 else 1.0)
            if self.have and (self.lost < LOST_FULL) and \
                    (self.last_long > 1.0):
                _d = abs(ctr[0] - self.u) + abs(ctr[1] - self.v)
                _ds = abs(long_side - self.last_long) / self.last_long
                score *= 1.0 / (1.0 + _d / 80.0 + _ds * 2.0)
            if self.have and (self.lost < LOST_FULL):
                key = (0, score)
            else:
                key = (1 if _dc <= SPOT_PREF_PX else 0, score)
            if best is None or key > best[0]:
                best = (key, (px, x, y, w, h, long_side, aspect, density,
                              ins, outs, corners, ctr[0], ctr[1]))
        self.quad_ms = time.ticks_diff(time.ticks_ms(), t0)
        if best is None:
            return None, n, dbg
        return best[1], n, dbg


    def _near_candidate(self, img, roi):
        """ROI 闭运算 -> 最大白块 -> 原来的四边扫描，返回候选元组。"""
        x0, y0, rw, rh = roi
        self.close_view = None
        self.close_data = None
        self.close_roi = None
        self.close_ms = 0
        self.quad_ms = 0
        src = None
        rgb = None
        arr = None
        closed = None
        try:
            t0 = time.ticks_ms()
            src = img.crop(roi)
            try:
                rgb = src.to_rgb888()
                arr = rgb.to_numpy_ref()
                if not self.conv_logged:
                    self.conv_logged = True
                    print("[CLOSE] RGB888转换: to_rgb888")
            except Exception:
                if (self.rgb_buf is None) or (self.rgb_shape != (rw, rh)):
                    self.rgb_buf = image.Image(rw, rh, image.RGB888)
                    self.rgb_shape = (rw, rh)
                src.copy_to(self.rgb_buf)
                arr = self.rgb_buf.to_numpy_ref()
                if not self.conv_logged:
                    self.conv_logged = True
                    print("[CLOSE] RGB888转换: copy_to 兜底")
            closed = _cv_lite.rgb888_close(
                [rh, rw], arr, CLOSE_ROI_KERNEL,
                CLOSE_ROI_ITERS, CLOSE_ROI_TH)
            self.close_ms = time.ticks_diff(time.ticks_ms(), t0)
            self.close_view = image.Image(
                rw, rh, image.GRAYSCALE,
                alloc=image.ALLOC_REF, data=closed)
            self.close_data = closed
            self.close_roi = roi
            t0 = time.ticks_ms()
            blobs = self.close_view.find_blobs(
                [(128, 255)], merge=False,
                area_threshold=PAPER_MIN_AREA,
                pixels_threshold=PAPER_MIN_AREA)
            self.quad_ms = time.ticks_diff(time.ticks_ms(), t0)
        except Exception as e:
            if not self.err_printed:
                self.err_printed = True
                print("近处闭运算异常(%s) -> 本帧回退原检测"
                      % einfo(e))
            return None, 0, ""
        if not blobs:
            return None, 0, "[闭运算:无白块] "
        big = []
        lim = int(rw * rh * 0.05)
        if lim < PAPER_MIN_AREA:
            lim = PAPER_MIN_AREA
        for b in blobs:
            if int(b[4]) >= lim:
                big.append(b)
        if len(big) > 1:
            return None, 0, "[闭运算:多块] "
        blobs = big
        if not blobs:
            return None, 0, "[闭运算:无白块] "
        best = None
        n = 0
        for b in blobs:
            bx = int(b[0])
            by = int(b[1])
            bw = int(b[2])
            bh = int(b[3])
            if (bw < 8) or (bh < 8):
                continue
            q = self._scan_quad(self.close_view, bx, by, bw, bh, x0, y0)
            if q is None:
                continue
            corners, ctr, long_side, short_side, qa = q
            cand = self._make_cand(
                img, x0 + bx, y0 + by, bw, bh, corners, ctr,
                long_side, short_side, qa)
            if cand is None:
                continue
            n += 1
            score = qa * (0.5 + min(cand[8] - cand[9], 80) / 80.0)
            if best is None or score > best[0]:
                best = (score, cand)
        if best is None:
            return None, n, "[闭运算:四边] "
        return best[1], n, "[闭运算+四边] "

    def detect(self, img):
        if FULL_FRAME_CLOSE:
            self.frame += 1
            cand, n, dbg = self._full_candidate(img)
            self.close_dbg = dbg
            self.last_n = n
            self.last_dbg = dbg
            self.did_full = True
            if cand is None:
                self.close_hit = False
                self.lost += 1
                self.state = "搜索"
                self.close_miss_n += 1
                if (self.close_miss_n % 20) == 1:
                    print("[CLOSE] 全帧未命中 n=%d %s"
                          % (n, dbg))
                return None
            self.close_miss_n = 0
            if not self._size_ok(cand[5], cand[7], cand[8] - cand[9]):
                self.last_dbg = "[全帧闭运算:尺寸闸门] "
                self.lost += 1
                self.state = "搜索"
                return None
            self.cands = [(cand[1], cand[2], cand[3], cand[4], 1)]
            if self._accept(cand):
                self.close_hit = True
                return {"center": (self.u, self.v),
                        "box": (cand[1], cand[2], cand[3], cand[4]),
                        "area": cand[0], "aspect": cand[6],
                        "density": cand[7], "long_side": cand[5],
                        "contrast": cand[8] - cand[9], "n": n,
                        "state": self.state,
                        "corners": self.corners}
            self.lost += 1
            self.state = "搜索"
            return None
        near = USE_CLOSE_ROI and self.have and (self.lost == 0) and \
            (self.last_long >= CLOSE_ROI_MIN_LONG)
        if not near:
            self.close_dbg = ""
            self.close_view = None
            self.close_data = None
            self.close_roi = None
            self.close_miss_n = 0
        if near:
            roi = self._close_roi()
            cand, n, dbg = self._near_candidate(img, roi)
            self.close_dbg = dbg
            if cand is not None:
                self.close_miss_n = 0
                self.frame += 1
                self.last_n = n
                self.last_dbg = dbg
                self.did_full = False
                if not self._size_ok(cand[5], cand[7], cand[8] - cand[9]):
                    self.last_dbg = "[近处闭运算:尺寸闸门] "
                    self.lost += 1
                    self.state = "搜索"
                    return None
                self.cands = [(cand[1], cand[2], cand[3], cand[4], 1)]
                if self._accept(cand):
                    self.close_hit = True
                    return {"center": (self.u, self.v),
                            "box": (cand[1], cand[2], cand[3], cand[4]),
                            "area": cand[0], "aspect": cand[6],
                            "density": cand[7], "long_side": cand[5],
                            "contrast": cand[8] - cand[9], "n": n,
                            "state": self.state,
                            "corners": self.corners}
                self.lost += 1
                self.state = "搜索"
                return None
            self.close_miss_n += 1
            if (self.close_miss_n % 20) == 1:
                print("[CLOSE] 未命中 n=%d roi=%s %s"
                      % (n, roi, dbg))
            self.lost += 1
            self.state = "搜索"
            return None
        self.close_hit = False
        return PaperDetector.detect(self, img)

# ============================================================================
#  视觉：激光光斑
# ============================================================================
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

    def _detect_blobs(self, img):
        x0, y0, w, h = self.roi()
        best = None
        for b in img.find_blobs(SPOT_THRESHOLDS, roi=(x0, y0, w, h),
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
            bx, by = norm_bbox(b[0], b[1], bw, bh, (x0, y0, w, h))
            cx = bx + bw / 2.0
            cy = by + bh / 2.0
            if best is None or area > best[0]:
                best = (area, cx, cy, (bx, by, bw, bh))
        if best is None:
            return None
        area, cx, cy, rect = best
        self._adapt(cx, cy)
        return {"uv": (cx, cy), "area": area, "rect": rect}


def apply_flip(sensor):
    """上下/左右翻转尽量多试几种写法：不同固件的 set_vflip 签名/是否分通道不同，
    现场实测 set_vflip(True) 不生效。如果最后还是倒的，就把摄像头模块整体转 180°
    装（装完记得把 CAM_VFLIP 改回 False，否则又会被翻回去）。"""
    for kw in ({}, {"chn": 0}, {"chn": 1}):
        try:
            sensor.set_vflip(CAM_VFLIP, **kw)
        except Exception:
            pass
        try:
            sensor.set_hmirror(CAM_HMIRROR, **kw)
        except Exception:
            pass


def log(msg):
    print(msg)


def einfo(e):
    """异常信息带类型 —— 这版固件上 Python 异常的字符串经常只有一个数字
    （现场被 `叠加层异常(0)` / `二值预览失败(0)` 坑过好几轮，其实就是
    KeyError(0)）。带上类型，下次一眼就能看出是什么错。"""
    return "%s: %s" % (type(e).__name__, e)


def main():
    global GRAY_DIRECT
    from media.sensor import Sensor, CAM_CHN_ID_0
    from media.display import Display
    from media.media import MediaManager
    import gc

    sensor = Sensor(id=2, width=1280, height=960, fps=90)
    sensor.reset()
    time.sleep_ms(100)
    apply_flip(sensor)
    sensor.set_framesize(width=IMG_W, height=IMG_H, chn=CAM_CHN_ID_0)
    if FULL_FRAME_CLOSE and GRAY_DIRECT:
        _fmt = Sensor.GRAYSCALE
    elif FULL_FRAME_CLOSE:
        _fmt = Sensor.RGB888
    else:
        _fmt = Sensor.RGB565
    try:
        sensor.set_pixformat(_fmt, chn=CAM_CHN_ID_0)
    except Exception as e:
        if not GRAY_DIRECT:
            raise
        print("灰度直出不可用(%s) -> 回退RGB888" % einfo(e))
        GRAY_DIRECT = False
        sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)
    apply_flip(sensor)
    if EXPO_FIX:
        try:
            sensor.set_auto_exposure(False, exposure_us=EXPO_US)
            print("曝光: 固定 exposure_us=%d（自动曝光已关）" % EXPO_US)
        except Exception as e:
            print("固定曝光不支持(%s)，继续自动曝光" % einfo(e))
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
    if not GRAY_DIRECT:
        try:
            px_probe_fast(sensor.snapshot(chn=CAM_CHN_ID_0))
        except Exception:
            pass

    import image
    _canvas = image.Image(DISPLAY_W, DISPLAY_H, image.RGB565)
    _sx = DISPLAY_W / float(IMG_W)
    _sy = DISPLAY_H / float(IMG_H)
    if (_sx > 1.0) or (_sy > 1.0):
        _sx = _sy = 1.0
    _ox = int((DISPLAY_W - IMG_W * _sx) * 0.5)
    _oy = int((DISPLAY_H - IMG_H * _sy) * 0.5)
    _preview_scaled = True
    _bin_mode = ""
    _bin_rgb = None
    _close_prev_bad = False
    _prev_err_printed = False
    _cv_shape = False
    _cv_text = False
    _ov_bad = []

    def _ov(name, fn, *a, **kw):
        """调一次绘制。某个 API 在这版固件上不支持时，只把它自己跳过 +
        打一行日志 —— 不能因为一个调用把整层叠加（框/靶心/激光）全带崩。
        2026-10-11 现场：`叠加层异常(0)` 就是这么来的（一个调用失败，全层没了）。"""
        if name in _ov_bad:
            return
        try:
            fn(*a, **kw)
        except Exception as e:
            _ov_bad.append(name)
            print("叠加层 '%s' 不支持(%s) -> 以后跳过它" % (name, e))

    def _overlay(target, det, spot_uv, s, ox, oy, mono, txt):
        """叠加层：候选框 / 四边形 / 靶心 / 激光 / 文字。

        target = 画布（此时 s=_sx, ox=_ox：原图坐标要缩放到画布）或
                 img  （此时 s=1, ox=0：原图坐标直接用）。
        mono=True 表示 target 是**二值图**（只有黑白），所有标记都用 0=黑 ——
        在二值图上画彩色是没意义的，而且黑色正好在白纸上最显眼。
        spot_uv 收两种写法：{"uv": (u, v)} 字典或 (u, v) 元组。
        （⚠ 2026-10-11 踩过：传进来是字典、这里按元组 index 取 —— 会抛
          KeyError(0)，日志里只显示 "(0)"，查了好几轮。两种都收，省得再犯。）
        """
        if spot_uv is not None and (type(spot_uv) is dict):
            spot_uv = spot_uv["uv"]
        if mono:
            c_ok = c_no = 0
            c_cross = c_spot = 0
        else:
            c_ok = (0, 255, 0)
            c_no = (255, 128, 0)
            c_cross = (255, 0, 0)
            c_spot = (255, 255, 0)
        for (bx, by, bw, bh, ok) in det.cands:
            _ov("rect", target.draw_rectangle,
                int(bx * s + ox), int(by * s + oy),
                int(bw * s), int(bh * s),
                color=c_ok if ok else c_no, thickness=1)
        dq = getattr(det, "debug_quad", None)
        if (dq is not None) and (not det.fresh()):
            for i in range(4):
                a = dq[i]
                b = dq[(i + 1) % 4]
                _ov("debugline", target.draw_line,
                    int(a[0] * s + ox), int(a[1] * s + oy),
                    int(b[0] * s + ox), int(b[1] * s + oy),
                    color=c_ok if not mono else 0, thickness=2)
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
        if txt and (not mono):
            _ov("text", target.draw_string_advanced,
                4, 4, 16, txt, color=(255, 255, 0))

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

    if BIN_PREVIEW:
        try:
            _pi = sensor.snapshot(chn=CAM_CHN_ID_0)
            _pth = scene_th(_pi, 0)
            _pi.binary([(_pth, 100, -PAPER_A_MAX, PAPER_A_MAX,
                         -PAPER_B_MAX, PAPER_B_MAX)])
            try:
                _canvas.draw_image(_pi, _ox, _oy,
                                   x_scale=_sx, y_scale=_sy)
                _bin_mode = "bin"
            except Exception as e:
                print("二值预览B(二值直画)不行: %s" % e)
            if not _bin_mode:
                try:
                    _bin_rgb = image.Image(IMG_W, IMG_H, image.RGB565)
                    _pi.copy_to(_bin_rgb)
                    _canvas.draw_image(_bin_rgb, _ox, _oy,
                                       x_scale=_sx, y_scale=_sy)
                    _bin_mode = "copy"
                except Exception as e:
                    print("二值预览C(预分配+copy_to)不行: %s" % e)
                    try:
                        del _bin_rgb
                    except Exception:
                        pass
            if not _bin_mode:
                try:
                    _prgb = _pi.to_rgb565()
                    _canvas.draw_image(_prgb, _ox, _oy,
                                       x_scale=_sx, y_scale=_sy)
                    _bin_mode = "rgb"
                    del _prgb
                except Exception as e:
                    print("二值预览A(转RGB565)不行: %s" % e)
            if _bin_mode:
                Display.show_image(_canvas)
            del _pi
        except Exception as e:
            print("二值化(binary)本身不行: %s" % e)
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
            print("二值化预览: 做法B（二值图直接缩放，零分配）")
        elif _bin_mode == "copy":
            print("二值化预览: 做法C（预分配 RGB565 + copy_to）")
        elif _bin_mode == "rgb":
            print("二值化预览: 做法A（每帧二值图转 RGB565）")
        else:
            print("二值化预览: 用不了 -> 本次跑彩色预览")
        print("预览叠加: 画布图形=%s 画布文字=%s"
              % ("可以" if _cv_shape else "不行(改画二值图上)",
                 "可以" if _cv_text else "不行"))

    target_det = ClosePaperDetector()
    spot_det = SpotDetector()

    last_spot = None
    cal_u = []
    cal_v = []
    t_print = time.ticks_ms()
    n_frame = 0
    n_frame_prev = 0
    n_tgt = 0
    n_spot = 0
    sum_grab = 0
    sum_tgt = 0
    sum_spot = 0
    sum_show = 0
    sum_n = 0

    print("=" * 68)
    print("vision_check: 纯视觉（不发串口）; 远处=main.py 原检测，"
          "近处=闭运算实验")
    print("  分辨率 %dx%d (S=%.2f, K=%.2fpx/°)  靶心=四边形对角线交点"
          % (IMG_W, IMG_H, S_SCALE, VIS_K))
    print("  阈值=Otsu(兜底:均值+8 夹[%d,72]) 长边%d~%dpx 密度(四边)%.2f "
          "对比>=%d 暗边>=%.0f%%"
          % (PAPER_TH, PAPER_MIN_LONG, PAPER_MAX_LONG,
             PAPER_DENSITY_QUAD_MIN, PAPER_CONTRAST_MIN,
             PAPER_DARK_FRAC_MIN * 100))
    print("  跟踪窗=长边x%.1f+%dpx 尺寸闸门[%.2f,%.2f]x 确认%d次 光斑每%d帧搜一次"
          % (TRACK_K, TRACK_PAD, SIZE_GATE_LO, SIZE_GATE_HI, CONFIRM_N,
             SPOT_EVERY))
    if FULL_FRAME_CLOSE:
        if GRAY_DIRECT:
            print("  全帧: GRAYSCALE直出 -> cv_lite灰度二值化130"
                  " -> image.close(1)[3x3] -> 白块 -> 原四边扫描")
        else:
            print("  全帧: RGB888 -> cv_lite二值化130(GRAYSCALE 0/255)"
                  " -> image.close(1)[3x3]"
                  " -> 白块 -> 原四边扫描"
                  "（沿用原来的黑框/对比度/尺寸闸门）")
    elif USE_CLOSE_ROI:
        print("  近处: cv_lite闭运算ROI -> 白块 -> 原四边扫描（不做碎片合并）"
              + ("；命中时预览=闭运算ROI"
                 if CLOSE_FULL_PREVIEW else ""))
    else:
        print("  近处: cv_lite 不可用 -> 全部走原检测")
    print("=" * 68)

    try:
        while True:
            os.exitpoint()
            now = time.ticks_ms()

            t0 = time.ticks_ms()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            t_grab = time.ticks_diff(time.ticks_ms(), t0)
            n_frame += 1

            t0 = time.ticks_ms()
            res = target_det.detect(img)
            t_tgt = time.ticks_diff(time.ticks_ms(), t0)
            if res is not None:
                n_tgt += 1

            t0 = time.ticks_ms()
            if CALIB_LASER:
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
                spot = {"uv": LASER_UV}
            if spot is not None:
                n_spot += 1
                SPOT_UV[0], SPOT_UV[1] = spot["uv"]

            if target_det.lost == 8 or (target_det.lost > 30 and
                                        target_det.lost % 90 == 0):
                log_scene(img)

            if (not _bin_mode) and target_det.fresh():
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

            t0 = time.ticks_ms()
            if (n_frame % SHOW_EVERY) == 0:
                _ok = False
                _direct = False
                if (CLOSE_FULL_PREVIEW and (not _close_prev_bad) and
                        (target_det.close_view is not None) and
                        (target_det.close_hit or CLOSE_PREVIEW_ANY)):
                    try:
                        _cr = target_det.close_roi
                        _cw, _ch = _cr[2], _cr[3]
                        _css = DISPLAY_W / float(_cw)
                        _css2 = DISPLAY_H / float(_ch)
                        if _css2 < _css:
                            _css = _css2
                        _cox = (DISPLAY_W - int(_cw * _css)) // 2
                        _coy = (DISPLAY_H - int(_ch * _css)) // 2
                        try:
                            _canvas.clear()
                        except Exception:
                            _canvas.draw_rectangle(
                                0, 0, DISPLAY_W, DISPLAY_H,
                                color=(0, 0, 0), fill=True)
                        _canvas.draw_image(
                            target_det.close_view, _cox, _coy,
                            x_scale=_css * target_det.close_scale,
                            y_scale=_css * target_det.close_scale)
                        _sp = None
                        if spot is not None:
                            _su, _sv = spot["uv"]
                            if (_cr[0] - 10 <= _su <= _cr[0] + _cw + 10) and \
                                    (_cr[1] - 10 <= _sv <= _cr[1] + _ch + 10):
                                _sp = (_su, _sv)
                        _overlay(
                            _canvas, target_det, _sp, _css,
                            _cox - _cr[0] * _css,
                            _coy - _cr[1] * _css,
                            False,
                            (("全帧闭运算" if target_det.close_hit else
                              "闭运算未命中") + " %dx%d" % (_cw, _ch))
                            if _cv_text else None)
                        _ok = True
                    except Exception as e:
                        try:
                            Display.show_image(target_det.close_view)
                            _ok = True
                            _direct = True
                        except Exception:
                            _close_prev_bad = True
                            print("闭运算ROI预览失败(%s) -> 回到全帧预览"
                                  % einfo(e))
                if (not _ok) and _bin_mode:
                    _th_bin = int(_scene_cache[1])
                    if _th_bin <= 0:
                        _th_bin = PAPER_TH
                    try:
                        img.binary([(_th_bin, 100,
                                     -PAPER_A_MAX, PAPER_A_MAX,
                                     -PAPER_B_MAX, PAPER_B_MAX)])
                    except Exception as e:
                        _bin_mode = ""
                        print("二值化 binary() 失败(%s) -> 改彩色预览" % einfo(e))
                if (not _ok) and _bin_mode:
                    try:
                        if _bin_mode == "bin":
                            _canvas.draw_image(img, _ox, _oy,
                                               x_scale=_sx, y_scale=_sy)
                        elif _bin_mode == "copy":
                            if _bin_rgb is None:
                                _bin_rgb = image.Image(
                                    IMG_W, IMG_H, image.RGB565)
                            img.copy_to(_bin_rgb)
                            _canvas.draw_image(_bin_rgb, _ox, _oy,
                                               x_scale=_sx, y_scale=_sy)
                        else:
                            _rgb = img.to_rgb565()
                            _canvas.draw_image(_rgb, _ox, _oy,
                                               x_scale=_sx, y_scale=_sy)
                            del _rgb
                        _s4 = _scene_cache[2]
                        _tag4 = "(固定)" if _s4 == 4.0 else \
                                ("(Otsu)" if _s4 == 1.0 else
                                 ("(自算Otsu)" if _s4 == 2.0 else "(回退)"))
                        _txt = ("th=%d%s %s 候选%d"
                                % (int(_scene_cache[1]), _tag4,
                                   target_det.state, target_det.last_n))
                        _sp = None
                        if spot is not None:
                            _sp = spot["uv"]
                        if not _cv_shape:
                            _overlay(img, target_det, _sp,
                                     1.0, 0, 0, True, None)
                        if _cv_shape:
                            _overlay(_canvas, target_det, _sp,
                                     _sx, _ox, _oy, False,
                                     _txt if _cv_text else None)
                        _ok = True
                    except Exception as e:
                        if _bin_mode == "bin":
                            _bin_mode = "copy"
                            print("二值直画失败(%s) -> 改用预分配copy_to"
                                  % einfo(e))
                        else:
                            _bin_mode = ""
                            print("二值图缩放/叠加失败(%s) -> 改彩色预览"
                                  % einfo(e))
                if _ok:
                    if not _direct:
                        try:
                            Display.show_image(_canvas)
                        except Exception as e:
                            print("推画面失败(%s)" % einfo(e))
                else:
                    if ALLOW_COLOR_FALLBACK:
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
                    elif not _prev_err_printed:
                        _prev_err_printed = True
                        print("预览绘制失败 -> 保持上一帧"
                              "（要彩色预览把 ALLOW_COLOR_FALLBACK=True）")
            t_show = time.ticks_diff(time.ticks_ms(), t0)
            del img

            sum_grab += t_grab
            sum_tgt += t_tgt
            sum_spot += t_spot
            sum_show += t_show
            sum_n += 1

            if time.ticks_diff(now, t_print) >= PRINT_MS:
                fps = ((n_frame - n_frame_prev) * 1000.0 /
                       max(1, time.ticks_diff(now, t_print)))
                n_frame_prev = n_frame
                t_print = now
                line = "FPS=%.1f " % fps
                if res is not None:
                    m = target_det.meas
                    _cs = ""
                    if target_det.close_hit:
                        _cs = (" [全帧闭运算 闭%.0fms 四边%.0fms]"
                               % (target_det.close_ms, target_det.quad_ms))
                    elif target_det.last_long >= CLOSE_ROI_MIN_LONG:
                        _cs = " [闭运算未命中]"
                    line += ("靶=锁定%s 距离≈%.2fm 靶心=(%.0f,%.0f) "
                             "长边=%.0fpx 比%.2f 密%.2f 对比=%.0f"
                             % (_cs,
                                FX_PX * PAPER_LONG_M / max(1.0, m[5]),
                                target_det.u, target_det.v,
                                m[5], m[6], m[7], m[8] - m[9]))
                else:
                    line += "靶=%s%s 候选%d th=%d %s" % (
                        target_det.state,
                        ("(全图)" if target_det.did_full else "(跟踪窗)"),
                        target_det.last_n, int(_scene_cache[1]),
                        target_det.last_dbg)
                    line += (" | 闭%.0f 扫%.0fms"
                             % (target_det.close_ms, target_det.quad_ms))
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
                sum_grab = 0
                sum_tgt = 0
                sum_spot = 0
                sum_show = 0
                sum_n = 0

            if (n_frame % GC_EVERY) == 0:
                gc.collect()
    except KeyboardInterrupt:
        print("用户停止")
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


if __name__ == "__main__":
    main()
