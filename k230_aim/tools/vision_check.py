# -*- coding: utf-8 -*-
"""vision_check.py —— K230 视觉验证 v13

在 CanMV IDE 里直接运行。画面显示在 IDE 帧缓冲，终端看数据。

画面标注：
    绿四边形 = 贴住黑胶带内沿的靶纸四边形（斜视时是梯形，不是正矩形）
    红十     = 靶心 = 四边形**两条对角线的交点**（透视不变的中心）
    黄圈     = 激光光斑

终端每 0.5 秒一行：
    FPS=32.0 靶=纸(锁定) 靶心=(430,300) 长边=222px 比1.41 密0.93 对比112
             距离≈0.60m 光斑=(319,216) 误差=(111,84)px=139.2px(17.9°)
             | ms 取图5 目标15 光斑3 显示3 | 靶12/60 斑12

—— 为什么用四边形而不是外接矩形（v13 的关键改动）——
1) 外接矩形（正四边形）在斜视时是"包住梯形的最小矩形"：边不贴靶纸，
   中心也不等于靶纸中心（透视下偏差随倾角增大），靶心自然就飘。
2) v13 用亮块当种子，向上下左右二分扫描出四条边，最小二乘拟合，
   求出四个角点 → 画出来是贴着黑胶带的梯形；靶心取**对角线交点**，
   这就是透视变换下的真正中心（也是同心圆圆心），与倾角无关。
3) 边长/面积也从四边形算：长边=两条长边均值（近大远小已含在内），
   密度=亮块像素/四边形面积（旋转、透视都不影响），距离更准。

—— 帧率说明（为什么不是 80fps）——
相机/ISP 确实能跑 90fps，瓶颈在 MicroPython 的经典 image 模块：
实测每次 find_blobs 约 10ms 固定开销 + 约 0.07μs/像素（日志反推）。
所以 v13 把开销压到“每帧只做一次 find_blobs”：
  · 靶纸：只在跟踪窗里搜一次（窗口=四边形长边x0.5+35px）；
  · 激光光斑：光轴固定不动，每 SPOT_EVERY 帧才搜一次，其它帧复用。
想再往上（40~60fps）就要换成资料里的 cv_lite（C 加速：
rgb888_find_rectangles_with_corners 直接给角点、rgb888_pnp_distance 直接给距离）。

依据（现场日志实测）：真靶纸 长边184~190px / 密度0.83~0.96 / 对比90+；
误检 密度0.51~0.73 / 对比22~67，还有一批长边36~38px 的小亮块。
调参顺序：PAPER_TH(58→52/64) → PAPER_CONTRAST_MIN(40) → PAPER_DARK_FRAC_MIN(0.70)。
"""
import os
import time
import math

# ============================ 画面 / 显示 ============================
IMG_W = 640
IMG_H = 480
DISPLAY_QUALITY = 40       # 越小传输越快（画面撕裂跟传输速度直接相关）
SHOW_EVERY = 2             # 每几帧往 IDE 推一次画面（识别仍每帧都算）

# ============================ 靶纸检测 ============================
PAPER_TH = 58              # 亮度阈值（0~100）
PAPER_TH_ALT = 72          # 兜底阈值：全图第一遍失败时再试一次
PAPER_A_MAX = 32           # |a| 上限（偏色背景会被排除）
PAPER_B_MAX = 32           # |b| 上限
PAPER_MIN_AREA = 900       # 最小亮块像素（2.2m 处 A4 约 60x42px ≈ 2000px）
PAPER_MAX_AREA_RATIO = 0.85
PAPER_MIN_LONG = 60        # 四边形长边下限 px
PAPER_MAX_LONG = 480       # 四边形长边上限 px
PAPER_ASPECT_MIN = 0.70    # 长边/短边（A4=1.41；斜视透视下会缩到接近 1，甚至 <1）
PAPER_ASPECT_MAX = 3.00
PAPER_DENSITY_MIN = 0.70   # 兜底（没拟合出四边形时）：像素/外接框面积
PAPER_DENSITY_QUAD_MIN = 0.72   # 拟合出四边形后：像素/四边形面积
PAPER_CONTRAST_MIN = 40    # 内亮度 - 外亮度（0~255 量程）
PAPER_DARK_MARGIN = 25     # 单个外侧采样点算“暗”的门槛
PAPER_DARK_FRAC_MIN = 0.70  # 外侧 12 个点里至少这么多比例比内部暗

# 四边形拟合
QUAD_SCAN_N = 7            # 每边取几行/几列做扫描（越多越稳，越慢）
QUAD_PERP_PX = 2           # 扫描时上下（左右）各看几像素，跨过 1~2px 印刷细线
QUAD_AREA_LO = 0.50        # 四边形面积 / 亮块像素 的合理范围
QUAD_AREA_HI = 1.35
# 扫描用的**相对**亮度门槛：纸面有阴影时，固定阈值会把暗的那半边切掉，
# 四边形就被拉进纸里（现场截图里"不贴胶带"就是这么来的）。
# 改成按纸面自身亮度 ins 的比例定：th_scan = ins*SCAN_TH_K，再夹在上下限之间。
# 胶带(≈20~55)低于它、纸面阴影侧(≈90~130)高于它 → 边才落在胶带内沿。
SCAN_TH_K = 0.50
SCAN_TH_LO = 50
SCAN_TH_HI = 88
QUAD_LIM_PAD = 18          # 扫描半径 = 中心到亮块该边的距离 + 这个余量

# 跟踪 / 闸门
# 跟踪窗必须**大于整张纸**（四边形扫描要摸到纸的四条边，窗口小了亮块会被裁掉）
TRACK_K = 0.50             # 跟踪窗 = 四边形长边 x TRACK_K + TRACK_PAD
TRACK_PAD = 30
SEED_EVERY = 2             # 锁定后每几帧做一次亮块搜索（其它帧只做四边形复测）
HOLD_FRAMES = 15           # 丢靶后还画多少帧
LOST_FULL = 18             # 丢这么多帧后放弃小窗，改全图搜索
FULL_EVERY = 3             # 每几帧做一次全图搜索
SMOOTH = 0.55              # 平滑系数
DEADBAND_PX = 1.5          # 平滑死区（小于它不动，画面不抖）
SIZE_GATE_LO = 0.55        # 跟踪时允许的长边变化范围
SIZE_GATE_HI = 1.45
CONFIRM_N = 3              # 全图候选连续确认几次才算重新锁定
CONFIRM_DXY = 35           # 确认时的位置一致范围 px
CONFIRM_DSIZE = 0.60       # 确认时的尺寸一致范围
PENDING_MISS = 3           # 确认过程中允许漏几次

# ============================ 激光光斑 ============================
SPOT_THRESHOLDS = [
    # 过曝白芯：纸面 LAB-L≈78 到不了 88，只有激光点会这么亮
    (88, 100, -40, 90, -50, 90),
    # 红边：a>=25 才算明显偏红（原来 a>=6 太松，纸面暖色区也进来了）
    (58, 100, 25, 90, -20, 70),
]
SPOT_MIN_AREA = 2
SPOT_MAX_AREA = 2500
SPOT_MAX_ASPECT = 3.0
SPOT_ROI_HALF = 45         # 只在学习到的光轴点附近找
SPOT_LEARN_FRAMES = 10
SPOT_NEAR_PX = 12          # 学习时位置一致性要求（松了会锁到误检上）
SPOT_EVERY = 4             # 每几帧搜一次光斑（光轴固定，中间帧复用）

# ============================ 距离 ============================
FX_PX = 445.0              # 像素焦距（640 宽；0.6m 处 A4 长边≈222px 反推）
PAPER_LONG_M = 0.297       # A4 长边（米）

# ============================ 串口 / 协议 ============================
UART_BAUD = 115200
HEARTBEAT_MS = 200
AIM_MS = 20
PRINT_MS = 500
GC_EVERY = 30

SOF = b"\xAA\x55"
MSG_AIM = 0x10
MSG_MODE = 0x11
MSG_HEARTBEAT = 0x12
MSG_SET_ZERO = 0x13
MSG_GIMBAL_STATE = 0x90
MSG_TEXT = 0x93
ST_READY = 0x10


def _crc_table():
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if (c & 1) else (c >> 1)
        t.append(c)
    return t


_CRC = _crc_table()


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ _CRC[(crc ^ b) & 0xFF]
    return crc & 0xFFFF


def build_frame(msg_id, payload=b"", seq=0):
    import ustruct
    body = ustruct.pack("<BBB", msg_id & 0xFF, seq & 0xFF,
                        len(payload)) + payload
    return SOF + body + ustruct.pack("<H", crc16(body))


def pack_aim(yaw_deg, pitch_deg, flags, quality):
    import ustruct
    return ustruct.pack("<hhBB",
                        max(-32768, min(32767, int(round(yaw_deg * 100.0)))),
                        max(-32768, min(32767, int(round(pitch_deg * 100.0)))),
                        flags & 0xFF, max(0, min(255, int(quality))))


class FrameParser(object):
    def __init__(self):
        self.buf = bytearray()
        self.ok = 0
        self.crc_err = 0

    def feed(self, data):
        out = []
        self.buf.extend(data)
        while True:
            i = self.buf.find(SOF)
            if i < 0:
                if len(self.buf) > 1:
                    self.buf = self.buf[-1:]
                break
            if i > 0:
                self.buf = self.buf[i:]
            if len(self.buf) < 5:
                break
            length = self.buf[4]
            if length > 240:
                self.buf = self.buf[2:]
                continue
            total = 5 + length + 2
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:5 + length])
            crc_rx = self.buf[5 + length] | (self.buf[6 + length] << 8)
            if crc16(body) != crc_rx:
                self.crc_err += 1
                self.buf = self.buf[2:]
                continue
            self.ok += 1
            out.append((body[0], body[1], bytes(body[3:])))
            self.buf = self.buf[total:]
        return out


# ============================ 视觉：靶纸四边形 ============================
def luma(r, g, b):
    return (r * 77 + g * 150 + b * 29) >> 8


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
        p = img.get_pixel(int(x), int(y))
    except Exception:
        return -1
    if p is None:
        return -1
    return luma(p[0], p[1], p[2])


def _probe(img, x, y, dx, dy, th):
    """沿射线前后各 QUAD_PERP_PX 取样，多数（>=3/5）亮才算“纸”。

    关键：印刷圆圈的细线在“圆的正左/正右”位置几乎与横向射线垂直，
    只做垂直方向取最大是跨不过去的（扫描会停在第一圈圆环上，四边形
    就缩到圆环范围）。沿射线方向取多数则与细线角度无关，一律能跨过。
    """
    if dx:
        offs = ((x - 2, y), (x - 1, y), (x, y), (x + 1, y), (x + 2, y))
    else:
        offs = ((x, y - 2), (x, y - 1), (x, y), (x, y + 1), (x, y + 2))
    k = 0
    for p in offs:
        v = px_luma(img, p[0], p[1])
        if v >= th:
            k += 1
    return k >= 3


def _edge(img, xc, yc, dx, dy, limit, th):
    """从 (xc,yc) 沿 (dx,dy) 二分找最后一个“纸”像素的距离；找不到返回 -1。

    纸是凸的：沿任一条从内部出发的射线，先纸后不纸（细线由 _probe 的
    多数投票跨过去），所以可以二分。
    """
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
    """最小二乘拟合 v = a*u + b。pts=[(u,v), ...]，点数<2 返回 None。"""
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
    b = (sv - a * su) / float(n)
    return a, b


def _lsq_robust(pts):
    """Theil-Sen：取所有点对斜率的中位数，再取截距中位数。

    斜视梯形的扫描里，靠近上/下边的行可能打到“上边/下边”而不是“左边/右边”，
    这些点能把普通最小二乘带偏几十像素；中位数法最多容忍一半的坏点。
    """
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
    if m % 2:
        a = sl[m // 2]
    else:
        a = 0.5 * (sl[m // 2 - 1] + sl[m // 2])
    bs = []
    for u, v in pts:
        bs.append(v - a * u)
    bs.sort()
    nb = len(bs)
    if nb % 2:
        b = bs[nb // 2]
    else:
        b = 0.5 * (bs[nb // 2 - 1] + bs[nb // 2])
    return a, b


def _cross(e1, e2):
    """e1 是竖边 x = a*y + b，e2 是横边 y = a*x + b，返回交点或 None。"""
    al, bl = e1
    at, bt = e2
    den = 1.0 - al * at
    if abs(den) < 1e-3:
        return None
    x = (al * bt + bl) / den
    y = at * x + bt
    return (x, y)


def quad_from_blob(img, bx, by, bw, bh, th):
    """用行/列二分扫描把亮块拟合成四边形。

    返回 (corners, center, long_side, short_side, quad_area) 或 None。
    corners=(TL,TR,BR,BL)；center=两条对角线交点（透视意义下的中心）。
    """
    icx = int(bx + bw / 2.0)
    icy = int(by + bh / 2.0)
    # 每条边的扫描半径 = 中心到亮块该边的距离 + QUAD_LIM_PAD。
    # 不能放太远：越过黑胶带后会摸到背景亮边，那一条边就会“跳”出去
    # （现场截图里上边飞到柜顶就是这个原因）。
    lim_l = icx - int(bx) + QUAD_LIM_PAD
    lim_r = int(bx + bw) - icx + QUAD_LIM_PAD
    lim_u = icy - int(by) + QUAD_LIM_PAD
    lim_d = int(by + bh) - icy + QUAD_LIM_PAD
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
    for p in (tl, tr, br, bl):
        if (p[0] < 1) or (p[0] > IMG_W - 2) or \
                (p[1] < 1) or (p[1] > IMG_H - 2):
            return None
    c = (tl, tr, br, bl)
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
    return (c, ctr, long_side, max(1.0, short_side), qa)


def paper_contrast(img, x, y, w, h):
    """内亮外暗校验：返回 (内部平均亮度, 外侧平均亮度, 外侧合格比例)。"""
    u = ((x + w * 0.30, y + h * 0.50), (x + w * 0.70, y + h * 0.50),
         (x + w * 0.50, y + h * 0.30), (x + w * 0.50, y + h * 0.70),
         (x + w * 0.50, y + h * 0.50))
    o = ((x - 5, y + h * 0.25), (x - 5, y + h * 0.50), (x - 5, y + h * 0.75),
         (x + w + 5, y + h * 0.25), (x + w + 5, y + h * 0.50),
         (x + w + 5, y + h * 0.75),
         (x + w * 0.25, y - 5), (x + w * 0.50, y - 5), (x + w * 0.75, y - 5),
         (x + w * 0.25, y + h + 5), (x + w * 0.50, y + h + 5),
         (x + w * 0.75, y + h + 5))
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


def norm_bbox(bx, by, bw, bh, roi):
    """兼容 find_blobs 返回“相对 ROI”或“全图”两种坐标。"""
    if roi[0] or roi[1]:
        if (bx + bw / 2.0) < roi[0] or (by + bh / 2.0) < roi[1]:
            return bx + roi[0], by + roi[1]
    return bx, by


def _scan_th(ref):
    """按纸面自身亮度定的扫描门槛（纸面有阴影时也能摸到真边）。"""
    t = ref * SCAN_TH_K
    if t < SCAN_TH_LO:
        t = SCAN_TH_LO
    elif t > SCAN_TH_HI:
        t = SCAN_TH_HI
    return t


def _requad(img, seed):
    """快速复测：拿上次的四边形当种子，只重扫四条边（省一次 find_blobs 的固定开销）。

    返回 (best, 亮块数=0, 诊断串)，格式与 find_paper 一致。
    """
    px = seed[0]
    c = seed[8]
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
    aspect = long_side / max(1.0, short_side)
    if (long_side < PAPER_MIN_LONG) or (long_side > PAPER_MAX_LONG) or \
            (aspect < PAPER_ASPECT_MIN) or (aspect > PAPER_ASPECT_MAX):
        return None, 0, "[复测:尺寸] "
    return (px, ctr[0], ctr[1], long_side, aspect, px / max(1.0, qa),
            ins, outs, corners), 0, ""


def find_paper(img, roi, th, dbg_on=False, seed=None):
    """在 roi 里找 A4 靶纸四边形。

    返回 (best, 亮块数, 诊断串)；
    best = (px, cu, cv, long_side, aspect, density, ins, outs, corners)
    corners=None 表示四边形没拟合出来（退回外接框）。
    seed=上次的测量元组时走快速复测（只重扫四边形，不找亮块）。
    """
    if seed is not None:
        return _requad(img, seed)
    best = None
    n = 0
    dbg = ""
    blobs = img.find_blobs(
        [(th, 100, -PAPER_A_MAX, PAPER_A_MAX, -PAPER_B_MAX, PAPER_B_MAX)],
        roi=roi, merge=True, margin=6,
        area_threshold=PAPER_MIN_AREA, pixels_threshold=PAPER_MIN_AREA)
    if not blobs:
        return None, 0, ""
    for b in blobs:
        n += 1
        x, y, w, h, px = b[0], b[1], b[2], b[3], b[4]
        x, y = norm_bbox(x, y, w, h, roi)
        if w < 8 or h < 8:
            continue
        box_long = w if w > h else h
        ins, outs, frac = paper_contrast(img, x, y, w, h)
        if dbg_on and n <= 3:
            dbg += "[%dpx 框%.0f 内%d 外%d 暗边%.0f%%] " % (
                px, box_long, ins, outs, frac * 100.0)
        if px < PAPER_MIN_AREA:
            continue
        if px > PAPER_MAX_AREA_RATIO * IMG_W * IMG_H:
            continue
        if box_long < PAPER_MIN_LONG * 0.8 or box_long > PAPER_MAX_LONG * 1.3:
            continue
        if (x <= 1) or (y <= 1) or \
                ((x + w) >= IMG_W - 1) or ((y + h) >= IMG_H - 1):
            continue
        if ins < 0 or (ins - outs) < PAPER_CONTRAST_MIN:
            continue
        if frac < PAPER_DARK_FRAC_MIN:
            continue
        q = quad_from_blob(img, x, y, w, h, _scan_th(ins))
        corners = None
        qa = 0.0
        if q is not None:
            corners, ctr, long_side, short_side, qa = q
            if (qa < QUAD_AREA_LO * px) or (qa > QUAD_AREA_HI * px):
                corners = None             # 拟合结果和亮块对不上，弃用
        if corners is None:
            long_side = box_long
            short_side = w if w < h else h
            density = px / float(w * h)
            dens_min = PAPER_DENSITY_MIN
            ctr = (x + w / 2.0, y + h / 2.0)
        else:
            density = px / max(1.0, qa)
            dens_min = PAPER_DENSITY_QUAD_MIN
        aspect = long_side / max(1.0, short_side)
        if long_side < PAPER_MIN_LONG or long_side > PAPER_MAX_LONG:
            continue
        if aspect < PAPER_ASPECT_MIN or aspect > PAPER_ASPECT_MAX:
            continue
        if density < dens_min:
            continue
        if dbg_on and n <= 3:
            dbg += "{四边%.0fx%.0f 密%.2f} " % (long_side, short_side, density)
        score = px * (0.5 + min(ins - outs, 80) / 80.0)
        if best is None or score > best[0]:
            best = (score, (px, ctr[0], ctr[1], long_side, aspect, density,
                            ins, outs, corners))
    if best is None:
        return None, n, dbg
    return best[1], n, dbg


class PaperTracker(object):
    """候选 -> 静态闸门(在 find_paper 里) -> 尺寸闸门 -> 连续确认 -> 平滑。

      · 跟踪中：只在上一帧四边形附近搜，长边必须是上次的 0.55~1.45 倍；
      · 丢靶后：全图搜索，候选要连续 CONFIRM_N 次位置(±35px)、尺寸(±60%)
        都对得上才重新锁定（一次性的背景亮块过不了这一关）。
    """

    def __init__(self):
        self.u = 0.0
        self.v = 0.0
        self.w = 0.0
        self.h = 0.0
        self.have = False
        self.lost = 0
        self.meas = None
        self.corners = None
        self.pend = None       # [cu, cv, long, cnt, miss]
        self.frame = 0
        self.last_n = 0
        self.last_dbg = ""
        self.did_full = False
        self.state = "搜索"

    def roi(self):
        r = int(max(self.w, self.h) * TRACK_K) + TRACK_PAD
        x = int(self.u - r)
        y = int(self.v - r)
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

    def _size_ok(self, long_side):
        if not self.have:
            return True
        old = max(self.w, self.h)
        return (SIZE_GATE_LO * old) <= long_side <= (SIZE_GATE_HI * old)

    def _confirm(self, cand):
        """全图候选的连续确认。返回 True = 确认为目标。"""
        if cand is None:
            if self.pend is not None:
                self.pend[4] += 1
                if self.pend[4] > PENDING_MISS:
                    self.pend = None
            return False
        cu, cv, lo = cand[1], cand[2], cand[3]
        # 重新锁定也要过尺寸合理性：纸不可能在半秒里变成 1/4 大。
        # （现场日志里锁到背景 71px 亮块就是这个口子漏的）丢靶超过 3 秒才放开。
        if self.have and (self.lost < 100):
            old = max(self.w, self.h)
            if (lo < 0.40 * old) or (lo > 2.5 * old):
                self.pend = None
                return False
        if self.pend is None:
            self.pend = [cu, cv, lo, 1, 0]
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
        cu, cv = cand[1], cand[2]
        w = max(1.0, cand[3])
        h = max(1.0, w / max(1.0, cand[4]))
        if not self.have:
            self.u, self.v = cu, cv
            self.have = True
        else:
            if abs(cu - self.u) > DEADBAND_PX:
                self.u += SMOOTH * (cu - self.u)
            if abs(cv - self.v) > DEADBAND_PX:
                self.v += SMOOTH * (cv - self.v)
        self.w = w
        self.h = h
        self.meas = cand
        self.corners = cand[8]
        self.lost = 0
        self.state = "锁定"

    def box(self):
        return (int(self.u - self.w / 2.0), int(self.v - self.h / 2.0),
                int(self.w), int(self.h))

    def step(self, img, finder):
        """跑一帧跟踪。finder = find_paper。返回本帧确认的候选或 None。"""
        self.frame += 1
        cand = None
        n = 0
        dbg = ""
        self.did_full = False
        if self.have and self.lost < LOST_FULL:
            if ((self.frame % SEED_EVERY) == 0) or (self.meas is None):
                cand, n, dbg = finder(img, self.roi(), PAPER_TH)
            else:
                # 隔帧只做四边形复测（省掉一次 find_blobs 的 ~13ms 固定开销），
                # 靶心仍然每帧都是新的；复测失败就立刻补一次完整搜索。
                cand, n, dbg = finder(img, self.roi(), PAPER_TH, False,
                                      self.meas)
                if cand is None:
                    cand, n, dbg = finder(img, self.roi(), PAPER_TH)
            if (cand is not None) and (not self._size_ok(cand[3])):
                dbg = "[尺寸闸门%.0fpx] " % cand[3] + dbg
                cand = None
            if (cand is None) and ((self.frame % FULL_EVERY) == 0):
                self.did_full = True
                c2, n2, d2 = finder(img, (0, 0, IMG_W, IMG_H), PAPER_TH)
                n += n2
                dbg += d2
                if self._confirm(c2):
                    cand = c2
                elif c2 is None:
                    c3, n3, d3 = finder(img, (0, 0, IMG_W, IMG_H),
                                         PAPER_TH_ALT)
                    n += n3
                    dbg += d3
                    if self._confirm(c3):
                        cand = c3
        else:
            if (self.frame % FULL_EVERY) == 0:
                self.did_full = True
                c2, n2, d2 = finder(img, (0, 0, IMG_W, IMG_H), PAPER_TH, True)
                if c2 is None:
                    c3, n3, d3 = finder(img, (0, 0, IMG_W, IMG_H),
                                         PAPER_TH_ALT, True)
                    c2, n2, d2 = c3, n2 + n3, d2 + d3
                n += n2
                dbg = d2
                if self._confirm(c2):
                    cand = c2
        if cand is not None:
            self._accept(cand)
        else:
            self.lost += 1
            if self.pend is not None:
                self.state = "确认%d/%d" % (self.pend[3], CONFIRM_N)
            else:
                self.state = "搜索"
        self.last_n = n
        self.last_dbg = dbg
        return cand


class SpotDetector(object):
    """找激光光斑：暖色/过曝亮块，学一个光轴点后锁定。"""

    def __init__(self):
        self.u = IMG_W / 2.0
        self.v = IMG_H / 2.0
        self.locked = False
        self.learn_u = self.u
        self.learn_v = self.v
        self.learn_n = 0
        self.meas = None

    def detect(self, img):
        x0 = int(self.u - SPOT_ROI_HALF)
        y0 = int(self.v - SPOT_ROI_HALF)
        if x0 < 0:
            x0 = 0
        if y0 < 0:
            y0 = 0
        w = min(IMG_W - x0, SPOT_ROI_HALF * 2)
        h = min(IMG_H - y0, SPOT_ROI_HALF * 2)
        roi = (x0, y0, w, h)
        best = None
        for b in img.find_blobs(SPOT_THRESHOLDS, roi=roi, merge=True,
                                pixels_threshold=SPOT_MIN_AREA,
                                area_threshold=SPOT_MIN_AREA):
            area = b[4]
            bw, bh = b[2], b[3]
            if area < SPOT_MIN_AREA or area > SPOT_MAX_AREA:
                continue
            if bw < 1 or bh < 1:
                continue
            if max(bw, bh) * 1.0 / min(bw, bh) > SPOT_MAX_ASPECT:
                continue
            bx, by = norm_bbox(b[0], b[1], bw, bh, roi)
            cu = bx + bw / 2.0
            cv = by + bh / 2.0
            d = abs(cu - self.u) + abs(cv - self.v)
            if best is None or d < best[0]:
                best = (d, area, cu, cv)
        if best is None:
            self.meas = None
            self.learn_n = 0
            return None
        self.meas = (best[1], best[2], best[3])
        if not self.locked:
            if self.learn_n == 0:
                self.learn_u, self.learn_v = best[2], best[3]
                self.learn_n = 1
            elif (abs(best[2] - self.learn_u) <= SPOT_NEAR_PX) and \
                    (abs(best[3] - self.learn_v) <= SPOT_NEAR_PX):
                self.learn_n += 1
                self.learn_u += (best[2] - self.learn_u) / float(self.learn_n)
                self.learn_v += (best[3] - self.learn_v) / float(self.learn_n)
            else:
                self.learn_u, self.learn_v = best[2], best[3]
                self.learn_n = 1
            if self.learn_n >= SPOT_LEARN_FRAMES:
                self.u, self.v = self.learn_u, self.learn_v
                self.locked = True
                print("光斑位置已锁定: (%.1f, %.1f)" % (self.u, self.v))
        return self.meas


def print_l_diag(img, roi):
    """丢靶时的亮度诊断，用来判断阈值该往哪调。"""
    err = None
    try:
        st = img.get_statistics(roi=roi)
        print("  [诊断] L均=%.0f 中=%.0f 大=%.0f"
              % (st.l_mean(), st.l_median(), st.l_max()))
        return
    except Exception as e:
        err = e
    try:
        h = img.get_histogram(roi=roi)
        print("  [诊断] Otsu=%d" % h.get_threshold().l_value())
    except Exception:
        print("  [诊断] 统计不可用: %s" % err)


def main():
    from media.sensor import Sensor, CAM_CHN_ID_0
    from media.display import Display
    from media.media import MediaManager
    import gc

    from ybUtils.YbUart import YbUart
    uart = YbUart(baudrate=UART_BAUD)
    parser = FrameParser()
    print("串口: YbUart @%d" % UART_BAUD)

    sensor = Sensor(id=2, width=1280, height=960, fps=90)
    sensor.reset()
    time.sleep_ms(100)
    sensor.set_framesize(width=IMG_W, height=IMG_H, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
    Display.init(Display.ST7701, width=IMG_W, height=IMG_H,
                 to_ide=True, quality=DISPLAY_QUALITY)
    MediaManager.init()
    sensor.run()
    clock = time.clock()

    tracker = PaperTracker()
    spot_det = SpotDetector()

    ready = False
    armed = False
    last_text = ""
    t_hb = time.ticks_ms()
    t_aim = time.ticks_ms()
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
    print("vision_check v13: 靶纸=贴黑胶带的四边形, 靶心=对角线交点")
    print("  阈值%d 长边%.0f~%.0fpx 密度(四边)%.2f 对比%d 暗边>=%.0f%%"
          % (PAPER_TH, PAPER_MIN_LONG, PAPER_MAX_LONG,
             PAPER_DENSITY_QUAD_MIN, PAPER_CONTRAST_MIN,
             PAPER_DARK_FRAC_MIN * 100))
    print("  跟踪窗=长边x%.1f+%dpx 尺寸闸门[%.2f,%.2f]x 确认%d次 光斑每%d帧搜一次"
          % (TRACK_K, TRACK_PAD, SIZE_GATE_LO, SIZE_GATE_HI, CONFIRM_N,
             SPOT_EVERY))
    print("=" * 68)

    try:
        while True:
            os.exitpoint()
            now = time.ticks_ms()

            # ---------- 串口 ----------
            try:
                if uart.any() > 0:
                    for msg_id, seq, payload in parser.feed(uart.read(256)):
                        if msg_id == MSG_GIMBAL_STATE and len(payload) == 21:
                            import ustruct
                            v = ustruct.unpack("<BBhhhhhhhBI", payload)
                            ready = bool(v[9] & ST_READY)
                        elif msg_id == MSG_TEXT:
                            try:
                                txt = payload.decode("utf-8")
                            except Exception:
                                txt = ""
                            if txt != last_text:
                                last_text = txt
                                print("  H723: %s" % txt)
            except Exception:
                pass

            if time.ticks_diff(now, t_hb) >= HEARTBEAT_MS:
                t_hb = now
                try:
                    uart.write(build_frame(MSG_HEARTBEAT))
                except Exception:
                    pass

            if ready and not armed:
                try:
                    uart.write(build_frame(MSG_SET_ZERO))
                    time.sleep_ms(20)
                    uart.write(build_frame(MSG_MODE, bytes([2, 0])))
                    armed = True
                    print("H723 READY -> SET_ZERO + AIM（激光打开，偏置 0）")
                except Exception as e:
                    print("发 MODE 失败: %s" % e)

            if armed and time.ticks_diff(now, t_aim) >= AIM_MS:
                t_aim = now
                try:
                    uart.write(build_frame(MSG_AIM,
                                           pack_aim(0.0, 0.0, 0x03, 200)))
                except Exception:
                    pass

            # ---------- 取图 ----------
            t0 = time.ticks_ms()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            t_grab = time.ticks_diff(time.ticks_ms(), t0)
            n_frame += 1

            # ---------- 靶纸四边形 ----------
            t0 = time.ticks_ms()
            cand = tracker.step(img, find_paper)
            t_tgt = time.ticks_diff(time.ticks_ms(), t0)
            if cand is not None:
                n_tgt += 1

            # ---------- 光斑（每 SPOT_EVERY 帧搜一次）----------
            t0 = time.ticks_ms()
            if (n_frame % SPOT_EVERY) == 0:
                spot = spot_det.detect(img)
            else:
                spot = spot_det.meas
            t_spot = time.ticks_diff(time.ticks_ms(), t0)
            if spot is not None:
                n_spot += 1

            if tracker.lost == 8 or (tracker.lost > 30 and
                                     tracker.lost % 90 == 0):
                print_l_diag(img, (0, 0, IMG_W, IMG_H))

            # ---------- 画 ----------
            if tracker.fresh():
                c = tracker.corners
                if c is not None:
                    for i in range(4):
                        a = c[i]
                        b = c[(i + 1) % 4]
                        img.draw_line(int(a[0]), int(a[1]),
                                      int(b[0]), int(b[1]),
                                      color=(0, 255, 0), thickness=2)
                else:
                    bx, by, bw, bh = tracker.box()
                    img.draw_rectangle(bx, by, bw, bh, color=(0, 255, 0),
                                       thickness=2)
                img.draw_cross(int(tracker.u), int(tracker.v),
                               color=(255, 0, 0), size=14, thickness=2)
            if spot is not None:
                img.draw_circle(int(spot[1]), int(spot[2]), 8,
                                color=(255, 255, 0), thickness=2)
                img.draw_cross(int(spot[1]), int(spot[2]),
                               color=(255, 255, 0), size=6, thickness=1)

            # 隔帧推画面：to_ide 的编码+USB 传输比处理慢，追太紧会在传输途中
            # 改写同一块缓冲，IDE 里就显示成"几块拼接+颜色错乱"的撕裂画面
            # （数据本身没问题，终端里的靶心/误差一直是对的）。
            t0 = time.ticks_ms()
            if (n_frame % SHOW_EVERY) == 0:
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
                if tracker.meas is not None and tracker.lost == 0:
                    px, cu, cv, long_side, aspect, density, ins, outs, c = \
                        tracker.meas
                    dist_m = FX_PX * PAPER_LONG_M / max(1.0, long_side)
                    line += ("靶=纸(%s%s) 靶心=(%.0f,%.0f) 长边=%.0fpx 比%.2f "
                             "密%.2f 对比=%.0f 距离≈%.2fm"
                             % (tracker.state,
                                "" if c is not None else "·框",
                                tracker.u, tracker.v, long_side, aspect,
                                density, ins - outs, dist_m))
                else:
                    line += "靶=%s%s 亮块=%d %s" % (
                        tracker.state,
                        ("(全图)" if tracker.did_full else "(跟踪窗)"),
                        tracker.last_n, tracker.last_dbg)
                if spot is not None:
                    if tracker.meas is not None and tracker.lost == 0:
                        eu = tracker.u - spot[1]
                        ev = tracker.v - spot[2]
                        err = math.sqrt(eu * eu + ev * ev)
                        line += (" | 光斑=(%.0f,%.0f) 误差=(%.0f,%.0f)px "
                                 "=%.1fpx(%.2f°)"
                                 % (spot[1], spot[2], eu, ev, err,
                                    err / FX_PX * 57.29578))
                    else:
                        line += " | 光斑=(%.0f,%.0f)" % (spot[1], spot[2])
                else:
                    line += " | 光斑: 未检出"
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
            uart.write(build_frame(MSG_MODE, bytes([1, 0])))
            time.sleep_ms(50)
        except Exception:
            pass
        try:
            uart.deinit()
        except Exception:
            pass
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
        print("已退出（激光已关闭）")


main()
