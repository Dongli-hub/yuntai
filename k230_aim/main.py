# -*- coding: utf-8 -*-
"""main.py —— K230 机载计算机瞄准程序（替代地瓜派上的 rdk_aim）

整体上电即自动运行：把本文件保存到 K230 SD 卡根目录、命名 main.py 即可。

它做的事（和地瓜派版一一对应）：
   相机取图 -> 找 A4 靶纸(白纸亮块 + 四周暗框校验) -> 找激光光斑
        -> 像素误差 -> 角度偏置 -> 串口发给 H723 -> 云台转过去

和 H723 的约定（协议不变）：
   帧: AA 55 | msg_id | seq | len | payload | crc16(小端)
   0x12 心跳 / 0x13 SET_ZERO / 0x11 MODE(0IDLE 1STAB 2AIM) / 0x10 AIM(偏置)
   0x90 遥测 / 0x91 ACK / 0x93 调试文本
   H723 端: 0.5s 收不到 AIM 自动切 STAB 并关激光（安全看门狗）

接线（实测确认，别再改回去）：
   K230 IO9 (TXD)  <-> H723 USART1 的 RX (PA10)
   K230 IO10(RXD)  <-> H723 USART1 的 TX (PA9)
   GND             <-> GND   （必须共地，否则一个字节都收不到）
   5V              <-  H723 UART10 排针的 VCC（USART1 三针端子上没有电源脚）
   H723 固件 gimbal_link.c: GL_LINK_UART_SEL = 0（走 USART1）
   K230 侧串口固定用 ybUtils.YbUart（IO9/IO10 被固件预占，machine.UART 打不开）

调参都在下面「配置区」。改完直接重新运行即可。
"""
import gc
import math
import os
import time

# ============================================================================
#  配置区
# ============================================================================

# --- 串口引脚 ---
UART_BACKEND = "auto"      # auto(=先 machine.UART, 失败退 YbUart) | machine | yb
UART_UNIT = 1
UART_TX = 9
UART_RX = 10
UART_BAUD = 115200

# --- 显示 ---
DISPLAY_MODE = "VIRT"      # VIRT(只用 IDE 画面) | LCD(接了屏) | OFF
DISPLAY_W = 640
DISPLAY_H = 480

# --- 相机 ---
IMG_W = 640
IMG_H = 480

# --- 靶纸检测：A4 白纸亮块 + “四周更暗”校验 ---
# 实测 L(0~100): 白纸 60~70、木柜 25~35、墙 45、黑胶带 20 —— 固定阈值就能分开；
# 黑胶带框正好提供“亮块四周更暗”的校验（白瓷砖地没有这圈暗框，不会误检）。
# 现场日志：真靶纸 长边184~190px/密度0.83~0.96/对比90+；误检 密度0.51~0.73/对比22~67。
# 所以下面加了长边范围、密度、对比三道静态闸门，外加尺寸/位置/连续确认三道时间闸门。
# 详细说明见 tools/vision_check.py 顶部。
PAPER_TH = 58              # 亮度阈值（0~100）
PAPER_TH_ALT = 72          # 兜底阈值：全图第一遍失败时再试
PAPER_A_MAX = 32           # |a| 上限（偏色背景会被排除）
PAPER_B_MAX = 32           # |b| 上限
PAPER_MIN_AREA = 900       # 最小像素面积（2.2m 处 A4 约 60x42px ≈ 2000px）
PAPER_MAX_AREA_RATIO = 0.85
PAPER_MIN_LONG = 60        # 长边下限 px（2.2m 处约 60px；再小就是背景小亮块）
PAPER_MAX_LONG = 480       # 长边上限 px
PAPER_ASPECT_MIN = 0.70    # A4=1.41；斜视透视下会缩到接近 1
PAPER_ASPECT_MAX = 3.00
PAPER_DENSITY_MIN = 0.70   # 兜底（没拟合出四边形时）：像素/外接框面积
PAPER_DENSITY_QUAD_MIN = 0.72  # 拟合出四边形后：像素/四边形面积
PAPER_CONTRAST_MIN = 40    # 内亮度 - 外亮度（0~255 量程）
PAPER_DARK_MARGIN = 25     # 单个外侧采样点算“暗”的门槛
PAPER_DARK_FRAC_MIN = 0.70  # 外侧 12 个点里至少这么多比例要比内部暗
# --- 四边形拟合（斜视时画出来是梯形，靶心=对角线交点=透视中心）---
QUAD_SCAN_N = 7            # 每边取几行/几列做扫描
QUAD_PERP_PX = 2           # 扫描时垂直方向各看几像素（跨过 1~2px 印刷细线）
QUAD_AREA_LO = 0.50        # 四边形面积 / 亮块像素 的合理范围
QUAD_AREA_HI = 1.35
# 扫描用的相对亮度门槛（纸面有阴影时固定阈值会把暗的那半边切掉）
SCAN_TH_K = 0.50
SCAN_TH_LO = 50
SCAN_TH_HI = 88
QUAD_LIM_PAD = 18          # 扫描半径 = 中心到亮块该边的距离 + 这个余量
# 跟踪窗必须大于整张纸（四边形扫描要摸到四条边，窗口小了亮块会被裁掉）
TRACK_K = 0.50             # 跟踪窗 = 四边形长边 x TRACK_K + TRACK_PAD
TRACK_PAD = 30
SEED_EVERY = 2             # 锁定后每几帧做一次亮块搜索（其它帧只做四边形复测）
HOLD_FRAMES = 15           # 丢靶后还画/还用多少帧
LOST_FULL = 18             # 丢这么多帧后放弃小窗，改全图搜索
FULL_EVERY = 3             # 每几帧做一次全图搜索
SMOOTH = 0.55              # 平滑系数
DEADBAND_PX = 1.5          # 平滑死区（小于它不动，画面不抖）
SIZE_GATE_LO = 0.55        # 跟踪时允许的长边变化范围（一帧内不可能变太多）
SIZE_GATE_HI = 1.45
CONFIRM_N = 3              # 全图候选连续确认几次才算重新锁定
CONFIRM_DXY = 35           # 确认时的位置一致范围 px
CONFIRM_DSIZE = 0.60       # 确认时的尺寸一致范围（±60%）
PENDING_MISS = 3           # 确认过程中允许漏几次
PAPER_LONG_M = 0.297       # A4 长边实际长度（米），用于估距离

# --- 激光光斑检测 ---
SPOT_ROI_HALF = 45         # 光轴固定，只在学习到的点附近找光斑
SPOT_THRESHOLDS = [        # LAB 阈值，可多组
    (88, 100, -40, 90, -50, 90),   # 过曝白芯（纸面 LAB-L≈78 到不了）
    (58, 100, 25, 90, -20, 70),    # 明显红边（a>=25，原 a>=6 太松）
]
SPOT_MIN_AREA = 2
SPOT_MAX_AREA = 3000
SPOT_MAX_ASPECT = 3.0
SPOT_EVERY = 4             # 每几帧搜一次光斑（光轴固定，中间帧复用）
SPOT_ADAPT_GAIN = 0.06     # 光轴点慢速自适应（把误检拖跑的风险限制住）
SPOT_ADAPT_LIMIT = 6.0     # 单次最多修正多少像素

# --- 瞄准控制（积分型视觉伺服 + P 项挂在实测姿态上）---
FX_PX = 445.0              # 像素焦距（640 宽）。0.6m 处 A4 长边≈222px 反推
SIGN_YAW = 1.0             # 偏置方向；若越转越远就把对应项改 -1
SIGN_PITCH = 1.0
KP_P_YAW = 0.75            # P 项：一次给出 75% 的偏差角度
KP_P_PITCH = 0.50
KI = 0.015                 # I 项：只抹静差
KI_BAND_PX = 40.0          # 误差小于这么多像素才允许积分
I_LIMIT_DEG = 3.0          # 积分限幅
DEADBAND_PX = 2.0          # 误差死区（640x480 下 1px≈3.5mm@1.5m；1280x720 用 6）
RATE_LIMIT_DPS = 8.0       # 偏置变化率限幅（防甩）
MAX_YAW_DEG = 170.0
MAX_PITCH_DEG = 60.0
LOST_COAST_S = 1.0         # 丢靶后还能沿用最后误差多久

# --- 时序 / 安全 ---
AIM_HZ = 50                # 给 H723 发 AIM 的频率
HEARTBEAT_MS = 200
MODE_REASSERT_MS = 2000    # 周期性重发 MODE（H723 可能被看门狗切回 STAB）
TELEM_TIMEOUT_MS = 1500    # 遥测断这么久 -> 主动发 STAB
LASER_ENABLE = True        # 是否让 H723 点激光（AIM 模式下才有效）
DEBUG_PRINT_MS = 1000      # 终端打印周期

# ============================================================================
#  协议
# ============================================================================
SOF = b"\xAA\x55"

MSG_AIM = 0x10
MSG_MODE = 0x11
MSG_HEARTBEAT = 0x12
MSG_SET_ZERO = 0x13
MSG_GIMBAL_STATE = 0x90
MSG_ACK = 0x91
MSG_TEXT = 0x93

MODE_IDLE = 0
MODE_STAB = 1
MODE_AIM = 2

FLAG_LASER_ON = 0x01
FLAG_AIM_VALID = 0x02
FLAG_LOCKED = 0x10

ST_READY = 0x10
ST_LASER_ON = 0x08


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
    y = max(-32768, min(32767, int(round(yaw_deg * 100.0))))
    p = max(-32768, min(32767, int(round(pitch_deg * 100.0))))
    return ustruct.pack("<hhBB", y, p, flags & 0xFF,
                        max(0, min(255, int(quality))))


def pack_mode(mode, arg=0):
    import ustruct
    return ustruct.pack("<BB", mode & 0xFF, arg & 0xFF)


class FrameParser(object):
    def __init__(self):
        self.buf = bytearray()
        self.ok = 0
        self.crc_err = 0
        self.bad_len = 0
        self.resync = 0

    def feed(self, data):
        out = []
        self.buf.extend(data)
        while True:
            i = self.buf.find(SOF)
            if i < 0:
                if len(self.buf) > 1:
                    self.resync += 1
                    # MicroPython 的 bytearray 不支持切片删除（del buf[x:]），
                    # 必须用重新切片代替，否则报
                    # "TypeError: 'bytearray' object doesn't support item deletion"
                    self.buf = self.buf[-1:]
                break
            if i > 0:
                self.resync += 1
                self.buf = self.buf[i:]
            if len(self.buf) < 5:
                break
            length = self.buf[4]
            if length > 240:
                self.bad_len += 1
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


_GIMBAL_FMT = "<BBhhhhhhhBI"


def unpack_state(payload):
    """-> dict or None"""
    import ustruct
    if len(payload) != 21:
        return None
    v = ustruct.unpack(_GIMBAL_FMT, payload)
    return {
        "state": v[0], "fault": v[1],
        "yaw": v[2] / 100.0, "pitch": v[3] / 100.0, "roll": v[4] / 100.0,
        "ymotor": v[5] / 100.0, "pmotor": v[6] / 100.0,
        "gyro_y": v[7] / 100.0, "gyro_z": v[8] / 100.0,
        "flags": v[9], "up_ms": v[10],
    }


# ============================================================================
#  串口链路
# ============================================================================
class Link(object):
    """串口后端自动选择。

    实测（CanMV v1.4.3 / k230_canmv_yahboom 固件）：
      · machine.UART(1, tx=Pin(9), rx=Pin(10)) 打不开 ——
        固件启动时已经把 IO9/IO10 分配给了它自己的功能（报
        "pin(9) is not a GPIO pin"），所以这条要放在后面当备选。
      · ybUtils.YbUart 是可用的，它在固件里占的就是 IO9(TXD)/IO10(RXD)。
    按顺序试，哪个先打开用哪个。
    """

    @staticmethod
    def _candidates():
        def yb():
            from ybUtils.YbUart import YbUart
            return YbUart(baudrate=UART_BAUD)

        def m_nopin():
            from machine import UART
            return UART(UART_UNIT, baudrate=UART_BAUD, bits=8,
                        parity=None, stop=0)

        def m_pins19():
            from machine import UART, Pin
            return UART(1, baudrate=UART_BAUD, tx=Pin(9), rx=Pin(10),
                        bits=8, parity=None, stop=0)

        def m_pins3233():
            from machine import UART, Pin
            return UART(3, baudrate=UART_BAUD, tx=Pin(32), rx=Pin(33),
                        bits=8, parity=None, stop=0)

        def uart3_fpioa():
            # 备用线路：12Pin GPIO 的 IO32(TXD)/IO33(RXD) 手动配成 UART3。
            # 和 YbUart 配 IO9/IO10 是同一套做法（先 FPIOA 再建 UART）。
            from machine import FPIOA, UART
            fp = FPIOA()
            fp.set_function(32, FPIOA.UART3_TXD, ie=0, oe=1, pu=1)
            fp.set_function(33, FPIOA.UART3_RXD, ie=1, oe=0, pu=1)
            return UART(3, baudrate=UART_BAUD)

        def yb_swapped():
            # IO9/IO10 反着用（IO10=TXD, IO9=RXD）：
            # 两根数据线如果接反了，不用拆线也能通信。
            from machine import FPIOA, UART
            fp = FPIOA()
            fp.set_function(10, FPIOA.UART1_TXD, ie=0, oe=1, pu=1)
            fp.set_function(9, FPIOA.UART1_RXD, ie=1, oe=0, pu=1)
            return UART(1, baudrate=UART_BAUD)

        return [("YbUart(亚博封装, IO9/IO10)", yb),
                ("UART1 IO9/IO10 反接", yb_swapped),
                ("UART3 手配 IO32/IO33", uart3_fpioa),
                ("UART(%d) 不指定引脚" % UART_UNIT, m_nopin),
                ("UART(1) tx=IO9 rx=IO10", m_pins19),
                ("UART(3) tx=IO32 rx=IO33", m_pins3233)]

    def __init__(self):
        self.dev = None
        self.name = ""
        for label, opener in self._candidates():
            try:
                self.dev = opener()
                self.name = label
                break
            except Exception as e:
                print("串口后端 %s 打不开: %s" % (label, e))
        if self.dev is None:
            raise OSError("没有任何串口后端能打开")
        print("串口: %s @%d" % (self.name, UART_BAUD))

    def send(self, data):
        try:
            self.dev.write(data)
        except Exception as e:
            print("串口写失败: %s" % e)

    def read(self, n=256):
        try:
            if hasattr(self.dev, "any") and self.dev.any() <= 0:
                return b""
            return self.dev.read(n) or b""
        except Exception:
            return b""

    def close(self):
        try:
            self.dev.deinit()
        except Exception:
            pass


# ============================================================================
#  视觉：靶纸（黑胶带框）检测
# ============================================================================
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


def paper_contrast(img, x, y, w, h):
    """内亮外暗校验：返回 (内部平均亮度, 外侧平均亮度, 外侧合格比例)。

    外侧取 12 个点（四边各 3 个），要求其中至少 PAPER_DARK_FRAC_MIN 的比例
    明显比内部暗 —— 背景柜子那种“只有一边有暗边”的亮块因此过不了。
    """
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
    aspect = long_side / max(1.0, short_side)
    if (long_side < PAPER_MIN_LONG) or (long_side > PAPER_MAX_LONG) or \
            (aspect < PAPER_ASPECT_MIN) or (aspect > PAPER_ASPECT_MAX):
        return None, 0, "[复测:尺寸] "
    return (px, bx, by, w, h, long_side, aspect, px / max(1.0, qa),
            ins, outs, corners, ctr[0], ctr[1]), 0, ""


def _probe(img, x, y, dx, dy, th):
    """沿射线前后各 2px 取样，多数（>=3/5）亮才算“纸”。

    印刷圆圈的细线在“圆的正左/正右”几乎与横向射线垂直，只做垂直方向
    取最大跨不过去（扫描会停在第一圈圆环，四边形缩到圆环范围）；
    沿射线取多数则与细线角度无关。
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


def quad_from_blob(img, bx, by, bw, bh, th):
    """把亮块拟合成四边形（斜视=梯形）。
    返回 (corners, center, long_side, short_side, quad_area) 或 None；
    center = 两条对角线交点 = 透视意义下的靶心。"""
    icx = int(bx + bw / 2.0)
    icy = int(by + bh / 2.0)
    # 每条边的扫描半径 = 中心到亮块该边的距离 + QUAD_LIM_PAD（放太远会越过
    # 黑胶带摸到背景亮边，那一条边就会“跳”出去）
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
        self.pend = None       # [cu, cv, long, cnt, miss]
        self.frame = 0
        self.last_n = 0
        self.last_dbg = ""
        self.did_full = False
        self.state = "搜索"
        self.err = 0
        self.err_msg = ""

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
        cu, cv, lo = cand[11], cand[12], cand[5]
        # 重新锁定也要过尺寸合理性：纸不可能在半秒里变成 1/4 大
        # （现场日志里锁到背景 71px 亮块就是这个口子漏的），丢靶>3s 才放开
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
        x, y, w, h = cand[1], cand[2], cand[3], cand[4]
        cu, cv = cand[11], cand[12]      # 四边形对角线交点 = 透视中心
        if not self.have:
            self.u, self.v, self.w, self.h = cu, cv, w, h
            self.have = True
        else:
            if abs(cu - self.u) > DEADBAND_PX:
                self.u += SMOOTH * (cu - self.u)
            if abs(cv - self.v) > DEADBAND_PX:
                self.v += SMOOTH * (cv - self.v)
            if abs(w - self.w) > 2 * DEADBAND_PX:
                self.w += SMOOTH * (w - self.w)
            if abs(h - self.h) > 2 * DEADBAND_PX:
                self.h += SMOOTH * (h - self.h)
        self.meas = cand
        self.corners = cand[10]
        self.lost = 0
        self.state = "锁定"

    def detect(self, img):
        self.frame += 1
        cand = None
        n = 0
        dbg = ""
        self.did_full = False
        try:
            if self.have and self.lost < LOST_FULL:
                if ((self.frame % SEED_EVERY) == 0) or (self.meas is None):
                    cand, n, dbg = self._find(img, self.roi(), PAPER_TH)
                else:
                    # 隔帧只做四边形复测（省 ~13ms），靶心仍每帧更新；
                    # 复测失败立刻补一次完整搜索。
                    cand, n, dbg = self._find(img, self.roi(), PAPER_TH,
                                              self.meas)
                    if cand is None:
                        cand, n, dbg = self._find(img, self.roi(), PAPER_TH)
                if (cand is not None) and (not self._size_ok(cand[5])):
                    dbg = "[尺寸闸门%.0fpx] " % cand[5] + dbg
                    cand = None
                if (cand is None) and ((self.frame % FULL_EVERY) == 0):
                    self.did_full = True
                    c2, n2, d2 = self._find(img, (0, 0, IMG_W, IMG_H),
                                            PAPER_TH)
                    n += n2
                    dbg += d2
                    if self._confirm(c2):
                        cand = c2
                    elif c2 is None:
                        c3, n3, d3 = self._find(img, (0, 0, IMG_W, IMG_H),
                                                PAPER_TH_ALT)
                        n += n3
                        dbg += d3
                        if self._confirm(c3):
                            cand = c3
            elif (self.frame % FULL_EVERY) == 0:
                self.did_full = True
                c2, n2, d2 = self._find(img, (0, 0, IMG_W, IMG_H),
                                        PAPER_TH)
                if c2 is None:
                    c3, n3, d3 = self._find(img, (0, 0, IMG_W, IMG_H),
                                            PAPER_TH_ALT)
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
            self._accept(cand)
        else:
            self.lost += 1
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

    def _find(self, img, roi, th, seed=None):
        """在 roi 里找 A4 靶纸，返回 (best, 亮块数, 诊断字符串)。

        seed=上次测量元组时走快速复测（只重扫四边形，不找亮块）。"""
        if seed is not None:
            return _requad(img, seed)
        best = None
        n = 0
        dbg = ""
        blobs = img.find_blobs(
            [(th, 100, -PAPER_A_MAX, PAPER_A_MAX,
              -PAPER_B_MAX, PAPER_B_MAX)],
            roi=roi, merge=True, margin=6,
            area_threshold=PAPER_MIN_AREA,
            pixels_threshold=PAPER_MIN_AREA)
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
            if n <= 3:
                dbg += "[%dpx 框%.0f 内%d 外%d 暗边%.0f%%] " % (
                    px, box_long, ins, outs, frac * 100.0)
            if box_long < PAPER_MIN_LONG * 0.8 or \
                    box_long > PAPER_MAX_LONG * 1.3:
                continue
            q = quad_from_blob(img, x, y, w, h, _scan_th(ins))
            corners = None
            qa = 0.0
            ctr = (x + w / 2.0, y + h / 2.0)
            if q is not None:
                corners, ctr, long_side, short_side, qa = q
                if (qa < QUAD_AREA_LO * px) or (qa > QUAD_AREA_HI * px):
                    corners = None            # 拟合和亮块对不上，弃用
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
            if px < PAPER_MIN_AREA:
                continue
            if px > PAPER_MAX_AREA_RATIO * IMG_W * IMG_H:
                continue
            if long_side < PAPER_MIN_LONG or long_side > PAPER_MAX_LONG:
                continue
            if aspect < PAPER_ASPECT_MIN or aspect > PAPER_ASPECT_MAX:
                continue
            if density < dens_min:
                continue
            if (x <= 1) or (y <= 1) or \
                    ((x + w) >= IMG_W - 1) or ((y + h) >= IMG_H - 1):
                continue
            if ins < 0 or (ins - outs) < PAPER_CONTRAST_MIN:
                continue
            if frac < PAPER_DARK_FRAC_MIN:
                continue
            score = px * (0.5 + min(ins - outs, 80) / 80.0)
            if best is None or score > best[0]:
                best = (score, (px, x, y, w, h, long_side, aspect, density,
                                ins, outs, corners, ctr[0], ctr[1]))
        if best is None:
            return None, n, dbg
        return best[1], n, dbg


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

    # ---- 原生 find_blobs 版（RGB565）----
    def _detect_blobs(self, img):
        x0, y0, w, h = self.roi()
        best = None
        # 两个阈值放一次调用（find_blobs 每次约 10ms 固定开销，能省则省）
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


# ============================================================================
#  瞄准控制
# ============================================================================
class AimCtrl(object):
    def __init__(self):
        self.yaw = 0.0
        self.pitch = 0.0
        self.i_u = 0.0
        self.i_v = 0.0
        self.err_px = 0.0
        self.valid = False
        self.locked = False

    def reset(self):
        self.i_u = 0.0
        self.i_v = 0.0

    def update(self, dt, err_u, err_v, att_rel):
        """err_u/err_v = 靶心 - 光斑（像素）；att_rel = 相对锁零基准的实测姿态"""
        self.err_px = math.sqrt(err_u * err_u + err_v * err_v)
        eu, ev = err_u, err_v
        if abs(eu) < DEADBAND_PX:
            eu = 0.0
        if abs(ev) < DEADBAND_PX:
            ev = 0.0

        # I 项用"没进死区前"的原始误差：死区只用来抑制 P 项抖动，
        # 不能让积分也停住，否则最后几像素的静差永远消不掉。
        e_deg_u_raw = err_u / FX_PX * 57.29578
        e_deg_v_raw = err_v / FX_PX * 57.29578
        e_deg_u = eu / FX_PX * 57.29578
        e_deg_v = ev / FX_PX * 57.29578

        # 条件积分：误差小才累加，避免"攒过头再冲出去"
        if self.err_px <= KI_BAND_PX and dt > 1e-4:
            self.i_u += KI * e_deg_u_raw
            self.i_v += KI * e_deg_v_raw
            self.i_u = max(-I_LIMIT_DEG, min(I_LIMIT_DEG, self.i_u))
            self.i_v = max(-I_LIMIT_DEG, min(I_LIMIT_DEG, self.i_v))

        if att_rel is not None:
            base_u = att_rel[0] + self.i_u
            base_v = att_rel[1] + self.i_v
        else:
            base_u = self.yaw + self.i_u
            base_v = self.pitch + self.i_v

        want_u = base_u + SIGN_YAW * KP_P_YAW * e_deg_u
        want_v = base_v + SIGN_PITCH * KP_P_PITCH * e_deg_v

        # 变化率限幅（防甩） + 绝对限幅
        step = RATE_LIMIT_DPS * max(dt, 1e-3)
        du = want_u - self.yaw
        dv = want_v - self.pitch
        if du > step:
            du = step
        elif du < -step:
            du = -step
        if dv > step:
            dv = step
        elif dv < -step:
            dv = -step
        self.yaw = max(-MAX_YAW_DEG, min(MAX_YAW_DEG, self.yaw + du))
        self.pitch = max(-MAX_PITCH_DEG, min(MAX_PITCH_DEG, self.pitch + dv))
        self.locked = self.err_px < DEADBAND_PX


# ============================================================================
#  显示 / 相机
# ============================================================================
def init_camera():
    from media.sensor import Sensor, CAM_CHN_ID_0
    sensor = Sensor(id=2, width=1280, height=960, fps=90)
    sensor.reset()
    time.sleep_ms(100)
    sensor.set_framesize(width=IMG_W, height=IMG_H, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
    return sensor


def print_fpioa_state():
    """启动时打印关键引脚功能，方便一眼确认串口引脚配对了。"""
    try:
        from machine import FPIOA
        fp = FPIOA()
        out = []
        for pin in (9, 10, 32, 33):
            try:
                out.append("IO%d=%s" % (pin, fp.get_pin_func(pin)))
            except Exception:
                out.append("IO%d=?" % pin)
        print("引脚功能: %s" % " ".join(out))
    except Exception:
        pass


_display = None
_media = None


def init_display():
    """返回画布 Image 或 None。没有屏幕时只用 IDE 画面视图（VIRT）。"""
    global _display, _media
    if DISPLAY_MODE == "OFF":
        return None
    try:
        from media.display import Display
        from media.media import MediaManager
        import image
        if DISPLAY_MODE == "LCD":
            Display.init(Display.ST7701, width=DISPLAY_W, height=DISPLAY_H,
                         to_ide=True)
        else:
            Display.init(Display.VIRT, width=DISPLAY_W, height=DISPLAY_H,
                         fps=30)
        _display = Display
        _media = MediaManager
        return image.Image(DISPLAY_W, DISPLAY_H, image.RGB565)
    except Exception as e:
        print("显示不可用(%s)，继续无显示运行" % e)
        _display = None
        _media = None
        return None


# ============================================================================
#  状态机
# ============================================================================
ST_WAIT_READY = 0
ST_SET_ZERO = 1
ST_TRACK = 2
ST_NAME = {ST_WAIT_READY: "WAIT_READY", ST_SET_ZERO: "SET_ZERO",
           ST_TRACK: "TRACK"}


def main():
    link = Link()
    parser = FrameParser()
    print_fpioa_state()
    sensor = init_camera()
    canvas = init_display()
    from media.media import MediaManager
    MediaManager.init()
    sensor.run()

    target_det = PaperDetector()
    spot_det = SpotDetector()
    ctrl = AimCtrl()

    state = ST_WAIT_READY
    state_t = time.ticks_ms()
    att_zero = None
    gz = None
    last_gz_t = 0
    t_hb = time.ticks_ms()
    t_aim = time.ticks_ms()
    t_mode = time.ticks_ms()
    t_print = time.ticks_ms()
    t_last_frame = time.ticks_ms()
    last_tgt = None
    last_tgt_t = 0
    last_spot = None
    n_frames = 0
    n_frames_prev = 0
    n_hit = 0
    n_spot = 0
    sent_mode = None

    print("=" * 60)
    print("K230 瞄准程序启动  %dx%d  靶纸=白纸亮块+暗框  显示=%s"
          % (IMG_W, IMG_H, DISPLAY_MODE))
    print("=" * 60)

    try:
        while True:
            os.exitpoint()
            now = time.ticks_ms()

            # ---------- 1. 收 H723 遥测 ----------
            data = link.read(256)
            if data:
                for msg_id, seq, payload in parser.feed(data):
                    if msg_id == MSG_GIMBAL_STATE:
                        s = unpack_state(payload)
                        if s is not None:
                            gz = s
                            last_gz_t = now
                    elif msg_id == MSG_TEXT:
                        try:
                            print("H723: %s" % payload.decode("utf-8"))
                        except Exception:
                            pass
                    elif msg_id == MSG_ACK:
                        pass

            # ---------- 2. 心跳 ----------
            if time.ticks_diff(now, t_hb) >= HEARTBEAT_MS:
                t_hb = now
                link.send(build_frame(MSG_HEARTBEAT))

            # ---------- 3. 安全：遥测断 -> STAB ----------
            if (state != ST_WAIT_READY) and \
                    (time.ticks_diff(now, last_gz_t) > TELEM_TIMEOUT_MS):
                link.send(build_frame(MSG_MODE, pack_mode(MODE_STAB)))
                sent_mode = MODE_STAB
                print("!! 遥测断流 -> 发 STAB，回到 WAIT_READY")
                state = ST_WAIT_READY
                state_t = now
                att_zero = None
                ctrl.reset()
                ctrl.yaw = 0.0
                ctrl.pitch = 0.0

            ready = bool(gz is not None and (gz["flags"] & ST_READY))

            # ---------- 4. 状态机 ----------
            if state == ST_WAIT_READY:
                if ready:
                    link.send(build_frame(MSG_MODE, pack_mode(MODE_STAB)))
                    sent_mode = MODE_STAB
                    link.send(build_frame(MSG_SET_ZERO))
                    att_zero = (gz["yaw"], gz["pitch"])
                    state = ST_SET_ZERO
                    state_t = now
                    print("H723 READY -> SET_ZERO，0.3s 后进 AIM")
            elif state == ST_SET_ZERO:
                if time.ticks_diff(now, state_t) >= 300:
                    link.send(build_frame(MSG_MODE, pack_mode(MODE_AIM)))
                    sent_mode = MODE_AIM
                    state = ST_TRACK
                    state_t = now
                    print("进入 AIM，开始闭环瞄准")
            elif state == ST_TRACK:
                # 周期重发 MODE：H723 被看门狗切回 STAB 后自动拉回来
                if time.ticks_diff(now, t_mode) >= MODE_REASSERT_MS:
                    t_mode = now
                    link.send(build_frame(MSG_MODE, pack_mode(MODE_AIM)))

                dt = time.ticks_diff(now, t_last_frame) / 1000.0
                t_last_frame = now
                img = sensor.snapshot()
                n_frames += 1

                res = target_det.detect(img)
                if (n_frames % SPOT_EVERY) == 0:
                    last_spot = spot_det.detect(img)
                spot = last_spot

                # 丢靶滑行：短时间内沿用最后一次误差继续闭环
                if res is not None:
                    n_hit += 1
                    tgt_uv = res["center"]
                    last_tgt = tgt_uv
                    last_tgt_t = now
                else:
                    if (time.ticks_diff(now, last_tgt_t) / 1000.0) > \
                            LOST_COAST_S:
                        tgt_uv = None
                    else:
                        tgt_uv = last_tgt

                att_rel = None
                if att_zero is not None and ready and gz is not None:
                    att_rel = (gz["yaw"] - att_zero[0],
                               gz["pitch"] - att_zero[1])

                if spot is not None:
                    n_spot += 1
                    su, sv = spot["uv"]
                else:
                    su, sv = spot_det.u, spot_det.v

                if tgt_uv is not None and spot is not None:
                    ctrl.valid = True
                    ctrl.update(max(dt, 1e-3), tgt_uv[0] - su, tgt_uv[1] - sv,
                                att_rel)
                else:
                    ctrl.valid = False
                    ctrl.locked = False

                # ---- 画到 IDE 画面 ----
                if canvas is not None:
                    if res is not None:
                        qc = res.get("corners")
                        if qc is not None:
                            for i in range(4):
                                a = qc[i]
                                b = qc[(i + 1) % 4]
                                img.draw_line(int(a[0]), int(a[1]),
                                              int(b[0]), int(b[1]),
                                              color=(0, 255, 0), thickness=2)
                        else:
                            bx, by, bw, bh = res["box"]
                            img.draw_rectangle(bx, by, bw, bh,
                                               color=(0, 255, 0), thickness=2)
                        img.draw_cross(int(res["center"][0]),
                                       int(res["center"][1]),
                                       color=(255, 0, 0), size=12,
                                       thickness=2)
                    if spot is not None:
                        img.draw_circle(int(su), int(sv), 8,
                                        color=(255, 255, 0), thickness=2)
                    else:
                        img.draw_circle(int(spot_det.u), int(spot_det.v), 8,
                                        color=(128, 128, 128), thickness=1)
                    img.draw_string_advanced(
                        4, 4, 18,
                        "%s err=%.0fpx yaw=%.1f pit=%.1f"
                        % (ST_NAME[state], ctrl.err_px, ctrl.yaw, ctrl.pitch),
                        color=(255, 220, 0))
                    if _display is not None:
                        _display.show_image(img)

            # ---------- 5. 发 AIM（50Hz，保持链路活着）----------
            if state in (ST_SET_ZERO, ST_TRACK) and \
                    time.ticks_diff(now, t_aim) >= (1000 // AIM_HZ):
                t_aim = now
                flags = 0
                if state == ST_TRACK and ctrl.valid:
                    flags |= FLAG_AIM_VALID
                if ctrl.locked:
                    flags |= FLAG_LOCKED
                if state == ST_TRACK and LASER_ENABLE:
                    flags |= FLAG_LASER_ON
                quality = 200 if ctrl.valid else 0
                link.send(build_frame(MSG_AIM,
                                      pack_aim(ctrl.yaw, ctrl.pitch,
                                               flags, quality)))

            # ---------- 6. 终端打印 ----------
            if time.ticks_diff(now, t_print) >= DEBUG_PRINT_MS:
                fps = ((n_frames - n_frames_prev) * 1000.0 /
                       max(1, time.ticks_diff(now, t_print)))
                n_frames_prev = n_frames
                t_print = now
                tgt_s = "无靶"
                if target_det.meas is not None and target_det.lost == 0:
                    tgt_s = "%s %.2fm" % (
                        target_det.state,
                        FX_PX * PAPER_LONG_M / max(1.0, target_det.meas[5]))
                if gz is not None:
                    print("[%s] %.1ffps gz(state=%d flags=0x%02X yaw=%.1f) "
                          "靶=%s 命中%d/%d 光斑%d err=%.0f yaw=%.1f pit=%.1f "
                          "ok=%d crc=%d"
                          % (ST_NAME[state], fps, gz["state"], gz["flags"],
                             gz["yaw"], tgt_s, n_hit, n_frames, n_spot,
                             ctrl.err_px, ctrl.yaw, ctrl.pitch,
                             parser.ok, parser.crc_err))
                else:
                    print("[%s] 等 H723 遥测... 靶=%s ok=%d crc=%d"
                          % (ST_NAME[state], tgt_s,
                             parser.ok, parser.crc_err))
            if (n_frames % 30) == 0:
                gc.collect()
            time.sleep_ms(1)
    except KeyboardInterrupt:
        print("用户停止")
    finally:
        # 退出时务必让云台回到安全态并关激光
        try:
            link.send(build_frame(MSG_MODE, pack_mode(MODE_STAB)))
            time.sleep_ms(50)
        except Exception:
            pass
        link.close()
        try:
            sensor.stop()
        except Exception:
            pass
        if canvas is not None:
            try:
                _display.deinit()
                os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
                time.sleep_ms(100)
                _media.deinit()
            except Exception:
                pass
        print("已退出")


main()
