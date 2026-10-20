# -*- coding: utf-8 -*-
"""main.py —— K230 视觉瞄准主程序

整体上电即自动运行：把本文件保存到 K230 SD 卡根目录、命名 main.py 即可。

K230 只做视觉：
   相机取图 -> 靶纸/激光像素坐标 -> AIM 串口帧 -> H723
H723 负责像素误差 -> 角度偏置 -> 云台控制。
调试画面请用 tools/vision_check.py；整机 main.py 不做显示。

和 H723 的约定（协议不变）：
   帧: AA 55 | msg_id | seq | len | payload | crc16(小端)
   0x12 心跳 / 0x13 SET_ZERO / 0x11 MODE(0IDLE 1STAB 2AIM) / 0x10 AIM(偏置)
   0x90 遥测 / 0x91 ACK / 0x93 调试文本
   H723 端: 0.5s 收不到 AIM 自动切 STAB 并关激光（安全看门狗）

接线：
   K230 IO9 (TXD)  <-> H723 USART1 的 RX (PA10)
   K230 IO10(RXD)  <-> H723 USART1 的 TX (PA9)
   GND             <-> GND（必须共地）
   5V              <-  H723 UART10 排针 VCC

调参都在下面「配置区」。改完直接重新运行即可。
"""
import gc
import math
import os
import time

try:
    import image
except Exception:
    image = None
try:
    import cv_lite as _cv_lite
except Exception:
    _cv_lite = None

# ============================================================================
#  配置区
# ============================================================================

# --- 串口引脚 ---
UART_BACKEND = "auto"      # auto(=先 machine.UART, 失败退 YbUart) | machine | yb
UART_UNIT = 1
UART_TX = 9
UART_RX = 10
UART_BAUD = 115200

CAM_VFLIP = True           # 画面上下颠倒 -> True
CAM_HMIRROR = False        # 画面左右镜像 -> True

PROC_W = 640
PROC_H = 480
IMG_W = PROC_W
IMG_H = PROC_H
S_SCALE = PROC_W / 640.0    # 像素常数缩放系数（640 档 = 1.0，800 档 = 1.25）
VIS_K = 7.8 * S_SCALE       # px/°（现场标定 7.8 @640x480，随分辨率同比缩放）
USE_CLOSE_DETECTOR = False  # True=新灰度闭运算检测器；False=10.13 已验证检测器
GRAY_DIRECT = True
CLOSE_BIN_TH = 130          # 二值化阈值
CLOSE_MORPH_SIZE = 1        # close(1) = 3x3
FULL_FRAME_CLOSE = True
USE_CLOSE_ROI = True
CLOSE_FRAME_COS = 0.65
CLOSE_ROI_KERNEL = 3
CLOSE_ROI_ITERS = 1
CLOSE_ROI_TH = 0
CLOSE_ROI_K = 0.52
CLOSE_ROI_PAD = int(round(18 * S_SCALE))
CLOSE_ROI_MIN_LONG = int(round(100 * S_SCALE))
EXPO_FIX = False
EXPO_US = 8000

PAPER_TH = 50              # 亮度阈值（0~100）的**下限**；实际由 scene_th() 自适应
PAPER_TH_FIXED = 0
# 近处反光试验旋钮：0=关闭（远处/未锁定始终自动）；
# >0=锁定且目标长边进入近处后，强制使用这个 LAB-L 阈值。
PAPER_TH_NEAR_FIXED = 0
PAPER_A_MAX = 32           # |a| 上限（偏色背景会被排除）
PAPER_B_MAX = 32           # |b| 上限
PAPER_MIN_AREA = int(350 * S_SCALE * S_SCALE)   # 最小像素面积（面积随 S² 缩放）
PAPER_MAX_AREA_RATIO = 0.50
PAPER_MIN_LONG = int(round(36 * S_SCALE))    # 长边下限 px（长度随 S 缩放）
PAPER_MAX_LONG = int(round(480 * S_SCALE))   # 长边上限 px
PAPER_ASPECT_MIN = 0.70    # A4=1.41；斜视透视下会缩到接近 1
PAPER_ASPECT_MAX = 2.00
PAPER_DENSITY_MIN = 0.56   # 兜底（没拟合出四边形时）下限
PAPER_DENSITY_MIN_STRONG = 0.45
PAPER_CONTRAST_STRONG = 70
PAPER_DENSITY_QUAD_MIN = 0.65
PAPER_DENSITY_NEAR_MIN = 0.72  # 近处纸面应明显更实心（大背景块密度约 0.6~0.7）
PAPER_CONTRAST_MIN = 30    # 内亮度 - 外亮度（0~255 量程）
PAPER_DARK_MARGIN = 25     # 单个外侧采样点算“暗”的门槛
PAPER_DARK_FRAC_MIN = 0.80  # 外侧 12 点里至少 10 点比纸暗
QUAD_SCAN_N = 3            # 每边取几行/几列做扫描
QUAD_PERP_PX = 1           # 扫描时垂直方向各看几像素
QUAD_AREA_LO = 0.50        # 四边形面积 / 亮块像素 的合理范围
QUAD_AREA_HI = 1.60
# 近处大靶纸：印刷同心圆会让边扫描停在内圈黑线上。长边超过这个值后，
# 只接受“贴着靶纸外沿”的四边形；缩在框内部的四边形一律弃用，
# 退回同一个亮块的外接框中心。这样斜视时保留透视中心，内圈拟合时不会乱跳。
QUAD_NEAR_LONG = int(round(190 * S_SCALE))
QUAD_NEAR_SIDE_K = 0.70    # 近处四边形长边 / 外接框长边 的下限
QUAD_NEAR_FILL_K = 0.50    # 近处四边形面积 / 外接框面积 的下限
QUAD_NEAR_EDGE_K = 0.30    # 四个角离外框边的最大距离（允许反光把框撑大）
QUAD_NEAR_DENSITY_HI = 1.30  # 近处 px/qa 的上限（px 可能含框外反光）
QUAD_NEAR_COS_LIM = 0.42   # 近处四边形内角允许偏离 90° 约 ±25°
# 扫描用的相对亮度门槛（纸面有阴影时固定阈值会把暗的那半边切掉）
SCAN_TH_K = 0.68           # 现场实证：0.5 时柜子面(≈95)会时过时不过 -> 边飞出去
SCAN_TH_LO = 65
SCAN_TH_HI = 125
QUAD_LIM_PAD = int(round(18 * S_SCALE))   # 扫描半径 = 中心到亮块该边的距离 + 这个余量
# "外侧采样点"离纸边的距离（paper_contrast 里那 12 个点）：固定 5px 时，
# 分辨率越高 = 物理上离纸边越近。跟着 S 缩放 = 物理含义完全不变。
PROBE_OFF_PX = int(round(5 * S_SCALE))
# 跟踪窗必须大于整张纸（四边形扫描要摸到四条边，窗口小了亮块会被裁掉）
TRACK_K = 0.50             # 跟踪窗 = 四边形长边 x TRACK_K + TRACK_PAD
# 跟踪窗半径 = 长边×TRACK_K + TRACK_PAD。K 不能小于 0.5（要装得下整张纸，
# 否则四边形扫描会被窗口裁掉）；能省的是"外扩量"：60 -> 35，窗口面积少 ~27%，
# 而 0.5×长边+35 仍然比整张纸（0.5×长边）大 35px，纸还是完整在窗内。
# 现场锁定后每 3 帧要在这个窗里做一次 find_blobs，这是"目标耗时"的大头。
TRACK_PAD = int(round(35 * S_SCALE))
TRACK_PAD_FAR = int(round(90 * S_SCALE))  # 远距离（纸小）时窗口外扩加大：防"候选0"丢靶
# 碎片合并只处理嵌套的同心碎片；相邻反光/背景不拼。
# 近处靶纸被印刷圈切成“外环 + 中心白盘”：中心盘的尺寸远小于外环，
# 旧规则（嵌套且长边比 >=0.75 才拼）会把它留在外面，导致中心盘自己
# 也能过闸门 -> 预览里一堆小框、锁定对象在中心盘和外环之间跳。
# 允许“外环有洞（密度低）+ 内盘同心 + 内盘不是噪点”时把内盘并回去。
MERGE_RING_DENSITY_MAX = 0.72
MERGE_CENTER_K = 0.15
MERGE_INNER_AREA_MIN = 0.03
SEED_EVERY = 16            # 锁定后每几帧做一次亮块搜索（12->16：帧率再提一截；
                           # 其余帧只做"四边形复测"（几毫秒），靶心照旧每帧更新；
                           # 复测失败会立刻补一次完整搜索，安全性不变）
RE_CHECK_EVERY = 40        # 锁定后每几帧做一次"全图复查"（20->40：每 40 帧一次
                           # 足够发现"锁错对象"，省下每 20 帧一次全图 find_blobs）：
                           # 一旦锁到旁边的白壁/白盒上，窗口搜索只会在那附近打转，
                           # 靶纸就在画面中间也不会被看见（截图实证）。全图复查若
                           # 发现"明显更好的合法候选"就直接换过去，见 detect()。
HOLD_FRAMES = 25           # 丢靶后还画/还用多少帧（15 -> 25：靶纸闪一下不再掉框）
# 丢靶容限：丢这么多帧之后不再找靶（用户要求：比赛里丢靶=失败，不做"找靶"）。
# 25 帧≈0.8s，足够扛住"一闪过"的漏检（这段时间窗口搜索还在跑），
# 再久就说明真的丢了，直接把偏置冻住（不让它照着旧误差乱推）。
LOST_FULL = 25
SMOOTH = 0.90              # 平滑系数（越大越跟手、越小越稳）
# 近处大靶纸：候选在“真纸”和“纸+背景大块”之间轻微切换时，测量的中心
# 会上下偏几十像素。这里比中距离更稳一档，把这种测量跳变压下去；
# 代价是多约 20ms 的群延迟（20fps 下），换近处激光不上下抖。
SMOOTH_NEAR = 0.90
DEADBAND_PX = 1.5 * S_SCALE   # 平滑死区（小于它不动，画面不抖）
SIZE_GATE_LO = 0.50        # 跟踪时允许的长边变化范围（不许换目标）
SIZE_GATE_HI = 1.35
# 最近转弯时目标会快速变大，尺寸可能一帧跨过正常闸门。只有“高密度 +
# 强对比”的真纸候选允许走救援通道；低密度糊块/背景大块仍然被挡住。
SIZE_RESCUE_DENSITY = 0.88
SIZE_RESCUE_CONTRAST = 30
SIZE_RESCUE_MAX = 2.50
CONFIRM_N = 1              # 连续确认几次才算锁定（2->1：一帧即锁）
CONFIRM_DXY = int(round(60 * S_SCALE))   # 确认时的位置一致范围 px（35 -> 60）
CONFIRM_DSIZE = 1.00       # 确认时的尺寸一致范围（±100%；2.0 会连"碎片"一起认进来）
PENDING_MISS = 8           # 确认过程中允许漏几次（5 -> 8）
JUMP_REJECT_PX = int(round(100 * S_SCALE))  # 跟踪期单帧中心跳变上限（超了当误检丢）
SMOOTH_FAR = 0.60          # 远距离平滑（0.30 -> 0.60：51ms -> 15ms）
FAR_LONG_PX = 120.0 * S_SCALE  # 长边小于它 = "远"（平滑/滤波/增益按它分档）
PAPER_LONG_M = 0.275       # 白色区域长边实际长度（米），用于估距离

# --- 激光光斑检测 ---
SPOT_ROI_HALF = int(round(45 * S_SCALE))  # 光轴固定，只在学习到的点附近找光斑
SPOT_THRESHOLDS = [        # LAB 阈值，可多组
    (88, 100, -40, 90, -50, 90),   # 过曝白芯（纸面 LAB-L≈78 到不了）
    (58, 100, 25, 90, -20, 70),    # 明显红边（a>=25，原 a>=6 太松）
]
SPOT_MIN_AREA = 2
# 远距离时激光在纸上的光斑/眩光会变大（用户现场观察），上限从 3000 放到 6000，
# 免得"光斑太大反而检不出来"（检不到光斑 -> 误差是旧的 -> 环路跟着旧误差走）。
SPOT_MAX_AREA = 6000
SPOT_MAX_ASPECT = 3.0
SPOT_EVERY = 4             # 每几帧搜一次光斑（光轴固定，中间帧复用）
SPOT_ADAPT_GAIN = 0.06     # 光轴点慢速自适应（把误检拖跑的风险限制住）
SPOT_ADAPT_LIMIT = 6.0     # 单次最多修正多少像素
SPOT_PREF_PX = int(round(70 * S_SCALE))
SPOT_UV = [IMG_W / 2.0, IMG_H / 2.0]   # 最近一次找到的激光点，主循环每帧更新
USE_FIXED_LASER = True
LASER_UV = (int(round(323 * S_SCALE)), int(round(246 * S_SCALE)))

AUTO_CALIB_LASER = False
CALIB_FRAMES = 60          # 标定取多少帧（≈1.5~2s）
CALIB_MIN_N = 12           # 至少检出多少帧才算数
CALIB_SHIFT_MAX = 30.0     # 中位数离标称值最大偏移（px）
CALIB_SPREAD_MAX = 6.0     # 允许的散布（px）

FX_PX = 445.0 * S_SCALE    # 像素焦距（随分辨率同比缩放），仅用于日志里估距离显示
LOST_COAST_S = 0.5         # 丢靶后还能沿用最后目标位置多久（仅本地显示/日志）

# --- 时序 / 安全 ---
AIM_HZ = 100               # 给 H723 发 AIM 的频率（H723 控制环 500Hz/2ms，见 0x1C）
HEARTBEAT_MS = 200
MODE_REASSERT_MS = 2000    # 周期性重发 MODE（H723 可能被看门狗切回 STAB）
TELEM_TIMEOUT_MS = 1500    # 遥测断这么久 -> 主动发 STAB
PARAM_SEND = []
PARAM_SEND_DELAY_MS = 30   # 每条之间隔一下，别把 H723 的接收缓冲挤爆
LASER_ENABLE = True        # 是否让 H723 点激光（AIM 模式下才有效）

DRAIN_OLD_FRAMES = True
DRAIN_MAX = 3              # 每次最多再丢几帧（防止极端情况下卡太久）
DRAIN_FAST_MS = 4          # "几乎是立刻拿到"的判据（ms）
DEBUG_PRINT_MS = 1000      # 终端打印周期
TRACE_EVERY = 0            # 0=关；逐帧写 SD 会明显拖慢主循环
LOG_TO_FILE = True         # 整机跑（没接电脑）时把日志写进 SD 卡，跑完插回电脑看
LOG_PATH = "/sdcard/k230_log.txt"
H723_TEXT_MIN_MS = 200     # H723 调试文本最快 200ms 一条（否则刷屏也占时间）
LOG_ONLY = False           # 只监听模式（云台单独测试用）：
                           #   True  = 只收 H723 日志并写进上面的日志文件，**不发
                           #           SET_ZERO / MODE(AIM) / AIM 帧**，H723 会一直停在
                           #           上电默认的 STAB —— 等价于"没接 K230"的独立测试；
                           #   False = 正常瞄准（改完这一个字即可，不用换文件）。

# ============================================================================
#  协议
# ============================================================================
SOF = b"\xAA\x55"

MSG_AIM = 0x10
MSG_MODE = 0x11
MSG_HEARTBEAT = 0x12
MSG_SET_ZERO = 0x13
MSG_PARAM = 0x15            # 运行时就地调参（H723 侧不用重新烧固件）
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


_log_f = None
_log_n = 0


def log(msg):
    """同时输出到终端和 SD 卡；每 8 条 flush 一次，避免 SD 写入拖慢视觉环。"""
    global _log_n
    print(msg)
    if _log_f is not None:
        try:
            _log_f.write(msg + "\n")
            _log_n += 1
            if (_log_n % 8) == 0:
                _log_f.flush()
        except Exception:
            pass


_trace_n = 0


def flog(msg):
    """只写 SD 卡、不 print（高速示踪用：40Hz 下 print 会把 USB 终端拖死）。"""
    global _trace_n
    if _log_f is None:
        return
    try:
        _log_f.write(msg + "\n")
        _trace_n += 1
        if (_trace_n % 40) == 0:
            _log_f.flush()
    except Exception:
        pass


def einfo(e):
    """异常信息带类型 —— 这版固件上 Python 异常的字符串经常只有一个数字
    （现场被 `叠加层异常(0)` / `二值预览失败(0)` 坑过好几轮，其实就是
    KeyError(0)）。带上类型，下次一眼就能看出是什么错。"""
    return "%s: %s" % (type(e).__name__, e)


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


def pack_aim_px(ut, vt, us, vs, flags, quality, k_x100):
    """AIM payload：只发像素坐标，偏置由 H723 算。
    ut/vt = 靶心像素，us/vs = 激光光斑像素（u16 小端）；flags/quality 原样。
    末尾多 2 字节 k_x100 = 像素/度 ×100：分辨率一换，px/° 跟着变，
    H723 收到就用这个值（没收到 = 旧固件，用默认 7.8）。"""
    import ustruct
    return ustruct.pack("<HHHHBBh",
                        max(0, min(IMG_W - 1, int(ut))),
                        max(0, min(IMG_H - 1, int(vt))),
                        max(0, min(IMG_W - 1, int(us))),
                        max(0, min(IMG_H - 1, int(vs))),
                        flags & 0xFF,
                        max(0, min(255, int(quality))),
                        max(-32768, min(32767, int(round(k_x100)))))


def pack_mode(mode, arg=0):
    import ustruct
    return ustruct.pack("<BB", mode & 0xFF, arg & 0xFF)


def pack_param(param_id, value):
    """GP_MSG_PARAM：param_id(u8) + value(i16 小端)，value = 真实值 × 100。"""
    import ustruct
    return ustruct.pack("<Bh", param_id & 0xFF,
                        max(-32768, min(32767, int(round(value)))))


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


_PIXBUF = [0, 0, 0]
_PIX_FAST = False      # get_pixel(x,y,buf) 这种"就地填"写法能不能用（见下）
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


_scene_cache = [0, 0.0, 0.0]   # [帧计数, 上一次算出的阈值, 预留]


def scene_th(img, n_frame=0):
    """按直方图用大津法决定白纸的 LAB-L 阈值（0~100）。

    优先设备 Otsu；结果不可用或不在 [38,62] 时用采样 Otsu；
    仍不可用则回退“画面均值 + 8”并夹到 [PAPER_TH,72]。
    每 10 帧重算一次，避免逐帧抖动。
    """
    if PAPER_TH_FIXED > 0:          # 手动固定阈值（调试/现场离线标定用）
        _scene_cache[1] = float(PAPER_TH_FIXED)
        _scene_cache[2] = 4.0       # 4 = 手动固定（日志里打 "(固定)"）
        return int(PAPER_TH_FIXED)
    if (n_frame % 10) != 0 and _scene_cache[1] > 0:
        return int(_scene_cache[1])
    t = -1.0
    src = 3.0                      # 1=设备Otsu  2=自算Otsu  3=回退老办法
    try:
        # CanMV/OpenMV：直方图 + Otsu 阈值（C 实现，比 Python 逐点快得多）
        thr = img.get_histogram().get_threshold().value()      # 0~255
        t = float(thr) * 100.0 / 255.0
        src = 1.0
    except Exception:
        t = -1.0
    if (t < 38.0) or (t > 62.0):
        # 设备直方图不可用/离谱 -> 自己算 Otsu（保证"确定性二值化"一定在跑）
        t = _otsu_from_samples(img)
        if (t >= 38.0) and (t <= 62.0):
            src = 2.0
    if (t < 38.0) or (t > 62.0):
        # ---- 兜底：老办法"30 点均值 + 8，夹 [PAPER_TH, 72]" ----
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
        # 双峰之间若有很宽的空隙，类间方差在空隙里是"平的"——直接取第一个
        #   最大值会落到空隙最左边（等于没切）。这里把得分相同（±0.2%）的
        #   平台段整段记下来，最后取中点（真实图像的直方图连续，不受影响）。
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
                    continue                    # 有一块是"背景大块" -> 不拼
                nested = ((a[0] >= c[0]) and (a[1] >= c[1]) and
                          (a[0] + a[2] <= c[0] + c[2]) and
                          (a[1] + a[3] <= c[1] + c[3])) or \
                         ((c[0] >= a[0]) and (c[1] >= a[1]) and
                          (c[0] + c[2] <= a[0] + a[2]) and
                          (c[1] + c[3] <= a[1] + a[3]))
                if not nested:
                    continue
                if nested and ((la < lc * 0.75) or (lc < la * 0.75)):
                    # 嵌套但尺寸差很多：只有中心白盘并回外环这一种情况放行。
                    # 外环是有洞的形状（密度低），内盘与外环同心，且内盘
                    # 至少占外框的 3% —— 不是噪点。背景大块是实心的，密度高，
                    # 因此不会在这里把靶纸吞掉。
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
                    continue                    # 拼完不像靶纸 -> 不拼
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
    # 每条边的扫描半径 = 中心到亮块该边的距离 + QUAD_LIM_PAD（放太远会越过
    # 黑胶带摸到背景亮边，那一条边就会“跳”出去）
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
#  显示 / 相机
# ============================================================================
def init_camera():
    global GRAY_DIRECT
    from media.sensor import Sensor, CAM_CHN_ID_0
    sensor = Sensor(id=2, width=1280, height=960, fps=90)
    sensor.reset()
    time.sleep_ms(100)
    apply_flip(sensor)
    sensor.set_framesize(width=IMG_W, height=IMG_H, chn=CAM_CHN_ID_0)
    if GRAY_DIRECT:
        try:
            sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_0)
        except Exception as e:
            GRAY_DIRECT = False
            log("灰度直出不可用(%s) -> 回退RGB565" % einfo(e))
            sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
    else:
        sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
    apply_flip(sensor)
    if EXPO_FIX:
        # 固定曝光（关自动），和 tools/vision_check.py 里那个开关是一回事：
        # 画面整体偏亮、纸面和白墙挤在同一个亮度带时，固定曝光能把对比度拉回来。
        try:
            sensor.set_auto_exposure(False, exposure_us=EXPO_US)
            log("曝光: 固定 exposure_us=%d（自动曝光已关）" % EXPO_US)
        except Exception as e:
            log("固定曝光不支持(%s)，继续自动曝光" % e)
    return sensor


def apply_flip(sensor):
    """上下/左右翻转尽量多试几种写法（不同固件的 set_vflip 签名/是否分通道不同，
    现场实测 set_vflip(True) 有时不生效）。若最后还是倒的，就把摄像头模块整体
    转 180° 装，并把 CAM_VFLIP 改回 False。"""
    for kw in ({}, {"chn": 0}, {"chn": 1}):
        try:
            sensor.set_vflip(CAM_VFLIP, **kw)
        except Exception:
            pass
        try:
            sensor.set_hmirror(CAM_HMIRROR, **kw)
        except Exception:
            pass


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
    from media.media import MediaManager
    MediaManager.init()
    sensor.run()
    if not GRAY_DIRECT:
        try:
            print("取像素: %s" % ("就地填（省分配）"
                                  if px_probe_fast(sensor.snapshot())
                                  else "普通模式"))
        except Exception:
            pass
    else:
        print("取像素: 灰度直出（单通道）")

    if USE_CLOSE_DETECTOR:
        target_det = ClosePaperDetector()
        _scene_cache[1] = float(CLOSE_BIN_TH)
        _scene_cache[2] = 4.0
    else:
        target_det = PaperDetector()
    spot_det = SpotDetector()
    vis_ut = 0                 # 靶心像素（最近一次有效帧）
    vis_vt = 0
    vis_us = 0                 # 激光光斑像素
    vis_vs = 0
    vis_q = 0                  # 质量分（用对比度归一化，0~255）
    vis_run = 0                # 连续"检测到目标+光斑"的帧数
    vis_ok = False             # 本帧是否有效（>=2 连帧才算）
    state = ST_WAIT_READY
    state_t = time.ticks_ms()
    gz = None
    last_gz_t = 0
    t_hb = time.ticks_ms()
    t_aim = time.ticks_ms()
    t_mode = time.ticks_ms()
    t_print = time.ticks_ms()
    t_diag = time.ticks_ms()
    last_tgt = None
    last_tgt_t = 0
    last_spot = None
    n_frames = 0
    n_frames_prev = 0
    n_hit = 0
    n_spot = 0
    sent_mode = None
    # 分段耗时统计（给 [TRACK] 行打印用，用来盯"实时性"有没有变差）
    sum_grab = 0
    sum_tgt = 0
    sum_spot = 0
    sum_drop = 0               # 低延迟取图：平均每次丢了几帧（0 = 队列本来就浅）
    sum_n = 0
    t_h723 = 0
    # 激光坐标开机自标定（见文件顶部 AUTO_CALIB_LASER 说明）
    global LASER_UV            # 自标定会重绑这个标定常数
    calib_n = 0
    cal_u = []
    cal_v = []
    laser_calibrated = False
    if GRAY_DIRECT:
        laser_calibrated = True
    spot_det.u, spot_det.v = LASER_UV[0], LASER_UV[1]   # 搜索窗先对准标称值
    spot_det.learned = True                             # 标定期间别让它自己漂

    print("=" * 60)
    print("K230 瞄准程序启动  %dx%d (S=%.2f K=%.2fpx/°)  靶纸=白纸亮块+暗框"
          % (IMG_W, IMG_H, S_SCALE, VIS_K))
    log("[CFG] 检测=%s 显示=无 逐帧SD示踪=%s"
        % ("灰度直出+连续白块/暗框" if GRAY_DIRECT
           else "RGB+连续白块/暗框",
           "开" if TRACE_EVERY > 0 else "关"))
    print("=" * 60)
    global _log_f
    if LOG_TO_FILE:
        try:
            _log_f = open(LOG_PATH, "w")
            log("(日志同时写入 %s；整机跑完把 SD 卡插回电脑看这个文件)" % LOG_PATH)
            log("[CFG] 新架构：K230 只上报像素坐标(ut,vt,us,vs)；"
                "偏置计算在 H723 500Hz 视觉外环（K=7.8px/°、限幅 yaw±120°/pitch±20°）"
                "；激光坐标固定=(%d,%d)" % (LASER_UV[0], LASER_UV[1]))
            log("[CFG] 处理分辨率 %dx%d (S=%.2f) -> 随帧下发 K=%.2fpx/°"
                "（H723 侧没收到 K 就用默认 7.8）"
                % (IMG_W, IMG_H, S_SCALE, VIS_K))
        except Exception as e:
            print("日志文件打不开(继续运行, 只是不写文件): %s" % e)

    try:
        while True:
            os.exitpoint()
            now = time.ticks_ms()
            err_u = 0.0            # 本地显示/日志用（偏置计算在 H723）
            err_v = 0.0

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
                            _txt = payload.decode("utf-8")
                        except Exception:
                            _txt = ""
                        if _txt:
                            _key = ("[MOTOR]" in _txt) or ("[WARN]" in _txt) or \
                                   ("[PARAM]" in _txt) or ("[MOTO]" in _txt) or \
                                   ("[CTRL]" in _txt) or ("[RAW]" in _txt)
                            if _key or \
                               (time.ticks_diff(now, t_h723) >= H723_TEXT_MIN_MS):
                                t_h723 = now
                                log("H723: %s" % _txt)
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

            ready = bool(gz is not None and (gz["flags"] & ST_READY))

            # ---------- 4. 状态机 ----------
            if state == ST_WAIT_READY:
                if ready:
                    for _pid, _val in PARAM_SEND:
                        link.send(build_frame(MSG_PARAM,
                                              pack_param(_pid, _val)))
                        time.sleep_ms(PARAM_SEND_DELAY_MS)
                    if PARAM_SEND:
                        log("已下发 %d 条现场参数（看下面的 H723: [PARAM] 回执）"
                            % len(PARAM_SEND))
                if ready and LOG_ONLY:
                    # 只监听模式：不切模式、不发 SET_ZERO，H723 保持 STAB。
                    # 电压/日志照收，SD 卡日志照写。
                    state = ST_TRACK
                    state_t = now
                    log("LOG_ONLY=1 只监听：保持 H723 的 STAB，不发送 SET_ZERO/AIM")
                elif ready:
                    link.send(build_frame(MSG_MODE, pack_mode(MODE_STAB)))
                    sent_mode = MODE_STAB
                    link.send(build_frame(MSG_SET_ZERO))
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
                if (not LOG_ONLY) and \
                        (time.ticks_diff(now, t_mode) >= MODE_REASSERT_MS):
                    t_mode = now
                    link.send(build_frame(MSG_MODE, pack_mode(MODE_AIM)))

                t0 = time.ticks_ms()
                img = sensor.snapshot()
                t_grab = time.ticks_diff(time.ticks_ms(), t0)
                if DRAIN_OLD_FRAMES:
                    # 只处理"最新"那一帧（见文件顶部 DRAIN_OLD_FRAMES 说明）：
                    # 拿到手太快 = 队列里还有旧帧 -> 再丢一帧。
                    _drop = 0
                    while (_drop < DRAIN_MAX) and (t_grab < DRAIN_FAST_MS):
                        t0 = time.ticks_ms()
                        img = sensor.snapshot()
                        t_grab = time.ticks_diff(time.ticks_ms(), t0)
                        _drop += 1
                    sum_drop += _drop
                n_frames += 1
                if n_frames == 1:
                    # 分辨率自检：ISP 没给到请求的尺寸时，下面所有"像素"判据
                    # 都会按错的比例工作（缩放的常数 + 实际的图对不上）。
                    # 一行日志就能排掉这种问题，别再靠猜。
                    try:
                        _aw, _ah = img.width(), img.height()
                    except Exception:
                        _aw, _ah = IMG_W, IMG_H
                    log("[CAM] 实际输出 %dx%d（请求 %dx%d，S=%.2f K=%.2f）"
                        % (_aw, _ah, IMG_W, IMG_H, S_SCALE, VIS_K))

                # 每 5 秒打一次画面亮度：无靶时用它判断"是没看到靶还是判据问题"
                if (target_det.lost > 0) and \
                        (time.ticks_diff(now, t_diag) >= 5000):
                    t_diag = now
                    log_scene(img)

                t0 = time.ticks_ms()
                res = target_det.detect(img)
                t_tgt = time.ticks_diff(time.ticks_ms(), t0)
                t0 = time.ticks_ms()
                if (not USE_FIXED_LASER) and ((n_frames % SPOT_EVERY) == 0):
                    last_spot = spot_det.detect(img)
                t_spot = time.ticks_diff(time.ticks_ms(), t0)
                spot = last_spot
                # ---- 激光坐标"开机自标定"（见文件顶部 AUTO_CALIB_LASER 说明）----
                if AUTO_CALIB_LASER and (not GRAY_DIRECT) and \
                        (not laser_calibrated):
                    if calib_n < CALIB_FRAMES:
                        _d = spot_det.detect(img)
                        if _d is not None:
                            cal_u.append(_d["uv"][0])
                            cal_v.append(_d["uv"][1])
                        calib_n += 1
                    else:
                        laser_calibrated = True
                        _n0 = len(cal_u)
                        if _n0 >= CALIB_MIN_N:
                            _su = sorted(cal_u)[_n0 // 2]
                            _sv = sorted(cal_v)[_n0 // 2]
                            _du = sorted([abs(a - _su) for a in cal_u])[_n0 // 2]
                            _dv = sorted([abs(a - _sv) for a in cal_v])[_n0 // 2]
                            _spread = _du if _du > _dv else _dv
                            if (abs(_su - LASER_UV[0]) > CALIB_SHIFT_MAX) or \
                                    (abs(_sv - LASER_UV[1]) > CALIB_SHIFT_MAX):
                                log("[CALIB] 光斑中位=(%.1f,%.1f) 离标称太远 -> "
                                    "保持 (%d,%d)（检错了吧？）"
                                    % (_su, _sv, LASER_UV[0], LASER_UV[1]))
                            elif _spread > CALIB_SPREAD_MAX:
                                log("[CALIB] 光斑散布 %.1fpx 太大 -> 保持 (%d,%d)"
                                    % (_spread, LASER_UV[0], LASER_UV[1]))
                            else:
                                _old = LASER_UV
                                LASER_UV = (int(_su + 0.5), int(_sv + 0.5))
                                log("[CALIB] 激光坐标自标定: (%d,%d)->(%d,%d) "
                                    "n=%d 散布%.1fpx"
                                    % (_old[0], _old[1],
                                       LASER_UV[0], LASER_UV[1],
                                       _n0, _spread))
                        else:
                            log("[CALIB] 只检出 %d 帧光斑（< %d）-> 保持 (%d,%d)"
                                % (_n0, CALIB_MIN_N,
                                   LASER_UV[0], LASER_UV[1]))
                sum_grab += t_grab
                sum_tgt += t_tgt
                sum_spot += t_spot
                sum_n += 1

                # 丢靶滑行：短时间内沿用最后一次目标位置（只用于本地显示/日志）
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

                if USE_FIXED_LASER:
                    # 固定激光坐标（标定常数）：不再检测光斑，少一层不确定
                    spot = {"uv": LASER_UV}
                if spot is not None:
                    n_spot += 1
                    su, sv = spot["uv"]
                else:
                    su, sv = spot_det.u, spot_det.v
                SPOT_UV[0], SPOT_UV[1] = su, sv

                if (res is not None) and (spot is not None) and \
                        (target_det.lost == 0):
                    if vis_run < 5:
                        vis_run += 1
                    vis_ut, vis_vt = res["center"]
                    vis_us, vis_vs = su, sv
                    vis_q = max(0, min(255, int(res.get("contrast", 0))))
                else:
                    vis_run = 0
                vis_ok = (vis_run >= 2)

                # 本地显示/日志用的像素误差（真正的偏置计算在 H723）
                err_u = (tgt_uv[0] - su) if tgt_uv is not None else 0.0
                err_v = (tgt_uv[1] - sv) if tgt_uv is not None else 0.0
            # ---------- 5. 发 AIM（100Hz）：只发像素坐标（新架构）----------
            if (not LOG_ONLY) and (state in (ST_SET_ZERO, ST_TRACK)) and \
                    time.ticks_diff(now, t_aim) >= (1000 // AIM_HZ):
                t_aim = now
                flags = 0
                if state == ST_TRACK and vis_ok:
                    flags |= FLAG_AIM_VALID | FLAG_LOCKED
                if state == ST_TRACK and LASER_ENABLE:
                    flags |= FLAG_LASER_ON
                link.send(build_frame(MSG_AIM,
                                      pack_aim_px(vis_ut, vis_vt,
                                                  vis_us, vis_vs,
                                                  flags, vis_q,
                                                  VIS_K * 100.0)))
            # ---------- 6. 终端打印 ----------
            # ---- 高速示踪（见 TRACE_EVERY 说明）：每 N 帧一条，只写文件 ----
            if (TRACE_EVERY > 0) and ((n_frames % TRACE_EVERY) == 0):
                if gz is not None:
                    flog("T %d e=(%.0f,%.0f) yaw=%.1f pit=%.1f gz=%.1f "
                         "ym=%.1f ok=%d t=%d"
                         % (n_frames, err_u, err_v, gz["yaw"], gz["pitch"],
                            gz["gyro_z"], gz["ymotor"],
                            1 if vis_ok else 0, int(_scene_cache[1])))
                else:
                    flog("T %d e=(%.0f,%.0f) 等遥测"
                         % (n_frames, err_u, err_v))
            if time.ticks_diff(now, t_print) >= DEBUG_PRINT_MS:
                fps = ((n_frames - n_frames_prev) * 1000.0 /
                       max(1, time.ticks_diff(now, t_print)))
                n_frames_prev = n_frames
                t_print = now
                tgt_s = "无靶"
                if target_det.meas is not None and target_det.lost == 0:
                    _m = target_det.meas
                    tgt_s = "%s %.2fm 长%dpx 比%.2f 密%.2f 对比%d" % (
                        target_det.state,
                        FX_PX * PAPER_LONG_M / max(1.0, _m[5]),
                        _m[5], _m[6], _m[7], _m[8] - _m[9])
                else:
                    # 没锁定时把"为什么没过闸门"一起写进日志（下次复盘要用）
                    tgt_s = "无靶[%s 候选%d %s]" % (
                        target_det.state, target_det.last_n,
                        target_det.last_dbg)
                if gz is not None:
                    log("[%s] %.1ffps th=%d gz(state=%d fault=%d flags=0x%02X "
                          "yaw=%.1f pit=%.1f roll=%.1f ymot=%.1f pmot=%.1f "
                          "gz=%.1f/%.1f up=%dms) 靶=%s 命中%d/%d 光斑%d "
                          "上报 u=(%d,%d) s=(%d,%d) q=%d ok=%d "
                          "像素差=(%.0f,%.0f) "
                          "ms 取图%.0f 目标%.0f 光斑%.0f 丢帧%.1f"
                          % (ST_NAME[state], fps, int(_scene_cache[1]),
                             gz["state"], gz["fault"],
                             gz["flags"], gz["yaw"], gz["pitch"], gz["roll"],
                             gz["ymotor"], gz["pmotor"], gz["gyro_y"],
                             gz["gyro_z"], gz["up_ms"], tgt_s,
                             n_hit, n_frames, n_spot,
                             vis_ut, vis_vt, vis_us, vis_vs, vis_q,
                             1 if vis_ok else 0, err_u, err_v,
                             sum_grab / float(max(1, sum_n)),
                             sum_tgt / float(max(1, sum_n)),
                             sum_spot / float(max(1, sum_n)),
                             sum_drop / float(max(1, sum_n))))
                else:
                    log("[%s] 等 H723 遥测... 靶=%s ok=%d crc=%d"
                        % (ST_NAME[state], tgt_s,
                           parser.ok, parser.crc_err))
                sum_grab = 0
                sum_tgt = 0
                sum_spot = 0
                sum_drop = 0
                sum_n = 0
            # gc.collect() 会卡 10~30ms：从"每 30 帧"放到"每 240 帧"（≈7s）。
            # MicroPython 在内存不够时会自己回收，这里只是兜底。
            if (n_frames % 240) == 0:
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
        # 日志兜底刷盘（log() 现在每 8 条才 flush 一次，见其说明）
        if _log_f is not None:
            try:
                _log_f.flush()
            except Exception:
                pass
        try:
            MediaManager.deinit()
        except Exception:
            pass
        print("已退出")


main()
