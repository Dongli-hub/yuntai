# -*- coding: utf-8 -*-
"""vision_check.py —— 视觉验证工具（视觉处理部分与 k230_aim/main.py 逐行一致）

在 CanMV IDE 里直接运行：画面显示在 IDE 右上角的『帧缓冲区』，终端看数据。
⚠ 预览画布固定 640x480（**这块板子的显示只支持这个尺寸**，见下面 DISPLAY_W
  处的说明）：分辨率设成 800x600 时，IDE 里看到的是"缩放后"的画面 —— 小一号，
  但绿框/红十/黄圈的位置关系完全一样，识别判据用的也是原图。

**纯视觉**：不开串口、不发任何指令给 H723（整机联调请用 main.py）。
本文件 = "能正常出画面的那版 vision_check.py"（显示/相机/主循环骨架原样保留）
        + 当前 main.py 的视觉处理（配置、检测函数、PaperDetector、SpotDetector 原文）。
所以：这里看到什么识别效果，整机 main.py 里就是什么效果。
（两边由 tools/sync_check.py 自动比对，改完一边记得同步另一边。）

启动先跑一段"纯取图测速"（1.5 秒），把两个上限分开：
    [PROBE] 纯取图 = 相机+ISP 能给多少帧；下面每行的 FPS = 端到端实际帧率。
    两者的差距 = 被我们的处理吃掉的。改代码前先看这两个数，别对着采集上限较劲。

画面标注：
    绿四边形 = 贴着黑胶带内沿的靶纸四边形（斜视时是梯形）
    红十     = 靶心 = 四边形两条对角线的交点（透视不变的中心）
    黄圈     = 激光光斑（固定坐标 = 标定常数 LASER_UV）

终端每 0.5 秒一行：
    FPS=31.2 靶=锁定 距离≈0.72m 靶心=(400,300) 长边=278px 比1.41 密0.93 对比112
             | 光斑=(390,315) 误差=(1,24)px=24.0px(2.5°)
             | ms 取图5 目标15 光斑3 显示3 | 靶33/35 斑32
    没锁定时会打出候选详情 [xxxpx 框xxx 内xx 外xx 暗边xx%]，
    丢靶久了再打一行 [SCENE] 画面亮度 —— 用来判断"没看到靶"还是"判据没过"。
"""
import os
import time
import math

# ============================ 画面 / 显示 ============================
# --- 相机 / 处理分辨率 ---
# 只改 PROC_W / PROC_H 这两个数（其余像素常数按 S_SCALE 自动缩放）。
#   640x480 = 当前值：预览 1:1 最清楚（800x600 那轮实测：全图搜索慢 1.5 倍、
#   预览还得缩一次更糊，识别并不更准）。
# PROC_W 必须是 16 的倍数（ISP 只按 16 对齐）。整机跑 800 档时 H723 要重烧 K。
PROC_W = 640
PROC_H = 480
IMG_W = PROC_W
IMG_H = PROC_H
S_SCALE = PROC_W / 640.0    # 像素常数缩放系数（640 档 = 1.0，800 档 = 1.25）
VIS_K = 7.8 * S_SCALE       # px/°（现场标定 7.8 @640x480，随分辨率同比缩放）
# --- 显示画布 ---
# ⚠ 2026-10-11 实测：这块板子的显示**只吃 640x480**。800x600 时
#   Display.init 直接报 "ST7701 unsupport 800x600"，VIRT 那条路在这版固件上
#   也不出画面（就是"跑了但看不到"）。所以画布固定 640x480，
#   处理图像用 draw_image(x_scale/y_scale) 缩着画上去 ——
#   识别/上报用的仍然是 PROC_W×PROC_H 的原图，只有"给你看的那一份"被缩了。
DISPLAY_W = 640
DISPLAY_H = 480
DISPLAY_QUALITY = 40       # 越小传输越快（画面撕裂跟传输速度直接相关）
# --- 二值化预览（2026-10-11 用户要求："直接让画面显示二值化时候的画面"）---
# True  = 推给 IDE 的是**二值化之后**的画面，也就是 find_blobs 真正看到的那张：
#           白 = 过阈值（"亮块"的原料）  黑 = 没过阈值
#         叠加：橙框 = 被闸门拒掉的候选，绿框 = 采纳的靶纸四边形/外接框，
#               红十字 = 靶心，黄圈 = 激光。左上角写着当前阈值和来源。
#         调试"为什么认不到靶"就看它：如果靶纸整片是黑的 -> 阈值太高；
#         如果纸和白墙糊成一大片白 -> 阈值太低；理想是"只有靶纸那一块白、
#         四周（黑胶带/木头）是黑的"。
# False = 原来的彩色画面（带同样的叠加）。看颜色/曝光时用。
BIN_PREVIEW = True
# --- 灰度预览（默认关；只在定位"阈值该放哪"时才临时打开）---
# False = 推给 IDE 的是**检测图**（灰度 -> 高斯 -> 二值 -> 开/腐 -> Canny 融合），
#         也就是识别真正在判别的画面（2026-10-15 用户："以后我看的都是二值化
#         之后的识别图像，不看灰度的"）。
# True  = 只灰度化，不二值化 —— 想量"纸面/胶带/木柜各多亮"时用。
GRAY_PREVIEW = False
# --- 预览里画不画"被拒候选框"（2026-10-14 用户："上面还有许多杂乱的框"）---
# False = 只画：绿色四边形/方框（采纳的靶）、红十字（靶心）、黄圈（激光）
# True  = 连被闸门拒掉的橙色候选框也画出来（查"为什么没锁上"时用；
#         大块候选动辄 640px 宽，画面会很乱）
# 候选的详细数据不受影响 —— 终端每 0.5 秒的 "候选N [xxxpx …✗原因]" 那行照打。
PREVIEW_CANDS = False
# --- 曝光 ---
# 2026-10-15 晚：**找到关自动曝光(AE)的真开关了**（把固件镜像 + 官方源码翻完）：
#   · `sensor.set_auto_exposure / set_auto_gain / get_exposure_us` 在这版固件里
#     是**空壳**（media/sensor.py 里就是 `pass` + `@wrap` 打印 "not support xxx
#     now..."）—— 所以以前怎么调都没用，不是写法问题；
#   · 真正生效的是 VICAP 的 dev attr：`Sensor.run()` 里会执行
#         kd_mpi_vicap_set_dev_attr(self._dev_id, self._dev_attr)
#     把 `pipe_ctrl.bits.ae_enable` 下发给 ISP。新版固件(v1.8+，官方例程
#     17-Sensor/camera_manual_exposure_lcd.py)的 `sensor.auto_exposure(False)`
#     改的就是这一位 —— 我们在 run() 之前直接清它（见本文件相机初始化那段）。
#   ⚠⚠ 2026-10-15 现场实测（这条最重要）：**不能"run() 之前就关 AE"** ——
#     AE 一关，曝光就停在传感器出厂默认值上，画面直接全黑（实测 均=32 大=57，
#     二值化后一片黑，候选0）。而且曝光时间写不进去（底层
#     kd_mpi_sensor_intg_time_set 没暴露给 Python）。
#   ⇒ 正确做法（apply_exposure 实现）：**先让 AE 正常跑 1.2 秒把画面调好，
#     再运行时下发 ae_enable=0 把当时的曝光/增益冻住**；冻完复测亮度，
#     万一明显变暗就自动把 AE 开回来（宁可不锁，也不能全黑）。
EXPO_FIX = True
EXPO_SETTLE_MS = 1200      # run() 后先让 AE 自适应多久再冻结
EXPO_DARK_RATIO = 0.55     # 冻结后亮度 < 冻结前×这个比例 -> 判定"变暗了"，回滚
# 冻住之后还能用**模拟增益**微调亮度（资料 06.1/5、6、9 的 sensor.again 写法：
#   g = sensor.again(); g.gain[0] = 值; sensor.again(g)）。
#   这就是"把曝光调低一点"的旋钮 —— 曝光时间写不进去，增益是唯一能压亮度的：
#     1.0 = 不动；0.75 = 压到 3/4（画面变暗、反光/高光收敛）；1.3 = 提亮。
#   现场调法：看 [SCENE] 均= 和二值图里那圈黑胶带有没有断口 ——
#   断口 = 打光太亮（局部过曝）把胶带"洗"到阈值之上，**往下压增益**就能把它
#   压回"黑"（阈值是绝对值，所以压暗对局部过曝是有效的）；纸会一起变暗，
#   只要还明显亮于阈值就行。太暗到看不清纸就往上回一点（0.9 / 1.0）。
# ⚠ 2026-10-15 现场实拍（最近那条线上底边被打光洗白 -> 圈断 -> 识别不出来）：
#   0.75 不够，压到 **0.6** 再试。
EXPO_GAIN_SCALE = 0.6
SHOW_EVERY = 2             # （整机用的节拍；本工具看 PREVIEW_EVERY，见下）
# 本工具**每帧都往 IDE 推**（2026-10-16 用户："画面还是有卡顿、不流畅"）：
#   推画面间隔才是"眼睛看到的帧率"—— 处理 85ms/帧时，每 2 帧推一次 = 5.9fps
#   观看帧率，每帧都推 = 8~9fps（多花的是 show_image 那次传输 ~10~30ms）。
#   workbench 里想省时间就把它改回 2。
#   （main.py 不用这个：整机推画面只给调试看，SHOW_EVERY=2 不占它的时间。）
PREVIEW_EVERY = 1
CAM_VFLIP = True           # 画面上下颠倒 -> True
CAM_HMIRROR = False        # 画面左右镜像 -> True
# --- cv_lite（资料《06.1 CV_lite图像处理专题》，板载 C 实现）---
#   ① grayscale_find_rectangles_with_corners(...)：**一次调用直接给出
#      "矩形框 + 4 个角点"**（例程 06.1/5 和 /6）——参考代码那套
#      threshold -> Canny -> findContours -> approxPolyDP 就是在 C 里跑，
#      我们拿它当"四边形"的来源（见 _cv_quad_for），比沿扫描线逐像素摸边
#      快得多也稳得多；
#   ② grayscale_threshold_binary(...)：快速二值化（例程 06.1/13）。
#   ⚠ 用 grayscale_* 要求 sensor 出图是**灰度**（例程 5/9/13 都是
#     `Sensor.GRAYSCALE` 这么配的），所以 CVLITE_ON=True 时本文件相机初始化
#     会把出图格式切成 GRAYSCALE。
#   匹配不上、或 cv_lite 不可用时**自动退回**原来的扫描拟合，行为不变。
CVLITE_ON = True
CV2_EVERY = 1              # 每几帧跑一次检测（1=每帧）。
# --- 「国一那套」的参数：**逐个对着国一原文填**（2026-10-16）---
GY_BLUR = 3                # ① GaussianBlur 核（国一 GAUSSIAN_BLUR_SIZE=3）
# ③ 反向二值化阈值（luma 0~255，**白 = 比它暗的像素**）。
#    ⚠⚠ 方向别再搞反了（2026-10-16 现场复盘，用户："再把阈值调低，
#       显示不出完整的矩形白框"）：这一条里 "白" = 暗于阈值 —— 想要白的**更多**
#       （比如让被照亮的胶带那几段也变白、白框连成整圈），是把这个数**往上加**；
#       往下减 = 白的更少 = 框更碎。60 那档只剩胶带在阴影里的上半圈是白的。
#    ✅ 2026-10-16 现场定稿：**72**（四轮轮询：10~80 粗挑 -> 65~70 -> 60~65
#       -> 65~75 扫完 71~75 之后，用户定在 72）。
#       高阈值（80 以上）木柜被照亮的边角也跟着变白、糊成一片；低阈值
#       （60 附近）胶带被照亮的下边/右边那两段先断、框缺边。72 是两头都行的点。
#       （理论参考：现场 P25=88~101 是胶带那一档、P50=103~123 是木柜那一档，
#         纸面 200+；那只是"取值范围"，最终以现场画面为准。）
#    调法：木柜/背景也成片变白 -> 往下减一点；白框还有缺口 -> 往上加一点。
#    （参照：国一原文写 35，那是他们现场 0~255 灰度下的数，很暗；
#      我们的场景黑胶带没被压那么黑，所以不能照抄。）
#    GY_TH 只影响**预览那张融合图**，不影响识别结果（识别走 cv_lite 的
#    灰度 Canny+轮廓那一步）—— 所以放心调。
GY_TH = 72
# ③ 闭运算核：(2n+1)²，0 = 不做。
#    ⚠ 2026-10-16（用户："画面还是有卡顿"）：**改成 0** —— 这一步只作用在
#      "预览那张融合图"上（识别不用它，国一原文用它是为了在轮廓前把断口补齐），
#      而这版固件的 open/close 很贵：3×3 实测 **29ms**，占国一那条链（~75ms）
#      的四成。th=72 时白带本身已经很宽，close 只是把边沿各补 1~2px；
#      [MT] 表里也能看出开运算只改 0.5% 像素。要重新补断口就改回 1（+29ms/次）。
GY_CLOSE_SIZE = 0
# 锁定后"只在上一帧框周围找"的 ROI（cv_lite 耗时 ∝ 面积）：
#   ROI 边长 = 上一帧长边 × CROP_K + 2×CROP_MARGIN_PX（再加运动量）。
#   只在 ROI 面积 <= CROP_MAX_FRAC×全图 时才裁（否则省不了多少、还要多一次复制），
#   裁完找不到 / 这版 cv_lite 不吃裁剪缓冲 -> 自动退回全图（一次性日志），
#   绝不会因为省时间把识别搞死。0 = 不裁。
CROP_K = 1.6
CROP_MARGIN_PX = 60
CROP_MAX_FRAC = 0.55
# 预览融合图的分辨率除数（2026-10-16）：cv_lite/形态学的耗时**与像素数成正比**，
#   半分辨率（320x240）= 1/4 像素 = 快 ~4 倍，贴回画布时按 2 倍放大画，
#   二值图放大 2 倍肉眼几乎看不出差别。只在 vision_check 里生效
#   （整机 _WANT_MASK=False，根本不建这张预览图）。1 = 全分辨率。
MASK_SCALE = 2
GY_AREA_MIN_RATIO = 0.0003  # ⑥ cv_lite 的"最小面积"是**相对比例**：国一 100px²/307200
GY_MAX_COS = 0.3           # ⑥ 角度判据（cv_lite 参数；国一 ANGLE_TOLERANCE=30°）
GY_CANNY1 = 50             # ⑤ Canny(50, 150) —— 在**灰度图**上做
GY_CANNY2 = 150
GY_EPS = 0.02              # ⑧ approxPolyDP 精度 = 周长 ×0.02（国一原值）
GY_AREA_RATIO = 0.7        # ⑦ 嵌套面积比 ≥0.7（国一 MIN_AREA_RATIO=0.7）
# --- 激光（固定坐标）---
USE_FIXED_LASER = True
LASER_UV = (int(round(315 * S_SCALE)), int(round(246 * S_SCALE)))
LASER_TRIM = (-3, -7)      # (du, dv)：正值 = 往右/往下
PAPER_MIN_LONG = int(round(36 * S_SCALE))    # 长边下限 px（长度随 S 缩放）
PAPER_MAX_LONG = int(round(480 * S_SCALE))   # 长边上限 px
PAPER_ASPECT_MIN = 0.70    # A4=1.41；斜视透视下会缩到接近 1
PAPER_ASPECT_MAX = 2.00
PAPER_CONTRAST_MIN = 30    # 内亮度 - 外亮度（0~255 量程）
# 胶带**外面**要比紧贴着的那一圈（胶带）亮多少 —— "纸->黑圈->背景又亮起来"。
# 深色背景上的一块白，外面一直是深的，这道判据把它挡住（见 paper_contrast）。
PAPER_RING_RISE_MIN = 15
PAPER_DARK_MARGIN = 25     # 单个外侧采样点算“暗”的门槛
PAPER_DARK_FRAC_MIN = 0.55  # 外侧 12 点里至少这么多比例更暗
PROBE_OFF_PX = int(round(5 * S_SCALE))
LOST_FULL = 25
SMOOTH = 0.90              # 平滑系数（越大越跟手、越小越稳）
DEADBAND_PX = 1.5 * S_SCALE   # 平滑死区（小于它不动，画面不抖）
JUMP_REJECT_BASE = int(round(35 * S_SCALE))  # 时间一致性护栏的位移基值（见 _accept）
SMOOTH_FAR = 0.60          # 远距离平滑（0.30 -> 0.60：51ms -> 15ms）
FAR_LONG_PX = 120.0 * S_SCALE  # 长边小于它 = "远"（平滑/滤波/增益按它分档）
PAPER_LONG_M = 0.275       # 白色区域长边实际长度（米），用于估距离
SPOT_ROI_HALF = int(round(45 * S_SCALE))  # 光轴固定，只在学习到的点附近找光斑
SPOT_THRESHOLDS = [        # LAB 阈值，可多组
    (88, 100, -40, 90, -50, 90),   # 过曝白芯（纸面 LAB-L≈78 到不了）
    (58, 100, 25, 90, -20, 70),    # 明显红边（a>=25，原 a>=6 太松）
]
SPOT_MIN_AREA = 2
SPOT_MAX_AREA = 6000
SPOT_MAX_ASPECT = 3.0
SPOT_EVERY = 4             # 每几帧搜一次光斑（光轴固定，中间帧复用）
SPOT_ADAPT_GAIN = 0.06     # 光轴点慢速自适应（把误检拖跑的风险限制住）
SPOT_ADAPT_LIMIT = 6.0     # 单次最多修正多少像素
SPOT_PREF_PX = int(round(70 * S_SCALE))
SPOT_UV = [IMG_W / 2.0, IMG_H / 2.0]   # 最近一次找到的激光点，主循环每帧更新
_canvas = None
# --- 国一反向二值化阈值（GY_TH）的自动轮换（2026-10-16 加，默认关）---
#   为什么能用轮换挑：**GY_TH 只影响"预览那张融合图"**（_cv2_find ② 那一步），
#   矩形检测本身走的是 cv_lite 在灰度图上的 Canny+轮廓，与它不是同一个阈值 ——
#   所以换档不会改变识别结果，只是让你一眼看出"哪一档白框最完整"。
#   注意方向：这一条里"白 = 比阈值暗"，所以**数值越大、白的越多**（60 那档
#   只剩胶带的阴影段是白的，框是碎的；往上加才会连成整圈）。
#   2026-10-16 第一轮：10~80 每 10 一档 -> 用户选中 70。
#   2026-10-16 第二轮：65~70 每 1 一档细看（用户还想再往低处比）。
#   2026-10-16 第三轮：60~65 每 1 一档（往低走左侧墙上的白块少，代价是胶带
#     被照亮的下边/右边先断）。
#   2026-10-16 第四轮：65~75 每 1 一档（补齐 71~75）。
#   ✅ 定稿：用户在 65~75 这圈里选了 **72** —— 轮换关掉，预览固定 72。
#     换场地/灯光想重挑就把这里改回 True，并把下表的区间改成要扫的那一段。
GY_TH_SWEEP = False
GY_TH_SWEEP_SEC = 3
GY_TH_SWEEP_LIST = [65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75]
# --- 激光光斑标定（2026-10-11 用户："黄圈不准"）---
# 黄圈画的就是 LASER_UV —— 它是**标定常数**，整机瞄准时"靶心-光斑"里的光斑
# 就用它。所以这个数偏多少像素，激光最终就偏多少像素（K=7.8px/° 时 8px≈1°，
# 0.6m 处约 1cm）。数值不准只能靠标定解决，不能靠"每帧检测光斑"
# （历史证明那个会飘、会闪，才改成固定常数的）。
#   CALIB_LASER = True -> 每帧找光斑、黄圈跟着"检出来的"走（肉眼确认有没有
#     圈住激光），每 0.5 秒打印中位数，形如：
#         [标定] 光斑 n=137 中位=(316.0,257.5) 建议 LASER_UV=(316,258)
#     把这个数填到 LASER_UV（**main.py 里也要填同一个数**），再改回 False。
#   标定时尽量：激光开着、靶纸在 0.5~1m、画面别晃 —— 中位数抗单帧误检。
CALIB_LASER = False


# --- 距离（与 main.py 一致）---
FX_PX = 445.0 * S_SCALE    # 像素焦距（随分辨率同比缩放）。0.6m 处 A4 长边≈222px 反推

# ============================ 时序 ============================
# ⚠ 2026-10-11：本文件是**纯视觉**工具 —— 不和 H723 通信（不占串口、不发
#   SET_ZERO/AIM）。原来那一段串口协议（UART/CRC/FrameParser/pack_aim）是
#   新架构之前留下的，pack_aim 还是旧的"发角度"格式，留着只会误导；
#   按用户要求一并删除。整机联调请用 main.py。
PRINT_MS = 500
GC_EVERY = 30

def luma(r, g, b):
    return (r * 77 + g * 150 + b * 29) >> 8


def px_luma(img, x, y):
    """取某点亮度（0~255）。取不到返回 -1。

    ⚠⚠ 2026-10-14（形态学预处理）：**灰度图和彩色图都要能取** ——
    预处理之后检测是在**灰度图**上跑的，灰度图的 get_pixel 返回单个数值
    （或 1 元组），彩色图返回 (r,g,b)。以前直接 p[0]/p[1]/p[2]，
    在灰度图上会抛异常 -> 所有采样全灭。
    · 检测图是灰度时（_GRAY_IMG=True）直接用普通写法（"就地填 buf"的快写法
      只在彩色图上验过，灰度图不走它）；
    · 彩色图照旧走快路径。
    """
    if x < 0:
        x = 0
    elif x >= IMG_W:
        x = IMG_W - 1
    if y < 0:
        y = 0
    elif y >= IMG_H:
        y = IMG_H - 1
    p = None
    # 2026-10-16：sensor 出图就是灰度（见 init_camera），取像素只有这一条路
    # （"就地填 buf"的快路径只对彩色图验过，已删）。
    try:
        p = img.get_pixel(int(x), int(y))
    except Exception:
        return -1
        if p is None:
            return -1
    # 彩色图 = (r,g,b)；灰度图 = 单个数或 1 元组 —— 两种都收
    try:
        n = len(p)
    except TypeError:
        return int(p)
    if n >= 3:
        return luma(p[0], p[1], p[2])
    if n == 1:
        return int(p[0])
    return -1
# 分段耗时探针（回答"这一帧的时间到底花在哪"，别再靠猜）：
#   [0] 预留            [1] find_blobs 累计 ms     [2] 预留     [3] find_blobs 次数
#   [4] 掩码链累计 ms   [5] 掩码链次数             [6] _find 累计 ms  [7] _find 次数
#   [8] 灰度化 ms       [9] 模糊+二值+形态 ms      [10] Canny 融合 ms [11] 还原灰度 ms
#   [12] cv_lite 找矩形累计 ms   [13] cv_lite 次数
#   [14] cv2 国一那套 累计 ms    [15] cv2 次数
# 打印见 prof_line()（[TRACK] 旁边那行 [PERF]）。用户报"卡顿"时先看它。
_PROF = [0.0, 0.0, 0.0, 0, 0.0, 0, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0]


def prof_line():
    """分段探针拼成一行（只读，不清零）。"""
    n2 = _PROF[15] if _PROF[15] > 0 else 1
    return ("国一%.0fms/次x%d[建图%.0f 矩形%.0f 喂像素%.0f万px]"
            % (_PROF[14] / float(n2), _PROF[15],
               _PROF[0] / float(n2), _PROF[2] / float(n2),
               _PROF[1] / float(n2) / 10000.0))


def prof_reset():
    for _i in range(len(_PROF)):
        _PROF[_i] = 0
_LAST_MASK = None


# ============================================================================
#  「国一那套」矩形检测 —— 用**板上真正存在的 OpenCV 接口**实现
#  （2026-10-16 第三次修正：现场日志 "no module named 'cv2'" 证明这颗固件
#    MicroPython 侧没有 cv2 模块；但镜像里带着完整 OpenCV C++（OpenCV-V1.0.3/
#    C908，cv::findContours / cv::warpPerspective / cv::getPerspectiveTransform
#    都在），对外暴露的绑定是 **cv_lite**（它的错误信息就写作 "cv2::xxx"）。
#    所以下面每一步都用 cv_lite / image 模块实现，等价于国一原文那 7 步。）
# ============================================================================
_GY_ERR = [0]
_CROP_ERR = [0]            # ROI 裁剪本身报错次数（第 1 次打日志；-1 = 已停用）
_CROP_MISS = [0]           # ROI 里连续没找到几次（连续 3 次就永久退回全图）
# 要不要建"预览那张融合图"（反向二值化 ∪ Canny）。
#   ⚠ 整机 main.py **不需要**（它推画面推的是原始帧，没人看掩码），
#     所以这里默认关 —— 省 40~60ms/次（国一那条链的大头）。
#   tools/vision_check.py 在自己的 main() 里把它打成 True（那边每帧都看掩码）。
_WANT_MASK = [False]
# 融合(b_or)失败计数（只报一次）
_GY_FUSE_ERR = [0]
# 国一的融合图已经成功当过预览（之后别再用老掩码覆盖它）
_GY_MASK_OK = [False]
# 国一那张融合图**本身**：国一那条链每 CV2_EVERY 帧才重建一次，中间帧的预览
# 直接复用这一张（否则中间帧没有掩码，vision_check 会掉回老路"普通二值化"，
# 现场就是"反向二值化 ↔ 普通二值化来回闪"）。见 _cv2_find ⑤ 后面的注释。
_GY_MASK_CACHE = [None]


def _gy_rect_corners(r):
    """cv_lite 的矩形（[x,y,w,h,c1x,c1y,c2x,c2y,c3x,c3y,c4x,c4y]）-> 4 个角点。"""
    try:
        pts = [(float(r[4]), float(r[5])), (float(r[6]), float(r[7])),
               (float(r[8]), float(r[9])), (float(r[10]), float(r[11]))]
    except Exception:
        return None
    return _gy_sort_cyclic(pts)


def _gy_sort_cyclic(c):
    """把 4 个角点按**绕质心的方位角**排序 —— 保证是"简单四边形"的循环顺序。

    ⚠ 2026-10-16 现场：cv_lite 返回的角点顺序**不保证**是环序（可能是按 x/y
    排、或从任一角点起、或反向），直接 c1->c2->c3->c4 连线就画成"菱形"，
    重心（对角线交点/鞋带公式）也会被带偏。按方位角排完就与输入顺序无关。
    """
    cx = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4.0
    cy = (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4.0
    return sorted(c, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))


def _pt_in_quad(p, c):
    """点 p 是否在四边形 c 里面（射线法）—— 国一"几何嵌套"判据用。"""
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


def _gy_quad_area(c):
    """四边形面积（鞋带公式）。"""
    _s = 0.0
    for i in range(4):
        x1, y1 = c[i]
        x2, y2 = c[(i + 1) % 4]
        _s += x1 * y2 - x2 * y1
    return abs(_s) * 0.5


def _gy_centroid4(c):
    """四边形面积加权重心 —— 与国一用 cv2.moments 取 m10/m00 等价（纯算术）。"""
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


def _cv2_find(img, hint=None):
    """「国一那套」矩形检测 —— 用板上自带的 OpenCV 接口（cv_lite）跑国一的每一步。

    ① 灰度        init_camera() 按 CVLITE_ON 出 GRAYSCALE，to_numpy_ref()
                    给的就是单通道缓冲的引用（零拷贝），cv_lite 直接吃它
    ② **反向二值化**  image.binary([(0, GY_TH-1)]) —— 暗的=白：黑胶带成为白色前景
                    ⚠ 在 img.copy() 的**副本**上做，绝不碰帧缓冲
    ③ 闭运算        close(GY_CLOSE_SIZE) —— 把圈上被洗白/断开的断口补起来
    ④ Canny        cv_lite.grayscale_find_edges(50,150) —— 在**灰度图**上做
    ⑤ 融合         b_or(闭运算二值图 ∪ Canny 边缘)
    ⑥ 矩形         cv_lite.grayscale_find_rectangles_with_corners(...) —— 这颗
                    模块内部就是 OpenCV 的 Canny+findContours+approxPolyDP+几何校验
    ⑦ 内外双轮廓   几何嵌套 + 面积比 ≥0.7（国一 find_rectangles 策略2）
    ⑧ 返回**内框**当画出来的那个矩形，靶心 = 内框重心
                    国一是拿外框（selected_rects[-1]）透视校正到正视再取内框重心，
                    数学上同一个点；板上没有 warpPerspective 绑定，直接取重心。
                    ⚠ 2026-10-16（用户："斜视时绿框在内外框之间来回切换"）：
                    改回**统一用内框**（纸的边界）—— 胶带的外沿在斜视/反光下时有时无，
                    有它时框画在胶带外沿、没它时画在纸边，就来回跳了；纸边一直很稳。
                    内外配对判据照旧（它保证"内框在外框里"这件事成立）。
    hint：(长边, 中心x, 中心y) —— 上一帧被采纳的那个框。两个用途：
      ① 斜视时画面里常同时存在"纸边/胶带外沿"两套尺寸，用它做时间一致性偏好；
      ② **锁定后只在它周围一块 ROI 里跑 cv_lite**（耗时 ∝ 面积）：目标远、框小
         时 ROI 只占画面几个百分点 -> 那一步从 24ms 掉到几 ms；目标近、框大时
         直接不裁（省不了多少还多一次复制）。裁剪不受支持/裁完没找到就退回全图。
    返回与 _find0 同格式的候选元组；认不出返回 None。
    """
    global _LAST_MASK
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
        # ① 灰度：相机本来就出灰度图，to_numpy_ref() = 单通道缓冲引用（零拷贝）
        gray_np = img.to_numpy_ref()
        # ②③④⑤ 预览那张融合图（反向二值化 + 闭运算 + Canny + 融合）：
        #   ⚠⚠ 2026-10-16：**只有 vision_check（调参工具）需要它**（_WANT_MASK）。
        #   整机 main.py 从头到尾不看掩码（推画面推的是原始帧），而这一步实测
        #   40~60ms/次（二值 7 + 闭运算 29 + Canny + 融合）—— 全是白花的。
        #   跳过它**不影响识别**：⑥ 的矩形检测走 cv_lite 自己在灰度图上的
        #   Canny+轮廓+几何校验，跟这张预览图没有关系。省下来的时间直接变成
        #   帧率（搜索阶段国一每帧都跑，省得最多）。
        fi = None
        if _WANT_MASK[0]:
            # 预览用**半分辨率**（MASK_SCALE=2）：二值化/闭运算/Canny 的耗时都
            #   与像素数成正比，1/4 像素 = 快约 4 倍；贴回画布时放大 2 倍画。
            # ②/③ 反向二值化 + 闭运算：**在副本上做**（img.copy()）——
            #   ① 不能污染 gray_np：cv_lite 的矩形检测要的是原始灰度图；
            #   ② 不能别名帧缓冲：_LAST_MASK 会缓存这张图给预览用，若它指的就是
            #      帧缓冲，下一帧 sensor.snapshot() 一覆盖，预览就变成别的画面
            #      （现场"两种二值化来回闪"里也有它的份）。
            _mw = IMG_W // MASK_SCALE
            _mh = IMG_H // MASK_SCALE
            gi = None
            if MASK_SCALE > 1:
                try:
                    gi = img.copy(x_scale=1.0 / MASK_SCALE,
                                  y_scale=1.0 / MASK_SCALE)
                except Exception:
                    gi = None
                    _mw, _mh = IMG_W, IMG_H
            if gi is None:
                _mw, _mh = IMG_W, IMG_H
                try:
                    gi = img.copy()
                except Exception:
                    gi = None
            if gi is None:
                gi = img.to_grayscale()      # 兜底①：至少换一张图
            if gi is None:
                gi = img                      # 兜底②：实在没有副本，就地做
            # 小图的灰度引用要在二值化**之前**取（同一块缓冲，之后会被改写）
            _gsn = None
            if (_mw != IMG_W) or (_mh != IMG_H):
                try:
                    _gsn = gi.to_numpy_ref()
                except Exception:
                    _gsn = None
            # ② 反向二值化：白 = 暗（比 GY_TH 暗的都变白 → 黑胶带成前景）
            if GY_TH <= 1:
                gi.binary([(0, 0)])
            else:
                gi.binary([(0, GY_TH - 1)])
            # ③ 闭运算：补断口
            if GY_CLOSE_SIZE > 0:
                gi.close(GY_CLOSE_SIZE)
            # ④ Canny 在灰度图上做（不是二值图上）
            edges_np = cv_lite.grayscale_find_edges(
                [_mh, _mw], _gsn if _gsn is not None else gray_np,
                GY_CANNY1, GY_CANNY2)
            # ⑤ 融合：闭运算二值图 ∪ Canny 边缘
            #   ⚠ b_or 要求两张图**同格式** —— 二值化后 gi 是"按位打包"的
            #   掩码图，直接和灰度边缘图做 OR 会不生效（现场"看不到融合"就
            #   是这个）。所以先 to_grayscale() 解包成 0/255 灰度，再 OR。
            #   两种融合写法都试（这版固件的 image 模块到底有哪个没敢赌）：
            #     b_or = OpenMV 式按位或（就地改）；add = 饱和加（白+白=白）。
            fi = gi
            if edges_np is not None:
                _ferr = None
                try:
                    gtmp = gi.to_grayscale()
                    if gtmp is not None:
                        gi = gtmp
                        fi = gtmp
                    ei = image.Image(_mw, _mh, image.GRAYSCALE,
                                     alloc=image.ALLOC_REF, data=edges_np)
                    try:
                        gi.b_or(ei)           # ① 按位或（就地改）
                    except Exception as e1:
                        _ferr = e1
                        _r = gi.add(ei)       # ② 饱和加（就地改或返回新图）
                        if _r is not None:
                            fi = _r
                        _ferr = None
                except Exception as e:
                    _ferr = e
                if _ferr is not None:
                    _GY_FUSE_ERR[0] += 1
                    if _GY_FUSE_ERR[0] == 1:
                        log("[CV2] 融合(b_or/add)失败(%s) -> 预览只有二值图"
                            % _ferr)
        # 预览：融合图（暗的=白）——"看到什么=识别什么"
        #   ⚠ **缓存这张图**（_GY_MASK_CACHE）：国一那条链锁定后每 CV2_EVERY
        #   帧才跑一次，中间帧 _LAST_MASK 本来是 None，vision_check 就会掉回
        #   老路"普通二值化"去凑预览 —— 那就是现场看到的"反向二值化 ↔ 普通
        #   二值化来回闪"。现在中间帧直接用缓存里的同一张图。
        #   ⚠ _GY_MASK_OK 两个场景都要置位：它是"国一那条链能用"的开关，
        #   detect() 里老路掩码链靠它停跑（整机也一样要停）。
        #   （_PROF[0] = 上面②③④⑤这几步的累计耗时，[PERF] 里打"建图xxms"）
        _PROF[0] += time.ticks_diff(time.ticks_ms(), _t)
        try:
            if fi is not None:
                _LAST_MASK = fi
                _GY_MASK_CACHE[0] = fi
            if not _GY_MASK_OK[0]:
                # 只报一次：让日志能对上"预览从哪一帧起固定用融合图"
                # （现场排"两种二值化来回闪"时就是靠这句话定位的）。
                if fi is not None:
                    log("[CV2] 融合图就绪 th=%d（预览固定用它，老路掩码链停跑）"
                        % GY_TH)
                else:
                    log("[CV2] 国一那套就绪（本机不看掩码 -> 不建融合图，"
                        "省 40~60ms/次；老路掩码链停跑）")
            _GY_MASK_OK[0] = True
        except Exception:
            pass
        # ⑥ 矩形（cv_lite 的 C 实现：Canny+轮廓+approxPolyDP+几何校验）
        _t2 = time.ticks_ms()
        #    ⚠ 这一步实测 24ms/次 —— 和官方文档里"480x640 灰度 find_rectangles
        #      40fps（≈25ms）"完全一致，**已经到 C 库的极限**，再快只能少喂像素：
        #      · 锁定后用 hint 在上一帧框周围裁一块 ROI（目标越远省得越多）；
        #      · 裁不了 / 裁完一个都没找到 -> 这一帧退回全图，绝不因此丢识别。
        rects = None
        _rx = _ry = 0
        _rw, _rh = IMG_W, IMG_H
        if (CROP_K > 0.0) and (_CROP_ERR[0] >= 0) and (hint is not None) and \
                (hint[0] > 8.0):
            _hl, _hx, _hy = hint[0], hint[1], hint[2]
            _side = int(_hl * CROP_K) + 2 * CROP_MARGIN_PX
            x0 = int(_hx - _side * 0.5)
            y0 = int(_hy - _side * 0.5)
            x1 = x0 + _side
            y1 = y0 + _side
            if x0 < 0:
                x0 = 0
            if y0 < 0:
                y0 = 0
            if x1 > IMG_W:
                x1 = IMG_W
            if y1 > IMG_H:
                y1 = IMG_H
            _rw = x1 - x0
            _rh = y1 - y0
            if ((_rw * _rh) > CROP_MAX_FRAC * IMG_W * IMG_H) or \
                    (_rw < 48) or (_rh < 48):
                _rw, _rh = IMG_W, IMG_H        # 框太大/太小：不裁
            else:
                try:
                    _sub = gray_np[y0:y0 + _rh, x0:x0 + _rw]
                    rects = cv_lite.grayscale_find_rectangles_with_corners(
                        [_rh, _rw], _sub, GY_CANNY1, GY_CANNY2,
                        GY_EPS, GY_AREA_MIN_RATIO, GY_MAX_COS, GY_BLUR)
                    _rx, _ry = x0, y0
                    if (rects is not None) and (len(rects) > 0):
                        _CROP_MISS[0] = 0
                        # ROI 内部坐标 -> 全图坐标（角点/外接框一起挪）
                        _out = []
                        for _r in rects:
                            _q = list(_r)
                            _q[0] += _rx
                            _q[1] += _ry
                            for _k in (4, 6, 8, 10):
                                _q[_k] += _rx
                            for _k in (5, 7, 9, 11):
                                _q[_k] += _ry
                            _out.append(_q)
                        rects = _out
                    else:
                        _CROP_MISS[0] += 1
                        if _CROP_MISS[0] >= 3:
                            # 连 3 帧在 ROI 里一个矩形都没有 = 目标已经不在
                            # 上一帧框附近了（或者裁剪缓冲这版 cv_lite 不认）——
                            # 退回全图，别再省这点时间。
                            _CROP_ERR[0] = -1
                            log("[CV2] ROI 连续 3 帧没找到 -> 退回全图检测")
                        rects = None
                except Exception as e:
                    _CROP_ERR[0] += 1
                    if _CROP_ERR[0] == 1:
                        log("[CV2] ROI 裁剪不可用(%s) -> 以后都跑全图" % e)
                    rects = None
        if rects is None:
            _rw, _rh = IMG_W, IMG_H
            rects = cv_lite.grayscale_find_rectangles_with_corners(
                [IMG_H, IMG_W], gray_np, GY_CANNY1, GY_CANNY2,
                GY_EPS, GY_AREA_MIN_RATIO, GY_MAX_COS, GY_BLUR)
        _PROF[2] += time.ticks_diff(time.ticks_ms(), _t2)
        _PROF[3] += 1
        if (_rw != IMG_W) or (_rh != IMG_H):
            _PROF[1] += _rw * _rh        # 这一拍实际喂了多少像素（[PERF] 里看）
        else:
            _PROF[1] += IMG_W * IMG_H
        if (rects is None) or (len(rects) < 1):
            return None
        # ⑦ 内外双轮廓配对：内框在外框里、面积比 ≥ GY_AREA_RATIO
        _pairs = []
        for _ro in rects:
            _co = _gy_rect_corners(_ro)
            if _co is None:
                continue
            _ao = _gy_quad_area(_co)
            _bi = None
            for _ri in rects:
                _ci = _gy_rect_corners(_ri)
                if _ci is None:
                    continue
                _ai = _gy_quad_area(_ci)
                if (_ai >= _ao) or (_ai < GY_AREA_RATIO * _ao):
                    continue
                if not _pt_in_quad(_ci[0], _co):
                    continue
                if (_bi is None) or (_ai > _bi[0]):
                    _bi = (_ai, _ci)
            if _bi is not None:
                _pairs.append((_bi[0], _bi[1], _co))
        # ⑦b 单矩形兜底：cv_lite 若没给出"内框"（比如它内部只取外轮廓），
        #     就把矩形自己当内=外（靶心=它的重心）—— 靶纸与胶带圈同心，
        #     重心就是靶心；有配对的优先用配对（内框重心更准）。
        if not _pairs:
            for _ro in rects:
                _co = _gy_rect_corners(_ro)
                if _co is None:
                    continue
                _pairs.append((_gy_quad_area(_co), _co, _co))
        if not _pairs:
            return None
        # 画面里可能不止一对 -> 用我们的判据挑：内亮外暗/暗边 + 激光点近优先 + 面积大
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
            _i2, _o2, _f2, _r2 = paper_contrast(img, _bx, _by, _bw, _bh)
            if (_i2 < 0) or ((_i2 - _o2) < PAPER_CONTRAST_MIN):
                continue
            if _f2 < PAPER_DARK_FRAC_MIN:
                continue
            # "纸 -> 黑胶带 -> 外面又亮起来"：外面几圈里最亮的那圈必须比紧贴的
            #   那圈（胶带）亮 PAPER_RING_RISE_MIN —— 深色背景上的一块白，外面
            #   一直深，过不了这条（老路的'圈外沿'检查删掉后补回来的）。
            if (_r2 >= 0.0) and (_r2 < _o2 + PAPER_RING_RISE_MIN):
                continue
            _cx = _bx + _bw / 2.0
            _cy = _by + _bh / 2.0
            _dc = abs(_cx - SPOT_UV[0]) + abs(_cy - SPOT_UV[1])
            # 排序键：有"内外配对"的优先（内框才是纸，最可信）>
            #         内框尺寸跟上一帧一致（斜视时防在"纸边/胶带外沿"之间来回切）>
            #         激光点近 > 面积大
            # 内框的"外接框长边"（和下面返回的 _long 同一个量），拿它和上一帧比
            _ic = _pj[1]
            _ixs = (_ic[0][0], _ic[1][0], _ic[2][0], _ic[3][0])
            _iys = (_ic[0][1], _ic[1][1], _ic[2][1], _ic[3][1])
            _il = max(max(_ixs) - min(_ixs), max(_iys) - min(_iys))
            _stable = 0
            prefer_long = hint[0] if hint is not None else 0.0
            if (prefer_long > 1.0) and \
                    (abs(_il - prefer_long) <= 0.18 * prefer_long):
                _stable = 1
            _key = (1 if _pj[1] is not _pj[2] else 0, _stable,
                    1 if _dc <= SPOT_PREF_PX else 0, _gy_quad_area(_pj[1]))
            if (_best is None) or (_key > _best[0]):
                _best = (_key, _pj)
        if _best is None:
            return None
        _inner = _best[1][1]          # 内框（纸的边界）
        _outer = _best[1][2]          # ★外框 = 国一"最后的矩形"（画出来/上报的）
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
            return None
        _long = max(_bw, _bh)
        _short = max(1, min(_bw, _bh))
        _aspect = _long / float(_short)
        if (_long < PAPER_MIN_LONG) or (_long > PAPER_MAX_LONG):
            return None
        if (_aspect < PAPER_ASPECT_MIN) or (_aspect > PAPER_ASPECT_MAX):
            return None
        _pxa = _gy_quad_area(_outer)
        _dens = _pxa / float(max(1.0, _bw * _bh))
        _ins, _outs, _frac, _rise = paper_contrast(img, _bx, _by, _bw, _bh)
        # 返回：外框做四边形（国一：外框才是最后的矩形），靶心 = 内框重心。
        #   ⚠ 2026-10-16 试过"统一改成内框"（想治斜视时绿框在外/内框之间切）——
        #   闭环仿真立刻变差（残差 1.8px -> 14.8px）：**巡检那条复测链（requad）
        #   的边就是纸边（_edge_precise 摸到黑圈就停）**，两边口径一变，
        #   锁定态 2/3 的帧来自复测、1/3 来自这条，两者差 ~20px，控制就把
        #   平均值当目标 -> 稳态残差。要统一得连 requad 一起改，留到下一轮。
        return (_pxa, _bx, _by, _bw, _bh, _long, _aspect, _dens,
                _ins, _outs, tuple(_outer), _ci[0], _ci[1])
    except Exception as e:
        _GY_ERR[0] += 1
        if _GY_ERR[0] == 1:
            log("[CV2] 国一那套（cv_lite 版）异常(%s) -> 退回色块/矩形候选路" % e)
        return None
    finally:
        _PROF[14] += time.ticks_diff(time.ticks_ms(), _t)
        _PROF[15] += 1


def log_scene(img):
    """每 5 秒打一行"画面亮度"（10x12 网格自己采样），用来判断为什么没锁到靶：
      · 均/大 都很小（均<60 且 大<90）-> 画面太暗 / 镜头被挡 / 对着暗墙
      · 有大亮块（大>=200）但没靶     -> 靶纸不在视野里，或太远太小
      · 均 100+ 且有大亮块            -> 视野正常，那是靶纸判据/阈值的问题
    2026-10-14（用户："看阈值到底该放哪"）加**分位数** P25/P50/P75：
      光看 均/小/大 分不出"纸面/黑胶带/木柜"三层各在哪 ——
      经验上：木柜 ~P50、黑胶带 ~P25、纸面 ~P75~P90。阈值（显示成 luma）
      落在 P25 和 P50 之间 = 胶带黑、木柜白 = 矩形框可见。
    以前没有这行时，"候选0"（一个亮块都没有）只能靠猜。"""
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
            "（预览阈值 GY_TH=%d：反向二值化里白的=比它暗，胶带落在 P25 那档、"
            "木柜在 P50 那档）  靶=无靶时看这行"
            % (s / float(k), lo, hi, p25, p50, p75,
               GY_TH))


def paper_contrast(img, x, y, w, h):
    """内亮外暗校验：返回 (内部平均亮度, 外侧平均亮度, 外侧合格比例, 更外侧最亮)。

    外侧取 12 个点（四边各 3 个），要求其中至少 PAPER_DARK_FRAC_MIN 的比例
    明显比内部暗 —— 背景柜子那种“只有一边有暗边”的亮块因此过不了。

    第 4 个返回值（2026-10-16 加）：再往外几圈里**最亮的那一圈**。真靶纸是
      "白纸 -> 一圈黑胶带 -> 背景又变亮"，而"深色背景上的一块白"外面一直
      是深的 —— 后者这圈亮不起来，就被这道判据挡住了（老路的"圈外沿"检查
      删掉之后，需要一个便宜的替代品；这里只是多采 24 个点）。
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
    # 更外侧：在 4 个更远的偏移上各取一圈，取最亮的那个平均值
    far = -1.0
    for d in (PROBE_OFF_PX + 10, PROBE_OFF_PX + 18, PROBE_OFF_PX + 28,
              PROBE_OFF_PX + 42, PROBE_OFF_PX + 60, PROBE_OFF_PX + 78):
        sf = 0
        nf = 0
        for p in ((x - d, y + h * 0.35), (x - d, y + h * 0.65),
                  (x + w + d, y + h * 0.35), (x + w + d, y + h * 0.65),
                  (x + w * 0.35, y - d), (x + w * 0.65, y - d),
                  (x + w * 0.35, y + h + d), (x + w * 0.65, y + h + d)):
            v = px_luma(img, p[0], p[1])
            if v >= 0:
                sf += v
                nf += 1
        if nf >= 4:
            m = sf / float(nf)
            if m > far:
                far = m
    return ins, so / float(no), ok / float(no), far
class PaperDetector(object):
    """A4 靶纸检测 —— **只有国一那条链（cv_lite）**。

    2026-10-16 大清理（用户："老方案不行就换，把没用的代码删掉"）：
      以前并存三条路（老路 find_blobs + 掩码链、cv_lite 找矩形候选、国一
      内外双轮廓），口径不同（纸边 vs 胶带外沿）、互相抢帧、还得靠"哪条成功过"
      来切换 —— 斜视时框在两种尺寸之间来回切、快速甩镜头时框卡住，根子都在这。
      现在只留国一那条链（_cv2_find），**每 CV2_EVERY 帧测一次**（=1 就是每帧），
      中间帧沿用上一帧的平滑值。一个来源，不会再打架。

    返回 dict: center/box/area/aspect/density/long_side/contrast/n/state
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
        self.frame = 0
        self.last_n = 0
        self.last_dbg = ""
        self.cands = []            # 给 vision_check 叠加用（现在最多一个）
        self.state = "搜索"
        self.err = 0
        self.err_msg = ""
        self.last_res = None
        # 平滑中心每帧的移动量（px）：护栏放宽用
        self.mot_px = 0.0
        # 时间一致性护栏的"实测运动量"：从**原始候选**算（被拒也更新）——
        #   mot_px 是从被采纳的中心算的，连续被拒时它冻住、护栏会锁死
        #   （现场："画面变化快时绿框停在原地，过一会才矫正"）。
        self.raw_u = None
        self.raw_v = None
        self.cand_mot = 0.0
        self.rej_n = 0             # 连续因"中心跳太远"被拒的帧数
        self.last_long = 0.0       # 上一帧被采纳的长边（尺寸闸门用同一个量）

    def fresh(self):
        return self.have and (self.lost == 0)

    def _accept(self, cand):
        """输出前最后一道口子：中心跳太远 / 尺寸突变 / 实心度突变 -> 这一帧
        当"没测到"（输出保持上一帧，H723 侧 hold 偏置），乱跳的帧到不了云台。

        允许位移 = 35px + 2×运动量，运动量取 max(平滑中心移动量, 原始候选移动量)
        —— 后者被拒也更新，所以快速甩镜头时护栏会跟着放宽，不会把自己锁死；
        连拒 2 帧再放宽一档、连拒 5 帧放到很宽（运动量本身封顶 120px）。
        """
        x, y, w, h = cand[1], cand[2], cand[3], cand[4]
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
            if abs(cu - self.u) > _jl:
                self.rej_n += 1
                return False
            if abs(cv - self.v) > _jl:
                self.rej_n += 1
                return False
            self.rej_n = 0
            old_l = self.last_long if self.last_long > 1.0 \
                else max(self.w, self.h)
            if old_l > 1.0:
                if (cand[5] < 0.85 * old_l) or (cand[5] > 1.18 * old_l):
                    return False
            if self.meas is not None:
                if abs(cand[7] - self.meas[7]) > 0.15:
                    return False
        if (not self.have) or (self.lost >= LOST_FULL):
            # 首次锁定 / 丢靶很久后重新锁定：旧的平滑中心没意义了，直接吸附
            self.u, self.v, self.w, self.h = cu, cv, w, h
            self.have = True
            self.corners = c
        else:
            # 靶纸小（远）时平滑做强一点：远处中心估计本来就抖
            k = SMOOTH_FAR if (0.0 < cand[5] < FAR_LONG_PX) else SMOOTH
            if abs(cu - self.u) > DEADBAND_PX:
                self.u += k * (cu - self.u)
            if abs(cv - self.v) > DEADBAND_PX:
                self.v += k * (cv - self.v)
            if abs(w - self.w) > 2 * DEADBAND_PX:
                self.w += k * (w - self.w)
            if abs(h - self.h) > 2 * DEADBAND_PX:
                self.h += k * (h - self.h)
            # 四角也做时间平滑：逐帧在几种拟合之间抖，画出来就是框在抖
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

    def detect(self, img):
        """返回 dict（center/box/...）或 None（这一帧没有可用测量）。"""
        self.frame += 1
        # 中间帧（CV2_EVERY>1 时）：沿用上一帧的结果 —— 框不动、偏置照发
        if self.have and (self.last_res is not None) and \
                ((self.frame % CV2_EVERY) != 0):
            self.last_n = 0
            return self.last_res
        cand = None
        try:
            # hint = (上一帧长边, 中心x, 中心y)：既做时间一致性偏好，也用来裁 ROI
            cand = _cv2_find(img, (self.last_long, self.u, self.v)
                             if self.have else None)
        except Exception as e:
            self.err += 1
            if self.err_msg != str(e):
                self.err_msg = str(e)
                # 写 log（不只 print）：异常文本要能进 SD 日志，一眼定位
                log("[ERROR] 靶纸检测异常: %s" % e)
            cand = None
        self.cands = []
        if cand is not None:
            self.cands.append((cand[1], cand[2], cand[3], cand[4], 1))
            if not self._accept(cand):
                cand = None
        self.last_n = 1 if (cand is not None) else 0
        if cand is None:
            self.lost += 1
            self.state = "锁定" if (self.have and self.lost < LOST_FULL) \
                else "搜索"
            self.last_dbg = "[丢 %d 帧]" % self.lost
            return None
        _du = self.u - self.meas[11]
        _dv = self.v - self.meas[12]
        _dd = _du * _du + _dv * _dv
        self.mot_px = _dd ** 0.5 if _dd > 0.0 else 0.0
        self.last_dbg = "[国一 内外双轮廓 ✓] 长边%.0f 密%.2f 对比%.0f" % (
            cand[5], cand[7], cand[8] - cand[9])
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

    # ---- 原生 find_blobs 版（RGB565）----
    def _detect_blobs(self, img):
        x0, y0, w, h = self.roi()
        # ⚠ 2026-10-13：拆成两次调用、并**优先选"过曝白芯"**。
        #   以前把两个阈值混在一起按面积挑最大块 —— 远距离时红色光晕(HALO)面积大，
        #   常常盖过白芯被选中；而光晕是**偏离激光真实中心**的（纸面散射不对称），
        #   于是"黄圈(假定光斑)与实际光斑"就有一个固定偏差，瞄准时激光整体偏一侧
        #   （现场：总往右下偏）。白芯（L 88~100）才是激光真正打中的那一点。
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
                # 光斑是在 ROI 里找的，坐标要加回 ROI 原点才是全图坐标
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


# ============================ 画面亮度诊断 ============================
def log(msg):
    print(msg)


def einfo(e):
    """异常信息带类型 —— 这版固件上 Python 异常的字符串经常只有一个数字
    （现场被 `叠加层异常(0)` / `二值预览失败(0)` 坑过好几轮，其实就是
    KeyError(0)）。带上类型，下次一眼就能看出是什么错。"""
    return "%s: %s" % (type(e).__name__, e)



# ============================ 主程序 ============================
def apply_exposure(sensor):
    """run() 之后把曝光"冻"在自动曝光已经调好的水平上（本固件唯一可行的固定曝光法）。

    ⚠ 为什么这么绕（2026-10-15 现场实测）：
      · sensor.set_auto_exposure / exposure 是空壳；底层
        kd_mpi_sensor_intg_time_set 也没暴露给 Python（日志原文 "module object
        has no attribute kd_mpi_sensor_intg_time_set"）；
      · 唯一真能关 AE 的开关是 VICAP dev attr 的 pipe_ctrl.bits.ae_enable，
        但它只在 run() 时下发 —— **run() 之前就清掉的话，曝光停在传感器出厂
        默认值上，画面全黑**（实测 均=32 大=57）。
      · 所以：先让 AE 正常跑 EXPO_SETTLE_MS 把画面调亮，再运行时下发
        ae_enable=0 冻住当时的曝光/增益；冻完复测亮度，明显变暗就把 AE 开回来
        （宁可不锁，也不能全黑）。
    返回给日志用的一句话。"""
    if not EXPO_FIX:
        return "关（EXPO_FIX=False）"

    def _snap_mean():
        try:
            _im = sensor.snapshot(chn=0)
            _v = _grid_mean(_im)
            del _im
            return _v
        except Exception:
            return -1.0

    # 1) 先让 AE 把画面调到正常亮度
    time.sleep_ms(EXPO_SETTLE_MS)
    _m1 = _snap_mean()
    # 2) 冻住：清 ae_enable 位，并**运行时**下发 dev attr
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
        except Exception as e:
            _how = _how + "（下发失败:%s）" % e
    else:
        _how = "拿不到 dev attr(%s)" % _why
    # 3) 复测亮度：明显变暗 -> 回滚（把 AE 开回来）
    time.sleep_ms(400)
    _m2 = _snap_mean()
    _tail = ""
    if (_m1 > 0) and (_m2 > 0) and (_m2 < _m1 * EXPO_DARK_RATIO):
        try:
            sensor._dev_attr.pipe_ctrl.bits.ae_enable = 1
            import mpp
            mpp.kd_mpi_vicap_set_dev_attr(sensor._dev_id, sensor._dev_attr)
            _tail = "；冻完变暗(%.0f->%.0f) 已把 AE 开回来" % (_m1, _m2)
        except Exception as e:
            _tail = "；冻完变暗(%.0f->%.0f) 且回滚失败(%s)" % (_m1, _m2, e)
    # 4) 冻住之后按 EXPO_GAIN_SCALE 微调模拟增益（"把曝光调低一点"的旋钮；
    #    增益不是曝光，但 AE 已经关了，它就直接决定画面亮度 ✔）
    if (EXPO_GAIN_SCALE > 0) and (abs(EXPO_GAIN_SCALE - 1.0) > 1e-6):
        try:
            _g = sensor.again()                   # 读当前增益（k_sensor_gain 对象）
            _old = float(_g.gain[0])
            _g.gain[0] = _old * EXPO_GAIN_SCALE
            sensor.again(_g)                      # 写回去
            time.sleep_ms(300)
            _m3 = _snap_mean()
            _tail += "；增益 %.2f->%.2f 亮度 %.0f" % (
                _old, _old * EXPO_GAIN_SCALE, _m3)
        except Exception as e:
            _tail += "；调增益失败(%s)" % e
    return "冻曝光: %s；亮度 %.0f -> %.0f%s" % (_how, _m1, _m2, _tail)


def _grid_mean(img):
    """10x10 网格采样平均亮度（0~255）：判断"冻结曝光后画面有没有变暗"。"""
    s = 0.0
    k = 0
    for gy in range(10):
        y = int(IMG_H * (gy + 0.5) / 10)
        for gx in range(10):
            x = int(IMG_W * (gx + 0.5) / 10)
            try:
                p = img.get_pixel(x, y)
            except Exception:
                continue
            try:
                v = ((p[0] + p[1] + p[2]) / 3.0) if isinstance(p, (tuple, list)) else p
            except Exception:
                continue
            s += v
            k += 1
    return (s / k) if k else -1.0


def main():
    from media.sensor import Sensor, CAM_CHN_ID_0
    from media.display import Display
    from media.media import MediaManager
    import gc
    global PAPER_TH_FIXED          # 阈值轮换（TH_SWEEP）要在运行时改它

    sensor = Sensor(id=2, width=1280, height=960, fps=90)
    sensor.reset()
    time.sleep_ms(100)
    apply_flip(sensor)
    # ⚠ 800x600 这类非标准尺寸也能设（ISP 会缩放，宽度自动 16 对齐）；
    #   万一这版固件不认，会在这里抛异常 —— 那就把 PROC_W/PROC_H 改回 640/480。
    sensor.set_framesize(width=IMG_W, height=IMG_H, chn=CAM_CHN_ID_0)
    # ⚠ 2026-10-15：CVLITE_ON 时出图必须用 **GRAYSCALE** —— 资料 06.1 的
    #   grayscale_find_rectangles_with_corners / grayscale_threshold_binary
    #   例程（5/9/13）都是这么配的（它们直接吃 to_numpy_ref() 的单通道缓冲）。
    #   整个检测链本来就跑在灰度语义上（px_luma 两种都支持），所以换成灰度
    #   不影响判据。
    sensor.set_pixformat(Sensor.GRAYSCALE if CVLITE_ON else Sensor.RGB565,
                         chn=CAM_CHN_ID_0)
    apply_flip(sensor)
    if EXPO_FIX:
        # ⚠ 2026-10-15 现场教训：**run() 之前别关 AE**（关了就停在出厂默认曝光上，
        #   画面全黑，实测 均=32）。这里什么都不做，等 run() 之后由
        #   apply_exposure() 执行"先让 AE 调好、再冻住"。
        log("曝光: 先让自动曝光(AE)自适应 %dms，run 后再冻住（见 apply_exposure）"
            % EXPO_SETTLE_MS)
    # 显示：画布固定 640x480（**这块板子的显示只支持这个尺寸**，2026-10-11
    # 实测 800x600 报 "ST7701 unsupport 800x600"，VIRT 那条路也不出画面）。
    # 处理图像在下面用 draw_image(x_scale/y_scale) 缩到画布上。
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
    # run() 之后才设曝光时间（AE 已在上面关掉；见 apply_exposure 说明）
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
    _msc = 1.0                    # 预览图的缩放倍数（半分辨率掩码 = MASK_SCALE）
    _cv_shape = False             # 画布上能不能画图形（下面探测）
    _cv_text = False              # 画布上能不能写字
    _ov_bad = []                  # 哪些绘制调用这版固件不支持（每个只报一次）

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

    def _overlay(target, det, spot_uv, s, ox, oy, ink, txt):
        """叠加层：候选框 / 四边形 / 靶心 / 激光 / 文字。

        target = 画布（此时 s=_sx, ox=_ox：原图坐标要缩放到画布）或
                 img  （此时 s=1, ox=0：原图坐标直接用）。
        ink=None 表示 target 是**彩色/画布** -> 用绿框/红叉/黄圈；
        ink=0 或 255 表示 target 是**单色二值图** -> 所有标记都用这个灰度值画
        （二值图上画彩色没意义）。2026-10-16：墨色由调用方给 ——
        老路掩码是"白纸黑边"（用 0=黑最显眼），国一融合图是"黑底白胶带"
        （必须用 255=白，否则靶心那个叉画在黑底上等于没画）。
        spot_uv 收两种写法：{"uv": (u, v)} 字典或 (u, v) 元组。
        （⚠ 2026-10-11 踩过：传进来是字典、这里按元组 index 取 —— 会抛
          KeyError(0)，日志里只显示 "(0)"，查了好几轮。两种都收，省得再犯。）
        """
        if spot_uv is not None and (type(spot_uv) is dict):
            spot_uv = spot_uv["uv"]
        if ink is not None:
            _v = int(ink)
            c_ok = c_no = c_cross = c_spot = _v
        else:
            c_ok = (0, 255, 0)
            c_no = (255, 128, 0)
            c_cross = (255, 0, 0)
            c_spot = (255, 255, 0)
        if PREVIEW_CANDS:
            # "被拒候选框"（橙）默认不画了（用户："上面还有许多杂乱的框"）——
            # 大块候选动辄 640px 宽，画出来满屏橙色。要看就开 PREVIEW_CANDS。
            for (bx, by, bw, bh, ok) in det.cands:
                _ov("rect", target.draw_rectangle,
                    int(bx * s + ox), int(by * s + oy),
                    int(bw * s), int(bh * s),
                    color=c_ok if ok else c_no, thickness=1)
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
        """把 img 变灰度，返回 (要画的对象, 走的路)。兼容三种固件行为。
        2026-10-14（55 号日志）：用户"并没有看到灰度图像，只有现实图像"——
        这版固件 `img.to_grayscale()` **不是原地转换：返回值才是灰度图**，
        上次忽略了返回值 -> 画的还是彩色。三条路挨个试：
          ① 返回值本身是灰度（分量<=2）        -> ("rv")
          ② 返回 None 且原地已转灰（中心像素判定）-> ("inplace")
          ③ 都不行 -> 预分配灰度缓冲 + copy_to   -> ("copy")
        返回 (None, "") = 这版固件灰度化用不了（调用方退回彩色预览）。
        """
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

    # ---------- 相机纯取图测速（1.5 秒）----------
    # 这一项把两个上限分开：
    #   ① 相机+ISP 到底能吐多少帧（纯 snapshot，不做任何处理）
    #   ② 我们的检测吃掉多少（下面每行打印里的"ms 取图/目标"）
    # 如果 ① 只有 40 帧左右，那再怎么优化处理代码也提不上去（瓶颈在采集）；
    # 如果 ① 有 80 帧，而实际 FPS 只有 40，那瓶颈就在我们的处理上。
    # 只有测出来才知道往哪儿使劲。
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

    # ---------- 灰度/二值预览：先探测这版固件支持哪种画法 ----------
    # 2026-10-11 现场日志："二值预览画不出来(0) -> 回到彩色预览" ——
    # `img.binary()` 是能跑的，卡在"BINARY 图直接 draw_image 到 RGB565 画布"。
    # 所以这里拿一帧真图把两种做法挨个试一遍，把能用那套记下来 + 打日志，
    # 以后换固件也能一眼看出是哪条路在走。
    _pv_what = "灰度预览" if GRAY_PREVIEW else "二值化预览"
    if BIN_PREVIEW or GRAY_PREVIEW:
        try:
            _pi = sensor.snapshot(chn=CAM_CHN_ID_0)
            if GRAY_PREVIEW:
                # 灰度预览：只灰度化、不二值化 —— 看清纸/胶带/木柜各多亮
                # （⚠ 必须用返回值：这版固件 to_grayscale 返回新图，不是原地转）
                _gt, _gm = _to_gray(_pi)
                print("灰度探测: 转换路径=%s (rv=用返回值/inplace=原地/copy=预分配)"
                      % (_gm or "失败"))
                if _gt is not None:
                    _pi = _gt
            # else：二值预览不做任何变换 —— 画的就是 _cv2_find 建的那张融合图
            #   （反向二值化 ∪ Canny，见 _GY_MASK_CACHE），跟识别口径完全一致。
            try:
                # 做法 B：图直接缩放（**零分配**，最不容易在跑起来之后翻车）
                _canvas.draw_image(_pi, _ox, _oy,
                                   x_scale=_sx, y_scale=_sy)
                _bin_mode = "bin"
            except Exception as e:
                print("预览B(直画缩放)不行: %s" % e)
            if not _bin_mode:
                try:
                    # 做法 C：转进一块**预分配**的 RGB565（只分配这一次，之后复用）
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
                    # 做法 A：每帧 to_rgb565（每帧要新建 614KB，最容易失败 -> 放最后）
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
        # 叠加层画在哪儿：先看画布上能不能画（不能就退回"画在二值图上"）
        # 2026-10-11 现场日志里那句"叠加层异常(0)"就是这么来的 —— 挨个试一遍，
        # 下次日志会直接写明到底哪一步不支持。
        # 逐个试四种图形调用（上次只试了矩形，结果运行时才发现有的调用不支持）
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
    _sweep_k = -1         # 阈值轮换当前档（见 TH_SWEEP）
    _gy_sw = -1           # 国一 GY_TH 轮换当前档（见 GY_TH_SWEEP）
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

    # 本工具每帧都要看检测图 —— 打开"强制建掩码"（整机 main.py 不开，
    # 那边锁定稳态大多数帧只做四边形复测，省时间）。
    #   （GY_TH 也在这里声明：国一阈值轮换要改这个模块级常量 —— Python 要求
    #     global 声明出现在本函数里第一次使用它**之前**。）
    #   _WANT_MASK：本工具**要那张融合图**（国一的预览就是你一直在看的画面），
    #     整机 main.py 里它是 False（那边没人看掩码，建了白花 40~60ms/次）。
    global _FORCE_MASK, GY_TH, _WANT_MASK
    _FORCE_MASK = True
    _WANT_MASK[0] = True

    print("=" * 68)
    print("vision_check: 纯视觉（不发串口）; 视觉处理 = k230_aim/main.py 原文")
    print("  分辨率 %dx%d (S=%.2f, K=%.2fpx/°)  靶心=四边形对角线交点"
          % (IMG_W, IMG_H, S_SCALE, VIS_K))
    def _ksize(n):
        return "关" if n <= 0 else ("%dx%d" % (2 * n + 1, 2 * n + 1))

    def _dark_frac(im, th):
        """采样统计：画面里有多少比例的像素比 th 暗（= 反向二值化里变白的比例）。

        为什么要有这个数（2026-10-16 用户："看不出白框完整不完整"）：
        眼睛分不清"只有胶带白"和"胶带 + 木柜暗部都白"。采样 20x16=320 点给个数：
          · 只有胶带时大约 **3%~10%**（胶带圈只占画面一小条）；
          · 明显 >15% = 木柜/背景也切进来了（白斑开始糊成片）。
        粗采样只做一次/2 秒，几毫秒，不影响帧率。
        """
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

    print("  检测=国一那条链（cv_lite）每%d帧测一次；外框=矩形/内框重心=靶心"
          % CV2_EVERY)
    print("  预览=反向二值化(暗=白 th=%d)%s∪Canny(%d,%d) -> 外框绿、靶心红"
          % (GY_TH,
             ("+闭运算%d×%d" % (2 * GY_CLOSE_SIZE + 1, 2 * GY_CLOSE_SIZE + 1))
             if GY_CLOSE_SIZE > 0 else "",
             GY_CANNY1, GY_CANNY2))
    print("  曝光=%s%s  光斑每%d帧搜一次"
          % ("冻结(run 后冻住 AE)" if EXPO_FIX else "自动",
             ("  增益x%.2f" % EXPO_GAIN_SCALE) if EXPO_GAIN_SCALE != 1.0 else "",
             SPOT_EVERY))
    print("  长边%d~%dpx 比例[%.2f,%.2f] 对比>=%d 暗边>=%.0f%%（_cv2_find 判据）"
          % (PAPER_MIN_LONG, PAPER_MAX_LONG,
             PAPER_ASPECT_MIN, PAPER_ASPECT_MAX,
             PAPER_CONTRAST_MIN, PAPER_DARK_FRAC_MIN * 100))
    print("=" * 68)

    try:
        while True:
            os.exitpoint()
            now = time.ticks_ms()

            # ---------- 国一反向二值化阈值轮换（默认关；见 GY_TH_SWEEP 说明）----------
            #   只改预览那张融合图用的 GY_TH，不碰识别链。
            if GY_TH_SWEEP and GY_TH_SWEEP_LIST:
                _sw2 = ((time.ticks_ms() // (GY_TH_SWEEP_SEC * 1000)) %
                        len(GY_TH_SWEEP_LIST))
                if _sw2 != _gy_sw:
                    _gy_sw = _sw2
                    GY_TH = GY_TH_SWEEP_LIST[_sw2]
                    print(">>> 国一阈值轮换: GY_TH=%d（看预览：只有胶带是白的"
                          "那一档最好）" % GY_TH)

            # ---------- 取图 ----------
            t0 = time.ticks_ms()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            t_grab = time.ticks_diff(time.ticks_ms(), t0)
            n_frame += 1

            # ---------- 靶纸（与 main.py 同一套 PaperDetector）----------
            # 预览用的掩码**每 3 帧建一次**（中间帧共用同一张 _LAST_MASK）：
            # 形态学实测 55ms/次，每帧都建会把"看画面"这件事变成卡顿本身。
            # 想让预览更实时就改小这个 3（代价就是帧率）。
            _FORCE_MASK = ((n_frame % 3) == 0)
            t0 = time.ticks_ms()
            res = target_det.detect(img)
            t_tgt = time.ticks_diff(time.ticks_ms(), t0)
            if res is not None:
                n_tgt += 1

            # ---------- 光斑（每 SPOT_EVERY 帧搜一次，其它帧复用）----------
            t0 = time.ticks_ms()
            if CALIB_LASER:
                # 标定模式：每帧都找光斑，黄圈跟着"检出来的"走
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
                # 固定激光坐标（标定常数）：不再检测光斑，少一层不确定
                spot = {"uv": LASER_UV}
            if spot is not None:
                n_spot += 1
                SPOT_UV[0], SPOT_UV[1] = spot["uv"]

            # 丢靶时打一行画面亮度：判断"没看到靶"还是"判据没过"
            if target_det.lost == 8 or (target_det.lost > 30 and
                                        target_det.lost % 90 == 0):
                log_scene(img)

            # ---------- [MASK] 读数：有多少比例的像素比 GY_TH 暗 ----------
            #   （= 反向二值化里"变白"的比例；只有胶带时 3~10%）
            #   每 2 秒一行，用来把"框完整不完整"从"看着像"变成"数得出来"。
            if time.ticks_diff(now, t_mask) >= 2000:
                t_mask = now
                _df = _dark_frac(img, GY_TH)
                if _df >= 0:
                    print("[MASK] GY_TH=%d 暗于此的像素=%.1f%% (只有胶带约 3~10%%；"
                          ">15%% 说明木柜也切进来了)" % (GY_TH, _df))

            # ---------- 画 ----------
            # （二值预览时不在原图上画 —— 反正会被 binary() 抹掉，改为在画布上画）
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

            # 推画面节拍 = PREVIEW_EVERY（本工具默认每帧都推；见顶部说明）。
            # 画布 640x480 是死的，处理图像按 _sx/_sy 缩着画上去（见 main 开头）。
            t0 = time.ticks_ms()
            if (n_frame % PREVIEW_EVERY) == 0:
                _ok = False
                if _bin_mode:
                    # 预览变换：灰度预览只灰度化；二值预览用**和检测同一个
                    # 阈值元组**切成黑白（白=过阈值）—— a/b 全放开到 ±128，
                    # 与 _find() 的 find_blobs 元组**逐字一致**（54 号日志教训：
                    # 之前这里还留着 ±32 的色度门槛，看到的二值图 ≠ 检测的图）。
                    # 每一步单独套 try：哪一步在这版固件上翻车，日志里直接点名
                    # （现场教训：一整块套一个 try，只报一句 "(0)"，白跑一轮）。
                    if GRAY_PREVIEW:
                        try:
                            _g2, _m2 = _to_gray(img)
                            if _g2 is None:
                                _bin_mode = ""
                                print("灰度化不可用 -> 改彩色预览")
                            else:
                                img = _g2
                        except Exception as e:
                            _bin_mode = ""
                            print("灰度化失败(%s) -> 改彩色预览" % einfo(e))
                    else:
                        # 检测图预览：**就是 _cv2_find 建的那张融合图**
                        #   （反向二值化 ∪ Canny）—— "看到什么 = 识别什么"。
                        #   _WANT_MASK=True 时 _cv2_find 每一步都会重建它，
                        #   所以这里直接用 _GY_MASK_CACHE（没有就退彩色预览）。
                        try:
                            _mg = None
                            _mg = _GY_MASK_CACHE[0]
                            if _mg is None:
                                _mg = _LAST_MASK
                            if _mg is None:
                                _msc = 1.0
                                _bin_mode = ""
                                print("检测图预览: 还没建出来 -> 改彩色预览")
                            else:
                                img = _mg
                                # 掩码是半分辨率建的（MASK_SCALE）-> 贴回画布放大画
                                _msc = float(MASK_SCALE)
                        except Exception as e:
                            _bin_mode = ""
                            print("检测图预览失败(%s) -> 改彩色预览" % einfo(e))
                if _bin_mode:
                    try:
                        if _bin_mode == "bin":
                            _canvas.draw_image(img, _ox, _oy,
                                               x_scale=_sx * _msc,
                                               y_scale=_sy * _msc)
                        elif _bin_mode == "copy":
                            img.copy_to(_bin_rgb)
                            _canvas.draw_image(_bin_rgb, _ox, _oy,
                                               x_scale=_sx, y_scale=_sy)
                        else:                       # "rgb"
                            _rgb = img.to_rgb565()
                            _canvas.draw_image(_rgb, _ox, _oy,
                                               x_scale=_sx, y_scale=_sy)
                            del _rgb
                        _txt = ("th=%d(反向) %s 长边%.0f"
                                % (GY_TH, target_det.state,
                                   target_det.last_long))
                        _sp = None
                        if spot is not None:
                            _sp = spot["uv"]
                        if not _cv_shape:
                            # 画布上画不了 -> 叠加直接画在二值图上。
                            # 墨色随掩码方向：国一融合图是黑底（用白 255），
                            # 老路掩码是白纸（用黑 0）。且**先拷一份再画** ——
                            # 这张图可能是 _GY_MASK_CACHE 里的缓存，直接画上去
                            # 会让框一帧帧累积在缓存里。
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
                    # 彩色预览（原来的路）：缩到画布上；画布缩放不支持就推原图
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
                             "长边=%.0fpx 比%.2f 密%.2f 对比=%.0f"
                             % (FX_PX * PAPER_LONG_M / max(1.0, m[5]),
                                target_det.u, target_det.v,
                                m[5], m[6], m[7], m[8] - m[9]))
                else:
                    line += "靶=%s 候选%d th=%d %s" % (
                        target_det.state, target_det.last_n, GY_TH,
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
                    # 标定：把这段时间的光斑位置取中位数（抗单帧误检），
                    # 直接把要填的数打出来 —— 不用你算、也不用猜。
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
                # 分段耗时一行：卡在哪一步，看这行（不是看感觉）
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


main()
