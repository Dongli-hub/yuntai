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
# ⚠ 2026-10-10 整机（连着 H723 打靶）**不需要**画面显示：显示只是上位机调试用的。
#   设成 "OFF" 后：不初始化显示、不画框、不往 USB 推图 —— 这三件事一点时间都不花。
#   （要在 CanMV IDE 里看画面时，用 tools/vision_check.py，不要动这里。）
DISPLAY_MODE = "OFF"       # VIRT(只用 IDE 画面) | LCD(接了屏) | OFF(整机用这个)
DISPLAY_W = 640
DISPLAY_H = 480
DISPLAY_QUALITY = 40       # IDE 传输质量（越小越快）
SHOW_EVERY = 2             # 每几帧推一次画面（识别/瞄准仍每帧都算）
CAM_VFLIP = True           # 画面上下颠倒 -> True
CAM_HMIRROR = False        # 画面左右镜像 -> True

# --- 相机 ---
IMG_W = 640
IMG_H = 480

# --- 靶纸检测：A4 白纸亮块 + “四周更暗”校验 ---
# 实测 L(0~100): 白纸 60~70、木柜 25~35、墙 45、黑胶带 20 —— 固定阈值就能分开；
# 黑胶带框正好提供“亮块四周更暗”的校验（白瓷砖地没有这圈暗框，不会误检）。
# 现场日志：真靶纸 长边184~190px/密度0.83~0.96/对比90+；误检 密度0.51~0.73/对比22~67。
# 所以下面加了长边范围、密度、对比三道静态闸门，外加尺寸/位置/连续确认三道时间闸门。
# 详细说明见 tools/vision_check.py 顶部。
# ⚠ 2026-10-08 改回 vision-v1（"视觉初版识别"，你验证过的那版）的值：
#   之前为追"候选0"把 58 降到 50/42，但那次"候选0"的真因是相机没照到靶纸，
#   不是阈值太高；降阈值反而把墙/柜面（内125~172）也放进来 -> 误检、云台乱跟。
PAPER_TH = 58              # 亮度阈值（0~100）的**下限**；实际由 scene_th() 自适应
PAPER_TH_ALT = 72          # 兜底阈值（自适应阈值 +14 也会用到）
# ⚠ 2026-10-09 用户现场：光线变亮、距离从 0.6m 变到 1.5m（大部分 0.6m 开外）。
#   固定阈值在亮场景下会把更亮的背景也算进来 -> 改成自适应：
#     阈值 = 画面整体亮度(折算 0~100) + 13，再夹在 [55, 82]
#   （依据：log(6) 里 画面均值L≈45、纸 L≈65，45+13=58 正好是分界线 ✓）
#   同时放宽最小面积/最小边长，保证 1.5~2.2m 纸变小了也能认。
PAPER_A_MAX = 32           # |a| 上限（偏色背景会被排除）
PAPER_B_MAX = 32           # |b| 上限
PAPER_MIN_AREA = 600       # 最小像素面积（1.5m≈88x62px，2.2m≈60x42px）
PAPER_MAX_AREA_RATIO = 0.85
PAPER_MIN_LONG = 55        # 长边下限 px（1.5m≈88px、2.2m≈60px，留余量）
PAPER_MAX_LONG = 480       # 长边上限 px
PAPER_ASPECT_MIN = 0.70    # A4=1.41；斜视透视下会缩到接近 1
PAPER_ASPECT_MAX = 3.00
# ⚠ 2026-10-09 现场日志（1.0~1.5m 抓不到靶）：
#   日志里真靶纸的候选长这样 —— [14490px 框211 内180 外130 暗边83% ✗密度]、
#   [6657px 框121 内141 外49 暗边100% ✗密度]：内亮、四周是黑胶带（对比/暗边都过），
#   唯一卡住的是"密度"。原因：这些帧四边形拟合没成功，退回"外接框"算密度，
#   而**旋转过的外接框天生比纸大**（斜 45° 时只有约 0.5），0.70 这一刀把真纸全砍了。
#   → 外接框这一档放宽到 0.45（防误检还有对比>=30、暗边>=62%、连续3帧确认三道关）；
#     四边形那一档保持 0.72 不变（四边形面积是旋转不变的，本来就准）。
PAPER_DENSITY_MIN = 0.45   # 兜底（没拟合出四边形时）：像素/外接框面积
PAPER_DENSITY_QUAD_MIN = 0.72  # 拟合出四边形后：像素/四边形面积
# ⚠ 2026-10-09 现场实测（光线变亮后）：
#   真靶纸 内−外 只有 32~43（亮光下"四周"也变亮），而背景块只有 21。
#   之前设 45 会把真靶纸挡掉 -> "容易丢靶、开机难找靶"。
#   现在 30：真纸 32~43 过 ✓，背景 21 拒 ✗，防误检再靠暗边比例(0.62)+面积+确认。
PAPER_CONTRAST_MIN = 30    # 内亮度 - 外亮度（0~255 量程）
PAPER_DARK_MARGIN = 25     # 单个外侧采样点算“暗”的门槛
PAPER_DARK_FRAC_MIN = 0.62  # 外侧 12 点里至少这么多比例更暗（真纸 67~83%，背景块 58%）
# --- 四边形拟合（斜视时画出来是梯形，靶心=对角线交点=透视中心）---
QUAD_SCAN_N = 7            # 每边取几行/几列做扫描
QUAD_PERP_PX = 2           # 扫描时垂直方向各看几像素（跨过 1~2px 印刷细线）
QUAD_AREA_LO = 0.50        # 四边形面积 / 亮块像素 的合理范围
QUAD_AREA_HI = 1.35
# 扫描用的相对亮度门槛（纸面有阴影时固定阈值会把暗的那半边切掉）
SCAN_TH_K = 0.68           # 现场实证：0.5 时柜子面(≈95)会时过时不过 -> 边飞出去
SCAN_TH_LO = 65
SCAN_TH_HI = 125
QUAD_LIM_PAD = 18          # 扫描半径 = 中心到亮块该边的距离 + 这个余量
# 跟踪窗必须大于整张纸（四边形扫描要摸到四条边，窗口小了亮块会被裁掉）
TRACK_K = 0.50             # 跟踪窗 = 四边形长边 x TRACK_K + TRACK_PAD
# 跟踪窗半径 = 长边×TRACK_K + TRACK_PAD。K 不能小于 0.5（要装得下整张纸，
# 否则四边形扫描会被窗口裁掉）；能省的是"外扩量"：60 -> 35，窗口面积少 ~27%，
# 而 0.5×长边+35 仍然比整张纸（0.5×长边）大 35px，纸还是完整在窗内。
# 现场锁定后每 3 帧要在这个窗里做一次 find_blobs，这是"目标耗时"的大头。
TRACK_PAD = 35
TRACK_PAD_FAR = 90         # 远距离（纸小）时窗口外扩加大：防止"候选0"直接丢靶
ACQ_ROI_K = 0.75           # 未锁定时先搜画面中间这块（0.75=中间 3/4，少扫 44% 像素）
ACQ_FULL_EVERY = 4         # 每几帧做一次全图搜索（其余帧只搜中间区域）
SEED_EVERY = 8             # 锁定后每几帧做一次亮块搜索（5->8：帧率再提一截；
                           # 其余帧只做"四边形复测"（几毫秒），靶心照旧每帧更新）
HOLD_FRAMES = 25           # 丢靶后还画/还用多少帧（15 -> 25：靶纸闪一下不再掉框）
# 丢靶容限：丢这么多帧之后不再找靶（用户要求：比赛里丢靶=失败，不做"找靶"）。
# 25 帧≈0.8s，足够扛住"一闪过"的漏检（这段时间窗口搜索还在跑），
# 再久就说明真的丢了，直接把偏置冻住（不让它照着旧误差乱推）。
LOST_FULL = 25
FULL_EVERY = 1             # 每几帧做一次全图搜索（3 -> 1：丢靶后逐帧全图找，恢复更快）
SMOOTH = 0.55              # 平滑系数
DEADBAND_PX = 1.5          # 平滑死区（小于它不动，画面不抖）
# ⚠ 2026-10-10 两轮日志的教训：
#   太严(0.55~1.45) -> 远距离纸面碎成小块时锁不上（卡"确认1/3"）；
#   太松(0.30~2.50) -> 会锁到"碎片/阴影块"上（日志里"锁定 1.92m"实际才 1m，
#     就是锁到了半张纸），中心偏几十像素 -> 偏置被一路推到 61°、激光甩出靶外。
#   现在折中 0.45~2.0，并且把"碎片"从源头解决（find_blobs 的 margin 2->4）。
# ⚠ 2026-10-10 第三轮日志定案：远处"剧烈晃动"的真因是**锁定对象在跳** ——
#   同一段里"距离"在 0.47m(长边280px) 和 1.61m(长边82px) 之间跳 3.4 倍，
#   也就是锁定从"整张纸"切到了"碎片/激光光斑"。两个靶心差几十像素，
#   环路就在两者之间来回推 -> 激光被晃出靶外。
#   国一那份代码里对应的做法是"嵌套矩形一致性检查"(内外矩形面积比 >= 0.7)：
#   检测结果必须自洽，不能一帧一个样。我们等价的做法就是：锁定时尺寸必须连续。
SIZE_GATE_LO = 0.65        # 跟踪时允许的长边变化范围（0.45 -> 0.65：不许换目标）
SIZE_GATE_HI = 1.60
CONFIRM_N = 2              # 连续确认几次才算锁定（3->2：按用户要求"识别快一点"）
# ⚠ 2026-10-10 现场日志（远距离"必定丢靶"的真因）：
#   丢靶那一帧的候选里明明有通过的靶纸 [24711px 框219 内170 外91 暗边100% ✓]，
#   但状态卡在"确认1/3"——因为远距离时纸的白块会在"整块"和"碎片"(框67/框73)
#   之间来回跳，±35px/±60% 的一致性判据被打破，确认计数一直归零。
#   -> 位置容差放到 60px、尺寸容差放到 ±120%（碎片/整块都算同一张纸），
#      漏检容忍 3->5。防误锁仍靠 对比>=30 + 暗边>=62% + 连续3次 三道关。
CONFIRM_DXY = 60           # 确认时的位置一致范围 px（35 -> 60）
CONFIRM_DSIZE = 1.00       # 确认时的尺寸一致范围（±100%；2.0 会连"碎片"一起认进来）
PENDING_MISS = 8           # 确认过程中允许漏几次（5 -> 8）
# 目标亮块必须比"整幅画面平均亮度"亮这么多（0~255 量程）：
# 现场那些害人的假候选内亮度只有 135~151，而真靶纸是 170~245，
# 整幅均值 107~126 —— 用"比均值亮 40"这一条就能把它们全部挡掉。
PAPER_MIN_ABOVE_SCENE = 35
# 跟踪期"测量防跳"（2026-10-10 新增，治"远处测到的目标乱跳 -> 偏置被推飞"）：
#   日志实测：远距离时同一段里的"距离"能在 0.72→0.75→0.86→1.01→0.55→2.17m 之间跳，
#   说明测到的是碎片/别的东西，中心一偏几十像素，闭环就照着错误差把云台推出去。
#   ① 已经在锁定时，新测量如果相对平滑中心跳 >60px 且尺寸也大变 -> 判为误检，本次不采纳；
#   ② 靶纸小（远）时把中心/角点的平滑做强一点（0.55 -> 0.30），把抖动滤掉。
# 60 -> 100：转弯时云台滞后，靶纸在画面里可以合法地跳得很快（实测一次 167px
# 是"底盘转+云台追"的正常现象，不是误检）。100px/帧 @40fps = 4000px/s，
# 真目标不可能这么快，仍然能挡住误检。
JUMP_REJECT_PX = 100       # 跟踪期单帧中心跳变上限（超过就当成误检丢掉）
SMOOTH_FAR = 0.30          # 远距离（长边 < FAR_LONG_PX）时的平滑系数
PAPER_LONG_M = 0.297       # A4 长边实际长度（米），用于估距离

# --- 激光光斑检测 ---
SPOT_ROI_HALF = 45         # 光轴固定，只在学习到的点附近找光斑
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

# --- 瞄准控制（积分型视觉伺服 + P 项挂在实测姿态上）---
FX_PX = 445.0              # 像素焦距（640 宽）。0.6m 处 A4 长边≈222px 反推
# ⚠ 2026-10-09 方向符号【固定】（用户要求：硬件与场地都不再变，不必每次开机自检）
#   依据：连续 5 次整机测试的开机自检结果完全一致
#     log(8)(9)(10)(11)(12):  SIGN_YAW=-1   SIGN_PITCH=-1
#   以后如果换了相机安装方向 / 换了云台机械结构，把这两项改回 1.0 或
#   临时把 AUTO_SIGN_CAL 设成 True 重新自检即可。
SIGN_YAW = -1.0             # 固定方向：yaw 偏置为正 -> 靶心在图像里向负方向跑
SIGN_PITCH = -1.0           # 固定方向：pitch 偏置为正 -> 靶心在图像里向负方向跑
AUTO_SIGN_CAL = False       # 关闭开机方向自检（符号已固定，不再让它改符号）
CAL_STEP_DEG = 4.0          # 标定用的偏置步长（度）
# ---- 开机增益标定（2026-10-10 新增）----
# 用户反复质疑"像素↔角度的换算在远处不准"。之前我试过"用运行数据在线辨识"，
# 结果 Ke 全是 0/负数（车一动，辨识就被目标自身的运动淹没）——此路不通。
# 唯一可靠的办法是**静止时故意给一个已知偏置，量画面误差动了多少像素**：
#     K = |误差变化| / 偏置步长        （px/°）
# 这个标定放在"刚锁住靶、还没进闭环"的时候做（约 2 秒，只动 ±4°，和以前的
# 方向自检同样的动作），量出来的 K 只允许落在 [4,16]，然后**用它做换算**。
# 于是"这台云台 + 当前距离"的真实 px/° 就精确了，不再是猜的 7.8。
# ⚠ 2026-10-10：第一次上线的版本把标定值直接用了，结果标定自己在阶跃过渡期取样，
#   量出 K_yaw=4.4（真值 7.8）→ 环路增益被放大 1.77 倍 → 静止也开始晃。
#   现在：①默认关闭（安全第一）；②标定逻辑修好（阶跃后先停稳再取样 + 双重闸门）
#   之后想用再手动打开。换算暂时仍用几何值 7.8 + 切平面放大校正。
AUTO_GAIN_CAL = False
_SIGN = [SIGN_YAW, SIGN_PITCH]   # AimCtrl 实际使用的符号（固定值）
# ⚠ 2026-10-09 按用户意见"减小 I、增大 P"：P 0.50 -> 0.70，I 0.015 -> 0.008。
#   偏置基准改成"纯增量"之后不再有双重计数，P 大一点才跟得上移动目标。
KP_P_YAW = 0.70            # P 项
KP_P_PITCH = 0.50
# ⚠ 2026-10-10 用 [W] 日志定量后的结论：D 项是当前晃动的主因 ——
#   像素误差一门就跳 ±8px(=±1°)，一阶差分 17°/s，D 被放大成 ±0.36°/帧；
#   实测偏置 pit 以 ~10°/s 在 ±1.4° 之间来回摆（1.9Hz），就是它喂出来的。
#   现在：D 用独立的慢滤波（见 update），增益收到很小、限幅也压到 0.8°/s。
#   跟随移动目标的滞后，后面用"目标像面速度前馈"来做，而不是靠裸 D。
KP_D_YAW = 0.06            # D 项（秒）：只留一点点阻尼
KP_D_PITCH = 0.04          # 俯仰阻尼
D_FILTER = 0.15            # D 用的慢滤波系数（τ≈0.2s）
D_LIMIT_DEG = 0.8          # D 项限幅（°/s）；0.06×0.8 = 0.05°/帧，噪声再也喂不饱它
# ---- 目标像面速度前馈（2026-10-10 新增）----
# 纯 P 环路的稳态滞后 = 目标角速度 / 环路增益：现场实测追移动目标时 eu 到
# -77~-85px（≈10°）还在追，最后激光被甩出靶外。这里用一条慢滤波估计"目标在
# 画面里的移动速度"，把对应的偏置变化率直接补上去（相当于让云台先动起来），
# 而不是加大 P —— 加大 P 会把已经调好的近靶区晃动带回来。
# ⚠ 2026-10-10 现场日志定案：这段低频（0.3~0.8Hz）"远处来回摆"就是它引起的。
#   前馈 = 对"误差变化率"的补偿，但速度估计带 0.3s 滤波 -> 相位交叉点正好
#   0.53Hz：在这个频率附近它从"阻尼"变成"正反馈"，把极限环自己养住。
#   所以先**关掉**（0.6 -> 0.0）。等环路在远处也稳了，如果想补"跟随滞后"，
#   再用"只在误差大时开、且滤波更快(0.25s)"的方式重新引入。
VEL_FF = 0.0               # 前馈系数（0 = 关闭）
# ---- 瞬态闸门（2026-10-10 新增，治"转弯时反偏->回正->甩出靶"）----
# 现场机理：底盘转弯时云台先滞后 -> 画面里出现一个大而快的误差 -> 视觉环当成
# 瞄准偏差去纠 -> 转弯结束稳定环把滞后补回来 -> 刚才那份"纠偏"就变成多余偏置
# -> 激光被甩出靶外。
# 判据：误差变化率超过 GATE_PX_S（约 25°/s）就"这一帧冻住偏置"，等底盘机动结束；
# 防死锁：连续被挡超过 GATE_TIMEOUT_MS 就强制放行一次（不会像老版本那样自锁）。
# ⚠ 2026-10-10 现场：转弯时误差以 ~137px/s 上升，200 的门槛拦不住 -> 环路把
#   "底盘转弯造成的暂时误差"当成瞄准偏差，1 秒内把偏置推到 120° 限幅 ->
#   反向冲到 +244px 把靶子甩出画面。门槛降到 80px/s（≈10°/s）：
#   转弯这种快速上升一律"冻住偏置"，等机动结束、误差稳定下来再纠。
GATE_PX_S = 80.0
GATE_TIMEOUT_MS = 500.0
VEL_FILTER = 0.10          # 速度估计慢滤波（τ≈0.3s；滤波太快会被像素噪声带飞）
VEL_MAX_DPS = 30.0         # 前馈最多补多少 °/s
# ⚠ 2026-10-08 用【本机实测】的 yaw 增益，不再照抄任何人：
# ⚠ 2026-10-10 修正（用户现场："越远晃动越厉害，像像素↔角度换算不对"）：
#   这里原来用开机自检的粗糙实测值 6.0 px/°，但那次自检的离散度极大
#   （13~37px / 4°），本身就不可信。而这个镜头只要 fx=445：
#       px/° = fx × π/180 = 445 × 0.01745 ≈ 7.8 px/°
#   —— 两个轴都一样（云台"转一度"，画面就移 7.8px，与距离无关）。
#   用 6.0 的后果：控制器以为 6px 才 1°，实际 7.8px 就 1° -> 每次都多给
#   30~40% 的偏置 -> 先超调、再反向，就是"越远越晃"（远处还叠加中心抖动）。
#   现在两个轴统一用几何值 7.8（要按实测改，也必须用"稳定后"的位移去量）。
PX_PER_DEG_YAW = 7.8       # yaw：fx×π/180（与 pitch 同值，几何上就该一样）
PX_PER_DEG_PITCH = 7.8     # pitch：fx×π/180
# ⚠ 2026-10-09 模型+现场双重结论：**这个环路不要积分**。
#   用实测参数（增益 6px/度、轴滞后 0.5~1.2s）做的模型扫描：
#     KI=0.004 时，小增益(0.02~0.03)反而发散（积分攒成自激）；
#     KI=0    时，增益 0.05~0.20 + 滞后 0.3~1.2s **全部稳定收敛到 err≈0**。
#   用户也明确要求" I 尽量不要"。静差的活儿交给 H723 自己的轴内环。
KI = 0.0                   # I 项关闭（置 0 后积分不再累加）
# 积分要在"剩下几十像素静差"时把它吃掉（现场实测稳态剩 50~66px），所以：
KI_BAND_PX = 150.0         # 误差小于这么多像素才允许积分（40 -> 150）
I_LIMIT_DEG = 15.0         # 积分限幅（3 -> 15）：要能补出 10° 量级的偏置
# 死区：10 -> 6 -> 4px。用户要求"激光始终钉在靶心很小的范围内"——
# 死区就是稳态残差的量级；4px ≈ 0.5°（1m 处 0.9cm）。近靶区增益本来就压低
# （GAIN_NEAR=0.10），死区变小不会引起抖动。
DEADBAND_PX = 4.0          # 误差死区（px）
ERR_FILTER = 0.5           # 误差 EMA 滤波系数：越大越信当帧（0.5 = 一半平滑）
# --- 自适应增益/方向（2026-10-09 新增）---
# 为什么必须自适应：这台云台"偏置→像素"的响应既慢（0.5~1s）又粘（静摩擦），
# 固定增益只有两种结局——大了自激（±25° 来回甩）、小了追不上。
# 做法：每 TUNE_PERIOD_S 量一次"误差比上次大还是小"：
#   变大 -> 方向翻转 + 增益减半      （说明方向或增益不对）
#   变小 -> 保持方向 + 增益微增      （方向对，可以更积极）
#   几乎不变 -> 增益放大一点          （被静摩擦/死区卡住）
# 初始值用之前 5 次自检一致的 -1/-1，但它会自己纠。
# ⚠ 2026-10-10 用 [W] 日志定量之后的分工：
#   近靶区（|误差|<20px）已经非常稳（实测 ev ±2px、pit ±0.2°），所以那里不用再收；
#   真正的问题是"追移动目标时的滞后" —— 日志里 eu=-77px(≈10°) 还在追，
#   属于纯 P 环路的稳态差（偏置速度 = g×误差×帧率）。所以把远区增益加回去，
#   近区保持温柔：远区靠"误差大"自动切过去，不需要手动切换。
GAIN_MIN = 0.05            # 允许的最小增益
# ⚠ 2026-10-10：远处 0.3~0.8Hz 的慢摆 = 环路"增益×延迟"太大。
#   实测：近处 g=0.14 时 err 只有 3~6px（非常稳）；远处用 0.20 就开始摆。
#   所以远区不再比近区更激进 —— 统一 0.14（近靶区还有 GAIN_NEAR=0.10 的软着陆）。
GAIN_MAX = 0.14            # 远距离追击用（0.20 -> 0.14，与近处一致）
GAIN_NEAR = 0.10           # 近靶区增益上限（见 update 里的"软着陆"）
NEAR_ERR_PX = 20.0         # 误差小于它算"近靶区"
LARGE_ERR_PX = 30.0        # 误差大于它 = "落后了"：自适应这时只许加增益（见 update）
# ⚠ 2026-10-10：靶纸越小（越远）中心估计越抖 —— 同样的几像素抖动，在远距离
#   变成更大的角度误差，环路就抖。所以按靶纸长边缩放增益 + 加强平滑：
SIZE_REF_PX = 150.0        # 长边 ≥150px(≈0.9m 以内) 用满增益
SIZE_GAIN_MIN = 0.45       # 缩放下限（2m 时 ≈66px -> 0.45 倍）
FAR_LONG_PX = 120.0        # 长边小于它 = "远"，误差滤波加强
ERR_FILTER_FAR = 0.32      # 远距离误差 EMA 系数（近处用 ERR_FILTER=0.5）
TUNE_PERIOD_S = 0.30       # 多久评估一次
# ⚠ 2026-10-08 现场实测：整机下陀螺 5~12°/s（人手平移云台），限幅 8°/s 比目标
#   移动还慢 -> 跟随严重滞后、停下才慢慢回正、稳态还差 ~40px（≈5°，在 0.65m
#   就是 5~6cm，正好落在 A4 靶纸边缘）。地瓜派整定后的原值是 60°/s：
#   rdk_aim/configs/default.yaml "偏置加速度限幅（220 -> 60°/s）"。
# ⚠ 2026-10-10 现场：小车一动云台就跟不上、马上丢靶 —— 主因就在这里。
#   原来单帧步长上限 0.6°：视觉 30fps 时偏置最快只能 18°/s 地变，
#   目标像面速度一超过这个数，偏置就永远追不上（越追越落后）。
#   现在放开到 2.5°/帧（30fps ≈ 75°/s），真正的上限交给 RATE_LIMIT_DPS。
# ⚠ 2026-10-10 现场日志：转弯/目标快速移动时，误差冲到 109~177px，
#   而偏置被这两个限幅卡在 ~32°/s，靶子直接跑出画面 -> 丢靶。
#   大误差时（>CATCH_ERR_PX）环路本来就该"全力追"，不该被限速卡住。
# ⚠ 2026-10-10 现场：150°/s + 4°/帧太猛 —— 转弯时 1 秒就把偏置推到 120° 限幅，
#   反向又冲 244px 把靶子甩出画面。回到温和值；真正治转弯的是下面的"瞬态闸门"。
RATE_LIMIT_DPS = 80.0      # 偏置变化率限幅（°/s）
MAX_STEP_DEG = 2.5         # 单帧偏置最大变化（度）
# ---- 大误差"全力追"（2026-10-10 新增，同日又按现场数据关掉）----
# 现场证明：转弯时把增益抬到 0.30 -> 过冲 240px+、直接丢靶。
# 所以 GAIN_CATCH 设成和 GAIN_MAX 一样（=不额外加力），只保留闸门逻辑。
CATCH_ERR_PX = 60.0
GAIN_CATCH = 0.14
# ⚠ 2026-10-10 现场日志定案：这条"丢靶后回参考位"**必须关掉** ——
#   日志里 1.47m 处丢靶后，偏置被 5°/s 拉回 0，云台跟着转了 30°，
#   等于"自己把相机转离靶子"；找靶又已经按用户要求删掉 -> 永久丢靶。
#   正确做法：丢靶就**冻住偏置**（激光和相机留在原处，等目标自己回来），
#   绝不自己转跑。比赛里丢靶就算失败，也没有"回去重来"这回事。
LOST_RETURN_ENABLE = False
LOST_RETURN_MS = 4000      # 仅当 LOST_RETURN_ENABLE=True 时才用
LOST_RETURN_DPS = 5.0
# ⚠ 2026-10-08 首次闭环测试用的小限幅：万一方向符号反了，云台只会小幅偏一下，
#   不会甩到 170°。确认 err 收敛、方向正确后再放开（yaw 170 / pitch 60）。
#   （2026-10-09：pitch 从 25 收到 12 —— 防止偏置把靶纸推出竖直视野）
# ⚠ 2026-10-10：小车在动，目标相对上电朝向的夹角会明显超过 40°。
#   放到 120°/30°（H723 侧的偏置安全限幅是 ±180°/±80°，行程软限位 ±80°，都还兜得住）
#   —— 如果以后发现"偏置被推到限幅还不停"，先回到 40/12 再查视觉。
MAX_YAW_DEG = 120.0
MAX_PITCH_DEG = 20.0       # 30->20：俯仰偏置太大时靶纸会被推出竖直视野（远距离尤其明显）
LOST_COAST_S = 0.5         # 丢靶后还能沿用最后误差多久（1.0->0.5：别再拿旧误差瞎推）

# --- 时序 / 安全 ---
AIM_HZ = 100               # 给 H723 发 AIM 的频率（H723 控制环 200Hz；50->100 让偏置更"鲜"）
HEARTBEAT_MS = 200
MODE_REASSERT_MS = 2000    # 周期性重发 MODE（H723 可能被看门狗切回 STAB）
TELEM_TIMEOUT_MS = 1500    # 遥测断这么久 -> 主动发 STAB
LASER_ENABLE = True        # 是否让 H723 点激光（AIM 模式下才有效）
DEBUG_PRINT_MS = 1000      # 终端打印周期
LOG_TO_FILE = True         # 整机跑（没接电脑）时把日志写进 SD 卡，跑完插回电脑看
LOG_PATH = "/sdcard/k230_log.txt"
# 晃动探针（写一行 [W] 到 SD 卡）：诊断用。
# ⚠ 2026-10-09：每 2 帧一条（~15 行/s）会让 SD 卡周期性刷盘（10~50ms），
#   在 30fps 的环里就是"卡一下、偏置晚一拍"。
#   现在只写"远距离（靶纸长边<160px）"那一段、每 5 帧一条（≈6 行/s）——
#   远处才是要诊断的工况，写入量只有原来的 1/6。
WOBBLE_LOG = True
H723_TEXT_MIN_MS = 200     # H723 调试文本最快 200ms 一条（否则刷屏也占时间）
LOG_ONLY = False           # ⚠ 只监听模式（云台单独测试用）：
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


_log_f = None
_log_n = 0


def log(msg):
    """同时输出到终端(接着 USB 能实时看)和 SD 卡日志文件(没接电脑时看)。

    ⚠ 2026-10-10：以前每条都 flush，SD 卡有时会卡 10~50ms —— 在 30fps 的
    视觉环里就是"卡一下、偏置晚一拍"。现在每 8 条 flush 一次，
    退出前再兜底刷一次（见 main() 的 finally）。
    """
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


_scene_cache = [0, 0.0, 0.0]   # [帧计数, 上一次算出的阈值, 画面平均亮度(0~255)]


def scene_mean():
    """上一次算出的整幅平均亮度（0~255）。还没算过返回 -1（=这条判据先不用）。"""
    return _scene_cache[2] if _scene_cache[2] > 0 else -1.0


def scene_th(img, n_frame=0):
    """按当前帧整体亮度自适应决定"白纸"的 LAB-L 阈值（0~100）。

    亮场景（纸更亮、背景也更亮）阈值自动抬高，暗场景自动降低：
        阈值 = 整体亮度(折算 0~100) + 13，夹在 [PAPER_TH, 82]
    每 10 帧才重算一次（采样 30 个点），省时间也避免逐帧抖动。
    """
    if (n_frame % 10) != 0 and _scene_cache[1] > 0:
        return int(_scene_cache[1])
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
    mean_l100 = (s / float(k)) * 100.0 / 255.0
    _scene_cache[2] = s / float(k)          # 0~255 均值，给"够不够亮"判据用
    t = mean_l100 + 13.0
    if t < PAPER_TH:
        t = PAPER_TH
    elif t > 82.0:
        t = 82.0
    _scene_cache[1] = t
    return int(t)


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
        log("[SCENE] 画面亮度 均=%.0f 小=%d 大=%d (0~255)  靶=无靶时看这行"
            % (s / float(k), lo, hi))


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
        # 上一帧"被采纳"的四边形长边：尺寸闸门必须拿同一个量比较
        # （以前拿的是外接框 max(w,h)，和 quad 长边不是一个量 -> 退远时误杀，见 _size_ok）
        self.last_long = 0.0

    def roi(self):
        # 窗口半径 = 长边×TRACK_K + 外扩量。
        # ⚠ 2026-10-10 现场日志：1.47m 丢靶那一帧是"候选0"——窗口里一个亮块
        #   都没有。远处靶纸小、窗口半径只有 0.5×88+35 = 79px，手一推/一窜，
        #   画面里的目标就能跳出去。所以远处（L<200px）把外扩量线性加大到
        #   TRACK_PAD_FAR；近处（纸大）窗口本来就够大，保持 35 省时间。
        L = max(self.w, self.h)
        if L < 200.0:
            k = (200.0 - L) / 140.0
            if k > 1.0:
                k = 1.0
            elif k < 0.0:
                k = 0.0
            pad = TRACK_PAD + (TRACK_PAD_FAR - TRACK_PAD) * k
        else:
            pad = TRACK_PAD
        r = int(L * TRACK_K) + int(pad)
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
        # ⚠ 2026-10-10 逻辑修正：以前拿 max(w,h)（**外接框**长边）来比，
        #   而 long_side 是**四边形**长边 —— 两个量在斜视/退远时差很多：
        #   日志实例：外接框还停在 180px，四边形长边已缩到 100px，
        #   结果 100 < 0.65×180 被当成"换目标"丢掉 -> 直接丢靶。
        #   改成和"上一帧被采纳的四边形长边"比（同一种量）。
        old = self.last_long if self.last_long > 1.0 else max(self.w, self.h)
        lo = SIZE_GATE_LO
        hi = SIZE_GATE_HI
        if self.lost > 0:
            # 丢靶滑行期间闸门放宽（2026-10-10）：车在动/正在退远时靶纸尺寸变化快，
            # 跟踪对象又是"上一帧被采纳的尺寸"，卡太紧会把真靶纸判成"换目标"。
            # 每多丢一帧多放 2%，最多放到 0.6 倍系数。
            k = 1.0 - 0.02 * self.lost
            if k < 0.6:
                k = 0.6
            lo = SIZE_GATE_LO * k
            hi = SIZE_GATE_HI / k
        return (lo * old) <= long_side <= (hi * old)

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
        c = cand[10]
        cu, cv = cand[11], cand[12]      # 四边形对角线交点 = 透视中心
        # ---- 防"测到的目标乱跳"（见 JUMP_REJECT_PX 说明）----
        # 已经锁着的时候，单帧中心突然跑 60px 以上、尺寸还同时大变 —— 那不是
        # 目标在动（30fps 下相当于 1800px/s），是误检。直接丢，当这次没测到。
        if self.have and (self.lost == 0):
            if abs(cu - self.u) > JUMP_REJECT_PX:
                return False
            if abs(cv - self.v) > JUMP_REJECT_PX:
                return False
            # 尺寸也一样：一帧之内长边变 45% 以上 = 换目标了（整张纸 <-> 碎片/
            # 激光光斑），必须丢掉。真实目标一帧的尺寸变化只有几个百分点。
            old_l = self.last_long if self.last_long > 1.0 \
                else max(self.w, self.h)
            if old_l > 1.0:
                if (cand[5] < 0.55 * old_l) or (cand[5] > 1.80 * old_l):
                    return False
        if not self.have:
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
            # 四边形四角也做时间平滑：现场"比"逐帧在 1.26<->1.44 来回跳，
            # 画出来就抖；平滑后绿框稳定（靶心用 u,v，本来就平滑）
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
        self.last_long = cand[5]      # 记住这次采纳的四边形长边（给尺寸闸门用）
        return True

    def detect(self, img):
        self.frame += 1
        # 自适应亮度阈值（亮场景抬高、暗场景降低），再算一个兜底档
        th_ad = scene_th(img, self.frame)
        th_alt = th_ad + 14
        if th_alt > 92:
            th_alt = 92
        cand = None
        n = 0
        dbg = ""
        self.did_full = False
        try:
            if self.have and self.lost < LOST_FULL:
                if ((self.frame % SEED_EVERY) == 0) or (self.meas is None):
                    cand, n, dbg = self._find(img, self.roi(), th_ad)
                else:
                    # 隔帧只做四边形复测（省 ~13ms），靶心仍每帧更新；
                    # 复测失败立刻补一次完整搜索。
                    cand, n, dbg = self._find(img, self.roi(), th_ad,
                                              self.meas)
                    if cand is None:
                        cand, n, dbg = self._find(img, self.roi(), th_ad)
                if (cand is not None) and (not self._size_ok(cand[5])):
                    dbg = "[尺寸闸门%.0fpx] " % cand[5] + dbg
                    cand = None
                # ⚠ 2026-10-10：这里原来还有一条"窗口失败 -> 中间区域/全图再搜一次"。
                #   用户明确要求：比赛里丢靶就算失败，main.py 不做"找靶"。
                #   把它删掉有两个好处：
                #   ① 省掉每帧最多两次全图 find_blobs（远处"目标耗时 100ms"就是它）；
                #   ② 不会再出现"拿着错误差满图乱找 -> 偏置被推飞"的剧烈晃动。
                #   短暂漏检（<LOST_FULL 帧）仍由上面的窗口搜索兜住。
            elif (not self.have) and ((self.frame % FULL_EVERY) == 0):
                # 只有"从没锁定过"（开机第一次找靶）才做搜索：
                # 先搜画面中间 ACQ_ROI_K 那块（少扫近一半像素 -> 快一截），
                # 每 ACQ_FULL_EVERY 帧再全图搜一次，边角上的靶子也不会漏。
                if (self.frame % ACQ_FULL_EVERY) == 0:
                    self.did_full = True
                    roi2 = (0, 0, IMG_W, IMG_H)
                else:
                    self.did_full = False
                    rw = int(IMG_W * ACQ_ROI_K)
                    rh = int(IMG_H * ACQ_ROI_K)
                    roi2 = ((IMG_W - rw) // 2, (IMG_H - rh) // 2, rw, rh)
                c2, n2, d2 = self._find(img, roi2, th_ad)
                if c2 is None and ((self.frame % ACQ_FULL_EVERY) == 0):
                    # 兜底阈值档只在全图那一帧再跑，省一次全图 find_blobs
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
                # 判为"误检"（中心跳太远）：这一帧按"没测到"处理，
                # 但平滑中心保持不变（滑行），避免被错误目标把云台推走。
                cand = None
        if cand is None:
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
            # margin：2026-10-10 2 -> 4。远距离时纸面被印刷圈/阴影切成碎片，
            # 间隙只有 1~3px，margin=2 合不起来（日志里全是"框73/81px 的碎片"）；
            # 而黑胶带在 1.3m 还有 ~7px 宽、2m 也有 ~4px，margin=4 不会把纸
            # 和背景粘在一起（当初 6 才会）。这是"整块/碎片来回跳"的正解。
            roi=roi, merge=True, margin=4,
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
            why = ""
            corners = None
            qa = 0.0
            ctr = (x + w / 2.0, y + h / 2.0)
            if box_long < PAPER_MIN_LONG * 0.8 or \
                    box_long > PAPER_MAX_LONG * 1.3:
                why = "框长"
                long_side = box_long
                short_side = w if w < h else h
                density = px / float(w * h)
            else:
                q = quad_from_blob(img, x, y, w, h, _scan_th(ins))
                if q is not None:
                    corners, ctr, long_side, short_side, qa = q
                    if (qa < QUAD_AREA_LO * px) or (qa > QUAD_AREA_HI * px):
                        corners = None        # 拟合和亮块对不上，弃用
                    else:
                        # 角点不能跑到亮块外框太远（否则就是某条边越过胶带摸到
                        # 背景亮边）。超了就退回外接框：宁可稳的正矩形，
                        # 也不要乱跳的梯形。
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
                elif density < dens_min:
                    why = "密度"
                elif ((x <= 1) or (y <= 1) or
                      ((x + w) >= IMG_W - 1) or ((y + h) >= IMG_H - 1)) and \
                        (corners is None):
                    # 贴边不再一票否决（2026-10-10 现场：转弯时靶纸被转到画面边缘，
                    # 唯一的真候选就因为"贴边"被丢掉 -> 永久丢靶）。
                    # 只要四边形拟合成功（=四个角都在画面内、对角线交点可信）就采纳；
                    # 只有"贴边 + 四边形没拟合出来"（中心可能是裁掉之后的偏心值）才丢弃。
                    why = "贴边"
                elif (scene_mean() > 0) and \
                        ((ins - scene_mean()) < PAPER_MIN_ABOVE_SCENE):
                    why = "不够亮"          # 阴影块/碎片冒充靶纸时挡在这
                elif ins < 0 or (ins - outs) < PAPER_CONTRAST_MIN:
                    why = "对比"
                elif frac < PAPER_DARK_FRAC_MIN:
                    why = "暗边"
            if n <= 6:
                dbg += "[%dpx 框%.0f 内%d 外%d 暗边%.0f%%%s] " % (
                    px, box_long, ins, outs, frac * 100.0,
                    (" ✗" + why) if why else " ✓")
            if why:
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
class SignCal(object):
    """开机方向自检（治"云台朝反方向跑"）：

    步骤：① 稳稳锁住靶后，先测 0.4s 的基准误差(基准=平均 err_u/err_v)；
          ② 只给 yaw 一个 +CAL_STEP_DEG 的小偏置，0.7s 后再测平均误差 —— 误差
             变小说明这个方向对（保持 SIGN_YAW），变大说明反了（取反）；
          ③ 对 pitch 做同样的事。
    全程只动 ±4°，靶不会跑出视野；中途丢靶就放弃自检、保持原符号并打日志。
    phase: 0 等目标 / 1 测基准 / 2 测 yaw / 3 测 pitch / 4 完成
    """

    def __init__(self):
        self.phase = 0
        self.t = 0
        self.n = 0
        self.su = 0.0
        self.sv = 0.0
        self.base_u = 0.0
        self.base_v = 0.0
        self.yaw_bias = 0.0
        self.pit_bias = 0.0
        # 标定出来的真实增益（px/°）：0 = 没量到
        self.k_yaw = 0.0
        self.k_pit = 0.0

    def done(self):
        return self.phase >= 4

    def step(self, now, eu, ev, ok):
        """返回 (yaw偏置, pitch偏置, 是否正在自检)。"""
        if self.phase >= 4:
            return 0.0, 0.0, False
        if not ok:                       # 丢靶/不稳 -> 这一帧不参与，稍后继续
            return self.yaw_bias, self.pit_bias, (self.phase != 0)
        if self.phase == 0:
            self.phase = 1
            self.t = now
            self.n = 0
            self.su = 0.0
            self.sv = 0.0
            log("[CAL] 方向自检开始（约 2 秒，请让靶纸保持在画面里）")
        d = time.ticks_diff(now, self.t)
        # 阶跃后先让云台停稳（300ms）再取样：否则把"过渡过程"也平均进去，
        # 量到的位移偏小 -> K 偏小 -> 环路被放大（现场就是这么晃起来的）。
        if (self.phase >= 2) and (d < 300):
            return self.yaw_bias, self.pit_bias, True
        self.su += eu
        self.sv += ev
        self.n += 1
        if self.phase == 1:
            self.yaw_bias = 0.0
            self.pit_bias = 0.0
            if d >= 400:
                self.base_u = self.su / max(1, self.n)
                self.base_v = self.sv / max(1, self.n)
                self.phase = 2
                self.t = now
                self.n = 0
                self.su = 0.0
                self.sv = 0.0
                log("[CAL] 基准: err_u=%.0f err_v=%.0f" % (self.base_u,
                                                           self.base_v))
        elif self.phase == 2:
            self.yaw_bias = CAL_STEP_DEG
            self.pit_bias = 0.0
            if d >= 1000:      # 700 -> 1000：等云台彻底停稳再取平均（阶跃后
                               # 若没停稳，量到的像素变化偏小 -> K 估小 -> 环路偏热）
                mu = self.su / max(1, self.n)
                # 符号不再自动改（用户要求固定）；这里只量"误差动了多少像素"。
                self.k_yaw = abs(mu - self.base_u) / CAL_STEP_DEG
                # 双重闸门：① 误差必须变小（符号对）② 量到的 K 必须落在几何值
                # 附近 [6.5,10]；否则判为"这次没量准"，不采用（保持 7.8）。
                if (abs(mu) >= abs(self.base_u)) or \
                        (self.k_yaw < 6.5) or (self.k_yaw > 10.0):
                    log("[CAL] 这次 yaw 标定不可信(K=%.1f)，丢弃，保持几何值"
                        % self.k_yaw)
                    self.k_yaw = 0.0
                log("[CAL] yaw +%.0f度: err_u %.0f -> %.0f  => K_yaw=%.1f px/度 "
                    "(几何值 7.8，符号%s)"
                    % (CAL_STEP_DEG, self.base_u, mu, self.k_yaw,
                       "正确" if abs(mu) < abs(self.base_u) else "可疑!"))
                self.phase = 3
                self.t = now
                self.n = 0
                self.su = 0.0
                self.sv = 0.0
        elif self.phase == 3:
            self.yaw_bias = 0.0
            self.pit_bias = CAL_STEP_DEG
            if d >= 1000:
                mv = self.sv / max(1, self.n)
                self.k_pit = abs(mv - self.base_v) / CAL_STEP_DEG
                if (abs(mv) >= abs(self.base_v)) or \
                        (self.k_pit < 6.5) or (self.k_pit > 10.0):
                    log("[CAL] 这次 pitch 标定不可信(K=%.1f)，丢弃，保持几何值"
                        % self.k_pit)
                    self.k_pit = 0.0
                log("[CAL] pitch +%.0f度: err_v %.0f -> %.0f  => K_pitch=%.1f px/度 "
                    "(几何值 7.8，符号%s)"
                    % (CAL_STEP_DEG, self.base_v, mv, self.k_pit,
                       "正确" if abs(mv) < abs(self.base_v) else "可疑!"))
                log("[CAL] 标定完成，进入闭环瞄准")
                self.phase = 4
                self.yaw_bias = 0.0
                self.pit_bias = 0.0
        return self.yaw_bias, self.pit_bias, (self.phase < 4)


class AimCtrl(object):
    def __init__(self):
        self.yaw = 0.0
        self.pitch = 0.0
        self.i_u = 0.0
        self.i_v = 0.0
        self.prev_eu = 0.0
        self.prev_ev = 0.0
        self.last_eu = 0.0
        self.last_ev = 0.0
        self.f_u = 0.0
        self.f_v = 0.0
        self.f_init = False
        # D 项专用慢滤波（与上面给 P 用的 EMA 分开，见 update 里的说明）
        self.df_u = 0.0
        self.df_v = 0.0
        self.df_init = False
        # 目标像面速度（前馈用）
        self.v_u = 0.0
        self.v_v = 0.0
        self.vf_init = False
        self.fu_prev = 0.0
        self.fv_prev = 0.0
        # 自适应增益/方向：g[i]=增益, sg[i]=方向(±1), la[i]=上次评估的|误差|
        self.g = [0.10, 0.08]      # 初值略提高响应（自适应会自己微调）
        self.sg = [_SIGN[0], _SIGN[1]]
        self.la = [None, None]
        self.t_acc = 0.0
        self.err_px = 0.0
        self.valid = False
        self.locked = False
        # ---- 在线辨识"偏置→像素"的真实增益 K（px/°）----
        # 物理关系：偏置动 Δo 度，画面误差应当**反向**动 K·Δo 像素。
        # 用最近约 2 秒的数据做最小二乘： K = -Σ(Δo·Δe)/Σ(Δo²)
        # 这样换算用的 K 就是"这台云台+这个距离"的实测值，而不是固定 7.8。
        self.k_px = PX_PER_DEG_YAW       # yaw 当前使用的 px/°（随辨识更新）
        self.k_px_v = PX_PER_DEG_PITCH   # pitch
        self.k_est = PX_PER_DEG_YAW      # 辨识出的原始估计（只用于日志观察）
        self.k_est_v = PX_PER_DEG_PITCH
        self.k_xy = 0.0
        self.k_xx = 0.0
        self.k_prev_o = 0.0
        self.k_prev_e = 0.0
        self.k_xy_v = 0.0
        self.k_xx_v = 0.0
        self.k_prev_ov = 0.0
        self.k_prev_ev = 0.0
        # 瞬态闸门用：误差变化率（px/s，轻滤波）+ 连续被挡时长
        self.rate_u = 0.0
        self.rate_v = 0.0
        self.r_prev_u = 0.0
        self.r_prev_v = 0.0
        self.gate_ms = 0.0
        # 目标在画面里的绝对位置（主循环每帧写入）：用来做"镜头切平面校正"
        self.tgt_u = IMG_W * 0.5
        self.tgt_v = IMG_H * 0.5
        self.sec2_u = 1.0      # 当前生效的放大系数（=1+(Δu/fx)²）
        self.sec2_v = 1.0

    def reset(self):
        self.i_u = 0.0
        self.i_v = 0.0
        self.f_init = False
        self.df_init = False
        self.vf_init = False
        self.v_u = 0.0
        self.v_v = 0.0

    def update(self, dt, err_u, err_v, att_rel, long_side=0.0):
        """err_u/err_v = 靶心 - 光斑（像素）；att_rel = 相对锁零基准的实测姿态；
        long_side = 靶纸四边形长边（px），用来做"远距离自动变温柔"。"""
        self.err_px = math.sqrt(err_u * err_u + err_v * err_v)
        self.last_eu = err_u
        self.last_ev = err_v
        # ---- 在线辨识"偏置→像素"真实增益 K（px/°），见 __init__ 里的说明 ----
        # 用【上一帧到现在】的偏置变化 Δo 和误差变化 Δe 做最小二乘（滑动窗口，
        # 每累计够一次就衰减一半，能在 2~3 秒内跟上距离变化）。
        do = self.yaw - self.k_prev_o
        de = err_u - self.k_prev_e
        self.k_xy += do * de
        self.k_xx += do * do
        self.k_prev_o, self.k_prev_e = self.yaw, err_u
        if self.k_xx > 20.0:
            kk = -self.k_xy / self.k_xx
            # ⚠ 2026-10-10 结论：**辨识值只做展示，不再参与换算**。
            #   现场两份日志里 Ke 全是 0 或负数（车的运动把"偏置->像素"的因果
            #   淹没了），一旦夹到 6.5 就会把换算从几何真值 7.8 一路拖到 6.5
            #   —— 那等于把环路增益偷偷放大 20%，只会更晃。
            #   所以：换算固定用几何值 7.8（已验证近处 2~6px 收敛），
            #   Ke 仍然打进日志，等哪天有"静止+小幅激励"的数据再决定要不要用。
            self.k_est = kk
            self.k_xy *= 0.5
            self.k_xx *= 0.5
        do = self.pitch - self.k_prev_ov
        de = err_v - self.k_prev_ev
        self.k_xy_v += do * de
        self.k_xx_v += do * do
        self.k_prev_ov, self.k_prev_ev = self.pitch, err_v
        if self.k_xx_v > 20.0:
            kk = -self.k_xy_v / self.k_xx_v
            self.k_est_v = kk
            self.k_xy_v *= 0.5
            self.k_xx_v *= 0.5
        self.long_side = long_side
        # 距离缩放系数：1.0（近）…… SIZE_GAIN_MIN（远）
        if long_side > 0.0:
            k_size = long_side / SIZE_REF_PX
            if k_size > 1.0:
                k_size = 1.0
            elif k_size < SIZE_GAIN_MIN:
                k_size = SIZE_GAIN_MIN
        else:
            k_size = 1.0
        # 误差 EMA 滤波（治"检测抖动 -> 环路被抖动带着抖"）：
        # 用滤波后的值去算角度，原始值只用于日志显示。
        # 远距离（靶纸小、中心抖）时把滤波加强一点：反正环路本来就该慢。
        fk = ERR_FILTER_FAR if (0.0 < long_side < FAR_LONG_PX) else ERR_FILTER
        if not self.f_init:
            self.f_u, self.f_v = err_u, err_v
            self.f_init = True
        else:
            self.f_u += fk * (err_u - self.f_u)
            self.f_v += fk * (err_v - self.f_v)
        # ---- 目标像面速度估计（前馈用；用"滤波后的误差"再慢滤一次，见 VEL_FF）----
        if dt > 1e-4:
            if not self.vf_init:
                self.fu_prev, self.fv_prev = self.f_u, self.f_v
                self.vf_init = True
            else:
                self.v_u += VEL_FILTER * ((self.f_u - self.fu_prev) / dt -
                                          self.v_u)
                self.v_v += VEL_FILTER * ((self.f_v - self.fv_prev) / dt -
                                          self.v_v)
                self.fu_prev, self.fv_prev = self.f_u, self.f_v
                vmax = VEL_MAX_DPS * PX_PER_DEG_YAW
                self.v_u = max(-vmax, min(vmax, self.v_u))
                vmax = VEL_MAX_DPS * PX_PER_DEG_PITCH
                self.v_v = max(-vmax, min(vmax, self.v_v))
            # 停在靶心附近时（误差进了死区）让速度估计自己衰减 ——
            # 否则像素噪声会在 v 上积成随机游走，偏置慢慢飘（现场：40 秒飘了 5°）。
            if abs(self.f_u) < DEADBAND_PX:
                self.v_u *= 0.90
            if abs(self.f_v) < DEADBAND_PX:
                self.v_v *= 0.90
        err_u = self.f_u
        err_v = self.f_v
        # ---- 瞬态闸门（见 GATE_PX_S 说明）----
        if dt > 1e-4:
            ru = (err_u - self.r_prev_u) / dt
            rv = (err_v - self.r_prev_v) / dt
        else:
            ru = 0.0
            rv = 0.0
        self.r_prev_u, self.r_prev_v = err_u, err_v
        self.rate_u += 0.5 * (ru - self.rate_u)
        self.rate_v += 0.5 * (rv - self.rate_v)
        if (abs(self.rate_u) > GATE_PX_S) or (abs(self.rate_v) > GATE_PX_S):
            self.gate_ms += dt * 1000.0
            if self.gate_ms < GATE_TIMEOUT_MS:
                # 底盘机动中：偏置冻住不动（稳定环会自己把滞后补回来）
                self.locked = False
                return
        else:
            self.gate_ms = 0.0
        eu, ev = err_u, err_v
        if abs(eu) < DEADBAND_PX:
            eu = 0.0
        if abs(ev) < DEADBAND_PX:
            ev = 0.0

        # I 项用"没进死区前"的原始误差：死区只用来抑制 P 项抖动，
        # 不能让积分也停住，否则最后几像素的静差永远消不掉。
        # 像素→角度：用**在线辨识出的真实 K**（初值是几何值 fx×π/180 = 7.8）。
        # 关键：这个数偏大 -> 环路偏软（慢）；偏小 -> 环路偏硬（过冲、左右/上下晃）。
        # 现场一直怀疑"远处换算不准"，所以这里不再写死，让数据自己定。
        # ---- 镜头切平面校正（2026-10-10 新增，治"远处被放大的换算误差"）----
        # 切平面投影下，局部"像素/度"不是常数：
        #     du/dθ = fx·sec²(α),  α = atan(Δu/fx),  Δu = 目标离画面中心多少像素
        # 画面中心 = fx·π/180 = 7.8 px/°；偏离 200px -> 9.4 (+20%)；
        # 偏离 280px(画面边) -> 10.8 (+40%)。远距离时靶纸常被甩到画面边上，
        # 环路按 7.8 算就等于把自己放大 20~40% -> 过冲 -> 振荡。
        du_off = (self.tgt_u - IMG_W * 0.5) / FX_PX
        self.sec2_u = 1.0 + du_off * du_off
        dv_off = (self.tgt_v - IMG_H * 0.5) / FX_PX
        self.sec2_v = 1.0 + dv_off * dv_off
        k_eff_u = self.k_px * self.sec2_u
        k_eff_v = self.k_px_v * self.sec2_v
        self.k_eff_u = k_eff_u
        self.k_eff_v = k_eff_v
        e_deg_u_raw = err_u / k_eff_u
        e_deg_v_raw = err_v / k_eff_v
        e_deg_u = eu / k_eff_u
        e_deg_v = ev / k_eff_v

        # D 项：误差变化率阻尼 —— 必须"再重滤一次"后才求导。
        # ⚠ 2026-10-10 现场定量结论：像素误差帧间跳 ±8px(=±1°) 时，直接差分是
        #   17°/s，D 项被放大到 ±0.36°/帧 —— 实测偏置 pit 正是以 ~10°/s 在
        #   ±1.4° 之间摆（1.9Hz）。所以 D 走一条独立的慢滤波（D_FILTER，τ≈0.2s），
        #   再加 0.8°/s 的硬限幅，噪声再也喂不饱它。
        if not self.df_init:
            self.df_u, self.df_v = e_deg_u_raw, e_deg_v_raw
            self.df_init = True
        else:
            self.df_u += D_FILTER * (e_deg_u_raw - self.df_u)
            self.df_v += D_FILTER * (e_deg_v_raw - self.df_v)
        d_u = 0.0
        d_v = 0.0
        if dt > 1e-4:
            d_u = (self.df_u - self.prev_eu) / dt
            d_u = max(-D_LIMIT_DEG, min(D_LIMIT_DEG, d_u))
            d_v = (self.df_v - self.prev_ev) / dt
            d_v = max(-D_LIMIT_DEG, min(D_LIMIT_DEG, d_v))
        self.prev_eu = self.df_u
        self.prev_ev = self.df_v

        # 条件积分：误差小才累加，避免"攒过头再冲出去"
        if self.err_px <= KI_BAND_PX and dt > 1e-4:
            # ⚠ 2026-10-09 现场抓到的硬 bug：这里原来写的是
            #     i_u += KI * e_deg_u_raw        （与误差同号）
            #   而比例项是  base + sg * g * e_deg  （sg=-1，与误差反号）
            #   => **积分一直在往比例项的反方向拉**，越积越大，把云台推到
            #   偏置限幅（现场日志：ev=-43 -> pit 却被推到 -12° 限幅），
            #   表现就是"无论怎么调 P 都不收敛 / 直接脱靶"。
            #   正确：积分必须与比例项同方向，乘上自适应方向 sg。
            self.i_u += self.sg[0] * KI * e_deg_u_raw
            self.i_v += self.sg[1] * KI * e_deg_v_raw
            self.i_u = max(-I_LIMIT_DEG, min(I_LIMIT_DEG, self.i_u))
            self.i_v = max(-I_LIMIT_DEG, min(I_LIMIT_DEG, self.i_v))

        # ⚠ 2026-10-09 重要修正（用户指出）：这里过去写的是
        #     base_u = att_rel[0] + i_u   ← att_rel 是【当前实测姿态】
        #   于是 want_u = 当前姿态 + Kp×误差，再当成"偏置"发给 H723；
        #   而 H723 的用法是 target = 上电基准 + 偏置 —— 当前姿态被重复
        #   计入了一次（双重计数），所以怎么调都收敛不好。
        #   正确做法：K230 只负责【增量修正量】，也就是在它自己累积的偏置
        #   self.yaw/self.pitch 上继续加修正；当前姿态由 H723 自己的稳定环
        #   负责（它 200Hz 在跑，比我们快得多）。att_rel 只用于日志/调试。
        base_u = self.yaw + self.i_u
        base_v = self.pitch + self.i_v

        # ---- 自适应：每 TUNE_PERIOD_S 评估一次"误差是变大还是变小" ----
        self.t_acc += dt
        if self.t_acc >= TUNE_PERIOD_S:
            self.t_acc = 0.0
            for i in (0, 1):
                cur = abs(e_deg_u) if i == 0 else abs(e_deg_v)
                last = self.la[i]
                if (last is not None) and (last > 1e-3):
                    err_px = (abs(e_deg_u) * self.k_eff_u) if i == 0 \
                        else (abs(e_deg_v) * self.k_eff_v)
                    if cur > last * 1.05:
                        # 误差变大：分两种情况
                        if err_px > LARGE_ERR_PX:
                            # 落后很多 = 在追移动目标，不是自激 -> 加增益
                            self.g[i] = min(GAIN_MAX, self.g[i] * 1.25)
                        else:
                            # 误差很小却还在变大才算自激 -> 收一点
                            self.g[i] = max(GAIN_MIN, self.g[i] * 0.85)
                    elif cur < last * 0.95:        # 在收敛 -> 保持并微增
                        self.g[i] = min(GAIN_MAX, self.g[i] * 1.15)
                    else:                          # 卡住(静摩擦) -> 加大步长
                        self.g[i] = min(GAIN_MAX, self.g[i] * 1.25)
                self.la[i] = cur

        # ---- 远快近稳（"软着陆"）----
        # 现场现象：激光在靶心左右来回晃。原因是外环"一帧跨过靶心、下一帧又
        # 反向"：延迟 ~150~200ms 时增益 0.3 已经太大。所以误差进到 NEAR_ERR_PX
        # 以内就把增益压到 GAIN_NEAR —— 远处照样快追，近了自动变温柔。
        g_u = self.g[0]
        g_v = self.g[1]
        if abs(e_deg_u) * self.k_eff_u < NEAR_ERR_PX:
            g_u = min(g_u, GAIN_NEAR)
        if abs(e_deg_v) * self.k_eff_v < NEAR_ERR_PX:
            g_v = min(g_v, GAIN_NEAR)
        # ---- 距离缩放（靶纸越小越温柔）----
        # 实测证据（2026-10-10 [W]）：同一个增益 g=0.14 在 0.5m 稳如泰山
        # （err 2~6px），到 1.5m 就出现 0.2Hz、±5° 的大幅摆动 —— 远处靶纸小，
        # 中心估计更抖、跟踪平滑更重(滞后更大)，环路能容忍的增益更低。
        if long_side > 0.0:
            k_far = 0.35 + 0.65 * min(1.0, long_side / 200.0)
        else:
            k_far = 1.0
        g_u *= k_size * k_far
        g_v *= k_size * k_far
        # ---- 大误差"全力追"（见 CATCH_ERR_PX 说明）----
        if abs(e_deg_u) * self.k_eff_u > CATCH_ERR_PX:
            if GAIN_CATCH > g_u:
                g_u = GAIN_CATCH
        if abs(e_deg_v) * self.k_eff_v > CATCH_ERR_PX:
            if GAIN_CATCH > g_v:
                g_v = GAIN_CATCH
        want_u = base_u + self.sg[0] * (g_u * e_deg_u + KP_D_YAW * d_u)
        want_v = base_v + self.sg[1] * (g_v * e_deg_v + KP_D_PITCH * d_v)

        # 变化率限幅（防甩） + 绝对限幅
        # 每帧偏置变化量双限幅：既限速度，也限单帧步长（防"延迟导致的大跳"）
        step = RATE_LIMIT_DPS * max(dt, 1e-3)
        if step > MAX_STEP_DEG:
            step = MAX_STEP_DEG
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
        # ---- 速度前馈增量（见 VEL_FF 说明）----
        ff_u = self.sg[0] * VEL_FF * (self.v_u / self.k_eff_u) * max(dt, 1e-3)
        ff_v = self.sg[1] * VEL_FF * (self.v_v / self.k_eff_v) * max(dt, 1e-3)
        if ff_u > step:
            ff_u = step
        elif ff_u < -step:
            ff_u = -step
        if ff_v > step:
            ff_v = step
        elif ff_v < -step:
            ff_v = -step
        self.yaw = max(-MAX_YAW_DEG, min(MAX_YAW_DEG,
                                        self.yaw + du + ff_u))
        self.pitch = max(-MAX_PITCH_DEG, min(MAX_PITCH_DEG,
                                            self.pitch + dv + ff_v))
        self.locked = self.err_px < DEADBAND_PX


# ============================================================================
#  显示 / 相机
# ============================================================================
def init_camera():
    from media.sensor import Sensor, CAM_CHN_ID_0
    sensor = Sensor(id=2, width=1280, height=960, fps=90)
    sensor.reset()
    time.sleep_ms(100)
    apply_flip(sensor)
    sensor.set_framesize(width=IMG_W, height=IMG_H, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
    apply_flip(sensor)
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
    sign_cal = SignCal()
    cal_active = False
    cal_yaw = 0.0
    cal_pit = 0.0
    cal_gain_done = False

    state = ST_WAIT_READY
    state_t = time.ticks_ms()
    att_zero = None
    gz = None
    last_gz_t = 0
    t_hb = time.ticks_ms()
    t_aim = time.ticks_ms()
    t_mode = time.ticks_ms()
    t_print = time.ticks_ms()
    t_diag = time.ticks_ms()
    t_last_frame = time.ticks_ms()
    last_tgt = None
    last_tgt_t = 0
    lock_run = 0
    last_err = None
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
    sum_n = 0
    t_h723 = 0

    print("=" * 60)
    print("K230 瞄准程序启动  %dx%d  靶纸=白纸亮块+暗框  显示=%s"
          % (IMG_W, IMG_H, DISPLAY_MODE))
    print("=" * 60)
    global _log_f
    if LOG_TO_FILE:
        try:
            _log_f = open(LOG_PATH, "w")
            log("(日志同时写入 %s；整机跑完把 SD 卡插回电脑看这个文件)" % LOG_PATH)
            log("[CFG] 方向符号已固定: SIGN_YAW=%+.0f SIGN_PITCH=%+.0f "
                "(自检%s)  偏置限幅 yaw=%.0f° pitch=%.0f°"
                % (SIGN_YAW, SIGN_PITCH,
                   "开" if AUTO_SIGN_CAL else "关", MAX_YAW_DEG,
                   MAX_PITCH_DEG))
        except Exception as e:
            print("日志文件打不开(继续运行, 只是不写文件): %s" % e)

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
                        # 限速：H723 的 [DBG] 文本最快 20 条/s，每条都要 print +
                        # 写 SD 卡，刷屏也会吃掉几毫秒。限到 200ms 一条。
                        if time.ticks_diff(now, t_h723) >= H723_TEXT_MIN_MS:
                            t_h723 = now
                            try:
                                log("H723: %s" % payload.decode("utf-8"))
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
                if (not LOG_ONLY) and \
                        (time.ticks_diff(now, t_mode) >= MODE_REASSERT_MS):
                    t_mode = now
                    link.send(build_frame(MSG_MODE, pack_mode(MODE_AIM)))

                dt = time.ticks_diff(now, t_last_frame) / 1000.0
                t_last_frame = now
                t0 = time.ticks_ms()
                img = sensor.snapshot()
                t_grab = time.ticks_diff(time.ticks_ms(), t0)
                n_frames += 1

                # 每 5 秒打一次画面亮度：无靶时用它判断"是没看到靶还是判据问题"
                if (target_det.lost > 0) and \
                        (time.ticks_diff(now, t_diag) >= 5000):
                    t_diag = now
                    log_scene(img)

                t0 = time.ticks_ms()
                res = target_det.detect(img)
                t_tgt = time.ticks_diff(time.ticks_ms(), t0)
                t0 = time.ticks_ms()
                if (n_frames % SPOT_EVERY) == 0:
                    last_spot = spot_det.detect(img)
                t_spot = time.ticks_diff(time.ticks_ms(), t0)
                spot = last_spot
                sum_grab += t_grab
                sum_tgt += t_tgt
                sum_spot += t_spot
                sum_n += 1

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

                # 只有"连续 N 帧都锁定"且"误差和上次不突跳"才允许闭环纠偏。
                # 整机日志里的教训：视觉大部分时间是瞎的、偶尔锁到的还是背景块，
                # 误差在 30/161/194 之间乱跳，闭环照着垃圾误差积分，云台就会
                # 一个劲往一个方向转（偏置被推到 12° 再冻住）。加了这道门，
                # 宁可不纠偏，也不跟着误检出方向。
                if (tgt_uv is not None) and (spot is not None) and \
                        (target_det.lost == 0):
                    lock_run += 1
                    if lock_run > 5:
                        lock_run = 5
                else:
                    # ⚠ 2026-10-09：原来这里直接清零，而靶纸会闪断（忽锁忽丢），
                    #   结果"连续 3 帧锁定"永远凑不齐 -> 偏置被冻住，云台停在
                    #   不相干的位置。改成"缓慢衰减"：闪一两帧不影响闭环。
                    if lock_run > 0:
                        lock_run -= 1
                err_u = (tgt_uv[0] - su) if tgt_uv is not None else 0.0
                err_v = (tgt_uv[1] - sv) if tgt_uv is not None else 0.0
                # ⚠ 2026-10-08 删掉原来的"误差突跳>60px 就跳过更新"闸门：
                #   它和 last_err 的刷新时机组合起来会死锁（抖动 -> 一直被挡 ->
                #   last_err 永不刷新 -> 永远被挡），现场表现是"偏置冻在小值、
                #   激光停在靶纸边缘"。现在改成在 AimCtrl 里做 EMA 滤波来抗抖，
                #   不再用"跳过更新"这种会自锁的办法。
                ok_now = (lock_run >= 2) and \
                    (tgt_uv is not None) and (spot is not None)
                if AUTO_GAIN_CAL and (not sign_cal.done()):
                    # 开机标定：刚锁住靶时给 ±4° 已知偏置，量出"偏置->像素"的
                    # 真实增益 K（px/°）。标定期间不做闭环（免得带着试探偏置积分）。
                    cy, cp, cal_on = sign_cal.step(now, err_u, err_v, ok_now)
                    cal_yaw, cal_pit = cy, cp
                    cal_active = cal_on
                    ctrl.valid = False
                    ctrl.locked = False
                    if ok_now:
                        last_err = (err_u, err_v)
                    if sign_cal.done() and (not cal_gain_done):
                        cal_gain_done = True
                        if 6.5 < sign_cal.k_yaw < 10.0:
                            ctrl.k_px = sign_cal.k_yaw
                        if 6.5 < sign_cal.k_pit < 10.0:
                            ctrl.k_px_v = sign_cal.k_pit
                        log("[CAL] 实测换算: K_yaw=%.1f K_pitch=%.1f px/度 "
                            "(几何值 7.8) -> 已用于换算"
                            % (ctrl.k_px, ctrl.k_px_v))
                elif ok_now:
                    cal_active = False
                    ctrl.valid = True
                    # 把目标在画面里的绝对位置喂给控制器：做镜头切平面校正用
                    ctrl.tgt_u, ctrl.tgt_v = tgt_uv
                    ctrl.update(max(dt, 1e-3), err_u, err_v, att_rel,
                                target_det.meas[5] if target_det.meas
                                else 0.0)
                    last_err = (err_u, err_v)
                    # ---- 晃动诊断（2026-10-10 新增）----
                    # 1 秒一行的 [TRACK] 看不出 2~5Hz 的晃，这里每 2 帧直接写一行
                    # 到 SD 日志（约 15 行/s），拿回来就能算出晃的频率和幅度。
                    if WOBBLE_LOG and ((n_frames % 5) == 0) and \
                            (_log_f is not None) and \
                            (0.0 < (target_det.meas[5] if target_det.meas
                                    else 0.0) < 160.0):
                        try:
                            # aty/atp = 云台【实际姿态】（遥测 50Hz 来的）：
                            # 用它区分"轴粘住->突然滑一下"（姿态是台阶）和
                            # "线性环路延迟振荡"（姿态是连续正弦）。
                            _log_f.write(
                                "[W] t=%d eu=%.1f ev=%.1f yaw=%.2f pit=%.2f "
                                "aty=%.2f atp=%.2f g=%.3f/%.3f px=%.0f\n"
                                % (now, ctrl.last_eu, ctrl.last_ev,
                                   ctrl.yaw, ctrl.pitch,
                                   (gz["yaw"] if gz is not None else 0.0),
                                   (gz["pitch"] if gz is not None else 0.0),
                                   ctrl.g[0], ctrl.g[1],
                                   target_det.meas[5]
                                   if target_det.meas else 0.0))
                        except Exception:
                            pass
                else:
                    cal_active = False
                    ctrl.valid = False
                    ctrl.locked = False
                # ---- 丢靶处理：默认**冻住偏置**（见 LOST_RETURN_ENABLE 说明）----
                # 只有在 LOST_RETURN_ENABLE=True 时才做"缓慢回参考位"。
                if LOST_RETURN_ENABLE and (not cal_active) and \
                        (target_det.lost > 0):
                    if (target_det.lost * 40) >= LOST_RETURN_MS:   # ≈40ms/帧
                        d_back = LOST_RETURN_DPS * max(dt, 1e-3)
                        if ctrl.yaw > d_back:
                            ctrl.yaw -= d_back
                        elif ctrl.yaw < -d_back:
                            ctrl.yaw += d_back
                        else:
                            ctrl.yaw = 0.0
                        if ctrl.pitch > d_back:
                            ctrl.pitch -= d_back
                        elif ctrl.pitch < -d_back:
                            ctrl.pitch += d_back
                        else:
                            ctrl.pitch = 0.0
                        ctrl.i_u = 0.0
                        ctrl.i_v = 0.0

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
                    # 隔帧推画面：to_ide 传输比处理慢，追太紧会出现撕裂/花屏
                    if (_display is not None) and ((n_frames % SHOW_EVERY) == 0):
                        _display.show_image(img)

            # ---------- 5. 发 AIM（50Hz，保持链路活着）----------
            if (not LOG_ONLY) and (state in (ST_SET_ZERO, ST_TRACK)) and \
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
                if cal_active:
                    # 方向自检中：直接下发试探偏置（±4°），并让 H723 应用它
                    link.send(build_frame(
                        MSG_AIM, pack_aim(cal_yaw, cal_pit,
                                          FLAG_AIM_VALID | FLAG_LASER_ON,
                                          200)))
                else:
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
                else:
                    # 没锁定时把"为什么没过闸门"一起写进日志（下次复盘要用）
                    tgt_s = "无靶[%s 候选%d %s]" % (
                        target_det.state, target_det.last_n,
                        target_det.last_dbg)
                if gz is not None:
                    log("[%s] %.1ffps gz(state=%d fault=%d flags=0x%02X "
                          "yaw=%.1f pit=%.1f roll=%.1f ymot=%.1f pmot=%.1f "
                          "gz=%.1f/%.1f up=%dms) 靶=%s 命中%d/%d 光斑%d "
                          "err=%.0f(eu=%.0f ev=%.0f) 偏置 yaw=%.1f pit=%.1f "
                          "自适应[g=%.2f/%.2f s=%+.0f/%+.0f] ok=%d crc=%d "
                          "ms 取图%.0f 目标%.0f 光斑%.0f K=%.1f/%.1f "
                          "Ke=%.1f/%.1f 放大=[%.2f/%.2f]"
                          % (ST_NAME[state], fps, gz["state"], gz["fault"],
                             gz["flags"], gz["yaw"], gz["pitch"], gz["roll"],
                             gz["ymotor"], gz["pmotor"], gz["gyro_y"],
                             gz["gyro_z"], gz["up_ms"], tgt_s,
                             n_hit, n_frames, n_spot, ctrl.err_px,
                             ctrl.last_eu, ctrl.last_ev,
                             ctrl.yaw, ctrl.pitch,
                             ctrl.g[0], ctrl.g[1], ctrl.sg[0], ctrl.sg[1],
                             parser.ok, parser.crc_err,
                             sum_grab / float(max(1, sum_n)),
                             sum_tgt / float(max(1, sum_n)),
                             sum_spot / float(max(1, sum_n)),
                             ctrl.k_px, ctrl.k_px_v,
                             ctrl.k_est, ctrl.k_est_v,
                             ctrl.sec2_u, ctrl.sec2_v))
                else:
                    log("[%s] 等 H723 遥测... 靶=%s ok=%d crc=%d"
                        % (ST_NAME[state], tgt_s,
                           parser.ok, parser.crc_err))
                sum_grab = 0
                sum_tgt = 0
                sum_spot = 0
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
