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
# --- 相机 / 处理分辨率（2026-10-11：做成一个开关）---
# 用户要求"试试提高分辨率"。**只改下面 PROC_W/PROC_H 两个数**，别的都不要动：
#   PROC_W 必须是 16 的倍数（ISP 只按 16 对齐，多出来的会被悄悄改掉）。
#     640x480 = 基准（现场实测：目标检测≈16ms/帧、整环≈41fps）
#     800x600 = 细一档（像素×1.56，检测耗时同比涨，预计 ≈30fps）
#     480x360 = 快一档（像素×0.56，预计 ≈60fps，远处细节少一半）
# 所有"像素"单位的常数自动按 S_SCALE 缩放（下面每个带 S_SCALE 的行），
# 所以换分辨率 = 只改这两个数，检测判据的"物理含义"不变。
# ⚠ 两个必须一起记住的：
#   ① 像素/度（K）随分辨率同比变化 —— main.py 会把它随 AIM 帧发给 H723；
#      本文件只是看画面，不涉及 K。
#   ② 分辨率越高，"每像素对应多少角度"越小 → 角精度不变（fx 同步放大），
#      买到的是"远处纸块更大、分割更细、量化噪声更小"。代价就是帧率。
# ⚠ 2026-10-11 用户现场实测后的结论（别被"提高分辨率"想当然）：
#   800x600 那一轮：锁定帧 12~18ms（和 640 档一模一样）、
#   **丢靶后的全图搜索 61~67ms（640 档 ~40ms，慢 1.5 倍）**、识别反而没更准
#   （当时的失败全是"阈值把纸面和背景粘成一块"，跟分辨率无关）。
#   而且这块板子的预览画布固定 640x480（显示只支持这个尺寸）：800 的图要缩
#   一次才看得见，预览反而更糊 —— 这就是"分辨率也不高"的由来。
#   所以默认改回 640x480（预览 1:1 更清楚、丢靶后重新找靶快 1.5 倍）。
#   想再对比 800 档就把这两个数改回 800/600（整机跑时 H723 也要跟着重烧 K）。
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
# --- 曝光（默认关；只有当二值化预览里"纸面和白墙糊成一片白"时才需要开）---
# 现场日志的旁证：画面均亮度 111~124/255、而纸面所在的那块"内亮度"只有
# 110~126（= LAB-L 43~49）—— 整个画面被自动曝光压进了一个很窄的亮度带，
# 纸面和白墙只差 3~5 个 L 单位，**任何全局阈值都切不开**。这时固定曝光是
# 釜底抽薪：曝光降下来，白纸还是白、灰墙会明显变暗，阈值就有余量了。
#   EXPO_FIX = True 打开；EXPO_US 是曝光时间（微秒），先试 8000，
#   画面太暗就往下调 4000 / 2000 / 1000。定下来后 main.py 里填同一个值。
EXPO_FIX = False
EXPO_US = 8000
SHOW_EVERY = 2             # 每几帧往 IDE 推一次画面（识别仍每帧都算）
CAM_VFLIP = True           # 画面上下颠倒 -> True
CAM_HMIRROR = False        # 画面左右镜像 -> True

# --- 靶纸检测：A4 白纸亮块 + “四周更暗”校验 ---
# 实测 L(0~100): 白纸 60~70、木柜 25~35、墙 45、黑胶带 20 —— 固定阈值就能分开；
# 黑胶带框正好提供“亮块四周更暗”的校验（白瓷砖地没有这圈暗框，不会误检）。
# 现场日志：真靶纸 长边184~190px/密度0.83~0.96/对比90+；误检 密度0.51~0.73/对比22~67。
# 所以下面加了长边范围、密度、对比三道静态闸门，外加尺寸/位置/连续确认三道时间闸门。
# 详细说明见 tools/vision_check.py 顶部。
# ⚠ 2026-10-08 改回 vision-v1（"视觉初版识别"，你验证过的那版）的值：
#   之前为追"候选0"把 58 降到 50/42，但那次"候选0"的真因是相机没照到靶纸，
#   不是阈值太高；降阈值反而把墙/柜面（内125~172）也放进来 -> 误检、云台乱跟。
# ⚠ 2026-10-11 深夜（上一版的教训，别改回去）：把阈值从"均值+8"降到"均值+1"
#   之后，现场日志立刻出现 [182906px 框640 内146 ✗框长] —— 阈值降到 46 就掉进
#   "木板/黑胶带"的亮度里，纸面跟背景糊成一整块，反而一张都认不到。
#   截图像素复算：木板/胶带最高 ≈50、纸面 52~60，所以阈值必须落在 50~52 之间，
#   "均值+8"（本地 45.9+8=53）正好；"均值+1"(=46) 是灾难。
#   纸面被切窄不是丢靶的原因（剩一半也够过闸门），当时的真凶是"不够亮"那条，
#   它后来整体删掉了（见下面"不够亮判据删除"的说明）。
PAPER_TH = 50              # 亮度阈值（0~100）的**下限**；实际由 scene_th() 自适应
# ⚠ 2026-10-11 现场（联调日志）：远处/换个背景之后，自动阈值会掉到 39~45，
#   于是"纸面 + 白墙 + 地板"全在阈值之上、被 find_blobs 粘成一个 26 万像素
#   的大块（日志里 [26xxxxpx 框800 内1xx 外1xx ✗框长] 那一串），真靶纸反而
#   永远出不来。根因是"自动阈值在这个场景偏低了"，所以加一个手动旋钮：
#     PAPER_TH_FIXED = 0   -> 自动（Otsu，原来的行为）
#     PAPER_TH_FIXED = 58  -> 强制用 58（0~100 的 LAB-L）
#   调法：开 BIN_PREVIEW，直接看二值化画面，把阈值调到"只有靶纸是白的、
#   别的大面积背景变黑"为止，再把这个数同步到 k230_aim/main.py（要一样）。
PAPER_TH_FIXED = 0
# ⚠ 2026-10-09 用户现场：光线变亮、距离从 0.6m 变到 1.5m（大部分 0.6m 开外）。
#   固定阈值在亮场景下会把更亮的背景也算进来 -> 改成自适应：
#     阈值 = 画面整体亮度(折算 0~100) + 13，再夹在 [55, 82]
#   （依据：log(6) 里 画面均值L≈45、纸 L≈65，45+13=58 正好是分界线 ✓）
#   同时放宽最小面积/最小边长，保证 1.5~2.2m 纸变小了也能认。
PAPER_A_MAX = 32           # |a| 上限（偏色背景会被排除）
PAPER_B_MAX = 32           # |b| 上限
# ⚠ 2026-10-11 深夜：600 -> 350。实测尺寸链：2.7m 时真纸亮块 960px、3.2m 只剩
#   ~600px —— 正好撞在 600 这个门槛上被丢掉（日志里远处只剩"大白物"能过闸门，
#   真纸反而消失），3m 开外"零状态锁上"锁的其实是别的白物。350px 覆盖到 ≈4m。
#   防误检交给 对比>=30 / 暗边>=62% / 比例<=2.0 / 长边>=40 / 密度 这些判据。
PAPER_MIN_AREA = int(350 * S_SCALE * S_SCALE)   # 最小像素面积（面积随 S² 缩放）
# ⚠ 2026-10-11：0.85 -> 0.50（借 QGimbal-Vision 的 max_area_ratio=0.5）：
#   单个亮块超过半屏一定不是靶纸（那是墙/地板糊成的背景块），提前挡掉。
PAPER_MAX_AREA_RATIO = 0.50
# ⚠ 2026-10-11 深夜：45 -> 40 -> 36。现场 2.7m 真纸 bbox 52px、3.2m 只剩 41px、
#   四边形还会再小 10%（边扫描在暗处提前收住，差 1~2px 就永远差一口气）。
#   36px 覆盖到 ≈3.7m；防误检靠 对比≥30/暗边≥62%/密度/比例/面积 这些动态判据。
PAPER_MIN_LONG = int(round(36 * S_SCALE))    # 长边下限 px（长度随 S 缩放）
PAPER_MAX_LONG = int(round(480 * S_SCALE))   # 长边上限 px
PAPER_ASPECT_MIN = 0.70    # A4=1.41；斜视透视下会缩到接近 1
# ⚠ 2026-10-11：3.00 -> 2.00。现场白墙/亮柜沿的假块实测长宽比 2.25~2.98
#   （163x402、128x380），真靶纸 1.10~1.6 —— 2.00 把白墙挡在外面，真纸不受影响。
PAPER_ASPECT_MAX = 2.00
# ⚠ 2026-10-09 现场日志（1.0~1.5m 抓不到靶）：
#   日志里真靶纸的候选长这样 —— [14490px 框211 内180 外130 暗边83% ✗密度]、
#   [6657px 框121 内141 外49 暗边100% ✗密度]：内亮、四周是黑胶带（对比/暗边都过），
#   唯一卡住的是"密度"。原因：这些帧四边形拟合没成功，退回"外接框"算密度，
#   而**旋转过的外接框天生比纸大**（斜 45° 时只有约 0.5），0.70 这一刀把真纸全砍了。
#   → 外接框这一档放宽到 0.45（防误检还有对比>=30、暗边>=62%、连续3帧确认三道关）；
#     四边形那一档保持 0.72 不变（四边形面积是旋转不变的，本来就准）。
# ⚠ 2026-10-11 深夜：0.40 -> 0.46。现场"右边桌上的白纸/白物"实测 密0.41~0.42、
#   对比98~119、比1.42 —— 和真纸的 比1.42/对比115 几乎一样，唯一能区分的是
#   实心度：真纸（含斜视）密0.94~1.04；几何上"实心矩形转 45°"的外接框填充率
#   也有 0.485，低于它 = 带洞/发虚的形状（那张白物 0.41 就是）。0.46 卡中间。
# ⚠ 2026-10-11 整机日志(25) 再收一刀：0.46 -> 0.56。
#   实测误锁实例：被扰动后锁上 "0.33m 长368px 比1.22 密0.53 对比65"、
#   "0.39m 长315px 比1.01 密0.50 对比53" —— 都是"木柜/背景大块把靶纸当洞"
#   的带洞形状（密 0.50~0.54），而同一份日志里真靶纸一直是 密0.98~1.00、
#   对比 109~141。0.56 正好卡在两者之间。
#   注：0.485 那个"实心矩形转 45°"的极端情况要靠**云台滚转≈0**（相机固定
#   在云台上）来排除；真靶纸实测（含斜视）密 0.92~1.04，离 0.56 很远。
PAPER_DENSITY_MIN = 0.56   # 兜底（没拟合出四边形时）下限
# ⚠ 2026-10-11 现场(30号日志)：上面 0.56 这一刀**误伤了掠射**——近处直角转弯、
#   激光路径与靶面夹角很小时，靶纸被压扁，外接框填充率只有 ~0.54（真纸！
#   那一帧 内183 外110 暗边83% 对比73，其余判据全过），结果被"密度"挡死 -> 脱靶。
#   而"木柜把纸当洞"的假块 对比 只有 53~65。所以按对比分档：
#     对比 >= 70（内亮外黑极强，靶纸/胶带的特征）-> 密度下限放宽到 0.45
#     否则                                     -> 仍用 0.56
PAPER_DENSITY_MIN_STRONG = 0.45
PAPER_CONTRAST_STRONG = 70
# ⚠ 2026-10-11 现场(30号日志)：0.72 -> 0.65。掠射时像素/四边形面积天生偏低
#   （亮块被阴影切窄、四边形摸到真实外沿，实测 px/qa ≈ 1/1.39 = 0.72，正好
#   卡在门槛上）。放宽到 0.65（允许四边形比亮块大 1.54 倍）；
#   反方向的"木柜把纸当洞"由下面的密度**上限 1.08** 拦（px/qa≈1.9 必拒）。
PAPER_DENSITY_QUAD_MIN = 0.65
# ⚠ 2026-10-09 现场实测（光线变亮后）：
#   真靶纸 内−外 只有 32~43（亮光下"四周"也变亮），而背景块只有 21。
#   之前设 45 会把真靶纸挡掉 -> "容易丢靶、开机难找靶"。
#   现在 30：真纸 32~43 过 ✓，背景 21 拒 ✗，防误检再靠暗边比例(0.62)+面积+确认。
# ⚠ 2026-10-11 深夜：25 -> 30。现场右下角"白柜沿"假块实测 内142 外115（对比只有
#   26~27），但它面积更大、评分反超真靶纸 -> 远处误锁（截图复算复现）。真靶纸在
#   本地实测 对比 41~112，30 正好卡在假块(≤27)与真纸(≥41)之间。
PAPER_CONTRAST_MIN = 30    # 内亮度 - 外亮度（0~255 量程）
PAPER_DARK_MARGIN = 25     # 单个外侧采样点算“暗”的门槛
# ⚠ 2026-10-11 深夜：0.62 -> 0.55（用户意见："62% 太严格，要基于实际性能"）。
#   真纸实测 92~100%（本场地）/67~83%（老场地斜视），背景假块 58%/50%/42%/33%。
#   0.55 给"斜视/半张被挡"留出余地；防误检改由 对比>=30 / 实心度>=0.46 /
#   激光点优先 三道一起兜（比只看"暗边比例"更稳）。
PAPER_DARK_FRAC_MIN = 0.55  # 外侧 12 点里至少这么多比例更暗
# --- 四边形拟合（斜视时画出来是梯形，靶心=对角线交点=透视中心）---
# ⚠ 2026-10-11 深夜提帧率：扫 7 行×垂直±2px 太贵（边扫描是锁定帧的大头）。
#   7->5、2->1 后边扫描开销约减半，5 个采样点做鲁棒拟合仍然足够。
QUAD_SCAN_N = 3            # 每边取几行/几列做扫描（2026-10-11: 5->3 再省一截）
QUAD_PERP_PX = 1           # 扫描时垂直方向各看几像素
QUAD_AREA_LO = 0.50        # 四边形面积 / 亮块像素 的合理范围
# ⚠ 2026-10-11 现场(30号日志)：1.35 -> 1.60。掠射（激光路径与靶面夹角很小）时
#   亮块会被"阴影/边缘软化"切窄，而边扫描会摸到靶纸真实外沿 —— 四边形比亮块大
#   四成是**正常**的（实测 qa/px≈1.39），1.35 当场把四边形弃掉、退回外接框，
#   再被密度门槛卡死 -> 脱靶。"木柜把纸当洞"那种假块会不会因此漏进来？
#   不会：它恰好相反（四边形≈纸 而亮块=木柜，px/qa≈1.9），下面的密度上限
#   1.08 那一闸专门拦它。
QUAD_AREA_HI = 1.60
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
# ⚠ 2026-10-11 现场日志定案：**find_blobs 的 merge=True 不能用**。
#   它是按"外接框"合并的（两块的外接框一靠近就并成一块），所以只要背景里
#   有个大亮块、它的外接框罩住/贴着靶纸，靶纸就会被吞进去 —— 日志里那个
#   "26 万像素 框800 / 18 万像素 框640 ✗框长"的大块就是这么来的：**真靶纸
#   永远不作为候选出现**（二值化图里靶纸明明是一块独立的白，日志里却看不到它）。
#   现在改成 merge=False + 自己写的小合并（merge_fragments）：只拼"像靶纸"的
#   碎片，绝不吸收背景大块。这个余量 = 允许拼起来的两块之间最大间隙（px）。
MERGE_GAP_PX = int(round(4 * S_SCALE))
# （ACQ_ROI_K / ACQ_FULL_EVERY / FULL_EVERY 已删除：找靶改成"每帧全图搜"，
#   这几个"中间区域/隔帧全图"的参数再也用不到，按用户意见一并清掉。）
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
# ⚠ 2026-10-10 延迟专项：这两个"时间平滑"是视觉链路里**最大的纯延迟来源**：
#   一阶 EMA 的群延迟 ≈ (1-α)/α ÷ 帧率。α=0.55@45fps → 18ms；α=0.30 → 51ms！
#   也就是说远处那 100ms 的"看不见的延迟"里，一大半是它自己加的。
#   比赛时小车是连续运动的，目标本身很平滑，不需要这么重的滤波 —— 提高 α。
# ⚠ 2026-10-11 深夜(37)：0.80 -> 0.90。群延迟 ≈ (1-α)/α × 帧周期(30ms)：
#   0.80→7.5ms、0.90→3.3ms。这条延迟**直接决定云台能用的环路增益上限**
#   （相位余量），所以在打不动执行机构迟滞的前提下，每削几毫秒都是白赚。
#   代价：中心噪声略大 —— 有 1.5px 死区 + 跟踪期防跳闸门兜着。
SMOOTH = 0.90              # 平滑系数（越大越跟手、越小越稳）
DEADBAND_PX = 1.5 * S_SCALE   # 平滑死区（小于它不动，画面不抖）
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
# ⚠ 2026-10-11：0.65 -> 0.50。直角弯时目标快速横移+运动模糊，纸面亮块会在
#   一两帧里缩到 0.57 倍（21 号日志实测 254px->144px），0.65 当场判"换目标"
#   直接丢靶；0.50 放行这种模糊收缩（防碎片误锁仍由 实心度/对比/暗边 把关）。
SIZE_GATE_LO = 0.50        # 跟踪时允许的长边变化范围（不许换目标）
# ⚠ 2026-10-11：1.60 -> 1.35。22 号日志：平台被拖到斜视后，纸块和背景糊成
#   长404px（1.58 倍）的"糊块"，1.58<1.60 正好放进来，锁到一帧假中心(68,61)后
#   直接丢靶。一帧内长边涨 35% 以上不可能是真目标（那得瞬移换距离），判糊块拒绝。
SIZE_GATE_HI = 1.35
# ⚠ 2026-10-11 用户意见：连续 2 帧一致才锁定太苛刻 —— 车在快速移动/画面抖时
#   根本凑不齐两帧一致，结果"只有停下来才勉强能锁上，那时靶早跑出画面了"。
#   改成 **1 帧就锁**；防误检不靠这个，靠三重动态判据(够亮/对比/暗边)+四边形拟合
#   +锁定后的防跳闸门(中心跳>100px、尺寸突变>45% 直接丢)。
CONFIRM_N = 1              # 连续确认几次才算锁定（2->1：一帧即锁）
# ⚠ 2026-10-10 现场日志（远距离"必定丢靶"的真因）：
#   丢靶那一帧的候选里明明有通过的靶纸 [24711px 框219 内170 外91 暗边100% ✓]，
#   但状态卡在"确认1/3"——因为远距离时纸的白块会在"整块"和"碎片"(框67/框73)
#   之间来回跳，±35px/±60% 的一致性判据被打破，确认计数一直归零。
#   -> 位置容差放到 60px、尺寸容差放到 ±120%（碎片/整块都算同一张纸），
#      漏检容忍 3->5。防误锁仍靠 对比>=30 + 暗边>=62% + 连续3次 三道关。
CONFIRM_DXY = int(round(60 * S_SCALE))   # 确认时的位置一致范围 px（35 -> 60）
CONFIRM_DSIZE = 1.00       # 确认时的尺寸一致范围（±100%；2.0 会连"碎片"一起认进来）
PENDING_MISS = 8           # 确认过程中允许漏几次（5 -> 8）
# ⚠ 2026-10-11 深夜："不够亮"判据整体删除 —— 它是按旧场地"真纸 170~245"定的；
#   现在 2.5m 外的真纸内亮度只有 113~119，画面均值 119~122（被白桌面抬高），
#   真纸反而被判"不够亮"（日志 [980px 框53 内117 外60 暗边100% ✗不够亮]，
#   其余判据全过 —— 这就是"放到远处再进画面锁不上"的最后一关）。
#   防误检交给 对比>=30 / 暗边>=62% / 比例 / 密度 这四道。
# 跟踪期"测量防跳"（2026-10-10 新增，治"远处测到的目标乱跳 -> 偏置被推飞"）：
#   日志实测：远距离时同一段里的"距离"能在 0.72→0.75→0.86→1.01→0.55→2.17m 之间跳，
#   说明测到的是碎片/别的东西，中心一偏几十像素，闭环就照着错误差把云台推出去。
#   ① 已经在锁定时，新测量如果相对平滑中心跳 >60px 且尺寸也大变 -> 判为误检，本次不采纳；
#   ② 靶纸小（远）时把中心/角点的平滑做强一点（0.55 -> 0.30），把抖动滤掉。
# 60 -> 100：转弯时云台滞后，靶纸在画面里可以合法地跳得很快（实测一次 167px
# 是"底盘转+云台追"的正常现象，不是误检）。100px/帧 @40fps = 4000px/s，
# 真目标不可能这么快，仍然能挡住误检。
JUMP_REJECT_PX = int(round(100 * S_SCALE))  # 跟踪期单帧中心跳变上限（超了当误检丢）
SMOOTH_FAR = 0.60          # 远距离平滑（0.30 -> 0.60：51ms -> 15ms）
# ⚠ 2026-10-11：FAR_LONG_PX 原来放在"瞄准参数区"，但视觉代码(_accept)里要用它 ——
#   同步到 vision_check.py 时只有"用"没有"定义"，直接 NameError。
#   凡是**视觉代码引用**的常量都必须放在这个视觉配置区里，同步才会带上。
FAR_LONG_PX = 120.0 * S_SCALE  # 长边小于它 = "远"（平滑/滤波/增益按它分档）
# ⚠ 2026-10-11：0.297 -> 0.275（用户实测：靶纸中间露出的白色区域 ≈19x27.5cm ——
#   黑胶带压住外沿，视觉量到的白块长边就是 0.275m，距离估算按它算才准）。
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
# 目标优先权（2026-10-11 深夜）：候选中心离"激光点"多近算"激光就打在它身上"。
#   现场：2.5m 真纸 980px 评分1186，旁边白布 2354px 评分3084 —— 按评分永远抢
#   不过近处大白物；但云台瞄准时激光必然打在靶纸上，所以激光点附近的候选优先。
SPOT_PREF_PX = int(round(70 * S_SCALE))
SPOT_UV = [IMG_W / 2.0, IMG_H / 2.0]   # 最近一次找到的激光点，主循环每帧更新
# ⚠ 2026-10-11 深夜（用户要求"少一层不确定"）：激光与相机刚性固联，光斑在画面
#   里的位置是固定的 —— 改成标定常数，不再每帧检测。好处：
#     ① 不会再出现"目标好好的、但光斑当帧没检到 -> ok=0 -> H723 冻结"的闪烁
#        （22/23 号日志里这种帧比比皆是）；
#     ② 省掉每 4 帧一次的光斑搜索（~3ms）和它的自适应漂移这一整层不确定。
#   取值依据：7 张现场图里黄圈（光斑在画面中的位置）中位 ≈(312,252)。
#   联调时若发现打点有固定偏心（总偏右/偏上），直接把 x/y 改同样像素数即可
#   （K=7.8px/°，10px ≈ 1.3°）。恢复检测把 USE_FIXED_LASER 改 False。
USE_FIXED_LASER = True
# ⚠ 2026-10-14：与 main.py 逐字同步（sync_check 会比对常量值）。
#   取值 = 71 号日志自标定实测 (315,246)；真正用的激光坐标 = LASER_UV + LASER_TRIM。
LASER_UV = (int(round(315 * S_SCALE)), int(round(246 * S_SCALE)))
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


_PIXBUF = [0, 0, 0]
_PIX_FAST = False      # get_pixel(x,y,buf) 这种"就地填"写法能不能用（见下）


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
        if _PIX_FAST:
            p = img.get_pixel(int(x), int(y), _PIXBUF)
        else:
            p = img.get_pixel(int(x), int(y))
    except Exception:
        return -1
    if p is None:
        return -1
    return luma(p[0], p[1], p[2])


_scene_cache = [0, 0.0, 0.0]   # [帧计数, 上一次算出的阈值, 预留]
# 分段耗时探针（2026-10-13 加，用来回答"目标 11ms 到底花在哪"）：
#   [0] 预留  [1] find_blobs（C 搜索）累计 ms  [2] 预留  [3] 调用次数
#   日志里的 `块x.x` 就是 [1]/[3] —— C 搜索快不快，一眼能看出来；
#   若它只占 1~2ms，那"目标"时间的大头就是 Python 侧的四边形复测。
_PROF = [0.0, 0.0, 0.0, 0]


def scene_th(img, n_frame=0):
    """按当前帧直方图用**大津法(Otsu)**自动决定"白纸"的 LAB-L 阈值（0~100）。

    2026-10-11 大改（参照 QGimbal-Vision 的"确定性二值化"，用户要求）：
      原来"画面均值+8"是启发式 —— 纸面/木板都挤在阈值附近时会忽切多忽切少，
      检测跟着场景亮度漂。Otsu 拿整帧直方图自动找"亮类 vs 暗类"的分界，
      **每帧确定**，不随均值漂。
      现场 7 张截图实测：Otsu 落在 L≈40~50，木板/胶带 30~43、纸面 52~68，
      正压在两者中间（Otsu 合法区间 [38,62] 之外才走兜底）。
    兜底：直方图 API 不可用或结果离谱（<38 或 >62）时，回退"30 点均值+8、
      夹 [PAPER_TH, 72]"（PC 仿真里 fake 没实现直方图时就走这条）。
    每 10 帧才重算一次（省时间也避免逐帧抖动）。
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
    隔约 16 像素采样 40x30 个点 -> 32 桶直方图 -> 找双峰分界 -> 折回 0~100。
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
        # ⚠ 双峰之间若有很宽的空隙，类间方差在空隙里是"平的"——直接取第一个
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


def paper_contrast(img, x, y, w, h):
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


def merge_fragments(blobs, roi):
    """把"被印刷圈/阴影切碎的靶纸"重新拼成一块，**但绝不吸收背景大块**。

    为什么不用 find_blobs(merge=True)：那个 merge 是按**外接框**做的 ——
    两块的外接框一靠近（margin 之内）就并成一块。于是只要背景里有个大亮块
    （白墙/亮桌面），它的外接框罩住靶纸，靶纸就被吞进去，最后只剩一个
    "26 万像素 框800 ✗框长"的大块：**真靶纸永远不作为候选出现**。
    （2026-10-11 现场：二值化图里靶纸明明是一块独立的白，日志里却只有大块。）

    这里的规则（三条都必须过，所以"像靶纸"才会被拼）：
      · 两块的间隙 <= MERGE_GAP_PX（印刷圈切割出来的碎片之间只有 1~3px），
        **或者一块的外接框整个嵌在另一块里面** —— 印刷圈把纸切成同心环时，
        内圈的外接框就在外圈里面，这种情况必须拼（不然最外圈那条"环带"的
        实心度只有 0.3~0.4，会被"密度"闸门判成假块，近处反而认不到）；
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
                gx = max(0, max(a[0], c[0]) - min(a[0] + a[2], c[0] + c[2]))
                gy = max(0, max(a[1], c[1]) - min(a[1] + a[3], c[1] + c[3]))
                nested = ((a[0] >= c[0]) and (a[1] >= c[1]) and
                          (a[0] + a[2] <= c[0] + c[2]) and
                          (a[1] + a[3] <= c[1] + c[3])) or \
                         ((c[0] >= a[0]) and (c[1] >= a[1]) and
                          (c[0] + c[2] <= a[0] + a[2]) and
                          (c[1] + c[3] <= a[1] + a[3]))
                if nested and ((la < lc * 0.75) or (lc < la * 0.75)):
                    # ⚠ 2026-10-11 现场教训：嵌套还必须"大小差不多"才算同心的
                    # 碎片（印刷圈切出来的内外圈几乎一样大，比值 >=0.9）。而
                    # "靶纸被一大块亮背景（木柜/白墙）罩住"也是嵌套，但尺寸差
                    # 一大截 —— 那种**绝不能拼**：拼了靶纸就没了，日志里表现为
                    # "阈值低的那几帧永远只有 11 万像素的大块、没靶纸"。
                    continue
                if (not nested) and \
                        ((gx > MERGE_GAP_PX) or (gy > MERGE_GAP_PX)):
                    continue                    # 离得太远 -> 不拼
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
    ⚠ 2026-10-11 深夜提帧率：5 个采样 -> 3 个。复测里每帧要上千次 _probe，
    这一刀直接把四边形复测的开销砍掉四成（锁定帧 17ms 的大头就是它）。
    ⚠ 2026-10-13：**再把热路径的内存分配砍掉**。原来是"先建 1 个外层元组 +
    3 个点元组、再 for 循环"，每帧 ~100 次调用 = 400 次分配（MicroPython 里
    一次分配要几微秒，攒起来 1~2ms/帧）。现在三个采样点手工展开，
    语义一字不差：还是 (x-1,y)(x,y)(x+1,y)（竖着时是那三点）里 >=2 点亮。
    """
    if dx:
        a0 = x - 1; b0 = y
        a1 = x;     b1 = y
        a2 = x + 1; b2 = y
    else:
        a0 = x; b0 = y - 1
        a1 = x; b1 = y
        a2 = x; b2 = y + 1
    k = 0
    if px_luma(img, a0, b0) >= th:
        k += 1
    if px_luma(img, a1, b1) >= th:
        k += 1
    if px_luma(img, a2, b2) >= th:
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
        # 上一帧所有候选的外接框（给二值化预览画框用）：
        # (x, y, w, h, 1=过闸门 / 0=被拒)。最多记 8 个，别把内存撑起来。
        self.cands = []
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
        # （现场日志里锁到背景 71px 亮块就是这个口子漏的）。
        # ⚠ 2026-10-11 深夜：窗口 100 帧(4~5s) 太长、0.40 倍太紧 —— 现场
        #   "先锁近处大靶再退远"时远处纸只有 ~100px，0.40*322=129 > 100 被卡，
        #   要等满 5 秒才放开，表现就是"远处怎么都对不上"。改成丢靶 30 帧
        #   (≈1s) / 0.30~3.0 倍；防误锁还有 对比/暗边/密度/激光点优先 四道关。
        if self.have and (self.lost < 30):
            old = max(self.w, self.h)
            if (lo < 0.30 * old) or (lo > 3.0 * old):
                self.pend = None
                return False
        if self.pend is None:
            self.pend = [cu, cv, lo, 1, 0]
            if CONFIRM_N <= 1:
                # "一帧即锁"（用户 2026-10-11）：AGC 把纸面压暗时，纸面只在个别
                # 帧能单独成块（日志 [7929px 内126 外47 ✓] 那种闪光帧），要等
                # 第二帧一致永远等不到 -> 直接确认。
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
        if (not self.have) or (self.lost >= LOST_FULL):
            # 首次锁定 / 丢靶很久后的重新锁定：旧的平滑中心早没意义了，直接吸附
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
        # 兜底档：每 3 帧补一次全图，单双拍方向不同 ——
        #   偶数拍 +8（更严）：背景把纸面糊住时抓"更亮的芯"；
        #   奇数拍 -6（更松）：AGC 把纸面压暗、主阈值把纸切没时抓回来。
        # （2026-10-11 现场：远处丢靶后再转回画面，纸面只有个别帧能过闸门，
        #   两个方向各补一半机会。两个方向都有过实测依据。）
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
                # ---- 全图复查（2026-10-11 深夜新增）----
                # 现场问题：锁到旁边的白色物体后，窗口搜索只在那附近打转，靶纸
                # 就在画面中央也不再看一眼。每 RE_CHECK_EVERY 帧做一次全图复查：
                # 全图里若有"评分 ≥1.6 倍 + 尺寸合理 + 位置差>40px"的合法候选，
                # 直接切过去（截图4 的误锁：真纸 38310 分 vs 白物 2228 分）。
                if (self.frame % RE_CHECK_EVERY) == 0:
                    cb, n2, d2 = self._find(img, (0, 0, IMG_W, IMG_H), th_ad)
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
                        # 换目标判据（任一成立）：
                        #   ① 激光点在它身上、不在当前目标身上；
                        #   ② 评分高 1.6 倍；
                        #   ③ 对比度高一大截（真纸"白纸+黑边"对比 90~110，误锁的
                        #      大暗块只有 20~50；这条专治"锁到大块后评分换不动"）。
                        if ((cbn and not cun) or (new_s > 1.6 * old_s) or
                                (new_con >= 60 and new_con >= 1.5 * old_con)) \
                                and \
                                (abs(cb[11] - self.u) +
                                 abs(cb[12] - self.v) > 40) and \
                                (0.35 * cur_l <= cb[5] <= 3.0 * cur_l):
                            # 直接吸附换目标（不走 _accept 的防跳闸门——
                            # 这是"纠正锁错对象"，不是"目标乱跳"）
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
                # ⚠ 2026-10-10：这里原来还有一条"窗口失败 -> 中间区域/全图再搜一次"。
                #   用户明确要求：比赛里丢靶就算失败，main.py 不做"找靶"。
                #   把它删掉有两个好处：
                #   ① 省掉每帧最多两次全图 find_blobs（远处"目标耗时 100ms"就是它）；
                #   ② 不会再出现"拿着错误差满图乱找 -> 偏置被推飞"的剧烈晃动。
                #   短暂漏检（<LOST_FULL 帧）仍由上面的窗口搜索兜住。
            else:
                # 开机第一次找靶 / 丢靶超过 LOST_FULL 帧后的重新找靶：
                # **每帧全图搜**（2026-10-11 用户意见）。
                # ⚠ 2026-10-11 修 bug：原来这里写的是 elif not self.have ——
                #   锁定过又丢靶超过 25 帧之后，两个分支都不进，检测器就永远
                #   "候选0"什么都不看了（现场日志：靶=搜索(跟踪窗) 候选0 几百帧，
                #   云台对着靶纸也不动）。用户说的"不做找靶"指的是别乱扫，
                #   靶回到画面里时必须能重新抓住。
                self.did_full = True
                c2, n2, d2 = self._find(img, (0, 0, IMG_W, IMG_H), th_ad)
                if c2 is None and ((self.frame % 4) == 0):
                    # 兜底阈值档每 4 帧补一次（全图一遍 ~35ms，1/4 频率省时间；
                    # 单双拍方向由 frame%8 决定，见上面 th_alt 的注释）
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
        self.cands = []            # 本帧所有候选（预览画框用，见 __init__）
        _t0 = time.ticks_ms()
        blobs = img.find_blobs(
            [(th, 100, -PAPER_A_MAX, PAPER_A_MAX,
              -PAPER_B_MAX, PAPER_B_MAX)],
            # margin：2026-10-10 2 -> 4。远距离时纸面被印刷圈/阴影切成碎片，
            # 间隙只有 1~3px，margin=2 合不起来（日志里全是"框73/81px 的碎片"）；
            # 而黑胶带在 1.3m 还有 ~7px 宽、2m 也有 ~4px，margin=4 不会把纸
            # 和背景粘在一起（当初 6 才会）。这是"整块/碎片来回跳"的正解。
            # ⚠ 2026-10-11 深夜定案：上面这套"margin 合并"整个作废 ——
            #   find_blobs 的 merge 是按**外接框**合并的，背景大块的外接框罩住
            #   靶纸就会把靶纸吞掉（现场日志：只剩 "18 万像素 框640 ✗框长"，
            #   真靶纸永远不作为候选出现）。改成 merge=False + merge_fragments()
            #   自己按"像不像靶纸"拼碎片，规则写在 merge_fragments 里。
            roi=roi, merge=False,
            area_threshold=PAPER_MIN_AREA,
            pixels_threshold=PAPER_MIN_AREA)
        # 分段探针：C 搜索到底占多少（见 _PROF 说明）
        _PROF[1] += time.ticks_diff(time.ticks_ms(), _t0)
        _PROF[3] += 1
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
                    elif long_side < 0.70 * box_long:
                        # 2026-10-11：四边形明显小于外接框 = 边缘扫描在阴影/印刷圈
                        # 处提前收住（2.7m 实测 框52 而四边形 44），退回外接框。
                        # ⚠ 0.90 -> 0.70（30 号日志）：**靶纸在画面里被转开/掠射时，
                        #   外接框天生比纸大**（斜 30° 时 框297 而四边形长边 245，
                        #   比值 0.82）—— 0.90 会把这种"好四边形"也弃掉，退回外接框
                        #   再被密度门槛卡死。0.70 只拦真正塌缩的拟合。
                        corners = None
                    elif _quad_angle_bad(corners):
                        # QGimbal-Vision 同款"四角接近 90°"判据：斜得离谱/乱接边
                        # 的四边形直接弃用（退回外接框，仍受其它闸门约束）。
                        corners = None
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
                    # 掠射/斜视时靶纸被压扁、外接框填充率会掉到 ~0.5，但它
                    # "内亮外黑"极强（30 号日志：对比 73~92、暗边 83~100%）；
                    # "木柜把纸当洞"的假块对比只有 53~65 —— 所以按对比分档。
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
                    # 上限 1.08：四边形面积比亮块还小 8% 以上 = 拟合塌缩在亮块
                    # 内部（现场日志 [长边67px 密1.54] 那种远处误锁就是这么来的；
                    # 真靶纸实测 密 0.78~1.02，四边形只会比亮块大、不会小）。
                    why = "密度"
                elif ((x <= 1) or (y <= 1) or
                      ((x + w) >= IMG_W - 1) or ((y + h) >= IMG_H - 1)) and \
                        (corners is None) and (box_long < 90):
                    # ⚠ 2026-10-11 修正：上一版写"贴边 + 四边形拟合成功才采纳"，
                    #   但这是**死循环** —— 靶纸被转到画面边时角点必然跑到画面外，
                    #   四边形一定拟合失败 -> 还是被贴边丢掉。现场日志实证：
                    #   [39775px 框258 内202 外127 暗边100% ✗贴边] 就是这么丢的。
                    #   现在：贴边 + 拟合失败 + **块够大(>=90px)** -> 判为
                    #   "被画面裁掉的靶纸"，放行（用外接框中心，略偏但比丢靶好）；
                    #   只有"贴边 + 拟合失败 + 块也小"才是噪声/背景边缘，才弃。
                    #   120 -> 90（2026-10-11 深夜）：远处纸从画面边缘进入时只有
                    #   ~100px（日志 [5419px 框102 内156 外108 暗边92% ✗贴边]
                    #   每一帧都在），120 的刀把它一直挡在外面。
                    why = "贴边"
                elif ins < 0 or (ins - outs) < PAPER_CONTRAST_MIN:
                    why = "对比"
                elif frac < PAPER_DARK_FRAC_MIN:
                    why = "暗边"
            if n <= 6:
                # 带上密度 + 是"四"(四边形)还是"框"(外接框)——现场复盘时一眼看出
                # 是哪条路被哪道闸门挡住（30 号日志那次就是因为没这个数，查了一轮）。
                dbg += "[%dpx 框%.0f 内%d 外%d 暗边%.0f%% 密%.2f%s%s] " % (
                    px, box_long, ins, outs, frac * 100.0, density,
                    ("四" if corners is not None else "框"),
                    (" ✗" + why) if why else " ✓")
            if len(self.cands) < 8:
                self.cands.append((x, y, w, h, 0 if why else 1))
            if why:
                continue
            score = px * (0.5 + min(ins - outs, 80) / 80.0)
            # 选优：激光点附近的候选优先（激光打在谁身上谁才是靶纸），
            # 同一档里再比评分（面积×对比）。
            dc = abs(ctr[0] - SPOT_UV[0]) + abs(ctr[1] - SPOT_UV[1])
            key = (1 if dc <= SPOT_PREF_PX else 0, score)
            if best is None or key > best[0]:
                best = (key, (px, x, y, w, h, long_side, aspect, density,
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
        # ⚠ 2026-10-13（与 main.py 同步）：拆成两次调用、**优先选"过曝白芯"**。
        #   以前两个阈值混在一起按面积挑最大块 —— 远距离时红色光晕面积大，常常盖过
        #   白芯被选中；光晕偏离激光真实中心（纸面散射不对称），于是"黄圈/假定光斑"
        #   与实际光斑有固定偏差，激光整体偏一侧（现场：总往右下偏）。
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
                bx, by = norm_bbox(b[0], b[1], bw, bh, (x0, y0, w, h))
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
def main():
    from media.sensor import Sensor, CAM_CHN_ID_0
    from media.display import Display
    from media.media import MediaManager
    import gc

    sensor = Sensor(id=2, width=1280, height=960, fps=90)
    sensor.reset()
    time.sleep_ms(100)
    apply_flip(sensor)
    # ⚠ 800x600 这类非标准尺寸也能设（ISP 会缩放，宽度自动 16 对齐）；
    #   万一这版固件不认，会在这里抛异常 —— 那就把 PROC_W/PROC_H 改回 640/480。
    sensor.set_framesize(width=IMG_W, height=IMG_H, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
    apply_flip(sensor)
    if EXPO_FIX:
        # 固定曝光（关自动）：画面偏亮、纸面/白墙被压进同一个亮度带时用它。
        # 这个 API 在这版固件上不一定有 —— 所以套 try，失败就继续自动曝光。
        try:
            sensor.set_auto_exposure(False, exposure_us=EXPO_US)
            print("曝光: 固定 exposure_us=%d（自动曝光已关）" % EXPO_US)
        except Exception as e:
            print("固定曝光不支持(%s)，继续自动曝光" % einfo(e))
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
    # 探一下"省分配的取像素"写法能不能用（px_luma 是复测热路径）
    try:
        px_probe_fast(sensor.snapshot(chn=CAM_CHN_ID_0))
    except Exception:
        pass

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

    # ---------- 二值化预览：先探测这版固件支持哪种画法 ----------
    # 2026-10-11 现场日志："二值预览画不出来(0) -> 回到彩色预览" ——
    # `img.binary()` 是能跑的，卡在"BINARY 图直接 draw_image 到 RGB565 画布"。
    # 所以这里拿一帧真图把两种做法挨个试一遍，把能用那套记下来 + 打日志，
    # 以后换固件也能一眼看出是哪条路在走。
    if BIN_PREVIEW:
        try:
            _pi = sensor.snapshot(chn=CAM_CHN_ID_0)
            _pth = scene_th(_pi, 0)
            _pi.binary([(_pth, 100, -PAPER_A_MAX, PAPER_A_MAX,
                         -PAPER_B_MAX, PAPER_B_MAX)])
            try:
                # 做法 B：二值图直接缩放（**零分配**，最不容易在跑起来之后翻车）
                _canvas.draw_image(_pi, _ox, _oy,
                                   x_scale=_sx, y_scale=_sy)
                _bin_mode = "bin"
            except Exception as e:
                print("二值预览B(二值直画)不行: %s" % e)
            if not _bin_mode:
                try:
                    # 做法 C：转进一块**预分配**的 RGB565（只分配这一次，之后复用）
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
                    # 做法 A：每帧 to_rgb565（每帧要新建 614KB，最容易失败 -> 放最后）
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

    target_det = PaperDetector()
    spot_det = SpotDetector()

    last_spot = None
    cal_u = []            # 标定用：光斑 u 采样（中位数抗误检）
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
    print("vision_check: 纯视觉（不发串口）; 视觉处理 = k230_aim/main.py 原文")
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
    print("=" * 68)

    try:
        while True:
            os.exitpoint()
            now = time.ticks_ms()

            # ---------- 取图 ----------
            t0 = time.ticks_ms()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            t_grab = time.ticks_diff(time.ticks_ms(), t0)
            n_frame += 1

            # ---------- 靶纸（与 main.py 同一套 PaperDetector）----------
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

            # 隔帧推画面：to_ide 传输比处理慢，追太紧会撕裂/花屏。
            # 画布 640x480 是死的，处理图像按 _sx/_sy 缩着画上去（见 main 开头）。
            t0 = time.ticks_ms()
            if (n_frame % SHOW_EVERY) == 0:
                _ok = False
                if _bin_mode:
                    # 二值化预览：用**和检测同一套阈值**把画面切成黑白，
                    # 这样"阈值到底切出了什么"是看得见的（白=过阈值）。
                    # 每一步单独套 try：哪一步在这版固件上翻车，日志里直接点名
                    # （现场教训：一整块套一个 try，只报一句 "(0)"，白跑一轮）。
                    _th_bin = int(_scene_cache[1])
                    if _th_bin <= 0:
                        _th_bin = PAPER_TH         # 还没算出阈值时的兜底
                    try:
                        img.binary([(_th_bin, 100,
                                     -PAPER_A_MAX, PAPER_A_MAX,
                                     -PAPER_B_MAX, PAPER_B_MAX)])
                    except Exception as e:
                        _bin_mode = ""
                        print("二值化 binary() 失败(%s) -> 改彩色预览" % einfo(e))
                if _bin_mode:
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
                            # 画布上画不了 -> 叠加直接画在二值图上（黑色最显眼）
                            _overlay(img, target_det, _sp,
                                     1.0, 0, 0, True, None)
                        if _cv_shape:
                            _overlay(_canvas, target_det, _sp,
                                     _sx, _ox, _oy, False,
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
                    line += "靶=%s%s 候选%d th=%d %s" % (
                        target_det.state,
                        ("(全图)" if target_det.did_full else "(跟踪窗)"),
                        target_det.last_n, int(_scene_cache[1]),
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
