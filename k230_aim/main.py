"""用于 2025 年电赛 E 题的 K230 靶面矩形检测程序。

本文件保持自包含，可直接保存为 CanMV IDE 中的 /sdcard/main.py，
并在上电后自动运行。

K230 检测靶面并计算标定激光射线到靶面目标点的三维角度误差；
云台闭环和电机控制仍在 H723 上完成。
"""

import gc
import math
import os
import time

try:
    import json
except ImportError:
    import ujson as json

import image
import uos
from machine import FPIOA, UART
from media.display import *
from media.media import *
from media.sensor import *
from ybUtils.YbKey import YbKey

try:
    import cv_lite
except Exception:
    cv_lite = None

try:
    from ulab import numpy as np
except Exception:
    np = None


# ---------------------------------------------------------------------------
# 用户配置
# ---------------------------------------------------------------------------

# 每次修改本文件时更新版本号，使启动日志能确认
# 板端实际运行的版本。
VISION_VERSION = "v111-20261104-fixed-laser-317-230"
RUN_MODE = "PRODUCTION"  # 测试或生产模式

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
REFERENCE_FRAME_WIDTH = 640
REFERENCE_FRAME_HEIGHT = 480
PIXEL_REFERENCE_WIDTH = 480
PIXEL_REFERENCE_HEIGHT = 360
REFERENCE_FOCAL_LENGTH_PX = 380.0
# 像素类常数以 480x360 为基准；320x240 的老标定值只用于换算角度基准。
PIXEL_SCALE = FRAME_WIDTH / PIXEL_REFERENCE_WIDTH
SENSOR_ID = 2
SENSOR_SOURCE_WIDTH = 1280
SENSOR_SOURCE_HEIGHT = 960
SENSOR_FPS = 90
TARGET_PROCESS_FPS = 80
FRAME_INTERVAL_MS = 1000 // TARGET_PROCESS_FPS
# 检测通道本身已经是 640x480；高分辨率通道仅保留给 IDE 预览。
# 独立高分辨率搜索不再有意义，关闭以避免重复取图和重复检测。
#
# 将间隔从 3 改回 1：即使跳过快照，ISP 仍会继续生成
# 通道 1，缓冲池也会持续轮转，而显示引擎仍在扫描上一次显示的缓冲，
# 从而出现撕裂预览。34/111 厘米测试后的截图
# 正是该现象：绿色边框只有两条完整竖边，
# 上下边只剩右半部分，即画面撕裂，而不是
# 检测错误（日志中的框在 40 多帧内稳定为 92.3x65.3 像素，比例 1.41）。
# 每帧取图并重新显示，可确保显示缓冲始终是最新一帧。
DISPLAY_EVERY_N_FRAMES = 1
# 光照统计和垃圾回收开销都较大，无需每帧执行。
PROFILE_EVERY_N_FRAMES = 15
GC_EVERY_N_FRAMES = 150

# 测试模式设置。
# 每个距离分别改成 laser_050 / laser_100 / laser_150。
TEST_CASE_NAME = "laser_150"
TEST_SAVE_ROOT = "/data/snapshot"
TEST_DISPLAY_ENABLED = True
# 标定抓图时关闭视觉指令发送，避免云台运动改变拍摄视角。
TEST_UART_ENABLED = False

# 生产联调期间按板载用户键保存一张无叠加 RGB 原图。
PRODUCTION_SNAPSHOT_ENABLED = True
PRODUCTION_SNAPSHOT_ROOT = "/data/snapshot"

DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
HIGH_RES_WIDTH = 640
HIGH_RES_HEIGHT = 480
# 高分辨率 ROI 微调未验证稳定前保持关闭。
# 640x480 通道仍只作为 IDE 预览缓冲。
HIGH_RES_VERIFY_ENABLED = False
HIGH_RES_VERIFY_EVERY_N_FRAMES = 2
HIGH_RES_VERIFY_MIN_WIDTH = 75 * PIXEL_SCALE
HIGH_RES_VERIFY_DISTANCE_M = 1.2

# 检测通道已经是 640x480，旧的 640x480 独立搜索仅保留为
# 已关闭的兼容路径，避免重复取图和重复检测。
HIGH_RES_SEARCH_ENABLED = False
HIGH_RES_SEARCH_EVERY_N_FRAMES = 3

# 联调显示通道：只查看角点检测后的原始候选四边形。
DISPLAY_ENABLED = True
DISPLAY_DRAW_RAW_RECTANGLES = True
DISPLAY_DRAW_FINAL_TARGET = False
DISPLAY_DRAW_LASER_PREDICTION = False
DISPLAY_DRAW_TARGET_TEXT = False

UART_BAUDRATE = 921600
UART_TX_PIN = 9
UART_RX_PIN = 10
VISION_LOST_VALUE = 404
VISION_LOST_SEND_INTERVAL_MS = 100

# 几何激光落点开关。关闭时只保留矩形检测/靶心跟踪，瞄准使用固定点。
GEOMETRY_LANDING_ENABLED = False
FIXED_LASER_POINT_X_PX = 317.0
FIXED_LASER_POINT_Y_PX = 230.0
GIMBAL_ANGLE_PER_PIXEL_DEG = 0.28 * 320.0 / FRAME_WIDTH

# 目标外框 300×210 mm，实际内框 260×170 mm，
# 因此黑胶带宽度为 (300-260)/2 = (210-170)/2 = 20 mm。
TARGET_OUTER_WIDTH_M = 0.30
TARGET_OUTER_HEIGHT_M = 0.21
BLACK_BORDER_WIDTH_M = 0.020
FOCAL_LENGTH_PX = REFERENCE_FOCAL_LENGTH_PX * FRAME_WIDTH / REFERENCE_FRAME_WIDTH
MIN_VALID_DISTANCE_M = 0.30
MAX_VALID_DISTANCE_M = 1.60

# 相机内参、畸变和激光相对相机的位姿。
# 使用 2026-11-04 当场多距离、多视角标定结果。
CALIBRATION_JSON_PATH = "/data/calibration.json"
# 当前相机和激光参数已按本车重新标定，禁止旧 calibration.json 覆盖。
USE_CAMERA_JSON_CALIBRATION = False
# 本次实车激光标定结果已经固化在下面；禁止旧 calibration.json 覆盖
# laser.origin_mm / laser.direction，避免板端继续使用参考项目参数。
USE_LASER_JSON_CALIBRATION = False
CAMERA_CALIBRATION = {
    # 2026-11-04 使用当前设备 0.5/1.0/1.5 m 多视角内框联合标定。
    "fx": 610.783983,
    "fy": 621.307476,
    "cx": 313.270140,
    "cy": 238.263723,
    "distortion": (
        -0.0303490623,
        -0.0389638370,
        0.0,
        0.0,
        0.1146951914,
    ),
}
# 2026-11-04 使用 0.5/1.0/1.5 m 共 17 张内框样本联合拟合。
LASER_ORIGIN_MM = (-1.068, 17.549, 0.0)
LASER_DIRECTION = (0.003813, -0.007213, 0.999967)
GEOMETRY_MAX_REPROJECTION_PX = 12.5
GEOMETRY_REPROJECTION_FRACTION = 0.08
GEOMETRY_MAX_REPROJECTION_CAP_PX = 16.0
GEOMETRY_MAX_TILT_DEG = 65.0
GEOMETRY_MIN_DEPTH_MM = 250.0
GEOMETRY_MAX_DEPTH_MM = 4000.0
GEOMETRY_MAX_COMMAND_DEG = 60.0
GEOMETRY_REFINE_TRIGGER_PX = 1.5
GEOMETRY_REFINE_ITERATIONS = 2
GEOMETRY_CORNER_REFINE_ENABLED = True
GEOMETRY_CORNER_SEARCH_PX = 3
GEOMETRY_CORNER_SAMPLES = 11
GEOMETRY_CORNER_MAX_SHIFT_PX = 2.0

# 灰度光斑慢速校正。只在靶纸内部搜索，每 5 个有效跟踪帧更新一次。
SPOT_CORRECTION_ENABLED = True
SPOT_CORRECTION_EVERY = 5
SPOT_CORRECTION_ALPHA = 0.25
SPOT_MIN_PEAK = 200
SPOT_MIN_CONTRAST = 45
SPOT_CORE_DROP = 25
SPOT_WINDOW_PX = 20
SPOT_BORDER_FRACTION = 0.5
SPOT_PAPER_MARGIN_MM = 4.0
SPOT_GATE_DEG = 3.0
SPOT_MAX_CORRECTION_DEG = 4.0

# 对发送给 H723 的俯仰误差进行缓慢自学习补偿。
#
# H723 根据电机角度闭环俯仰，因此电机到达目标角后就会停止，
# 但相机仍可能未到达期望的世界角度。
# 2026-10-11 台架实测：当调试俯仰误差为 0.0045 度时，
# 相机仍偏离世界目标 1.6 度；不同姿态下
# 该偏差在 0.9 到 9.7 度之间。云台看不到这部分残余，
# 因为其自身误差已经为零。
#
# 在此累积该残差可使云台持续修正，直到图像误差
# 归零，最终让激光落到靶心。
# 这是一个慢速外环：只在目标大致静止时学习，
# 具有限幅，并且目标丢失时不清零。
# 2026-10-14 第 4 轮诊断：暂时关掉，用来分清"进入中心区后慢慢趋中被磨进去"
# 究竟是谁在做——是这个慢学习器，还是 H723 那个纯比例位置环。
# 关掉之后 trim 恒为 0，静态基准回到原始零位（残余不再被吸收），读数时注意
# 这个前提；要恢复把下一行改回 True 即可。
PITCH_TRIM_ENABLED = False
PITCH_TRIM_GAIN = 0.02
PITCH_TRIM_LIMIT_DEG = 15.0
PITCH_TRIM_STATIC_PX_PER_S = 40.0 * FRAME_WIDTH / REFERENCE_FRAME_WIDTH

# 联调只使用 H723 的蓝牙遥测，关闭 K230 的 SD 卡周期日志。
RUN_LOG_ENABLED = False
RUN_LOG_PATH = "/data/snapshot/current_run.log"
# 原间隔为 3，每 3 帧中有 1 帧额外耗时 5 到 19 毫秒（字符串格式化和文件写入）。
# 当前间隔 5，便于捕获短时几何失败；现场稳定后可改回 15。
LOG_EVERY_N_FRAMES = 5

# 搜索、确认、跟踪和重捕获统一只使用 cv_lite。
USE_CV_LITE = True

# cv_lite 与 Canny 参数。
CANNY_LOW = 100
CANNY_HIGH = 220
RED_TARGET_CANNY_GAIN = 2.0
NEAR_FIXED_CANNY_DISTANCE_M = 0.70
NEAR_FIXED_CANNY_LOW = 75
NEAR_FIXED_CANNY_HIGH = 185
APPROX_EPSILON = 0.02
AREA_MIN_RATIO = 0.001
# 国赛方案在此处使用 0.5，将真正的筛选交给
# 几何条件；0.3 会在远距离丢弃可用四边形。
MAX_ANGLE_COS = 0.5
# 新靶使用红色圆环，依赖高 Canny 阈值抑制其弱边缘。
GAUSSIAN_BLUR = 1

# 几何优先的筛选条件来自同一脚本，只保留必要的
# 形状硬门槛：远距离时 2 厘米黑框只有 2 到 3 像素宽，硬门槛仍然有效，
# 白暗覆盖率不再作为硬门槛，只作为评分项。
# 按 30x21cm 外框、380px 焦距和 1.5m 实际最远距离预留少量余量。
MIN_TARGET_BOX_AREA = int(
    0.85 * FOCAL_LENGTH_PX * FOCAL_LENGTH_PX *
    TARGET_OUTER_WIDTH_M * TARGET_OUTER_HEIGHT_M /
    (MAX_VALID_DISTANCE_M * MAX_VALID_DISTANCE_M)
)
MIN_TARGET_SIDE_PX = int(round(
    0.85 * FOCAL_LENGTH_PX * TARGET_OUTER_HEIGHT_M /
    MAX_VALID_DISTANCE_M
))
MAX_OPPOSITE_SIDE_DELTA_PX = 120.0 * PIXEL_SCALE
MAX_EDGE_ANGLE_DEG = 30.0

# 480x360 下为675像素，640x480 下为1200像素。
MIN_ABSOLUTE_AREA = int(675 * PIXEL_SCALE * PIXEL_SCALE)
ASPECT_RATIO_MIN = 0.55
ASPECT_RATIO_MAX = 1.85
MIN_RECTANGULARITY = 0.55
NEAR_GEOMETRY_MIN_LONG_PX = int(round(90 * PIXEL_SCALE))
NEAR_ASPECT_RATIO_MIN = 0.40
NEAR_ASPECT_RATIO_MAX = 2.30
NEAR_MIN_RECTANGULARITY = 0.42
NEAR_MAX_EDGE_ANGLE_DEG = 40.0
NEAR_MAX_OPPOSITE_SIDE_DELTA_PX = 160.0 * PIXEL_SCALE
# 白色覆盖率只作为评分项，不作为硬门槛：不均匀光照和反光造成的波动
# 远大于目标模型预测。
ACQUIRE_MIN_WHITE_RATIO = 0.15
TRACK_MIN_WHITE_RATIO = 0.10
# 只有几乎全黑的四边形会被直接丢弃；旧版本曾使用
# 0.05 的硬门槛，这是远距离最先导致失败的条件。
MIN_WHITE_RATIO_FLOOR = 0.02
# 胶带内边缘判据：候选边外侧应为黑胶带，内侧应为白纸。
BORDER_OUTSIDE_PX = 2
BORDER_INSIDE_PX = 3
BORDER_MIN_INSIDE_GRAY = 90
BORDER_MIN_CONTRAST = 30
MIN_INNER_EDGE_SCORE = 8
MIN_DARK_BORDER_RATIO = 0.05
# 同一目标的宽度和高度应给出相近的相机距离。
# 仅作为评分项：透视条件下，即使四边形正确，两者也可能不一致。
MIN_DISTANCE_CONSISTENCY = 0.55

# 远距离识别：约 1.2 米外的目标黑框只有 2 到 3 像素，
# 因此在空搜索帧中再执行一次低 Canny 阈值检测。
LONG_RANGE_CANNY_SCALE = 0.70

# 亮度配置采样：画面任意位置的灯曾通过全局最大值抬高
# 白色阈值，因此现在只从画面中央区域采样，
# 并使用分位数而不是最小值和最大值。
PROFILE_ROI_FRACTION = 0.60
PROFILE_LOW_PERCENTILE = 0.05
PROFILE_HIGH_PERCENTILE = 0.95

# 时序逻辑。
CONFIRM_WINDOW_FRAMES = 5
CONFIRM_HITS_REQUIRED = 3
# 快速重捕获：确认阶段落在预测框上的候选
# 直接计为完整命中，短暂闪烁一两帧的目标
# 可立即恢复，无需重新满足五帧内命中三次的条件。
CONFIRM_FAST_CENTER_FACTOR = 0.60
CONFIRM_FAST_AREA_RATIO_MIN = 0.70
CONFIRM_FAST_AREA_RATIO_MAX = 1.40
# 离开跟踪状态前允许连续漏检的帧数。远距离偶尔漏检
# 属于正常现象，而随后重捕获大约需要二十个
# 无目标帧，这期间云台完全看不到目标，因此短暂丢失
# 不应直接破坏锁定。
HOLD_LOST_FRAMES = 12
# 短暂丢失预测：跟踪器丢失目标时不再立即发送 404，
# 而是按目标速度外推上一个框，并在该时间内继续发送，
# 使云台在空档期继续运动，避免先停住再突然跳动。
PREDICT_SEND_MS = 250
TRACK_AREA_RATIO_MIN = 0.78
TRACK_AREA_RATIO_MAX = 1.30
TRACK_AXIS_RATIO_MIN = 0.90
TRACK_AXIS_RATIO_MAX = 1.12
TRACK_CENTER_RADIUS_FACTOR = 0.75
NEAR_TRACK_AREA_RATIO_MIN = 0.65
NEAR_TRACK_AREA_RATIO_MAX = 1.60
NEAR_TRACK_AXIS_RATIO_MIN = 0.80
NEAR_TRACK_AXIS_RATIO_MAX = 1.30
NEAR_TRACK_CENTER_RADIUS_FACTOR = 1.00
INNER_CANDIDATE_SCORE_FACTOR = 2.00
# 原值为 0.60。跟踪 ROI 是真实裁剪区域（裁剪检测与 cv_lite），
# 因此边距直接决定开销：面积按 ((1+2m)^2) 缩放，0.60 改为 0.30 后
# 裁剪面积降至 53%，这是检测阶段最大的一项提速。
# 在 80 帧每秒下，目标每帧只移动几个像素，每侧 0.30 个框宽
# 仍然足够。若快速移动时开始频繁丢锁，
# 应先将该值调回 0.45 附近。
TRACK_DETECTION_ROI_MARGIN_FACTOR = 0.30
NEAR_TRACK_DISTANCE_M = 0.90
NEAR_TRACK_DETECTION_ROI_MARGIN_FACTOR = 0.30
# 跟踪模式中的低成本预筛选：近距离大目标会产生大量四边形，
# build_candidate（白暗采样与几何计算）每帧可能耗时 5 到 11 毫秒，
# 而跟踪器最终也会丢弃远离预测位置的候选，因此先按中心距离
# 过滤四边形。每轴 1.0 个框宽已经比较宽松，
# 即使目标一帧移动半个框宽也不会被丢弃。
TRACK_CANDIDATE_GATE_FACTOR = 1.00
LAYER_SWITCH_SCORE_FACTOR = 0.55
LAYER_SWITCH_VOTES = 2
LAYER_ALTERNATE_AREA_RATIO_MIN = 0.55
LAYER_ALTERNATE_AREA_RATIO_MAX = 1.55
LAYER_ALTERNATE_AXIS_RATIO_MIN = 0.72
LAYER_ALTERNATE_AXIS_RATIO_MAX = 1.30
CONFIRM_OUTER_AREA_RATIO_MIN = 1.08
CONFIRM_OUTER_AREA_RATIO_MAX = 1.45
MAX_TRACK_VELOCITY_PX_PER_S = 2250.0 * PIXEL_SCALE
SMOOTH_CENTER_ALPHA = 0.50
SMOOTH_SIZE_ALPHA = 0.30
RECAPTURE_ROI_FRAMES = 8
RECAPTURE_FULL_FRAMES = 12
RECAPTURE_ROI_MARGIN_FACTOR = 0.30
LIGHT_FALLBACK_ENABLED = True

# 使用 None 保持传感器增益不变；后续若暗光测试显示画面过暗，
# 可使用约 16 到 24 的增益值。
SENSOR_GAIN = None

DEBUG_PRINT = False
DEBUG_PRINT_EVERY_N_FRAMES = 120


# ---------------------------------------------------------------------------
# 亮度配置。
# ---------------------------------------------------------------------------

PROFILE_DARK = {
    "canny_low": 35,
    "canny_high": 100,
    "white_threshold": 105,
}

PROFILE_NORMAL = {
    "canny_low": CANNY_LOW,
    "canny_high": CANNY_HIGH,
    "white_threshold": 165,
}

PROFILE_BRIGHT = {
    "canny_low": 70,
    "canny_high": 180,
    "white_threshold": 200,
}


def get_gray_value(img, x, y):
    try:
        value = img.get_pixel(int(x), int(y))
    except Exception:
        return 0

    if isinstance(value, (tuple, list)):
        return (value[0] + value[1] + value[2]) // 3
    return value


def estimate_mean_brightness(img):
    return sample_image_statistics(img)["mean"]


def sample_image_statistics(img, roi=None, step=24):
    if roi is None:
        x0 = 0
        y0 = 0
        x1 = FRAME_WIDTH
        y1 = FRAME_HEIGHT
    else:
        x0 = max(0, int(roi[0]))
        y0 = max(0, int(roi[1]))
        x1 = min(FRAME_WIDTH, int(roi[0] + roi[2]))
        y1 = min(FRAME_HEIGHT, int(roi[1] + roi[3]))

    if x1 <= x0 or y1 <= y0:
        return {
            "mean": 0,
            "min": 0,
            "max": 0,
            "spread": 0,
            "median": 0,
            "count": 0,
        }

    samples = []
    total = 0

    for y in range(y0 + step // 2, y1, step):
        for x in range(x0 + step // 2, x1, step):
            value = get_gray_value(img, x, y)
            samples.append(value)
            total += value

    count = len(samples)
    if count == 0:
        return {
            "mean": 0,
            "min": 0,
            "max": 0,
            "spread": 0,
            "median": 0,
            "count": 0,
        }

    # 使用分位数代替最小值和最大值，避免单个灯具或暗物体
    # 使计算出的阈值大幅摆动。
    samples.sort()
    low_value = samples[int(count * PROFILE_LOW_PERCENTILE)]
    high_index = min(count - 1, int(count * PROFILE_HIGH_PERCENTILE))
    high_value = samples[high_index]

    return {
        "mean": total // count,
        "min": low_value,
        "max": high_value,
        "spread": high_value - low_value,
        "median": samples[count // 2],
        "count": count,
    }


def center_profile_roi():
    """画面中央区域；没有跟踪 ROI 时使用该区域。"""
    width = int(FRAME_WIDTH * PROFILE_ROI_FRACTION)
    height = int(FRAME_HEIGHT * PROFILE_ROI_FRACTION)
    return ((FRAME_WIDTH - width) // 2, (FRAME_HEIGHT - height) // 2, width, height)


def make_near_fixed_profile(previous_profile):
    """近距离复用上一帧光照参数，只固定 Canny 阈值。"""
    return {
        "level": "near_fixed",
        "mean": previous_profile.get("mean", 0),
        "min": previous_profile.get("min", 0),
        "max": previous_profile.get("max", 0),
        "spread": previous_profile.get("spread", 0),
        "canny_low": NEAR_FIXED_CANNY_LOW,
        "canny_high": NEAR_FIXED_CANNY_HIGH,
        "white_threshold": previous_profile.get(
            "white_threshold", PROFILE_NORMAL["white_threshold"]
        ),
    }


def make_dynamic_profile(
    img,
    roi=None,
    near_target=False,
    previous_profile=None,
):
    if near_target and previous_profile is not None:
        return make_near_fixed_profile(previous_profile)

    # 过去画面边缘的灯会通过全局最大值抬高白色阈值，
    # 因此搜索配置现在只从画面中央区域读取。
    if roi is None:
        roi = center_profile_roi()

    stats = sample_image_statistics(img, roi)
    mean_value = stats["mean"]
    spread = stats["spread"]

    if mean_value < 70:
        level = "dark"
    elif mean_value > 190:
        level = "bright"
    else:
        level = "normal"

    # 保留按画面亮度动态调节，并整体提高阈值，
    # 优先保留黑胶带的高对比边缘，抑制红色圆环的弱边缘。
    base_low = int(mean_value * 0.24 + 20)
    base_low = max(25, min(80, base_low))
    base_high = int(base_low * 2.6)
    base_high = max(base_low + 35, min(200, base_high))
    canny_low = int(base_low * RED_TARGET_CANNY_GAIN)
    canny_low = max(35, min(145, canny_low))
    canny_high = int(base_high * RED_TARGET_CANNY_GAIN)
    canny_high = max(canny_low + 60, min(255, canny_high))

    white_threshold = int(stats["min"] + spread * 0.52)
    white_threshold = max(85, min(215, white_threshold))

    return {
        "level": level,
        "mean": mean_value,
        "min": stats["min"],
        "max": stats["max"],
        "spread": spread,
        "canny_low": canny_low,
        "canny_high": canny_high,
        "white_threshold": white_threshold,
    }


def make_light_fallback_profile(profile):
    """为一次全图重捕获构建较低阈值的检测配置。"""
    canny_low = max(15, int(profile["canny_low"] * 0.72))
    canny_high = max(canny_low + 30, int(profile["canny_high"] * 0.78))

    return {
        "level": profile["level"] + "_fallback",
        "mean": profile["mean"],
        "min": profile["min"],
        "max": profile["max"],
        "spread": profile["spread"],
        "canny_low": canny_low,
        "canny_high": canny_high,
        "white_threshold": max(70, profile["white_threshold"] - 20),
    }


def make_long_range_profile(profile):
    """用于远距离目标的低阈值配置，此时边框仅约 2 到 3 像素宽。

    模糊核和默认 Canny 参数针对 4 到 8 像素宽的边框调整。
    超过约 1.2 米后，边框比模糊核还细，因此需要使用更低边缘阈值
    进行第二次检测才能可靠找到目标。
    """
    canny_low = max(15, int(profile["canny_low"] * LONG_RANGE_CANNY_SCALE))
    canny_high = max(canny_low + 30, int(profile["canny_high"] * 0.78))

    return {
        "level": profile["level"] + "_long",
        "mean": profile["mean"],
        "min": profile["min"],
        "max": profile["max"],
        "spread": profile["spread"],
        "canny_low": canny_low,
        "canny_high": canny_high,
        "white_threshold": max(70, profile["white_threshold"] - 10),
    }


def make_tracking_roi(last_candidate, margin_factor=0.30):
    if last_candidate is None:
        return None

    margin_x = max(20, int(last_candidate["w"] * margin_factor))
    margin_y = max(20, int(last_candidate["h"] * margin_factor))
    x = max(0, int(last_candidate["x"] - margin_x))
    y = max(0, int(last_candidate["y"] - margin_y))
    width = min(FRAME_WIDTH - x, int(last_candidate["w"] + 2 * margin_x))
    height = min(FRAME_HEIGHT - y, int(last_candidate["h"] + 2 * margin_y))
    return (x, y, width, height)


def crop_for_detection(img, roi):
    if roi is None:
        return img, 0, 0

    x = max(0, int(roi[0]))
    y = max(0, int(roi[1]))
    width = min(FRAME_WIDTH - x, int(roi[2]))
    height = min(FRAME_HEIGHT - y, int(roi[3]))

    if width < 32 or height < 32:
        return img, 0, 0

    try:
        return img.copy(roi=(x, y, width, height)), x, y
    except Exception:
        return img, 0, 0


def pace_frame(last_frame_ms):
    elapsed_ms = time.ticks_diff(time.ticks_ms(), last_frame_ms)
    if elapsed_ms < FRAME_INTERVAL_MS:
        time.sleep_ms(FRAME_INTERVAL_MS - elapsed_ms)
    return time.ticks_ms()


# ---------------------------------------------------------------------------
# 几何辅助函数。
# ---------------------------------------------------------------------------


def polygon_area(corners):
    area = 0.0
    count = len(corners)

    for i in range(count):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % count]
        area += x1 * y2 - x2 * y1

    return abs(area) * 0.5


def bounding_box(corners):
    xs = [point[0] for point in corners]
    ys = [point[1] for point in corners]
    min_x = int(min(xs))
    max_x = int(max(xs))
    min_y = int(min(ys))
    max_y = int(max(ys))
    return min_x, min_y, max_x - min_x + 1, max_y - min_y + 1


def center_of_corners(corners):
    x_sum = 0.0
    y_sum = 0.0

    for point in corners:
        x_sum += point[0]
        y_sum += point[1]

    return x_sum / len(corners), y_sum / len(corners)


def perspective_center_of_corners(corners):
    """直接按透视关系计算投影矩形的中心。

    检测到的四个角点是平面矩形的投影；目标未正对相机时会形成梯形，
    此时简单平均四个角点会偏向较近的一侧。因此改用两条对角线的交点：
    利用投影变换可以直接得到矩形中心的对应点，计算量仅为两次叉积。

    当角点未按环形顺序排列，或四边形退化、两条对角线不在两条边段
    内部相交时，回退到普通平均值。
    """
    if len(corners) != 4:
        return center_of_corners(corners)

    (x0, y0), (x1, y1), (x2, y2), (x3, y3) = corners
    diagonal_a_x = x2 - x0
    diagonal_a_y = y2 - y0
    diagonal_b_x = x3 - x1
    diagonal_b_y = y3 - y1

    denominator = diagonal_a_x * diagonal_b_y - diagonal_a_y * diagonal_b_x
    if abs(denominator) < 1e-6:
        return center_of_corners(corners)

    offset_x = x1 - x0
    offset_y = y1 - y0
    t = (offset_x * diagonal_b_y - offset_y * diagonal_b_x) / denominator
    s = (offset_x * diagonal_a_y - offset_y * diagonal_a_x) / denominator
    if t < -0.05 or t > 1.05 or s < -0.05 or s > 1.05:
        return center_of_corners(corners)

    return x0 + t * diagonal_a_x, y0 + t * diagonal_a_y


def angle_between(v1, v2):
    dot = v1[0] * v2[0] + v1[1] * v2[1]
    length_1 = math.sqrt(v1[0] * v1[0] + v1[1] * v1[1])
    length_2 = math.sqrt(v2[0] * v2[0] + v2[1] * v2[1])

    if length_1 == 0 or length_2 == 0:
        return 0.0

    cosine = dot / (length_1 * length_2)
    cosine = max(-1.0, min(1.0, cosine))
    return math.degrees(math.acos(cosine))


def edge_direction_deg(vector):
    return math.degrees(math.atan2(vector[1], vector[0]))


def line_angle_difference_deg(first, second):
    difference = abs(first - second) % 180.0
    return min(difference, 180.0 - difference)


def rectangle_shape_ok(corners, near_target=False):
    """仅依据几何关系判断目标，参考 2025 年电赛国赛方案。

    四项检查均不读取像素值：每条边足够长、对边长度接近、
    对边平行、相邻边垂直。亮度类门槛在目标最困难的场景下容易失效，
    例如 1.6 米处 2 厘米边框仅约三个像素宽；几何条件仍能稳定成立，
    因此现在由这些条件承担主要筛选。
    """
    if len(corners) != 4:
        return False

    vectors = []
    lengths = []
    for index in range(4):
        x1, y1 = corners[index]
        x2, y2 = corners[(index + 1) % 4]
        vector = (x2 - x1, y2 - y1)
        vectors.append(vector)
        lengths.append(math.sqrt(vector[0] * vector[0] + vector[1] * vector[1]))

    if min(lengths) < MIN_TARGET_SIDE_PX:
        return False

    opposite_limit = (NEAR_MAX_OPPOSITE_SIDE_DELTA_PX
                      if near_target else MAX_OPPOSITE_SIDE_DELTA_PX)
    angle_limit = (NEAR_MAX_EDGE_ANGLE_DEG
                   if near_target else MAX_EDGE_ANGLE_DEG)

    if abs(lengths[0] - lengths[2]) > opposite_limit:
        return False
    if abs(lengths[1] - lengths[3]) > opposite_limit:
        return False

    angles = [edge_direction_deg(vector) for vector in vectors]
    if line_angle_difference_deg(angles[0], angles[2]) > angle_limit:
        return False
    if line_angle_difference_deg(angles[1], angles[3]) > angle_limit:
        return False

    for index in range(4):
        perpendicular = line_angle_difference_deg(
            angles[index], angles[(index + 1) % 4]
        )
        if abs(perpendicular - 90.0) > angle_limit:
            return False

    return True


def point_in_polygon(x, y, corners):
    inside = False
    j = len(corners) - 1

    for i in range(len(corners)):
        xi, yi = corners[i]
        xj, yj = corners[j]

        if ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / (yj - yi) + xi
        ):
            inside = not inside

        j = i

    return inside


def sample_white_ratio(img, corners, white_threshold):
    x, y, width, height = bounding_box(corners)
    if width < 3 or height < 3:
        return 0.0

    # 采样步长比整除 10 更粗：样本数受 (框尺寸/步长)^2 限制，步长越大
    # Python 开销越低。回退前已用整除 8 在 55 帧每秒下验证；
    # 若白色比例噪声增大，再改回整除 10。
    step = max(3, min(width, height) // 8)
    white_pixels = 0
    total_pixels = 0

    for sample_y in range(y + step // 2, y + height, step):
        for sample_x in range(x + step // 2, x + width, step):
            if point_in_polygon(sample_x, sample_y, corners):
                total_pixels += 1
                if get_gray_value(img, sample_x, sample_y) >= white_threshold:
                    white_pixels += 1

    if total_pixels == 0:
        return 0.0
    return white_pixels / total_pixels


def order_corners(corners):
    center_x, center_y = center_of_corners(corners)
    return sorted(
        corners,
        key=lambda point: math.atan2(
            point[1] - center_y,
            point[0] - center_x,
        ),
    )


def sample_inner_edge_score(img, corners, white_threshold):
    """统计四条边满足“外侧黑胶带、内侧白纸”的次数。"""
    ordered = order_corners(corners)
    if len(ordered) != 4:
        return 0

    center_x, center_y = center_of_corners(ordered)
    dark_threshold = max(35, min(120, int(white_threshold * 0.60)))
    inside_threshold = max(
        BORDER_MIN_INSIDE_GRAY,
        min(155, int(white_threshold * 0.72)),
    )
    score = 0

    for index in range(4):
        first_x, first_y = ordered[index]
        second_x, second_y = ordered[(index + 1) % 4]
        edge_x = second_x - first_x
        edge_y = second_y - first_y
        edge_length = math.sqrt(edge_x * edge_x + edge_y * edge_y)
        if edge_length <= 0.0:
            continue

        normal_x = edge_y / edge_length
        normal_y = -edge_x / edge_length
        midpoint_x = (first_x + second_x) * 0.5
        midpoint_y = (first_y + second_y) * 0.5
        if (normal_x * (midpoint_x - center_x) +
                normal_y * (midpoint_y - center_y)) < 0.0:
            normal_x = -normal_x
            normal_y = -normal_y

        for fraction in (0.25, 0.50, 0.75):
            edge_point_x = first_x + edge_x * fraction
            edge_point_y = first_y + edge_y * fraction
            outside_x = int(round(
                edge_point_x + normal_x * BORDER_OUTSIDE_PX
            ))
            outside_y = int(round(
                edge_point_y + normal_y * BORDER_OUTSIDE_PX
            ))
            if (
                outside_x < 0 or outside_x >= FRAME_WIDTH
                or outside_y < 0 or outside_y >= FRAME_HEIGHT
            ):
                continue
            outside = get_gray_value(img, outside_x, outside_y)

            inside_total = 0
            inside_valid = True
            for offset in range(1, BORDER_INSIDE_PX + 1):
                inside_x = int(round(
                    edge_point_x - normal_x * offset
                ))
                inside_y = int(round(
                    edge_point_y - normal_y * offset
                ))
                if (
                    inside_x < 0 or inside_x >= FRAME_WIDTH
                    or inside_y < 0 or inside_y >= FRAME_HEIGHT
                ):
                    inside_valid = False
                    break
                inside_total += get_gray_value(img, inside_x, inside_y)
            if not inside_valid:
                continue
            inside = inside_total / float(BORDER_INSIDE_PX)

            if (
                outside <= dark_threshold
                and inside >= inside_threshold
                and inside - outside >= BORDER_MIN_CONTRAST
            ):
                score += 1

    return score


def bump_diagnostic(diagnostics, name, amount=1):
    if diagnostics is None:
        return
    diagnostics[name] = diagnostics.get(name, 0) + amount


def calculate_distance_metrics(width, height, is_inner):
    width_m = TARGET_OUTER_WIDTH_M
    height_m = TARGET_OUTER_HEIGHT_M

    if is_inner:
        width_m -= 2.0 * BLACK_BORDER_WIDTH_M
        height_m -= 2.0 * BLACK_BORDER_WIDTH_M

    distance_width = FOCAL_LENGTH_PX * width_m / max(1.0, width)
    distance_height = FOCAL_LENGTH_PX * height_m / max(1.0, height)
    larger_distance = max(distance_width, distance_height)

    if larger_distance <= 0:
        distance_consistency = 0.0
    else:
        distance_consistency = min(distance_width, distance_height) / larger_distance

    return {
        "distance_m": (distance_width + distance_height) * 0.5,
        "distance_width_m": distance_width,
        "distance_height_m": distance_height,
        "distance_consistency": distance_consistency,
    }


def refresh_candidate_distance(candidate):
    metrics = calculate_distance_metrics(
        candidate["w"], candidate["h"], candidate["inner"]
    )
    candidate["distance_m"] = metrics["distance_m"]
    candidate["distance_width_m"] = metrics["distance_width_m"]
    candidate["distance_height_m"] = metrics["distance_height_m"]
    candidate["distance_consistency"] = metrics["distance_consistency"]


def build_candidate(
    img,
    raw_corners,
    profile,
    source="unknown",
    diagnostics=None,
    min_white_ratio=ACQUIRE_MIN_WHITE_RATIO,
):
    if raw_corners is None or len(raw_corners) != 4:
        return None

    corners = []
    for point in raw_corners:
        corners.append((float(point[0]), float(point[1])))

    area = polygon_area(corners)
    if area < MIN_ABSOLUTE_AREA or area < FRAME_WIDTH * FRAME_HEIGHT * AREA_MIN_RATIO:
        bump_diagnostic(diagnostics, "area_reject")
        return None

    x, y, width, height = bounding_box(corners)
    if width <= 0 or height <= 0:
        bump_diagnostic(diagnostics, "area_reject")
        return None

    near_target = max(width, height) >= NEAR_GEOMETRY_MIN_LONG_PX
    aspect_min = NEAR_ASPECT_RATIO_MIN if near_target else ASPECT_RATIO_MIN
    aspect_max = NEAR_ASPECT_RATIO_MAX if near_target else ASPECT_RATIO_MAX
    min_rectangularity = (NEAR_MIN_RECTANGULARITY
                          if near_target else MIN_RECTANGULARITY)

    aspect_ratio = width / height
    if aspect_ratio < aspect_min or aspect_ratio > aspect_max:
        bump_diagnostic(diagnostics, "aspect_reject")
        return None

    if width * height < MIN_TARGET_BOX_AREA:
        bump_diagnostic(diagnostics, "area_reject")
        return None

    if not rectangle_shape_ok(corners, near_target):
        bump_diagnostic(diagnostics, "angle_reject")
        return None

    rectangularity = area / (width * height)
    if rectangularity < min_rectangularity:
        bump_diagnostic(diagnostics, "rectangularity_reject")
        return None

    white_ratio = sample_white_ratio(img, corners, profile["white_threshold"])
    # 低于下限时，四边形完全不是白色目标；高于下限但低于
    # 名义要求时只扣分，因为反光和不均匀光照造成的覆盖率波动
    # 大于模型预测。
    if white_ratio < MIN_WHITE_RATIO_FLOOR:
        bump_diagnostic(diagnostics, "white_reject")
        return None
    if white_ratio < min_white_ratio:
        bump_diagnostic(diagnostics, "white_low")

    inner_edge_score = sample_inner_edge_score(
        img, corners, profile["white_threshold"]
    )
    dark_border_ratio = inner_edge_score / 12.0
    if inner_edge_score < MIN_INNER_EDGE_SCORE:
        bump_diagnostic(diagnostics, "dark_border_low")

    center_x, center_y = perspective_center_of_corners(corners)
    is_inner = inner_edge_score >= MIN_INNER_EDGE_SCORE
    distance_metrics = calculate_distance_metrics(width, height, is_inner)
    distance_consistency = distance_metrics["distance_consistency"]

    absolute_quality = (0.4 + 0.6 * min(1.0, white_ratio))
    absolute_quality *= 0.75 + 0.25 * min(
        1.0, white_ratio / max(MIN_WHITE_RATIO_FLOOR, min_white_ratio)
    )
    absolute_quality *= 0.4 + 0.6 * min(1.0, rectangularity)
    absolute_quality *= 0.65 + 0.35 * distance_consistency
    absolute_quality *= 0.55 + 0.45 * dark_border_ratio
    score = area * absolute_quality

    return {
        "x": x,
        "y": y,
        "w": width,
        "h": height,
        "cx": center_x,
        "cy": center_y,
        "area": area,
        "white": white_ratio,
        "dark_border": dark_border_ratio,
        "inner_edge_score": inner_edge_score,
        "rect": rectangularity,
        "distance_m": distance_metrics["distance_m"],
        "distance_width_m": distance_metrics["distance_width_m"],
        "distance_height_m": distance_metrics["distance_height_m"],
        "distance_consistency": distance_consistency,
        "base_score": score,
        "score": score,
        "corners": corners,
        "source": source,
        "layer": "inner" if is_inner else "unknown",
        "alternate_layer": False,
        "inner": is_inner,
    }


def mark_nested_candidates(candidates):
    for index, candidate in enumerate(candidates):
        for other in candidates:
            if candidate is other:
                continue

            if other["area"] < candidate["area"] * 1.15:
                continue

            center_distance_x = abs(candidate["cx"] - other["cx"])
            center_distance_y = abs(candidate["cy"] - other["cy"])
            if (
                center_distance_x < other["w"] * 0.2
                and center_distance_y < other["h"] * 0.2
            ):
                candidate["inner"] = True
                candidate["layer"] = "inner"
                if other["layer"] == "unknown":
                    other["layer"] = "outer"
                refresh_candidate_distance(candidate)
                break


def candidate_in_distance_range(candidate):
    return (
        MIN_VALID_DISTANCE_M
        <= candidate["distance_m"]
        <= MAX_VALID_DISTANCE_M
    )


def candidate_center_in_roi(candidate, roi):
    if roi is None:
        return True

    return (
        roi[0] <= candidate["cx"] <= roi[0] + roi[2]
        and roi[1] <= candidate["cy"] <= roi[1] + roi[3]
    )


def center_distance(candidate_a, candidate_b):
    dx = candidate_a["cx"] - candidate_b["cx"]
    dy = candidate_a["cy"] - candidate_b["cy"]
    return math.sqrt(dx * dx + dy * dy)


def tracking_gate_limits(reference):
    near_target = (
        reference is not None
        and max(reference["w"], reference["h"]) >= NEAR_GEOMETRY_MIN_LONG_PX
    )
    if near_target:
        return (
            NEAR_TRACK_AREA_RATIO_MIN,
            NEAR_TRACK_AREA_RATIO_MAX,
            NEAR_TRACK_AXIS_RATIO_MIN,
            NEAR_TRACK_AXIS_RATIO_MAX,
            NEAR_TRACK_CENTER_RADIUS_FACTOR,
        )
    return (
        TRACK_AREA_RATIO_MIN,
        TRACK_AREA_RATIO_MAX,
        TRACK_AXIS_RATIO_MIN,
        TRACK_AXIS_RATIO_MAX,
        TRACK_CENTER_RADIUS_FACTOR,
    )


def set_best_observed_diagnostics(candidates, diagnostics):
    if diagnostics is None:
        return

    if not candidates:
        diagnostics["candidate_source"] = "none"
        diagnostics["best_distance_cm"] = 0.0
        diagnostics["lost_reason"] = "no_candidate"
        return

    observed = None
    for candidate in candidates:
        if observed is None or candidate["base_score"] > observed["base_score"]:
            observed = candidate

    diagnostics["candidate_source"] = observed["source"]
    diagnostics["best_distance_cm"] = observed["distance_m"] * 100.0


def select_best_candidate(candidates, state, last, diagnostics=None, roi=None):
    mark_nested_candidates(candidates)
    set_best_observed_diagnostics(candidates, diagnostics)

    valid_distance_candidates = []
    for candidate in candidates:
        candidate["score"] = candidate["base_score"]

        if not candidate_in_distance_range(candidate):
            bump_diagnostic(diagnostics, "distance_reject")
            continue

        valid_distance_candidates.append(candidate)

    if not valid_distance_candidates:
        if diagnostics is not None and candidates:
            diagnostics["lost_reason"] = "distance"
        return None

    # 当前靶面只稳定产生一个角点候选，且画面确认它贴着黑胶带内沿。
    # 远距时边沿采样不足会让 inner_edge_score 偶发跌破阈值，若不强制
    # 标记，几何计算会错误使用 300x210 外框尺寸。
    if len(valid_distance_candidates) == 1:
        candidate = valid_distance_candidates[0]
        if not candidate["inner"]:
            candidate["inner"] = True
            candidate["layer"] = "inner"
            refresh_candidate_distance(candidate)
            bump_diagnostic(diagnostics, "forced_inner")

    pool = []
    for candidate in valid_distance_candidates:
        if candidate["inner"]:
            candidate["score"] *= INNER_CANDIDATE_SCORE_FACTOR
        pool.append(candidate)
    scored_candidates = []

    if state in ("TRACK", "RECAPTURE_ROI"):
        strict_candidates = []
        alternate_candidates = []
        area_min, area_max, axis_min, axis_max, center_factor = \
            tracking_gate_limits(last)

        for candidate in pool:
            candidate["alternate_layer"] = False

            if not candidate_center_in_roi(candidate, roi):
                bump_diagnostic(diagnostics, "roi_reject")
                continue

            if last is None:
                strict_candidates.append(candidate)
                continue

            # 一旦已经锁定内框，不允许因为远处少检到一帧就立即跳到外框。
            # 内框连续丢失达到状态机重捕获阈值后，才会重新自由选层。
            if last["layer"] == "inner" and not candidate["inner"]:
                bump_diagnostic(diagnostics, "inner_hold_reject")
                continue

            max_center_distance = max(
                25.0, last["w"] * center_factor
            )
            alternate_center_distance = max(
                30.0, last["w"] * max(1.00, center_factor)
            )
            target_center_distance = center_distance(candidate, last)
            if target_center_distance > alternate_center_distance:
                bump_diagnostic(diagnostics, "center_reject")
                continue

            area_ratio = candidate["area"] / last["area"]
            width_ratio = candidate["w"] / max(1.0, last["w"])
            height_ratio = candidate["h"] / max(1.0, last["h"])
            layer_mismatch = (
                last["layer"] != "unknown"
                and candidate["layer"] != "unknown"
                and candidate["layer"] != last["layer"]
            )
            strict_size = (
                area_min <= area_ratio <= area_max
                and axis_min <= width_ratio <= axis_max
                and axis_min <= height_ratio <= axis_max
            )

            if not layer_mismatch and strict_size:
                center_quality = 1.0 - 0.35 * (
                    target_center_distance / max_center_distance
                )
                size_error = abs(area_ratio - 1.0)
                size_error += 0.5 * abs(width_ratio - 1.0)
                size_error += 0.5 * abs(height_ratio - 1.0)
                size_quality = 1.0 / (1.0 + size_error)
                candidate["score"] = (
                    candidate["base_score"]
                    * max(0.35, center_quality)
                    * size_quality
                )
                strict_candidates.append(candidate)
                continue

            alternate_size = (
                LAYER_ALTERNATE_AREA_RATIO_MIN
                <= area_ratio
                <= LAYER_ALTERNATE_AREA_RATIO_MAX
                and LAYER_ALTERNATE_AXIS_RATIO_MIN
                <= width_ratio
                <= LAYER_ALTERNATE_AXIS_RATIO_MAX
                and LAYER_ALTERNATE_AXIS_RATIO_MIN
                <= height_ratio
                <= LAYER_ALTERNATE_AXIS_RATIO_MAX
            )
            if alternate_size:
                candidate["alternate_layer"] = True
                candidate["score"] = (
                    candidate["base_score"] * LAYER_SWITCH_SCORE_FACTOR
                )
                alternate_candidates.append(candidate)
                bump_diagnostic(diagnostics, "alternate_count")
            elif layer_mismatch:
                bump_diagnostic(diagnostics, "layer_reject")
            else:
                bump_diagnostic(diagnostics, "size_reject")

        inner_alternates = [
            candidate
            for candidate in alternate_candidates
            if candidate["inner"]
        ]
        if inner_alternates:
            scored_candidates = inner_alternates
        elif strict_candidates:
            scored_candidates = strict_candidates
        else:
            scored_candidates = alternate_candidates

        if not scored_candidates:
            if diagnostics is not None:
                if roi is not None and diagnostics.get("roi_reject", 0) > 0:
                    diagnostics["lost_reason"] = "roi"
                elif diagnostics.get("alternate_count", 0) > 0:
                    diagnostics["lost_reason"] = "layer"
                elif diagnostics.get("layer_reject", 0) > 0:
                    diagnostics["lost_reason"] = "layer"
                elif diagnostics.get("inner_hold_reject", 0) > 0:
                    diagnostics["lost_reason"] = "inner_hold"
                elif diagnostics.get("size_reject", 0) > 0:
                    diagnostics["lost_reason"] = "size"
                elif diagnostics.get("center_reject", 0) > 0:
                    diagnostics["lost_reason"] = "center"
                else:
                    diagnostics["lost_reason"] = "continuity"
            return None
    else:
        scored_candidates = pool

    best = None
    for candidate in scored_candidates:
        candidate_is_inner = candidate["inner"]
        best_is_inner = best is not None and best["inner"]
        if (
            best is None
            or (candidate_is_inner and not best_is_inner)
            or (
                candidate_is_inner == best_is_inner
                and candidate["score"] > best["score"]
            )
        ):
            best = candidate

    # 最终选中的框如果仍只是 unknown，而本帧没有任何明确的 outer
    # 候选，就按当前靶面实际检测到的内沿处理。这不改变候选选择和跟踪。
    if best is not None and not best["inner"]:
        has_explicit_outer = False
        for candidate in valid_distance_candidates:
            if candidate["layer"] == "outer":
                has_explicit_outer = True
                break
        if not has_explicit_outer:
            best["inner"] = True
            best["layer"] = "inner"
            refresh_candidate_distance(best)
            bump_diagnostic(diagnostics, "forced_inner_final")

    if best is not None and diagnostics is not None:
        diagnostics["candidate_source"] = best["source"]
        diagnostics["best_distance_cm"] = best["distance_m"] * 100.0

    return best


def _vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vec_scale(a, scale):
    return (a[0] * scale, a[1] * scale, a[2] * scale)


def _vec_cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _vec_norm(a):
    return math.sqrt(_vec_dot(a, a))


def _vec_unit(a):
    length = _vec_norm(a)
    if length < 1e-10:
        raise ValueError("degenerate vector")
    return _vec_scale(a, 1.0 / length)


def _solve_linear(matrix, rhs):
    """Small Gaussian elimination used by the 8x8 target homography."""
    count = len(rhs)
    rows = [list(matrix[i]) + [rhs[i]] for i in range(count)]
    for column in range(count):
        pivot = column
        for row in range(column + 1, count):
            if abs(rows[row][column]) > abs(rows[pivot][column]):
                pivot = row
        if abs(rows[pivot][column]) < 1e-12:
            raise ValueError("singular homography")
        rows[column], rows[pivot] = rows[pivot], rows[column]
        divisor = rows[column][column]
        for index in range(column, count + 1):
            rows[column][index] /= divisor
        for row in range(count):
            if row == column:
                continue
            factor = rows[row][column]
            for index in range(column, count + 1):
                rows[row][index] -= factor * rows[column][index]
    return tuple(rows[index][count] for index in range(count))


def _homography(source, destination):
    matrix = []
    rhs = []
    for index in range(4):
        x, y = source[index]
        u, v = destination[index]
        matrix.append((x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y))
        rhs.append(u)
        matrix.append((0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y))
        rhs.append(v)
    return _solve_linear(matrix, rhs) + (1.0,)


def _transform_homography(matrix, point):
    x, y = point
    denominator = matrix[6] * x + matrix[7] * y + matrix[8]
    if abs(denominator) < 1e-10:
        raise ValueError("point at infinity")
    return (
        (matrix[0] * x + matrix[1] * y + matrix[2]) / denominator,
        (matrix[3] * x + matrix[4] * y + matrix[5]) / denominator,
    )


def _geometry_order_quad(points):
    if len(points) != 4:
        raise ValueError("four corners required")
    center_x, center_y = center_of_corners(points)
    ordered = sorted(
        [(float(p[0]), float(p[1])) for p in points],
        key=lambda p: math.atan2(p[1] - center_y, p[0] - center_x),
    )
    start = min(
        range(4),
        key=lambda index: ordered[index][0] + ordered[index][1],
    )
    ordered = ordered[start:] + ordered[:start]
    signs = []
    for index in range(4):
        a = ordered[index]
        b = ordered[(index + 1) % 4]
        c = ordered[(index + 2) % 4]
        signs.append(
            (b[0] - a[0]) * (c[1] - b[1]) -
            (b[1] - a[1]) * (c[0] - b[0])
        )
    if min(signs) <= 1e-5:
        raise ValueError("non-convex quad")
    return tuple(ordered)


class PoseCamera(object):
    def __init__(self, width, height, fx, fy, cx, cy, distortion):
        if fx <= 0.0 or fy <= 0.0 or len(distortion) != 5:
            raise ValueError("invalid camera calibration")
        self.width = width
        self.height = height
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        self.distortion = tuple(float(value) for value in distortion)

    def normalized(self, pixel):
        xd = (pixel[0] - self.cx) / self.fx
        yd = (pixel[1] - self.cy) / self.fy
        x, y = xd, yd
        k1, k2, p1, p2, k3 = self.distortion
        for _ in range(12):
            r2 = x * x + y * y
            radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
            if abs(radial) < 1e-8:
                raise ValueError("invalid distortion")
            x = (xd - 2.0 * p1 * x * y - p2 * (r2 + 2.0 * x * x)) / radial
            y = (yd - p1 * (r2 + 2.0 * y * y) - 2.0 * p2 * x * y) / radial
        projected = self.project((x, y, 1.0))
        error_x = projected[0] - pixel[0]
        error_y = projected[1] - pixel[1]
        if error_x * error_x + error_y * error_y > 0.0625:
            raise ValueError("distortion did not converge")
        return x, y

    def project(self, point):
        if point[2] <= 0.0:
            raise ValueError("point behind camera")
        x = point[0] / point[2]
        y = point[1] / point[2]
        r2 = x * x + y * y
        k1, k2, p1, p2, k3 = self.distortion
        radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        return (
            self.fx * (
                x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
            ) + self.cx,
            self.fy * (
                y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
            ) + self.cy,
        )


class TargetPlanePose(object):
    def __init__(self, r1, r2, translation):
        self.r1 = r1
        self.r2 = r2
        self.translation = translation
        self.normal = _vec_unit(_vec_cross(r1, r2))

    def camera_point(self, plane_point):
        return _vec_add(
            self.translation,
            _vec_add(
                _vec_scale(self.r1, plane_point[0]),
                _vec_scale(self.r2, plane_point[1]),
            ),
        )

    def plane_point(self, camera_point):
        relative = _vec_sub(camera_point, self.translation)
        return (
            _vec_dot(relative, self.r1),
            _vec_dot(relative, self.r2),
        )

    def intersection(self, origin, direction):
        denominator = _vec_dot(self.normal, direction)
        if abs(denominator) < 1e-8:
            raise ValueError("ray parallel to target")
        distance = (
            _vec_dot(self.normal, _vec_sub(self.translation, origin)) /
            denominator
        )
        if distance <= 0.0:
            raise ValueError("intersection behind ray")
        return _vec_add(origin, _vec_scale(direction, distance))


def _rotate_axis_angle(axis_angle, vector):
    angle = _vec_norm(axis_angle)
    if angle < 1e-12:
        return _vec_add(vector, _vec_cross(axis_angle, vector))
    axis = _vec_scale(axis_angle, 1.0 / angle)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return _vec_add(
        _vec_add(
            _vec_scale(vector, cosine),
            _vec_scale(_vec_cross(axis, vector), sine),
        ),
        _vec_scale(axis, _vec_dot(axis, vector) * (1.0 - cosine)),
    )


def _pose_with_delta(pose, delta):
    rotation = delta[:3]
    translation_delta = delta[3:]
    return TargetPlanePose(
        _rotate_axis_angle(rotation, pose.r1),
        _rotate_axis_angle(rotation, pose.r2),
        _vec_add(pose.translation, translation_delta),
    )


def _pose_normalized_residuals(pose, plane, normalized, fx, fy):
    residuals = []
    for index in range(4):
        plane_point = plane[index]
        normalized_point = normalized[index]
        camera_point = pose.camera_point(plane_point)
        if camera_point[2] <= 0.0:
            raise ValueError("point behind camera")
        residuals.append(
            fx * (
                camera_point[0] / camera_point[2] -
                normalized_point[0]
            )
        )
        residuals.append(
            fy * (
                camera_point[1] / camera_point[2] -
                normalized_point[1]
            )
        )
    return residuals


def _refine_target_pose(pose, plane, normalized, fx, fy, iterations):
    """Reference-project Gauss-Newton refinement in undistorted coordinates."""
    try:
        residuals = _pose_normalized_residuals(
            pose, plane, normalized, fx, fy
        )
    except ValueError:
        return pose
    cost = sum(value * value for value in residuals)

    for _ in range(iterations):
        jtj = [[0.0] * 6 for _ in range(6)]
        jtr = [0.0] * 6
        for index in range(4):
            plane_x, plane_y = plane[index]
            rotation_point = _vec_add(
                _vec_scale(pose.r1, plane_x),
                _vec_scale(pose.r2, plane_y),
            )
            camera_point = _vec_add(rotation_point, pose.translation)
            x_value, y_value, z_value = camera_point
            inverse_z = 1.0 / z_value
            ax = inverse_z
            az = -x_value * inverse_z * inverse_z
            by = inverse_z
            bz = -y_value * inverse_z * inverse_z

            ju = (
                fx * az * rotation_point[1],
                fx * (ax * rotation_point[2] - az * rotation_point[0]),
                -fx * ax * rotation_point[1],
                fx * ax,
                0.0,
                fx * az,
            )
            jv = (
                fy * (bz * rotation_point[1] - by * rotation_point[2]),
                -fy * bz * rotation_point[0],
                fy * by * rotation_point[0],
                0.0,
                fy * by,
                fy * bz,
            )
            for row, residual in (
                (ju, residuals[index * 2]),
                (jv, residuals[index * 2 + 1]),
            ):
                for row_index in range(6):
                    jtr[row_index] += row[row_index] * residual
                    for column_index in range(row_index, 6):
                        jtj[row_index][column_index] += (
                            row[row_index] * row[column_index]
                        )

        for row_index in range(6):
            for column_index in range(row_index):
                jtj[row_index][column_index] = jtj[column_index][row_index]

        improved = False
        damping = 0.0
        for _attempt in range(4):
            system = []
            for row_index in range(6):
                row = []
                for column_index in range(6):
                    value = jtj[row_index][column_index]
                    if row_index == column_index:
                        value += damping * jtj[row_index][row_index]
                    row.append(value)
                system.append(row)
            try:
                delta = _solve_linear(
                    system,
                    [-value for value in jtr],
                )
                candidate = _pose_with_delta(pose, delta)
                candidate_residuals = _pose_normalized_residuals(
                    candidate, plane, normalized, fx, fy
                )
            except ValueError:
                damping = damping * 10.0 if damping else 1e-3
                continue
            candidate_cost = sum(
                value * value for value in candidate_residuals
            )
            if candidate_cost < cost:
                converged = candidate_cost > cost * 0.999
                pose = candidate
                residuals = candidate_residuals
                cost = candidate_cost
                improved = True
                break
            damping = damping * 10.0 if damping else 1e-3
        if not improved or converged or cost < 1e-6:
            break
    return pose


def _estimate_target_pose(corners, camera, width_mm, height_mm):
    ordered = _geometry_order_quad(corners)
    plane = (
        (0.0, 0.0),
        (width_mm, 0.0),
        (width_mm, height_mm),
        (0.0, height_mm),
    )
    normalized = [camera.normalized(point) for point in ordered]
    matrix = _homography(plane, normalized)
    column1 = (matrix[0], matrix[3], matrix[6])
    column2 = (matrix[1], matrix[4], matrix[7])
    column3 = (matrix[2], matrix[5], matrix[8])
    norm1 = _vec_norm(column1)
    norm2 = _vec_norm(column2)
    if min(norm1, norm2) < 1e-10:
        raise ValueError("degenerate target pose")
    if abs(norm1 - norm2) / max(norm1, norm2) > 0.30:
        raise ValueError("inconsistent target dimensions")
    factor = 2.0 / (norm1 + norm2)
    if column3[2] < 0.0:
        factor = -factor
    r1 = _vec_unit(_vec_scale(column1, factor))
    raw2 = _vec_scale(column2, factor)
    r2 = _vec_unit(_vec_sub(raw2, _vec_scale(r1, _vec_dot(r1, raw2))))
    pose = TargetPlanePose(r1, r2, _vec_scale(column3, factor))
    residual = 0.0
    for plane_point, image_point in zip(plane, ordered):
        projected = camera.project(pose.camera_point(plane_point))
        dx = projected[0] - image_point[0]
        dy = projected[1] - image_point[1]
        residual = max(residual, math.sqrt(dx * dx + dy * dy))
    diagonal = max(
        math.sqrt(
            (ordered[0][0] - ordered[2][0]) ** 2 +
            (ordered[0][1] - ordered[2][1]) ** 2
        ),
        math.sqrt(
            (ordered[1][0] - ordered[3][0]) ** 2 +
            (ordered[1][1] - ordered[3][1]) ** 2
        ),
    )
    refined = False
    if residual > GEOMETRY_REFINE_TRIGGER_PX:
        pose = _refine_target_pose(
            pose,
            plane,
            normalized,
            camera.fx,
            camera.fy,
            GEOMETRY_REFINE_ITERATIONS,
        )
        residual = 0.0
        for plane_point, image_point in zip(plane, ordered):
            projected = camera.project(pose.camera_point(plane_point))
            dx = projected[0] - image_point[0]
            dy = projected[1] - image_point[1]
            residual = max(residual, math.sqrt(dx * dx + dy * dy))
        refined = True
    gate = min(
        GEOMETRY_MAX_REPROJECTION_CAP_PX,
        max(
            GEOMETRY_MAX_REPROJECTION_PX,
            GEOMETRY_REPROJECTION_FRACTION * diagonal,
        ),
    )
    if residual > gate:
        raise ValueError("pose residual {:.1f}px".format(residual))
    return pose, residual, ordered, refined


def _aim_solution(pose, laser_origin, laser_direction, target_mm):
    direction = _vec_unit(laser_direction)
    hit = pose.intersection(laser_origin, direction)
    hit_mm = pose.plane_point(hit)
    target_camera = pose.camera_point(target_mm)
    desired = _vec_sub(target_camera, laser_origin)
    if desired[2] <= 0.0 or direction[2] <= 0.0:
        raise ValueError("target or laser behind camera")
    yaw = (
        math.atan2(desired[0], desired[2]) -
        math.atan2(direction[0], direction[2])
    )
    pitch = (
        math.atan2(
            desired[1],
            math.sqrt(desired[0] * desired[0] + desired[2] * desired[2]),
        ) -
        math.atan2(
            direction[1],
            math.sqrt(
                direction[0] * direction[0] +
                direction[2] * direction[2]
            ),
        )
    )
    return {
        "yaw_deg": math.degrees(yaw),
        "pitch_deg": math.degrees(pitch),
        "hit_mm": hit_mm,
        "target_mm": target_mm,
        "error_mm": (target_mm[0] - hit_mm[0], target_mm[1] - hit_mm[1]),
        "depth_mm": target_camera[2],
    }


def _geometry_reject_code(message):
    message = str(message)
    if "pose residual" in message:
        return 4
    if (
        "inconsistent target dimensions" in message
        or "degenerate target pose" in message
        or "singular homography" in message
        or "ray parallel" in message
        or "intersection behind" in message
        or "point behind camera" in message
    ):
        return 5
    if "distortion" in message:
        return 6
    if "tilt" in message:
        return 7
    if "depth" in message:
        return 8
    if "angular error" in message or "target or laser behind" in message:
        return 9
    if "four corners" in message or "non-convex" in message:
        return 10
    return 3


def _load_calibration():
    camera_values = CAMERA_CALIBRATION
    origin = LASER_ORIGIN_MM
    direction = _vec_unit(LASER_DIRECTION)
    if USE_CAMERA_JSON_CALIBRATION or USE_LASER_JSON_CALIBRATION:
        try:
            with open(CALIBRATION_JSON_PATH, "r") as stream:
                data = json.load(stream)
            camera = data["camera"]
            laser = data["laser"]
            if USE_CAMERA_JSON_CALIBRATION:
                if camera.get("calibrated") is not True:
                    raise ValueError("camera not calibrated")
                if (camera.get("width"), camera.get("height")) != (
                    FRAME_WIDTH, FRAME_HEIGHT
                ):
                    raise ValueError("calibration resolution mismatch")
                camera_values = {
                    "fx": float(camera["fx"]),
                    "fy": float(camera["fy"]),
                    "cx": float(camera["cx"]),
                    "cy": float(camera["cy"]),
                    "distortion": tuple(
                        camera.get("distortion", (0,) * 5)
                    ),
                }
            if (
                USE_LASER_JSON_CALIBRATION
                and laser.get("calibrated") is True
            ):
                origin = tuple(float(value) for value in laser["origin_mm"])
                direction = _vec_unit(tuple(
                    float(value) for value in laser["direction"]
                ))
            print("[GEOM] calibration.json loaded")
        except Exception as exc:
            print("[GEOM] calibration fallback:", exc)
    camera = PoseCamera(
        FRAME_WIDTH,
        FRAME_HEIGHT,
        camera_values["fx"],
        camera_values["fy"],
        camera_values["cx"],
        camera_values["cy"],
        camera_values["distortion"],
    )
    print(
        "[GEOM] laser origin=%s direction=%s" % (origin, direction)
    )
    return camera, origin, direction


def _find_laser_spot(gray, corners, width_mm, height_mm, inset_mm):
    if np is None:
        return None, "spot_disabled"
    if hasattr(gray, "to_numpy_ref"):
        gray = gray.to_numpy_ref()
    height = gray.shape[0]
    width = gray.shape[1]
    to_image = _homography(
        ((0.0, 0.0), (width_mm, 0.0),
         (width_mm, height_mm), (0.0, height_mm)),
        corners,
    )
    x_min = inset_mm
    y_min = inset_mm
    x_max = width_mm - inset_mm
    y_max = height_mm - inset_mm
    quad = [
        _transform_homography(to_image, point)
        for point in (
            (x_min, y_min), (x_max, y_min),
            (x_max, y_max), (x_min, y_max),
        )
    ]
    x0 = max(0, int(min(point[0] for point in quad)) + 1)
    x1 = min(width, int(max(point[0] for point in quad)))
    y0 = max(0, int(min(point[1] for point in quad)) + 1)
    y1 = min(height, int(max(point[1] for point in quad)))
    if x1 - x0 < 8 or y1 - y0 < 8:
        return None, "spot_roi_small"

    roi = gray[y0:y1, x0:x1]
    peak_index = int(np.argmax(roi))
    peak_y = peak_index // (x1 - x0)
    peak_x = peak_index % (x1 - x0)
    peak = float(roi[peak_y, peak_x])
    if peak < SPOT_MIN_PEAK or peak - float(np.mean(roi)) < SPOT_MIN_CONTRAST:
        return None, "spot_dim"

    radius = SPOT_WINDOW_PX
    wy0 = max(0, peak_y - radius)
    wy1 = min(y1 - y0, peak_y + radius + 1)
    wx0 = max(0, peak_x - radius)
    wx1 = min(x1 - x0, peak_x + radius + 1)
    core = np.clip(
        np.array(
            roi[wy0:wy1, wx0:wx1],
            dtype=np.float32,
        ) - (peak - SPOT_CORE_DROP),
        0,
        255,
    )
    strongest = float(np.max(core))
    border = max(
        float(np.max(core[0, :])),
        float(np.max(core[-1, :])),
        float(np.max(core[:, 0])),
        float(np.max(core[:, -1])),
    )
    if strongest <= 0.0 or border > SPOT_BORDER_FRACTION * strongest:
        return None, "spot_too_large"
    rows = np.sum(core, axis=1)
    columns = np.sum(core, axis=0)
    total = float(np.sum(rows))
    if total <= 0.0:
        return None, "spot_empty"
    center_y = 0.0
    center_x = 0.0
    for index in range(len(rows)):
        center_y += index * float(rows[index])
    for index in range(len(columns)):
        center_x += index * float(columns[index])
    return (
        (
            x0 + wx0 + center_x / total,
            y0 + wy0 + center_y / total,
        ),
        "spot_ok",
    )


def _fit_line_tls(points):
    count = len(points)
    if count < 3:
        return None
    mean_x = sum(point[0] for point in points) / count
    mean_y = sum(point[1] for point in points) / count
    sxx = sum((point[0] - mean_x) ** 2 for point in points)
    syy = sum((point[1] - mean_y) ** 2 for point in points)
    sxy = sum(
        (point[0] - mean_x) * (point[1] - mean_y)
        for point in points
    )
    if sxx + syy < 1e-9:
        return None
    angle = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    return (mean_x, mean_y), (math.cos(angle), math.sin(angle))


def _intersect_fitted_lines(first, second):
    (px, py), (dx, dy) = first
    (qx, qy), (ex, ey) = second
    denominator = dx * ey - dy * ex
    if abs(denominator) < 1e-6:
        return None
    distance = ((qx - px) * ey - (qy - py) * ex) / denominator
    return (px + distance * dx, py + distance * dy)


def refine_geometry_corners(img, corners):
    """Refine only the final selected quad for pose input; tracker data stays unchanged."""
    if not GEOMETRY_CORNER_REFINE_ENABLED:
        return corners, False, 0.0
    try:
        ordered = _geometry_order_quad(corners)
    except Exception:
        return corners, False, 0.0

    fitted_sides = []
    for side_index in range(4):
        first = ordered[side_index]
        second = ordered[(side_index + 1) % 4]
        edge_x = second[0] - first[0]
        edge_y = second[1] - first[1]
        edge_length = math.sqrt(edge_x * edge_x + edge_y * edge_y)
        if edge_length < 12.0:
            return ordered, False, 0.0
        unit_x = edge_x / edge_length
        unit_y = edge_y / edge_length
        normal_x = -unit_y
        normal_y = unit_x
        samples = []
        for sample_index in range(1, GEOMETRY_CORNER_SAMPLES + 1):
            fraction = sample_index / (GEOMETRY_CORNER_SAMPLES + 1.0)
            base_x = first[0] + edge_x * fraction
            base_y = first[1] + edge_y * fraction
            best_offset = None
            best_gradient = -1.0
            for offset in range(
                -GEOMETRY_CORNER_SEARCH_PX,
                GEOMETRY_CORNER_SEARCH_PX + 1,
            ):
                center_x = int(round(base_x + normal_x * offset))
                center_y = int(round(base_y + normal_y * offset))
                before_x = int(round(
                    base_x + normal_x * (offset - 1.0)
                ))
                before_y = int(round(
                    base_y + normal_y * (offset - 1.0)
                ))
                after_x = int(round(
                    base_x + normal_x * (offset + 1.0)
                ))
                after_y = int(round(
                    base_y + normal_y * (offset + 1.0)
                ))
                if (
                    center_x < 0 or center_x >= FRAME_WIDTH or
                    center_y < 0 or center_y >= FRAME_HEIGHT or
                    before_x < 0 or before_x >= FRAME_WIDTH or
                    before_y < 0 or before_y >= FRAME_HEIGHT or
                    after_x < 0 or after_x >= FRAME_WIDTH or
                    after_y < 0 or after_y >= FRAME_HEIGHT
                ):
                    continue
                gradient = abs(
                    get_gray_value(img, after_x, after_y) -
                    get_gray_value(img, before_x, before_y)
                )
                if gradient > best_gradient:
                    best_gradient = gradient
                    best_offset = offset
            if best_offset is not None and best_gradient >= 8.0:
                samples.append((
                    base_x + normal_x * best_offset,
                    base_y + normal_y * best_offset,
                ))
        line = _fit_line_tls(samples)
        if line is None:
            line = (first, (unit_x, unit_y))
        fitted_sides.append(line)

    refined = []
    max_shift = 0.0
    for corner_index in range(4):
        corner = _intersect_fitted_lines(
            fitted_sides[(corner_index - 1) % 4],
            fitted_sides[corner_index],
        )
        original = ordered[corner_index]
        if corner is None:
            refined.append(original)
            continue
        shift = math.sqrt(
            (corner[0] - original[0]) ** 2 +
            (corner[1] - original[1]) ** 2
        )
        if shift > GEOMETRY_CORNER_MAX_SHIFT_PX:
            refined.append(original)
        else:
            refined.append(corner)
            max_shift = max(max_shift, shift)

    # Keep the same winding and reject a refinement that made the quad concave.
    signs = []
    for index in range(4):
        a = refined[index]
        b = refined[(index + 1) % 4]
        c = refined[(index + 2) % 4]
        signs.append(
            (b[0] - a[0]) * (c[1] - b[1]) -
            (b[1] - a[1]) * (c[0] - b[0])
        )
    if min(signs) <= 1e-5 or max(signs) <= 1e-5:
        return ordered, False, 0.0
    return tuple(refined), True, max_shift


class PlanarLaserAimer(object):
    def __init__(self):
        self.camera, self.origin, self.direction = _load_calibration()
        self.correction = (0.0, 0.0, 0.0)
        self.valid_frames = 0
        self.spot_status = "spot_wait"

    def corrected_direction(self):
        if self.correction == (0.0, 0.0, 0.0):
            return self.direction
        return _vec_unit(_vec_add(self.direction, self.correction))

    def correction_deg(self):
        if self.correction == (0.0, 0.0, 0.0):
            return 0.0, 0.0
        corrected = self.corrected_direction()
        return (
            math.degrees(
                math.atan2(corrected[0], corrected[2]) -
                math.atan2(self.direction[0], self.direction[2])
            ),
            math.degrees(
                math.atan2(corrected[1], corrected[2]) -
                math.atan2(self.direction[1], self.direction[2])
            ),
        )

    def _update_spot(self, gray, corners, pose, width_mm, height_mm, inset_mm):
        point, status = _find_laser_spot(
            gray, corners, width_mm, height_mm, inset_mm
        )
        if point is None:
            self.spot_status = status
            return
        normalized = self.camera.normalized(point)
        hit = pose.intersection(
            (0.0, 0.0, 0.0),
            (normalized[0], normalized[1], 1.0),
        )
        observed = _vec_unit(_vec_sub(hit, self.origin))
        current = self.corrected_direction()
        angle = math.degrees(math.acos(max(
            -1.0, min(1.0, _vec_dot(observed, current))
        )))
        if angle > SPOT_GATE_DEG:
            self.spot_status = "spot_gated"
            return
        wanted = _vec_sub(observed, self.direction)
        step = _vec_scale(
            _vec_sub(wanted, self.correction),
            SPOT_CORRECTION_ALPHA,
        )
        correction = _vec_add(self.correction, step)
        limit = math.radians(SPOT_MAX_CORRECTION_DEG)
        size = _vec_norm(correction)
        if size > limit:
            correction = _vec_scale(correction, limit / size)
        self.correction = correction
        self.spot_status = "spot_ok"

    def solve(self, target, gray):
        corners = target.get("corners")
        if corners is None or len(corners) != 4:
            return None
        is_inner = bool(target.get("inner"))
        if is_inner:
            width_mm = (
                TARGET_OUTER_WIDTH_M - 2.0 * BLACK_BORDER_WIDTH_M
            ) * 1000.0
            height_mm = (
                TARGET_OUTER_HEIGHT_M - 2.0 * BLACK_BORDER_WIDTH_M
            ) * 1000.0
            spot_inset_mm = SPOT_PAPER_MARGIN_MM
        else:
            width_mm = TARGET_OUTER_WIDTH_M * 1000.0
            height_mm = TARGET_OUTER_HEIGHT_M * 1000.0
            spot_inset_mm = BLACK_BORDER_WIDTH_M * 1000.0 + SPOT_PAPER_MARGIN_MM

        pose, residual, ordered, pose_refined = _estimate_target_pose(
            corners, self.camera, width_mm, height_mm
        )
        tilt = math.degrees(math.acos(max(
            -1.0, min(1.0, pose.normal[2])
        )))
        if tilt > GEOMETRY_MAX_TILT_DEG:
            raise ValueError("target tilt too large")
        target_mm = (width_mm * 0.5, height_mm * 0.5)
        solution = _aim_solution(
            pose, self.origin, self.corrected_direction(), target_mm
        )
        if not (
            GEOMETRY_MIN_DEPTH_MM <=
            solution["depth_mm"] <=
            GEOMETRY_MAX_DEPTH_MM
        ):
            raise ValueError("target depth out of range")
        if max(
            abs(solution["yaw_deg"]), abs(solution["pitch_deg"])
        ) > GEOMETRY_MAX_COMMAND_DEG:
            raise ValueError("angular error out of range")

        hit_camera = pose.intersection(
            self.origin, self.corrected_direction()
        )
        predicted_pixel = self.camera.project(hit_camera)
        self.valid_frames += 1
        if (
            SPOT_CORRECTION_ENABLED
            and self.valid_frames % SPOT_CORRECTION_EVERY == 0
        ):
            try:
                self._update_spot(
                    gray,
                    ordered,
                    pose,
                    width_mm,
                    height_mm,
                    spot_inset_mm,
                )
            except Exception as exc:
                self.spot_status = "spot_disabled:" + str(exc)

        correction_yaw, correction_pitch = self.correction_deg()
        solution.update({
            "laser_uv": predicted_pixel,
            "reprojection_px": residual,
            "pose_refined": pose_refined,
            "tilt_deg": tilt,
            "spot_status": self.spot_status,
            "correction_yaw_deg": correction_yaw,
            "correction_pitch_deg": correction_pitch,
        })
        return solution


# ---------------------------------------------------------------------------
# 检测器。
# ---------------------------------------------------------------------------


def detect_with_cv_lite(
    img,
    profile,
    diagnostics=None,
    detection_roi=None,
    min_white_ratio=ACQUIRE_MIN_WHITE_RATIO,
    reference=None,
    raw_rectangles=None,
):
    if not USE_CV_LITE or cv_lite is None:
        return []

    crop_start_ms = time.ticks_ms()
    detection_img, offset_x, offset_y = crop_for_detection(img, detection_roi)
    if detection_roi is None or detection_img is img:
        detection_width = FRAME_WIDTH
        detection_height = FRAME_HEIGHT
    else:
        detection_width = detection_roi[2]
        detection_height = detection_roi[3]

    try:
        image_shape = [detection_height, detection_width]
        image_np = detection_img.to_numpy_ref()
        if diagnostics is not None:
            diagnostics["crop_ms"] += float(
                time.ticks_diff(time.ticks_ms(), crop_start_ms)
            )
        cv_start_ms = time.ticks_ms()
        rectangles = cv_lite.grayscale_find_rectangles_with_corners(
            image_shape,
            image_np,
            profile["canny_low"],
            profile["canny_high"],
            APPROX_EPSILON,
            AREA_MIN_RATIO,
            MAX_ANGLE_COS,
            GAUSSIAN_BLUR,
        )
        if diagnostics is not None:
            diagnostics["cv_ms"] += float(
                time.ticks_diff(time.ticks_ms(), cv_start_ms)
            )
    except Exception:
        return []

    bump_diagnostic(diagnostics, "cv_lite_raw", len(rectangles))
    candidates = []
    cand_start_ms = time.ticks_ms()
    for rect in rectangles:
        if len(rect) < 12:
            continue

        corners = [
            (rect[4] + offset_x, rect[5] + offset_y),
            (rect[6] + offset_x, rect[7] + offset_y),
            (rect[8] + offset_x, rect[9] + offset_y),
            (rect[10] + offset_x, rect[11] + offset_y),
        ]
        if raw_rectangles is not None:
            raw_rectangles.append(tuple(corners))
        # 在开销较大的 build_candidate 之前先按中心距离筛选，
        # 仅用于已经大致知道目标位置的跟踪模式。
        if reference is not None:
            gate = TRACK_CANDIDATE_GATE_FACTOR * max(
                1.0, float(reference["w"]), float(reference["h"])
            )
            quad_cx = (corners[0][0] + corners[2][0]) * 0.5
            quad_cy = (corners[0][1] + corners[2][1]) * 0.5
            if (
                abs(quad_cx - reference["cx"]) > gate
                or abs(quad_cy - reference["cy"]) > gate
            ):
                bump_diagnostic(diagnostics, "candidate_gate")
                continue
        candidate = build_candidate(
            img,
            corners,
            profile,
            "cv_lite",
            diagnostics,
            min_white_ratio,
        )
        if candidate is not None:
            candidates.append(candidate)

    if diagnostics is not None:
        diagnostics["cand_ms"] += float(
            time.ticks_diff(time.ticks_ms(), cand_start_ms)
        )
    bump_diagnostic(diagnostics, "cv_lite_count", len(candidates))
    return candidates


def detect_candidates(
    img,
    profile,
    detection_mode,
    diagnostics=None,
    detection_roi=None,
    high_res_img=None,
    frame_count=0,
    reference=None,
    raw_rectangles=None,
):
    min_white_ratio = ACQUIRE_MIN_WHITE_RATIO
    if detection_mode in ("track", "recapture_roi"):
        min_white_ratio = TRACK_MIN_WHITE_RATIO

    cv_lite_candidates = detect_with_cv_lite(
        img,
        profile,
        diagnostics,
        detection_roi,
        min_white_ratio,
        reference if detection_mode == "track" else None,
        raw_rectangles,
    )

    candidates = cv_lite_candidates

    # 所有检测状态统一使用 cv_lite。
    if detection_mode in ("search", "recapture_full"):
        if (
            detection_mode == "recapture_full"
            and not candidates
            and LIGHT_FALLBACK_ENABLED
        ):
            bump_diagnostic(diagnostics, "light_fallback")
            fallback_profile = make_light_fallback_profile(profile)
            fallback_cv = detect_with_cv_lite(
                img,
                fallback_profile,
                diagnostics,
                detection_roi,
                min_white_ratio,
            )
            for candidate in fallback_cv:
                candidate["source"] = "cv_lite_fallback"
            candidates = fallback_cv

    if not candidates and detection_mode in ("search", "recapture_full"):
    # 先执行远距离检测：在当前 640x480 画面上使用较低 Canny 阈值
        # 成本较低，适合检测 2 到 3 像素宽的边框。
        if frame_count % HIGH_RES_SEARCH_EVERY_N_FRAMES == 0:
            bump_diagnostic(diagnostics, "long_range_search")
            long_profile = make_long_range_profile(profile)
            long_cv = detect_with_cv_lite(
                img,
                long_profile,
                diagnostics,
                detection_roi,
                min_white_ratio,
            )
            for candidate in long_cv:
                candidate["source"] = "cv_lite_long"
            candidates = long_cv
            if candidates:
                bump_diagnostic(diagnostics, "long_range_hit")

    if not candidates and detection_mode in ("search", "recapture_full"):
        candidates = detect_high_res_search(
            img,
            high_res_img,
            profile,
            diagnostics,
            frame_count,
        )

    if diagnostics is not None:
        diagnostics["candidate_count"] = len(candidates)
    return candidates


def detect_high_res_search(
    low_img,
    high_res_img,
    profile,
    diagnostics=None,
    frame_count=0,
):
    """在 640x480 通道上执行一次 cv_lite 全图检测。

    这是全图搜索的最后手段。160 厘米处 2 厘米黑框在 640x480 画面中
    约 4.8 像素宽；检测后会将角点换算回当前 640x480 检测坐标系再进行评分，
    因此跟踪器、距离模型和显示缩放均保持不变。
    """
    if not HIGH_RES_SEARCH_ENABLED or cv_lite is None or high_res_img is None:
        return []

    if frame_count % HIGH_RES_SEARCH_EVERY_N_FRAMES != 0:
        return []

    bump_diagnostic(diagnostics, "hi_res_search")
    scale_x = HIGH_RES_WIDTH / FRAME_WIDTH
    scale_y = HIGH_RES_HEIGHT / FRAME_HEIGHT

    try:
        high_res_gray = high_res_img.to_grayscale()
        rectangles = cv_lite.grayscale_find_rectangles_with_corners(
            [HIGH_RES_HEIGHT, HIGH_RES_WIDTH],
            high_res_gray.to_numpy_ref(),
            profile["canny_low"],
            profile["canny_high"],
            APPROX_EPSILON,
            AREA_MIN_RATIO,
            MAX_ANGLE_COS,
            GAUSSIAN_BLUR,
        )
    except Exception:
        bump_diagnostic(diagnostics, "hi_res_error")
        return []

    bump_diagnostic(diagnostics, "hi_res_raw", len(rectangles))
    candidates = []
    for rect in rectangles:
        if len(rect) < 12:
            continue

        corners = []
        for index in range(4):
            corner_index = 4 + index * 2
            corners.append(
                (
                    rect[corner_index] / scale_x,
                    rect[corner_index + 1] / scale_y,
                )
            )

        # 此调用不记录诊断信息，使低分辨率筛选计数
        # 在运行日志中保持单一且明确的含义。
        candidate = build_candidate(
            low_img,
            corners,
            profile,
            "high_res_search",
            None,
            ACQUIRE_MIN_WHITE_RATIO,
        )
        if candidate is not None:
            candidates.append(candidate)

    if candidates:
        bump_diagnostic(diagnostics, "hi_res_hit")

    return candidates


def should_refine_high_res(candidate, detection_mode, frame_count):
    if not HIGH_RES_VERIFY_ENABLED:
        return False
    if detection_mode == "track" and frame_count % HIGH_RES_VERIFY_EVERY_N_FRAMES != 0:
        return False

    return (
        candidate["w"] < HIGH_RES_VERIFY_MIN_WIDTH
        or candidate["distance_m"] > HIGH_RES_VERIFY_DISTANCE_M
        or candidate["dark_border"] < MIN_DARK_BORDER_RATIO
    )


def refine_candidates_with_high_res(
    low_img,
    high_res_img,
    candidates,
    profile,
    diagnostics,
    detection_mode,
    frame_count,
):
    if (
        not HIGH_RES_VERIFY_ENABLED
        or high_res_img is None
        or cv_lite is None
        or not candidates
    ):
        return candidates

    scale_x = HIGH_RES_WIDTH / FRAME_WIDTH
    scale_y = HIGH_RES_HEIGHT / FRAME_HEIGHT
    min_white_ratio = ACQUIRE_MIN_WHITE_RATIO
    if detection_mode in ("track", "recapture_roi"):
        min_white_ratio = TRACK_MIN_WHITE_RATIO

    refined = []
    for candidate in candidates:
        if not should_refine_high_res(candidate, detection_mode, frame_count):
            refined.append(candidate)
            continue

        bump_diagnostic(diagnostics, "high_res_refine")
        margin = max(12, int(candidate["w"] * 0.35 * scale_x))
        roi_x = max(0, int(candidate["x"] * scale_x) - margin)
        roi_y = max(0, int(candidate["y"] * scale_y) - margin)
        roi_x1 = min(
            HIGH_RES_WIDTH,
            int((candidate["x"] + candidate["w"]) * scale_x) + margin,
        )
        roi_y1 = min(
            HIGH_RES_HEIGHT,
            int((candidate["y"] + candidate["h"]) * scale_y) + margin,
        )
        roi_w = roi_x1 - roi_x
        roi_h = roi_y1 - roi_y
        if roi_w < 32 or roi_h < 32:
            refined.append(candidate)
            continue

        try:
            roi_img = high_res_img.copy(roi=(roi_x, roi_y, roi_w, roi_h))
            roi_gray = roi_img.to_grayscale()
            rectangles = cv_lite.grayscale_find_rectangles_with_corners(
                [roi_h, roi_w],
                roi_gray.to_numpy_ref(),
                profile["canny_low"],
                profile["canny_high"],
                APPROX_EPSILON,
                AREA_MIN_RATIO,
                MAX_ANGLE_COS,
                GAUSSIAN_BLUR,
            )
        except Exception:
            refined.append(candidate)
            continue

        best = None
        for rect in rectangles:
            if len(rect) < 12:
                continue

            corners = []
            for index in range(4):
                corner_index = 4 + index * 2
                corner_x = (roi_x + rect[corner_index]) / scale_x
                corner_y = (roi_y + rect[corner_index + 1]) / scale_y
                corners.append((corner_x, corner_y))

            high_res_candidate = build_candidate(
                low_img,
                corners,
                profile,
                "high_res",
                diagnostics,
                min_white_ratio,
            )
            if high_res_candidate is None:
                continue

            center_error = center_distance(high_res_candidate, candidate)
            if center_error > max(20.0, candidate["w"] * 0.60):
                continue

            area_ratio = high_res_candidate["area"] / max(1.0, candidate["area"])
            if area_ratio < 0.45 or area_ratio > 2.20:
                continue

            if best is None or high_res_candidate["score"] > best["score"]:
                best = high_res_candidate

        if best is not None:
            refined.append(best)
            bump_diagnostic(diagnostics, "high_res_accept")
        else:
            refined.append(candidate)

    return refined


# ---------------------------------------------------------------------------
# 目标状态机。
# ---------------------------------------------------------------------------


class TargetTracker:
    def __init__(self):
        self.reset()

    def reset(self):
        self.state = "SEARCH"
        self.last = None
        self.output = None
        self.confirm_hits = 0
        self.confirm_age = 0
        self.lost_count = 0
        self.recapture_count = 0
        self.layer_conflict_count = 0
        self.vx = 0.0
        self.vy = 0.0
        self.last_seen_ms = None
        self.selected_candidate = None

    def detection_mode(self):
        if self.state == "TRACK":
            return "track"
        if self.state == "CONFIRM":
            return "confirm"
        if self.state == "RECAPTURE_ROI":
            return "recapture_roi"
        if self.state == "RECAPTURE_FULL":
            return "recapture_full"
        return "search"

    def begin_recapture(self):
        self.state = "RECAPTURE_ROI"
        self.confirm_hits = 0
        self.confirm_age = 0
        self.lost_count = 0
        self.recapture_count = 0
        self.layer_conflict_count = 0

    def detection_roi(self):
        if self.state == "TRACK" and self.last is not None:
            margin_factor = TRACK_DETECTION_ROI_MARGIN_FACTOR
            if self.last.get("distance_m", NEAR_TRACK_DISTANCE_M + 1.0) <= \
                    NEAR_TRACK_DISTANCE_M:
                margin_factor = NEAR_TRACK_DETECTION_ROI_MARGIN_FACTOR
            return make_tracking_roi(
                self.last, margin_factor
            )
        if self.state == "RECAPTURE_ROI" and self.last is not None:
            return make_tracking_roi(
                self.last, RECAPTURE_ROI_MARGIN_FACTOR
            )
        return None

    def copy_candidate(self, candidate):
        copied = {}
        for key in candidate:
            copied[key] = candidate[key]
        return copied

    def predicted_reference(self, now_ms):
        if self.last is None:
            return None

        reference = self.copy_candidate(self.last)
        if self.state == "TRACK" and self.last_seen_ms is not None:
            elapsed_s = time.ticks_diff(now_ms, self.last_seen_ms) / 1000.0
            if 0.0 < elapsed_s < 0.5:
                reference["cx"] += self.vx * elapsed_s
                reference["cy"] += self.vy * elapsed_s
        return reference

    def predict_reference(self, now_ms):
        """根据速度外推上一个目标框，不依赖当前跟踪状态。

        用于在短暂丢靶期间继续向云台提供目标。跟踪器可能停留在确认或
        重捕获状态数帧，若这段时间冻结目标，云台会产生滞后，并在重新
        捕获时突然跳动。丢靶时间超过 PREDICT_SEND_MS 后返回 None。
        """
        if self.last is None or self.last_seen_ms is None:
            return None

        elapsed_s = time.ticks_diff(now_ms, self.last_seen_ms) / 1000.0
        if elapsed_s <= 0.0 or elapsed_s > PREDICT_SEND_MS / 1000.0:
            return None

        predicted = self.copy_candidate(self.last)
        predicted["cx"] += self.vx * elapsed_s
        predicted["cy"] += self.vy * elapsed_s
        predicted["x"] = int(predicted["cx"] - predicted["w"] * 0.5)
        predicted["y"] = int(predicted["cy"] - predicted["h"] * 0.5)
        predicted["predicted"] = True
        return predicted

    def fast_confirm_ok(self, candidate):
        """候选落在预测框上，直接接受，无需继续投票。"""
        reference = self.predicted_reference(time.ticks_ms())
        if reference is None:
            return False

        if (
            center_distance(candidate, reference)
            > reference["w"] * CONFIRM_FAST_CENTER_FACTOR
        ):
            return False

        area_ratio = candidate["area"] / max(1.0, reference["area"])
        return CONFIRM_FAST_AREA_RATIO_MIN <= area_ratio <= CONFIRM_FAST_AREA_RATIO_MAX

    def is_similar(self, candidate, reference=None):
        if reference is None:
            reference = self.last
        if reference is None:
            return False

        if (
            reference["layer"] != "unknown"
            and candidate["layer"] != "unknown"
            and candidate["layer"] != reference["layer"]
        ):
            return False

        distance = center_distance(candidate, reference)
        area_min, area_max, axis_min, axis_max, center_factor = \
            tracking_gate_limits(reference)
        max_distance = max(24.0, reference["w"] * center_factor)
        area_ratio = candidate["area"] / reference["area"]
        width_ratio = candidate["w"] / max(1.0, reference["w"])
        height_ratio = candidate["h"] / max(1.0, reference["h"])

        return (
            distance <= max_distance
            and area_min <= area_ratio <= area_max
            and axis_min <= width_ratio <= axis_max
            and axis_min <= height_ratio <= axis_max
        )

    def is_confirmation_candidate(self, candidate):
        if self.last is None:
            return True

        reference = self.last
        center_limit = max(30.0, reference["w"] * 1.00)
        if center_distance(candidate, reference) > center_limit:
            return False

        area_ratio = candidate["area"] / max(1.0, reference["area"])
        if (
            CONFIRM_OUTER_AREA_RATIO_MIN
            <= area_ratio
            <= CONFIRM_OUTER_AREA_RATIO_MAX
        ):
            candidate["layer"] = "outer"
            self.last = candidate
            return True

        if 0.65 <= area_ratio < 0.95:
            if candidate["layer"] == "unknown":
                candidate["layer"] = "inner"
            return True

        if 0.95 <= area_ratio < CONFIRM_OUTER_AREA_RATIO_MIN:
            self.last = candidate
            return True

        return False

    def update_velocity(self, candidate, now_ms):
        if self.last is None or self.last_seen_ms is None:
            self.vx = 0.0
            self.vy = 0.0
            return

        elapsed_s = time.ticks_diff(now_ms, self.last_seen_ms) / 1000.0
        if elapsed_s <= 0.0 or elapsed_s > 0.5:
            return

        measured_vx = (candidate["cx"] - self.last["cx"]) / elapsed_s
        measured_vy = (candidate["cy"] - self.last["cy"]) / elapsed_s
        self.vx = self.vx * 0.35 + measured_vx * 0.65
        self.vy = self.vy * 0.35 + measured_vy * 0.65
        self.vx = max(-MAX_TRACK_VELOCITY_PX_PER_S,
                      min(MAX_TRACK_VELOCITY_PX_PER_S, self.vx))
        self.vy = max(-MAX_TRACK_VELOCITY_PX_PER_S,
                      min(MAX_TRACK_VELOCITY_PX_PER_S, self.vy))

    def smooth_candidate(self, candidate):
        if self.output is None:
            self.output = self.copy_candidate(candidate)
            return self.output

        previous = self.output
        smoothed = self.copy_candidate(candidate)
        smoothed["cx"] = previous["cx"] + SMOOTH_CENTER_ALPHA * (
            candidate["cx"] - previous["cx"]
        )
        smoothed["cy"] = previous["cy"] + SMOOTH_CENTER_ALPHA * (
            candidate["cy"] - previous["cy"]
        )
        smoothed["w"] = previous["w"] + SMOOTH_SIZE_ALPHA * (
            candidate["w"] - previous["w"]
        )
        smoothed["h"] = previous["h"] + SMOOTH_SIZE_ALPHA * (
            candidate["h"] - previous["h"]
        )
        smoothed["x"] = int(smoothed["cx"] - smoothed["w"] * 0.5)
        smoothed["y"] = int(smoothed["cy"] - smoothed["h"] * 0.5)
        smoothed["area"] = smoothed["w"] * smoothed["h"] * smoothed["rect"]

        metrics = calculate_distance_metrics(
            smoothed["w"], smoothed["h"], smoothed["inner"]
        )
        smoothed["distance_m"] = metrics["distance_m"]
        smoothed["distance_width_m"] = metrics["distance_width_m"]
        smoothed["distance_height_m"] = metrics["distance_height_m"]
        smoothed["distance_consistency"] = metrics["distance_consistency"]
        self.output = smoothed
        return smoothed

    def accept_candidate(self, candidate, now_ms):
        self.update_velocity(candidate, now_ms)
        self.last = candidate
        self.last_seen_ms = now_ms
        return self.smooth_candidate(candidate)

    def update(self, candidates, diagnostics=None):
        now_ms = time.ticks_ms()
        detection_mode = self.detection_mode()
        if diagnostics is not None:
            diagnostics["selected_mode"] = detection_mode
            diagnostics["target_held"] = 0

        reference = self.predicted_reference(now_ms)
        candidate = select_best_candidate(
            candidates,
            self.state,
            reference,
            diagnostics,
            self.detection_roi(),
        )
        self.selected_candidate = candidate

        if self.state == "TRACK":
            if candidate is not None and self.is_similar(candidate, reference):
                self.lost_count = 0
                self.layer_conflict_count = 0
                return self.accept_candidate(candidate, now_ms)

            if candidate is not None and candidate.get("alternate_layer"):
                self.layer_conflict_count += 1
                if self.layer_conflict_count >= LAYER_SWITCH_VOTES:
                    previous = reference if reference is not None else self.last
                    if previous is not None:
                        area_ratio = candidate["area"] / max(
                            1.0, previous["area"]
                        )
                        if area_ratio > 1.08:
                            candidate["layer"] = "outer"
                        elif area_ratio < 0.92:
                            candidate["layer"] = "inner"
                    self.layer_conflict_count = 0
                    self.output = None
                    return self.accept_candidate(candidate, now_ms)

                if self.output is not None:
                    if diagnostics is not None:
                        diagnostics["target_held"] = 1
                    return self.output
                if self.last is not None:
                    if diagnostics is not None:
                        diagnostics["target_held"] = 1
                    return self.last

            self.layer_conflict_count = 0
            self.lost_count += 1
            if self.lost_count > HOLD_LOST_FRAMES:
                self.begin_recapture()
                return None

            if self.output is not None:
                if diagnostics is not None:
                    diagnostics["target_held"] = 1
                return self.output
            if self.last is not None:
                if diagnostics is not None:
                    diagnostics["target_held"] = 1
                return self.last
            return None

        if self.state == "SEARCH":
            if candidate is None:
                return None

            self.state = "CONFIRM"
            self.confirm_hits = 1
            self.confirm_age = 1
            self.last = candidate
            self.last_seen_ms = now_ms
            self.output = None
            return None

        if self.state == "CONFIRM":
            self.confirm_age += 1
            if candidate is not None:
                if self.is_confirmation_candidate(candidate):
                    self.confirm_hits += 1
                else:
                    self.confirm_hits = 1
                    self.last = candidate
                self.last_seen_ms = now_ms
                if self.fast_confirm_ok(candidate):
                    # 正好落在预测框上，说明是同一目标返回，
                    # 跳过后续投票。
                    self.confirm_hits = CONFIRM_HITS_REQUIRED

            if self.confirm_hits >= CONFIRM_HITS_REQUIRED:
                self.state = "TRACK"
                self.lost_count = 0
                self.layer_conflict_count = 0
                self.vx = 0.0
                self.vy = 0.0
                self.output = None
                if self.last is not None:
                    return self.accept_candidate(self.last, now_ms)
                return None

            if self.confirm_age >= CONFIRM_WINDOW_FRAMES:
                self.reset()
            return None

        if self.state in ("RECAPTURE_ROI", "RECAPTURE_FULL"):
            if candidate is not None:
                self.state = "CONFIRM"
                self.confirm_hits = 1
                self.confirm_age = 1
                self.last = candidate
                self.last_seen_ms = now_ms
                self.recapture_count = 0
                self.output = None
                return None

            self.recapture_count += 1
            if (
                self.state == "RECAPTURE_ROI"
                and self.recapture_count > RECAPTURE_ROI_FRAMES
            ):
                self.state = "RECAPTURE_FULL"
                self.recapture_count = 0
            elif (
                self.state == "RECAPTURE_FULL"
                and self.recapture_count > RECAPTURE_FULL_FRAMES
            ):
                if diagnostics is not None:
                    diagnostics["lost_reason"] = "recapture_timeout"
                self.reset()
            return None

        self.reset()
        return None


# ---------------------------------------------------------------------------
# 串口协议。
# ---------------------------------------------------------------------------


def configure_vision_uart():
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, FPIOA.UART1_TXD, ie=0, oe=1)
    fpioa.set_function(UART_RX_PIN, FPIOA.UART1_RXD, ie=1, oe=0)
    return UART(UART.UART1, UART_BAUDRATE)


def build_vision_frame(
    yaw_deg,
    pitch_deg,
    target,
    laser_point,
    target_vx=0.0,
    target_vy=0.0,
    image_age_ms=0.0,
    geometry_state=0,
    reprojection_px=0.0,
    pose_refined=0,
    target_w=0.0,
    target_h=0.0,
    target_layer=0,
):
    # 末尾：1=几何成功；2=无新鲜靶框；3~11=几何拒绝；12=固定激光点。
    # K230 只在状态 1 时发送本帧，其余状态发送视觉无效帧。
    return "{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.1f},{:.1f},{:.2f},{},{:.1f},{},{:.1f},{:.1f},{:.2f}\n".format(
        yaw_deg,
        pitch_deg,
        target["cx"],
        target["cy"],
        laser_point["x"],
        laser_point["y"],
        laser_point.get("distance_m", target["distance_m"]),
        target_vx,
        target_vy,
        image_age_ms / 1000.0,
        geometry_state,
        reprojection_px,
        1 if pose_refined else 0,
        target_w,
        target_h,
        target_layer / 100.0,
    )


def build_vision_lost_frame():
    return "{},{},0,0,0,0,0\n".format(
        VISION_LOST_VALUE, VISION_LOST_VALUE
    )


def calculate_fixed_laser_point(target):
    """固定激光落点；只从目标复制距离用于现有遥测协议。"""
    if target is None:
        return None
    return {
        "x": FIXED_LASER_POINT_X_PX,
        "y": FIXED_LASER_POINT_Y_PX,
        "distance_m": target["distance_m"],
    }


def calculate_gimbal_error(target, laser_point):
    """固定像素点到靶心的角度误差；geometry_state=12 表示直接角度。"""
    if target is None or laser_point is None:
        return None
    return {
        "yaw_deg": (
            target["cx"] - laser_point["x"]
        ) * GIMBAL_ANGLE_PER_PIXEL_DEG,
        "pitch_deg": (
            target["cy"] - laser_point["y"]
        ) * GIMBAL_ANGLE_PER_PIXEL_DEG,
    }


def update_pitch_trim(trim_deg, gimbal_error, tracker):
    """缓慢学习云台自身无法消除的俯仰残差。

    仅在目标大致静止时学习，避免把跟踪滞后误认为固定偏移。
    """
    if not PITCH_TRIM_ENABLED or gimbal_error is None:
        return trim_deg

    speed_px_per_s = abs(tracker.vx) + abs(tracker.vy)
    if speed_px_per_s > PITCH_TRIM_STATIC_PX_PER_S:
        return trim_deg

    trim_deg += PITCH_TRIM_GAIN * gimbal_error["pitch_deg"]
    if trim_deg > PITCH_TRIM_LIMIT_DEG:
        trim_deg = PITCH_TRIM_LIMIT_DEG
    elif trim_deg < -PITCH_TRIM_LIMIT_DEG:
        trim_deg = -PITCH_TRIM_LIMIT_DEG
    return trim_deg


def apply_pitch_trim(gimbal_error, trim_deg):
    """返回发送给 H723 的误差，并叠加学习到的俯仰补偿。"""
    if gimbal_error is None or trim_deg == 0.0:
        return gimbal_error

    return {
        "yaw_deg": gimbal_error["yaw_deg"],
        "pitch_deg": gimbal_error["pitch_deg"] + trim_deg,
    }


def send_vision_frame(
    uart,
    target,
    laser_point,
    gimbal_error,
    diagnostics,
    last_lost_send_ms,
    target_vx=0.0,
    target_vy=0.0,
    image_age_ms=0.0,
    geometry_state=0,
    reprojection_px=0.0,
    pose_refined=0,
    target_w=0.0,
    target_h=0.0,
    target_layer=0,
):
    """发送一帧串口数据，并返回更新后的丢靶帧时间戳。

    正式运行和抓图测试共用同一路径，包括目标丢失时的 404 帧。
    """
    if uart is None:
        return last_lost_send_ms

    # 没有新鲜三维解算时绝不使用固定点、方向投影点或上帧旧点。
    # 404 会让 H723 进入已有的安全自稳路径。
    if target is None or gimbal_error is None:
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_lost_send_ms) < (
            VISION_LOST_SEND_INTERVAL_MS
        ):
            return last_lost_send_ms
        frame = build_vision_lost_frame()
        uart.write(frame.encode())
        diagnostics["tx_frame"] = frame.strip()
        return now_ms

    frame = build_vision_frame(
        gimbal_error["yaw_deg"],
        gimbal_error["pitch_deg"],
        target,
        laser_point,
        target_vx,
        target_vy,
        image_age_ms,
        geometry_state,
        reprojection_px,
        pose_refined,
        target_w,
        target_h,
        target_layer,
    )
    uart.write(frame.encode())
    diagnostics["tx_frame"] = frame.strip()

    return last_lost_send_ms


# ---------------------------------------------------------------------------
# 测试模式辅助函数。
# ---------------------------------------------------------------------------


def ensure_dir(path):
    try:
        uos.stat(path)
        return
    except OSError:
        pass

    parent = path[: path.rindex("/")]
    if parent and parent != path:
        ensure_dir(parent)

    try:
        uos.mkdir(path)
    except OSError:
        pass


def append_capture_record(csv_path, record):
    file_exists = True
    try:
        uos.stat(csv_path)
    except OSError:
        file_exists = False

    try:
        with open(csv_path, "a") as output:
            if not file_exists:
                output.write(
                    "index,file,mean,detected,tracked,x,y,w,h,cx,cy,white,"
                    "dark_border,area,distance_cm,laser_x,laser_y,fps,"
                    "gimbal_yaw_deg,"
                    "gimbal_pitch_deg,candidate_source,"
                    "candidate_count,best_distance_cm,distance_reject,size_reject,"
                    "center_reject,roi_reject,lost_reason,selected_mode,layer\n"
                )
            output.write(record)
    except Exception as error:
        print("CSV write failed:", error)


def new_frame_diagnostics(selected_mode):
    return {
        "selected_mode": selected_mode,
        "candidate_source": "none",
        "candidate_count": 0,
        "cv_lite_raw": 0,
        "cv_lite_count": 0,
        "best_distance_cm": 0.0,
        "distance_reject": 0,
        "size_reject": 0,
        "center_reject": 0,
        "roi_reject": 0,
        "area_reject": 0,
        "aspect_reject": 0,
        "angle_reject": 0,
        "rectangularity_reject": 0,
        "layer_reject": 0,
        "alternate_count": 0,
        "dark_border_low": 0,
        "high_res_refine": 0,
        "high_res_accept": 0,
        "absolute_reject": 0,
        "white_reject": 0,
        "candidate_gate": 0,
        "inner_hold_reject": 0,
        "light_fallback": 0,
        "lost_reason": "-",
        "target_held": 0,
        "tx_frame": "-",
        "detect_ms": 0.0,
        "display_ms": 0.0,
        "image_age_ms": 0.0,
        # 用于定位耗时来源的分项计时，反映每帧时间实际
        # 花在哪里；对应调试行中的 b= 字段。
        "crop_ms": 0.0,
        "cv_ms": 0.0,
        "cand_ms": 0.0,
        "profile_ms": 0.0,
        "log_ms": 0.0,
        "flush_ms": 0.0,
        "tx_ms": 0.0,
        "hi_res_search": 0,
        "hi_res_raw": 0,
        "hi_res_hit": 0,
        "hi_res_error": 0,
        "long_range_search": 0,
        "long_range_hit": 0,
        "white_low": 0,
        "pitch_trim_deg": 0.0,
        "geometry_valid": 0,
        "geometry_state": 0,
        "geometry_reject": "-",
        "reprojection_px": 0.0,
        "pose_refined": 0,
        "corner_refined": 0,
        "corner_shift": 0.0,
        "spot_status": "-",
        "spot_correction": (0.0, 0.0),
    }


def format_frame_log(
    state,
    target,
    laser_point,
    light_profile,
    mean_brightness,
    fps,
    diagnostics,
    frame_count,
    win_fps,
    period_ms,
    snap_ms,
    track_ms,
    gc_ms,
    pace_ms,
    tracker,
    sent_error,
):
    common = (
        "frame={} state={} mode={} fps={:.1f} winfps={:.1f} "
        "per={:.1f} snap={:.1f} det={:.1f} trk={:.1f} tx={:.1f} "
        "gc={:.1f} pace={:.1f} img_age={:.1f}"
    ).format(
        frame_count,
        state,
        diagnostics["selected_mode"],
        fps,
        win_fps,
        period_ms,
        snap_ms,
        diagnostics["detect_ms"],
        track_ms,
        diagnostics["tx_ms"],
        gc_ms,
        pace_ms,
        diagnostics["image_age_ms"],
    )
    common += (
        " source={} candidate_count={} cv_lite={}/{} best_distance_cm={:.1f} "
        "distance_reject={} size_reject={} center_reject={} roi_reject={} "
        "area_reject={} aspect_reject={} angle_reject={} rect_reject={} "
        "layer_reject={} inner_hold={} alternate_count={} dark_border_low={} "
        "high_res_refine={} high_res_accept={} "
        "absolute_reject={} white_reject={} light_fallback={} "
        "lost_reason={} tx={} profile={} mean={} disp_ms={:.1f} "
        "hi_res_search={} hi_res_raw={} "
        "hi_res_hit={} hi_res_error={} long_range={}/{} white_low={} trim={:+.1f} "
        "geom={} reproj={:.1f} refine={} corner=({},{:.2f}) "
        "spot={} corr=({:+.2f},{:+.2f}) "
        "geom_reject={}"
    ).format(
        diagnostics["candidate_source"],
        diagnostics["candidate_count"],
        diagnostics["cv_lite_count"],
        diagnostics["cv_lite_raw"],
        diagnostics["best_distance_cm"],
        diagnostics["distance_reject"],
        diagnostics["size_reject"],
        diagnostics["center_reject"],
        diagnostics["roi_reject"],
        diagnostics["area_reject"],
        diagnostics["aspect_reject"],
        diagnostics["angle_reject"],
        diagnostics["rectangularity_reject"],
        diagnostics["layer_reject"],
        diagnostics["inner_hold_reject"],
        diagnostics["alternate_count"],
        diagnostics["dark_border_low"],
        diagnostics["high_res_refine"],
        diagnostics["high_res_accept"],
        diagnostics["absolute_reject"],
        diagnostics["white_reject"],
        diagnostics["light_fallback"],
        diagnostics["lost_reason"],
        diagnostics["tx_frame"],
        light_profile["level"],
        mean_brightness,
        diagnostics["display_ms"],
        diagnostics["hi_res_search"],
        diagnostics["hi_res_raw"],
        diagnostics["hi_res_hit"],
        diagnostics["hi_res_error"],
        diagnostics["long_range_search"],
        diagnostics["long_range_hit"],
        diagnostics["white_low"],
        diagnostics["pitch_trim_deg"],
        diagnostics["geometry_state"],
        diagnostics["reprojection_px"],
        diagnostics["pose_refined"],
        diagnostics["corner_refined"],
        diagnostics["corner_shift"],
        diagnostics["spot_status"],
        diagnostics["spot_correction"][0],
        diagnostics["spot_correction"][1],
        diagnostics["geometry_reject"],
    )

    if target is None:
        return common + " target=none held={} gimbal=({}, {})".format(
            diagnostics["target_held"],
            VISION_LOST_VALUE,
            VISION_LOST_VALUE,
        )

    gimbal_error = sent_error
    velocity_text = "-"
    if tracker is not None:
        velocity_text = "({:+.1f},{:+.1f})".format(tracker.vx, tracker.vy)
    predicted_text = "1" if target.get("predicted") else "0"

    return common + (
        " target_held={} predicted={} layer={} bbox=({},{},{},{}) "
        "center=({:.2f},{:.2f}) vel_px_s={} distance_cm={:.1f} "
        "laser=({:.2f},{:.2f}) gimbal=({:+.2f},{:+.2f})"
    ).format(
        diagnostics["target_held"],
        predicted_text,
        target["layer"],
        target["x"],
        target["y"],
        target["w"],
        target["h"],
        target["cx"],
        target["cy"],
        velocity_text,
        target["distance_m"] * 100.0,
        laser_point["x"] if laser_point is not None else VISION_LOST_VALUE,
        laser_point["y"] if laser_point is not None else VISION_LOST_VALUE,
        gimbal_error["yaw_deg"]
        if gimbal_error is not None
        else VISION_LOST_VALUE,
        gimbal_error["pitch_deg"]
        if gimbal_error is not None
        else VISION_LOST_VALUE,
    )


class RunLogger:
    def __init__(self, mode, case_name):
        self.file = None
        # 运行期不主动 flush，避免 eMMC 刷新阻塞主循环。
        self.write_ms = 0.0
        self.flush_ms = 0.0

        if not RUN_LOG_ENABLED:
            return

        try:
            ensure_dir(RUN_LOG_PATH[: RUN_LOG_PATH.rindex("/")])
            self.file = open(RUN_LOG_PATH, "w")
            self.file.write(
                "boot version={} mode={} case={} frame={}x{} target={}x{}cm "
                "border={}cm distance_gate={:.1f}-{:.1f}m "
                "target_fps={} log_every_n={} "
                "uart=UART1 tx={} rx={} baud={} "
                "camera=({:.1f},{:.1f},{:.1f},{:.1f}) "
                "laser_origin=({:.1f},{:.1f},{:.1f}) "
                "laser_dir=({:.3f},{:.3f},{:.3f}) "
                "pitch_trim_gain={:.3f} pitch_trim_limit={:.1f}\n".format(
                    VISION_VERSION,
                    mode,
                    case_name if case_name is not None else "-",
                    FRAME_WIDTH,
                    FRAME_HEIGHT,
                    int(TARGET_OUTER_WIDTH_M * 100),
                    int(TARGET_OUTER_HEIGHT_M * 100),
                    int(BLACK_BORDER_WIDTH_M * 100),
                    MIN_VALID_DISTANCE_M,
                    MAX_VALID_DISTANCE_M,
                    TARGET_PROCESS_FPS,
                    LOG_EVERY_N_FRAMES,
                    UART_TX_PIN,
                    UART_RX_PIN,
                    UART_BAUDRATE,
                    CAMERA_CALIBRATION["fx"],
                    CAMERA_CALIBRATION["fy"],
                    CAMERA_CALIBRATION["cx"],
                    CAMERA_CALIBRATION["cy"],
                    LASER_ORIGIN_MM[0],
                    LASER_ORIGIN_MM[1],
                    LASER_ORIGIN_MM[2],
                    LASER_DIRECTION[0],
                    LASER_DIRECTION[1],
                    LASER_DIRECTION[2],
                    PITCH_TRIM_GAIN if PITCH_TRIM_ENABLED else 0.0,
                    PITCH_TRIM_LIMIT_DEG,
                )
            )
        except Exception as error:
            self.file = None
            print("run log open failed:", error)

    def log(self, message):
        if self.file is None:
            return

        try:
            write_start_ms = time.ticks_ms()
            self.file.write("[{}] {}\n".format(time.ticks_ms(), message))
            self.write_ms = float(time.ticks_diff(time.ticks_ms(), write_start_ms))
            self.flush_ms = 0.0
        except Exception:
            pass

    def trace(self, message):
        """Synchronous startup trace; survives an IDE interrupt before frame 1."""
        if self.file is None:
            return

        try:
            self.file.write(
                "[{}] STAGE {}\n".format(time.ticks_ms(), message)
            )
            self.file.flush()
        except Exception:
            pass

    def close(self):
        if self.file is None:
            return

        try:
            self.file.flush()
            self.file.close()
        except Exception:
            pass
        self.file = None


# ---------------------------------------------------------------------------
# 绘图与诊断。
# ---------------------------------------------------------------------------


def draw_candidate(img, candidate, valid, scale=1.0, color=None):
    if candidate is None:
        return

    if color is None:
        color = (0, 255, 0) if valid else (255, 0, 0)
    corners = candidate.get("corners")
    if corners is not None and len(corners) == 4:
        center_x, center_y = perspective_center_of_corners(corners)
    else:
        center_x = candidate["cx"]
        center_y = candidate["cy"]

    if corners is not None and len(corners) == 4 and hasattr(img, "draw_line"):
        # 所见即所得轮廓：当前帧的原始角点包含透视信息，
        # 因此绘制边框跟随黑色框的透视形状，而不是只
        # 在四点接触目标的轴对齐外接框。
        points = []
        for point in corners:
            points.append(
                (int(point[0] * scale + 0.5), int(point[1] * scale + 0.5))
            )
        for index in range(4):
            x0, y0 = points[index]
            x1, y1 = points[(index + 1) % 4]
            img.draw_line(x0, y0, x1, y1, color=color, thickness=2)
    else:
        img.draw_rectangle(
            int(candidate["x"] * scale + 0.5),
            int(candidate["y"] * scale + 0.5),
            int(candidate["w"] * scale + 0.5),
            int(candidate["h"] * scale + 0.5),
            color=color,
            thickness=2,
        )

    img.draw_cross(
        int(center_x * scale + 0.5),
        int(center_y * scale + 0.5),
        color=color,
        size=5,
        thickness=2,
    )


def draw_raw_rectangle(img, corners, scale=1.0, color=(255, 255, 255)):
    if corners is None or len(corners) != 4:
        return

    points = []
    for point in corners:
        points.append(
            (int(point[0] * scale + 0.5), int(point[1] * scale + 0.5))
        )
    for index in range(4):
        x0, y0 = points[index]
        x1, y1 = points[(index + 1) % 4]
        img.draw_line(x0, y0, x1, y1, color=color, thickness=2)


def draw_laser_prediction(img, laser_point, scale=1.0):
    if laser_point is None:
        return

    x = int(laser_point["x"] * scale)
    y = int(laser_point["y"] * scale)
    img.draw_circle(x, y, 10, color=(0, 0, 255), thickness=2)
    img.draw_cross(x, y, color=(0, 0, 255), size=7, thickness=2)


# ---------------------------------------------------------------------------
# 主程序。
# ---------------------------------------------------------------------------


def main():
    sensor = None
    uart = None
    logger = None
    media_initialized = False
    display_initialized = False
    tracker = TargetTracker()
    laser_aimer = None
    snapshot_key = YbKey() if PRODUCTION_SNAPSHOT_ENABLED else None
    snapshot_last_key = False
    snapshot_dir = None
    snapshot_index = 1
    clock = time.clock()
    frame_count = 0
    light_profile = PROFILE_NORMAL
    mean_brightness = 0
    last_state = None
    last_frame_ms = time.ticks_ms()
    last_lost_send_ms = 0
    pitch_trim_deg = 0.0
    # 帧率使用真实测量：clock.fps() 是自启动以来的长窗口平均，
    # 上升很慢且掩盖实际帧率；因此改为根据两次调试打印之间
    # 处理的帧数和时间计算窗口帧率。
    debug_print_tick = time.ticks_ms()
    debug_print_frame = 0
    log_fps_tick = time.ticks_ms()
    log_fps_frame = 0

    try:
        logger = RunLogger("PRODUCTION", None)
        if logger is not None:
            logger.trace("logger-ready")
        if GEOMETRY_LANDING_ENABLED:
            try:
                laser_aimer = PlanarLaserAimer()
            except Exception as exc:
                print("[GEOM] disabled:", exc)
        if logger is not None:
            logger.trace(
                "aimer-ready={}".format(1 if laser_aimer is not None else 0)
            )
        uart = configure_vision_uart()
        if logger is not None:
            logger.trace("uart-ready")
        if logger is not None:
            logger.trace("sensor-config-begin")
        sensor = Sensor(
            id=SENSOR_ID,
            width=SENSOR_SOURCE_WIDTH,
            height=SENSOR_SOURCE_HEIGHT,
            fps=SENSOR_FPS,
        )
        sensor.reset()
        time.sleep_ms(100)

        sensor.set_framesize(
            width=FRAME_WIDTH,
            height=FRAME_HEIGHT,
            chn=CAM_CHN_ID_0,
        )
        sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_0)
        if (
            DISPLAY_ENABLED
            or HIGH_RES_VERIFY_ENABLED
            or PRODUCTION_SNAPSHOT_ENABLED
        ):
            sensor.set_framesize(
                width=HIGH_RES_WIDTH,
                height=HIGH_RES_HEIGHT,
                chn=CAM_CHN_ID_1,
            )
            sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)

        if logger is not None:
            logger.trace("sensor-config-done")

        if SENSOR_GAIN is not None:
            try:
                gain = k_sensor_gain()
                gain.gain[0] = SENSOR_GAIN
                sensor.again(gain)
            except Exception:
                pass

        if DISPLAY_ENABLED:
            Display.init(
                Display.ST7701,
                width=DISPLAY_WIDTH,
                height=DISPLAY_HEIGHT,
                to_ide=True,
            )
            display_initialized = True

        MediaManager.init()
        media_initialized = True
        if logger is not None:
            logger.trace("media-ready")
        sensor.run()
        if logger is not None:
            logger.trace("sensor-running")

        # 周期调试行的耗时分解。循环按 TARGET_PROCESS_FPS 控制节拍，
        # 关键是要知道每帧时间花在什么地方：等待两次取图、检测、跟踪，
        # 还是向 IDE 推送画面。下面的每一项都会填入统计。
        #
        prev_loop_start_ms = None
        # 垃圾回收和节拍休眠位于循环末尾，因此它们的读数会在
        # 下一帧的调试行中显示。
        prev_gc_ms = 0
        prev_pace_ms = 0

        while True:
            os.exitpoint()
            clock.tick()
            loop_start_ms = time.ticks_ms()
            period_ms = (
                0
                if prev_loop_start_ms is None
                else time.ticks_diff(loop_start_ms, prev_loop_start_ms)
            )
            prev_loop_start_ms = loop_start_ms

            # Age starts before snapshot: sensor wait/exposure/readout plus all
            # later detection and formatting are part of the latency seen by H723.
            image_capture_ms = time.ticks_ms()
            if frame_count == 0 and logger is not None:
                logger.trace("first-snapshot-begin")
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            if frame_count == 0 and logger is not None:
                logger.trace("first-snapshot-done")
            high_res_img = None
            # 通道 1 只用于屏幕预览，且高分辨率复核已关闭，
            # 因此每帧取该通道是纯开销；每隔 N 帧取一次
            # 已足够提供人眼可读的预览。
            if (DISPLAY_ENABLED or HIGH_RES_VERIFY_ENABLED) and (
                frame_count % DISPLAY_EVERY_N_FRAMES == 0
            ):
                high_res_img = sensor.snapshot(chn=CAM_CHN_ID_1)
            snap_ms = time.ticks_diff(time.ticks_ms(), loop_start_ms)

            if frame_count % PROFILE_EVERY_N_FRAMES == 0:
                profile_start_ms = time.ticks_ms()
                profile_roi = None
                if tracker.state == "TRACK":
                    profile_roi = make_tracking_roi(tracker.last)
                near_profile = (
                    tracker.last is not None
                    and tracker.last.get("distance_m", 99.0) <=
                    NEAR_FIXED_CANNY_DISTANCE_M
                )
                light_profile = make_dynamic_profile(
                    img,
                    profile_roi,
                    near_profile,
                    light_profile,
                )
                mean_brightness = light_profile["mean"]
                profile_ms = time.ticks_diff(time.ticks_ms(), profile_start_ms)
            else:
                profile_ms = 0

            detection_mode = tracker.detection_mode()
            diagnostics = new_frame_diagnostics(detection_mode)
            diagnostics["profile_ms"] = float(profile_ms)
            raw_rectangles = [] if (
                DISPLAY_ENABLED and DISPLAY_DRAW_RAW_RECTANGLES
            ) else None
            stage_ms = time.ticks_ms()
            candidates = detect_candidates(
                img,
                light_profile,
                detection_mode,
                diagnostics,
                tracker.detection_roi(),
                high_res_img,
                frame_count,
                tracker.last,
                raw_rectangles,
            )
            diagnostics["detect_ms"] = float(
                time.ticks_diff(time.ticks_ms(), stage_ms)
            )
            track_start_ms = time.ticks_ms()
            candidates = refine_candidates_with_high_res(
                img,
                high_res_img,
                candidates,
                light_profile,
                diagnostics,
                detection_mode,
                frame_count,
            )
            target = tracker.update(candidates, diagnostics)
            track_ms = time.ticks_diff(time.ticks_ms(), track_start_ms)
            best_candidate = tracker.selected_candidate

            # 跟踪器仍可短时预测框用于显示和重捕获；控制只接受本帧新鲜的
            # 三维解算，不能拿预测框继续伪造激光控制量。
            predicted_target = None
            if target is None:
                predicted_target = tracker.predict_reference(time.ticks_ms())
            control_target = target if target is not None else predicted_target
            geometry_state = 0
            geometry_solution = None
            reprojection_px = 0.0
            pose_refined = 0
            if GEOMETRY_LANDING_ENABLED and target is not None:
                if laser_aimer is not None:
                    try:
                        geometry_target = target
                        refined_corners, corner_refined, corner_shift = (
                            refine_geometry_corners(
                                img, target.get("corners")
                            )
                        )
                        if corner_refined:
                            geometry_target = {}
                            for key in target:
                                geometry_target[key] = target[key]
                            geometry_target["corners"] = refined_corners
                        diagnostics["corner_refined"] = (
                            1 if corner_refined else 0
                        )
                        diagnostics["corner_shift"] = corner_shift
                        geometry_solution = laser_aimer.solve(
                            geometry_target, img
                        )
                    except Exception as exc:
                        diagnostics["geometry_reject"] = str(exc)
                        geometry_state = _geometry_reject_code(
                            diagnostics["geometry_reject"]
                        )
                        if geometry_state == 4:
                            try:
                                reprojection_px = float(
                                    diagnostics["geometry_reject"]
                                    .split(" ")[2]
                                    .replace("px", "")
                                )
                                pose_refined = 1
                            except Exception:
                                reprojection_px = 0.0
                    if (
                        geometry_solution is None
                        and geometry_state == 0
                    ):
                        geometry_state = 10
                        diagnostics["geometry_reject"] = "invalid corners"
                else:
                    geometry_state = 11
                    diagnostics["geometry_reject"] = "aimer unavailable"
            elif not GEOMETRY_LANDING_ENABLED and control_target is not None:
                geometry_state = 12
                diagnostics["geometry_reject"] = "fixed laser point"
            elif control_target is not None:
                geometry_state = 2
                diagnostics["geometry_reject"] = "predicted target"
            if geometry_solution is not None:
                geometry_state = 1
                laser_point = {
                    "x": geometry_solution["laser_uv"][0],
                    "y": geometry_solution["laser_uv"][1],
                    "distance_m": geometry_solution["depth_mm"] / 1000.0,
                }
                gimbal_error = {
                    "yaw_deg": geometry_solution["yaw_deg"],
                    "pitch_deg": geometry_solution["pitch_deg"],
                }
                diagnostics["geometry_valid"] = 1
                diagnostics["geometry_state"] = 1
                diagnostics["reprojection_px"] = geometry_solution[
                    "reprojection_px"
                ]
                reprojection_px = geometry_solution["reprojection_px"]
                diagnostics["pose_refined"] = (
                    1 if geometry_solution["pose_refined"] else 0
                )
                pose_refined = diagnostics["pose_refined"]
                diagnostics["spot_status"] = geometry_solution["spot_status"]
                diagnostics["spot_correction"] = (
                    geometry_solution["correction_yaw_deg"],
                    geometry_solution["correction_pitch_deg"],
                )
            else:
                diagnostics["geometry_valid"] = 0
                diagnostics["geometry_state"] = geometry_state
                if (
                    not GEOMETRY_LANDING_ENABLED
                    and control_target is not None
                ):
                    laser_point = calculate_fixed_laser_point(control_target)
                    gimbal_error = calculate_gimbal_error(
                        control_target, laser_point
                    )
                else:
                    laser_point = None
                    gimbal_error = None
            pitch_trim_deg = update_pitch_trim(pitch_trim_deg, gimbal_error, tracker)
            diagnostics["pitch_trim_deg"] = pitch_trim_deg
            sent_error = apply_pitch_trim(gimbal_error, pitch_trim_deg)

            tx_start_ms = time.ticks_ms()
            image_age_ms = float(min(
                1000,
                max(0, time.ticks_diff(time.ticks_ms(), image_capture_ms)),
            ))
            diagnostics["image_age_ms"] = image_age_ms
            last_lost_send_ms = send_vision_frame(
                uart,
                control_target,
                laser_point,
                sent_error,
                diagnostics,
                last_lost_send_ms,
                tracker.vx,
                tracker.vy,
                image_age_ms,
                geometry_state,
                reprojection_px,
                pose_refined,
                control_target["w"] if control_target is not None else 0.0,
                control_target["h"] if control_target is not None else 0.0,
                (
                    2 if control_target is not None and
                    control_target.get("inner")
                    else (
                        1 if control_target is not None and
                        control_target.get("layer") == "outer"
                        else 0
                    )
                ),
            )
            diagnostics["tx_ms"] = float(
                time.ticks_diff(time.ticks_ms(), tx_start_ms)
            )

            if snapshot_key is not None:
                snapshot_pressed = snapshot_key.is_pressed() == 1
                if snapshot_pressed and not snapshot_last_key:
                    if snapshot_dir is None:
                        snapshot_dir = "{}/laser_prod_{}".format(
                            PRODUCTION_SNAPSHOT_ROOT,
                            time.ticks_ms(),
                        )
                        ensure_dir(snapshot_dir)
                    if high_res_img is None:
                        high_res_img = sensor.snapshot(chn=CAM_CHN_ID_1)
                    path = "{}/{}_mean{}.jpg".format(
                        snapshot_dir,
                        snapshot_index,
                        mean_brightness,
                    )
                    high_res_img.save(path)
                    print("production snapshot:", path)
                    snapshot_index += 1
                snapshot_last_key = snapshot_pressed

            # 只有本帧实际获取了通道 1 图像时才绘制，
            # 否则 high_res_img 为空，没有可绘制的画面。
            if DISPLAY_ENABLED and high_res_img is not None:
                stage_ms = time.ticks_ms()
                display_scale = HIGH_RES_WIDTH / FRAME_WIDTH
                # 绘制用于计算瞄准点的同一目标，使屏幕上的框中心
                # 与瞄准点完全重合；当激光位于靶心时，
                # 两者也应对应。
                overlay_candidate = (
                    target
                    if target is not None
                    else (
                        predicted_target
                        if predicted_target is not None
                        else best_candidate
                    )
                )
                if DISPLAY_DRAW_RAW_RECTANGLES and raw_rectangles:
                    for corners in raw_rectangles:
                        draw_raw_rectangle(
                            high_res_img,
                            corners,
                            display_scale,
                        )
                if DISPLAY_DRAW_FINAL_TARGET and overlay_candidate is not None:
                    if target is not None:
                        outline_color = (0, 255, 0)      # 已锁定
                    elif predicted_target is not None or tracker.state in (
                        "CONFIRM",
                        "RECAPTURE_ROI",
                        "RECAPTURE_FULL",
                    ):
                        outline_color = (255, 255, 0)    # 确认中或使用预测位置
                    else:
                        outline_color = (255, 0, 0)      # 无候选
                    draw_candidate(
                        high_res_img,
                        overlay_candidate,
                        target is not None,
                        display_scale,
                        color=outline_color,
                    )
                if DISPLAY_DRAW_LASER_PREDICTION:
                    draw_laser_prediction(
                        high_res_img, laser_point, display_scale
                    )
                if DISPLAY_DRAW_TARGET_TEXT:
                    try:
                        if overlay_candidate is None:
                            target_text = "target none"
                        else:
                            target_text = "target {} ({:.0f},{:.0f})".format(
                                overlay_candidate["layer"],
                                overlay_candidate["cx"],
                                overlay_candidate["cy"],
                            )
                        high_res_img.draw_string(
                            8,
                            8,
                            target_text,
                            color=(255, 255, 255),
                        )
                    except Exception:
                        pass
                Display.show_image(high_res_img)
                diagnostics["display_ms"] = float(
                    time.ticks_diff(time.ticks_ms(), stage_ms)
                )

            if logger is not None and (
                tracker.state != last_state
                or frame_count % LOG_EVERY_N_FRAMES == 0
            ):
                log_start_ms = time.ticks_ms()
                now_log_ms = time.ticks_ms()
                log_span_ms = time.ticks_diff(now_log_ms, log_fps_tick)
                log_span_frames = frame_count - log_fps_frame
                win_fps = (
                    log_span_frames * 1000.0 / log_span_ms
                    if log_span_ms > 0
                    else 0.0
                )
                logger.log(
                    format_frame_log(
                        tracker.state,
                        control_target,
                        laser_point,
                        light_profile,
                        mean_brightness,
                        clock.fps(),
                        diagnostics,
                        frame_count,
                        win_fps,
                        period_ms,
                        snap_ms,
                        track_ms,
                        prev_gc_ms,
                        prev_pace_ms,
                        tracker,
                        sent_error,
                    )
                )
                # 包含字符串格式化、文件写入以及本记录触发的刷新耗时。
                #
                diagnostics["log_ms"] = float(
                    time.ticks_diff(time.ticks_ms(), log_start_ms)
                )
                diagnostics["flush_ms"] = logger.flush_ms
                last_state = tracker.state
                log_fps_tick = time.ticks_ms()
                log_fps_frame = frame_count

            if DEBUG_PRINT and frame_count % DEBUG_PRINT_EVERY_N_FRAMES == 0:
                # 窗口帧率才是真实值：它等于实际产生帧数除以
                # 两次打印之间的实际时间；clock.fps() 是自启动以来的平均值，
                # 通常远低于瞬时帧率。
                now_print_ms = time.ticks_ms()
                span_ms = time.ticks_diff(now_print_ms, debug_print_tick)
                span_frames = frame_count - debug_print_frame
                win_fps = (span_frames * 1000.0 / span_ms) if span_ms > 0 else 0.0
                debug_print_tick = now_print_ms
                debug_print_frame = frame_count
                if target is None:
                    print(
                        "frame={} state={} target=none fps={:.1f} winfps={:.1f} "
                        "t={:.0f}/{:.0f}/{:.0f}/{:.0f}/{:.0f}ms"
                        "(per/snap/det/trk/dis) "
                        "b={:.0f}/{:.0f}/{:.0f}/{:.0f}/{:.0f}/{:.0f}/"
                        "{:.0f}/{:.0f}/{:.0f}ms"
                        "(crop/cv/cand/prof/log/flush/gc/tx/pace)".format(
                            frame_count,
                            tracker.state,
                            clock.fps(),
                            win_fps,
                            period_ms,
                            snap_ms,
                            diagnostics["detect_ms"],
                            track_ms,
                            diagnostics["display_ms"],
                            diagnostics["crop_ms"],
                            diagnostics["cv_ms"],
                            diagnostics["cand_ms"],
                            diagnostics["profile_ms"],
                            diagnostics["log_ms"],
                            diagnostics["flush_ms"],
                            prev_gc_ms,
                            diagnostics["tx_ms"],
                            prev_pace_ms,
                        )
                    )
                else:
                    print(
                        "frame={} state={} target=({},{},{},{}) "
                        "err=({:+.1f},{:+.1f})deg laser=({:.0f},{:.0f}) "
                        "trim={:+.1f}deg det={}/{} area={} white={} wlow={} "
                        "long={}/{} hi={}/{} d={:.0f}cm fps={:.1f} winfps={:.1f} "
                        "geom={} reproj={:.1f} refine={} corner=({},{:.2f}) "
                        "spot={} "
                        "corr=({:+.2f},{:+.2f}) "
                        "t={:.0f}/{:.0f}/{:.0f}/{:.0f}/{:.0f}ms"
                        "(per/snap/det/trk/dis) "
                        "b={:.0f}/{:.0f}/{:.0f}/{:.0f}/{:.0f}/{:.0f}/"
                        "{:.0f}/{:.0f}/{:.0f}ms"
                        "(crop/cv/cand/prof/log/flush/gc/tx/pace)".format(
                            frame_count,
                            tracker.state,
                            target["x"],
                            target["y"],
                            target["w"],
                            target["h"],
                            gimbal_error["yaw_deg"] if gimbal_error else 0.0,
                            gimbal_error["pitch_deg"] if gimbal_error else 0.0,
                            laser_point["x"] if laser_point else 0.0,
                            laser_point["y"] if laser_point else 0.0,
                            pitch_trim_deg,
                            diagnostics["cv_lite_count"],
                            diagnostics["cv_lite_raw"],
                            diagnostics["area_reject"],
                            diagnostics["white_reject"],
                            diagnostics["white_low"],
                            diagnostics["long_range_search"],
                            diagnostics["long_range_hit"],
                            diagnostics["hi_res_search"],
                            diagnostics["hi_res_hit"],
                            target["distance_m"] * 100.0,
                            clock.fps(),
                            win_fps,
                            diagnostics["geometry_state"],
                            diagnostics["reprojection_px"],
                            diagnostics["pose_refined"],
                            diagnostics["corner_refined"],
                            diagnostics["corner_shift"],
                            diagnostics["spot_status"],
                            diagnostics["spot_correction"][0],
                            diagnostics["spot_correction"][1],
                            period_ms,
                            snap_ms,
                            diagnostics["detect_ms"],
                            track_ms,
                            diagnostics["display_ms"],
                            diagnostics["crop_ms"],
                            diagnostics["cv_ms"],
                            diagnostics["cand_ms"],
                            diagnostics["profile_ms"],
                            diagnostics["log_ms"],
                            diagnostics["flush_ms"],
                            prev_gc_ms,
                            diagnostics["tx_ms"],
                            prev_pace_ms,
                        )
                    )

            frame_count += 1
            if frame_count % GC_EVERY_N_FRAMES == 0:
                gc_start_ms = time.ticks_ms()
                gc.collect()
                prev_gc_ms = time.ticks_diff(time.ticks_ms(), gc_start_ms)
            pace_start_ms = time.ticks_ms()
            last_frame_ms = pace_frame(last_frame_ms)
            prev_pace_ms = time.ticks_diff(time.ticks_ms(), pace_start_ms)

    except KeyboardInterrupt as error:
        if logger is not None:
            logger.log("K230 vision stopped reason={}".format(error))
    except BaseException as error:
        if logger is not None:
            logger.log("K230 vision fatal reason={}".format(error))
    finally:
        if logger is not None:
            logger.close()
        if uart is not None:
            uart.deinit()
        if isinstance(sensor, Sensor):
            sensor.stop()
        if display_initialized:
            Display.deinit()
        if media_initialized:
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
            time.sleep_ms(100)
            MediaManager.deinit()


def run_test_mode():
    sensor = None
    uart = None
    logger = None
    media_initialized = False
    display_initialized = False
    key = YbKey()
    tracker = TargetTracker()
    clock = time.clock()
    frame_count = 0
    mean_brightness = 0
    capture_index = 1
    last_key_state = False
    last_state = None
    last_frame_ms = time.ticks_ms()
    session_name = "{}_{}".format(TEST_CASE_NAME, time.ticks_ms())
    save_dir = "{}/{}".format(TEST_SAVE_ROOT, session_name)
    csv_path = "{}/captures.csv".format(save_dir)
    light_profile = PROFILE_NORMAL
    last_lost_send_ms = 0
    pitch_trim_deg = 0.0

    try:
        ensure_dir(save_dir)
        print("test mode save_dir:", save_dir)
        logger = RunLogger("TEST", TEST_CASE_NAME)
        if TEST_UART_ENABLED:
            uart = configure_vision_uart()

        sensor = Sensor(
            id=SENSOR_ID,
            width=SENSOR_SOURCE_WIDTH,
            height=SENSOR_SOURCE_HEIGHT,
            fps=SENSOR_FPS,
        )
        sensor.reset()
        time.sleep_ms(100)
        sensor.set_framesize(
            width=FRAME_WIDTH,
            height=FRAME_HEIGHT,
            chn=CAM_CHN_ID_0,
        )
        sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_0)
        sensor.set_framesize(
            width=HIGH_RES_WIDTH,
            height=HIGH_RES_HEIGHT,
            chn=CAM_CHN_ID_1,
        )
        sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)

        if TEST_DISPLAY_ENABLED:
            Display.init(
                Display.ST7701,
                width=DISPLAY_WIDTH,
                height=DISPLAY_HEIGHT,
                to_ide=True,
            )
            display_initialized = True

        MediaManager.init()
        media_initialized = True
        sensor.run()

        while True:
            os.exitpoint()
            clock.tick()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            gray = img
            high_res_img = None
            if (TEST_DISPLAY_ENABLED or HIGH_RES_VERIFY_ENABLED) and (
                frame_count % DISPLAY_EVERY_N_FRAMES == 0
            ):
                high_res_img = sensor.snapshot(chn=CAM_CHN_ID_1)

            if frame_count % PROFILE_EVERY_N_FRAMES == 0:
                profile_roi = None
                if tracker.state == "TRACK":
                    profile_roi = make_tracking_roi(tracker.last)
                light_profile = make_dynamic_profile(gray, profile_roi)
                mean_brightness = light_profile["mean"]

            detection_mode = tracker.detection_mode()
            diagnostics = new_frame_diagnostics(detection_mode)
            stage_ms = time.ticks_ms()
            candidates = detect_candidates(
                gray,
                light_profile,
                detection_mode,
                diagnostics,
                tracker.detection_roi(),
                high_res_img,
                frame_count,
                tracker.last,
            )
            diagnostics["detect_ms"] = float(
                time.ticks_diff(time.ticks_ms(), stage_ms)
            )
            candidates = refine_candidates_with_high_res(
                gray,
                high_res_img,
                candidates,
                light_profile,
                diagnostics,
                detection_mode,
                frame_count,
            )
            target = tracker.update(candidates, diagnostics)
            best_candidate = tracker.selected_candidate
            prediction_target = (
                target if target is not None else best_candidate
            )
            laser_point = calculate_fixed_laser_point(prediction_target)
            gimbal_error = calculate_gimbal_error(
                prediction_target, laser_point
            )
            pitch_trim_deg = update_pitch_trim(
                pitch_trim_deg,
                gimbal_error if target is not None else None,
                tracker,
            )
            diagnostics["pitch_trim_deg"] = pitch_trim_deg

            sent_error = apply_pitch_trim(gimbal_error, pitch_trim_deg)
            last_lost_send_ms = send_vision_frame(
                uart,
                target,
                laser_point,
                sent_error,
                diagnostics,
                last_lost_send_ms,
                tracker.vx,
                tracker.vy,
            )

            key_pressed = key.is_pressed() == 1
            if key_pressed and not last_key_state:
                if high_res_img is None:
                    high_res_img = sensor.snapshot(chn=CAM_CHN_ID_1)
                path = "{}/{}_{}_mean{}.jpg".format(
                    save_dir,
                    session_name,
                    capture_index,
                    mean_brightness,
                )
                # Save the RGB preview channel: the direct grayscale channel can
                # expose the sensor's padded stride as four tearing quadrants.
                high_res_img.save(path)

                record_target = target if target is not None else best_candidate
                if record_target is None:
                    x = y = width = height = 0
                    center_x = center_y = 0
                    white_ratio = 0.0
                    dark_border_ratio = 0.0
                    area = 0.0
                    distance_cm = 0.0
                    laser_x = 0.0
                    laser_y = 0.0
                    gimbal_yaw_deg = VISION_LOST_VALUE
                    gimbal_pitch_deg = VISION_LOST_VALUE
                    detected = 0
                    tracked = 0
                    record_layer = "-"
                else:
                    x = record_target["x"]
                    y = record_target["y"]
                    width = record_target["w"]
                    height = record_target["h"]
                    center_x = record_target["cx"]
                    center_y = record_target["cy"]
                    white_ratio = record_target["white"]
                    dark_border_ratio = record_target["dark_border"]
                    area = record_target["area"]
                    record_layer = record_target["layer"]
                    detected = 1
                    tracked = 1 if target is not None else 0
                    if laser_point is None:
                        distance_cm = 0.0
                        laser_x = 0.0
                        laser_y = 0.0
                    else:
                        distance_cm = laser_point["distance_m"] * 100.0
                        laser_x = laser_point["x"]
                        laser_y = laser_point["y"]
                    if gimbal_error is None:
                        gimbal_yaw_deg = VISION_LOST_VALUE
                        gimbal_pitch_deg = VISION_LOST_VALUE
                    else:
                        gimbal_yaw_deg = gimbal_error["yaw_deg"]
                        gimbal_pitch_deg = gimbal_error["pitch_deg"]

                record_values = [
                    capture_index,
                    path,
                    mean_brightness,
                    detected,
                    tracked,
                    x,
                    y,
                    width,
                    height,
                    int(center_x),
                    int(center_y),
                    white_ratio,
                    dark_border_ratio,
                    area,
                    distance_cm,
                    laser_x,
                    laser_y,
                    clock.fps(),
                    gimbal_yaw_deg,
                    gimbal_pitch_deg,
                    diagnostics["candidate_source"],
                    diagnostics["candidate_count"],
                    diagnostics["best_distance_cm"],
                    diagnostics["distance_reject"],
                    diagnostics["size_reject"],
                    diagnostics["center_reject"],
                    diagnostics["roi_reject"],
                    diagnostics["lost_reason"],
                    diagnostics["selected_mode"],
                    record_layer,
                ]
                record = ",".join([str(value) for value in record_values]) + "\n"
                append_capture_record(csv_path, record)
                if logger is not None:
                    logger.log(
                        "capture={} detected={} tracked={} distance_cm={:.1f} "
                        "laser=({:.1f},{:.1f})".format(
                            capture_index,
                            detected,
                            tracked,
                            distance_cm,
                            laser_x,
                            laser_y,
                        )
                    )
                print("captured:", path, "detected:", detected)

                capture_index += 1
                time.sleep_ms(200)

            # 未获取通道 1 图像的帧不绘制叠加层。
            if TEST_DISPLAY_ENABLED and high_res_img is not None:
                # 仅当按键已将图片保存后才绘制叠加层，
                # 这样抓拍图片中不会带绿色框、绿色十字或蓝色
                # 瞄准点，可以直接从原图测量靶心和
                # 实际激光光斑，不受叠加层干扰。
                stage_ms = time.ticks_ms()
                display_scale = HIGH_RES_WIDTH / FRAME_WIDTH
                overlay_candidate = target if target is not None else best_candidate
                if overlay_candidate is not None:
                    draw_candidate(
                        high_res_img,
                        overlay_candidate,
                        target is not None,
                        display_scale,
                    )
                draw_laser_prediction(high_res_img, laser_point, display_scale)
                Display.show_image(high_res_img)
                diagnostics["display_ms"] = float(
                    time.ticks_diff(time.ticks_ms(), stage_ms)
                )

            last_key_state = key_pressed
            frame_count += 1

            if logger is not None and (
                tracker.state != last_state
                or frame_count % LOG_EVERY_N_FRAMES == 0
            ):
                logger.log(
                    format_frame_log(
                        tracker.state,
                        target,
                        laser_point,
                        light_profile,
                        mean_brightness,
                        clock.fps(),
                        diagnostics,
                        frame_count,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        tracker,
                        sent_error if target is not None else None,
                    )
                )
                last_state = tracker.state

            if DEBUG_PRINT and frame_count % DEBUG_PRINT_EVERY_N_FRAMES == 0:
                if target is None:
                    print(
                        "test frame={} state={} target=none fps={:.1f}".format(
                            frame_count,
                            tracker.state,
                            clock.fps(),
                        )
                    )
                else:
                    print(
                        "test frame={} state={} target=({},{},{},{}) fps={:.1f}".format(
                            frame_count,
                            tracker.state,
                            target["x"],
                            target["y"],
                            target["w"],
                            target["h"],
                            clock.fps(),
                        )
                    )

            if frame_count % GC_EVERY_N_FRAMES == 0:
                gc.collect()
            last_frame_ms = pace_frame(last_frame_ms)

    except KeyboardInterrupt as error:
        if logger is not None:
            logger.log("K230 test mode stopped reason={}".format(error))
    except BaseException as error:
        if logger is not None:
            logger.log("K230 test mode fatal reason={}".format(error))
    finally:
        if logger is not None:
            logger.close()
        if uart is not None:
            uart.deinit()
        if isinstance(sensor, Sensor):
            sensor.stop()
        if display_initialized:
            Display.deinit()
        if media_initialized:
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
            time.sleep_ms(100)
            MediaManager.deinit()


if __name__ == "__main__":
    if RUN_MODE == "TEST":
        run_test_mode()
    else:
        main()
