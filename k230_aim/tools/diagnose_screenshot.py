# -*- coding: utf-8 -*-
"""用一张 IDE 预览截图复现当前 K230 检测链。"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pc_field_test as F  # noqa: E402


def crop_preview(path):
    data = np.fromfile(path, dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit("无法读取截图")
    # 截图中的 IDE 预览区（1620x1488）；统一缩回 640x480 检测坐标。
    crop = image[170:1294, 36:1587]
    color = cv2.resize(crop, (640, 480), interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, (640, 480), interpolation=cv2.INTER_AREA)
    return color, gray


def main():
    if len(sys.argv) < 2:
        raise SystemExit("用法: python diagnose_screenshot.py <截图>")
    color, gray = crop_preview(sys.argv[1])
    program = F.load_prog()
    if "--relax" in sys.argv:
        program["MAX_VALID_DISTANCE_M"] = 2.20
    frame = F.GrayFrame(np.ascontiguousarray(gray))
    profile = program["make_dynamic_profile"](frame, None)
    tracker = program["TargetTracker"]()
    diagnostics = None
    candidates = []
    target = None
    for frame_index in range(4):
        diagnostics = program["new_frame_diagnostics"](
            tracker.detection_mode()
        )
        candidates = program["detect_candidates"](
            frame,
            profile,
            tracker.detection_mode(),
            diagnostics,
            tracker.detection_roi(),
            None,
            frame_index,
            tracker.last,
        )
        target = tracker.update(candidates, diagnostics)
        print(
            "frame %d: state=%s candidates=%d target=%s" %
            (frame_index, tracker.state, len(candidates), target is not None)
        )
    laser = None
    if target is not None:
        corners = target["corners"]
        x0 = max(0, int(min(point[0] for point in corners)))
        x1 = min(640, int(max(point[0] for point in corners)) + 1)
        y0 = max(0, int(min(point[1] for point in corners)))
        y1 = min(480, int(max(point[1] for point in corners)) + 1)
        roi = gray[y0:y1, x0:x1]
        print(
            "roi stats: mean=%.1f max=%.1f p99=%.1f" %
            (
                float(np.mean(roi)),
                float(np.max(roi)),
                float(np.percentile(roi, 99)),
            )
        )
        lab = cv2.cvtColor(color[y0:y1, x0:x1], cv2.COLOR_BGR2LAB)
        threshold = program["LIVE_LASER_WHITE_THRESHOLD"]
        l_value = lab[:, :, 0].astype(np.float32) * 100.0 / 255.0
        a_value = lab[:, :, 1].astype(np.int16) - 128
        b_value = lab[:, :, 2].astype(np.int16) - 128
        mask = (
            (l_value >= threshold[0]) & (l_value <= threshold[1]) &
            (a_value >= threshold[2]) & (a_value <= threshold[3]) &
            (b_value >= threshold[4]) & (b_value <= threshold[5])
        ).astype(np.uint8)
        print("lab threshold pixels:", int(np.sum(mask)))
        component_count, _, stats, centers = cv2.connectedComponentsWithStats(
            mask, 4
        )
        for component in range(1, component_count):
            print(
                "lab component: area=%d box=(%d,%d,%d,%d) center=(%.1f,%.1f)" %
                (
                    int(stats[component, cv2.CC_STAT_AREA]),
                    int(stats[component, cv2.CC_STAT_LEFT]),
                    int(stats[component, cv2.CC_STAT_TOP]),
                    int(stats[component, cv2.CC_STAT_WIDTH]),
                    int(stats[component, cv2.CC_STAT_HEIGHT]),
                    float(centers[component][0]),
                    float(centers[component][1]),
                )
            )
    if target is not None and program["LIVE_LASER_SPOT_ENABLED"]:
        laser, laser_status = program["LiveLaserSpotTracker"]().update(
            frame, target
        )
        print("laser_status:", laser_status)
    print("profile:", profile)
    print("candidates:", len(candidates))
    for index, candidate in enumerate(candidates):
        print(
            "candidate[%d]: layer=%s inner=%s bbox=(%.1f,%.1f) "
            "distance=%.4fm width_r=%.4f height_r=%.4f consistency=%.3f "
            "corners=%s" % (
                index,
                candidate.get("layer"),
                candidate.get("inner"),
                candidate.get("w", 0.0),
                candidate.get("h", 0.0),
                candidate.get("distance_m", 0.0),
                candidate.get("distance_width_m", 0.0),
                candidate.get("distance_height_m", 0.0),
                candidate.get("distance_consistency", 0.0),
                candidate.get("corners"),
            )
        )
    print("target:", target)
    print("laser:", laser)
    for key in (
        "candidate_count",
        "cv_lite_raw",
        "cv_lite_count",
        "area_reject",
        "aspect_reject",
        "angle_reject",
        "rectangularity_reject",
        "distance_reject",
        "white_reject",
        "white_low",
        "dark_border_low",
        "long_range_search",
        "long_range_hit",
        "best_distance_cm",
        "lost_reason",
        "selected_mode",
    ):
        print("%s=%s" % (key, diagnostics.get(key)))
    return 0 if target is not None else 1


if __name__ == "__main__":
    sys.exit(main())
