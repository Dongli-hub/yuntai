# -*- coding: utf-8 -*-
"""Host-side regression for the reference red-blob laser detector."""

import io
import os
import re
import sys
import types

import cv2
import numpy as np


ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
MAIN_PY = os.path.join(ROOT, "k230_aim", "main.py")


def install_stubs():
    image_module = types.ModuleType("image")
    sys.modules["image"] = image_module
    sys.modules["uos"] = types.ModuleType("uos")

    machine = types.ModuleType("machine")
    machine.FPIOA = type("FPIOA", (), {})
    machine.UART = type("UART", (), {"UART1": 1})
    sys.modules["machine"] = machine

    media = types.ModuleType("media")
    media.__path__ = []
    sys.modules["media"] = media
    display = types.ModuleType("media.display")
    display.Display = type("Display", (), {"ST7701": 1})
    sys.modules["media.display"] = display
    media_media = types.ModuleType("media.media")
    media_media.MediaManager = type("MediaManager", (), {})
    sys.modules["media.media"] = media_media
    sensor = types.ModuleType("media.sensor")
    sensor.Sensor = type("Sensor", (), {})
    sensor.CAM_CHN_ID_0 = 0
    sensor.CAM_CHN_ID_1 = 1
    sys.modules["media.sensor"] = sensor

    yb = types.ModuleType("ybUtils")
    yb.__path__ = []
    sys.modules["ybUtils"] = yb
    yb_key = types.ModuleType("ybUtils.YbKey")
    yb_key.YbKey = type("YbKey", (), {})
    sys.modules["ybUtils.YbKey"] = yb_key

    ulab = types.ModuleType("ulab")
    ulab.__path__ = []
    ulab.numpy = np
    sys.modules["ulab"] = ulab
    sys.modules["ulab.numpy"] = np


def load_program():
    install_stubs()
    source = io.open(MAIN_PY, encoding="utf-8").read()
    source = re.sub(r"(?m)^main\(\)\s*$", "# main()", source)
    namespace = {"__name__": "main_module"}
    exec(compile(source, MAIN_PY, "exec"), namespace)
    return namespace


class FakeBlob(object):
    def __init__(self, x, y, w, h, pixels, cx, cy):
        self._values = (x, y, w, h, pixels, cx, cy)

    def __getitem__(self, index):
        return self._values[index]

    def x(self):
        return self._values[0]

    def y(self):
        return self._values[1]

    def w(self):
        return self._values[2]

    def h(self):
        return self._values[3]

    def pixels(self):
        return self._values[4]

    def cx(self):
        return self._values[5]

    def cy(self):
        return self._values[6]


class ColorFrame(object):
    """Small OpenCV emulation of CanMV RGB565 image.find_blobs(LAB)."""

    def __init__(self, bgr):
        self.bgr = bgr

    def find_blobs(self, thresholds, roi=None, merge=False, **kwargs):
        threshold = thresholds[0]
        lab = cv2.cvtColor(self.bgr, cv2.COLOR_BGR2LAB)
        lightness = lab[:, :, 0].astype(np.float64) * (100.0 / 255.0)
        a_channel = lab[:, :, 1].astype(np.int16) - 128
        b_channel = lab[:, :, 2].astype(np.int16) - 128
        selected = (
            (lightness >= threshold[0]) &
            (lightness <= threshold[1] + 1e-4) &
            (a_channel >= threshold[2]) &
            (a_channel <= threshold[3]) &
            (b_channel >= threshold[4]) &
            (b_channel <= threshold[5])
        )
        mask = (selected.astype(np.uint8) * 255)
        if roi is not None:
            keep = np.zeros_like(mask)
            x, y, w, h = [int(value) for value in roi]
            keep[max(0, y):y + h, max(0, x):x + w] = mask[
                max(0, y):y + h, max(0, x):x + w
            ]
            mask = keep
        count, _labels, stats, centers = cv2.connectedComponentsWithStats(
            mask, 4
        )
        blobs = []
        for index in range(1, count):
            pixels = int(stats[index, cv2.CC_STAT_AREA])
            if pixels < int(kwargs.get("pixels_threshold", 1)):
                continue
            blobs.append(FakeBlob(
                int(stats[index, cv2.CC_STAT_LEFT]),
                int(stats[index, cv2.CC_STAT_TOP]),
                int(stats[index, cv2.CC_STAT_WIDTH]),
                int(stats[index, cv2.CC_STAT_HEIGHT]),
                pixels,
                float(centers[index][0]),
                float(centers[index][1]),
            ))
        return blobs


def target_data(corners, distance_m=1.0):
    return {
        "corners": corners,
        "inner": True,
        "distance_m": distance_m,
    }


def synthetic_case(program):
    corners = ((250.0, 130.0), (470.0, 130.0),
               (470.0, 350.0), (250.0, 350.0))
    tracker = program["LiveLaserSpotTracker"]()
    image = np.full((480, 640, 3), 165, dtype=np.uint8)
    cv2.circle(image, (360, 235), 6, (255, 255, 255), -1)
    point, status = tracker.update(
        ColorFrame(image), target_data(corners)
    )
    assert point is not None and status == "live:white", status
    assert abs(point["x"] - 360.0) <= 2.0, point
    assert abs(point["y"] - 235.0) <= 2.0, point

    held, status = tracker.update(
        ColorFrame(np.full((480, 640, 3), 165, dtype=np.uint8)),
        target_data(corners),
    )
    assert held is None and status.startswith("lost:"), status

    red_only = np.full((480, 640, 3), 165, dtype=np.uint8)
    cv2.circle(red_only, (360, 235), 6, (20, 20, 255), -1)
    tracker = program["LiveLaserSpotTracker"]()
    point, status = tracker.update(
        ColorFrame(red_only), target_data(corners)
    )
    assert point is None and status.startswith("lost:"), status

    white_paper = np.full((480, 640, 3), 180, dtype=np.uint8)
    cv2.rectangle(white_paper, (280, 200), (360, 260), (255, 255, 255), -1)
    far_corners = (
        (270.0, 190.0), (370.0, 190.0),
        (370.0, 270.0), (270.0, 270.0),
    )
    tracker = program["LiveLaserSpotTracker"]()
    point, status = tracker.update(
        ColorFrame(white_paper), target_data(far_corners)
    )
    assert point is None and status.startswith("lost:"), status


def format_case(program):
    diagnostics = program["new_frame_diagnostics"]("NEAR")
    diagnostics["live_laser_status"] = "live:white"
    target = {
        "layer": "inner",
        "x": 260,
        "y": 150,
        "w": 160,
        "h": 180,
        "cx": 340.0,
        "cy": 240.0,
        "distance_m": 1.0,
        "predicted": False,
    }
    tracker = type("Tracker", (), {"vx": 0.0, "vy": 0.0})()
    line = program["format_frame_log"](
        "TRACK",
        target,
        {"x": 332.9, "y": 219.4},
        {"level": "normal"},
        100,
        80.0,
        diagnostics,
        1,
        80.0,
        12.5,
        1.0,
        2.0,
        3.0,
        0.0,
        tracker,
        {"yaw_deg": 0.2, "pitch_deg": -0.1},
    )
    assert "live_spot=live:white" in line


def field_image_case(program):
    path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "field_shots",
        "field_laser_1m.jpg",
    )
    if not os.path.isfile(path):
        print("field image skipped:", path)
        return
    data = np.fromfile(path, dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise AssertionError("cannot decode field image")
    corners = ((303.0, 175.0), (557.0, 175.0),
               (557.0, 351.0), (303.0, 351.0))
    tracker = program["LiveLaserSpotTracker"]()
    point, status = tracker.update(
        ColorFrame(image), target_data(corners)
    )
    assert point is not None and status == "live", status
    print("field laser:", point["x"], point["y"], status)


def main():
    program = load_program()
    synthetic_case(program)
    format_case(program)
    field_image_case(program)
    print("live laser tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
