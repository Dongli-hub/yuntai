# -*- coding: utf-8 -*-
"""离线验证靶面距离使用内框四边平均值和相机 fx/fy。"""
import io
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pc_sim_main as S  # noqa: E402


def load_program():
    S.install_fake_modules()
    source = io.open(S.MAIN_PY, encoding="utf-8").read()
    source = re.sub(r"(?m)^main\(\)\s*$", "# main()", source)
    program = {"__name__": "distance_test"}
    exec(compile(source, S.MAIN_PY, "exec"), program)
    return program


def expected_distance(fx, fy, width_px, height_px):
    distance_width = fx * 0.260 / width_px
    distance_height = fy * 0.170 / height_px
    return 0.5 * (distance_width + distance_height)


def check_axis_aligned(program):
    corners = (
        (100.0, 100.0),
        (165.0, 100.0),
        (165.0, 142.5),
        (100.0, 142.5),
    )
    metrics = program["calculate_distance_metrics"](
        corners, 65.0, 42.5, True
    )
    fx = program["CAMERA_CALIBRATION"]["fx"]
    fy = program["CAMERA_CALIBRATION"]["fy"]
    expected = expected_distance(fx, fy, 65.0, 42.5)
    error = abs(metrics["distance_m"] - expected)
    print("axis: r=%.6f expected=%.6f err=%.9f" %
          (metrics["distance_m"], expected, error))
    return error < 1e-6


def check_perspective(program):
    corners = (
        (100.0, 100.0),
        (165.0, 100.0),
        (160.0, 142.5),
        (100.0, 142.5),
    )
    metrics = program["calculate_distance_metrics"](
        corners, 65.0, 42.5, True
    )
    fx = program["CAMERA_CALIBRATION"]["fx"]
    fy = program["CAMERA_CALIBRATION"]["fy"]
    right_px = math.sqrt(5.0 * 5.0 + 42.5 * 42.5)
    expected = expected_distance(fx, fy, 62.5, 0.5 * (right_px + 42.5))
    error = abs(metrics["distance_m"] - expected)
    print("perspective: r=%.6f expected=%.6f err=%.9f" %
          (metrics["distance_m"], expected, error))
    return error < 1e-6


def check_planar_pose_range(program):
    camera_values = program["CAMERA_CALIBRATION"]
    camera = program["PoseCamera"](
        program["FRAME_WIDTH"],
        program["FRAME_HEIGHT"],
        camera_values["fx"],
        camera_values["fy"],
        camera_values["cx"],
        camera_values["cy"],
        camera_values["distortion"],
    )
    width_mm = 260.0
    height_mm = 170.0
    target_z_mm = 1500.0
    world = (
        (-width_mm * 0.5, -height_mm * 0.5, target_z_mm),
        (width_mm * 0.5, -height_mm * 0.5, target_z_mm),
        (width_mm * 0.5, height_mm * 0.5, target_z_mm),
        (-width_mm * 0.5, height_mm * 0.5, target_z_mm),
    )
    corners = tuple(camera.project(point) for point in world)
    estimator = program["TargetRangeEstimator"]()
    solution = estimator.solve({
        "corners": corners,
        "inner": True,
    })
    expected_raw = target_z_mm * 0.001
    expected_cal = (
        program["POSE_RANGE_CAL_SCALE"] * expected_raw +
        program["POSE_RANGE_CAL_OFFSET_M"]
    )
    raw_error = abs(solution["raw_range_m"] - expected_raw)
    cal_error = abs(solution["range_m"] - expected_cal)
    print(
        "pose range raw=%.6f expected=%.6f err=%.9f; "
        "cal=%.6f expected=%.6f err=%.9f" % (
            solution["raw_range_m"], expected_raw, raw_error,
            solution["range_m"], expected_cal, cal_error,
        )
    )
    return raw_error < 1e-5 and cal_error < 1e-5


def check_intrinsic_angle(program):
    fx = program["CAMERA_CALIBRATION"]["fx"]
    fy = program["CAMERA_CALIBRATION"]["fy"]
    error = program["calculate_gimbal_error"](
        {"cx": 330.0, "cy": 250.0},
        {"x": 310.0, "y": 240.0},
    )
    expected_yaw = math.degrees(math.atan2(20.0, fx))
    expected_pitch = math.degrees(math.atan2(10.0, fy))
    yaw_error = abs(error["yaw_deg"] - expected_yaw)
    pitch_error = abs(error["pitch_deg"] - expected_pitch)
    print(
        "intrinsic angle: yaw=%.6f pitch=%.6f err=(%.9f,%.9f)" % (
            error["yaw_deg"], error["pitch_deg"],
            yaw_error, pitch_error,
        )
    )
    return yaw_error < 1e-9 and pitch_error < 1e-9


def main():
    program = load_program()
    ok = check_axis_aligned(program)
    ok = check_perspective(program) and ok
    ok = check_planar_pose_range(program) and ok
    ok = check_intrinsic_angle(program) and ok
    print("distance test:", "OK" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
