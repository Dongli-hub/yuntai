# -*- coding: utf-8 -*-
"""解析网页串口助手保存的 records-*.json 并分析 H723 CSV。"""
import json
import math
import sys


def load_rows(path):
    with open(path, "rb") as stream:
        records = json.load(stream)
    chunks = []
    for item in records:
        if item.get("type") != "read":
            continue
        chunks.append(bytes(
            int(token.strip(), 16)
            for token in item.get("data", "").split(",")
            if token.strip()
        ))
    lines = b"".join(chunks).decode("ascii", "replace").splitlines()
    return [line.split(",") for line in lines if line[:1].isdigit()]


def stat(name, values):
    values = [float(value) for value in values]
    mean = sum(values) / max(1, len(values))
    variance = sum((value - mean) ** 2 for value in values) / max(1, len(values))
    print("%s n=%d mean=%.3f sd=%.3f min=%.3f max=%.3f" % (
        name, len(values), mean, math.sqrt(variance),
        min(values) if values else 0.0, max(values) if values else 0.0,
    ))


def main():
    rows = load_rows(sys.argv[1])
    rows = [row for row in rows if len(row) >= 55]
    aim = [row for row in rows if row[1] == "2" and row[2] == "1"]
    print("rows=%d aim=%d" % (len(rows), len(aim)))
    print("column_counts=%s" % sorted(set(len(row) for row in rows)))
    for row in rows[:3]:
        print("RAW", ",".join(row))
    for name, index in (
        ("yawE", 11), ("pitchE", 12), ("yawCmd", 21),
        ("pitchCmd", 26), ("range", 10), ("targetW", 33),
        ("targetH", 34), ("reportedFps", 28),
    ):
        stat(name, [row[index] for row in aim])
    chassis = [row for row in rows if row[36] == "1"]
    for name, index in (
        ("chassisSpeed", 38), ("chassisGz", 39),
        ("geometryFF", 45), ("chassisGzFF", 46),
        ("chassisGzLead", 47),
    ):
        stat(name, [row[index] for row in chassis])
    if chassis:
        first = chassis[0]
        last = chassis[-1]
        print("chassis counters first: good=%s bad=%s seq=%s raw=%s ver=%s" % (
            first[40], first[41], first[42], first[50], first[51]
        ))
        print("chassis counters last: good=%s bad=%s seq=%s raw=%s ver=%s" % (
            last[40], last[41], last[42], last[50], last[51]
        ))
    phase_names = {0: "TRACK", 1: "APPROACH", 2: "PIVOT", 3: "EXIT", 4: "FAULT"}
    for phase in sorted(set(int(row[37]) for row in rows)):
        phase_rows = [row for row in rows if int(row[37]) == phase]
        phase_aim = [
            row for row in phase_rows if row[1] == "2" and row[2] == "1"
        ]
        if phase_aim:
            print(
                "phase=%s count=%d aim=%d |yawE|=%.3f |pitchE|=%.3f" % (
                    phase_names.get(phase, str(phase)), len(phase_rows),
                    len(phase_aim),
                    sum(abs(float(row[11])) for row in phase_aim) /
                    len(phase_aim),
                    sum(abs(float(row[12])) for row in phase_aim) /
                    len(phase_aim),
                )
            )
    track_rows = [row for row in rows if int(row[37]) == 0]
    for route in sorted(set(int(row[43]) & 3 for row in track_rows)):
        route_rows = [
            row for row in track_rows if (int(row[43]) & 3) == route
        ]
        if route_rows:
            print(
                "track_route=%d count=%d |yawE|=%.3f |pitchE|=%.3f "
                "yawE=%.3f geometryFF=%.3f gzFF=%.3f" % (
                    route, len(route_rows),
                    sum(abs(float(row[11])) for row in route_rows) /
                    len(route_rows),
                    sum(abs(float(row[12])) for row in route_rows) /
                    len(route_rows),
                    sum(float(row[11]) for row in route_rows) /
                    len(route_rows),
                    sum(float(row[45]) for row in route_rows) /
                    len(route_rows),
                    sum(float(row[46]) for row in route_rows) /
                    len(route_rows),
                )
            )
    for low, high in ((0.0, 0.8), (0.8, 1.2), (1.2, 99.0)):
        band = [
            row for row in track_rows
            if low <= float(row[10]) < high
        ]
        if band:
            print(
                "track_distance=%.1f-%.1f count=%d |yawE|=%.3f "
                "|pitchE|=%.3f" % (
                    low, high, len(band),
                    sum(abs(float(row[11])) for row in band) / len(band),
                    sum(abs(float(row[12])) for row in band) / len(band),
                )
            )

    laser_steps = []
    target_steps = []
    for previous, current in zip(aim, aim[1:]):
        if int(current[0]) - int(previous[0]) <= 0:
            continue
        laser_steps.append(math.hypot(
            float(current[8]) - float(previous[8]),
            float(current[9]) - float(previous[9]),
        ))
        target_steps.append(math.hypot(
            float(current[6]) - float(previous[6]),
            float(current[7]) - float(previous[7]),
        ))
    if laser_steps:
        print(
            "laser_step mean=%.2f max=%.2f gt10=%d/%d" % (
                sum(laser_steps) / len(laser_steps), max(laser_steps),
                sum(step > 10.0 for step in laser_steps), len(laser_steps),
            )
        )
    if target_steps:
        print(
            "target_step mean=%.2f max=%.2f gt10=%d/%d" % (
                sum(target_steps) / len(target_steps), max(target_steps),
                sum(step > 10.0 for step in target_steps), len(target_steps),
            )
        )

    bad = []
    for row in aim:
        target_x = float(row[6])
        target_y = float(row[7])
        laser_x = float(row[8])
        laser_y = float(row[9])
        distance = float(row[10])
        target_w = float(row[33])
        target_h = float(row[34])
        if (
            target_x < 150 or target_x > 450 or
            target_y < 100 or target_y > 350 or
            laser_x < 200 or laser_x > 420 or
            laser_y < 120 or laser_y > 320 or
            distance < 0.3 or distance > 2.2 or
            target_w < 50 or target_h < 30
        ):
            bad.append(row)
    print("bad_fields=%d/%d" % (len(bad), len(aim)))

    frame_deltas = []
    for previous, current in zip(aim, aim[1:]):
        delta = (int(current[5]) - int(previous[5])) & 0xFF
        if 0 < delta < 50:
            frame_deltas.append(delta)
    duration_ms = int(aim[-1][0]) - int(aim[0][0]) if len(aim) > 1 else 0
    frames = sum(frame_deltas)
    print("vision_fps=%.2f duration=%.2fs frames=%d" % (
        frames * 1000.0 / max(1, duration_ms), duration_ms / 1000.0, frames
    ))
    all_deltas = []
    for previous, current in zip(rows, rows[1:]):
        delta = (int(current[5]) - int(previous[5])) & 0xFF
        if 0 < delta < 50:
            all_deltas.append(delta)
    all_duration_ms = int(rows[-1][0]) - int(rows[0][0])
    print("vision_fps_all=%.2f duration=%.2fs frames=%d" % (
        sum(all_deltas) * 1000.0 / max(1, all_duration_ms),
        all_duration_ms / 1000.0, sum(all_deltas)
    ))

    print("chassis_valid=%d raw_nonzero=%d good_nonzero=%d "
          "version_errors=%d speed_nonzero=%d geomFF_nonzero=%d "
          "gzFF_nonzero=%d" % (
              sum(row[36] == "1" for row in rows),
              sum(int(row[50]) > 0 for row in rows),
              sum(int(row[40]) > 0 for row in rows),
              sum(int(row[51]) > 0 for row in rows),
              sum(int(row[38]) != 0 for row in rows),
              sum(float(row[45]) != 0.0 for row in rows),
              sum(float(row[46]) != 0.0 for row in rows),
          ))

    vision_rows = [row for row in rows if len(row) >= 55]
    if vision_rows:
        first = vision_rows[0]
        last = vision_rows[-1]
        print("vision counters first: raw=%s lines=%s crc=%s" % (
            first[52], first[53], first[54]
        ))
        print("vision counters last: raw=%s lines=%s crc=%s" % (
            last[52], last[53], last[54]
        ))
    extended_rows = [row for row in rows if len(row) >= 60]
    if extended_rows:
        first = extended_rows[0]
        last = extended_rows[-1]
        print(
            "route guard first: zone=%s corner=%s hold=%s "
            "pitch_guard=%s yaw_recovery=%s" % (
                first[55], first[56], first[57], first[58], first[59]
            )
        )
        print(
            "route guard last: zone=%s corner=%s hold=%s "
            "pitch_guard=%s yaw_recovery=%s" % (
                last[55], last[56], last[57], last[58], last[59]
            )
        )
    pose_rows = [row for row in rows if len(row) >= 62]
    if pose_rows:
        aim_pose = [
            row for row in pose_rows
            if row[1] == "2" and row[2] == "1" and row[61] == "1"
        ]
        if aim_pose:
            stat("poseRangeMs", [row[60] for row in aim_pose])
            print("pose_distance_valid=%d/%d" % (
                len(aim_pose),
                sum(row[1] == "2" and row[2] == "1" for row in pose_rows),
            ))
    timing_rows = [row for row in rows if len(row) >= 64]
    if timing_rows:
        aim_timing = [
            row for row in timing_rows if row[1] == "2" and row[2] == "1"
        ]
        if aim_timing:
            stat("alphaBetaRate", [row[62] for row in aim_timing])
            stat("targetVxPxS", [row[63] for row in aim_timing])
    chassis_age_rows = [row for row in rows if len(row) >= 65]
    if chassis_age_rows:
        stat("chassisFrameAgeMs", [
            row[64] for row in chassis_age_rows if row[36] == "1"
        ])
    timestamp_rows = [row for row in rows if len(row) >= 69]
    if timestamp_rows:
        chassis_timestamp_rows = [
            row for row in timestamp_rows if row[36] == "1"
        ]
        if chassis_timestamp_rows:
            stat("alphaBetaFeedforward", [
                row[65] for row in chassis_timestamp_rows
            ])
            stat("chassisSampleAgeRaw", [
                row[67] for row in chassis_timestamp_rows
            ])
            stat("chassisSampleAgeRelative", [
                row[68] for row in chassis_timestamp_rows
            ])
    reason_rows = [row for row in rows if len(row) >= 70]
    if reason_rows:
        print("vision lost reasons=%s" % {
            code: sum(row[69] == code for row in reason_rows)
            for code in sorted(set(row[69] for row in reason_rows))
        })
    segments = []
    for row in rows:
        if not segments or int(row[0]) < int(segments[-1][-1][0]):
            segments.append([])
        segments[-1].append(row)
    for index, segment in enumerate(segments):
        first = segment[0]
        last = segment[-1]
        duration_ms = int(last[0]) - int(first[0])
        if len(first) >= 55 and len(last) >= 55 and duration_ms > 0:
            raw_delta = int(last[52]) - int(first[52])
            line_delta = int(last[53]) - int(first[53])
            crc_delta = int(last[54]) - int(first[54])
            print(
                "segment[%d] duration=%.2fs raw_delta=%s line_delta=%s "
                "crc_delta=%s raw_rate=%.0fB/s line_rate=%.1f/s" % (
                    index, duration_ms / 1000.0, raw_delta, line_delta,
                    crc_delta, raw_delta * 1000.0 / duration_ms,
                    line_delta * 1000.0 / duration_ms,
                )
            )

    spikes = [
        row for row in aim
        if abs(float(row[11])) > 0.8 or abs(float(row[12])) > 0.8 or
        abs(float(row[21])) > 8.0 or abs(float(row[26])) > 4.0
    ]
    print("spikes=%d" % len(spikes))
    for row in spikes[:80]:
        print(
            "t=%s age=%s seq=%s pxT=(%s,%s) pxL=(%s,%s) r=%s "
            "yawE=%s pitchE=%s gyro=%s yawCmd=%s pitchFb=%s "
            "pitchTgt=%s pitchCmd=%s" % (
                row[0], row[4], row[5], row[6], row[7], row[8], row[9],
                row[10], row[11], row[12], row[20], row[21], row[24],
                row[25], row[26],
            )
        )


if __name__ == "__main__":
    main()
