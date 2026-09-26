#!/usr/bin/env python3
"""标定"光斑位置 - 靶纸距离"表（视差标定，光斑检不到时的兜底精度全靠它）。

用法：
    python3 tools/calib_boresight_table.py --source 0 --port /dev/ttyS1

为什么需要这张表：
    激光和相机不同轴（你的激光装在镜头【下面】），所以光斑在画面里的
    像素位置会随靶纸距离变化：

        光斑方向角 ≈ 两轴夹角 + 两轴平移 / 距离

    只标一个固定点，在标定距离上很准，换一个距离就可能偏几十像素
    —— 这正是"标定一个点到底够不够"这个问题的答案：不够，除非两轴
    完全重合（物理上做不到）。

    做法很土但有效：把靶纸摆在 2~3 个距离上，各测一次光斑像素位置，
    运行时用靶纸单应矩阵算出的距离去查表插值。不用额外买测距模块。

标定完写进 configs/boresight.yaml（程序启动会自动叠加这个文件）。

注意：光斑能正常检到时，始终优先用实测光斑；这张表只在光斑检不到
（反光、过曝、IR-cut 把激光滤掉）时兜底。
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim import protocol as proto  # noqa: E402
from eaim.app import AimApp, RunOptions  # noqa: E402
from eaim.config import (find_default_config, load_config, read_yaml,  # noqa: E402
                         save_yaml)


def measure(app, seconds=2.5):
    """测这一段里的光斑像素位置（中位数）和靶纸距离（中位数）。"""
    t_end = time.monotonic() + seconds
    us, vs, ds = [], [], []
    n_spot = n_target = 0
    while time.monotonic() < t_end:
        f, _ = app.camera.read()
        if f is not None:
            # 保持 AIM 模式但不给偏置：云台稳稳指着，别乱动
            app.gimbal_link.send(proto.MsgId.AIM,
                                 proto.pack_aim(0.0, 0.0,
                                                proto.AimFlags.LASER_ON, 0))
            tgt = app.detector.detect(f.image)
            sp = app.spot_detector.detect(f.image)
            if tgt is not None:
                n_target += 1
                if tgt.distance_m > 0.05:
                    ds.append(tgt.distance_m)
            if sp is not None:
                n_spot += 1
                us.append(sp.uv[0])
                vs.append(sp.uv[1])
        time.sleep(0.002)
    out = {
        "spot": (float(np.median(us)), float(np.median(vs))) if us else None,
        "dist": float(np.median(ds)) if ds else None,
        "n_spot": n_spot, "n_target": n_target,
    }
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0")
    ap.add_argument("--port", default=None, help="云台串口（不给就不开云台）")
    ap.add_argument("--distances", default="0.5,0.8,1.2,1.6",
                    help="要标定的距离（米，逗号分隔）")
    ap.add_argument("--seconds", type=float, default=2.5, help="每个距离测多久")
    ap.add_argument("--no-gimbal", action="store_true")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    cfg = load_config(find_default_config())
    if args.port:
        cfg.link_gimbal.port = args.port
    cfg.link_gimbal.enable = not args.no_gimbal
    cfg.link_car.enable = False
    cfg.app.show_window = False
    # 标定时把门控放宽：不同距离光斑位置本来就会跑，门控太紧会什么都检不到
    cfg.laser.gate_enable = False

    app = AimApp(cfg, RunOptions(laser=True, quiet=True))
    if not app.setup():
        return 1
    app.gimbal_link.send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.AIM))
    time.sleep(0.4)

    print("=" * 68)
    print(" 光斑-距离标定：一共 %d 个距离点" % len(args.distances.split(",")))
    print(" 每个距离：把靶纸摆好 -> 按回车 -> 保持不动 %.1f 秒" % args.seconds)
    print(" 要求：激光光斑在画面里能看见（跑不过就先调 laser 阈值）")
    print("=" * 68)

    table = []
    for token in args.distances.split(","):
        token = token.strip()
        if not token:
            continue
        nominal = float(token)
        print()
        print("── 请把靶纸摆到【%.2f m】处（卷尺量到纸面），激光要打在纸上" % nominal)
        try:
            input("   摆好后按回车开始测量…")
        except EOFError:
            break
        m = measure(app, args.seconds)
        if m["spot"] is None:
            print("   ✘ 这个距离没检到光斑，跳过（先确认激光亮点在画面里）")
            continue
        dist = m["dist"] if m["dist"] else nominal
        if m["dist"] and abs(m["dist"] - nominal) > 0.25:
            print("   ⚠ 注意：单应矩阵算出的距离是 %.2fm，和你说的 %.2fm 差得有点多，"
                  "以程序算的 %.2fm 为准（运行时用的也是它）"
                  % (m["dist"], nominal, dist))
        print("   ✔ 光斑=(%.1f, %.1f)  距离=%.2fm  (光斑 %d 帧 / 靶纸 %d 帧)"
              % (m["spot"][0], m["spot"][1], dist, m["n_spot"], m["n_target"]))
        table.append([round(dist, 3), round(m["spot"][0], 1), round(m["spot"][1], 1)])

    app.gimbal_link.send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.STAB))
    app.shutdown()

    if len(table) < 2:
        print("\n✘ 有效点少于 2 个，写出来也没有意义，不保存。")
        return 1

    table.sort(key=lambda t: t[0])
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = args.out or os.path.join(base_dir, "configs", "boresight.yaml")
    data = {}
    if os.path.exists(out):
        try:
            data = read_yaml(out) or {}
        except Exception:                    # noqa: BLE001
            data = {}
    data.setdefault("calib", {})
    data["calib"]["boresight_table"] = table
    # 单点也顺手写一个（取中间那个距离的点），老路径也能用
    mid = table[len(table) // 2]
    data["calib"].setdefault("boresight_uv", [mid[1], mid[2]])
    save_yaml(out, data)

    print()
    print("=" * 68)
    print("已写入 %s" % out)
    for row in table:
        print("   %.2f m -> 光斑 (%.1f, %.1f)" % (row[0], row[1], row[2]))
    print("接下来正常跑 main.py run 就行（会自动叠加这个文件）。")
    print("=" * 68)
    return 0


if __name__ == "__main__":
    sys.exit(main())
