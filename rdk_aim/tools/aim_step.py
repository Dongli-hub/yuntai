#!/usr/bin/env python3
"""分步瞄准（开环 + 每步实测修正）：验证"偏置→图像"模型，并做定点打靶。

为什么需要它：
    视觉闭环（main.py run）依赖"偏置变化 → 图像变化"的模型足够准。
    实测发现 yaw 轴的**命令→实际转角有 2 倍放大**（陀螺姿态积分量只有
    真实的一半，见 docs/05 里 Mahony 采样频率那条），再加上 yaw 轴本身
    响应慢（0.5s 级延迟），闭环增益一高就在限幅之间打摆。
    这个工具改用"一步一测"的方式：

        测当前误差 -> 按实测增益算出需要的偏置 -> 发下去 -> 等它稳定 -> 再测

    每一步只走一小段并且等稳定，所以不会振荡；即使模型有 30% 误差，
    迭代几步也能把误差压到很小。适合"定点打靶"这种目标不动的场合。

用法：
    python3 tools/aim_step.py --port /dev/ttyS1 --source 0
    python3 tools/aim_step.py --port /dev/ttyS1 --steps 4 --yaw-gain 22 --pitch-gain 18

增益默认值来自 2026-09-27 实测：
    yaw   6° 偏置 -> 靶心横移 133px  => 22 px/°
    pitch 6° 偏置 -> 靶心纵移 110px  => 18 px/°
（如果换了距离/装了别的镜头，用 tools/calib_boresight.py 重新测。）
"""

import argparse
import math
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim import protocol as proto  # noqa: E402
from eaim.app import AimApp, RunOptions  # noqa: E402
from eaim.config import find_default_config, load_config  # noqa: E402


def measure(app, seconds=1.6, scale=1.0):
    """测一段时间内的靶心/光斑中位位置。"""
    t_end = time.monotonic() + seconds
    tus, tvs, sus, svs = [], [], [], []
    while time.monotonic() < t_end:
        f, _ = app.camera.read()
        if f is not None:
            tgt = app.detector.detect(f.image, scale)
            sp = app.spot_detector.detect(f.image, scale)
            if tgt is not None:
                tus.append(tgt.uv[0])
                tvs.append(tgt.uv[1])
            if sp is not None:
                sus.append(sp.uv[0])
                svs.append(sp.uv[1])
        time.sleep(0.002)
    return {
        "target": (float(np.median(tus)), float(np.median(tvs))) if tus else None,
        "spot": (float(np.median(sus)), float(np.median(svs))) if svs else None,
        "n_t": len(tus), "n_s": len(sus),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0")
    ap.add_argument("--port", default="/dev/ttyS1")
    ap.add_argument("--steps", type=int, default=3, help="迭代几步")
    ap.add_argument("--yaw-gain", type=float, default=22.0, help="px/°（实测）")
    ap.add_argument("--pitch-gain", type=float, default=18.0, help="px/°（实测）")
    ap.add_argument("--settle", type=float, default=1.8, help="每步等多久再测")
    ap.add_argument("--limit", type=float, default=30.0, help="偏置限幅（度）")
    ap.add_argument("--pre-offset", default=None,
                    help="先给一个初始偏置，格式 yaw,pitch（例：9,4）。"
                         "靶纸不在画面中间、第一次根本检不到时用它先把云台转过去")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="检测缩放：1.0=全分辨率（默认，准），0.5=和主程序一样快")
    args = ap.parse_args()

    cfg = load_config(find_default_config())
    cfg.link_gimbal.port = args.port
    cfg.link_gimbal.enable = True
    cfg.link_car.enable = False
    cfg.laser.roi_px = 0            # 测量工具：全图搜索，宁可慢也要找得到
    cfg.app.show_window = False

    app = AimApp(cfg, RunOptions(laser=True, quiet=True))
    if not app.setup():
        return 1

    _keep = threading.Event()
    state = {"yaw": 0.0, "pitch": 0.0}

    def _keepalive():
        n = 0
        while not _keep.is_set():
            try:
                # ⚠ 偏置必须【持续】发：H723 侧 0.5s 收不到 AIM 帧就会切 STAB
                # 并把偏置清零（看门狗）。只发 MODE=AIM 是不够的 ——
                # 踩过这个坑：偏置累加到 26°，云台纹丝不动。
                app.gimbal_link.send(
                    proto.MsgId.AIM,
                    proto.pack_aim(state["yaw"], state["pitch"],
                                   proto.AimFlags.LASER_ON, 0))
                if n % 5 == 0:
                    app.gimbal_link.send(proto.MsgId.MODE,
                                         proto.pack_mode(proto.AimMode.AIM))
            except Exception:                                   # noqa: BLE001
                pass
            n += 1
            _keep.wait(0.1)

    threading.Thread(target=_keepalive, daemon=True).start()

    yaw = pitch = 0.0
    if args.pre_offset:
        try:
            yaw, pitch = (float(v) for v in args.pre_offset.split(","))
            print("先给初始偏置 (%.1f, %.1f)°" % (yaw, pitch))
        except Exception:                                     # noqa: BLE001
            print("--pre-offset 格式错，应为 yaw,pitch（例：9,4）")
            return 1
    print("=" * 70)
    print(" 分步瞄准：yaw %.1f px/°  pitch %.1f px/°  最多 %d 步"
          % (args.yaw_gain, args.pitch_gain, args.steps))
    print("=" * 70)

    try:
        for k in range(args.steps + 1):
            state["yaw"] = yaw
            state["pitch"] = pitch
            app.aim.set_offset(yaw, pitch)
            time.sleep(args.settle)
            m = measure(app, scale=args.scale)
            if m["target"] is None or m["spot"] is None:
                print("  第 %d 步：靶心=%s 光斑=%s —— 有一个没检到，重试"
                      % (k, m["target"] is not None, m["spot"] is not None))
                continue
            tu, tv = m["target"]
            su, sv = m["spot"]
            eu, ev = tu - su, tv - sv
            err = math.hypot(eu, ev)
            print("  第 %d 步：off=(%+6.2f,%+6.2f)  靶心=(%.0f,%.0f) 光斑=(%.0f,%.0f)  "
                  "误差=(%+.1f,%+.1f) |e|=%.1fpx" % (k, yaw, pitch, tu, tv, su, sv, eu, ev, err))
            if err < 8.0:
                print("  ✔ 已打中（<8px）")
                break
            if k == args.steps:
                break
            # 下一步要走的偏置增量：误差要反向抵消（偏置 +1° -> 靶心 +gain px）
            dyaw = -eu / args.yaw_gain
            dpitch = -ev / args.pitch_gain
            yaw = max(-args.limit, min(args.limit, yaw + dyaw))
            pitch = max(-args.limit, min(args.limit, pitch + dpitch))
            print("        -> 需要补偿 dyaw=%+.2f° dpitch=%+.2f°  =>  新的偏置 (%.2f, %.2f)"
                  % (dyaw, dpitch, yaw, pitch))
    except KeyboardInterrupt:
        print("\nCtrl-C")
    finally:
        _keep.set()
        app.gimbal_link.send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.STAB))
        time.sleep(0.1)
        app.shutdown()
    print("结束，已切回 STAB。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
