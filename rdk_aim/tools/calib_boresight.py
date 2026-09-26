#!/usr/bin/env python3
"""标定"光轴点" + 自动判定 yaw/pitch 偏置的正负号。

用法：
    python tools/calib_boresight.py --port-gimbal /dev/ttyUSB0 --source 0
    python tools/calib_boresight.py --source 0 --no-gimbal    # 云台手动对准靶面

它做三件事（每一步都有打印，看清哪步失败）：

  1. **光轴点**：激光与相机同轴 => 光斑在图像里位置固定。
     实时测 5s 取中位数，就是这个常数。写进 calib.boresight_uv 后，
     即使激光关着（发挥部分 3 的激光门控）也能准确瞄准。

  2. **偏置符号 sign_yaw / sign_pitch**：
     光斑不动、靶心动，所以判符号必须看**靶心**怎么动：
     给 yaw 偏置 +step 度，靶心像素往左走（u 减小）=> sign_yaw = +1。
     符号判错的话云台会"越修越偏"，直接失控，所以这一步不能省。

  3. **实测每度多少像素**：应约等于 fx*pi/180。
     如果差得远，说明 fx 标定有问题或相机分辨率与配置不符。
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim import protocol as proto  # noqa: E402
from eaim.app import AimApp, RunOptions  # noqa: E402
from eaim.config import find_default_config, load_config, save_yaml  # noqa: E402


def measure(app, tag: str, seconds: float = 1.2):
    """在 seconds 秒内统计靶心与光斑的中位位置。"""
    t_end = time.monotonic() + seconds
    tus, tvs, sus, svs = [], [], [], []
    while time.monotonic() < t_end:
        f, _ = app.camera.read()
        if f is not None:
            app.gimbal_link.send(
                proto.MsgId.AIM,
                proto.pack_aim(app.aim.yaw_deg, app.aim.pitch_deg,
                               proto.AimFlags.LASER_ON, 0))
            tgt = app.detector.detect(f.image)
            sp = app.spot_detector.detect(f.image)
            if tgt is not None:
                tus.append(tgt.uv[0])
                tvs.append(tgt.uv[1])
            if sp is not None:
                sus.append(sp.uv[0])
                svs.append(sp.uv[1])
        time.sleep(0.002)
    out = {
        "target": (float(np.median(tus)), float(np.median(tvs))) if tus else None,
        "spot": (float(np.median(sus)), float(np.median(svs))) if svs else None,
        "n_target": len(tus), "n_spot": len(sus),
    }
    print("  [%s] 靶心=%s (n=%d)  光斑=%s (n=%d)"
          % (tag, _fmt(out["target"]), out["n_target"],
             _fmt(out["spot"]), out["n_spot"]))
    return out


def _fmt(p):
    return "无" if p is None else "(%.1f, %.1f)" % p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0")
    ap.add_argument("--port-gimbal", default=None)
    ap.add_argument("--no-gimbal", action="store_true")
    ap.add_argument("--step", type=float, default=6.0, help="判符号用的偏置步长（度）")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    cfg = load_config(find_default_config())
    if args.port_gimbal:
        cfg.link_gimbal.port = args.port_gimbal
    cfg.link_gimbal.enable = not args.no_gimbal
    cfg.link_car.enable = False
    cfg.app.show_window = False

    app = AimApp(cfg, RunOptions(laser=True, quiet=True))
    if not app.setup():
        return 1
    # 直接进 AIM 模式（跳过状态机），只做标定
    app.gimbal_link.send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.AIM))
    time.sleep(0.3)

    print("=" * 64)
    print("第 1 步：标定光轴点（请让激光打在靶纸上，画面里能看到靶纸）")
    print("=" * 64)
    base = measure(app, "静置", 3.0)
    if base["spot"] is None:
        print("检测不到光斑 —— 先跑 tools/check_laser.py 调阈值，或接受用光轴点兜底")
        boresight = cfg.calib.boresight_uv
    else:
        boresight = base["spot"]
        print("光轴点 = (%.1f, %.1f)" % boresight)

    sign_yaw = cfg.calib.sign_yaw
    sign_pitch = cfg.calib.sign_pitch
    pxu = pxv = 0.0
    if base["target"] is None:
        print("\n画面里没检测到靶纸 —— 无法判定符号。"
              "请把靶纸放进画面（可以临时只放一张贴了黑框的 A4）后重跑。")
    else:
        print("\n" + "=" * 64)
        print("第 2 步：给 yaw 偏置 +%.1f 度，看靶心往哪边跑" % args.step)
        print("=" * 64)
        app.aim.set_offset(0.0, 0.0)
        time.sleep(0.6)
        a = measure(app, "yaw=0")
        app.aim.set_offset(args.step, 0.0)
        time.sleep(0.8)
        b = measure(app, "yaw=+%.0f" % args.step)
        app.aim.set_offset(0.0, 0.0)
        if a["target"] and b["target"]:
            du = b["target"][0] - a["target"][0]
            pxu = abs(du) / args.step
            sign_yaw = -1.0 if du > 0 else 1.0
            print("  -> du/dyaw = %+.1f px/%.0f° = %+.2f px/°   sign_yaw = %+.0f"
                  % (du, args.step, du / args.step, sign_yaw))
        print("\n" + "=" * 64)
        print("第 3 步：给 pitch 偏置 +%.1f 度，看靶心往哪边跑" % args.step)
        print("=" * 64)
        time.sleep(0.5)
        a = measure(app, "pitch=0")
        app.aim.set_offset(0.0, args.step)
        time.sleep(0.8)
        b = measure(app, "pitch=+%.0f" % args.step)
        app.aim.set_offset(0.0, 0.0)
        if a["target"] and b["target"]:
            dv = b["target"][1] - a["target"][1]
            pxv = abs(dv) / args.step
            sign_pitch = -1.0 if dv > 0 else 1.0
            print("  -> dv/dpitch = %+.1f px/%.0f° = %+.2f px/°   sign_pitch = %+.0f"
                  % (dv, args.step, dv / args.step, sign_pitch))
        fx_deg = cfg.calib.fx * np.pi / 180.0
        fy_deg = cfg.calib.fy * np.pi / 180.0
        print("\n理论增益：fx 方向 %.2f px/°，fy 方向 %.2f px/°" % (fx_deg, fy_deg))
        if pxu > 0 and abs(pxu - fx_deg) / fx_deg > 0.25:
            print("警告：yaw 实测与理论差 %.0f%%，检查 fx 标定 / 分辨率 / 是否加了数字变焦"
                  % (100 * abs(pxu - fx_deg) / fx_deg))
        if pxv > 0 and abs(pxv - fy_deg) / fy_deg > 0.25:
            print("警告：pitch 实测与理论差 %.0f%%" % (100 * abs(pxv - fy_deg) / fy_deg))

    print("\n" + "=" * 64)
    print("结果")
    print("=" * 64)
    print("boresight_uv: %s" % (_fmt(boresight)))
    print("sign_yaw: %+.1f    sign_pitch: %+.1f" % (sign_yaw, sign_pitch))
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = args.out or os.path.join(base_dir, "configs", "boresight.yaml")
    data = {"calib": {"sign_yaw": float(sign_yaw), "sign_pitch": float(sign_pitch)}}
    if boresight:
        data["calib"]["boresight_uv"] = [round(float(boresight[0]), 1),
                                         round(float(boresight[1]), 1)]
    save_yaml(out, data)
    print("已写入 %s" % out)
    print("运行：python main.py run --extra-config configs/boresight.yaml")
    app.gimbal_link.send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.STAB))
    app.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

