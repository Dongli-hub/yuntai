#!/usr/bin/env python3
"""检查"激光光斑在这个相机里到底能不能被检出来"，并顺便标定光轴点。

用法：
    python tools/check_laser.py --source 0 --port /dev/ttyUSB0
    python tools/check_laser.py --source 0 --no-gimbal     # 云台手动对着靶面

输出：
  * 光斑检出率
  * 光斑平均像素位置（**这就是 calib.boresight_uv，标定一次长期有效**）
  * 蓝优势统计（用来判断 laser.b_min / laser.b_minus_others 该设多少）

如果检出率很低：
  1) 先把 laser.b_minus_others 从 35 降到 20、laser.b_min 从 150 降到 100
  2) 还不行就是 IR-cut 把 405nm 滤掉了 -> 用 calib.boresight_uv 兜底，
     程序会自动走"光轴点"路径（见 docs/03 风险 R3）
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim.config import find_default_config, load_config, save_yaml  # noqa: E402
from eaim import protocol as proto  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--port", default=None, help="云台串口（不填则不开激光）")
    ap.add_argument("--no-gimbal", action="store_true")
    ap.add_argument("--duration", type=float, default=6.0)
    ap.add_argument("--save", action="store_true", help="把光轴点写进 configs/boresight.yaml")
    args = ap.parse_args()

    cfg = load_config(find_default_config())
    if args.port:
        cfg.link_gimbal.port = args.port
    cfg.link_gimbal.enable = not args.no_gimbal

    from eaim.app import AimApp, RunOptions

    app = AimApp(cfg, RunOptions(duration_s=args.duration, laser=True, quiet=True))
    if not app.setup():
        return 1
    print("=" * 60)
    print("激光光斑自检：%0.1fs。请让云台指向靶面（或任意白墙）" % args.duration)
    print("=" * 60)
    frames = hits = 0
    us, vs, advs, bs = [], [], [], []
    t_end = time.monotonic() + args.duration
    last = 0.0
    while time.monotonic() < t_end:
        f, _ = app.camera.read()
        if f is not None:
            frames += 1
            app.gimbal_link.send(proto.MsgId.AIM,
                                 proto.pack_aim(0.0, 0.0, proto.AimFlags.LASER_ON, 0))
            sp = app.spot_detector.detect(f.image)
            b, g, r = cv2.split(f.image)
            adv = b.astype(np.int16) - np.maximum(g, r).astype(np.int16)
            advs.append(float(adv.max()))
            bs.append(float(b.max()))
            if sp is not None:
                hits += 1
                us.append(sp.uv[0])
                vs.append(sp.uv[1])
        now = time.monotonic()
        if now - last > 1.0:
            last = now
            print("  帧=%d 光斑=%d 最大蓝优势=%.0f 最大B=%.0f"
                  % (frames, hits, advs[-1] if advs else 0, bs[-1] if bs else 0))
    rate = 100.0 * hits / max(1, frames)
    print("-" * 60)
    print("光斑检出率 = %.0f%%（%d/%d）" % (rate, hits, frames))
    if advs:
        a = np.array(advs)
        print("蓝优势 max 的统计：中位 %.0f P10 %.0f 最大 %.0f"
              % (np.median(a), np.percentile(a, 10), a.max()))
    if hits:
        u, v = float(np.median(us)), float(np.median(vs))
        print("光斑平均位置 = (%.1f, %.1f)  <- calib.boresight_uv" % (u, v))
        print("检测抖动（标准差）= (%.2f, %.2f) px"
              % (float(np.std(us)), float(np.std(vs))))
        if args.save:
            base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            p = os.path.join(base, "configs", "boresight.yaml")
            save_yaml(p, {"calib": {"boresight_uv": [round(u, 1), round(v, 1)]}})
            print("已写入 %s" % p)
            print("用法：python main.py run --extra-config configs/boresight.yaml")
    else:
        print("检测不到光斑。按 docs/03 风险 R3 的两步走：")
        print("  1) laser.b_min -> 100, laser.b_minus_others -> 20 再试")
        print("  2) 仍不行则用 calib.boresight_uv 兜底（同轴 => 它是常数）")
    app.shutdown()
    return 0 if rate >= 50.0 else 2


if __name__ == "__main__":
    sys.exit(main())
