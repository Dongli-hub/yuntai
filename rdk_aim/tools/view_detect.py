#!/usr/bin/env python3
"""可视化调试：实时看靶纸检测、光斑检测、误差。

用法：
    python tools/view_detect.py --source 0                    # 有显示器：实时窗口
    python tools/view_detect.py --source 0 --snapshot out/dbg --interval 1.0
                                                              # 无显示器：存图

按键（窗口模式）：
    q/ESC 退出
    r     切换 target.require_red（用空白 A4 调试时关掉它）
    b     在 laser.b_minus_others 的 20/35/60 之间切换
    s     存一张当前标注帧
"""

import argparse
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim import protocol as proto  # noqa: E402
from eaim import viz  # noqa: E402
from eaim.app import AimApp, RunOptions  # noqa: E402
from eaim.config import find_default_config, load_config  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0")
    ap.add_argument("--port-gimbal", default=None)
    ap.add_argument("--no-gimbal", action="store_true")
    ap.add_argument("--laser", action="store_true", help="打开激光（测光斑）")
    ap.add_argument("--snapshot", default=None, help="无显示器时存图目录")
    ap.add_argument("--interval", type=float, default=1.0, help="存图间隔 s")
    ap.add_argument("--duration", type=float, default=0.0, help="运行多久后退出，0=一直")
    args = ap.parse_args()

    cfg = load_config(find_default_config())
    if args.port_gimbal:
        cfg.link_gimbal.port = args.port_gimbal
    cfg.link_gimbal.enable = not args.no_gimbal
    cfg.link_car.enable = False
    cfg.app.show_window = args.snapshot is None
    cfg.app.record_video = False

    app = AimApp(cfg, RunOptions(laser=args.laser, quiet=True))
    if not app.setup():
        return 1
    if args.snapshot:
        os.makedirs(args.snapshot, exist_ok=True)
    print("调试中：q/ESC 退出，r 切换 require_red，b 切蓝优势门限，s 存图")
    last_dump = 0.0
    t_end = time.monotonic() + args.duration if args.duration > 0 else None
    show = cfg.app.show_window
    while True:
        if t_end is not None and time.monotonic() >= t_end:
            break
        f, _ = app.camera.read()
        if f is None:
            time.sleep(0.002)
            continue
        app._process_vision(f.image, f.t)
        if app.gimbal_link is not None and args.laser:
            app.gimbal_link.send(
                proto.MsgId.AIM,
                proto.pack_aim(app.aim.yaw_deg, app.aim.pitch_deg,
                               proto.AimFlags.LASER_ON, 0))
        lines = []
        if app.target is not None:
            lines.append("conf=%.2f ring=%.2f dist=%.2fm red=%d"
                         % (app.target.confidence, app.target.ring_score,
                            app.target.distance_m, app.target.red_px))
        else:
            lines.append("靶纸：未检出")
        lines.append("require_red=%s  b_minus=%d"
                     % (cfg.target.require_red, cfg.laser.b_minus_others))
        vis = viz.draw_overlay(f.image.copy(), target=app.target, spot=app.spot,
                               lines=lines, state="VIEW", fps=app.camera.fps_measured)
        if app.target is not None and app.target.rect is not None:
            rect = cv2.warpPerspective(f.image, app.target.rect.h_img2rect,
                                       app.target.rect.size)
            vis = viz.draw_rectified_inset(vis, rect, max_w=200)
        if show:
            cv2.imshow("view_detect", vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("r"):
                cfg.target.require_red = not cfg.target.require_red
            if key == ord("b"):
                seq = [20, 35, 60]
                i = seq.index(cfg.laser.b_minus_others) if \
                    cfg.laser.b_minus_others in seq else 1
                cfg.laser.b_minus_others = seq[(i + 1) % len(seq)]
            if key == ord("s"):
                cv2.imwrite("view_%d.jpg" % int(time.time()), vis)
        elif args.snapshot and time.monotonic() - last_dump >= args.interval:
            last_dump = time.monotonic()
            p = os.path.join(args.snapshot, "view_%05d.jpg"
                             % int(time.monotonic() - app.t0))
            cv2.imwrite(p, vis)
            print("存图 %s  靶纸=%s 光斑=%s"
                  % (p, "有" if app.target else "无", app.spot.uv if app.spot else "无"))
    if show:
        cv2.destroyAllWindows()
    app.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

