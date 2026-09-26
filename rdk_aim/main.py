#!/usr/bin/env python3
"""rdk_aim 入口。

子命令：
  run     真机运行（相机 + 两条串口）
  sim     PC 全闭环仿真（无硬件）
  check   只做检测，打印靶纸/光斑的检出率
  unwind  解绕（跑完一轮后必做，见 docs/03 风险 R2）
  replay  对录像/图像做离线检测统计

示例：
  python main.py sim --duration 12 --video out/sim.mp4
  python main.py run --set control.kp=0.4 --port-gimbal /dev/ttyUSB0
  python main.py run --draw --laps 1 --start-car
  python main.py check --source 0 --duration 8
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from eaim.app import AimApp, RunOptions          # noqa: E402
from eaim.config import (find_calibration_overlays, find_config,  # noqa: E402
                         load_config)


def _parse_set(pairs):
    out = {}
    for item in pairs or []:
        if "=" not in item:
            raise SystemExit("--set 需要 key=value 形式: %s" % item)
        key, _, value = item.partition("=")
        out[key.strip()] = value.strip()
    return out


def _base_config(args):
    extra = getattr(args, "extra_config", None)
    # 自动叠加标定文件（configs/intrinsic.yaml、configs/boresight.yaml，存在才用），
    # 这样日常启动就一条 `python3 main.py run`，不用手动带 --extra-config
    extras = find_calibration_overlays()
    if extra:
        extras.append(extra)
    cfg = load_config(args.config, extra_path=extras,
                      overrides=_parse_set(getattr(args, "set", None)))
    if getattr(args, "source", None) is not None:
        cfg.camera.source = args.source
    if getattr(args, "device", None) is not None:
        cfg.camera.device = args.device
    if getattr(args, "port_gimbal", None):
        cfg.link_gimbal.port = args.port_gimbal
    if getattr(args, "port_car", None):
        cfg.link_car.port = args.port_car
    if getattr(args, "baud_gimbal", None):
        cfg.link_gimbal.baud = args.baud_gimbal
    if getattr(args, "baud_car", None):
        cfg.link_car.baud = args.baud_car
    if getattr(args, "no_gimbal", False):
        cfg.link_gimbal.enable = False
    if getattr(args, "no_car", False):
        cfg.link_car.enable = False
    if getattr(args, "video", False):
        cfg.app.record_video = True
    if getattr(args, "window", False):
        cfg.app.show_window = True
    if getattr(args, "snapshot", None):
        cfg.app.snapshot_every_s = args.snapshot
    if getattr(args, "out", None):
        cfg.app.run_dir = args.out
    return cfg


def _add_common(p):
    p.add_argument("--config", default=None, help="主配置文件（默认 configs/default.yaml）")
    p.add_argument("--extra-config", dest="extra_config", default=None,
                   help="叠加配置文件（例如 configs/intrinsic.yaml）")
    p.add_argument("--set", action="append", default=[],
                   help="覆盖配置，例如 --set control.kp=0.4（可多次）")
    p.add_argument("--source", default=None, help="相机来源: uvc|mipi|file|sim")
    p.add_argument("--device", default=None, help="相机索引或文件路径")
    p.add_argument("--port-gimbal", default=None, help="云台串口")
    p.add_argument("--port-car", default=None, help="小车串口")
    p.add_argument("--baud-gimbal", type=int, default=None)
    p.add_argument("--baud-car", type=int, default=None)
    p.add_argument("--no-gimbal", action="store_true", help="不打开云台链路")
    p.add_argument("--no-car", action="store_true", help="不打开小车链路")
    p.add_argument("--video", action="store_true", help="录制标注视频")
    p.add_argument("--window", action="store_true", help="显示实时画面（需要显示器）")
    p.add_argument("--snapshot", type=float, default=None,
                   help="每 N 秒存一张标注帧 jpg")
    p.add_argument("--out", default=None, help="输出目录")


def cmd_run(args):
    cfg = _base_config(args)
    opt = RunOptions(duration_s=args.duration, draw=args.draw, laps=args.laps,
                     auto_start_car=args.start_car, laser=not args.no_laser,
                     budget_s=args.budget, start_corner=args.car_corner,
                     quiet=args.quiet)
    app = AimApp(cfg, opt)
    return app.run()


def cmd_sim(args):
    # sim.yaml 是叠加在 default.yaml 之上的覆盖层，不是完整配置
    if not args.extra_config:
        args.extra_config = find_config("sim") or None
    cfg = _base_config(args)
    cfg.camera.source = "sim"
    cfg.camera.width = args.width
    cfg.camera.height = args.height
    cfg.camera.fps = args.fps
    cfg.sim.car_enable = not args.handheld
    cfg.sim.car_lap_time_s = args.lap_time
    cfg.sim.car_start_corner = args.car_corner
    cfg.sim.distance_m = args.distance
    cfg.app.record_video = args.video
    from eaim.sim import SimWorld

    world = SimWorld(cfg)
    opt = RunOptions(duration_s=args.duration, draw=args.draw, laps=args.laps,
                     auto_start_car=args.start_car or not args.handheld,
                     laser=not args.no_laser, budget_s=args.budget,
                     start_corner=args.car_corner, quiet=args.quiet)
    app = AimApp(cfg, opt, sim_world=world)
    rc = app.run()
    st = world.status()
    print("\n=== 仿真真值 ===")
    print("光斑到靶心距离 = %.2f mm（%s）"
          % (st.miss_mm, "在靶纸上" if st.spot_on_paper else "不在靶纸上"))
    if args.draw:
        print("画圆模式下请用 out/*.csv 的 sim_d2_mm 列统计 D2（赛题要求 <=20mm）")
    else:
        print("该值可直接用来判断 D1：赛题要求 <=20mm")
    print("累计拐角 %d 次，完成 %d 圈，相对偏航角 %.0f°"
          % (len(world.corner_events), st.laps_done, st.rel_yaw_deg))
    print("相对偏航角每圈净变化 -360 度 —— 这就是「绕线」的物理来源，"
          "跑完必须解绕（python main.py unwind）")
    return rc


def cmd_check(args):
    cfg = _base_config(args)
    app = AimApp(cfg, RunOptions(laser=args.laser, quiet=args.quiet))
    return app.check(duration=args.duration, laser=args.laser)


def cmd_unwind(args):
    cfg = _base_config(args)
    app = AimApp(cfg, RunOptions(quiet=args.quiet))
    return app.unwind(duration=args.duration)


def cmd_ports(args):
    """列出串口设备名 —— 地瓜派转接板上的串口到底是哪个，用这条命令确认。"""
    names = []
    try:
        from serial.tools import list_ports
        for p in list_ports.comports():
            names.append((p.device, p.description or ""))
    except Exception as exc:
        print("pyserial 不可用(%s)，改用 /dev 枚举" % exc)
    if not names:
        import glob
        for pat in ("/dev/ttyS*", "/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyAMA*"):
            for d in sorted(glob.glob(pat)):
                names.append((d, ""))
    if not names:
        print("没找到任何串口设备。")
    for dev, desc in names:
        print("  %-18s %s" % (dev, desc))
    print()
    print("接对了吗？用这条命令验证（H723 每 20ms 会发一帧 GIMBAL_STATE）：")
    print("  python main.py ping --port /dev/ttyS1")
    print("如果没有任何输出，依次检查：")
    print("  1) TX/RX 是否交叉（地瓜派 TX -> H723 RX(PA10)，地瓜派 RX <- H723 TX(PA9)）")
    print("  2) 是否共地（只接 TX/RX 两根一定会失败）")
    print("  3) 波特率 115200 是否与 H723 固件一致")
    print("  4) 当前用户是否在 dialout 组：sudo usermod -aG dialout $USER 后重新登录")
    return 0


def cmd_ping(args):
    """只收不发，看 H723 有没有在说话 —— 联调第一步就该跑这个。"""
    cfg = _base_config(args)
    if args.port:
        cfg.link_gimbal.port = args.port
    if args.baud:
        cfg.link_gimbal.baud = args.baud
    cfg.link_gimbal.enable = True
    cfg.link_car.enable = False
    from eaim import protocol as proto
    from eaim.link import SerialLink

    link = SerialLink(cfg.link_gimbal, "ping")
    if not link.start():
        print("打不开 %s：%s" % (cfg.link_gimbal.port, link.error))
        return 1
    print("监听 %s @%d，%.0f 秒（H723 每 20ms 一帧遥测）..."
          % (cfg.link_gimbal.port, cfg.link_gimbal.baud, args.duration))
    t_end = time.time() + args.duration
    st_count = 0
    last_state = None
    newer = 0.0
    while time.time() < t_end:
        for f in link.read_frames():
            if f.msg_id == proto.MsgId.GIMBAL_STATE:
                st_count += 1
                last_state = proto.unpack_gimbal_state(f.payload)
            elif f.msg_id == proto.MsgId.TEXT:
                print("  H723 %s" % f.payload.decode("utf-8", "replace"))
            elif f.msg_id == proto.MsgId.ACK:
                print("  H723 ACK %s" % proto.unpack_ack(f.payload))
            else:
                print("  收到 %s (%d 字节)" % (f.name, len(f.payload)))
        if st_count and time.time() - newer > 1.0:
            newer = time.time()
            if last_state:
                print("  遥测: %s" % last_state.describe())
        time.sleep(0.002)
    print("-" * 60)
    print("收到 %d 帧遥测；%s" % (st_count, link.stats()))
    if st_count:
        print("链路 OK。%s"
              % ("云台已就绪" if (last_state and last_state.ready())
                 else "但云台还没进 RUNNING（看上面的 H723 启动日志卡在哪一步）"))
    elif link.parser.crc_err > 0:
        print("收到字节但 CRC 全错 —— 波特率大概率不对（两边都要 115200）")
    elif link.parser.frames == 0:
        print("一个字节都没收到 —— 检查 TX/RX 交叉、共地、H723 是否已上电")
    link.close()
    return 0 if st_count else 2


def cmd_replay(args):
    cfg = _base_config(args)
    cfg.camera.source = "file"
    cfg.camera.device = args.file
    cfg.link_gimbal.enable = False
    cfg.link_car.enable = False
    cfg.app.snapshot_every_s = args.snapshot or 0.0
    app = AimApp(cfg, RunOptions(laser=not args.no_laser, quiet=args.quiet))
    return app.check(duration=args.duration, laser=False)


def build_parser():
    p = argparse.ArgumentParser(description="电赛 E 题机载计算机视觉瞄准程序")
    sub = p.add_subparsers(dest="cmd")

    r = sub.add_parser("run", help="真机运行")
    _add_common(r)
    r.add_argument("--duration", type=float, default=0.0, help="运行多少秒后退出，0=一直跑")
    r.add_argument("--draw", action="store_true", help="画圆模式（发挥部分 3）")
    r.add_argument("--laps", type=int, default=1, help="小车跑几圈")
    r.add_argument("--start-car", action="store_true", help="锁定后自动让小车起跑")
    r.add_argument("--car-corner", type=int, default=0, help="小车起点所在段 0..3")
    r.add_argument("--no-laser", action="store_true", help="不放激光")
    r.add_argument("--budget", type=float, default=0.0, help="时间预算秒（比赛用 2/4）")
    r.add_argument("--quiet", action="store_true")
    r.set_defaults(func=cmd_run)

    s = sub.add_parser("sim", help="PC 全闭环仿真（无硬件）")
    _add_common(s)
    s.add_argument("--duration", type=float, default=12.0)
    s.add_argument("--draw", action="store_true")
    s.add_argument("--laps", type=int, default=1)
    s.add_argument("--start-car", action="store_true")
    s.add_argument("--handheld", action="store_true", help="小车不动（只验证瞄准）")
    s.add_argument("--car-corner", type=int, default=0)
    s.add_argument("--lap-time", type=float, default=20.0)
    s.add_argument("--distance", type=float, default=0.8, help="相机到靶面距离(m)")
    s.add_argument("--width", type=int, default=1280)
    s.add_argument("--height", type=int, default=720)
    s.add_argument("--fps", type=int, default=60)
    s.add_argument("--no-laser", action="store_true")
    s.add_argument("--budget", type=float, default=0.0)
    s.add_argument("--quiet", action="store_true")
    s.set_defaults(func=cmd_sim)

    c = sub.add_parser("check", help="只看检测，不跑状态机")
    _add_common(c)
    c.add_argument("--duration", type=float, default=8.0)
    c.add_argument("--laser", action="store_true", help="同时打开激光（测光斑）")
    c.add_argument("--quiet", action="store_true")
    c.set_defaults(func=cmd_check)

    u = sub.add_parser("unwind", help="解绕（跑完一轮后必做）")
    _add_common(u)
    u.add_argument("--duration", type=float, default=8.0)
    u.add_argument("--quiet", action="store_true")
    u.set_defaults(func=cmd_unwind)

    rp = sub.add_parser("replay", help="对录像做离线检测统计")
    _add_common(rp)
    rp.add_argument("file", help="视频文件路径")
    rp.add_argument("--duration", type=float, default=10.0)
    rp.add_argument("--no-laser", action="store_true")
    rp.add_argument("--quiet", action="store_true")
    rp.set_defaults(func=cmd_replay)

    pt = sub.add_parser("ports", help="列出串口设备名")
    pt.set_defaults(func=cmd_ports)

    pg = sub.add_parser("ping", help="只收不发，确认 H723 有没有在通信")
    _add_common(pg)
    pg.add_argument("--port", default=None)
    pg.add_argument("--baud", type=int, default=None)
    pg.add_argument("--duration", type=float, default=5.0)
    pg.set_defaults(func=cmd_ping)
    return p


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not getattr(args, "cmd", None):
        parser.print_help()
        return 1
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
