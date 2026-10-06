#!/usr/bin/env python3
"""给云台一个固定偏置，同时看三路数据，判断"到底动了没有"。

为什么需要它：出问题时只看电机角会骗人 ——
    电机编码器说"我转了 6°"，但画面里靶心一动不动，那就不是"偏置没生效"，
    而是【电机没有真正驱动机构】（驱动器内部计数在走、轴却没动）。
    这个工具把三路一起打出来，一眼就能分辨：

        t     电机角(偏航,俯仰)      IMU角(偏航,俯仰)      靶心(px)      光斑(px)

    * 电机角变了 + IMU角跟着变 + 靶心在画面里移动  -> 正常（偏置生效）
    * 电机角变了 + IMU角不动   + 靶心不动          -> **电机没真正驱动**（查电机电源/使能/驱动器）
    * 电机角不动 + 其它都不动                      -> 偏置没下发（查链路/模式）

用法：
    python3 tools/offset_test.py --port /dev/ttyS1 --yaw 20 --pitch 0 --seconds 8
    python3 tools/offset_test.py --yaw 0 --pitch 15 --seconds 6
"""

import argparse
import os
import sys
import threading
import time

import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim import protocol as proto  # noqa: E402
from eaim.app import AimApp, RunOptions  # noqa: E402
from eaim.config import find_default_config, load_config  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyS1")
    ap.add_argument("--source", default="0")
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--pitch", type=float, default=0.0)
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--period", type=float, default=0.5)
    ap.add_argument("--save", default=None, help="把最后一帧存成这个文件（对比用）")
    ap.add_argument("--no-camera", action="store_true",
                    help="不打开相机（实时画面正在用时也能跑，只看电机角/IMU角）")
    args = ap.parse_args()

    cfg = load_config(find_default_config())
    cfg.link_gimbal.port = args.port
    cfg.link_gimbal.enable = True
    cfg.link_car.enable = False
    state = {"yaw": args.yaw, "pitch": args.pitch}
    _stop = threading.Event()

    # ---- 不看图模式：直接用串口，不去碰相机（实时画面正在用时也能跑）----
    if args.no_camera:
        from eaim.link import SerialLink
        link = SerialLink(cfg.link_gimbal, "offset_test")
        if not link.start():
            print("打不开 %s：%s" % (args.port, link.error))
            return 1

        def _keep():
            while not _stop.is_set():
                # 周期性重发 MODE：H723 侧有 0.5s 的 AIM 看门狗，一旦降级到
                # STAB 就不会自己回来。只发一次 MODE 的话，只要那一帧和
                # AIM 抢了顺序（或者丢了一帧），整个测试就变成"发偏置但云台
                # 不动"——实测就这么白白浪费过好几轮标定。
                link.send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.AIM))
                link.send(proto.MsgId.AIM,
                          proto.pack_aim(state["yaw"], state["pitch"],
                                         proto.AimFlags.LASER_ON, 0))
                _stop.wait(0.1)

        threading.Thread(target=_keep, daemon=True).start()
        link.send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.AIM))
        time.sleep(0.5)
        print("=" * 60)
        print(" 固定偏置 (%.1f, %.1f)°，%.0f 秒（不看图）"
              % (args.yaw, args.pitch, args.seconds))
        print(" %-6s %-18s %-18s" % ("t", "电机角(偏,俯)", "IMU角(偏,俯)"))
        print("=" * 60)
        st = None
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.seconds:
            for fr in link.read_frames():
                if fr.msg_id == proto.MsgId.GIMBAL_STATE:
                    try:
                        st = proto.unpack_gimbal_state(fr.payload)
                    except Exception:                          # noqa: BLE001
                        pass
            if st is not None:
                print(" %-6.1f %-18s %-18s"
                      % (time.monotonic() - t0,
                         "(%.1f, %.1f)" % (st.yaw_motor_deg, st.pitch_motor_deg),
                         "(%.1f, %.1f)" % (st.yaw_deg, st.pitch_deg)))
            time.sleep(max(0.05, args.period))
        _stop.set()
        link.send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.STAB))
        link.close()
        return 0

    app = AimApp(cfg, RunOptions(laser=True, quiet=True))
    if not app.setup():
        return 1

    def _keepalive():
        while not _stop.is_set():
            # 周期性重发 MODE：H723 有 0.5s 的 AIM 看门狗，降级到 STAB 后
            # 不会自己回来。只发一次 MODE 的话，只要它和 AIM 抢了顺序，
            # 测试就变成"偏置发下去了、云台却不动"（实测白标过好几轮）。
            app.gimbal_link.send(proto.MsgId.MODE,
                                 proto.pack_mode(proto.AimMode.AIM))
            app.gimbal_link.send(
                proto.MsgId.AIM,
                proto.pack_aim(state["yaw"], state["pitch"],
                               proto.AimFlags.LASER_ON, 0))
            _stop.wait(0.1)

    threading.Thread(target=_keepalive, daemon=True).start()
    app.gimbal_link.send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.AIM))
    time.sleep(0.5)

    print("=" * 78)
    print(" 固定偏置 (%.1f, %.1f)°，观察 %.0f 秒" % (args.yaw, args.pitch, args.seconds))
    print(" %-6s %-16s %-16s %-16s %-14s" % ("t", "电机角(偏,俯)", "IMU角(偏,俯)", "靶心px", "光斑px"))
    print("=" * 78)

    t0 = time.monotonic()
    st = None
    last_img = None
    while time.monotonic() - t0 < args.seconds:
        for fr in app.gimbal_link.read_frames():
            if fr.msg_id == proto.MsgId.GIMBAL_STATE:
                try:
                    st = proto.unpack_gimbal_state(fr.payload)
                except Exception:                              # noqa: BLE001
                    pass
        f, _ = (None, None) if args.no_camera else app.camera.read()
        tgt = sp = None
        if f is not None:
            last_img = f.image
            t = app.detector.detect(f.image)
            s = app.spot_detector.detect(f.image)
            tgt = t.uv if t is not None else None
            sp = s.uv if s is not None else None
        print(" %-6.1f %-16s %-16s %-16s %-14s"
              % (time.monotonic() - t0,
                 ("(%.1f, %.1f)" % (st.yaw_motor_deg, st.pitch_motor_deg)) if st else "?",
                 ("(%.1f, %.1f)" % (st.yaw_deg, st.pitch_deg)) if st else "?",
                 ("(%.0f, %.0f)" % tgt) if tgt else "无",
                 ("(%.0f, %.0f)" % sp) if sp else "无"))
        time.sleep(max(0.05, args.period))

    if args.save and last_img is not None:
        cv2.imwrite(args.save, last_img)
        print("最后一帧已存到 %s" % args.save)

    _stop.set()
    app.gimbal_link.send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.STAB))
    time.sleep(0.1)
    app.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
