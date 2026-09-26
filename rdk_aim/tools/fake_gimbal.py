#!/usr/bin/env python3
"""假云台：在 PC 上模拟 H723 侧，用来在没有硬件时联调链路。

用法（Linux，用 socat 造一对虚拟串口）：
    socat -d -d pty,raw,echo=0 pty,raw,echo=0
    # 会打印两个 /dev/pts/N，例如 /dev/pts/3 和 /dev/pts/5
    python tools/fake_gimbal.py --port /dev/pts/5
    python main.py run --port-gimbal /dev/pts/3 --source 0

它做的事与真实 H723 侧应当实现的完全一致：
  * 收到 MODE -> 切换状态并回 ACK
  * 收到 AIM  -> 更新内部偏置，并以 100Hz 回 GIMBAL_STATE
  * 收不到 AIM 超过 watchdog 秒 -> 自动降回 STAB 并关激光（看门狗）
  * 模拟一阶执行器（有延迟、有速率限制），Ctrl-C 时打印统计

在 Windows 上可以用 com0com 造一对虚拟串口，把 --port 指向其中一个。
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim import protocol as proto  # noqa: E402
from eaim.config import LinkConfig  # noqa: E402
from eaim.link import SerialLink  # noqa: E402


class FakeGimbal:
    def __init__(self, tau: float = 0.03, max_rate: float = 320.0,
                 watchdog: float = 0.5):
        self.mode = proto.AimMode.IDLE
        self.cmd_yaw = self.cmd_pitch = 0.0
        self.app_yaw = self.app_pitch = 0.0
        self.rel_yaw = 0.0
        self.laser = False
        self.tau = tau
        self.max_rate = max_rate
        self.watchdog = watchdog
        self.last_aim_t = time.monotonic()
        self.t0 = time.monotonic()
        self.aim_count = 0
        self.state = 6

    def on_frame(self, frame, link: SerialLink) -> None:
        if frame.msg_id == proto.MsgId.AIM:
            msg = proto.unpack_aim(frame.payload)
            self.cmd_yaw = msg["yaw_deg"]
            self.cmd_pitch = msg["pitch_deg"]
            self.laser = bool(msg["flags"] & proto.AimFlags.LASER_ON)
            self.last_aim_t = time.monotonic()
            self.aim_count += 1
        elif frame.msg_id == proto.MsgId.MODE:
            msg = proto.unpack_mode(frame.payload)
            self.mode = int(msg["mode"])
            if self.mode != proto.AimMode.AIM:
                self.laser = False
            if self.mode == proto.AimMode.STAB:
                self.cmd_yaw = self.cmd_pitch = 0.0
            print("  MODE -> %s" % proto.AimMode.name_of(self.mode))
            link.send(proto.MsgId.ACK, proto.pack_ack(frame.msg_id, 0))
        elif frame.msg_id == proto.MsgId.SET_ZERO:
            self.cmd_yaw = self.cmd_pitch = 0.0
            print("  SET_ZERO")
            link.send(proto.MsgId.ACK, proto.pack_ack(frame.msg_id, 0))
        elif frame.msg_id == proto.MsgId.VERSION:
            link.send(proto.MsgId.VERSION, b"FAKE-GIMBAL-1.0")

    def step(self, dt: float) -> None:
        # 看门狗：断流就降回 STAB 关激光（真实固件必须有这一条）
        if self.mode == proto.AimMode.AIM and \
                (time.monotonic() - self.last_aim_t) > self.watchdog:
            print("  [看门狗] %.2fs 没收到 AIM -> 自动降回 STAB" % self.watchdog)
            self.mode = proto.AimMode.STAB
            self.laser = False
            self.cmd_yaw = self.cmd_pitch = 0.0
        if self.mode in (proto.AimMode.IDLE, proto.AimMode.ESTOP):
            return
        max_step = self.max_rate * dt
        alpha = 1.0 - math.exp(-dt / max(1e-3, self.tau))
        for name in ("yaw", "pitch"):
            cur = self.app_yaw if name == "yaw" else self.app_pitch
            tgt = self.cmd_yaw if name == "yaw" else self.cmd_pitch
            step = max(-max_step, min(max_step, tgt - cur))
            new = cur + step * alpha
            if name == "yaw":
                self.app_yaw = new
            else:
                self.app_pitch = new

    def telemetry(self) -> bytes:
        flags = 0
        if self.mode in (proto.AimMode.AIM, proto.AimMode.STAB, proto.AimMode.UNWIND):
            flags |= 0x01 | 0x02
        flags |= 0x04
        if self.laser:
            flags |= 0x08
        return proto.pack_gimbal_state(
            self.state, 0, self.app_yaw, self.app_pitch, 0.0,
            self.rel_yaw, self.app_pitch, 0.0, 0.0, flags,
            int((time.monotonic() - self.t0) * 1000.0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--hz", type=float, default=100.0)
    ap.add_argument("--watchdog", type=float, default=0.5)
    args = ap.parse_args()

    link = SerialLink(LinkConfig(True, args.port, args.baud, args.hz, args.watchdog),
                      "fake-gimbal")
    if not link.start():
        print("打不开 %s: %s" % (args.port, link.error))
        return 1
    print("假云台已启动：%s @%d。等待地瓜派连接..." % (args.port, args.baud))
    g = FakeGimbal(watchdog=args.watchdog)
    last = time.monotonic()
    last_tx = 0.0
    try:
        while True:
            for frame in link.read_frames():
                g.on_frame(frame, link)
            now = time.monotonic()
            dt = now - last
            last = now
            g.step(dt)
            if now - last_tx >= 1.0 / max(1.0, args.hz):
                last_tx = now
                link.send(proto.MsgId.GIMBAL_STATE, g.telemetry())
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\n统计：收到 AIM %d 帧；%s" % (g.aim_count, link.stats()))
    finally:
        link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

