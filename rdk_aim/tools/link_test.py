#!/usr/bin/env python3
"""双向链路测试：一边收、一边发，把"收不到数据"这个问题一刀切成两半。

用法：
    python3 tools/link_test.py --port /dev/ttyS1 --seconds 15

它做三件事：
  1) 收：有没有 H723 发来的合法遥测帧
  2) 发：每 100ms 发一次 AIM 指令，让偏航偏置在 ±5° 之间慢慢来回摆
  3) 收尾：无论怎么退出，都发 MODE=STAB + 关激光（安全）

怎么判断结果：

    云台会左右摆  -> 【地瓜派 -> H723】这条方向是通的
                     那么问题在【H723 -> 地瓜派】：
                     要么 H723 的 TX(PA9) 没接到地瓜派的 RX，
                     要么 TX/RX 接反了（两个 TX 怼在一起）

    云台完全不动  -> 两个方向都不通，优先怀疑：
                     · H723 没上电 / 没在跑
                     · GND 没共
                     · TX/RX 没交叉

⚠ 安全：全程不带 LASER_ON 标志，激光不会亮；结束时自动切 STAB。
"""

import argparse
import math
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim import protocol as proto  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyS1")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=18.0)
    ap.add_argument("--amplitude", type=float, default=20.0, help="偏航偏置摆幅（度）")
    ap.add_argument("--hold", type=float, default=3.0,
                    help="每个方向停多久（秒）")
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        print("没装 pyserial：pip3 install pyserial")
        return 1

    try:
        dev = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as exc:                                   # noqa: BLE001
        print("打不开 %s：%s" % (args.port, exc))
        return 1

    parser = proto.FrameParser()
    seq = 0
    rx_frames = 0
    rx_bytes = 0
    last_state = None
    rx_text = []

    def send(msg_id, payload=b""):
        nonlocal seq
        dev.write(proto.build_frame(msg_id, payload, seq))
        seq = (seq + 1) & 0xFF

    def cleanup():
        try:
            send(proto.MsgId.AIM, proto.pack_aim(0.0, 0.0, 0, 0))
            send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.STAB))
            time.sleep(0.1)
            dev.close()
        except Exception:                                      # noqa: BLE001
            pass

    def on_sigint(_sig, _frm):
        print("\n收到 Ctrl-C，收尾中...")
        cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, on_sigint)

    print("=" * 66)
    print(" 双向链路测试：%s @%d，持续 %.0f 秒" % (args.port, args.baud, args.seconds))
    print("=" * 66)
    print(" ⚠ 现在请盯着云台，它应该这样动：")
    print("     先慢慢转到一边 → 停 %.0f 秒 → 再慢慢转到另一边 → 停 %.0f 秒 ……"
          % (args.hold, args.hold))
    print("   幅度 ±%.0f°，和「慢慢漂移」完全不一样，不会看错。" % args.amplitude)
    print("   如果云台【完全不动】，说明地瓜派的指令没能送到 H723。")
    print("-" * 66)

    # 先切到 AIM 模式（只有 AIM 模式才会应用偏置），再给 0 偏置让基准稳一下
    send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.AIM))
    send(proto.MsgId.AIM, proto.pack_aim(0.0, 0.0, 0, 0))
    time.sleep(0.3)

    t0 = time.time()
    next_tx = t0
    next_report = t0 + 1.0
    try:
        while time.time() - t0 < args.seconds:
            now = time.time()

            # ---- 发：100Hz 偏航偏置「方波」摆动（大幅 + 停顿，肉眼最容易分辨）----
            if now >= next_tx:
                next_tx = now + 0.01
                phase = int((now - t0) / max(0.5, args.hold)) % 2
                yaw = args.amplitude if phase == 0 else -args.amplitude
                send(proto.MsgId.AIM, proto.pack_aim(yaw, 0.0, 0, 0))

            # ---- 收 ----
            data = dev.read(512)
            if data:
                rx_bytes += len(data)
                for frame in parser.feed(data):
                    rx_frames += 1
                    if frame.msg_id == proto.MsgId.GIMBAL_STATE:
                        try:
                            last_state = proto.unpack_gimbal_state(frame.payload)
                        except Exception:                      # noqa: BLE001
                            pass
                    elif frame.msg_id == proto.MsgId.TEXT:
                        rx_text.append(frame.payload.decode("utf-8", "replace"))

            # ---- 每秒汇报一次 ----
            if now >= next_report:
                next_report = now + 1.0
                print("  t=%4.1fs  发出偏置=%+5.1f°  收到 %d 字节 / %d 帧"
                      % (now - t0,
                         args.amplitude
                         if int((now - t0) / max(0.5, args.hold)) % 2 == 0
                         else -args.amplitude,
                         rx_bytes, rx_frames))
                # 把 H723 的调试文本实时打出来（这是最有用的信息来源）
                while rx_text:
                    print("        H723 | %s" % rx_text.pop(0))
                if last_state is not None:
                    print("        遥测 | %s" % last_state.describe())
    finally:
        cleanup()

    print("-" * 66)
    print("结果：收到 %d 字节 / %d 帧；%s" % (rx_bytes, rx_frames, parser.stats()))
    if rx_frames:
        print("✔ 双向都通。")
    elif rx_bytes:
        print("△ 收到字节但都是非法帧 —— 波特率或线序有问题。")
    else:
        print("✘ 一个字节都没收到。请看上面的提示对着查线。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
