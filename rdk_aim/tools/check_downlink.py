#!/usr/bin/env python3
"""只验证一个方向：地瓜派 -> H723（上行/下行分开查，一次定位一根线）。

用法：
    python3 tools/check_downlink.py --port /dev/ttyS1

原理（不用你看云台，也不用示波器）：
    H723 每 5 秒会发一条 [STAT] 日志，里面有【它自己收到的帧数 rx=N】。
    我们发一批【无害的心跳帧】，再看 rx 有没有涨：

        rx 涨了   -> 地瓜派 TX -> H723 RX 这根线是通的
        rx 没涨   -> 这根线没通（或 H723 的 RX 脚悬空，只采到噪声）

为什么发心跳而不是发偏置指令：
    心跳只是更新一下"链路还活着"的标记，不改变任何控制行为 ——
    云台不会动、激光不会亮，怎么发都不会出事。

顺便会把 H723 的 TEXT 日志实时打出来，方便一眼看出它在干什么。
"""

import argparse
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim import protocol as proto  # noqa: E402

STAT_RE = re.compile(r"rx=(\d+)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyS1")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--count", type=int, default=200, help="发多少帧心跳")
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
    baseline = None
    latest = None
    text_count = 0

    def pump(seconds: float, quiet: bool = False):
        """读 seconds 秒，返回期间见到的 [STAT] rx 值（可能为 None）"""
        nonlocal baseline, latest, text_count
        t_end = time.time() + seconds
        while time.time() < t_end:
            data = dev.read(256)
            if not data:
                continue
            for frame in parser.feed(data):
                if frame.msg_id != proto.MsgId.TEXT:
                    continue
                text_count += 1
                line = frame.payload.decode("utf-8", "replace")
                if not quiet:
                    print("        H723 | %s" % line)
                m = STAT_RE.search(line)
                if m:
                    latest = int(m.group(1))
                    if baseline is None:
                        baseline = latest
        return latest

    def send_heartbeats(n: int, hz: float = 100.0):
        nonlocal seq
        period = 1.0 / hz
        for _ in range(n):
            dev.write(proto.build_frame(proto.MsgId.HEARTBEAT_G, b"", seq))
            seq = (seq + 1) & 0xFF
            time.sleep(period)

    print("=" * 66)
    print(" 验证方向：地瓜派 TX -> H723 RX   端口 %s @%d" % (args.port, args.baud))
    print("=" * 66)
    print("[1/3] 先只听 7 秒，取 H723 的接收计数基准 ...")
    pump(7.0)
    if latest is None:
        print("      没收到 [STAT] 日志 —— 说明【H723 -> 地瓜派】这个方向也不通，")
        print("      先把那根线弄通再来测本方向（用 tools/find_serial.py 确认）。")
        dev.close()
        return 2
    print("      基准: H723 已收到 %d 帧" % latest)

    print("[2/3] 发 %d 帧心跳（无害，云台不动、激光不亮）..." % args.count)
    send_heartbeats(args.count)
    print("      发完了，再听 7 秒，看 H723 的计数有没有涨 ...")
    pump(7.0)

    print("-" * 66)
    print("结果: 基准 rx=%d  ->  现在 rx=%s" % (baseline, latest))
    if latest is not None and baseline is not None and latest > baseline:
        print("✔ 通了！地瓜派 TX -> H723 RX 这根线是好的（收到 %d 帧增量）"
              % (latest - baseline))
        print("  两个方向都通的话，可以直接开始标定：")
        print("    python3 tools/check_laser.py --port %s --source 0 --save" % args.port)
    else:
        print("✘ 没涨。地瓜派 -> H723 这根线还是不通，检查：")
        print("   1) 地瓜派的 TX（不是 RX！）要接到 H723 的 PA10(USART1_RX)")
        print("   2) 那根线插牢了吗（杜邦线很容易虚接）")
        print("   3) 共地了吗")
        print("   4) 和另一根线对调一下试试（交叉接最容易被搞反）")
    print("（本次共收到 %d 条 TEXT 日志）" % text_count)
    dev.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

