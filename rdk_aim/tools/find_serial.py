#!/usr/bin/env python3
"""自动找出 H723 接在地瓜派的哪个串口上。

用法：
    python3 tools/find_serial.py                    # 扫 /dev/ttyS1..S7 + USB 串口
    python3 tools/find_serial.py --seconds 3        # 每个口听 3 秒
    python3 tools/find_serial.py --include-ttyS0    # 连调试串口一起扫（有风险，见下）
    python3 tools/find_serial.py --port /dev/ttyS1  # 只测指定端口

判据不是"收到字节"，而是**能不能解析出合法的协议帧**：
   H723 上电后会每 20ms 发一帧 GIMBAL_STATE。
   只有 CRC 正确、帧格式正确的数据才算数 —— 这样即使线接错、
   或者串口上飘着噪声，也不会误判。

⚠ 默认跳过 /dev/ttyS0：在 RDK 上它通常是系统调试控制台，
  去读它有可能干扰到那个控制台。要扫它得显式加 --include-ttyS0。
"""

import argparse
import glob
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim import protocol as proto  # noqa: E402


def default_ports(include_ttyS0: bool):
    ports = []
    for p in sorted(glob.glob("/dev/ttyS*")):
        if (not include_ttyS0) and p.endswith("ttyS0"):
            continue
        ports.append(p)
    ports += sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
    return ports


def probe(port: str, baud: int, seconds: float):
    """返回 (状态, 字节数, 帧数, 各类消息计数, 最后一帧状态)"""
    try:
        import serial
    except ImportError:
        return ("没有装 pyserial", 0, 0, {}, None)
    try:
        dev = serial.Serial(port, baud, timeout=0.1)
    except Exception as exc:                                  # noqa: BLE001
        return ("打不开(%s)" % str(exc)[:40], 0, 0, {}, None)

    parser = proto.FrameParser()
    nbytes, frames = 0, 0
    kinds = {}
    last_state = None
    t_end = time.time() + seconds
    try:
        while time.time() < t_end:
            data = dev.read(512)
            if not data:
                continue
            nbytes += len(data)
            for frame in parser.feed(data):
                frames += 1
                kinds[frame.msg_id] = kinds.get(frame.msg_id, 0) + 1
                if frame.msg_id == proto.MsgId.GIMBAL_STATE:
                    try:
                        last_state = proto.unpack_gimbal_state(frame.payload)
                    except Exception:                          # noqa: BLE001
                        pass
    finally:
        try:
            dev.close()
        except Exception:                                      # noqa: BLE001
            pass
    return ("OK", nbytes, frames, kinds, last_state)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="只测这个端口")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=2.0, help="每个端口听多久")
    ap.add_argument("--include-ttyS0", action="store_true",
                    help="连 /dev/ttyS0（系统调试串口）一起扫")
    args = ap.parse_args()

    ports = [args.port] if args.port else default_ports(args.include_ttyS0)
    if not ports:
        print("没找到任何串口设备（/dev/ttyS* / ttyUSB* / ttyACM* 都没有）")
        return 1

    print("=" * 66)
    print(" 扫描串口，找 H723（波特率 %d，每个口听 %.1fs）" % (args.baud, args.seconds))
    print(" 判据：能不能解析出合法的协议帧，而不是「收到字节就算」")
    print("=" * 66)

    hits = []
    for p in ports:
        print("%-16s ..." % p, end="", flush=True)
        status, nbytes, frames, kinds, st = probe(p, args.baud, args.seconds)
        if frames:
            desc = ", ".join("%s×%d" % (proto.MsgId.name(k), v)
                             for k, v in sorted(kinds.items()))
            print(" ★ 命中！%d 字节 / %d 帧（%s）" % (nbytes, frames, desc))
            if st is not None:
                print("%-16s    遥测: %s" % ("", st.describe()))
                print("%-16s    %s" % ("", "云台已就绪"
                                       if st.ready() else
                                       "云台还没就绪（看上面的启动日志卡在哪步）"))
            hits.append(p)
        elif nbytes:
            print(" 收到 %d 字节但一帧都不合法 —— 波特率不对？" % nbytes)
        else:
            print(" 没数据（%s）" % status)

    print("-" * 66)
    if len(hits) == 1:
        print("结论: H723 在 %s" % hits[0])
        print("接下来: python3 main.py ping --port %s" % hits[0])
    elif len(hits) > 1:
        print("结论: 有多个口都在发合法帧 %s —— 检查是不是接了不止一个设备" % hits)
    else:
        print("结论: 没找到 H723。按顺序查：")
        print("  1) H723 上电了吗？（上电后它每 20ms 就会发遥测）")
        print("  2) TX/RX 交叉了吗？地瓜派 TX -> H723 RX(PA10)；地瓜派 RX <- H723 TX(PA9)")
        print("  3) 共地了吗？（只接 TX/RX 两根一定不通）")
        print("  4) 换一个串口再扫（转接板上可能有多个串口座）")
    return 0 if hits else 2


if __name__ == "__main__":
    sys.exit(main())
