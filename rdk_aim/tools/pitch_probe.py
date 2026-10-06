"""俯仰自激探针：在线把可疑参数一个个关掉，看俯仰电机还摆不摆。

背景（2026-09-27 晚 现场实测）：
  H723 的俯仰轴用【电机编码器】做位置环，IMU 装在它下面**看不见它**，
  所以姿态遥测里俯仰一直是 2.5° 纹波不出问题，但俯仰电机角在
  AIM 模式下会以 ~1Hz 在 133↔260 之间自己摆 —— 相机跟着上下跳，
  靶纸一会儿在画面里、一会儿跑出去，视觉环根本没法收敛。

用法（地瓜派上，e_aim 目录）：

    python3 tools/pitch_probe.py --port /dev/ttyS1            # 只观察，不改参数
    python3 tools/pitch_probe.py --port /dev/ttyS1 --off 0x0A # 关掉平台俯仰补偿再看
    python3 tools/pitch_probe.py --port /dev/ttyS1 --on 0x0A  # 恢复

参数编号见 eaim/protocol.py 的 ParamId：0x0A = 平台俯仰补偿符号。
"""

import argparse
import os
import statistics
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim import protocol as proto                     # noqa: E402
from eaim.config import find_default_config, load_config  # noqa: E402
from eaim.link import SerialLink                       # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="俯仰自激探针")
    ap.add_argument("--port", default="/dev/ttyS1")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--off", default="", help="先把这个参数设为 0（十六进制，如 0x0A）")
    ap.add_argument("--on", default="", help="把参数恢复成 1")
    ap.add_argument("--set", action="append", default=[],
                    help="在线写参数，可重复：--set 0x05=8 --set 0x06=20")
    args = ap.parse_args()

    cfg = load_config(find_default_config())
    cfg.link_gimbal.port = args.port
    cfg.link_gimbal.enable = True
    link = SerialLink(cfg.link_gimbal, "pitch_probe")
    if not link.start():
        print("打不开 %s：%s" % (args.port, link.error))
        return 1

    stop = threading.Event()

    def keep():
        while not stop.is_set():
            link.send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.AIM))
            link.send(proto.MsgId.AIM, proto.pack_aim(0.0, 0.0, 0, 0))
            stop.wait(0.1)

    threading.Thread(target=keep, daemon=True).start()
    time.sleep(0.4)
    for tag, val in (("off", args.off), ("on", args.on)):
        if val:
            pid = int(str(val), 0)
            v = 0.0 if tag == "off" else 1.0
            link.send(proto.MsgId.PARAM, proto.pack_param(pid, v))
            print("已把参数 0x%02X 设为 %g" % (pid, v))
    for item in args.set:
        if "=" in item:
            k, v = item.split("=", 1)
            pid, val = int(k.strip(), 0), float(v)
            link.send(proto.MsgId.PARAM, proto.pack_param(pid, val))
            print("已把参数 0x%02X 设为 %g" % (pid, val))
    time.sleep(0.4)

    print("%-6s %-10s %-10s %s" % ("t", "俯仰电机角", "IMU俯仰", "变化"))
    vals = []
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
            vals.append(float(st.pitch_motor_deg))
            print("%-6.2f %-10.1f %-10.2f %s"
                  % (time.monotonic() - t0, st.pitch_motor_deg, st.pitch_deg,
                     ("%.1f" % (vals[-1] - vals[-2])) if len(vals) > 1 else ""))
        time.sleep(0.25)
    stop.set()
    link.send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.STAB))
    link.close()
    if len(vals) > 3:
        span = max(vals) - min(vals)
        steps = [abs(b - a) for a, b in zip(vals, vals[1:])]
        print("-" * 50)
        print("俯仰电机角: 范围 %.1f（峰峰 %.1f），逐次跳变中位 %.2f"
              % (statistics.mean(vals), span, statistics.median(steps)))
        print("判据：峰峰 < 2 且逐次跳变 < 1 = 稳；峰峰 > 10 = 在自激/被打摆")
    return 0


if __name__ == "__main__":
    sys.exit(main())
