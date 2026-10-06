#!/usr/bin/env python3
"""一键急停：让云台立刻停止发速度指令（电机变软），并可选切回 STAB。

什么时候用：
  * 云台出现异常抖动/飞车/撞到东西，先按住这个命令停住
  * 调试完不想让电机一直使劲（发 ESTOP 后轴是松的，可以用手掰）

用法：
    python3 tools/estop.py --port /dev/ttyS1          # 急停（电机松）
    python3 tools/estop.py --port /dev/ttyS1 --stab   # 急停后切 STAB（保持稳定）

⚠ H723 侧 0.5s 收不到 AIM/心跳帧也会自动切 STAB，所以急停是"立刻"生效的，
   真正让电机松掉需要它持续保持在 ESTOP（本命令发完就退出，之后会掉回 STAB）。
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim import protocol as proto  # noqa: E402
from eaim.config import find_default_config, load_config  # noqa: E402
from eaim.link import SerialLink  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyS1")
    ap.add_argument("--stab", action="store_true", help="急停后切回 STAB")
    ap.add_argument("--hold", type=float, default=0.0,
                    help="保持急停这么多秒（期间持续发心跳，电机一直是松的）")
    args = ap.parse_args()

    cfg = load_config(find_default_config())
    cfg.link_gimbal.port = args.port
    cfg.link_gimbal.enable = True
    link = SerialLink(cfg.link_gimbal, "estop")
    if not link.start():
        print("打不开 %s：%s" % (args.port, link.error))
        return 1
    link.send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.ESTOP))
    link.send(proto.MsgId.AIM, proto.pack_aim(0.0, 0.0, 0, 0))
    print("已发 ESTOP（停止发速度指令，电机松）")
    t_end = time.monotonic() + max(0.0, args.hold)
    while time.monotonic() < t_end:
        # 保持"链路活着"但不改模式：否则 0.5s 后看门狗会自动切回 STAB
        link.send(proto.MsgId.HEARTBEAT_G, b"")
        time.sleep(0.2)
    if args.stab:
        time.sleep(0.3)
        link.send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.STAB))
        print("已切回 STAB（保持稳定）")
    link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
