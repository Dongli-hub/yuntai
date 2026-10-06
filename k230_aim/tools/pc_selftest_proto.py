# -*- coding: utf-8 -*-
"""pc_selftest_proto.py —— 在电脑上验证 K230 协议层（不需要硬件）

用法（Windows PowerShell）：
    cd D:\\STM32\\STM32projects\\yuntai2
    python k230_aim\\tools\\pc_selftest_proto.py

做的事：把 k230_aim/lib/proto_k230.py 的每个打包结果
和地瓜派在用的 rdk_aim/eaim/protocol.py 逐字节比对；
再用地瓜派版构造的字节流喂给 K230 版解析器，验证解析一致。
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "rdk_aim"))
sys.path.insert(0, os.path.join(ROOT, "k230_aim", "lib"))

from eaim import protocol as ref          # noqa: E402
import proto_k230 as k230                 # noqa: E402

fail = 0


def check(name, a, b):
    global fail
    if a == b:
        print("  PASS  %-32s %s" % (name,
                                    a.hex() if isinstance(a, bytes) else a))
    else:
        fail += 1
        print("  FAIL  %-32s\n        ref =%s\n        k230=%s"
              % (name, a, b))


print("=" * 70)
print("K230 协议层自测（与 rdk_aim 逐字节比对）")
print("=" * 70)

check("crc16('123456789')", ref.crc16_modbus(b"123456789"),
      k230.crc16_modbus(b"123456789"))

check("HEARTBEAT_G", ref.build_frame(ref.MsgId.HEARTBEAT_G),
      k230.build_frame(k230.MSG_HEARTBEAT_G))

check("MODE AIM", ref.build_frame(ref.MsgId.MODE, ref.pack_mode(2, 0)),
      k230.build_frame(k230.MSG_MODE, k230.pack_mode(k230.MODE_AIM, 0)))

check("MODE STAB", ref.build_frame(ref.MsgId.MODE, ref.pack_mode(1, 0)),
      k230.build_frame(k230.MSG_MODE, k230.pack_mode(k230.MODE_STAB, 0)))

check("SET_ZERO", ref.build_frame(ref.MsgId.SET_ZERO),
      k230.build_frame(k230.MSG_SET_ZERO))

for yaw, pitch, flags, q in ((0.0, 0.0, 0x03, 200),
                             (1.25, -0.75, 0x1F, 255),
                             (-12.34, 3.21, 0x00, 0)):
    check("AIM yaw=%.2f pitch=%.2f" % (yaw, pitch),
          ref.build_frame(ref.MsgId.AIM, ref.pack_aim(yaw, pitch, flags, q)),
          k230.build_frame(k230.MSG_AIM,
                           k230.pack_aim(yaw, pitch, flags, q)))

check("UNWIND 120dps",
      ref.build_frame(ref.MsgId.UNWIND, ref.pack_unwind(120.0)),
      k230.build_frame(k230.MSG_UNWIND, k230.pack_unwind(120.0)))

check("PARAM PITCH_KP=8",
      ref.build_frame(ref.MsgId.PARAM,
                      ref.pack_param(ref.ParamId.PITCH_KP, 8.0)),
      k230.build_frame(k230.MSG_PARAM,
                       k230.pack_param(k230.PARAM_PITCH_KP, 8.0)))

payload = ref.pack_gimbal_state(6, 0, 12.34, -5.67, 1.23, 45.6, -30.1,
                                2.5, -1.5, 0x1D, 123456)
st = k230.unpack_gimbal_state(payload)
ref_st = ref.unpack_gimbal_state(payload)
same = (st.state == ref_st.state and st.flags == ref_st.flags and
        abs(st.yaw_deg - ref_st.yaw_deg) < 1e-9 and
        abs(st.pitch_deg - ref_st.pitch_deg) < 1e-9 and
        st.ready() == ref_st.ready() and st.uptime_ms == ref_st.uptime_ms)
if same:
    print("  PASS  %-32s %s" % ("遥测解析一致", st.describe()))
else:
    fail += 1
    print("  FAIL  遥测解析不一致")

stream = (b"\x00\x11\xAA" + ref.build_frame(ref.MsgId.HEARTBEAT_G) +
          ref.build_frame(ref.MsgId.GIMBAL_STATE, payload) +
          ref.build_frame(ref.MsgId.AIM, ref.pack_aim(0.0, 0.0, 0x03, 200)) +
          b"\xAA\x55\x12\x00")          # 尾部半截帧：应留在缓冲区
parser = k230.FrameParser()
frames = parser.feed(stream[:7])
frames += parser.feed(stream[7:])
ids = [f[0] for f in frames]
if ids == [k230.MSG_HEARTBEAT_G, k230.MSG_GIMBAL_STATE, k230.MSG_AIM] \
        and len(parser.buf) == 4 and parser.crc_err == 0:
    print("  PASS  %-32s ids=%s %s" % ("流式解析(粘包/噪声)", ids,
                                        parser.stats()))
else:
    fail += 1
    print("  FAIL  流式解析 ids=%s %s" % (ids, parser.stats()))

print("=" * 70)
print("结果：%s" % ("全部通过" if fail == 0 else "%d 项失败" % fail))
sys.exit(1 if fail else 0)
