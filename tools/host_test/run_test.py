#!/usr/bin/env python3
"""跨语言协议回归测试：STM32 侧的 gimbal_proto.c  <->  上位机 eaim/protocol.py

    python tools/host_test/run_test.py

做的事：
  1) 用 PC 上的 gcc 把 Core/Src/gimbal_proto.c 编译成可执行文件（不碰 HAL）
  2) 让 C 侧生成帧 -> 喂给 Python 解析器，逐字段比对
  3) 让 Python 生成帧 -> 喂给 C 侧解析器，逐字段比对
  4) 故意注入断包/垃圾字节/CRC 错误，验证 C 侧解析器能重新同步

为什么值得做：协议错了在板子上极难定位（不知道是发错还是收错），
而这个测试能在 1 秒内把帧格式、字节序、CRC、限幅全部验一遍。
"""

import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))           # yuntai2/
RDK = os.path.join(os.path.dirname(PROJ), "yuntai2", "rdk_aim")
if not os.path.isdir(RDK):
    RDK = os.path.join(PROJ, "rdk_aim")
sys.path.insert(0, RDK)

from eaim import protocol as proto   # noqa: E402

CC = os.environ.get("CC", "gcc")
BIN = os.path.join(HERE, "build", "proto_selftest")
if os.name == "nt":
    BIN += ".exe"


def build():
    os.makedirs(os.path.dirname(BIN), exist_ok=True)
    cmd = [CC, "-O1", "-Wall", "-I", os.path.join(PROJ, "Core", "Inc"),
           os.path.join(HERE, "proto_selftest.c"),
           os.path.join(PROJ, "Core", "Src", "gimbal_proto.c"),
           "-o", BIN]
    r = subprocess.run(cmd, capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    if r.returncode != 0:
        print("gcc 编译失败：\n%s\n%s" % (r.stdout, r.stderr))
        return False
    if r.stderr.strip():
        print("gcc 警告：\n%s" % r.stderr.strip())
    return True


def run(*args):
    r = subprocess.run([BIN] + list(args), capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    if r.returncode != 0:
        raise RuntimeError("自测程序返回 %d: %s" % (r.returncode, r.stderr))
    return r.stdout


def to_hex(data: bytes) -> str:
    return data.hex().upper()


def from_hex(text: str) -> bytes:
    return bytes.fromhex(text.strip())


# ---------------------------------------------------------------------------
def test_crc():
    out = run("gen")
    m = re.search(r"crc789=([0-9A-F]{4})", out)
    assert m, "没找到 C 侧 CRC 自检输出"
    assert int(m.group(1), 16) == proto.crc16_modbus(b"123456789"), "CRC 与 Python 不一致"
    assert int(m.group(1), 16) == 0x4B37


def test_c_generates_python_parses():
    out = run("gen")
    lines = [l.strip() for l in out.splitlines() if re.fullmatch(r"[0-9A-F]+", l.strip())]
    assert len(lines) >= 3, "C 侧没有输出 3 帧"
    parser = proto.FrameParser()
    frames = parser.feed(from_hex("".join(lines)))
    assert len(frames) == 3, "Python 解析出的帧数 = %d" % len(frames)
    assert parser.crc_err == 0

    st = proto.unpack_gimbal_state(frames[0].payload)
    assert st.state == 6 and st.fault == 0
    assert abs(st.yaw_deg - 12.34) < 0.01, st.yaw_deg
    assert abs(st.pitch_deg + 5.67) < 0.01, st.pitch_deg
    assert abs(st.roll_deg - 1.50) < 0.01
    assert abs(st.yaw_motor_deg + 90.00) < 0.01
    assert abs(st.pitch_motor_deg - 10.25) < 0.01
    assert abs(st.gyro_y_dps - 2.50) < 0.01
    assert abs(st.gyro_z_dps + 1.25) < 0.01
    assert st.uptime_ms == 123456
    assert st.closed_loop and st.speed_mode and st.encoder_ok and st.laser_on
    assert st.ready(), "READY 位没解析出来"

    ack = proto.unpack_ack(frames[1].payload)
    assert ack["msg_id"] == proto.MsgId.MODE and ack["code"] == 0
    assert frames[2].msg_id == proto.MsgId.TEXT
    assert frames[2].payload.decode("ascii") == "[H723] BMI088 OK -> Motor Boot Delay"


def test_python_generates_c_parses():
    aim = proto.build_frame(proto.MsgId.AIM,
                            proto.pack_aim(12.34, -5.67, 0x0F, 200), 3)
    mode = proto.build_frame(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.AIM, 1), 4)
    zero = proto.build_frame(proto.MsgId.SET_ZERO, b"", 5)
    unwind = proto.build_frame(proto.MsgId.UNWIND, proto.pack_unwind(30.0), 6)
    hb = proto.build_frame(proto.MsgId.HEARTBEAT_G, b"", 7)
    blob = to_hex(aim + mode + zero + unwind + hb)
    out = run("parse", blob)

    assert "msg=0x10 seq=3 len=6 yaw=1234 pitch=-567 flags=15 quality=200" in out, out
    assert "msg=0x11 seq=4 len=2 mode=2 arg=1" in out, out
    assert "msg=0x13 seq=5 len=0 zero" in out, out
    assert "msg=0x14 seq=6 len=2 unwind_dps10=300" in out, out
    assert "msg=0x12 seq=7 len=0 hb" in out, out
    assert "stats frames=5 crc_err=0 bad_len=0" in out.replace(" ", " "), out
    print("   C 侧统计:", [l for l in out.splitlines() if l.startswith("stats")][0])


def test_noise_and_partial_frames():
    """断包 + 垃圾字节 + CRC 错误都要能恢复。"""
    good = proto.build_frame(proto.MsgId.AIM, proto.pack_aim(1.0, -1.0, 1, 9), 1)
    bad = bytearray(proto.build_frame(proto.MsgId.MODE, proto.pack_mode(2), 2))
    bad[-1] ^= 0xFF                                    # 破坏 CRC
    blob = b"\x00\x11\x22" + good[:4] + good[4:] + bytes(bad) + good
    out = run("parse", to_hex(blob))
    assert out.count("msg=0x10") == 2, out
    assert "crc_err=1" in out, out
    assert "resync=" in out, out
    print("   ", [l for l in out.splitlines() if l.startswith("stats")][0])


def test_scale_clamp():
    """超范围的角度必须限幅（不能溢出成反号），常规值要与 Python 侧一致。"""
    for deg, expect in ((1e9, 32767), (-1e9, -32768)):
        frame = proto.build_frame(proto.MsgId.AIM, proto.pack_aim(deg, 0.0, 0, 0), 0)
        out = run("parse", to_hex(frame))
        m = re.search(r"yaw=(-?\d+)", out)
        assert m, out
        assert int(m.group(1)) == expect, "%s -> %s（期望 %d）" % (deg, m.group(1), expect)

    # 常规范围：C 的"四舍五入"与 Python 的 round() 在 .5 附近可能差 1 个 LSB
    # （0.01° = 1.1m 处 0.2mm），不要求逐位相同，但绝不允许更大偏差
    import random
    random.seed(7)
    worst = 0
    for _ in range(40):
        deg = random.uniform(-170.0, 170.0)
        frame = proto.build_frame(proto.MsgId.AIM, proto.pack_aim(deg, -deg, 0, 0), 0)
        out = run("parse", to_hex(frame))
        m = re.search(r"yaw=(-?\d+) pitch=(-?\d+)", out)
        assert m, out
        py = proto.unpack_aim(proto.pack_aim(deg, -deg, 0, 0))
        worst = max(worst,
                    abs(int(m.group(1)) - round(py["yaw_deg"] * 100)),
                    abs(int(m.group(2)) - round(py["pitch_deg"] * 100)))
    assert worst <= 1, "C 与 Python 的角度量化最大差 %d 个 LSB" % worst


def test_longest_payload():
    """GIMBAL_STATE 是固定 21 字节，改协议时最容易忘记同步这个长度。"""
    assert proto.pack_gimbal_state(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11).__len__() == 21


TESTS = [test_crc, test_c_generates_python_parses, test_python_generates_c_parses,
         test_noise_and_partial_frames, test_scale_clamp, test_longest_payload]


def main():
    print("编译 %s ..." % os.path.relpath(BIN, PROJ))
    if not build():
        return 1
    failed = 0
    for t in TESTS:
        try:
            t()
            print("  [PASS] %s" % t.__name__)
        except Exception as exc:                       # noqa: BLE001
            failed += 1
            print("  [FAIL] %s: %s" % (t.__name__, exc))
    print("-" * 60)
    print("共 %d 项，失败 %d 项" % (len(TESTS), failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
