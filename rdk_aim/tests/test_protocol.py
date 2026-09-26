"""协议层测试：帧粘包/断包/CRC 校验，以及所有消息的打包解包往返。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim import protocol as proto  # noqa: E402


def test_crc_known_value():
    # CRC16/Modbus 对 "123456789" 的标准结果
    assert proto.crc16_modbus(b"123456789") == 0x4B37


def test_frame_roundtrip():
    parser = proto.FrameParser()
    payload = proto.pack_aim(12.34, -5.67, proto.AimFlags.LASER_ON, 200)
    wire = proto.build_frame(proto.MsgId.AIM, payload, 7)
    frames = parser.feed(wire)
    assert len(frames) == 1
    f = frames[0]
    assert f.msg_id == proto.MsgId.AIM and f.seq == 7
    msg = proto.unpack_aim(f.payload)
    assert abs(msg["yaw_deg"] - 12.34) < 1e-6
    assert abs(msg["pitch_deg"] + 5.67) < 1e-6
    assert msg["flags"] == proto.AimFlags.LASER_ON
    assert msg["quality"] == 200


def test_parser_sticky_packets():
    """粘包：一次喂入 3 帧，必须解析出 3 帧；且跨调用断包也能拼起来。"""
    parser = proto.FrameParser()
    blob = b"".join(proto.build_frame(proto.MsgId.HEARTBEAT_G, b"", i)
                    for i in range(3))
    assert len(parser.feed(blob)) == 3
    wire = proto.build_frame(proto.MsgId.SET_ZERO, b"", 9)
    assert parser.feed(wire[:3]) == []
    got = parser.feed(wire[3:])
    assert len(got) == 1 and got[0].seq == 9


def test_parser_resync_and_crc_error():
    """截断/垃圾字节后必须能重新同步；CRC 错必须被丢弃。"""
    parser = proto.FrameParser()
    good = proto.build_frame(proto.MsgId.HEARTBEAT_G, b"", 1)
    assert parser.feed(b"\x00\x11\x22\xAA") == []
    assert len(parser.feed(good)) == 1
    bad = bytearray(proto.build_frame(proto.MsgId.HEARTBEAT_G, b"", 2))
    bad[-1] ^= 0xFF
    assert parser.feed(bytes(bad)) == []
    assert parser.crc_err == 1
    assert len(parser.feed(good)) == 1


def test_gimbal_state_roundtrip():
    payload = proto.pack_gimbal_state(6, 0, 12.5, -3.25, 1.0, 90.0, -10.0,
                                      2.5, -1.25, 0x1F, 123456)
    st = proto.unpack_gimbal_state(payload)
    assert st.state == 6
    assert abs(st.yaw_deg - 12.5) < 1e-6
    assert abs(st.pitch_deg + 3.25) < 1e-6
    assert abs(st.yaw_motor_deg - 90.0) < 1e-6
    assert st.uptime_ms == 123456
    assert st.closed_loop and st.speed_mode and st.encoder_ok and st.laser_on
    assert st.ready()


def test_car_messages():
    payload = proto.pack_car_state(1, 2, 3, 45, -150, 19800, 0x18, 0x01)
    st = proto.unpack_car_state(payload)
    assert st.state == 1 and st.lap_index == 2 and st.segment == 3
    assert st.progress == 45 and st.speed_mm_s == -150
    assert st.lap_time_ms == 19800 and st.on_corner
    assert st.running
    ev = proto.unpack_car_event(proto.pack_car_event(proto.CarEvent.CORNER, 2, 0))
    assert ev.event == proto.CarEvent.CORNER and ev.index == 2
    cmd = proto.unpack_car_cmd(proto.pack_car_cmd(proto.CarCmd.SET_LAPS, 2))
    assert cmd["cmd"] == proto.CarCmd.SET_LAPS and cmd["arg"] == 2


def test_aim_state_and_mode():
    st = proto.unpack_aim_state(proto.pack_aim_state(True, 4, 200, 0x05, -12.0, 34.0))
    assert st["locked"] and st["mode"] == 4 and st["quality"] == 200
    assert st["err_u"] == -12.0 and st["err_v"] == 34.0
    m = proto.unpack_mode(proto.pack_mode(proto.AimMode.AIM, 1))
    assert m["mode"] == proto.AimMode.AIM and m["arg"] == 1


def test_scale_clamps():
    assert proto.scale_deg(0.0) == 0
    assert proto.scale_deg(1.234) == 123
    assert proto.scale_deg(1e9) == 32767
    assert proto.scale_deg(-1e9) == -32768
    assert abs(proto.unscale_deg(123) - 1.23) < 1e-9


def test_pack_param():
    """0x15 PARAM 的字节编码必须与 H723 侧 gp_get_i16 的解析一致。

    这个测试是"跨语言契约"的守卫：改一边忘了另一边，在这里就会红。
    """
    # 12.50 -> 1250 = 0x04E2，小端 E2 04
    assert proto.pack_param(proto.ParamId.PITCH_KP, 12.5) == b"\x05\xE2\x04"
    # -1.00 -> -100 = 0xFF9C
    assert proto.pack_param(proto.ParamId.PITCH_FF_SIGN, -1.0) == b"\x03\x9C\xFF"
    # 0 -> 关掉前馈
    assert proto.pack_param(proto.ParamId.PITCH_FF_GAIN, 0.0) == b"\x04\x00\x00"
    # 超出 int16 自动夹住，不抛异常（现场手抖输入 1e6 也不能把链路搞崩）
    assert proto.pack_param(0x05, 1e6) == b"\x05\xFF\x7F"
    assert proto.pack_param(0x05, -1e6) == b"\x05\x00\x80"
    # 帧能正常装起来并被解析（msg_id 分配正确）
    wire = proto.build_frame(proto.MsgId.PARAM,
                             proto.pack_param(proto.ParamId.PITCH_FF_GAIN, 8.0), 3)
    frames = proto.FrameParser().feed(wire)
    assert len(frames) == 1 and frames[0].msg_id == 0x15
    assert frames[0].name == "PARAM"
