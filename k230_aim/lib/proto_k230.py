# -*- coding: utf-8 -*-
"""proto_k230.py —— 与 H723 的串口协议（K230 / MicroPython 版）

和 rdk_aim/eaim/protocol.py 完全同格式：
    AA 55 | msg_id | seq | len | payload | crc_lo crc_hi
    CRC16/Modbus（0xA001），覆盖 msg_id..payload

可在电脑上用 CPython 导入（做离线自测），也可在 K230 上运行。
"""
try:
    import ustruct as struct
except ImportError:
    import struct

SOF = b"\xAA\x55"
HEADER_LEN = 5
CRC_LEN = 2
MAX_PAYLOAD = 240

# 地瓜派 -> H723
MSG_AIM = 0x10
MSG_MODE = 0x11
MSG_HEARTBEAT_G = 0x12
MSG_SET_ZERO = 0x13
MSG_UNWIND = 0x14
MSG_PARAM = 0x15
# H723 -> 地瓜派
MSG_GIMBAL_STATE = 0x90
MSG_ACK = 0x91
MSG_VERSION = 0x92
MSG_TEXT = 0x93

MODE_IDLE = 0
MODE_STAB = 1
MODE_AIM = 2
MODE_UNWIND = 3
MODE_ESTOP = 4

FLAG_LASER_ON = 0x01
FLAG_AIM_VALID = 0x02
FLAG_BOOST = 0x04
FLAG_DRAWING = 0x08
FLAG_LOCKED = 0x10

ST_FLAG_READY = 0x10        # 遥测 flags bit4 = H723 已就绪

# 0x15 PARAM 的 param_id（现场调参用）
PARAM_DUMP = 0x00
PARAM_YAW_FF_SIGN = 0x01
PARAM_YAW_FF_GAIN = 0x02
PARAM_PITCH_FF_SIGN = 0x03
PARAM_PITCH_FF_GAIN = 0x04
PARAM_PITCH_KP = 0x05
PARAM_PITCH_KI = 0x06
PARAM_PITCH_KD = 0x07
PARAM_PITCH_ILIM = 0x08
PARAM_PITCH_OUT_RPM = 0x09
PARAM_PITCH_PLAT_SIGN = 0x0A
PARAM_YAW_STICTION_RPM = 0x0B
PARAM_YAW_ILIM = 0x0C


def _crc_table():
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if (c & 1) else (c >> 1)
        t.append(c)
    return t


_CRC = _crc_table()


def crc16_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ _CRC[(crc ^ b) & 0xFF]
    return crc & 0xFFFF


def build_frame(msg_id, payload=b"", seq=0):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = struct.pack("<BBB", msg_id & 0xFF, seq & 0xFF,
                       len(payload)) + payload
    return SOF + body + struct.pack("<H", crc16_modbus(body))


class FrameParser(object):
    """流式解析器（和地瓜派版同样的重同步策略）。"""

    def __init__(self, max_buffer=2048):
        self.buf = bytearray()
        self.max_buffer = max_buffer
        self.crc_err = 0
        self.bad_len = 0
        self.resync = 0
        self.frames_ok = 0

    def feed(self, data):
        out = []
        self.buf.extend(data)
        while True:
            i = self.buf.find(SOF)
            if i < 0:
                if len(self.buf) > 1:
                    self.resync += 1
                    # MicroPython 的 bytearray 不支持切片删除（del buf[x:]），
                    # 只能用重新切片，否则报 TypeError
                    self.buf = self.buf[-1:]
                break
            if i > 0:
                self.resync += 1
                self.buf = self.buf[i:]
            if len(self.buf) < HEADER_LEN:
                break
            length = self.buf[4]
            if length > MAX_PAYLOAD:
                self.bad_len += 1
                self.buf = self.buf[2:]
                continue
            total = HEADER_LEN + length + CRC_LEN
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:HEADER_LEN + length])
            crc_rx = self.buf[HEADER_LEN + length] | \
                (self.buf[HEADER_LEN + length + 1] << 8)
            if crc16_modbus(body) != crc_rx:
                self.crc_err += 1
                self.buf = self.buf[2:]
                continue
            self.frames_ok += 1
            out.append((body[0], body[1], bytes(body[3:])))
            self.buf = self.buf[total:]
        if len(self.buf) > self.max_buffer:
            self.buf = self.buf[-self.max_buffer:]
        return out

    def stats(self):
        return "ok=%d crc_err=%d bad_len=%d resync=%d" % (
            self.frames_ok, self.crc_err, self.bad_len, self.resync)


def scale_deg(deg):
    raw = int(round(float(deg) * 100.0))
    return max(-32768, min(32767, raw))


def pack_aim(yaw_deg, pitch_deg, flags, quality=0):
    return struct.pack("<hhBB", scale_deg(yaw_deg), scale_deg(pitch_deg),
                       flags & 0xFF, max(0, min(255, int(quality))) & 0xFF)


def pack_mode(mode, arg=0):
    return struct.pack("<BB", mode & 0xFF, arg & 0xFF)


def pack_unwind(speed_dps=120.0):
    return struct.pack("<H", max(1, min(65535, int(round(speed_dps * 10)))))


def pack_param(param_id, value):
    raw = int(round(float(value) * 100.0))
    return struct.pack("<Bh", param_id & 0xFF,
                       max(-32768, min(32767, raw)))


_GIMBAL_FMT = "<BBhhhhhhhBI"


class GimbalState(object):
    """只需属性访问，避免在 MicroPython 里用 dataclass。"""

    __slots__ = ("state", "fault", "yaw_deg", "pitch_deg", "roll_deg",
                 "yaw_motor_deg", "pitch_motor_deg", "gyro_y_dps",
                 "gyro_z_dps", "flags", "uptime_ms")

    def __init__(self, v):
        self.state = v[0]
        self.fault = v[1]
        self.yaw_deg = v[2] / 100.0
        self.pitch_deg = v[3] / 100.0
        self.roll_deg = v[4] / 100.0
        self.yaw_motor_deg = v[5] / 100.0
        self.pitch_motor_deg = v[6] / 100.0
        self.gyro_y_dps = v[7] / 100.0
        self.gyro_z_dps = v[8] / 100.0
        self.flags = v[9]
        self.uptime_ms = v[10]

    def ready(self):
        return bool(self.flags & ST_FLAG_READY)

    def laser_on(self):
        return bool(self.flags & 0x08)

    def describe(self):
        return ("state=%d fault=%d yaw=%.2f pitch=%.2f roll=%.2f "
                "motor=(%.1f,%.1f) gyro=(%.1f,%.1f) flags=0x%02X up=%dms"
                % (self.state, self.fault, self.yaw_deg, self.pitch_deg,
                   self.roll_deg, self.yaw_motor_deg, self.pitch_motor_deg,
                   self.gyro_y_dps, self.gyro_z_dps, self.flags,
                   self.uptime_ms))


def unpack_gimbal_state(payload):
    return GimbalState(struct.unpack(_GIMBAL_FMT, payload))


def pack_ack(msg_id, code):
    return struct.pack("<BB", msg_id & 0xFF, code & 0xFF)


def pack_text(text):
    return text.encode("utf-8", "replace")[:64]


def unpack_ack(payload):
    msg_id, code = struct.unpack("<BB", payload)
    return {"msg_id": msg_id, "code": code}
