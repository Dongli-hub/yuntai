"""串口协议：帧格式 + CRC16 + 各消息的打包/解包。

帧格式（两条链路共用）::

    +------+------+--------+-----+-----+---------------+--------+--------+
    | 0xAA | 0x55 | msg_id | seq | len | payload (len) | crc_lo | crc_hi |
    +------+------+--------+-----+-----+---------------+--------+--------+
                    +--------- CRC16/Modbus 覆盖这一段 ---------+

设计要点：
  * 固定两字节帧头 + 长度 + CRC16：粘包、断包、干扰都能可靠识别与重同步
  * 小端编码（STM32 与 x86/ARM64 都是小端，memcpy 直接可用）
  * 偏置类量统一用 int16 表示 degree*100（0.01° 分辨率），
    yaw 限幅 ±327.67° 足够（实际不会超过 ±180）
  * 所有消息都是定长，STM32 侧解析不需要动态内存

本模块两边都要用：地瓜派（Python 本文件）和 H723/RCT6（C 侧按同样字节序实现，
见 docs/01_通信协议.md 里附的 C 参考实现）。
"""

import struct
from dataclasses import dataclass
from typing import List, Optional

__all__ = [
    "SOF", "HEADER_LEN", "MAX_PAYLOAD",
    "MsgId", "AimMode", "CarCmd", "CarEvent", "AimFlags",
    "crc16_modbus", "build_frame", "Frame", "FrameParser",
    "pack_aim", "unpack_aim", "pack_mode", "pack_gimbal_state", "unpack_gimbal_state",
    "pack_ack", "unpack_ack", "pack_param", "ParamId",
    "pack_car_cmd", "unpack_car_cmd", "pack_aim_state", "unpack_aim_state",
    "pack_car_state", "unpack_car_state", "pack_car_event", "unpack_car_event",
    "scale_deg", "unscale_deg",
]

SOF = b"\xAA\x55"
HEADER_LEN = 5          # AA 55 msg seq len
CRC_LEN = 2
MAX_PAYLOAD = 240


class MsgId:
    """消息 ID 分配。

    0x1x = 地瓜派 -> H723
    0x2x = 地瓜派 -> RCT6
    0x9x = H723  -> 地瓜派
    0xAx = RCT6  -> 地瓜派
    """

    AIM = 0x10
    MODE = 0x11
    HEARTBEAT_G = 0x12
    SET_ZERO = 0x13
    UNWIND = 0x14
    PARAM = 0x15

    CAR_CMD = 0x20
    AIM_STATE = 0x21
    HEARTBEAT_C = 0x22

    GIMBAL_STATE = 0x90
    ACK = 0x91
    VERSION = 0x92
    TEXT = 0x93

    CAR_STATE = 0xA0
    CAR_EVENT = 0xA1

    NAME = {
        AIM: "AIM", MODE: "MODE", HEARTBEAT_G: "HB_G", SET_ZERO: "SET_ZERO",
        UNWIND: "UNWIND", PARAM: "PARAM",
        CAR_CMD: "CAR_CMD", AIM_STATE: "AIM_STATE",
        HEARTBEAT_C: "HB_C", GIMBAL_STATE: "GIMBAL_STATE", ACK: "ACK",
        VERSION: "VERSION", CAR_STATE: "CAR_STATE", CAR_EVENT: "CAR_EVENT",
        TEXT: "TEXT",
    }

    @classmethod
    def name(cls, msg_id: int) -> str:
        return cls.NAME.get(msg_id, "0x%02X" % msg_id)


class ParamId:
    """0x15 PARAM 的 param_id。

    单位约定：value = 真实值 × 100（H723 侧同样按 ×100 解析）。
    这些参数只存在 H723 的 RAM 里，重新上电恢复代码里的默认值 ——
    故意的：避免"上次调歪的值被记住"，让每次上电的状态都是已知的。
    调好之后要把数值写回固件默认值再烧一次固化。
    """

    DUMP = 0x00             # 让 H723 回一行当前参数
    YAW_FF_SIGN = 0x01
    YAW_FF_GAIN = 0x02
    PITCH_FF_SIGN = 0x03    # ±1（俯仰振荡第一嫌疑人）
    PITCH_FF_GAIN = 0x04    # 0 = 关掉前馈
    PITCH_KP = 0x05
    PITCH_KI = 0x06
    PITCH_KD = 0x07
    PITCH_ILIM = 0x08
    PITCH_OUT_RPM = 0x09
    PITCH_PLAT_SIGN = 0x0A   # 平台俯仰补偿：0=关，±1=开并定方向


class AimMode:
    """H723 的工作模式。"""

    IDLE = 0        # 不使能、不控制（电机自由）
    STAB = 1        # 只做姿态稳定，忽略偏置（掉线后的安全模式）
    AIM = 2         # 稳定 + 应用地瓜派下发的瞄准偏置
    UNWIND = 3      # 解绕：把相对偏航角慢慢转回 0，激光必须关闭
    ESTOP = 4       # 急停：停止发速度指令

    NAME = {IDLE: "IDLE", STAB: "STAB", AIM: "AIM", UNWIND: "UNWIND", ESTOP: "ESTOP"}

    @classmethod
    def name_of(cls, mode: int) -> str:
        return cls.NAME.get(mode, "?%d" % mode)


class AimFlags:
    """AIM 消息里的 flags 位定义。"""

    LASER_ON = 0x01
    AIM_VALID = 0x02        # 视觉本轮有效（跟踪正常）
    BOOST = 0x04            # 拐角加强
    DRAWING = 0x08          # 正在画圆
    LOCKED = 0x10           # 已锁定（给 H723 只做记录/联锁用）

    @staticmethod
    def describe(flags: int) -> str:
        names = []
        for bit, label in ((AimFlags.LASER_ON, "laser"),
                           (AimFlags.AIM_VALID, "valid"),
                           (AimFlags.BOOST, "boost"),
                           (AimFlags.DRAWING, "draw"),
                           (AimFlags.LOCKED, "locked")):
            if flags & bit:
                names.append(label)
        return "|".join(names) if names else "-"


class CarCmd:
    START = 1
    STOP = 2
    SET_LAPS = 3
    ZERO_TIMER = 4
    SET_SPEED = 5

    NAME = {START: "START", STOP: "STOP", SET_LAPS: "SET_LAPS",
            ZERO_TIMER: "ZERO_TIMER", SET_SPEED: "SET_SPEED"}


class CarEvent:
    CORNER = 1          # 过一个拐角
    LAP_DONE = 2        # 完成一圈
    STARTED = 3
    STOPPED = 4

    NAME = {CORNER: "CORNER", LAP_DONE: "LAP_DONE", STARTED: "STARTED",
            STOPPED: "STOPPED"}


# --------------------------------------------------------------------------
# CRC16/Modbus（多项式 0xA001），带查表加速
# --------------------------------------------------------------------------
def _build_crc_table():
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
        table.append(crc)
    return tuple(table)


_CRC_TABLE = _build_crc_table()


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc = (crc >> 8) ^ _CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc & 0xFFFF


# --------------------------------------------------------------------------
# 帧
# --------------------------------------------------------------------------
def build_frame(msg_id: int, payload: bytes = b"", seq: int = 0) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload 太长: %d" % len(payload))
    body = struct.pack("<BBB", msg_id & 0xFF, seq & 0xFF, len(payload)) + payload
    return SOF + body + struct.pack("<H", crc16_modbus(body))


@dataclass
class Frame:
    msg_id: int
    seq: int
    payload: bytes

    @property
    def name(self) -> str:
        return MsgId.name(self.msg_id)


class FrameParser:
    """流式帧解析器。

    用法::

        parser = FrameParser()
        for frame in parser.feed(serial_bytes):
            handle(frame)

    统计量：crc_err / bad_len / resync 次数，现场排障时非常有用
    （crc_err 持续增长 = 波特率不匹配或线太长；resync = 有干扰或对端错帧）。
    """

    def __init__(self, max_buffer: int = 4096):
        self._buf = bytearray()
        self.max_buffer = max_buffer
        self.crc_err = 0
        self.bad_len = 0
        self.resync = 0
        self.frames_ok = 0

    def reset(self) -> None:
        self._buf.clear()
        self.crc_err = 0
        self.bad_len = 0
        self.resync = 0
        self.frames_ok = 0

    def feed(self, data: bytes) -> List[Frame]:
        self._buf.extend(data)
        out: List[Frame] = []
        while True:
            start = self._buf.find(SOF)
            if start < 0:
                # 没有帧头：只保留最后一个字节（可能是半个帧头）
                if len(self._buf) > 1:
                    self.resync += 1
                    del self._buf[:-1]
                break
            if start > 0:
                self.resync += 1
                del self._buf[:start]
            if len(self._buf) < HEADER_LEN:
                break
            length = self._buf[4]
            if length > MAX_PAYLOAD:
                self.bad_len += 1
                del self._buf[:2]          # 丢掉这个假帧头，继续找
                continue
            total = HEADER_LEN + length + CRC_LEN
            if len(self._buf) < total:
                break
            body = bytes(self._buf[2:HEADER_LEN + length])
            crc_rx = struct.unpack_from("<H", self._buf, HEADER_LEN + length)[0]
            if crc16_modbus(body) != crc_rx:
                self.crc_err += 1
                del self._buf[:2]
                continue
            out.append(Frame(msg_id=body[0], seq=body[1], payload=body[3:]))
            self.frames_ok += 1
            del self._buf[:total]
        if len(self._buf) > self.max_buffer:
            del self._buf[:-self.max_buffer]
        return out

    def stats(self) -> str:
        return ("ok=%d crc_err=%d bad_len=%d resync=%d buf=%d"
                % (self.frames_ok, self.crc_err, self.bad_len, self.resync, len(self._buf)))


# --------------------------------------------------------------------------
# 角度编码
# --------------------------------------------------------------------------
def scale_deg(deg: float) -> int:
    """度 -> int16(度*100)，并夹到 int16 范围。"""
    raw = int(round(float(deg) * 100.0))
    return max(-32768, min(32767, raw))


def unscale_deg(raw: int) -> float:
    return float(raw) / 100.0


# --------------------------------------------------------------------------
# 地瓜派 -> H723
# --------------------------------------------------------------------------
_AIM_FMT = "<hhBB"          # yaw*100, pitch*100, flags, quality


def pack_aim(yaw_deg: float, pitch_deg: float, flags: int, quality: int = 0) -> bytes:
    return struct.pack(_AIM_FMT, scale_deg(yaw_deg), scale_deg(pitch_deg),
                       flags & 0xFF, max(0, min(255, int(quality))) & 0xFF)


def unpack_aim(payload: bytes):
    yaw, pitch, flags, quality = struct.unpack(_AIM_FMT, payload)
    return {"yaw_deg": unscale_deg(yaw), "pitch_deg": unscale_deg(pitch),
            "flags": flags, "quality": quality}


def pack_mode(mode: int, arg: int = 0) -> bytes:
    return struct.pack("<BB", mode & 0xFF, arg & 0xFF)


def unpack_mode(payload: bytes):
    mode, arg = struct.unpack("<BB", payload)
    return {"mode": mode, "arg": arg}


def pack_unwind(speed_dps: float = 120.0) -> bytes:
    """解绕指令：speed_dps 是解绕角速度（°/s）。默认 120°/s = 20rpm。"""
    return struct.pack("<H", max(1, min(65535, int(round(speed_dps * 10)))))


def pack_param(param_id: int, value: float) -> bytes:
    """运行时就地调参（0x15）：param_id(u8) + value(i16, 真实值×100)。

    这是给我们自己现场排障用的，不是比赛流程的一部分：
    俯仰振荡这类问题必须"改一个数 -> 掰一下 -> 看结果"，
    重新烧写一轮要两分钟，根本试不出组合。

    例：pack_param(ParamId.PITCH_FF_GAIN, 0.0)   -> 关掉俯仰前馈
        pack_param(ParamId.PITCH_FF_SIGN, -1.0)  -> 前馈符号取反
    """
    raw = int(round(float(value) * 100.0))
    return struct.pack("<Bh", param_id & 0xFF, max(-32768, min(32767, raw)))


# state, fault, yaw, pitch, roll, yaw_motor, pitch_motor, gyro_y, gyro_z, flags, uptime
_GIMBAL_FMT = "<BBhhhhhhhBI"


def pack_gimbal_state(state: int, fault: int, yaw_deg: float, pitch_deg: float,
                      roll_deg: float, yaw_motor_deg: float, pitch_motor_deg: float,
                      gyro_y_dps: float, gyro_z_dps: float, flags: int,
                      uptime_ms: int) -> bytes:
    return struct.pack(
        _GIMBAL_FMT, state & 0xFF, fault & 0xFF,
        scale_deg(yaw_deg), scale_deg(pitch_deg), scale_deg(roll_deg),
        scale_deg(yaw_motor_deg), scale_deg(pitch_motor_deg),
        scale_deg(gyro_y_dps), scale_deg(gyro_z_dps),
        flags & 0xFF, int(uptime_ms) & 0xFFFFFFFF,
    )


@dataclass
class GimbalState:
    state: int = 0
    fault: int = 0
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    yaw_motor_deg: float = 0.0
    pitch_motor_deg: float = 0.0
    gyro_y_dps: float = 0.0
    gyro_z_dps: float = 0.0
    flags: int = 0
    uptime_ms: int = 0

    @property
    def closed_loop(self) -> bool:
        return bool(self.flags & 0x01)

    @property
    def speed_mode(self) -> bool:
        return bool(self.flags & 0x02)

    @property
    def encoder_ok(self) -> bool:
        return bool(self.flags & 0x04)

    @property
    def laser_on(self) -> bool:
        return bool(self.flags & 0x08)

    def ready(self) -> bool:
        """H723 是否已经可以接受瞄准偏置。

        判据用 flags 里专门的 READY 位（bit4），**不要用 state 数值比大小**：
        state 是固件内部枚举，改一次状态机就可能变。
        （踩过：状态机去掉一个 INIT 状态后 RUNNING 从 6 变成 5，
          上位机一直等不到，表现是卡在 BOOT_WAIT。）
        """
        return bool(self.flags & 0x10)

    def describe(self) -> str:
        return ("state=%d fault=%d yaw=%.2f pitch=%.2f roll=%.2f "
                "motor=(%.1f,%.1f) gyro=(%.1f,%.1f) flags=0x%02X up=%dms"
                % (self.state, self.fault, self.yaw_deg, self.pitch_deg,
                   self.roll_deg, self.yaw_motor_deg, self.pitch_motor_deg,
                   self.gyro_y_dps, self.gyro_z_dps, self.flags, self.uptime_ms))


def unpack_gimbal_state(payload: bytes) -> GimbalState:
    values = struct.unpack(_GIMBAL_FMT, payload)
    return GimbalState(
        state=values[0], fault=values[1],
        yaw_deg=unscale_deg(values[2]), pitch_deg=unscale_deg(values[3]),
        roll_deg=unscale_deg(values[4]), yaw_motor_deg=unscale_deg(values[5]),
        pitch_motor_deg=unscale_deg(values[6]), gyro_y_dps=unscale_deg(values[7]),
        gyro_z_dps=unscale_deg(values[8]), flags=values[9], uptime_ms=values[10],
    )


def pack_ack(msg_id: int, code: int) -> bytes:
    return struct.pack("<BB", msg_id & 0xFF, code & 0xFF)


def pack_text(text: str) -> bytes:
    """H723 侧的调试文本（替代原来的串口 printf，不会污染二进制协议流）。"""
    return text.encode("utf-8", "replace")[:64]


def unpack_ack(payload: bytes):
    msg_id, code = struct.unpack("<BB", payload)
    return {"msg_id": msg_id, "code": code}


# --------------------------------------------------------------------------
# 地瓜派 -> RCT6
# --------------------------------------------------------------------------
def pack_car_cmd(cmd: int, arg: int = 0, arg2: int = 0) -> bytes:
    return struct.pack("<BBBB", cmd & 0xFF, arg & 0xFF, arg2 & 0xFF, 0)


def unpack_car_cmd(payload: bytes):
    cmd, arg, arg2, _ = struct.unpack("<BBBB", payload)
    return {"cmd": cmd, "arg": arg, "arg2": arg2}


_AIM_STATE_FMT = "<BBBBhh"


def pack_aim_state(locked: bool, mode: int, quality: int, flags: int,
                   err_u: float, err_v: float) -> bytes:
    return struct.pack(_AIM_STATE_FMT, 1 if locked else 0, mode & 0xFF,
                       max(0, min(255, int(quality))), flags & 0xFF,
                       int(max(-32768, min(32767, round(err_u)))),
                       int(max(-32768, min(32767, round(err_v)))))


def unpack_aim_state(payload: bytes):
    locked, mode, quality, flags, err_u, err_v = struct.unpack(_AIM_STATE_FMT, payload)
    return {"locked": bool(locked), "mode": mode, "quality": quality,
            "flags": flags, "err_u": float(err_u), "err_v": float(err_v)}


# --------------------------------------------------------------------------
# RCT6 -> 地瓜派
# --------------------------------------------------------------------------
_CAR_STATE_FMT = "<BBBBhHBB"


def pack_car_state(state: int, lap_index: int, segment: int, progress: int,
                   speed_mm_s: int, lap_time_ms: int, gray_bits: int,
                   flags: int) -> bytes:
    return struct.pack(_CAR_STATE_FMT, state & 0xFF, lap_index & 0xFF,
                       segment & 0xFF, progress & 0xFF,
                       int(max(-32768, min(32767, speed_mm_s))),
                       int(max(0, min(65535, lap_time_ms))) & 0xFFFF,
                       gray_bits & 0xFF, flags & 0xFF)


@dataclass
class CarState:
    state: int = 0
    lap_index: int = 0
    segment: int = 0
    progress: int = 0           # 当前段的进度 0..100 (%)
    speed_mm_s: int = 0
    lap_time_ms: int = 0
    gray_bits: int = 0
    flags: int = 0

    @property
    def running(self) -> bool:
        return bool(self.state & 0x01)

    @property
    def on_corner(self) -> bool:
        return bool(self.flags & 0x01)

    def describe(self) -> str:
        return ("state=%d lap=%d seg=%d prog=%d%% v=%dmm/s t=%dms gray=0x%02X f=0x%02X"
                % (self.state, self.lap_index, self.segment, self.progress,
                   self.speed_mm_s, self.lap_time_ms, self.gray_bits, self.flags))


def unpack_car_state(payload: bytes) -> CarState:
    values = struct.unpack(_CAR_STATE_FMT, payload)
    return CarState(state=values[0], lap_index=values[1], segment=values[2],
                    progress=values[3], speed_mm_s=values[4], lap_time_ms=values[5],
                    gray_bits=values[6], flags=values[7])


_CAR_EVENT_FMT = "<BBH"


def pack_car_event(event: int, index: int = 0, value: int = 0) -> bytes:
    return struct.pack(_CAR_EVENT_FMT, event & 0xFF, index & 0xFF,
                       int(max(0, min(65535, value))) & 0xFFFF)


@dataclass
class CarEventMsg:
    event: int = 0
    index: int = 0
    value: int = 0

    @property
    def name(self) -> str:
        return CarEvent.NAME.get(self.event, "?%d" % self.event)


def unpack_car_event(payload: bytes) -> CarEventMsg:
    event, index, value = struct.unpack(_CAR_EVENT_FMT, payload)
    return CarEventMsg(event=event, index=index, value=value)


def pack_heartbeat(t_ms: Optional[int] = None) -> bytes:
    if t_ms is None:
        return b""
    return struct.pack("<I", int(t_ms) & 0xFFFFFFFF)
