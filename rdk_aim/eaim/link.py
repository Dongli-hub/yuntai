"""串口链路：线程收发 + 自动重连 + 统计 + 看门狗。

两条链路（H723 云台、RCT6 小车）共用这一个类，只是消息 ID 不同。

几个工程上的要点：
  * **接收在独立线程**：主循环只做"取帧 -> 处理 -> 下发"，
    绝不能被串口读写阻塞。读线程只管把解析好的 Frame 丢进队列。
  * **自动重连**：USB 转串口在插拔/干扰后可能掉线，
    重连失败的日志很有用（现场最常见的故障就是这个）。
  * **rx_age() 看门狗**：上层用它判断对端是否还活着，
    超时就降级到安全模式（H723 侧也一样有 AIM 断流看门狗）。
  * 端口写 "loop://" 等带 "://" 的形式时走 pyserial 的 URL 模式，
    这样在 PC 上不接硬件也能测链路逻辑。
"""

import threading
import time
from collections import deque
from typing import Deque, List, Optional

from . import protocol as proto
from .config import LinkConfig
from .protocol import Frame

__all__ = ["SerialLink", "make_link"]


class SerialLink:
    def __init__(self, cfg: LinkConfig, name: str = "link"):
        self.cfg = cfg
        self.name = name
        self.parser = proto.FrameParser()
        self._serial = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._tx_lock = threading.Lock()
        self._rx_queue: Deque[Frame] = deque(maxlen=512)
        self._seq = 0
        self._last_rx = 0.0
        self.tx_frames = 0
        self.tx_bytes = 0
        self.rx_frames = 0
        self.tx_errors = 0
        self.reconnects = 0
        self.error = ""
        self.opened = False

    # ------------------------------------------------------------------
    def start(self) -> bool:
        if not self.cfg.enable:
            self.error = "链路已禁用"
            return False
        self.opened = self._open()
        if not self.opened:
            return False
        self._stop.clear()
        self._thread = threading.Thread(target=self._rx_loop,
                                        name="rx-" + self.name, daemon=True)
        self._thread.start()
        return True

    def _open(self) -> bool:
        try:
            import serial                      # 延迟导入：没装 pyserial 也能 import 本模块
        except Exception as exc:
            self.error = "没装 pyserial: %s" % exc
            return False
        port = str(self.cfg.port)
        timeout = 0.02
        try:
            if "://" in port:
                self._serial = serial.serial_for_url(port, self.cfg.baud,
                                                     timeout=timeout)
            else:
                self._serial = serial.Serial(port, self.cfg.baud, timeout=timeout)
            self.error = ""
            return True
        except Exception as exc:
            self.error = "打开 %s 失败: %s" % (port, exc)
            return False

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            if self._serial is None:
                time.sleep(0.5)
                if self._open():
                    self.reconnects += 1
                continue
            try:
                waiting = self._serial.in_waiting
                data = self._serial.read(waiting if waiting > 0 else 1)
            except Exception as exc:
                self.error = "读 %s 失败: %s" % (self.cfg.port, exc)
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
                continue
            if not data:
                continue
            for frame in self.parser.feed(data):
                self._rx_queue.append(frame)
                self.rx_frames += 1
            self._last_rx = time.monotonic()

    # ------------------------------------------------------------------
    def send(self, msg_id: int, payload: bytes = b"", seq: Optional[int] = None) -> bool:
        if not self.cfg.enable:
            return False
        if seq is None:
            seq = self._seq
            self._seq = (self._seq + 1) & 0xFF
        wire = proto.build_frame(msg_id, payload, seq)
        with self._tx_lock:
            if self._serial is None:
                self.tx_errors += 1
                return False
            try:
                self._serial.write(wire)
            except Exception as exc:
                self.error = "写 %s 失败: %s" % (self.cfg.port, exc)
                self.tx_errors += 1
                return False
        self.tx_frames += 1
        self.tx_bytes += len(wire)
        return True

    def read_frames(self) -> List[Frame]:
        out = []
        while self._rx_queue:
            out.append(self._rx_queue.popleft())
        return out

    def quick_send(self, msg_id: int, payload: bytes = b"") -> bool:
        """不经过线程的直发（用于退出前发安全指令这种一次性动作）。"""
        return self.send(msg_id, payload)

    def healthy(self) -> bool:
        if not self.cfg.enable:
            return False
        return self.rx_age() <= self.cfg.telemetry_timeout_s

    def rx_age(self) -> float:
        if self._last_rx <= 0.0:
            return 1e9
        return time.monotonic() - self._last_rx

    def stats(self) -> str:
        return ("%s %s@%d tx=%d rx=%d err=%d reconn=%d age=%.2fs | %s"
                % (self.name, self.cfg.port, self.cfg.baud, self.tx_frames,
                   self.rx_frames, self.tx_errors, self.reconnects,
                   self.rx_age(), self.parser.stats()))

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        self.opened = False


def make_link(cfg: LinkConfig, name: str):
    """按配置返回链路对象；enable=False 时也返回一个"哑链路"，便于统一调用。"""
    if not cfg.enable:
        return DummyLink(name)
    return SerialLink(cfg, name)


class DummyLink:
    """disabled 链路的占位实现，接口与 SerialLink 一致。"""

    def __init__(self, name: str):
        self.name = name
        self.parser = proto.FrameParser()
        self.tx_frames = 0

    def start(self) -> bool:
        return True

    def send(self, msg_id: int, payload: bytes = b"", seq: Optional[int] = None) -> bool:
        self.tx_frames += 1
        return True

    def read_frames(self):
        return []

    def healthy(self) -> bool:
        return False

    def rx_age(self) -> float:
        return 1e9

    def stats(self) -> str:
        return "%s disabled" % self.name

    def close(self) -> None:
        pass

