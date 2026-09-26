"""相机取帧。

关键设计：**只保留最新帧**。
Linux 上 V4L2 默认会缓冲好几帧，一旦处理比采集慢，缓冲区就堆积，
延迟从 30ms 涨到几百 ms —— 视觉伺服直接失效。所以：
  * CAP_PROP_BUFFERSIZE = 1
  * 取帧线程持续 read() 丢旧帧，只留最近一帧给主循环
另外把时间戳记在"读出的那一刻"，让控制环用真实 dt。
"""

import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

import cv2
import numpy as np

from .config import CameraConfig

__all__ = ["Camera", "Frame"]


def _fourcc(code: str) -> int:
    """FOURCC 编码。OpenCV 4.x 是 VideoWriter_fourcc，新版挪到了 VideoWriter.fourcc，
    两个都试一下，免得在地瓜派上因为版本差异直接崩。"""
    if hasattr(cv2, "VideoWriter_fourcc"):
        return cv2.VideoWriter_fourcc(*code[:4].ljust(4))
    return cv2.VideoWriter.fourcc(*code[:4].ljust(4))


def _fourcc_str(value: int) -> str:
    """把 FOURCC 的整数还原成 4 个字符，方便打印出来核对。"""
    try:
        return "".join(chr((int(value) >> (8 * i)) & 0xFF) for i in range(4))
    except Exception:                                          # noqa: BLE001
        return "?"


@dataclass
class Frame:
    image: np.ndarray
    t: float                    # 单调时钟时间戳（秒）
    seq: int


class Camera:
    """带后台取帧线程的相机封装。

    source:
      uvc   —— USB 摄像头（cv2.VideoCapture(index)）
      mipi  —— 地瓜派 MIPI 相机，device 填 GStreamer 管道字符串
      file  —— 视频文件（离线复现）
      sim   —— 由外部 frame_fn 提供（PC 全闭环仿真）
    """

    def __init__(self, cfg: CameraConfig,
                 frame_fn: Optional[Callable[[float], np.ndarray]] = None):
        self.cfg = cfg
        self.frame_fn = frame_fn
        self._cap: Optional[cv2.VideoCapture] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest: Optional[Frame] = None
        self._seq = 0
        self._read_seq = -1
        self.error = ""
        self.fps_measured = 0.0
        self.actual_fourcc = ""
        self.actual_fps = 0.0

    def open(self) -> bool:
        cfg = self.cfg
        if cfg.source == "sim":
            if self.frame_fn is None:
                self.error = "source=sim 但没有提供 frame_fn"
                return False
            return True
        if cfg.source == "mipi":
            # CAP_GSTREAMER 在某些 OpenCV 构建里没有，做个兜底，
            # 免得在板子上报一个莫名其妙的 AttributeError
            backend = getattr(cv2, "CAP_GSTREAMER", None)
            self._cap = (cv2.VideoCapture(str(cfg.device), backend)
                         if backend is not None else cv2.VideoCapture(str(cfg.device)))
        elif cfg.source == "file":
            self._cap = cv2.VideoCapture(str(cfg.device))
        else:
            # 显式指定 V4L2 后端，避免 OpenCV 自己挑到别的后端导致属性设置失效
            self._cap = cv2.VideoCapture(int(cfg.device), cv2.CAP_V4L2) \
                if hasattr(cv2, "CAP_V4L2") else cv2.VideoCapture(int(cfg.device))
            if self._cap.isOpened():
                # ⚠⚠ 属性设置顺序非常关键（实测踩过）：
                #   V4L2 下"设分辨率"会重新协商整条视频管线，
                #   把之前设好的 FOURCC 冲回默认的 YUYV。
                #   结果就是：配置里明明写了 MJPG，实际却跑在 YUYV 上，
                #   帧率从 60fps 掉到 9fps —— 视觉伺服直接废掉一半。
                #   正确顺序：分辨率 -> FOURCC -> 读回核对 -> 帧率。
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.width)
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.height)
                if cfg.fourcc:
                    want = _fourcc(cfg.fourcc)
                    self._cap.set(cv2.CAP_PROP_FOURCC, want)
                    if int(self._cap.get(cv2.CAP_PROP_FOURCC)) != want:
                        # 有些驱动第一次设不进去，再设一次
                        self._cap.set(cv2.CAP_PROP_FOURCC, want)
                self._cap.set(cv2.CAP_PROP_FPS, cfg.fps)
                # 把实际生效的格式/帧率记下来，启动时打印核对
                self.actual_fourcc = _fourcc_str(
                    int(self._cap.get(cv2.CAP_PROP_FOURCC)))
                self.actual_fps = float(self._cap.get(cv2.CAP_PROP_FPS))
        if self._cap is None or not self._cap.isOpened():
            self.error = "打开相机失败: source=%s device=%s" % (cfg.source, cfg.device)
            return False
        # ⚠ buffer_size 只在显式给了正数时才设。
        #   实测：设成 1 会让 V4L2 无法流水线采集，帧率从 50fps 掉到 17fps。
        #   我们靠取帧线程"持续丢弃旧帧"来防积压，不需要动驱动缓冲。
        if cfg.buffer_size and cfg.buffer_size > 0:
            try:
                self._cap.set(cv2.CAP_PROP_BUFFERSIZE, cfg.buffer_size)
            except Exception:
                pass
        if cfg.exposure > 0:
            # V4L2 下要先关自动曝光再写手动值，否则会被自动曝光覆盖
            try:
                self._cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
                self._cap.set(cv2.CAP_PROP_EXPOSURE, cfg.exposure)
            except Exception:
                pass
        try:
            self._cap.set(cv2.CAP_PROP_AUTOFOCUS, cfg.autofocus)
        except Exception:
            pass
        ok, frame = self._cap.read()
        if not ok or frame is None:
            self.error = "相机能打开但读不到帧"
            return False
        for _ in range(max(0, cfg.warmup_frames - 1)):
            self._cap.read()
        return True

    def start(self) -> bool:
        if not self.open():
            return False
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="camera", daemon=True)
        self._thread.start()
        return True

    def _loop(self) -> None:
        cfg = self.cfg
        period = 1.0 / max(1, cfg.fps)
        ema = None
        last_t = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if cfg.source == "sim":
                img = self.frame_fn(now)
                if img is None:
                    time.sleep(0.002)
                    continue
            else:
                ok, img = self._cap.read()
                if not ok or img is None:
                    if cfg.source == "file":       # 视频文件循环播放，方便离线复现
                        self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        continue
                    time.sleep(0.01)
                    continue
            if self._stop.is_set():
                break
            self._seq += 1
            with self._lock:
                self._latest = Frame(image=img, t=now, seq=self._seq)
            t_now = time.monotonic()
            elapsed = t_now - last_t
            last_t = t_now
            if elapsed > 1e-6:
                inst = 1.0 / elapsed
                ema = inst if ema is None else (0.9 * ema + 0.1 * inst)
                self.fps_measured = ema
            slack = (now + period) - time.monotonic()
            if slack > 0:
                time.sleep(min(slack, period))

    def read(self) -> Tuple[Optional[Frame], bool]:
        """返回 (新帧 or None, 是否有任何帧)。只有新帧才返回 Frame 对象。"""
        with self._lock:
            frame = self._latest
            if frame is None:
                return None, False
            if frame.seq == self._read_seq:
                return None, True
            self._read_seq = frame.seq
            return frame, True

    def latest(self) -> Optional[Frame]:
        with self._lock:
            return self._latest

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def resolution(self) -> Tuple[int, int]:
        if self._cap is not None:
            w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if w > 0 and h > 0:
                return w, h
        return self.cfg.width, self.cfg.height
