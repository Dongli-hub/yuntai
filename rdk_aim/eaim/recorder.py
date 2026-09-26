"""运行记录：CSV 日志（+ 可选录像）。

现场调参时最有价值的东西就是"上一次跑完的误差曲线"：
  * 误差始终不为 0 且不变号 -> 还没收敛（kp 太小 / 有静差）
  * 误差周期性变号 -> 增益太大，振荡
  * 拐角处误差突然变大再慢慢回来 -> 超前补偿不够
  * 误差突然跳到几百 px -> 目标丢了（看 valid 列）
"""

import csv
import os
import time
from typing import Dict, List, Optional

import cv2
import numpy as np

__all__ = ["Recorder"]

FIELDS = [
    "t", "state", "target_ok", "src", "conf", "dist_m",
    "tu", "tv", "su", "sv", "err_u", "err_v", "err_norm",
    "yaw", "pitch", "valid", "coast", "locked", "boost", "quality", "laser",
    "spot_area", "ring_score",
    "gz_state", "gz_ready", "gz_yaw", "gz_pitch", "gz_motor",
    "car_lap", "car_seg", "car_prog", "car_run",
    "draw_phase", "draw_err",
    "sim_miss_mm", "sim_on_paper", "sim_spot_x", "sim_spot_y", "sim_d2_mm",
]


class Recorder:
    def __init__(self, run_dir: str, name: str = "run", video: bool = False,
                 frame_size: Optional[tuple] = None, fps: float = 30.0):
        os.makedirs(run_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.base = os.path.join(run_dir, "%s_%s" % (name, stamp))
        self.csv_path = self.base + ".csv"
        self.video_path = self.base + ".mp4" if video else None
        self._fh = open(self.csv_path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._fh, fieldnames=FIELDS,
                                      extrasaction="ignore")
        self._writer.writeheader()
        self._writerows_since_flush = 0
        self._video = None
        if self.video_path and frame_size:
            from .camera import _fourcc
            fourcc = _fourcc("mp4v")
            self._video = cv2.VideoWriter(self.video_path, fourcc,
                                          max(1.0, fps),
                                          (int(frame_size[0]), int(frame_size[1])))
        self.rows: List[Dict[str, float]] = []
        self.t0 = time.monotonic()

    # ------------------------------------------------------------------
    def log(self, **kwargs) -> None:
        row = dict.fromkeys(FIELDS, "")
        for key, value in kwargs.items():
            if key in row:
                row[key] = value
        self._writer.writerow(row)
        self._writerows_since_flush += 1
        if self._writerows_since_flush >= 30:
            self._fh.flush()
            self._writerows_since_flush = 0
        if "err_norm" in kwargs:
            self.rows.append({"err_norm": float(kwargs["err_norm"] or 0.0),
                              "valid": float(kwargs.get("valid") or 0),
                              "t": float(kwargs.get("t") or 0.0)})

    def write_frame(self, frame: np.ndarray) -> None:
        if self._video is not None and frame is not None:
            self._video.write(frame)

    def summary(self) -> str:
        if not self.rows:
            return "无误差样本"
        errs = np.array([r["err_norm"] for r in self.rows], dtype=np.float64)
        vals = np.array([r["valid"] for r in self.rows], dtype=np.float64)
        if len(errs) == 0:
            return "无误差样本"
        rms = float(np.sqrt(np.mean(errs ** 2)))
        return ("样本=%d 有效率=%.0f%% |e| RMS=%.2fpx 中位=%.2fpx 最大=%.2fpx"
                % (len(errs), 100.0 * vals.mean(), rms,
                   float(np.median(errs)), float(errs.max())))

    def close(self) -> None:
        try:
            self._fh.flush()
            self._fh.close()
        except Exception:
            pass
        if self._video is not None:
            try:
                self._video.release()
            except Exception:
                pass
            self._video = None

    def paths(self) -> Dict[str, str]:
        out = {"csv": self.csv_path}
        if self.video_path:
            out["video"] = self.video_path
        return out
