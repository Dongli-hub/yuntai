"""调试叠加显示。

板子上通常没有显示器，所以：
  * show_window=True 时会 cv2.imshow（PC 调试用）
  * 否则可以用 snapshot_every_s 周期性把标注帧存成 jpg，
    跑完一轮直接把 jpg 拷出来看就行
"""

from typing import Optional

import cv2
import numpy as np

__all__ = ["draw_overlay", "draw_rectified_inset", "draw_banner"]


def draw_banner(frame: np.ndarray, text: str, color=(0, 255, 255)) -> None:
    cv2.putText(frame, text, (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                (0, 0, 0), 5, cv2.LINE_AA)
    cv2.putText(frame, text, (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                color, 2, cv2.LINE_AA)


def draw_overlay(frame: np.ndarray, target=None, spot=None, aim=None,
                 lines=None, state: str = "", fps: float = 0.0,
                 want_inset: bool = False, rect_image=None) -> np.ndarray:
    """在帧上画检测/控制结果。返回值就是同一个帧（原地修改）。"""
    if target is not None and getattr(target, "quad", None) is not None:
        quad = np.asarray(target.quad, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(frame, [quad], True, (0, 200, 0), 2, cv2.LINE_AA)
    if target is not None:
        u, v = int(target.uv[0]), int(target.uv[1])
        cv2.drawMarker(frame, (u, v), (0, 220, 0), cv2.MARKER_CROSS, 26, 2)
        cv2.circle(frame, (u, v), 4, (0, 255, 0), -1, cv2.LINE_AA)
    if spot is not None:
        su, sv = int(round(spot.uv[0])), int(round(spot.uv[1]))
        cv2.circle(frame, (su, sv), 12, (255, 120, 255), 2, cv2.LINE_AA)
        cv2.drawMarker(frame, (su, sv), (255, 80, 255), cv2.MARKER_TILTED_CROSS,
                       16, 2)
        if target is not None:
            cv2.line(frame, (su, sv),
                     (int(target.uv[0]), int(target.uv[1])), (0, 165, 255), 2,
                     cv2.LINE_AA)
            mid = ((su + int(target.uv[0])) // 2, (sv + int(target.uv[1])) // 2)
            if aim is not None:
                cv2.putText(frame, "%.1fpx" % aim.err_norm, (mid[0] + 6, mid[1] - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2,
                            cv2.LINE_AA)
    if want_inset and rect_image is not None:
        frame = draw_rectified_inset(frame, rect_image)
    # HUD
    y = 70
    if state:
        draw_banner(frame, state, (0, 255, 255))
    cv2.putText(frame, "fps=%.0f" % fps, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 255), 2, cv2.LINE_AA)
    y += 24
    if aim is not None:
        text = ("err=%.1fpx off=(%.2f,%.2f) valid=%d coast=%d lock=%d %s"
                % (aim.err_norm, aim.yaw_deg, aim.pitch_deg, aim.valid,
                   aim.coasting, aim.locked, "BOOST" if aim.boost else ""))
        cv2.putText(frame, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 0), 1, cv2.LINE_AA)
        y += 22
    for line in (lines or []):
        cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (200, 255, 200), 1, cv2.LINE_AA)
        y += 20
    return frame


def draw_rectified_inset(frame: np.ndarray, rect_image: np.ndarray,
                         max_w: int = 200) -> np.ndarray:
    """把透视矫正后的正视图缩成小图贴到右上角，方便肉眼确认矫正是否正确。"""
    if rect_image is None or rect_image.size == 0:
        return frame
    h, w = rect_image.shape[:2]
    scale = max_w / float(max(1, w))
    small = cv2.resize(rect_image, (max_w, max(1, int(round(h * scale)))),
                       interpolation=cv2.INTER_AREA)
    fh, fw = frame.shape[:2]
    sh, sw = small.shape[:2]
    x0 = fw - sw - 12
    y0 = 12
    if x0 < 0 or y0 + sh > fh:
        return frame
    frame[y0:y0 + sh, x0:x0 + sw] = small
    cv2.rectangle(frame, (x0 - 1, y0 - 1), (x0 + sw, y0 + sh), (0, 255, 255), 1)
    return frame

