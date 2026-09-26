"""主状态机定义 + 扫描规划器。

设计沿用 H723 固件里那套"每一步都打印"的思路：
现场出问题先看状态日志，卡在哪个状态一目了然。
"""

from enum import IntEnum
from typing import List, Tuple

__all__ = ["State", "SweepPlanner"]


class State(IntEnum):
    INIT = 0            # 程序刚起来
    BOOT_WAIT = 1       # 等 H723 走完启动流程并回报状态
    UNWIND = 2          # 解绕（测试之间用）
    ACQUIRE = 3         # 扫描找靶
    LOCK = 4            # 视觉伺服收敛到靶心
    TRACK = 5           # 小车行驶中持续瞄准
    DRAW = 6            # 沿 r=6cm 红圈画圆
    LOST = 7            # 靶短时丢失
    FAULT = 8
    DONE = 9
    ESTOP = 10

    @property
    def label(self) -> str:
        return _LABEL.get(int(self), str(int(self)))


_LABEL = {
    State.INIT: "INIT",
    State.BOOT_WAIT: "BOOT_WAIT",
    State.UNWIND: "UNWIND",
    State.ACQUIRE: "ACQUIRE",
    State.LOCK: "LOCK",
    State.TRACK: "TRACK",
    State.DRAW: "DRAW",
    State.LOST: "LOST",
    State.FAULT: "FAULT",
    State.DONE: "DONE",
    State.ESTOP: "ESTOP",
}


class SweepPlanner:
    """扫描捕获的点位生成器。

    两层设计，都是"先水平、后上下"：
      * 靶在竖直方向的高度范围很小（<=50cm，相机装车后俯仰需求只有几度到十几度），
        所以**俯仰是次要自由度**：先在 pitch=0 扫一整圈水平；
      * 水平那一圈按**单调绕圈**排序（0 -> +60 -> +120 -> 180 -> -120 -> -60），
        总行程只有 360 度；而"从中心向外交替"的排法总行程要 1245 度，
        在 2s 的时间预算里根本跑不完。

    算一笔账（fx=640@1280 宽 => 水平视场 90 度，步长 60 度）：
      水平层 7 个点：移动 360 度 @260度/s = 1.38s，停留 7x70ms = 0.49s
      合计约 1.87s —— **能在 2s 预算内扫完一整圈**。
    注意第一帧（yaw=0）本身就覆盖 ±45 度，如果车大致朝着靶，
    往往第一个点就命中了。
    """

    def __init__(self, mode: str, yaw_range: float, pitch_range: float,
                 step_deg: float):
        self.mode = mode
        self.yaw_range = max(0.0, float(yaw_range))
        self.pitch_range = max(0.0, float(pitch_range))
        self.step = max(1.0, float(step_deg))
        self.points = self._build()
        self.index = 0

    def _levels(self, limit: float) -> List[float]:
        n = int(limit // self.step)
        raw = [i * self.step for i in range(-n, n + 1)]
        if n * self.step < limit - 1e-6:
            raw += [limit, -limit]
        return sorted(set(raw), key=lambda v: (abs(v), v))

    def _build(self) -> List[Tuple[float, float]]:
        pitches = self._levels(self.pitch_range)
        # 单调绕圈顺序：按"从 0 出发沿正方向转一圈"的行程角排序
        yaws = sorted(self._levels(self.yaw_range), key=lambda v: v % 360.0)
        pts: List[Tuple[float, float]] = []
        if self.mode == "full":
            # 逐层栅格：从下俯仰到上俯仰，每层绕一整圈
            for p in sorted(pitches, reverse=True):
                for y in yaws:
                    pts.append((y, p))
        else:
            # 先 pitch=0 整圈（最常见的情况），再按 |pitch| 逐层加俯仰
            for p in pitches:
                for y in yaws:
                    pts.append((y, p))
        return pts

    def reset(self) -> None:
        self.index = 0

    def next_point(self) -> Tuple[float, float]:
        if not self.points:
            return (0.0, 0.0)
        pt = self.points[self.index % len(self.points)]
        self.index += 1
        return pt

    def __len__(self) -> int:
        return len(self.points)

    def estimate_duration(self, dwell_ms: float, move_dps: float = 200.0) -> float:
        """粗略估计扫完一遍要多久（秒），用于判断时间预算够不够。"""
        total = 0.0
        prev = (0.0, 0.0)
        for pt in self.points:
            move = max(abs(pt[0] - prev[0]), abs(pt[1] - prev[1])) / max(1.0, move_dps)
            total += move + dwell_ms / 1000.0
            prev = pt
        return total
