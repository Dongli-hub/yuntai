"""主状态机定义 + 扫描规划器。

设计沿用 H723 固件里那套"每一步都打印"的思路：
现场出问题先看状态日志，卡在哪个状态一目了然。
"""

from enum import IntEnum
from typing import List, Optional, Tuple

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
      * 水平那一圈按【由近及远、一左一右】排序：
            0 -> +60 -> -60 -> +120 -> -120 -> +175 -> -175
        这样"偏一点"的靶纸在前两个点就能找到，不用先绕一整圈。
        （以前是单调绕圈 0->+60->+120->180->-120->-60：如果靶在左边，
        要绕 300 度才轮到它，白白多花 1 秒多。）

    注意第一帧（yaw=0）本身就覆盖 ±45 度，如果车大致朝着靶，
    往往第一个点就命中了；实际移动速度由 acquire.scan_dps 限制（慢一点，
    避免运动模糊导致"路过却没看见"）。
    """

    def __init__(self, mode: str, yaw_range: float, pitch_range: float,
                 step_deg: float, center: Tuple[float, float] = (0.0, 0.0),
                 pitch_layers: Optional[List[float]] = None):
        self.mode = mode
        self.yaw_range = max(0.0, float(yaw_range))
        self.pitch_range = max(0.0, float(pitch_range))
        self.step = max(1.0, float(step_deg))
        # 扫描中心：云台基线（偏置 0）看不到靶纸时，把整个扫描网格挪过去。
        # 例：配置 acquire.center_yaw_deg=9 / center_pitch_deg=13，
        # 就等价于"先把云台转到 (9°,13°)，再从那里开始扫"。
        self.center = (float(center[0]), float(center[1]))
        # 俯仰层：给了就用它（按给定顺序，一般"先上后下"），
        # 不给就按 pitch_range 对称生成。
        self.pitch_layers = [float(x) for x in pitch_layers] if pitch_layers else []
        # 水平搜索方式（2026-09-30 用户要求）：
        #   circle = 同一方向连续转一圈（0→60→120→180→240→300，回到 0）
        #   zigzag = 原来的一左一右（0→+60→−60→+120→−120…）
        # 单向转的好处是"不会在两个方向之间来回甩"，观感与电缆走向都更可控。
        self.one_way = str(mode) == "circle"
        self.points = self._build()
        self.index = 0

    def _levels(self, limit: float) -> List[float]:
        n = int(limit // self.step)
        raw = [i * self.step for i in range(-n, n + 1)]
        if n * self.step < limit - 1e-6:
            raw += [limit, -limit]
        # 由近及远；同一距离先正后负（"一左一右"里的先右后左）
        return sorted(set(raw), key=lambda v: (abs(v), -v))

    def _build(self) -> List[Tuple[float, float]]:
        if self.pitch_layers:
            pitches = list(self.pitch_layers)
        else:
            pitches = self._levels(self.pitch_range)
        # 水平方向：单向一圈 或 左右交替（见 __init__ 的说明）
        if self.one_way:
            n = max(1, int(round(360.0 / max(1.0, self.step))))
            yaws = [i * self.step for i in range(n)]      # 0,60,...,300
        else:
            yaws = self._levels(self.yaw_range)
        pts: List[Tuple[float, float]] = []
        if self.mode == "full":
            # 逐层栅格：从下俯仰到上俯仰，每层绕一整圈
            for p in sorted(pitches, reverse=True):
                for y in yaws:
                    pts.append((y, p))
        else:
            # 一层一层扫：先在 pitch=0 扫一整圈（层内"由近及远、一左一右"），
            # 再 pitch=+35、pitch=-35。
            # 为什么不让俯仰在每两个点之间跳：实测云台会"大幅前后摆动"，
            # 又吵又容易甩到摄像头线（用户反馈）。俯仰只在换层时动两次。
            for p in pitches:
                for y in yaws:
                    pts.append((y, p))
        if self.center != (0.0, 0.0):
            pts = [(y + self.center[0], p + self.center[1]) for (y, p) in pts]
        return pts

    def reset(self) -> None:
        self.index = 0

    def set_center(self, yaw_deg: float, pitch_deg: float) -> None:
        """把扫描网格中心挪到指定偏置。

        用途（用户实测要求）：丢靶之后，应该以【丢靶前一刻的云台位置】为中心
        左右去找，而不是每次都回到上电零位重新扫一圈 —— 后者要多转几十度。
        第一次捕获时调用方传的就是上电零位（0,0），行为不变。
        """
        c = (float(yaw_deg), float(pitch_deg))
        if c == self.center:
            self.reset()
            return
        self.center = c
        self.points = self._build()
        self.reset()

    def next_point(self) -> Tuple[float, float]:
        if not self.points:
            return (0.0, 0.0)
        pt = self.points[self.index % len(self.points)]
        self.index += 1
        return pt

    def fine_points(self, center: Tuple[float, float], span_deg: float = 16.0,
                    step_deg: float = 8.0, pitch_span_deg: float = 8.0):
        """丢靶后的"就近细扫"点位：以丢靶前一刻的位置为中心，小幅度左右找。

        为什么要单独有这么一档（2026-09-27 现场实测）：
          大范围扫描一格是 60°。一旦检测漏了几帧被判丢靶，云台就会"啪"地
          甩 60° 出去 —— 靶纸早就出画面了，然后下一个点再甩 120°，
          现场看到的现象就是"光斑在靶纸宽度范围内来回匀速摆动，永远停不下来"。
          而丢靶的真实原因往往只是那一瞬间模糊/反光/被裁掉，靶纸就在原地。
          所以先以丢靶前的位置为中心，±16° 一左一右慢慢找（11 个点 ≈ 2~3 秒），
          找到了立刻回到瞄准；细扫走完还是没有，才交给大范围扫描。
        """
        cy, cp = float(center[0]), float(center[1])
        n = max(1, int(round(span_deg / max(1.0, step_deg))))
        ys = []
        for i in range(n + 1):
            if i == 0:
                ys.append(0.0)
            else:
                ys.append(i * step_deg)
                ys.append(-i * step_deg)
        pts = [(cy + y, cp) for y in ys]
        if pitch_span_deg >= 1.0:
            for dp in (pitch_span_deg, -pitch_span_deg):
                for y in (0.0, step_deg, -step_deg):
                    pts.append((cy + y, cp + dp))
        return pts

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
