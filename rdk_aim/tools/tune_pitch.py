#!/usr/bin/env python3
"""俯仰振荡的现场定位工具：运行时就地调参 + "掰一下看衰减"自动判定。

用法：
    python3 tools/tune_pitch.py --port /dev/ttyS1

为什么需要这个工具：
    "俯仰被手掰一下就越抖越大"可能是三种完全不同的病：
      A. 陀螺前馈符号反了     -> 前馈变成正反馈，越推越抖
      B. 前馈增益太大         -> 环路增益超过 1，临界振荡
      C. 位置环 KP/KI 太大    -> 和驱动器内环打架，极限环
    三者的现象几乎一样，但解法完全相反。靠"改代码->烧写->掰一下"一轮要
    两分钟，一天也试不完，而且很容易记混上一轮改了什么。
    所以这里做两件事：
      1) 用 0x15 PARAM 消息在线改参数（H723 侧立刻生效，掉电恢复默认）
      2) 自动记录"掰一下松手后"的俯仰角曲线，算出振荡是衰减还是发散，
         把"我觉得好像好点了"变成两个数字

先搞清楚一件事（很重要）：
    IMU 装在【俯仰电机下面】的中间平台上，所以**俯仰电机自己转，IMU 看不见**。
    也就是说"用手掰俯仰轴"这个动作，陀螺前馈根本不参与 ——
    掰一下会抖，只能是【电机编码器位置环】的问题（KP/KI 太大、反馈太旧、
    机械间隙），前馈是替罪羊。
    要测前馈，必须倾【底座/车体】（那才是 IMU 看得见的运动），用菜单的 f。

推荐排查顺序（每次只改一个变量！）：
    1) 先看抖动的频率（工具会自动算）：
         >3Hz  -> 增益型自激：按 p 用安全预设（KP=8 KI=20），再按 t 掰一下
         <2Hz  -> 积分/摩擦型极限环：把 KI 降下来（k 8 15 0.1），再按 t
    2) 位置环稳了以后，再按 f 倾底座，检查前馈符号对不对（corr 应为负）
    3) 需要的话按 - / + 换前馈符号、按 g 改前馈增益
    4) 调好后把最终数值写回 yuntai_task.c 里的默认值，重新烧写固化

还有一条相关的（云台整体低头/抬头时镜头不跟着找回来）：
    俯仰位置环量的是"电机相对底座"的角度，底座整体倾斜时它压根不变，
    所以必须由 IMU 提供"世界倾斜了多少"-> 平台俯仰补偿（c 命令）。
    符号注意：补偿符号与前馈符号【相反】（老固件 FF=-1 配 COMP=+1）。
    测法：把底座慢慢倾斜 10°，看靶面光斑 —— 基本不动就对了。

安全：
  · 全程只发 MODE=STAB（只稳定、不带瞄准偏置），不会让云台乱扫
  · 不会发 LASER_ON 标志
  · 退出时自动切回 STAB
"""

import argparse
import os
import statistics
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim import protocol as proto  # noqa: E402


# --------------------------------------------------------------------------
class Link:
    """串口 + 后台收帧线程。

    收帧必须在后台线程做：主线程要留给 input() 菜单，不能阻塞在读串口上。
    所有写操作加锁，防止"菜单发一帧"和"心跳线程发一帧"在串口上交叉写。
    """

    def __init__(self, port: str, baud: int):
        import serial
        self.dev = serial.Serial(port, baud, timeout=0.02)
        self._wlock = threading.Lock()
        self._lock = threading.Lock()
        self.parser = proto.FrameParser()
        self.seq = 0
        self.state = None            # 最新一帧遥测
        self.texts = []              # 未打印的 TEXT
        self.recording = False
        self.samples = []            # (t, pitch_motor_deg, gyro_y_dps)
        self.running = True
        self.rx_frames = 0
        self.t0 = time.monotonic()
        self._th = threading.Thread(target=self._reader, daemon=True)
        self._th.start()

    # ---------------- 发送 ----------------
    def send(self, msg_id, payload=b""):
        with self._wlock:
            self.dev.write(proto.build_frame(msg_id, payload, self.seq))
            self.seq = (self.seq + 1) & 0xFF

    def set_param(self, param_id, value):
        self.send(proto.MsgId.PARAM, proto.pack_param(param_id, value))

    # ---------------- 后台收 ----------------
    def _reader(self):
        next_hb = 0.0
        while self.running:
            now = time.monotonic()
            if now >= next_hb:
                next_hb = now + 0.2          # 5Hz 心跳，喂 H723 的掉线看门狗
                try:
                    self.send(proto.MsgId.HEARTBEAT_G)
                except Exception:            # noqa: BLE001
                    pass
            try:
                data = self.dev.read(512)
            except Exception:                # noqa: BLE001
                data = b""
            if not data:
                continue
            for f in self.parser.feed(data):
                self.rx_frames += 1
                if f.msg_id == proto.MsgId.GIMBAL_STATE:
                    try:
                        st = proto.unpack_gimbal_state(f.payload)
                    except Exception:        # noqa: BLE001
                        continue
                    with self._lock:
                        self.state = st
                        if self.recording:
                            self.samples.append((now - self.t0, st.pitch_motor_deg,
                                                 st.gyro_y_dps))
                elif f.msg_id == proto.MsgId.TEXT:
                    with self._lock:
                        self.texts.append(f.payload.decode("utf-8", "replace"))

    def drain_text(self):
        with self._lock:
            out, self.texts = self.texts[:], []
        return out

    def start_record(self):
        with self._lock:
            self.samples = []
            self.recording = True

    def stop_record(self):
        with self._lock:
            self.recording = False
            return list(self.samples)

    def close(self):
        self.running = False
        time.sleep(0.05)
        try:
            self.dev.close()
        except Exception:                    # noqa: BLE001
            pass


# --------------------------------------------------------------------------
def analyze(samples):
    """把一段"掰一下松手后"的曲线判成：收敛 / 等幅 / 发散。

    判据用"峰值点之后三个 1 秒窗口的峰峰值(pk-pk)"：
        pp1 (0.2~1.2s)  pp2 (1.2~2.2s)  pp3 (2.2~3.2s)
    正常系统 pp 应该单调下降；等幅说明有东西在持续供能（摩擦极限环、
    间隙、或者环路增益正好在临界点）；递增就是不稳定。
    """
    if len(samples) < 30:
        print("  ✘ 遥测样本太少（%d 个）—— 先确认链路通不通" % len(samples))
        return None

    xs = [s[1] for s in samples]
    med = statistics.median(xs)
    dev = [abs(x - med) for x in xs]
    i_peak = max(range(len(dev)), key=lambda i: dev[i])
    t_peak = samples[i_peak][0]
    print("  静止位置 %.2f° | 手工扰动峰值 %.2f°（t=%.1fs）"
          % (med, dev[i_peak], t_peak))

    def window(a, b):
        return [s for s in samples if t_peak + a <= s[0] < t_peak + b]

    def pp(seg):
        if len(seg) < 5:
            return 0.0
        v = [s[1] for s in seg]
        return max(v) - min(v)

    p1, p2, p3 = pp(window(0.2, 1.2)), pp(window(1.2, 2.2)), pp(window(2.2, 3.2))
    g1, g3 = pp([(s[0], s[2]) for s in window(0.2, 1.2)]), \
        pp([(s[0], s[2]) for s in window(2.2, 3.2)])
    print("  峰峰值：第1秒 %.2f° | 第2秒 %.2f° | 第3秒 %.2f°  （陀螺 %.1f -> %.1f dps）"
          % (p1, p2, p3, g1, g3))

    # 振荡频率：用第 2 秒窗口里过中值的次数估
    freq = 0.0
    seg = window(1.2, 2.2)
    if len(seg) >= 10:
        signs = [1 if s[1] - med > 0 else -1 for s in seg]
        crosses = sum(1 for i in range(1, len(signs)) if signs[i] != signs[i - 1])
        if crosses >= 2:
            freq = crosses / 2.0
            print("  振荡频率 ≈ %.2f Hz（周期 %.0f ms）" % (freq, 1000.0 / freq))

    if p1 < 0.5:
        print("  ⚠ 扰动幅度太小（%.2f°），这一轮结论不可信：掰大一点再试一次" % p1)
        return None
    if p2 < 0.15:
        print("  ✔ 第 2 秒就已经基本不抖了 —— 收敛得很好")
        return "ok"

    r = p3 / p2 if p2 > 1e-9 else 0.0
    if r > 1.15:
        print("  ✘ 还在发散（第3秒/第2秒 = %.2f）—— 这个参数组合不能用！" % r)
        return "grow"
    if r > 0.80:
        print("  ✘ 等幅振荡（第3秒/第2秒 = %.2f）—— 有东西在持续供能" % r)
        if freq >= 3.0:
            print("     频率 %.1fHz 偏高 -> 典型的【增益型自激】：位置环 KP 相对" % freq)
            print("     驱动器速度环的延迟太大。按 p 用安全预设（KP=8 KI=20）再试。")
        elif freq > 0.0:
            print("     频率 %.1fHz 偏低 -> 典型的【积分/摩擦型极限环】：KI 太大或者" % freq)
            print("     机构有间隙/静摩擦。按 k 8 15 0.1 把 KI 降下来再试。")
        return "flat"
    print("  ✔ 在收敛（第3秒/第2秒 = %.2f）" % r)
    return "decay"


def nudge_test(link, seconds=6.0, label=""):
    print()
    print("  ┌─ 掰一下测试 %s" % label)
    print("  │ 按回车开始记录，然后立刻【用手把俯仰轴掰偏约 10°、松手】")
    print("  │ （松手后不要再碰它，也不要碰云台底座）")
    print("  └─ 记录 %.0f 秒…" % seconds)
    try:
        input()
    except EOFError:
        return None
    link.start_record()
    t_end = time.time() + seconds
    while time.time() < t_end:
        time.sleep(0.25)
        st = link.state
        if st is not None:
            sys.stdout.write("\r      pitch_motor=%+7.2f°  gyro_y=%+6.1f dps   "
                             % (st.pitch_motor_deg, st.gyro_y_dps))
            sys.stdout.flush()
    print()
    samples = link.stop_record()
    for s in link.drain_text():
        print("      H723 | %s" % s)
    return analyze(samples)


def _pearson(xs, ys):
    n = min(len(xs), len(ys))
    if n < 5:
        return 0.0
    mx = sum(xs) / n
    my = sum(ys) / n
    cov = sum((xs[i] - mx) * (ys[i] - my) for i in range(n))
    sx = math.sqrt(sum((x - mx) ** 2 for x in xs[:n]))
    sy = math.sqrt(sum((y - my) ** 2 for y in ys[:n]))
    if sx < 1e-9 or sy < 1e-9:
        return 0.0
    return cov / (sx * sy)


def tilt_test(link, seconds=7.0):
    """测前馈极性：倾底座（IMU 看得见的运动）看电机是否反向补偿。

    IMU 在俯仰电机下面，所以倾底座时相机是跟着抬/低头的；
    前馈的作用就是让俯仰电机反向转、把相机找回来。
    于是"电机角速度"和"陀螺俯仰角速度"应该是**负相关**：
        底座低头(gyro>0) -> 电机应反向转(θ' < 0)
    相关系数为正说明前馈在帮倒忙（符号反了）。
    """
    print("""
  ┌─ 前馈极性检查（倾底座，别碰俯仰轴）
  │ 按回车开始记录，然后用手扶住【云台底座/车体】绕横轴前后点动 3~4 次，
  │ 每次约 10~20°、0.3 秒做完，幅度快一点更容易看出来
  └─ 记录 %.0f 秒…""" % seconds)
    try:
        input()
    except EOFError:
        return None
    link.start_record()
    t_end = time.time() + seconds
    while time.time() < t_end:
        time.sleep(0.25)
    samples = link.stop_record()
    for s in link.drain_text():
        print("      H723 | %s" % s)
    if len(samples) < 40:
        print("  ✘ 遥测样本太少，没法判断")
        return None
    vs, gs = [], []
    for i in range(1, len(samples) - 1):
        dt = samples[i + 1][0] - samples[i - 1][0]
        if dt <= 1e-6:
            continue
        g = samples[i][2]
        if abs(g) < 3.0:          # 底座没动的时候不参与统计
            continue
        vs.append((samples[i + 1][1] - samples[i - 1][1]) / dt)   # 电机角速度 °/s
        gs.append(g)
    if len(vs) < 15:
        print("  ✘ 底座动得太少（只采到 %d 个有效样本），重来一次、幅度大一点" % len(vs))
        return None
    corr = _pearson(vs, gs)
    print("  有效样本 %d（底座真的在动的时刻）" % len(vs))
    print("  陀螺俯仰角速度范围 %.0f ~ %.0f dps；电机角速度范围 %.0f ~ %.0f dps"
          % (min(gs), max(gs), min(vs), max(vs)))
    print("  相关系数 corr = %+.2f" % corr)
    if corr < -0.30:
        print("  ✔ 负相关：底座动的时候电机在【反向补】—— 前馈符号是对的")
    elif corr > 0.30:
        print("  ✘ 正相关：电机在【跟着一起动】—— 前馈符号反了，按 - / + 换一个再测")
    else:
        print("  ? 相关不明显：多半是倾得太慢（位置环把它压住了），再快一点、幅度大一点")
    return corr


# --------------------------------------------------------------------------
GAIN_HINT = 8.0     # 试符号时用的"安全增益"（理论抵消量约 9.55）
SAFE_PRESET = "安全预设（KP=8 KI=20 KD=0.1）"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyS1")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    try:
        link = Link(args.port, args.baud)
    except Exception as exc:                                    # noqa: BLE001
        print("打不开 %s：%s" % (args.port, exc))
        return 1

    def cleanup():
        try:
            link.send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.STAB))
            time.sleep(0.1)
        except Exception:                                       # noqa: BLE001
            pass
        link.close()

    print("=" * 70)
    print(" 俯仰振荡现场定位：%s @%d" % (args.port, args.baud))
    print("=" * 70)

    # 进 STAB（只稳定，不带瞄准偏置），并等遥测滚起来
    link.send(proto.MsgId.MODE, proto.pack_mode(proto.AimMode.STAB))
    t_end = time.time() + 1.5
    while time.time() < t_end and link.state is None:
        time.sleep(0.05)
    st = link.state
    if st is None:
        print("✘ 1.5 秒内没收到任何遥测帧 —— 先查串口/线序/共地，工具退出。")
        cleanup()
        return 1
    print("遥测正常：%s" % st.describe())

    # 检查固件是否支持 0x15 PARAM（旧固件不会回 [PARAM] 文本）
    link.set_param(proto.ParamId.DUMP, 0)
    time.sleep(0.4)
    texts = link.drain_text()
    for s in texts:
        print("  H723 | %s" % s)
    if not any("[PARAM]" in s for s in texts):
        print("✘ H723 没有回 [PARAM]，说明板上还是旧固件（0x15 未实现）。")
        print("  请先在 CubeIDE 里重新编译烧写，再回来跑这个工具。")
        cleanup()
        return 1

    menu = """
---------------------------- 菜单 ----------------------------
 0        关掉俯仰前馈（gain=0）后立刻做一次掰一下测试
 -        前馈符号取 -1（保持安全增益）后测试
 +        前馈符号取 +1（保持安全增益）后测试
 g  <值>              设置前馈增益（例如 g 8）
 k  <kp> <ki> <kd>    设置俯仰 PID（例如 k 8 20 0.1）
 p        一键套用%s（推荐先试这个）
 f        前馈极性检查：倾底座，工具算陀螺与电机角速度的相关系数
 c  <值>  平台俯仰补偿：c 0 关 / c 1 开 / c -1 开且符号取反
 t        只做一次"掰一下"测试（不改参数）
 d        打印 H723 当前参数
 q        退出（自动切回 STAB）
--------------------------------------------------------------
推荐：先按 p（降 KP/KI）再按 t 掰一下；位置环稳了再用 f 查前馈符号。
""" % SAFE_PRESET
    print(menu)

    try:
        while True:
            try:
                cmd = input("tune> ").strip()
            except EOFError:
                break
            if not cmd:
                continue
            parts = cmd.split()
            op = parts[0].lower()
            try:
                if op == "0":
                    link.set_param(proto.ParamId.PITCH_FF_GAIN, 0.0)
                    time.sleep(0.3)
                    for s in link.drain_text():
                        print("  H723 | %s" % s)
                    nudge_test(link, label="（前馈已关闭）")
                elif op in ("-", "+"):
                    sign = -1.0 if op == "-" else 1.0
                    link.set_param(proto.ParamId.PITCH_FF_GAIN, GAIN_HINT)
                    link.set_param(proto.ParamId.PITCH_FF_SIGN, sign)
                    time.sleep(0.3)
                    for s in link.drain_text():
                        print("  H723 | %s" % s)
                    nudge_test(link, label="（前馈 %+.0f x %.1f）" % (sign, GAIN_HINT))
                elif op == "g" and len(parts) >= 2:
                    link.set_param(proto.ParamId.PITCH_FF_GAIN, float(parts[1]))
                    time.sleep(0.3)
                    for s in link.drain_text():
                        print("  H723 | %s" % s)
                elif op == "k" and len(parts) >= 4:
                    link.set_param(proto.ParamId.PITCH_KP, float(parts[1]))
                    link.set_param(proto.ParamId.PITCH_KI, float(parts[2]))
                    link.set_param(proto.ParamId.PITCH_KD, float(parts[3]))
                    time.sleep(0.3)
                    for s in link.drain_text():
                        print("  H723 | %s" % s)
                elif op == "p":
                    link.set_param(proto.ParamId.PITCH_KP, 8.0)
                    link.set_param(proto.ParamId.PITCH_KI, 20.0)
                    link.set_param(proto.ParamId.PITCH_KD, 0.1)
                    time.sleep(0.3)
                    for s in link.drain_text():
                        print("  H723 | %s" % s)
                    print("  已套用 %s" % SAFE_PRESET)
                    nudge_test(link, label="（安全预设）")
                elif op == "f":
                    tilt_test(link)
                elif op == "c" and len(parts) >= 2:
                    link.set_param(proto.ParamId.PITCH_PLAT_SIGN, float(parts[1]))
                    time.sleep(0.3)
                    for s in link.drain_text():
                        print("  H723 | %s" % s)
                    print("  平台俯仰补偿已设为 %+.0f（0 = 关闭）。"
                          % float(parts[1]))
                    print("  验证：慢慢把底座倾斜约 10°，看靶面上光斑动不动 ——")
                    print("        基本不动 = 符号对；动得比关闭时更大 = 用另一个符号。")
                elif op == "t":
                    nudge_test(link)
                elif op == "d":
                    link.set_param(proto.ParamId.DUMP, 0)
                    time.sleep(0.3)
                    for s in link.drain_text():
                        print("  H723 | %s" % s)
                elif op in ("q", "quit", "exit"):
                    break
                else:
                    print(menu)
            except ValueError:
                print("参数格式不对，例：g 8 / k 8 45 0.1")
    except KeyboardInterrupt:
        print("\nCtrl-C")
    finally:
        cleanup()
    print("已退出，H723 已切回 STAB（不会乱动）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
