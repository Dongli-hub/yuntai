# -*- coding: utf-8 -*-
"""k230_link_test.py —— K230 <-> H723 通信自检（只测通信，不碰电机）

用途：K230 用 Type-C 接上位机、H723 只供 MCU 电（电机不供电）时，
      单独确认这条串口链路是不是通的。

程序只做三件事：
  1) 每 200ms 发一帧 HEARTBEAT(0x12)
  2) 收 H723 的遥测(0x90) / ACK(0x91) / 调试文本(0x93) 并统计
  3) 每秒打印一次结论式报告，同时把状态画到 IDE 的画面视图里

判断标准（不看电机、不看编码器、不看 READY）：
  * 收到 GIMBAL_STATE  -> H723 -> K230 方向通
  * 收到 ACK(0x12)     -> K230 -> H723 方向也通（H723 收到心跳会回 ACK）
  两个都有 = 双向通信 OK。

没有独立屏幕也没关系：显示初始化失败会自动跳过，只看串行终端输出即可。

自动挑串口：这块亚博固件把 IO9/IO10 提前占给了它自己，所以
machine.UART(1, tx=Pin(9)) 会失败；本程序会把下面几种后端挨个试一遍，
哪个能收到字节就用哪个，并在最后打印一张对照表：
    YbUart(亚博封装) / UART(1) 不指定引脚 / UART(1,IO9/IO10) / UART(3,IO32/IO33)

LOOPBACK = True 时做"自环测试"：把 K230 的 IO9 和 IO10 用杜邦线短接，
程序应该收到自己发出的心跳（证明 K230 这侧收发都好）。
"""
import os
import time

# ============================ 参数 ============================
UART_BAUD = 115200

# 自动遍历所有可能的后端，每个试 PROBE_S 秒
AUTO_BACKEND = True
PROBE_S = 3.0

# 自环测试：先把 IO9 和 IO10 用杜邦线短接再打开这个开关
LOOPBACK = False

DISPLAY_MODE = "VIRT"      # VIRT(只用 IDE 画面) | LCD(带屏) | OFF
DISPLAY_W = 640
DISPLAY_H = 480

HEARTBEAT_MS = 200
PRINT_MS = 1000
# ==============================================================

SOF = b"\xAA\x55"
MSG_AIM = 0x10
MSG_MODE = 0x11
MSG_HEARTBEAT = 0x12
MSG_SET_ZERO = 0x13
MSG_GIMBAL_STATE = 0x90
MSG_ACK = 0x91
MSG_VERSION = 0x92
MSG_TEXT = 0x93


def _crc_table():
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if (c & 1) else (c >> 1)
        t.append(c)
    return t


_CRC = _crc_table()


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ _CRC[(crc ^ b) & 0xFF]
    return crc & 0xFFFF


def build_frame(msg_id, payload=b"", seq=0):
    import ustruct
    body = ustruct.pack("<BBB", msg_id & 0xFF, seq & 0xFF,
                        len(payload)) + payload
    return SOF + body + ustruct.pack("<H", crc16(body))


class FrameParser(object):
    def __init__(self):
        self.buf = bytearray()
        self.ok = 0
        self.crc_err = 0
        self.bad_len = 0
        self.resync = 0
        self.bytes_rx = 0

    def feed(self, data):
        out = []
        self.bytes_rx += len(data)
        self.buf.extend(data)
        while True:
            i = self.buf.find(SOF)
            if i < 0:
                if len(self.buf) > 1:
                    self.resync += 1
                    # MicroPython 的 bytearray 不支持切片删除，见 main.py 里的说明
                    self.buf = self.buf[-1:]
                break
            if i > 0:
                self.resync += 1
                self.buf = self.buf[i:]
            if len(self.buf) < 5:
                break
            length = self.buf[4]
            if length > 240:
                self.bad_len += 1
                self.buf = self.buf[2:]
                continue
            total = 5 + length + 2
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:5 + length])
            crc_rx = self.buf[5 + length] | (self.buf[6 + length] << 8)
            if crc16(body) != crc_rx:
                self.crc_err += 1
                self.buf = self.buf[2:]
                continue
            self.ok += 1
            out.append((body[0], body[1], bytes(body[3:])))
            self.buf = self.buf[total:]
        return out


def _open_machine(unit, tx=None, rx=None):
    from machine import UART, Pin
    if tx is None:
        # 不指定引脚：用固件已经配好的那组（亚博固件把 IO9/IO10 提前占了）
        return UART(unit, baudrate=UART_BAUD, bits=8, parity=None, stop=0)
    return UART(unit, baudrate=UART_BAUD, tx=Pin(tx), rx=Pin(rx),
                bits=8, parity=None, stop=0)


def _open_yb():
    from ybUtils.YbUart import YbUart
    return YbUart(baudrate=UART_BAUD)


def _open_yb_swapped():
    """IO9/IO10 反着用：IO10 当 TXD，IO9 当 RXD。

    如果两根数据线接反了（K230 的 IO9 接到了对面 TX、IO10 接到对面 RX），
    不用拆线，用这个后端就能正常通信 —— K230 的 FPIOA 允许这样重映射。
    """
    from machine import FPIOA, UART
    fp = FPIOA()
    fp.set_function(10, FPIOA.UART1_TXD, ie=0, oe=1, pu=1)
    fp.set_function(9, FPIOA.UART1_RXD, ie=1, oe=0, pu=1)
    return UART(1, baudrate=UART_BAUD)


def _open_uart3_fpioa():
    """手动把 12Pin GPIO 上的 IO32/IO33 配成 UART3 再打开。

    和 YbUart 配 IO9/IO10 是同一套做法：先 FPIOA 指定功能，再建 UART。
    如果 UART1(IO9/IO10) 的接收有问题，可以改用这一路，
    把线挪到 12Pin GPIO 第 3 脚(IO33=UART3_RXD) / 第 5 脚(IO32=UART3_TXD)。
    """
    from machine import FPIOA, UART
    fp = FPIOA()
    fp.set_function(32, FPIOA.UART3_TXD, ie=0, oe=1, pu=1)
    fp.set_function(33, FPIOA.UART3_RXD, ie=1, oe=0, pu=1)
    return UART(3, baudrate=UART_BAUD)


def candidates():
    return [
        ("YbUart(亚博封装)", _open_yb),
        ("UART1 IO9/IO10 反接", _open_yb_swapped),
        ("UART3 手配 IO32/IO33", _open_uart3_fpioa),
        ("UART(1) 不指定引脚", lambda: _open_machine(1)),
        ("UART(1) IO9/IO10", lambda: _open_machine(1, 9, 10)),
        ("UART(3) IO32/IO33", lambda: _open_machine(3, 32, 33)),
    ]


def probe_device(dev, seconds):
    """给一个已打开的串口：发心跳 + 收字节，返回 (收到字节数, 解析出帧数)。"""
    p = FrameParser()
    n_bytes = 0
    n_frames = 0
    t_end = time.ticks_ms() + int(seconds * 1000)
    t_hb = time.ticks_ms()
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        os.exitpoint()
        now = time.ticks_ms()
        if time.ticks_diff(now, t_hb) >= HEARTBEAT_MS:
            t_hb = now
            try:
                dev.write(build_frame(MSG_HEARTBEAT))
            except Exception:
                pass
        data = uart_read(dev)
        if data:
            n_bytes += len(data)
            n_frames += len(p.feed(data))
        time.sleep_ms(2)
    return n_bytes, n_frames


def uart_read(dev, n=256):
    try:
        if hasattr(dev, "any") and dev.any() <= 0:
            return b""
        return dev.read(n) or b""
    except Exception:
        return b""


def hexstr(b):
    return " ".join("%02X" % x for x in b)


def print_fpioa_state():
    """打印关键引脚当前被分配成什么功能（确认真的配成串口了）。"""
    try:
        from machine import FPIOA
        fp = FPIOA()
        out = []
        for pin in (9, 10, 32, 33):
            try:
                out.append("IO%d=%s" % (pin, fp.get_pin_func(pin)))
            except Exception as e:
                out.append("IO%d=?(%s)" % (pin, e))
        print("引脚功能: %s" % " ".join(out))
    except Exception as e:
        print("FPIOA 查询失败: %s" % e)


def main():
    dev = None
    uart_name = ""
    table = []
    if AUTO_BACKEND and not LOOPBACK:
        print("自动遍历串口后端（每个 %d 秒）..." % int(PROBE_S))
        for label, opener in candidates():
            try:
                d = opener()
            except Exception as e:
                print("  %-22s 打不开：%s" % (label, e))
                table.append((label, "打不开", str(e), 0, 0))
                continue
            n_bytes, n_frames = probe_device(d, PROBE_S)
            print("  %-22s 已打开：收到 %d 字节 / %d 帧"
                  % (label, n_bytes, n_frames))
            table.append((label, "已打开", "", n_bytes, n_frames))
            if n_bytes > 0:
                dev = d
                uart_name = label
                # ★ 关键：找到能收数据的后端就立刻停手！
                #   继续试后面那些后端时，构造 Pin(9)/Pin(10) 会把 IO9/IO10
                #   重新配成 GPIO，等于把刚配好的串口引脚"掀掉" ——
                #   那样主循环里既发不出去也收不到（踩过这个坑）。
                print("  -> 这个后端能收数据，直接用它，"
                      "不再尝试其它后端（避免改动引脚配置）")
                break
            else:
                try:
                    d.deinit()
                except Exception:
                    pass
        if dev is None:
            # 谁都没数据：退回到第一个能打开的后端，继续跑，方便看实时状态
            for label, opener in candidates():
                try:
                    dev = opener()
                    uart_name = label + "（当前无数据）"
                    break
                except Exception:
                    continue
        print("-" * 56)
    else:
        for label, opener in candidates():
            try:
                dev = opener()
                uart_name = label
                break
            except Exception as e:
                print("  %s 打不开: %s" % (label, e))
    if dev is None:
        print("!! 没有任何串口后端能打开——先解决这个")
        return
    print("=" * 56)
    print("K230 通信自检")
    print("使用串口: %s @ %d" % (uart_name, UART_BAUD))
    print_fpioa_state()
    print("=" * 56)

    parser = FrameParser()

    # ---- 显示（没有屏也能跑，失败就跳过） ----
    canvas = None
    display_ok = False
    if DISPLAY_MODE != "OFF":
        try:
            from media.display import Display
            from media.media import MediaManager
            import image
            if DISPLAY_MODE == "LCD":
                Display.init(Display.ST7701, width=DISPLAY_W,
                             height=DISPLAY_H, to_ide=True)
            else:
                Display.init(Display.VIRT, width=DISPLAY_W,
                             height=DISPLAY_H, fps=30)
            MediaManager.init()
            canvas = image.Image(DISPLAY_W, DISPLAY_H, image.RGB565)
            display_ok = True
            print("显示已初始化（画到 IDE 画面视图）")
        except Exception as e:
            print("显示初始化失败(%s)，只输出文字" % e)

    n_state = 0
    n_ack = 0
    n_ack_hb = 0
    n_text = 0
    hb_sent = 0
    last_state = None
    t_hb = time.ticks_ms()
    t_print = time.ticks_ms()
    t0 = time.ticks_ms()

    print("开始：每 200ms 发一帧心跳，等待 H723 回应...")
    try:
        while True:
            os.exitpoint()
            now = time.ticks_ms()

            if time.ticks_diff(now, t_hb) >= HEARTBEAT_MS:
                t_hb = now
                try:
                    dev.write(build_frame(MSG_HEARTBEAT))
                    hb_sent += 1
                except Exception as e:
                    print("发送失败: %s" % e)

            data = uart_read(dev)
            if data:
                first_rx = (parser.bytes_rx == 0)
                frames = parser.feed(data)
                if first_rx:
                    print("★ 第一次收到字节（共 %d 个）: %s"
                          % (len(data), hexstr(data[:48])))
                    print("  ↑ 能收到字节就说明物理链路是通的："
                          "若里面能看到 YUNTAI-UART7 之类的字样，"
                          "那就是 H723 新固件在发信标")
                for msg_id, seq, payload in frames:
                    if msg_id == MSG_GIMBAL_STATE:
                        n_state += 1
                        if len(payload) == 21:
                            import ustruct
                            last_state = ustruct.unpack("<BBhhhhhhhBI",
                                                        payload)
                    elif msg_id == MSG_ACK:
                        n_ack += 1
                        if len(payload) >= 1 and payload[0] == MSG_HEARTBEAT:
                            n_ack_hb += 1
                    elif msg_id == MSG_TEXT:
                        n_text += 1
                        try:
                            print("  H723: %s" % payload.decode("utf-8"))
                        except Exception:
                            print("  H723 TEXT %r" % payload)
                    elif msg_id == MSG_VERSION:
                        print("  H723 VERSION: %r" % payload)

            if time.ticks_diff(now, t_print) >= PRINT_MS:
                t_print = now
                print("-" * 56)
                print("心跳发出 %d | 收到: 遥测%d ACK%d(其中心跳ACK%d) 文本%d "
                      "| 字节%d ok%d crc_err%d bad_len%d"
                      % (hb_sent, n_state, n_ack, n_ack_hb, n_text,
                         parser.bytes_rx, parser.ok, parser.crc_err,
                         parser.bad_len))
                if last_state is not None:
                    print("  遥测: state=%d fault=%d yaw=%.2f pitch=%.2f "
                          "flags=0x%02X up=%dms"
                          % (last_state[0], last_state[1],
                             last_state[2] / 100.0, last_state[3] / 100.0,
                             last_state[9], last_state[10]))
                # ---- 结论 ----
                if LOOPBACK and parser.ok > 0:
                    print("  >>> 自环测试 OK：K230 的收发都正常"
                          "（说明问题在 H723 侧/接线上）")
                elif LOOPBACK:
                    print("  >>> 自环收不到自己的帧：IO9-IO10 短接了吗？"
                          "或者这个后端不是 IO9/IO10")
                elif n_state > 0 and n_ack_hb > 0:
                    print("  >>> 双向通信 OK（H723 收到心跳并回了 ACK）")
                elif n_state > 0:
                    print("  >>> H723->K230 通了；还没收到心跳 ACK："
                          "等 1~2 秒，若一直没有看 K230 TX / H723 RX 这根线")
                elif parser.bytes_rx > 0 and parser.ok == 0:
                    print("  >>> 收到字节但一帧都解不出来：波特率或线材问题"
                          "（两边都必须 115200 8N1，共地）")
                else:
                    print("  >>> 一个字节都没收到。按顺序查：")
                    print("      1) H723 烧的是新固件吗？旧固件走 USART1，"
                          "UART7 上什么都不会发")
                    print("      2) 接线是否交叉：K230 IO9(TX)->H723 UART7 的 RX，"
                          "IO10(RX)->UART7 的 TX，GND 必须共地")
                    print("      3) H723 上电了吗（电机可以不供电）")

                if display_ok and canvas is not None:
                    canvas.clear()
                    y = 6
                    canvas.draw_string_advanced(6, y, 24,
                                                "K230 <-> H723 LINK TEST",
                                                color=(255, 255, 255))
                    y += 34
                    lines = [
                        "uart: %s" % uart_name,
                        "hb_sent=%d  state=%d  ack=%d(hb %d)  text=%d"
                        % (hb_sent, n_state, n_ack, n_ack_hb, n_text),
                        "rx_bytes=%d ok=%d crc_err=%d bad_len=%d"
                        % (parser.bytes_rx, parser.ok, parser.crc_err,
                           parser.bad_len),
                    ]
                    if last_state is not None:
                        lines.append("state=%d fault=%d up=%dms"
                                     % (last_state[0], last_state[1],
                                        last_state[10]))
                        lines.append("yaw=%.2f pitch=%.2f flags=0x%02X"
                                     % (last_state[2] / 100.0,
                                        last_state[3] / 100.0,
                                        last_state[9]))
                    if n_state > 0 and n_ack_hb > 0:
                        lines.append("RESULT: BOTH OK")
                        col = (0, 255, 0)
                    elif n_state > 0 or parser.bytes_rx > 0:
                        lines.append("RESULT: PARTIAL")
                        col = (255, 220, 0)
                    else:
                        lines.append("RESULT: NO DATA")
                        col = (255, 80, 80)
                    for ln in lines:
                        canvas.draw_string_advanced(6, y, 20, ln,
                                                    color=(200, 255, 200))
                        y += 28
                    canvas.draw_string_advanced(6, y + 10, 22, lines[-1],
                                                color=col)
                    Display.show_image(canvas)
            time.sleep_ms(2)
    except KeyboardInterrupt:
        print("用户停止")
    finally:
        try:
            dev.deinit()
        except Exception:
            pass
        if display_ok:
            try:
                Display.deinit()
                os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
                time.sleep_ms(100)
                MediaManager.deinit()
            except Exception:
                pass
        if table:
            print("-" * 56)
            print("串口后端对照表：")
            for label, st, err, nb, nf in table:
                print("  %-22s %-6s 字节=%-6d 帧=%-4d %s"
                      % (label, st, nb, nf, err))
        print("已退出。通信统计：ok=%d crc_err=%d 字节=%d"
              % (parser.ok, parser.crc_err, parser.bytes_rx))


main()
