# -*- coding: utf-8 -*-
"""t0_env.py —— K230 环境探测（第一步，不接任何外设也能跑）

在 CanMV IDE 里直接运行，把「串行终端」的输出发回来。
"""
import sys
import os
import time


def show(title, fn):
    try:
        print("%-28s %s" % (title, fn()))
    except Exception as e:
        print("%-28s FAIL: %s" % (title, e))


print("=" * 60)
print("K230 环境探测 t0_env")
print("=" * 60)

show("sys.implementation", lambda: sys.implementation)
show("sys.version", lambda: sys.version)
show("os.uname()", lambda: os.uname())
show("machine.unique_id()", lambda: __import__("machine").unique_id())

print("-" * 60)
print("模块可用性：")
for name in ("media.sensor", "media.display", "media.media", "image",
             "machine", "ustruct", "struct", "math", "time",
             "ybUtils.YbUart", "ybUtils.YbKey", "libs.YbProtocol",
             "nncase", "ulab", "numpy", "cv2"):
    try:
        __import__(name)
        print("  %-22s OK" % name)
    except Exception as e:
        print("  %-22s FAIL (%s)" % (name, e))

print("-" * 60)
print("FPIOA / UART 可用引脚：")
try:
    from machine import FPIOA
    fpioa = FPIOA()
    for fn_name in ("UART0_TXD", "UART1_TXD", "UART2_TXD", "UART3_TXD",
                    "UART1_RXD", "UART3_RXD"):
        try:
            fpioa.help(getattr(FPIOA, fn_name), func=True)
        except Exception as e:
            print("  %s: %s" % (fn_name, e))
except Exception as e:
    print("  FPIOA FAIL: %s" % e)

print("-" * 60)
print("UART 实例化测试（不接线也不影响）：")


def try_uart(title, unit, tx, rx):
    try:
        from machine import UART, Pin
        u = UART(unit, baudrate=115200, tx=Pin(tx), rx=Pin(rx),
                 bits=8, parity=None, stop=0)
        u.write(b"\xAA")
        u.deinit()
        print("  %-34s OK" % title)
        return True
    except Exception as e:
        print("  %-34s FAIL (%s)" % (title, e))
        return False


try_uart("UART1 tx=IO9  rx=IO10 (EXPORT)", 1, 9, 10)
try_uart("UART3 tx=IO32 rx=IO33 (12Pin)", 3, 32, 33)

try:
    from ybUtils.YbUart import YbUart
    u = YbUart(baudrate=115200)
    u.write(b"\xAA")
    try:
        u.deinit()
    except Exception:
        pass
    print("  %-34s OK" % "YbUart(115200) 亚博封装")
except Exception as e:
    print("  %-34s FAIL (%s)" % ("YbUart", e))

print("-" * 60)
print("屏幕 / 摄像头（只做一次初始化，随后立即释放）：")
sensor = None
try:
    from media.sensor import Sensor, CAM_CHN_ID_0
    sensor = Sensor()
    sensor.reset()
    print("  Sensor OK")
    try:
        print("  sensor id: %s" % sensor.get_sensor_id())
    except Exception as e:
        print("  sensor id 不可读 (%s)" % e)
    sensor.set_framesize(width=640, height=480, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
    print("  640x480 RGB565 设置 OK")
    sensor.stop()
    print("  已释放")
except Exception as e:
    print("  Sensor FAIL: %s" % e)

print("=" * 60)
print("探测结束。请把以上全部输出发回。")
print("=" * 60)
