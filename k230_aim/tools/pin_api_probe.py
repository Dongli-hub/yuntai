# -*- coding: utf-8 -*-
"""pin_api_probe.py —— 问清楚"这块板子读一个引脚电平的正确写法"

为什么要问：wire_check 里我用 FPIOA.set_function(...)+Pin() 读 IO9/IO10，
结果连内部上拉都读成 0，这不符合物理常识（悬空脚应该被上拉成高），
说明我的读法在这块固件上不对。这个脚本一次把正确姿势找出来。

它会打印：
  1) 亚博官方读引脚的实现（YbKey.py / YbRGB.py 源码）
  2) FPIOA 里和 GPIO 相关的常量有哪些
  3) 用三四种不同写法读 IO10 的电平，看哪个能读出真实值
"""
import time

print("=" * 58)
print("引脚 API 探测 pin_api_probe")
print("=" * 58)

for path in ("/sdcard/ybUtils/YbKey.py", "/sdcard/ybUtils/YbRGB.py"):
    try:
        src = open(path).read()
        print("-" * 58)
        print("---- %s（前 900 字）----" % path)
        print(src[:900])
    except Exception as e:
        print("%s 读不了: %s" % (path, e))

print("-" * 58)
try:
    from machine import FPIOA
    fp = FPIOA()
    gpio_names = [n for n in dir(FPIOA) if "GPIO" in n.upper()]
    print("FPIOA 里含 GPIO 的常量（前 12 个）: %s" % gpio_names[:12])
    print("FPIOA.UART1_TXD = %s" % getattr(FPIOA, "UART1_TXD", "无"))
    print("FPIOA.UART1_RXD = %s" % getattr(FPIOA, "UART1_RXD", "无"))
    print("FPIOA.GPIO10 = %s   FPIOA.GPIO9 = %s"
          % (getattr(FPIOA, "GPIO10", "无"), getattr(FPIOA, "GPIO9", "无")))
    print("当前 IO9 func=%s   IO10 func=%s"
          % (fp.get_pin_func(9), fp.get_pin_func(10)))
except Exception as e:
    print("FPIOA 查询失败: %s" % e)

print("-" * 58)
try:
    from machine import Pin
    print("Pin 的属性: %s"
          % [n for n in dir(Pin) if n.isupper()][:12])
except Exception as e:
    print("Pin 查询失败: %s" % e)


def read_level(title, make_pin, n=200):
    try:
        p = make_pin()
        ones = 0
        flips = 0
        last = None
        for _ in range(n):
            v = 1 if p.value() else 0
            ones += v
            if last is not None and v != last:
                flips += 1
            last = v
            time.sleep_ms(1)
        print("  %-40s 高=%3d%%  跳变=%d" % (title, ones * 100 // n, flips))
    except Exception as e:
        print("  %-40s 失败: %s" % (title, e))


print("-" * 58)
print("用不同写法读 IO10（K230 的 RXD，应接 H723 的 TX/PE08）：")


def way1():
    from machine import Pin
    return Pin(10, Pin.IN, Pin.PULL_UP)


def way2():
    from machine import FPIOA, Pin
    fp = FPIOA()
    fn = getattr(FPIOA, "GPIO10", None)
    fp.set_function(10, fn if fn is not None else 10, ie=1, oe=0, pu=1)
    return Pin(10, Pin.IN)


def way3():
    from machine import FPIOA, Pin
    fp = FPIOA()
    fn = getattr(FPIOA, "GPIO10", None)
    fp.set_function(10, fn if fn is not None else 10, ie=1, oe=0, pu=0)
    return Pin(10, Pin.IN)


read_level("Pin(10, IN, PULL_UP)", way1)
read_level("FPIOA(10,GPIO,pu=1)+Pin", way2)
read_level("FPIOA(10,GPIO,pu=0)+Pin", way3)

# 再来一次，但每次采样前立刻重新配置（防止被别的代码改回去）
print("-" * 58)
print("同样三种写法读 IO9（K230 的 TXD，应接 H723 的 RX/PE07）：")


def w1():
    from machine import Pin
    return Pin(9, Pin.IN, Pin.PULL_UP)


def w2():
    from machine import FPIOA, Pin
    fp = FPIOA()
    fn = getattr(FPIOA, "GPIO9", None)
    fp.set_function(9, fn if fn is not None else 9, ie=1, oe=0, pu=1)
    return Pin(9, Pin.IN)


read_level("Pin(9, IN, PULL_UP)", w1)
read_level("FPIOA(9,GPIO,pu=1)+Pin", w2)

print("=" * 58)
print("请把以上全部输出发回")
print("=" * 58)
