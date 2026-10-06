# -*- coding: utf-8 -*-
"""k230_uart_probe.py —— 查清这块板子的串口到底挂在哪些引脚上

在 CanMV IDE 里直接运行，把输出发回来。它会打印：
  1) UART0~UART4 的 TXD/RXD 当前分别分配给了哪个 IO 引脚（FPIOA）
  2) /sdcard/ybUtils/YbUart.py 的源码前一段（看亚博封装用的哪个 UART/引脚）
  3) /sdcard/ybUtils 目录下有什么
"""
import os

print("=" * 56)
print("K230 串口 / FPIOA 探测")
print("=" * 56)

try:
    from machine import FPIOA
    fp = FPIOA()
    print("UART 功能 -> 引脚:")
    for name in ("UART0_TXD", "UART0_RXD",
                 "UART1_TXD", "UART1_RXD",
                 "UART2_TXD", "UART2_RXD",
                 "UART3_TXD", "UART3_RXD",
                 "UART4_TXD", "UART4_RXD"):
        try:
            func = getattr(FPIOA, name)
            print("  %-11s -> IO%s" % (name, fp.get_pin_num(func)))
        except Exception as e:
            print("  %-11s 查询失败 (%s)" % (name, e))
except Exception as e:
    print("FPIOA 不可用: %s" % e)

print("-" * 56)
try:
    print("IO9 / IO10 / IO32 / IO33 当前功能:")
    fp2 = FPIOA()
    for pin in (9, 10, 32, 33):
        try:
            print("  IO%-3d func=%s" % (pin, fp2.get_pin_func(pin)))
        except Exception as e:
            print("  IO%-3d 查询失败 (%s)" % (pin, e))
except Exception:
    pass

print("-" * 56)
try:
    print("/sdcard/ybUtils 目录: %s" % os.listdir("/sdcard/ybUtils"))
except Exception as e:
    print("列目录失败: %s" % e)

for path in ("/sdcard/ybUtils/YbUart.py", "/sdcard/ybUtils/YbUart.pyc"):
    try:
        with open(path, "r") as f:
            src = f.read()
        print("-" * 56)
        print("---- %s 前 1600 字符 ----" % path)
        print(src[:1600])
        break
    except Exception as e:
        print("%s 读不了: %s" % (path, e))

print("=" * 56)
print("探测结束，请把以上输出发回")
print("=" * 56)
