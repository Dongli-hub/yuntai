# -*- coding: utf-8 -*-
"""wire_check.py —— 不用拆线、不用短接，查"对面这根线到底通不通"

原理：把 K230 的引脚临时从"串口"改成"普通输入脚"，分两次采样：
    第一次：不带上拉（pu=0）—— 这时引脚电平完全由外部决定
    第二次：打开内部上拉（pu=1）—— 如果外面是悬空的，会被拉成高

三种结果的含义：
  A. 两次都读到高（且偶尔有低）  -> 对面在驱动这根线，而且很可能正在发数据
  B. 无上拉读低、加上拉读高      -> 这根线悬空：没接上 / 对面没供电 / 对面没驱动
  C. 加上拉还是读低              -> 被拉死到地（接错脚或短路）

要测的两根线：
  IO10（K230 的 RXD）应接 H723 的 TX(PE08) —— 这个是重点，对面应该有驱动
  IO9 （K230 的 TXD）应接 H723 的 RX(PE07) —— 对面是输入脚，正常应表现为悬空

跑完记得重启 K230（或复位）把引脚恢复成串口。
"""
import time

SAMPLE_N = 400
SAMPLE_MS = 1


def sample_pin(pin, pull):
    """把 pin 配成 GPIO 输入，采样 SAMPLE_N 次，返回 (高电平次数, 跳变次数)。"""
    from machine import FPIOA, Pin
    fp = FPIOA()
    # GPIO 功能号：优先找 FPIOA.GPIOxx 常量；找不到就用引脚号本身
    # （亚博这块固件里 IO10 默认 func=10，正好就是 GPIO10）
    fn = getattr(FPIOA, "GPIO%d" % pin, None)
    if fn is None:
        fn = pin
    fp.set_function(pin, fn, ie=1, oe=0, pu=pull)
    p = Pin(pin, Pin.IN)
    ones = 0
    flips = 0
    last = None
    for _ in range(SAMPLE_N):
        v = 1 if p.value() else 0
        ones += v
        if last is not None and v != last:
            flips += 1
        last = v
        time.sleep_ms(SAMPLE_MS)
    return ones, flips


def describe(pin, name, ones_nopull, flips_nopull, ones_pull, flips_pull):
    pct_no = ones_nopull * 100 // SAMPLE_N
    pct_pu = ones_pull * 100 // SAMPLE_N
    print("-" * 58)
    print("%s（IO%d）:" % (name, pin))
    print("   不上拉: 高电平 %d%%  跳变 %d 次" % (pct_no, flips_nopull))
    print("   加上拉: 高电平 %d%%  跳变 %d 次" % (pct_pu, flips_pull))
    if pct_no == 0 and pct_pu == 0:
        print("   >>> 被拉死到低电平：接错脚了，或者这根线对地短路")
    elif pct_no == 0 and pct_pu > 80:
        print("   >>> 悬空：这根线没接到对面的驱动脚上"
              "（没接、对面没供电、或对面那个脚是输入）")
    elif pct_no > 80 and pct_pu > 80:
        if flips_nopull > 0 or flips_pull > 0:
            print("   >>> 有数字信号在动！对面正在发数据"
                  "（这条线是通的，而且是活的）")
        else:
            print("   >>> 线被稳定驱动为高：对面已上电、空闲中"
                  "（如果对面应该在不停发数据，那就说明它没在发）")
    else:
        print("   >>> 电平不稳定（%d%%/%d%%）：接触不良或强干扰" % (pct_no, pct_pu))


print("=" * 58)
print("线缆体检 wire_check")
print("=" * 58)

n1, f1 = sample_pin(10, 0)
n2, f2 = sample_pin(10, 1)
describe(10, "K230 的 RXD（应接 H723 的 TX/PE08）", n1, f1, n2, f2)

n3, f3 = sample_pin(9, 0)
n4, f4 = sample_pin(9, 1)
describe(9, "K230 的 TXD（应接 H723 的 RX/PE07）", n3, f3, n4, f4)

print("=" * 58)
print("怎么读结果：")
print("  · IO10 显示“有数字信号在动/稳定驱动为高” -> 对面在发，"
      "问题就在 K230 的串口解析这一侧")
print("  · IO10 显示“悬空” -> 对面那根线根本没在驱动（H723 没跑 / 没供电 /"
      " 线没接上）")
print("  · IO10 显示“被拉死到低” -> 接错脚或短路")
print("跑完请复位/重启 K230，把 IO9/IO10 恢复成串口。")
print("=" * 58)
