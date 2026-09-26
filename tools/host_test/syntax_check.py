#!/usr/bin/env python3
"""PC 端语法/类型检查：用最小 HAL 桩把改过的 C 文件编一遍。

    python tools/host_test/syntax_check.py

为什么值得做：这里没有 arm-none-eabi-gcc，等你在 CubeIDE 里编译才发现
拼写错误/少个参数就白跑一趟。这个脚本能在几秒内把这些低级错误抓出来。

注意：它只做 -fsyntax-only，不链接、不生成任何固件；
它的 HAL 桩也只保证"类型和签名存在"，不代表真实 HAL 的行为。
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(os.path.dirname(HERE))
STUB = os.path.join(HERE, "stub")
INC = os.path.join(PROJ, "Core", "Inc")
SRC = os.path.join(PROJ, "Core", "Src")

# 只检查"我自己写的、且依赖 HAL 的"文件；gimbal_proto.c 已在 run_test.py 里真编译过
FILES = [
    "yuntai_task.c",
    "gimbal_link.c",
    "jc4310.c",
    "can_bsp.c",
    "debug_uart.c",
    "pid.c",
]


def main():
    cc = os.environ.get("CC", "gcc")
    failed = 0
    for name in FILES:
        cmd = [cc, "-fsyntax-only", "-std=gnu11", "-Wall", "-Wextra",
               "-Wno-unused-parameter", "-Wno-unused-variable",
               # 隐式声明（调用了没定义的函数）在 CubeIDE 的新版 GCC 里是错误，
               # 而且在链接期一定会炸（undefined reference）。这里当错误对待。
               "-Werror=implicit-function-declaration",
               "-include", os.path.join(STUB, "pre.h"),
               "-I", STUB, "-I", INC,
               os.path.join(SRC, name)]
        # 必须显式指定 utf-8：源码里有中文，gcc 输出也是 UTF-8，
        # 用系统默认编码（GBK）解码会直接抛异常
        r = subprocess.run(cmd, capture_output=True, text=True,
                           encoding="utf-8", errors="replace")
        err = (r.stdout or "") + (r.stderr or "")
        if r.returncode != 0:
            failed += 1
            print("  [FAIL] %s" % name)
            print("\n".join("      " + l for l in err.strip().splitlines()[:20]))
        else:
            warn = [l for l in err.strip().splitlines()
                    if "warning" in l and "unused" not in l]
            print("  [PASS] %-16s%s" % (name, ("  %d 条告警" % len(warn)) if warn else ""))
            for l in warn[:5]:
                print("      " + l)
    print("-" * 60)
    print("共 %d 个文件，失败 %d 个" % (len(FILES), failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
