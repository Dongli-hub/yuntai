#!/usr/bin/env python3
"""极简测试运行器（不依赖 pytest）。

    python tests/run_all.py

会自动发现同目录下的 test_*.py，调用其中所有 test_* 函数。
任何 AssertionError 都算失败，最后给出汇总。
"""

import importlib.util
import os
import sys
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)


def load_module(path):
    name = os.path.splitext(os.path.basename(path))[0]
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    files = sorted(f for f in os.listdir(HERE)
                   if f.startswith("test_") and f.endswith(".py"))
    total = failed = 0
    for f in files:
        mod = load_module(os.path.join(HERE, f))
        names = [n for n in dir(mod) if n.startswith("test_") and callable(getattr(mod, n))]
        for name in names:
            total += 1
            t0 = time.time()
            try:
                getattr(mod, name)()
                print("  [PASS] %-28s %-34s %.0fms"
                      % (f, name, (time.time() - t0) * 1000))
            except Exception:
                failed += 1
                print("  [FAIL] %-28s %-34s" % (f, name))
                traceback.print_exc()
    print("-" * 72)
    print("共 %d 项，失败 %d 项" % (total, failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

