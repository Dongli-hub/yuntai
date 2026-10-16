# -*- coding: utf-8 -*-
"""一次性脚本：把 main.py 的「国一那套」相关代码原样搬到 vision_check.py。

搬三样（这三样历史上一直是手工复制、漏过好几次）：
  1) GY_TH 等参数的**注释块 + 赋值行**（CONSTS 里列的名字）；
  2) _GY_FUSE_ERR / _GY_MASK_CACHE 这类模块级小变量（EXTRA_LINES 里列的）；
  3) _cv2_find 函数块 / PaperDetector 类块（含它们头顶挂着的注释）。

做法：两边都用"头顶注释块 + ast 的 end_lineno"定位，整块替换 —— 不用手抄。
跑完请立刻 python k230_aim/tools/sync_check.py 复核（本脚本只负责搬运，
一致性判定以 sync_check 为准）。跑完可以删掉本文件。
"""
import ast
import io
import os
import re
import sys


ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
MAIN = os.path.join(ROOT, "k230_aim", "main.py")
TOOL = os.path.join(ROOT, "k230_aim", "tools", "vision_check.py")

# 要搬的模块级"简单赋值"名字（连同它头顶的注释块一起搬）
CONSTS = ["GY_TH", "GY_CLOSE_SIZE", "CV2_EVERY", "PAPER_RING_RISE_MIN",
          "MASK_SCALE", "CROP_K", "CROP_MARGIN_PX", "CROP_MAX_FRAC"]
# 目标文件还没有这条常量时，插到哪个常量后面（保持和 main.py 同样的位置）
INSERT_AFTER = {"PAPER_RING_RISE_MIN": "PAPER_CONTRAST_MIN",
                "MASK_SCALE": "GY_CLOSE_SIZE",
                "CROP_K": "GY_CLOSE_SIZE",
                "CROP_MARGIN_PX": "CROP_K",
                "CROP_MAX_FRAC": "CROP_MARGIN_PX"}
# 要搬的"整行"定义（注释 + 赋值，看名字在两边各出现一次）
EXTRA_LINES = ["_GY_FUSE_ERR", "_GY_MASK_CACHE", "_WANT_MASK",
               "_CROP_MISS", "_CROP_ERR"]
# 要搬的函数/类（整块替换，含头顶注释）
BLOCKS = ["_cv2_find", "PaperDetector", "SpotDetector", "log_scene", "px_luma",
          "paper_contrast", "prof_line"]


def read(path):
    return io.open(path, encoding="utf-8").read().split("\n")


def head_start(lines, start):
    """从索引 start（0-based，某条语句的第一行）往上吃注释/空行。"""
    i = start - 1
    while i >= 0:
        s = lines[i].strip()
        if s == "" or s.startswith("#"):
            i -= 1
        else:
            break
    return i + 1


def block_span(lines, name):
    """返回 (起点, 终点) —— 含头顶注释，0-based，终点不含。"""
    tree = ast.parse("\n".join(lines))
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and \
                node.name == name:
            return head_start(lines, node.lineno - 1), node.end_lineno
    raise SystemExit("找不到 %s" % name)


def const_span(lines, name, quiet=False):
    """顶层简单赋值（含头顶注释块）的 (起点, 终点)。"""
    tree = ast.parse("\n".join(lines))
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and \
                isinstance(node.targets[0], ast.Name) and \
                node.targets[0].id == name:
            return head_start(lines, node.lineno - 1), node.end_lineno
    if quiet:
        return None
    raise SystemExit("找不到常量 %s" % name)


def line_span(lines, name):
    """单独一行的赋值（+头顶注释）的 (起点, 终点)。"""
    for i, ln in enumerate(lines):
        if re.match(r"^%s\s*=" % re.escape(name), ln):
            return head_start(lines, i), i + 1
    return None


def insert_span(lines, anchor):
    """目标文件里还没有这条时：插到 anchor 那行的后面（(a, a) = 插入点）。"""
    for i, ln in enumerate(lines):
        if re.match(r"^%s\s*=" % re.escape(anchor), ln):
            return i + 1, i + 1
    raise SystemExit("找不到锚点行 %s" % anchor)


def main():
    m = read(MAIN)
    t = read(TOOL)
    # ---- 第一遍：目标文件还缺的常量，按 INSERT_AFTER 逐个插进去 ----
    #   （一次只插一条、插完重新定位 —— 否则同一锚点上的多条会插错位置）
    for name in CONSTS:
        if name not in INSERT_AFTER:
            continue
        if const_span(t, name, quiet=True) is not None:
            continue
        anchor = INSERT_AFTER[name]
        a1, b1 = const_span(m, name)
        _a, _b = const_span(t, anchor)
        print("  [插入] %-18s 在 %s 之后" % (name, anchor))
        t = t[:_b] + m[a1:b1] + t[_b:]
    # ---- 第二遍：常量/小变量/大块，从文件末尾往前替换 ----
    jobs = []
    for name in CONSTS:
        a1, b1 = const_span(m, name)
        jobs.append(("const:" + name, (a1, b1), const_span(t, name)))
    for name in EXTRA_LINES:
        job = line_span(t, name)
        if job is None:
            job = insert_span(t, "_GY_ERR")
        jobs.append(("line:" + name, line_span(m, name), job))
    for name in BLOCKS:
        jobs.append(("block:" + name, block_span(m, name), block_span(t, name)))
    # 从文件末尾往前改，前面的行号才不会失效
    jobs.sort(key=lambda j: j[2][0], reverse=True)
    for tag, (a1, b1), (a2, b2) in jobs:
        seg = m[a1:b1]
        print("%-22s main[%d:%d] -> vc[%d:%d]  %d 行"
              % (tag, a1 + 1, b1, a2 + 1, b2, len(seg)))
        t = t[:a2] + seg + t[b2:]
    out = "\n".join(t)
    if not out.endswith("\n"):
        out += "\n"
    io.open(TOOL, "w", encoding="utf-8", newline="\n").write(out)
    print("已写入 %s" % TOOL)
    return 0


if __name__ == "__main__":
    sys.exit(main())
