# -*- coding: utf-8 -*-
"""sync_check.py —— 一次把三处"必须一样、但分散在别的文件里"的东西查掉（PC 上跑）

    python k230_aim/tools/sync_check.py

为什么要有它：这个工程有三处必须"逐字/逐值一致"，历史上已经因为忘了同步
白跑过好几轮现场：

  1) main.py 与 tools/vision_check.py 的视觉代码
     —— 用户看的是 vision_check 的画面，跑的是 main.py 的逻辑。
        这里逐函数比源码、逐常量比数值（含 S_SCALE 缩放之后的最终值）。
  2) main.py 的 AIM payload 长度 与 H723 的 GP_AIM_PAYLOAD_LEN
     —— 长度对不上 = 帧格式对不上；串口上什么都看不出来，极难定位。
  3) 顺便打印两边算出来的分辨率 / K，方便和现场日志对账。

返回码：0 = 全部一致；1 = 有不一致（细节打印在上面）。
"""
import ast
import builtins
import io
import os
import re
import struct
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
MAIN = os.path.join(ROOT, "k230_aim", "main.py")
TOOL = os.path.join(ROOT, "k230_aim", "tools", "vision_check.py")
PROTO = os.path.join(ROOT, "Core", "Inc", "gimbal_proto.h")

# 这几个名字两边"本来就该不一样"，不参与逐字比对：
#   main   = 整机主循环（状态机/串口/画图），vision_check 只留视觉骨架
#   log    = main 要写 SD 卡，vision_check 只 print
#   apply_flip = 相机翻转的兼容写法，两边各自维护
IGNORE = {"main", "log", "apply_flip", "ClosePaperDetector"}


def parse(path):
    return ast.parse(io.open(path, encoding="utf-8").read(), path)


def defs(tree):
    """顶层函数/类：名字 -> 源码（ast.unparse 归一化缩进/注释差异）"""
    out = {}
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.ClassDef)):
            out[node.name] = ast.unparse(node)
    return out


def consts(tree):
    """顶层简单赋值：名字 -> 值（算不出来的跳过，最后单独提示）"""
    env = {"int": int, "round": round, "float": float,
           "list": list, "tuple": tuple}
    out = {}
    skipped = []
    for node in tree.body:
        if not (isinstance(node, ast.Assign) and len(node.targets) == 1 and
                isinstance(node.targets[0], ast.Name)):
            continue
        name = node.targets[0].id
        try:
            out[name] = eval(compile(ast.Expression(node.value), "<c>", "eval"),
                             dict(env))
            env[name] = out[name]
        except Exception:
            skipped.append(name)
            env.pop(name, None)
    return out, skipped


def diff_defs(a, b, bad):
    only_a = sorted(set(a) - set(b))
    only_b = sorted(set(b) - set(a))
    if only_a:
        print("  [提示] 只在 main.py 里: %s" % ", ".join(only_a))
    if only_b:
        print("  [提示] 只在 vision_check.py 里: %s" % ", ".join(only_b))
    n = 0
    for name in sorted(set(a) & set(b)):
        if name in IGNORE:
            continue
        if a[name] != b[name]:
            bad.append("函数/类不一致: %s" % name)
            print("  [FAIL] %s 两边源码不同" % name)
        n += 1
    print("  函数/类逐字比对: %d 个" % n)


def diff_consts(a, b, bad):
    n = 0
    for name in sorted(set(a) & set(b)):
        va, vb = a[name], b[name]
        if isinstance(va, float) or isinstance(vb, float):
            same = abs(float(va) - float(vb)) < 1e-9
        else:
            same = (va == vb)
        n += 1
        if not same:
            bad.append("常量不一致: %s (%r vs %r)" % (name, va, vb))
            print("  [FAIL] %s: main=%r  vision_check=%r" % (name, va, vb))
    print("  常量数值比对: %d 个" % n)


def aim_len_check(bad):
    """main.py 的 AIM payload 长度 vs H723 的 GP_AIM_PAYLOAD_LEN[_K]"""
    src = io.open(MAIN, encoding="utf-8").read()
    m = re.search(r'def pack_aim_px\(.*?ustruct\.pack\(\s*"<([^"]+)"',
                  src, re.S)
    if not m:
        bad.append("main.py 里找不到 pack_aim_px 的格式串")
        return
    fmt = m.group(1)
    py_len = struct.calcsize("<" + fmt)
    hdr = io.open(PROTO, encoding="utf-8").read()
    m2 = re.search(r"#define\s+GP_AIM_PAYLOAD_LEN\s+(\d+)u", hdr)
    m3 = re.search(r"#define\s+GP_AIM_PAYLOAD_LEN_K\s+(\d+)u", hdr)
    if not m2:
        bad.append("gimbal_proto.h 里找不到 GP_AIM_PAYLOAD_LEN")
        return
    len_base = int(m2.group(1))
    len_k = int(m3.group(1)) if m3 else 0
    print('  main.py AIM payload = %d 字节（"<%s"）' % (py_len, fmt))
    print("  H723  GP_AIM_PAYLOAD_LEN=%d  PAYLOAD_LEN_K=%d" % (len_base, len_k))
    if py_len not in (len_base, len_k):
        bad.append("AIM payload 长度对不上：K230 发 %d，H723 认 %d/%d"
                   % (py_len, len_base, len_k))
        print("  [FAIL] 长度对不上 —— 改完一边必须改另一边！")
    else:
        print("  [OK] AIM payload 长度一致")


def undef_check(path, bad):
    """找出"用了但没定义"的名字（拼写错/改名漏改）。

    为什么加这条：2026-10-11 把 `_bin_ok` 改名成 `_bin_mode` 时漏了两处引用，
    板子上跑起来才炸 `NameError: name '_bin_ok' isn't defined` ——
    py_compile 只管语法，这种错它抓不到，现场白跑一轮。
    这里只做"模块级能看见的名字"这一个层面的检查（赋值/import/def/class/
    函数参数/for 目标/with as/except as/全局声明 + 内建），够抓改名漏改这类错。
    """
    tree = ast.parse(io.open(path, encoding="utf-8").read(), path)
    defined = set(dir(builtins))
    loaded = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name):
                    defined.add(t.id)
        elif isinstance(node, (ast.AugAssign, ast.AnnAssign)):
            if isinstance(node.target, ast.Name):
                defined.add(node.target.id)
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef,
                               ast.ClassDef)):
            defined.add(node.name)
            if isinstance(node, ast.FunctionDef):
                a = node.args
                allargs = (list(a.posonlyargs) + list(a.args) +
                           list(a.kwonlyargs))
                for x in allargs:
                    defined.add(x.arg)
                if a.vararg:
                    defined.add(a.vararg.arg)
                if a.kwarg:
                    defined.add(a.kwarg.arg)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            for al in node.names:
                nm = (al.asname or al.name)
                defined.add(nm.split(".")[0] if al.asname is None else nm)
        elif isinstance(node, ast.Global) or isinstance(node, ast.Nonlocal):
            for n in node.names:
                defined.add(n)
        elif isinstance(node, (ast.For, ast.AsyncFor)):
            if isinstance(node.target, ast.Name):
                defined.add(node.target.id)
        elif isinstance(node, ast.withitem):
            if node.optional_vars is not None and \
                    isinstance(node.optional_vars, ast.Name):
                defined.add(node.optional_vars.id)
        elif isinstance(node, ast.ExceptHandler):
            if node.name:
                defined.add(node.name)
        elif isinstance(node, ast.Lambda):
            # lambda 的参数也是"定义的名字"（items.sort(key=lambda it: ...)）
            a = node.args
            for x in (list(a.posonlyargs) + list(a.args) +
                      list(a.kwonlyargs)):
                defined.add(x.arg)
            if a.vararg:
                defined.add(a.vararg.arg)
            if a.kwarg:
                defined.add(a.kwarg.arg)
        elif isinstance(node, ast.comprehension):
            if isinstance(node.target, ast.Name):
                defined.add(node.target.id)
        elif isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
            defined.add(node.id)
        elif isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load):
            # 归一化：<stdin> 之类的来源不用管
            loaded.setdefault(node.id, node.lineno)
    miss = sorted(k for k in loaded if k not in defined)
    if miss:
        for k in miss:
            bad.append("%s: 用了没定义的名字 '%s'（第 %d 行附近）"
                       % (os.path.basename(path), k, loaded[k]))
            print("  [FAIL] %s: '%s' 未定义（第 %d 行）"
                  % (os.path.basename(path), k, loaded[k]))
    else:
        print("  未定义名字检查: %s OK" % os.path.basename(path))


def main():
    bad = []
    tm, tt = parse(MAIN), parse(TOOL)
    print("1) main.py  vs  tools/vision_check.py")
    diff_defs(defs(tm), defs(tt), bad)
    cm, skm = consts(tm)
    ct, skt = consts(tt)
    diff_consts(cm, ct, bad)
    for k in ("PROC_W", "PROC_H", "S_SCALE", "VIS_K"):
        print("  %-8s main=%r  vision_check=%r" % (k, cm.get(k), ct.get(k)))
    print("2) main.py 的 AIM payload  vs  H723 gimbal_proto.h")
    aim_len_check(bad)
    print("3) 未定义名字检查（改了名漏改引用这种错，py_compile 抓不到）")
    undef_check(MAIN, bad)
    undef_check(TOOL, bad)
    print("-" * 60)
    if bad:
        print("结果：不一致 %d 处" % len(bad))
        for b in bad:
            print("  - %s" % b)
        return 1
    print("结果：全部一致")
    return 0


if __name__ == "__main__":
    sys.exit(main())
