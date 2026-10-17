# -*- coding: utf-8 -*-
"""一次性脚本：把 main.py 的「国一那套」相关代码原样搬到 vision_check.py。

搬三样（这三样历史上一直是手工复制、漏过好几次）：
  1) GY_TH 等参数的**注释块 + 赋值行**（CONSTS 里列的名字）；
  2) _GY_FUSE_ERR 这类模块级小变量（EXTRA_LINES 里列的）；
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
CONSTS = ["GY_TH", "GY_TH_FAR",
          "GY_CLOSE_SIZE", "GY_CLOSE_RETRY", "CV2_EVERY",
          "GY_CANNY1", "GY_CANNY2",
          "PAPER_MIN_SIDE", "QUAD_ANGLE_COS_MAX",
          "QUAD_OPPOSITE_DIFF_MAX", "QUAD_SIDE_RATIO_MAX",
          "GY_AREA_RATIO",
          "PAPER_MIN_LONG", "PAPER_MAX_LONG",
          "PAPER_CONTRAST_MIN", "PAPER_DARK_FRAC_MIN",
          "PAPER_INNER_MAX", "INTERIOR_TRANS_MIN",
          "EXPO_TARGET_MEAN",
          "EXPO_SW_GAIN", "EXPO_AUTO_GAIN", "EXPO_SW_MIN", "EXPO_SW_MAX",
          "EXPO_HW_GAIN"]
# 目标文件还没有这条常量时，插到哪个常量后面（保持和 main.py 同样的位置）
INSERT_AFTER = {"GY_TH_FAR": "GY_TH",
                "GY_CLOSE_RETRY": "GY_CLOSE_SIZE",
                "GY_CANNY1": "GY_CLOSE_RETRY",
                "GY_CANNY2": "GY_CANNY1",
                "PAPER_MIN_SIDE": "GY_CANNY2",
                "QUAD_ANGLE_COS_MAX": "PAPER_MIN_SIDE",
                "QUAD_OPPOSITE_DIFF_MAX": "QUAD_ANGLE_COS_MAX",
                "QUAD_SIDE_RATIO_MAX": "QUAD_OPPOSITE_DIFF_MAX",
                "GY_AREA_RATIO": "QUAD_SIDE_RATIO_MAX",
                "PAPER_MIN_LONG": "GY_AREA_RATIO",
                "PAPER_MAX_LONG": "PAPER_MIN_LONG",
                "PAPER_CONTRAST_MIN": "PAPER_ASPECT_MAX",
                "PAPER_DARK_FRAC_MIN": "PAPER_DARK_MARGIN",
                "PAPER_INNER_MAX": "PAPER_DARK_FRAC_MIN",
                "INTERIOR_TRANS_MIN": "PAPER_INNER_MAX",
                "EXPO_SW_GAIN": "EXPO_DARK_RATIO",
                "EXPO_AUTO_GAIN": "EXPO_SW_GAIN",
                "EXPO_SW_MIN": "EXPO_AUTO_GAIN",
                "EXPO_SW_MAX": "EXPO_SW_MIN",
                "EXPO_HW_GAIN": "EXPO_SW_MAX",
                "EXPO_TARGET_MEAN": "EXPO_HW_GAIN"}
# 要搬的"整行"定义（注释 + 赋值，看名字在两边各出现一次）
EXTRA_LINES = ["_GY_FUSE_ERR", "_GY_DBG", "_GY_SRC_DBG",
               "_SW_EXPO_ERR", "_SW_FRAME", "_SW_SCALED", "_GY_TH_NOW"]
# 要搬的函数/类（整块替换，含头顶注释）
BLOCKS = ["_cv2_find", "PaperDetector",
          "log_scene", "px_luma", "paper_contrast", "prof_line",
          "_gy_sort_cyclic", "_pt_in_quad",
          "_gy_quad_area", "_gy_centroid4", "_quad_sides", "_quad_geom_ok",
          "_mask_runs", "_fit_line_pts", "_fit_line_pts_robust",
          "_line_isect", "_quad_convex",
          "_boundary_points", "_cross3", "_convex_hull", "_pt_seg_dist",
          "_rdp_open", "_approx_quad", "_hull_quad_from_bbox",
          "_mask_hull_pair",
          "_mask_band_quad", "_mask_band_quads",
          "_frame_gray", "_apply_sw_scale", "_interior_trans",
          "apply_exposure", "_grid_mean"]
# 工具侧还没有的"新函数"：插到哪个已有函数块之后（和 INSERT_AFTER 同理）
BLOCK_INSERT_AFTER = {"_mask_runs": "_gy_centroid4",
                      "_fit_line_pts": "_mask_runs",
                      "_line_isect": "_fit_line_pts",
                      "_quad_convex": "_line_isect",
                      "_mask_band_quad": "_quad_convex",
                      "_mask_band_quads": "_mask_band_quad",
                      "_frame_gray": "_mask_band_quads",
                      "_interior_trans": "_frame_gray",
                      "_apply_sw_scale": "_grid_mean"}
# 主文件已经删除、工具里可能还残留的旧块（从源文件消失时一并清掉）
REMOVE_BLOCKS = ["_paper_quad_cand", "_scene_th", "_gy_rect_corners",
                 "_cv2_probe"]


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
    # 先清掉主文件里已经不存在的旧视觉块，避免工具继续保留另一条检测路。
    for name in REMOVE_BLOCKS:
        try:
            a2, b2 = block_span(t, name)
        except SystemExit:
            continue
        print("  [删除] 旧函数 %s vc[%d:%d]" % (name, a2 + 1, b2))
        t = t[:a2] + t[b2:]
    # 新加的“融合图直接拟合” helper 是一整段，逐函数插入容易把锚点顺序搞乱；
    # 工具里没有时一次性整段插到 _frame_gray 前，后续仍按普通 BLOCKS 替换。
    mtxt = "\n".join(m)
    ttxt = "\n".join(t)
    _a = mtxt.index("def _mask_runs(")
    _b = mtxt.index("def _cv2_find(", _a)
    if "\ndef _mask_runs(" not in ttxt:
        _c = ttxt.index("def _cv2_find(")
        print("  [插入] 融合图轮廓拟合 helper 段")
        ttxt = ttxt[:_c] + mtxt[_a:_b] + ttxt[_c:]
    else:
        _c = ttxt.index("def _mask_runs(")
        _d = ttxt.index("def _cv2_find(", _c)
        print("  [替换] 融合图轮廓拟合 helper 段")
        ttxt = ttxt[:_c] + mtxt[_a:_b] + ttxt[_d:]
    t = ttxt.split("\n")
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
        try:
            span2 = block_span(t, name)
        except SystemExit:
            anchor = BLOCK_INSERT_AFTER.get(name)
            if anchor is None:
                raise
            _a, _b = block_span(t, anchor)
            span2 = (_b, _b)
            print("  [插入] %s 在 %s 之后" % (name, anchor))
        jobs.append(("block:" + name, block_span(m, name), span2))
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
