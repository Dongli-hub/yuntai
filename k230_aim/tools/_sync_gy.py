# -*- coding: utf-8 -*-
"""把 main.py 的共享视觉常量和函数原样搬到 tools/vision_check.py。

用法：
    python k230_aim/tools/_sync_gy.py
    python k230_aim/tools/sync_check.py

只替换两边都存在的名字；如果 vision_check 缺少某个新名字，脚本会明确报错，
先手工在 vision_check 里加一次，再跑本脚本。这样不会因为插入锚点错位而把
两个文件的视觉代码悄悄改成两套。
"""
import ast
import io
import os
import sys


ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
MAIN = os.path.join(ROOT, "k230_aim", "main.py")
TOOL = os.path.join(ROOT, "k230_aim", "tools", "vision_check.py")

CONSTS = [
    "EXPO_FIX", "EXPO_SETTLE_MS", "EXPO_DARK_RATIO", "EXPO_HW_GAIN",
    "EXPO_TARGET_MEAN", "EXPO_SCALE_MIN", "EXPO_SCALE_MAX",
    "CV2_EVERY",
    "PAPER_TH", "PAPER_TH_FAR", "EDGE_DILATE", "EDGE_DILATE_RETRY",
    "CANNY1", "CANNY2", "CONTOUR_EPS", "CONTOUR_AREA_MIN",
    "QUAD_EVAL_MAX", "CROP_K", "CROP_MIN",
    "PAPER_MIN_SIDE", "QUAD_ANGLE_COS_MAX", "QUAD_OPPOSITE_DIFF_MAX",
    "QUAD_SIDE_RATIO_MAX", "PAPER_CONTRAST_MIN", "EDGE_DARK_FRAC_MIN",
    "EDGE_DARK_RATIO_MAX", "INTERIOR_TRANS_MIN",
    "INTERIOR_TRANS_MIN_FAR",
    "PAPER_MIN_LONG", "PAPER_MAX_LONG", "PAPER_RECT_RATIO_MIN",
    "PAPER_RECT_RATIO_MAX", "PAPER_DARK_MARGIN",
    "PAPER_BLOB_RATIO_MIN", "PAPER_BLOB_RATIO_MAX",
    "PAPER_BLOB_MIN_PX", "PAPER_BLOB_MIN_AREA",
    "LOST_FULL", "SMOOTH", "DEADBAND_PX", "ACCEPT_RELOCK_N",
    "JUMP_REJECT_BASE", "SMOOTH_FAR", "FAR_LONG_PX", "PAPER_LONG_M",
    "LASER_RELOCK_PX",
]

LINES = [
    "_PROF", "_VIS_ERR", "_EDGE_READY", "_VIS_DBG",
    "_ACC_DBG", "_PAPER_TH_NOW", "_EXPO_SCALE", "_EXPO_SCALED",
]

BLOCKS = [
    "_gy_sort_cyclic", "_pt_in_quad", "_gy_quad_area", "_diag_center",
    "_quad_convex", "_quad_sides", "_quad_geom_ok", "_luma_xy",
    "_edge_probe",
    "_paper_blob", "_blob_quad", "_paper_blobs", "_cand_from_quad",
    "_preview_mask", "_rect_quads", "_fallback_bbox", "_frame_gray",
    "_cv2_find", "log_scene", "px_luma", "_paper_mean",
    "prof_line", "_interior_trans", "_apply_expo_scale",
    "apply_exposure", "_grid_mean", "PaperDetector",
]


def read(path):
    return io.open(path, encoding="utf-8").read().split("\n")


def head_start(lines, start):
    i = start - 1
    while i >= 0:
        s = lines[i].strip()
        if (s == "") or s.startswith("#"):
            i -= 1
        else:
            break
    return i + 1


def block_span(lines, name):
    tree = ast.parse("\n".join(lines))
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and \
                node.name == name:
            return head_start(lines, node.lineno - 1), node.end_lineno
    return None


def const_span(lines, name):
    tree = ast.parse("\n".join(lines))
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and \
                isinstance(node.targets[0], ast.Name) and \
                node.targets[0].id == name:
            return head_start(lines, node.lineno - 1), node.end_lineno
    return None


def line_span(lines, name):
    import re
    for i, ln in enumerate(lines):
        if re.match(r"^%s\s*=" % re.escape(name), ln):
            return head_start(lines, i), i + 1
    return None


def main():
    m = read(MAIN)
    t = read(TOOL)
    jobs = []
    for name in CONSTS:
        a1 = const_span(m, name)
        a2 = const_span(t, name)
        if (a1 is None) or (a2 is None):
            raise SystemExit("常量 %s 只在一边存在；请先手工补齐" % name)
        jobs.append(("const:" + name, a1, a2))
    for name in LINES:
        a1 = line_span(m, name)
        a2 = line_span(t, name)
        if (a1 is None) or (a2 is None):
            raise SystemExit("变量 %s 只在一边存在；请先手工补齐" % name)
        jobs.append(("line:" + name, a1, a2))
    for name in BLOCKS:
        a1 = block_span(m, name)
        a2 = block_span(t, name)
        if (a1 is None) or (a2 is None):
            raise SystemExit("函数 %s 只在一边存在；请先手工补齐" % name)
        jobs.append(("block:" + name, a1, a2))
    jobs.sort(key=lambda j: j[2][0], reverse=True)
    for tag, (a1, b1), (a2, b2) in jobs:
        seg = m[a1:b1]
        print("%-24s main[%d:%d] -> vc[%d:%d] %d行"
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
