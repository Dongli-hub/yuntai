# -*- coding: utf-8 -*-
"""pc_field_test.py —— 用**现场截图**回归检测链（PC 上跑，不需要 K230）

    python k230_aim/tools/pc_field_test.py [截图目录]

为什么要这个：2026-10-17 现场"只有一处位置能识别"，而 pc_vision_test 的
合成场景全绿 —— 说明合成场景没覆盖住真实画面的难点（印刷框 L 和木柜重叠、
远近亮度差、IDE 截图被横向拉伸）。所以把用户现场那两张灰度截图收进
`tools/field_shots/`，每次改完视觉代码都拿它们跑一遍：

  A 近处（gray_a）  真值靶心≈(316,247)  长边≈270~300px
  B 远处（gray_b）  真值靶心≈(326,236)  长边≈126~137px
  各自再乘 0.85 / 0.92 / 1.00 / 1.08 四档模拟"曝光/光照变化"，
  **每一档都必须锁上、靶心漂移 ≤6px**（0.9 那一档历史上最刁钻）。

说明：截图 = IDE 里 640x480 画布被横向拉伸(854/640)后再放大的，
缩回 640x480 就基本还原原图（横向拉伸与整体缩放一起抵掉）。
检测用的是 main.py 的**原文**（exec 执行、去掉 main()），假模块环境来自
pc_sim_main.py；假 cv_lite 与板上 C 函数对齐（含 max_angle_cos 角度闸门）。

返回码：0 = 全过；1 = 有不达标（细节打印在上面）。
"""
import io
import os
import re
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pc_sim_main as S          # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SHOT_DIR = os.path.join(HERE, "field_shots")

# 场景真值（像素，640x480 归一化坐标）：中心容差与长边范围现场量出来的
CASES = [
    ("gray_a.png", "近处", 316.0, 247.0, 6.0),
    ("gray_b.png", "远处", 326.5, 236.0, 6.0),
]
SCALES = (0.85, 0.92, 1.00, 1.08)


class GrayFrame(object):
    """模拟 CanMV 灰度 image：get_pixel 返回单值、copy() 出副本、binary 就地改。"""

    def __init__(self, arr):
        self.np = arr

    def to_numpy_ref(self):
        return self.np

    def copy(self):
        return GrayFrame(self.np.copy())

    def to_grayscale(self):
        return self

    def get_pixel(self, x, y):
        return int(self.np[int(y), int(x)])

    def binary(self, thresholds, **kw):
        lo = thresholds[0][0]
        hi = thresholds[0][1]
        m = (self.np >= lo) & (self.np <= hi)
        self.np = np.where(m, 255, 0).astype(np.uint8)
        return self

    def close(self, size, **kw):
        k = 2 * int(size) + 1
        self.np = cv2.morphologyEx(self.np, cv2.MORPH_CLOSE,
                                   np.ones((k, k), np.uint8))
        return self

    def b_or(self, other):
        self.np = cv2.bitwise_or(self.np, other.np)
        return self

    def find_blobs(self, thresholds, roi=None, merge=True, margin=4,
                   pixels_threshold=3, area_threshold=3):
        """对应 CanMV 灰度图 find_blobs（纸块兜底要用）：返回 FakeBlob（含质心）。"""
        lo = thresholds[0][0]
        hi = thresholds[0][1]
        mask = np.zeros(self.np.shape, np.uint8)
        mask[(self.np >= lo) & (self.np <= hi)] = 255
        if roi is not None:
            rx, ry, rw, rh = roi
            keep = np.zeros_like(mask)
            y1 = max(0, min(self.np.shape[0], ry + rh))
            x1 = max(0, min(self.np.shape[1], rx + rw))
            keep[max(0, ry):y1, max(0, rx):x1] = \
                mask[max(0, ry):y1, max(0, rx):x1]
            mask = keep
        if merge and (margin > 0):
            k = np.ones((2 * margin + 1, 2 * margin + 1), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        n, _labels, stats, cents = cv2.connectedComponentsWithStats(mask, 4)
        out = []
        for i in range(1, n):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < max(int(pixels_threshold), int(area_threshold)):
                continue
            out.append(S.FakeBlob(int(stats[i, cv2.CC_STAT_LEFT]),
                                  int(stats[i, cv2.CC_STAT_TOP]),
                                  int(stats[i, cv2.CC_STAT_WIDTH]),
                                  int(stats[i, cv2.CC_STAT_HEIGHT]), area,
                                  float(cents[i][0]), float(cents[i][1])))
        return out

    def draw_line(self, *a, **kw):
        pass

    def draw_cross(self, *a, **kw):
        pass

    def draw_rectangle(self, *a, **kw):
        pass


def load_prog():
    """把 main.py 原文 exec 进来（去掉末尾的 main() 调用）—— 和 pc_vision_test 同款。"""
    S.install_fake_modules()
    src = io.open(S.MAIN_PY, encoding="utf-8").read()
    src = re.sub(r"(?m)^main\(\)\s*$", "# main()", src)
    g = {"__name__": "m"}
    exec(compile(src, S.MAIN_PY, "exec"), g)
    return g


def norm_img(path):
    # ⚠ 2026-10-17：**不能用 cv2.imread** —— 桌面那份的路径里有中文
    #   （Desktop\\K230代码\\调试工具\\...），cv2.imread/imwrite 走的是 ANSI
    #   路径，直接读不出来（返回 None）。所以这里用"Python 读字节 + imdecode"。
    try:
        with open(path, "rb") as f:
            buf = np.frombuffer(f.read(), np.uint8)
        im = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    except Exception:
        im = None
    if im is None:
        return None
    im = cv2.resize(im, (640, 480), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)


def save_png(path, img_bgr):
    """写 PNG（同样避开 cv2.imwrite 的中文路径问题）。"""
    try:
        ok, buf = cv2.imencode(".png", img_bgr)
        if ok:
            with open(path, "wb") as f:
                f.write(buf.tobytes())
            return True
    except Exception:
        pass
    return False


def main():
    shot_dir = sys.argv[1] if len(sys.argv) > 1 else SHOT_DIR
    g = load_prog()
    ok = True
    for name, label, tx, ty, tol in CASES:
        path = os.path.join(shot_dir, name)
        gray0 = norm_img(path)
        print("== %s（%s）真值≈(%.0f,%.0f) ==" % (name, label, tx, ty))
        if gray0 is None:
            print("   [SKIP] 读不到 %s（把现场截图放进 %s 再跑）"
                  % (path, shot_dir))
            ok = False
            continue
        for sc in SCALES:
            gray = np.clip(gray0.astype(np.float32) * sc, 0,
                           255).astype(np.uint8)
            det = g["PaperDetector"]()
            res = None
            tries = 0
            while (res is None) and (tries < 3):   # 允许轮换用掉 1~2 帧
                res = det.detect(GrayFrame(np.ascontiguousarray(gray)))
                tries += 1
            vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            if res is None:
                print("   亮度x%.2f: FAIL 没锁上 (%s)" % (sc, det.last_dbg))
                ok = False
            else:
                dx = abs(res["center"][0] - tx)
                dy = abs(res["center"][1] - ty)
                good = (dx <= tol) and (dy <= tol)
                print("   亮度x%.2f: %s 靶心=(%.1f,%.1f) 长边=%.0f "
                      "误差=(%.1f,%.1f) 第%d帧锁上 图=%s"
                      % (sc, "OK" if good else "FAIL", res["center"][0],
                         res["center"][1], res["long_side"], dx, dy, tries,
                         g["_GY_SRC_DBG"][0]))
                # 锁定后的第二帧会带 hint。134r 修的那个 bug 就在这里：
                # 旧代码拿"本帧内框长边"去和"上一帧外框长边"比，外框/内框
                # 两套尺度会互相抢。同一张图连续两帧必须还锁在同一个外框上。
                res2 = det.detect(GrayFrame(np.ascontiguousarray(gray.copy())))
                if res2 is None:
                    print("     锁定复测: FAIL（同一张图第二帧丢了）")
                    good = False
                else:
                    dl = abs(res2["long_side"] - res["long_side"])
                    dc = max(abs(res2["center"][0] - res["center"][0]),
                             abs(res2["center"][1] - res["center"][1]))
                    lock_ok = (dl <= max(4.0, 0.03 * res["long_side"])) and \
                        (dc <= 2.0)
                    print("     锁定复测: %s 长边变化=%.0fpx 中心漂移=%.1fpx"
                          % ("OK" if lock_ok else "FAIL", dl, dc))
                    good = good and lock_ok
                ok = ok and good
                c = res["corners"]
                if c is not None:
                    cv2.polylines(vis, [np.array(
                        [[int(p[0]), int(p[1])] for p in c], np.int32)],
                        True, (0, 255, 0), 2)
                cv2.drawMarker(vis,
                               (int(res["center"][0]), int(res["center"][1])),
                               (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
            save_png(os.path.join(HERE, "field_out_%s_%.2f.png"
                                  % (name.replace(".png", ""), sc)), vis)
    print("结果：%s（叠加图写在 %s/field_out_*.png）"
          % ("现场回归 OK" if ok else "现场回归有问题", HERE))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
